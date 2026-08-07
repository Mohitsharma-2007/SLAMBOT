"""OpenRouter integration (§9).

Three features, all built on summarised text — §9 is explicit that raw sensor
streams never go to the API, both for cost and because a text LLM cannot
usefully interpret them:

  1. ``analyze_session`` — session stats in, structured tuning suggestions out.
  2. ``describe_map`` — a text-encoded occupancy-grid summary in, plain-language
     description out. Room detection is a local flood-fill; the model only puts
     words to numbers we computed ourselves.
  3. ``chat`` — free-form Q&A about the robot, with the current state as context.

Model selection follows §9's rule: never hardcode a model id. At startup we
query ``/models``, keep only entries with ``pricing.prompt == "0"``, and pick the
first configured candidate that appears in that set — falling back through the
candidate list, then to any free model at all.

Suggestions from the model are always validated against the tuning registry
before the UI can apply them, so a hallucinated parameter name or an
out-of-range value cannot reach the hardware (§9.3's bounds-checked-intermediate
rule, applied to the advisor path too).
"""

from __future__ import annotations

import json
import logging
import math
import re
from collections import deque
from typing import Any

import httpx

import tuning
from config import SETTINGS

logger = logging.getLogger("slam_bot.ai")

SYSTEM_TUNING = (
    "You are a robotics tuning advisor for a small differential-drive SLAM "
    "robot (2x N20 6V geared motors with quadrature encoders, DRV8833 driver, "
    "RPLIDAR A1M8, 2S LiPo, ROS2 slam_toolbox + Nav2 with SmacPlanner2D and "
    "the DWB controller). "
    "Respond ONLY with a JSON object of the form "
    '{"suggestions": [{"parameter": str, "suggested_value": number, '
    '"reasoning": str}], "summary": str}. '
    "Use only parameter names from the provided allowed list. Keep every "
    "suggested value inside the stated min/max. Prefer few, high-confidence "
    "changes over many speculative ones. If the data does not justify a "
    "change, return an empty suggestions array and say so in the summary."
)

SYSTEM_MAP = (
    "You are a robotics mapping analyst. Given a numeric summary of a 2D "
    "occupancy grid built by slam_toolbox, describe it in plain language for a "
    "hobbyist: how many room-like regions it suggests, roughly how open the "
    "space is, where the choke points are, and whether the map looks "
    "trustworthy or under-explored. Be concrete and brief — at most 200 words. "
    "Do not invent details the numbers do not support."
)

SYSTEM_CHAT = (
    "You are the onboard assistant for a hobbyist SLAM robot. You can explain "
    "the robot's telemetry, SLAM/Nav2 behaviour, wiring and tuning. You cannot "
    "command motion — if asked to drive the robot, say the operator must use "
    "the Dashboard's controls. Be concise and practical."
)


class AiAdvisor:
    def __init__(self) -> None:
        self.model: str | None = None
        self.free_models: list[str] = []
        self.last_error: str | None = None
        self._history: deque[dict[str, str]] = deque(maxlen=20)

    @property
    def configured(self) -> bool:
        return bool(SETTINGS.openrouter_key)

    # -- model resolution (§9) --------------------------------------------
    async def resolve_model(self) -> str | None:
        """Pick a free-tier model at startup. Never hardcodes an id."""
        candidates = SETTINGS.openrouter_candidates
        if not self.configured:
            self.last_error = "OPENROUTER_API_KEY is not set"
            # Still record the candidate order so the UI can show what *would*
            # be tried once a key is present.
            self.model = None
            return None

        if not SETTINGS.resolve_model_at_startup:
            self.model = candidates[0] if candidates else None
            return self.model

        try:
            async with httpx.AsyncClient(timeout=20.0) as client:
                resp = await client.get(
                    f"{SETTINGS.openrouter_base}/models",
                    headers=self._headers(),
                )
                resp.raise_for_status()
                payload = resp.json()
        except Exception as exc:
            logger.warning("could not query OpenRouter /models: %s", exc)
            self.last_error = f"model probe failed: {exc}"
            self.model = candidates[0] if candidates else None
            return self.model

        free: list[str] = []
        for entry in payload.get("data", []) or []:
            if not isinstance(entry, dict):
                continue
            pricing = entry.get("pricing") or {}
            prompt_price = str(pricing.get("prompt", "")).strip()
            completion_price = str(pricing.get("completion", "0")).strip()
            # §9: filter for pricing.prompt == "0". Also require a free
            # completion price — a zero prompt price with a paid completion is
            # not actually free.
            if _is_zero(prompt_price) and _is_zero(completion_price):
                model_id = entry.get("id")
                if isinstance(model_id, str):
                    free.append(model_id)

        self.free_models = sorted(free)
        for candidate in candidates:
            if candidate in free:
                self.model = candidate
                self.last_error = None
                logger.info("AI advisor using free model %s", candidate)
                return candidate

        if free:
            # None of the configured candidates are currently available — §9
            # anticipates exactly this rotation, so take any free model.
            self.model = free[0]
            self.last_error = (
                "none of the configured candidates were free; fell back to "
                f"{self.model}"
            )
            logger.info("AI advisor fell back to %s", self.model)
            return self.model

        self.model = candidates[0] if candidates else None
        self.last_error = "no free-tier models reported by OpenRouter"
        return self.model

    def _headers(self) -> dict[str, str]:
        headers = {
            "Authorization": f"Bearer {SETTINGS.openrouter_key}",
            "Content-Type": "application/json",
            # OpenRouter uses these for attribution / rate-limit tiering.
            "HTTP-Referer": "http://localhost:8000",
            "X-Title": "SLAM Bot",
        }
        return headers

    # -- core call ---------------------------------------------------------
    async def _complete(
        self,
        system: str,
        user: str,
        *,
        json_mode: bool = False,
        max_tokens: int = 1200,
    ) -> tuple[bool, str]:
        if not self.configured:
            return False, "OPENROUTER_API_KEY is not set on the backend."
        if self.model is None:
            await self.resolve_model()
        if self.model is None:
            return False, self.last_error or "no model available"

        body: dict[str, Any] = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            "max_tokens": max_tokens,
        }
        if json_mode:
            body["response_format"] = {"type": "json_object"}

        tried: list[str] = []
        for model in self._model_attempts():
            body["model"] = model
            tried.append(model)
            try:
                async with httpx.AsyncClient(
                    timeout=SETTINGS.openrouter_timeout_s
                ) as client:
                    resp = await client.post(
                        f"{SETTINGS.openrouter_base}/chat/completions",
                        headers=self._headers(),
                        json=body,
                    )
            except Exception as exc:
                self.last_error = f"request to {model} failed: {exc}"
                continue

            if resp.status_code == 429:
                self.last_error = f"{model} rate-limited"
                continue
            if resp.status_code >= 400:
                detail = resp.text[:300]
                self.last_error = f"{model} returned {resp.status_code}: {detail}"
                # 404/400 usually means the model id went away — try the next.
                continue

            try:
                payload = resp.json()
                content = payload["choices"][0]["message"]["content"]
            except (KeyError, IndexError, ValueError, TypeError) as exc:
                self.last_error = f"unexpected response shape from {model}: {exc}"
                continue

            if model != self.model:
                logger.info("AI advisor switched to fallback model %s", model)
                self.model = model
            self.last_error = None
            return True, content or ""

        return False, self.last_error or f"all models failed (tried: {', '.join(tried)})"

    def _model_attempts(self) -> list[str]:
        """Current model first, then the remaining configured candidates."""
        order: list[str] = []
        if self.model:
            order.append(self.model)
        for candidate in SETTINGS.openrouter_candidates:
            if candidate not in order:
                order.append(candidate)
        return order

    # -- §9.1 session analysis --------------------------------------------
    async def analyze_session(self, summary: dict[str, Any]) -> dict[str, Any]:
        allowed = _allowed_params_block()
        user = (
            "Session summary (JSON):\n"
            + json.dumps(summary, indent=2, default=str)
            + "\n\nAllowed parameters (name, current, min, max, unit, effect):\n"
            + allowed
            + "\n\nSuggest tuning changes that would improve mapping quality "
            "and reduce collisions/stalls in the next run."
        )
        ok, content = await self._complete(SYSTEM_TUNING, user, json_mode=True)
        if not ok:
            return {"ok": False, "error": content, "suggestions": []}

        parsed = _extract_json(content)
        if parsed is None:
            return {
                "ok": False,
                "error": "model did not return parseable JSON",
                "raw": content[:2000],
                "suggestions": [],
            }

        suggestions, rejected = _validate_suggestions(parsed.get("suggestions"))
        return {
            "ok": True,
            "model": self.model,
            "summary": str(parsed.get("summary", ""))[:2000],
            "suggestions": suggestions,
            "rejected": rejected,
        }

    # -- §9.2 map description ---------------------------------------------
    async def describe_map(self, map_summary: dict[str, Any]) -> dict[str, Any]:
        user = (
            "Occupancy grid summary (JSON):\n"
            + json.dumps(map_summary, indent=2, default=str)
            + "\n\nDescribe this map in plain language."
        )
        ok, content = await self._complete(SYSTEM_MAP, user, max_tokens=600)
        if not ok:
            return {"ok": False, "error": content}
        return {"ok": True, "model": self.model, "description": content.strip()}

    # -- chat --------------------------------------------------------------
    async def chat(self, message: str, context: dict[str, Any]) -> dict[str, Any]:
        user = (
            "Current robot state (JSON):\n"
            + json.dumps(context, indent=2, default=str)
            + f"\n\nOperator question: {message}"
        )
        ok, content = await self._complete(SYSTEM_CHAT, user, max_tokens=800)
        if not ok:
            return {"ok": False, "error": content}
        self._history.append({"role": "user", "content": message})
        self._history.append({"role": "assistant", "content": content})
        return {"ok": True, "model": self.model, "reply": content.strip()}


def _is_zero(price: str) -> bool:
    """OpenRouter reports prices as decimal strings; '0', '0.0', '0e0' all free."""
    if not price:
        return False
    try:
        return float(price) == 0.0
    except ValueError:
        return False


def _allowed_params_block() -> str:
    lines: list[str] = []
    for param in tuning.PARAMS:
        if param.kind == "angle_mask":
            continue  # not a scalar the model can usefully suggest
        bounds = f"{param.minimum}..{param.maximum}"
        lines.append(
            f"- {param.name} (default {param.default}, range {bounds} "
            f"{param.unit}): {param.help}"
        )
    return "\n".join(lines)


def _extract_json(content: str) -> dict[str, Any] | None:
    """Parse a JSON object from a model reply, tolerating markdown fences."""
    text = content.strip()
    if text.startswith("```"):
        text = re.sub(r"^```[a-zA-Z]*\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    try:
        parsed = json.loads(text)
        return parsed if isinstance(parsed, dict) else {"suggestions": parsed}
    except json.JSONDecodeError:
        pass
    # Fall back to the outermost brace pair.
    start = text.find("{")
    end = text.rfind("}")
    if start != -1 and end > start:
        try:
            parsed = json.loads(text[start : end + 1])
            return parsed if isinstance(parsed, dict) else None
        except json.JSONDecodeError:
            return None
    return None


def _validate_suggestions(
    raw: Any,
) -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
    """Keep only suggestions naming a real parameter with an in-range value.

    §9.3's principle generalised: nothing a model produced reaches the hardware
    without passing through a bounds check first. Rejections are returned so the
    UI can show them rather than silently hiding the model's mistakes.
    """
    accepted: list[dict[str, Any]] = []
    rejected: list[dict[str, str]] = []

    if not isinstance(raw, list):
        return accepted, rejected

    for entry in raw[:20]:
        if not isinstance(entry, dict):
            rejected.append({"parameter": "?", "reason": "not an object"})
            continue
        name = str(entry.get("parameter", "")).strip()
        param = tuning.BY_NAME.get(name)
        if param is None:
            rejected.append({"parameter": name or "?", "reason": "unknown parameter"})
            continue
        if param.kind == "angle_mask":
            rejected.append({"parameter": name, "reason": "not a scalar parameter"})
            continue
        try:
            requested = float(entry.get("suggested_value"))
        except (TypeError, ValueError):
            rejected.append({"parameter": name, "reason": "non-numeric value"})
            continue
        if math.isnan(requested) or math.isinf(requested):
            rejected.append({"parameter": name, "reason": "value not finite"})
            continue

        clamped = param.coerce(requested)
        note = ""
        if float(clamped) != requested:
            note = f"clamped from {requested} to {clamped}"
        accepted.append(
            {
                "parameter": name,
                "suggested_value": clamped,
                "requested_value": requested,
                "reasoning": str(entry.get("reasoning", ""))[:600],
                "note": note,
            }
        )
    return accepted, rejected


# ---------------------------------------------------------------------------
# Local map analysis — computed here, not by the model (§9.2)
# ---------------------------------------------------------------------------
def summarize_map(
    width: int,
    height: int,
    resolution: float,
    rle: list[int],
) -> dict[str, Any]:
    """Text-safe numeric summary of the occupancy grid, incl. flood-fill rooms."""
    if width <= 0 or height <= 0 or not rle:
        return {"empty": True}

    cells = _rle_decode(rle, width * height)

    free = sum(1 for c in cells if 0 <= c < 25)
    occupied = sum(1 for c in cells if c >= 65)
    unknown = sum(1 for c in cells if c < 0)
    total = width * height

    cell_area = resolution * resolution
    regions = _flood_fill_regions(cells, width, height)
    # Ignore specks — a "room" under 0.5 m² is noise, not a room.
    min_cells = max(4, int(0.5 / cell_area)) if cell_area > 0 else 4
    rooms = [r for r in regions if r >= min_cells]
    rooms.sort(reverse=True)

    return {
        "empty": False,
        "grid": {
            "width_cells": width,
            "height_cells": height,
            "resolution_m": round(resolution, 4),
            "width_m": round(width * resolution, 2),
            "height_m": round(height * resolution, 2),
        },
        "cells": {
            "free": free,
            "occupied": occupied,
            "unknown": unknown,
            "total": total,
        },
        "ratios": {
            "explored_pct": round(100.0 * (total - unknown) / total, 1) if total else 0,
            "free_pct": round(100.0 * free / total, 1) if total else 0,
            "occupied_pct": round(100.0 * occupied / total, 1) if total else 0,
        },
        "free_area_m2": round(free * cell_area, 2),
        "occupied_area_m2": round(occupied * cell_area, 2),
        "room_like_regions": len(rooms),
        "region_areas_m2": [round(r * cell_area, 2) for r in rooms[:10]],
        "largest_region_pct_of_free": (
            round(100.0 * rooms[0] / free, 1) if rooms and free else None
        ),
    }


def _rle_decode(rle: list[int], expected: int) -> list[int]:
    out: list[int] = []
    for i in range(0, len(rle) - 1, 2):
        out.extend([rle[i]] * rle[i + 1])
        if len(out) >= expected:
            break
    if len(out) < expected:
        out.extend([-1] * (expected - len(out)))
    return out[:expected]


def _flood_fill_regions(cells: list[int], width: int, height: int) -> list[int]:
    """4-connected connected-component sizes over free space.

    Iterative, not recursive: a 4000x4000 grid would blow the Python stack.
    """
    seen = bytearray(len(cells))
    sizes: list[int] = []

    def is_free(index: int) -> bool:
        value = cells[index]
        return 0 <= value < 25

    for start in range(len(cells)):
        if seen[start] or not is_free(start):
            continue
        size = 0
        stack = [start]
        seen[start] = 1
        while stack:
            index = stack.pop()
            size += 1
            x = index % width
            y = index // width
            if x > 0:
                n = index - 1
                if not seen[n] and is_free(n):
                    seen[n] = 1
                    stack.append(n)
            if x < width - 1:
                n = index + 1
                if not seen[n] and is_free(n):
                    seen[n] = 1
                    stack.append(n)
            if y > 0:
                n = index - width
                if not seen[n] and is_free(n):
                    seen[n] = 1
                    stack.append(n)
            if y < height - 1:
                n = index + width
                if not seen[n] and is_free(n):
                    seen[n] = 1
                    stack.append(n)
        sizes.append(size)
    return sizes


ADVISOR = AiAdvisor()
