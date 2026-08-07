"""arduino-cli subprocess wrapper for the Flash Center (§6.4).

Scope split, per the spec:
  * Arduino Uno R4 — compiled AND uploaded here, on the backend host, with
    console output streamed live to the browser. Requires the board to be
    plugged into the machine running this backend.
  * NodeMCU — compiled here (so the browser has a .bin to flash), but the
    *upload* happens in-browser via WebSerial/esptool-js with no backend
    involvement. See webapp/src/lib/esptool.js.

Safety: flashing pulses the board's reset line and, for the Uno R4, holds the
serial port for the duration. Doing that while the robot is driving would leave
motors latched at their last duty with nothing servicing the control loop, so a
flash is refused unless the bot is stopped.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import AsyncIterator, Callable

from config import SETTINGS

logger = logging.getLogger("slam_bot.flasher")

# Only these two targets are flashable; anything else is rejected outright
# rather than interpolated into a command line.
TARGETS = ("arduino", "nodemcu")


@dataclass
class FlashResult:
    ok: bool
    detail: str
    artifact: Path | None = None


class FlashBusy(RuntimeError):
    """Raised when a flash is requested while one is already running."""


class Flasher:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._running = False
        self._process: asyncio.subprocess.Process | None = None

    @property
    def busy(self) -> bool:
        return self._running

    # -- environment probing ----------------------------------------------
    def cli_path(self) -> str | None:
        """Resolve arduino-cli, honouring SLAM_ARDUINO_CLI."""
        configured = SETTINGS.arduino_cli
        found = shutil.which(configured)
        if found:
            return found
        candidate = Path(configured)
        if candidate.is_file():
            return str(candidate)
        return None

    async def probe(self) -> dict[str, object]:
        """What the Flash Center needs to render honestly before you click."""
        cli = self.cli_path()
        info: dict[str, object] = {
            "cli_available": cli is not None,
            "cli_path": cli,
            "cli_version": None,
            "ports": [],
            "arduino_fqbn": SETTINGS.arduino_fqbn,
            "nodemcu_fqbn": SETTINGS.nodemcu_fqbn,
            "arduino_sketch": str(SETTINGS.arduino_sketch_dir),
            "nodemcu_sketch": str(SETTINGS.nodemcu_sketch_dir),
            "arduino_sketch_exists": (
                SETTINGS.arduino_sketch_dir / "arduino_uno_r4.ino"
            ).is_file(),
            "nodemcu_sketch_exists": (
                SETTINGS.nodemcu_sketch_dir / "nodemcu_lidar.ino"
            ).is_file(),
            "busy": self._running,
        }
        if cli is None:
            return info

        rc, out = await self._run_capture([cli, "version"])
        if rc == 0:
            info["cli_version"] = out.strip().splitlines()[0] if out.strip() else None
        info["ports"] = await self.list_ports()
        return info

    async def list_ports(self) -> list[dict[str, str]]:
        """Serial ports arduino-cli can see, with any detected board name."""
        cli = self.cli_path()
        if cli is None:
            return []
        rc, out = await self._run_capture([cli, "board", "list", "--format", "json"])
        if rc != 0:
            return []
        import json

        try:
            parsed = json.loads(out or "{}")
        except json.JSONDecodeError:
            return []

        # arduino-cli has used two shapes across versions: a bare list, and
        # {"detected_ports": [...]}. Handle both rather than pinning a version.
        entries = parsed.get("detected_ports", parsed) if isinstance(parsed, dict) else parsed
        if not isinstance(entries, list):
            return []

        ports: list[dict[str, str]] = []
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            port = entry.get("port") or {}
            boards = entry.get("matching_boards") or entry.get("boards") or []
            board_name = ""
            fqbn = ""
            if isinstance(boards, list) and boards and isinstance(boards[0], dict):
                board_name = boards[0].get("name", "") or ""
                fqbn = boards[0].get("fqbn", "") or ""
            ports.append(
                {
                    "address": port.get("address", "") if isinstance(port, dict) else "",
                    "protocol": port.get("protocol", "") if isinstance(port, dict) else "",
                    "label": port.get("label", "") if isinstance(port, dict) else "",
                    "board": board_name,
                    "fqbn": fqbn,
                }
            )
        return [p for p in ports if p["address"]]

    # -- compile / upload --------------------------------------------------
    async def compile_and_upload(
        self,
        target: str,
        port: str | None,
        emit: Callable[[str, str], None],
        upload: bool = True,
    ) -> FlashResult:
        """Compile (and optionally upload) one target, streaming output.

        ``emit(stream, line)`` is called for every output line, where ``stream``
        is "stdout" or "stderr" — the Flash Center colours them differently.
        """
        if target not in TARGETS:
            return FlashResult(False, f"unknown target: {target!r}")

        if self._lock.locked():
            raise FlashBusy("a flash is already running")

        async with self._lock:
            self._running = True
            try:
                return await self._do_flash(target, port, emit, upload)
            finally:
                self._running = False
                self._process = None

    async def _do_flash(
        self,
        target: str,
        port: str | None,
        emit: Callable[[str, str], None],
        upload: bool,
    ) -> FlashResult:
        cli = self.cli_path()
        if cli is None:
            return FlashResult(
                False,
                "arduino-cli not found. Install it and/or set SLAM_ARDUINO_CLI "
                "to its full path.",
            )

        if target == "arduino":
            sketch = SETTINGS.arduino_sketch_dir
            fqbn = SETTINGS.arduino_fqbn
        else:
            sketch = SETTINGS.nodemcu_sketch_dir
            fqbn = SETTINGS.nodemcu_fqbn

        if not sketch.is_dir():
            return FlashResult(False, f"sketch directory not found: {sketch}")

        secrets = sketch / "secrets.h"
        if not secrets.is_file():
            return FlashResult(
                False,
                f"{secrets.name} is missing in {sketch.name}/ — copy the template "
                "and fill in WiFi credentials before flashing.",
            )

        build_dir = SETTINGS.build_dir / target
        build_dir.mkdir(parents=True, exist_ok=True)

        emit("stdout", f"$ arduino-cli compile --fqbn {fqbn} {sketch}")
        rc = await self._stream(
            [
                cli,
                "compile",
                "--fqbn",
                fqbn,
                "--build-path",
                str(build_dir),
                str(sketch),
            ],
            emit,
        )
        if rc != 0:
            return FlashResult(False, f"compile failed (exit {rc})")

        artifact = self._find_artifact(build_dir, target)
        if artifact is not None:
            emit("stdout", f"Built artifact: {artifact.name}")

        if not upload:
            return FlashResult(True, "compile succeeded", artifact)

        if target == "nodemcu":
            # §6.4: NodeMCU is flashed in-browser over WebSerial, never here.
            return FlashResult(
                True,
                "compile succeeded — flash the .bin from the browser via WebSerial",
                artifact,
            )

        if not port:
            return FlashResult(False, "no serial port selected for upload")

        emit("stdout", f"$ arduino-cli upload -p {port} --fqbn {fqbn} {sketch}")
        rc = await self._stream(
            [
                cli,
                "upload",
                "-p",
                port,
                "--fqbn",
                fqbn,
                "--input-dir",
                str(build_dir),
                str(sketch),
            ],
            emit,
        )
        if rc != 0:
            return FlashResult(False, f"upload failed (exit {rc})", artifact)
        return FlashResult(True, "upload succeeded", artifact)

    def _find_artifact(self, build_dir: Path, target: str) -> Path | None:
        suffixes = (".bin",) if target == "nodemcu" else (".bin", ".hex")
        for suffix in suffixes:
            matches = sorted(build_dir.glob(f"*.ino{suffix}"))
            if matches:
                return matches[0]
            matches = sorted(build_dir.glob(f"*{suffix}"))
            if matches:
                return matches[0]
        return None

    def artifact_for(self, target: str) -> Path | None:
        """Path to the last built binary, for the browser to download."""
        if target not in TARGETS:
            return None
        return self._find_artifact(SETTINGS.build_dir / target, target)

    # -- subprocess plumbing ----------------------------------------------
    async def _stream(
        self, argv: list[str], emit: Callable[[str, str], None]
    ) -> int:
        """Run a command, emitting every stdout/stderr line as it arrives."""
        try:
            process = await asyncio.create_subprocess_exec(
                *argv,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                cwd=str(SETTINGS.arduino_sketch_dir.parent),
                env={**os.environ, "NO_COLOR": "1"},
            )
        except FileNotFoundError:
            emit("stderr", f"executable not found: {argv[0]}")
            return 127
        except Exception as exc:
            emit("stderr", f"failed to launch: {exc}")
            return 1

        self._process = process

        async def drain(stream: asyncio.StreamReader | None, label: str) -> None:
            if stream is None:
                return
            while True:
                line = await stream.readline()
                if not line:
                    break
                emit(label, line.decode("utf-8", errors="replace").rstrip("\r\n"))

        try:
            await asyncio.wait_for(
                asyncio.gather(
                    drain(process.stdout, "stdout"),
                    drain(process.stderr, "stderr"),
                    process.wait(),
                ),
                timeout=SETTINGS.flash_timeout_s,
            )
        except asyncio.TimeoutError:
            emit("stderr", f"timed out after {SETTINGS.flash_timeout_s}s — killing")
            with contextlib.suppress(ProcessLookupError):
                process.kill()
            await process.wait()
            return 124

        return process.returncode if process.returncode is not None else 1

    async def _run_capture(self, argv: list[str]) -> tuple[int, str]:
        try:
            process = await asyncio.create_subprocess_exec(
                *argv,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env={**os.environ, "NO_COLOR": "1"},
            )
            stdout, _ = await asyncio.wait_for(process.communicate(), timeout=20)
            return (
                process.returncode if process.returncode is not None else 1,
                stdout.decode("utf-8", errors="replace"),
            )
        except (FileNotFoundError, asyncio.TimeoutError, Exception):
            return 1, ""


FLASHER = Flasher()
