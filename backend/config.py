"""Backend configuration, from environment variables with sane defaults.

Nothing security-relevant is hardcoded. §9 in particular forbids baking an
OpenRouter model id into the code, so the model is resolved at startup from a
candidate list that can be overridden here.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


def _env_str(key: str, default: str) -> str:
    value = os.environ.get(key, "").strip()
    return value or default


def _env_int(key: str, default: int) -> int:
    try:
        return int(os.environ.get(key, "").strip() or default)
    except ValueError:
        return default


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, "").strip() or default)
    except ValueError:
        return default


def _env_bool(key: str, default: bool) -> bool:
    raw = os.environ.get(key, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


def _env_list(key: str, default: list[str]) -> list[str]:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return list(default)
    return [item.strip() for item in raw.split(",") if item.strip()]


BACKEND_DIR = Path(__file__).resolve().parent
REPO_ROOT = BACKEND_DIR.parent


@dataclass
class Settings:
    host: str = field(default_factory=lambda: _env_str("SLAM_HOST", "0.0.0.0"))
    port: int = field(default_factory=lambda: _env_int("SLAM_PORT", 8000))

    # CORS origins for the Vite dev server. In production the webapp is served
    # as static files from this same origin and this list is unused.
    cors_origins: list[str] = field(
        default_factory=lambda: _env_list(
            "SLAM_CORS_ORIGINS",
            [
                "http://localhost:5173",
                "http://127.0.0.1:5173",
            ],
        )
    )

    # --- robot link -------------------------------------------------------
    # §11.2's watchdog is 2 s firmware-side; the backend pings well inside that
    # so a quiet-but-healthy link never trips it.
    heartbeat_hz: float = field(
        default_factory=lambda: _env_float("SLAM_HEARTBEAT_HZ", 4.0)
    )
    # A manual drive command expires this fast unless the browser refreshes it,
    # so a closed laptop lid cannot leave the robot driving.
    manual_command_ttl_ms: int = field(
        default_factory=lambda: _env_int("SLAM_MANUAL_TTL_MS", 500)
    )
    device_stale_ms: int = field(
        default_factory=lambda: _env_int("SLAM_DEVICE_STALE_MS", 3000)
    )

    # --- frontend push rates ---------------------------------------------
    status_hz: float = field(default_factory=lambda: _env_float("SLAM_STATUS_HZ", 5.0))
    scan_hz: float = field(default_factory=lambda: _env_float("SLAM_SCAN_HZ", 8.0))
    map_hz: float = field(default_factory=lambda: _env_float("SLAM_MAP_HZ", 1.0))

    # --- arduino-cli (§6.4) ----------------------------------------------
    arduino_cli: str = field(
        default_factory=lambda: _env_str("SLAM_ARDUINO_CLI", "arduino-cli")
    )
    arduino_fqbn: str = field(
        default_factory=lambda: _env_str("SLAM_ARDUINO_FQBN", "arduino:renesas_uno:unor4wifi")
    )
    arduino_sketch_dir: Path = field(
        default_factory=lambda: Path(
            _env_str(
                "SLAM_ARDUINO_SKETCH",
                str(REPO_ROOT / "firmware" / "arduino_uno_r4"),
            )
        )
    )
    nodemcu_sketch_dir: Path = field(
        default_factory=lambda: Path(
            _env_str(
                "SLAM_NODEMCU_SKETCH",
                str(REPO_ROOT / "firmware" / "nodemcu_lidar"),
            )
        )
    )
    nodemcu_fqbn: str = field(
        default_factory=lambda: _env_str("SLAM_NODEMCU_FQBN", "esp8266:esp8266:nodemcuv2")
    )
    build_dir: Path = field(
        default_factory=lambda: Path(
            _env_str("SLAM_BUILD_DIR", str(REPO_ROOT / "build"))
        )
    )
    flash_timeout_s: int = field(
        default_factory=lambda: _env_int("SLAM_FLASH_TIMEOUT_S", 300)
    )

    # --- OpenRouter (§9) --------------------------------------------------
    openrouter_key: str = field(
        default_factory=lambda: _env_str("OPENROUTER_API_KEY", "")
    )
    openrouter_base: str = field(
        default_factory=lambda: _env_str(
            "OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1"
        )
    )
    # Ordered fallback candidates, per §9: "a fallback list of 2-3 candidates in
    # case the first is unavailable". Overridable, and the resolver also queries
    # /models for anything with pricing.prompt == "0" rather than trusting these
    # blindly.
    openrouter_candidates: list[str] = field(
        default_factory=lambda: _env_list(
            "OPENROUTER_MODELS",
            [
                "deepseek/deepseek-chat-v3.1:free",
                "meta-llama/llama-3.3-70b-instruct:free",
                "qwen/qwen-2.5-72b-instruct:free",
            ],
        )
    )
    openrouter_timeout_s: float = field(
        default_factory=lambda: _env_float("OPENROUTER_TIMEOUT_S", 90.0)
    )
    # Set to False to skip the startup /models probe (offline development).
    resolve_model_at_startup: bool = field(
        default_factory=lambda: _env_bool("SLAM_RESOLVE_MODEL", True)
    )

    # --- ROS2 -------------------------------------------------------------
    # The backend runs fine with ROS2 absent — the Dashboard, Live Map polar
    # plot, Logs, Flash Center and Tuning Panel all work without it. Only /map
    # and Nav2 goals need it.
    enable_ros: bool = field(default_factory=lambda: _env_bool("SLAM_ENABLE_ROS", True))

    # --- static frontend --------------------------------------------------
    serve_webapp: bool = field(
        default_factory=lambda: _env_bool("SLAM_SERVE_WEBAPP", True)
    )
    webapp_dist: Path = field(
        default_factory=lambda: Path(
            _env_str("SLAM_WEBAPP_DIST", str(REPO_ROOT / "webapp" / "dist"))
        )
    )

    # --- session logging --------------------------------------------------
    session_log_dir: Path = field(
        default_factory=lambda: Path(
            _env_str("SLAM_SESSION_DIR", str(REPO_ROOT / "sessions"))
        )
    )


SETTINGS = Settings()
