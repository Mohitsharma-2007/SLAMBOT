"""Tuning parameter registry — the single source of truth for §10.

Every parameter the web UI can change is declared here exactly once, along with
which device owns it. The registry drives three things at runtime:

  * validation and clamping of incoming ``/tuning/update`` messages,
  * routing — a parameter goes to the Arduino, the NodeMCU, the ROS2 param
    server, or several of those, depending on ``targets``,
  * the schema the frontend renders its sliders from, so the UI never carries a
    hardcoded copy of the parameter list that can drift from the backend's.

§10 marks ``encoder_cpr`` / ``wheel_diameter_mm`` / ``wheel_base_mm`` as
"measure and set — no safe default". They are declared ``measured=True`` so the
UI can flag them as needing real measurement rather than presenting the
placeholder as authoritative.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

Target = Literal["arduino", "nodemcu", "ros"]


@dataclass(frozen=True)
class Param:
    """One tunable parameter."""

    name: str
    label: str
    kind: Literal["int", "float", "angle_mask"]
    default: Any
    targets: tuple[Target, ...]
    section: str
    minimum: float | None = None
    maximum: float | None = None
    step: float | None = None
    unit: str = ""
    help: str = ""
    # True for the odometry-geometry values that must be physically measured.
    measured: bool = False
    # ROS2 parameters this value maps onto, as (node_name, param_name, scale).
    # scale converts our unit into the ROS unit (e.g. mm/s -> m/s is 0.001).
    ros_targets: tuple[tuple[str, str, float], ...] = field(default_factory=tuple)

    def coerce(self, value: Any) -> Any:
        """Validate and clamp an incoming value into this parameter's domain."""
        if self.kind == "angle_mask":
            return _coerce_angle_masks(value)

        if self.kind == "int":
            try:
                out: float | int = int(round(float(value)))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{self.name}: not a number: {value!r}") from exc
        else:
            try:
                out = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{self.name}: not a number: {value!r}") from exc
            if out != out or out in (float("inf"), float("-inf")):
                raise ValueError(f"{self.name}: not finite: {value!r}")

        if self.minimum is not None:
            out = max(self.minimum, out)
        if self.maximum is not None:
            out = min(self.maximum, out)
        return int(out) if self.kind == "int" else float(out)


def _coerce_angle_masks(value: Any) -> list[dict[str, float]]:
    """Normalise ``lidar_angle_filter`` into a list of masked-out sectors.

    Accepts ``None`` / ``[]`` (no masking, the §10 default) or a list of
    ``{"start_deg": x, "end_deg": y}``. Sectors are allowed to wrap through
    zero (start > end) — the NodeMCU handles that case.
    """
    if value in (None, "", []):
        return []
    if not isinstance(value, list):
        raise ValueError("lidar_angle_filter: expected a list of sectors")

    masks: list[dict[str, float]] = []
    for entry in value[:4]:  # NodeMCU holds at most 4 masks
        if not isinstance(entry, dict):
            raise ValueError("lidar_angle_filter: each sector must be an object")
        try:
            start = float(entry["start_deg"]) % 360.0
            end = float(entry["end_deg"]) % 360.0
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                "lidar_angle_filter: sector needs numeric start_deg and end_deg"
            ) from exc
        masks.append({"start_deg": start, "end_deg": end})
    return masks


# ---------------------------------------------------------------------------
# The §10 table, verbatim in its defaults, plus the PID gains the closed-loop
# wheel controller needs (firmware-side, so they belong in the same registry).
# ---------------------------------------------------------------------------
PARAMS: tuple[Param, ...] = (
    # --- motion ------------------------------------------------------------
    Param(
        name="max_pwm_duty",
        label="Max PWM duty",
        kind="int",
        default=200,
        minimum=0,
        maximum=255,
        step=1,
        unit="/255",
        section="Motion",
        targets=("arduino",),
        help=(
            "Motor speed cap. Protects the 6 V N20 motors from the 8.4 V pack "
            "seen at DRV8833 VM. 200/255 is ~78% duty ≈ 6.5 V average."
        ),
    ),
    Param(
        name="max_linear_speed_mm_s",
        label="Max linear speed",
        kind="int",
        default=300,
        minimum=0,
        maximum=1000,
        step=10,
        unit="mm/s",
        section="Motion",
        targets=("arduino", "ros"),
        ros_targets=(
            ("controller_server", "FollowPath.max_vel_x", 0.001),
            ("velocity_smoother", "max_velocity.0", 0.001),
        ),
        help="Clamped in firmware and pushed to Nav2's velocity limits.",
    ),
    Param(
        name="max_angular_speed_mdeg_s",
        label="Max angular speed",
        kind="int",
        default=60000,
        minimum=0,
        maximum=360000,
        step=1000,
        unit="mdeg/s",
        section="Motion",
        targets=("arduino", "ros"),
        ros_targets=(
            # mdeg/s -> rad/s : /1000 to deg/s, then *pi/180.
            ("controller_server", "FollowPath.max_vel_theta", 1.7453292519943296e-05),
            ("velocity_smoother", "max_velocity.2", 1.7453292519943296e-05),
        ),
        help="60000 mdeg/s = 60 °/s, the §10 default.",
    ),
    Param(
        name="collision_stop_distance_mm",
        label="Collision stop distance",
        kind="int",
        default=100,
        minimum=0,
        maximum=1000,
        step=10,
        unit="mm",
        section="Motion",
        targets=("arduino",),
        help=(
            "Firmware halts forward motion when the nearest LIDAR return is "
            "closer than this. Enforced on the MCU, not only in the planner."
        ),
    ),
    # --- LIDAR -------------------------------------------------------------
    Param(
        name="lidar_min_range_mm",
        label="LIDAR min range",
        kind="int",
        default=150,
        minimum=0,
        maximum=2000,
        step=10,
        unit="mm",
        section="LIDAR",
        targets=("nodemcu", "ros"),
        ros_targets=(("slam_bot_bridge", "scan_range_min", 0.001),),
        help="Returns closer than this are discarded as chassis/noise.",
    ),
    Param(
        name="lidar_max_range_mm",
        label="LIDAR max range",
        kind="int",
        default=6000,
        minimum=100,
        maximum=12000,
        step=100,
        unit="mm",
        section="LIDAR",
        targets=("nodemcu", "ros"),
        ros_targets=(("slam_bot_bridge", "scan_range_max", 0.001),),
        help="A1M8's rated ceiling is 12 m; 6 m is the §10 default.",
    ),
    Param(
        name="lidar_angle_filter",
        label="Angle mask sectors",
        kind="angle_mask",
        default=[],
        section="LIDAR",
        targets=("nodemcu",),
        help=(
            "Sectors to mask out where the robot's own chassis blocks the "
            "beam. None by default (§10). Up to 4 sectors; may wrap past 0°."
        ),
    ),
    # --- Navigation --------------------------------------------------------
    Param(
        name="obstacle_inflation_radius_mm",
        label="Obstacle inflation radius",
        kind="int",
        default=150,
        minimum=0,
        maximum=1000,
        step=10,
        unit="mm",
        section="Navigation",
        targets=("ros",),
        ros_targets=(
            ("global_costmap/global_costmap", "inflation_layer.inflation_radius", 0.001),
            ("local_costmap/local_costmap", "inflation_layer.inflation_radius", 0.001),
        ),
        help="Nav2 costmap safety margin around obstacles.",
    ),
    # --- Odometry geometry — must be measured (§10) -------------------------
    Param(
        name="encoder_cpr",
        label="Encoder counts per revolution",
        kind="float",
        default=700.0,
        minimum=1.0,
        maximum=20000.0,
        step=1.0,
        unit="counts/rev",
        section="Odometry",
        targets=("arduino", "ros"),
        measured=True,
        ros_targets=(("slam_bot_bridge", "encoder_cpr", 1.0),),
        help=(
            "Counts per OUTPUT-SHAFT revolution, i.e. encoder CPR x gearbox "
            "ratio, and x2 for the 2x decode this firmware uses. Measure it: "
            "mark a wheel, roll exactly 10 turns, read the tick delta / 10."
        ),
    ),
    Param(
        name="wheel_diameter_mm",
        label="Wheel diameter",
        kind="float",
        default=43.0,
        minimum=1.0,
        maximum=500.0,
        step=0.5,
        unit="mm",
        section="Odometry",
        targets=("arduino", "ros"),
        measured=True,
        ros_targets=(("slam_bot_bridge", "wheel_diameter", 0.001),),
        help="Measure with calipers, under load — tyres compress.",
    ),
    Param(
        name="wheel_base_mm",
        label="Wheel base (track width)",
        kind="float",
        default=150.0,
        minimum=1.0,
        maximum=1000.0,
        step=1.0,
        unit="mm",
        section="Odometry",
        targets=("arduino", "ros"),
        measured=True,
        ros_targets=(("slam_bot_bridge", "wheel_base", 0.001),),
        help=(
            "Centre-to-centre distance between the two wheels' contact "
            "patches. Errors here show up as rotational odometry drift."
        ),
    ),
    # --- Wheel velocity PID (firmware-side closed loop) --------------------
    Param(
        name="pid_kp",
        label="Wheel PID — Kp",
        kind="float",
        default=0.60,
        minimum=0.0,
        maximum=10.0,
        step=0.01,
        section="Wheel PID",
        targets=("arduino",),
        help="Proportional gain on wheel velocity error, in duty per mm/s.",
    ),
    Param(
        name="pid_ki",
        label="Wheel PID — Ki",
        kind="float",
        default=2.50,
        minimum=0.0,
        maximum=50.0,
        step=0.05,
        section="Wheel PID",
        targets=("arduino",),
        help="Integral gain. Does the real work at N20 scale; Kp alone stalls.",
    ),
    Param(
        name="pid_kd",
        label="Wheel PID — Kd",
        kind="float",
        default=0.0,
        minimum=0.0,
        maximum=5.0,
        step=0.01,
        section="Wheel PID",
        targets=("arduino",),
        help="Derivative gain. Usually 0 — encoder noise makes it twitchy.",
    ),
    # --- Motion calibration ------------------------------------------------
    # Found on the bench with firmware/movement_test before the full stack was
    # wired. They are here rather than as firmware #defines because every one
    # of them is chassis-specific and you find the right value by driving the
    # robot and watching, which is exactly the loop a reflash ruins.
    #
    # invert_* are booleans carried as 0/1 ints so they use the same numeric
    # path as everything else in the registry; the firmware casts them back.
    Param(
        name="invert_left",
        label="Invert left motor",
        kind="int",
        default=0,
        minimum=0,
        maximum=1,
        step=1,
        section="Motion calibration",
        targets=("arduino",),
        help=(
            "1 flips the left motor's direction. Set this if the wheel spins "
            "backwards; it is applied to the duty only, so encoder direction "
            "and odometry stay correct."
        ),
    ),
    Param(
        name="invert_right",
        label="Invert right motor",
        kind="int",
        default=0,
        minimum=0,
        maximum=1,
        step=1,
        section="Motion calibration",
        targets=("arduino",),
        help=(
            "1 flips the right motor's direction. Set this if the wheel spins "
            "backwards; it is applied to the duty only, so encoder direction "
            "and odometry stay correct."
        ),
    ),
    Param(
        name="trim_left",
        label="Left wheel trim",
        kind="float",
        default=1.00,
        minimum=0.5,
        maximum=1.0,
        step=0.01,
        section="Motion calibration",
        targets=("arduino",),
        help=(
            "Scales the left wheel's speed target. Only ever trims down, so "
            "the duty cap that protects the 6 V motors keeps its headroom — "
            "to fix a veer, slow the faster wheel rather than speeding the "
            "slower one."
        ),
    ),
    Param(
        name="trim_right",
        label="Right wheel trim",
        kind="float",
        default=1.00,
        minimum=0.5,
        maximum=1.0,
        step=0.01,
        section="Motion calibration",
        targets=("arduino",),
        help="Same, for the right wheel. Drive a straight line and trim the faster side.",
    ),
    Param(
        name="turn_boost",
        label="Turn-in-place boost",
        kind="float",
        default=1.60,
        minimum=1.0,
        maximum=3.0,
        step=0.05,
        section="Motion calibration",
        targets=("arduino",),
        help=(
            "Multiplies the rotation command when there is no forward motion. "
            "Spinning on the spot scrubs both tyres sideways, which costs far "
            "more than an arc. Still clamped by the angular speed limit."
        ),
    ),
    Param(
        name="kick_duty",
        label="Kickstart duty",
        kind="int",
        default=110,
        minimum=0,
        maximum=255,
        step=5,
        section="Motion calibration",
        targets=("arduino",),
        help=(
            "Minimum duty applied for a moment when the wheels leave "
            "standstill. Static friction on these gearmotors is well above "
            "rolling friction, so without it a press can leave them stalled "
            "and silent. 0 disables. Never applied to a stopped wheel."
        ),
    ),
    Param(
        name="kick_ms",
        label="Kickstart duration",
        kind="int",
        default=90,
        minimum=0,
        maximum=500,
        step=10,
        unit="ms",
        section="Motion calibration",
        targets=("arduino",),
        help=(
            "How long the kickstart floor is held. Too long and the start is "
            "jerky; too short and the wheel never breaks away."
        ),
    ),
)

BY_NAME: dict[str, Param] = {p.name: p for p in PARAMS}

DEFAULTS: dict[str, Any] = {p.name: p.default for p in PARAMS}

# Firmware structs use short field names for the three limits that appear in
# the spec's §11.1 struct listing; map registry name -> on-wire firmware key.
FIRMWARE_KEY: dict[str, str] = {
    "max_linear_speed_mm_s": "max_linear_speed",
    "max_angular_speed_mdeg_s": "max_angular_speed",
    "collision_stop_distance_mm": "collision_stop_mm",
}


def to_firmware_key(name: str) -> str:
    return FIRMWARE_KEY.get(name, name)


def sections() -> list[str]:
    """Section names in declaration order, for the Tuning Panel's layout."""
    seen: list[str] = []
    for p in PARAMS:
        if p.section not in seen:
            seen.append(p.section)
    return seen


def schema() -> list[dict[str, Any]]:
    """JSON-serialisable description of the registry for the frontend."""
    return [
        {
            "name": p.name,
            "label": p.label,
            "kind": p.kind,
            "default": p.default,
            "min": p.minimum,
            "max": p.maximum,
            "step": p.step,
            "unit": p.unit,
            "section": p.section,
            "targets": list(p.targets),
            "help": p.help,
            "measured": p.measured,
        }
        for p in PARAMS
    ]


def coerce_updates(updates: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    """Validate a batch of updates.

    Returns ``(accepted, errors)``. Unknown keys are reported rather than
    silently dropped, so a frontend/backend mismatch surfaces instead of a
    slider that appears to work but changes nothing.
    """
    accepted: dict[str, Any] = {}
    errors: list[str] = []
    for key, raw in updates.items():
        param = BY_NAME.get(key)
        if param is None:
            errors.append(f"unknown parameter: {key}")
            continue
        try:
            accepted[key] = param.coerce(raw)
        except ValueError as exc:
            errors.append(str(exc))
    return accepted, errors


def split_by_target(values: dict[str, Any]) -> dict[Target, dict[str, Any]]:
    """Route a set of accepted values to the devices that care about them."""
    out: dict[Target, dict[str, Any]] = {"arduino": {}, "nodemcu": {}, "ros": {}}
    for key, value in values.items():
        param = BY_NAME[key]
        for target in param.targets:
            out[target][key] = value
    return out


def ros_overrides(values: dict[str, Any]) -> list[tuple[str, str, Any]]:
    """Expand values into ``(node, ros_param_name, converted_value)`` triples."""
    triples: list[tuple[str, str, Any]] = []
    for key, value in values.items():
        param = BY_NAME[key]
        for node, ros_name, scale in param.ros_targets:
            triples.append((node, ros_name, float(value) * scale))
    return triples
