"""Offline smoke test — verifies the backend wires up without hardware or ROS2.

Run:  python backend/smoke_test.py

Checks the pure-logic pieces that are easy to get wrong and hard to notice on
real hardware: the tuning registry's clamping and routing, RLE round-tripping,
LaserScan binning, flood-fill room detection, and AI-suggestion validation.
Exercises the WebSocket protocol end to end against a fake robot using
FastAPI's TestClient, so a protocol regression fails here rather than on the
bench.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# Must be set before importing the app: no ROS2 and no network at import time.
os.environ.setdefault("SLAM_ENABLE_ROS", "0")
os.environ.setdefault("SLAM_RESOLVE_MODEL", "0")
# Slow the heartbeat to its floor (0.5 Hz -> one tick per 2 s) so the control
# loop's pings/cmd_vel don't interleave with the frames this test asserts on.
os.environ.setdefault("SLAM_HEARTBEAT_HZ", "0.5")

FAILURES: list[str] = []


def check(label: str, condition: bool, detail: str = "") -> None:
    if condition:
        print(f"  PASS  {label}")
    else:
        print(f"  FAIL  {label}" + (f" -- {detail}" if detail else ""))
        FAILURES.append(label)


def test_tuning_registry() -> None:
    import tuning

    print("\n[tuning registry]")
    # Assert the §10 table is present by name rather than by count: a count
    # check fails every time an extra parameter is added, which says nothing
    # about whether the spec's parameters are still there.
    spec_10 = {
        "max_pwm_duty", "max_linear_speed_mm_s", "max_angular_speed_mdeg_s",
        "collision_stop_distance_mm", "lidar_min_range_mm",
        "lidar_max_range_mm", "lidar_angle_filter",
        "obstacle_inflation_radius_mm", "encoder_cpr", "wheel_diameter_mm",
        "wheel_base_mm", "pid_kp", "pid_ki", "pid_kd",
    }
    missing = sorted(spec_10 - set(tuning.BY_NAME))
    check("all §10 parameters present", not missing, f"missing {missing}")

    # Motion calibration — the values found on the bench with movement_test.
    # These must be runtime-tunable, not firmware #defines, or retuning the
    # robot means a reflash.
    calib = {
        "invert_left", "invert_right", "trim_left", "trim_right",
        "turn_boost", "kick_duty", "kick_ms",
    }
    missing_calib = sorted(calib - set(tuning.BY_NAME))
    check("motion calibration parameters present", not missing_calib,
          f"missing {missing_calib}")
    check("calibration targets the Arduino only",
          all(tuning.BY_NAME[n].targets == ("arduino",) for n in calib
              if n in tuning.BY_NAME))
    # Trim must never scale *up*: the duty cap is the only thing keeping the
    # 6 V motors safe on an 8.4 V pack, and a >1 trim would eat its headroom.
    check("trim cannot exceed 1.0",
          tuning.BY_NAME["trim_left"].coerce(2.0) == 1.0
          and tuning.BY_NAME["trim_right"].coerce(2.0) == 1.0)

    spec_defaults = {
        "max_pwm_duty": 200,
        "max_linear_speed_mm_s": 300,
        "max_angular_speed_mdeg_s": 60000,
        "lidar_min_range_mm": 150,
        "lidar_max_range_mm": 6000,
        "lidar_angle_filter": [],
        "obstacle_inflation_radius_mm": 150,
        "collision_stop_distance_mm": 100,
    }
    for name, expected in spec_defaults.items():
        check(f"default {name} == {expected}",
              tuning.DEFAULTS[name] == expected,
              f"got {tuning.DEFAULTS.get(name)}")

    accepted, errors = tuning.coerce_updates({"max_pwm_duty": 999})
    check("over-range clamps to 255", accepted["max_pwm_duty"] == 255,
          str(accepted))
    accepted, errors = tuning.coerce_updates({"max_pwm_duty": -5})
    check("under-range clamps to 0", accepted["max_pwm_duty"] == 0, str(accepted))

    accepted, errors = tuning.coerce_updates({"nonexistent_param": 1})
    check("unknown param reported, not silently dropped",
          not accepted and len(errors) == 1, f"{accepted} {errors}")

    accepted, errors = tuning.coerce_updates({"max_pwm_duty": "abc"})
    check("non-numeric rejected", not accepted and errors, f"{accepted} {errors}")

    routed = tuning.split_by_target(
        {"max_linear_speed_mm_s": 250, "lidar_min_range_mm": 200,
         "obstacle_inflation_radius_mm": 180}
    )
    check("linear speed routes to arduino + ros",
          "max_linear_speed_mm_s" in routed["arduino"]
          and "max_linear_speed_mm_s" in routed["ros"], str(routed))
    check("lidar range routes to nodemcu, not arduino",
          "lidar_min_range_mm" in routed["nodemcu"]
          and "lidar_min_range_mm" not in routed["arduino"], str(routed))
    check("inflation radius is ros-only",
          routed["ros"].get("obstacle_inflation_radius_mm") == 180
          and not routed["arduino"].get("obstacle_inflation_radius_mm"),
          str(routed))

    check("firmware key shortening",
          tuning.to_firmware_key("max_linear_speed_mm_s") == "max_linear_speed"
          and tuning.to_firmware_key("collision_stop_distance_mm") == "collision_stop_mm"
          and tuning.to_firmware_key("max_pwm_duty") == "max_pwm_duty")

    triples = tuning.ros_overrides({"max_linear_speed_mm_s": 300})
    values = [round(v, 4) for _, _, v in triples]
    check("mm/s -> m/s conversion for ROS", all(v == 0.3 for v in values),
          str(triples))

    triples = tuning.ros_overrides({"max_angular_speed_mdeg_s": 60000})
    # 60000 mdeg/s = 60 deg/s = 1.0472 rad/s
    check("mdeg/s -> rad/s conversion for ROS",
          all(abs(v - 1.0471975) < 1e-4 for _, _, v in triples), str(triples))

    masks = tuning.BY_NAME["lidar_angle_filter"].coerce(
        [{"start_deg": 350, "end_deg": 10}]
    )
    check("angle mask accepts a wrapping sector",
          masks == [{"start_deg": 350.0, "end_deg": 10.0}], str(masks))
    check("angle mask empty by default",
          tuning.BY_NAME["lidar_angle_filter"].coerce(None) == [])

    measured = {p.name for p in tuning.PARAMS if p.measured}
    check("odometry geometry flagged as measure-and-set",
          measured == {"encoder_cpr", "wheel_diameter_mm", "wheel_base_mm"},
          str(measured))


def test_state_and_rle() -> None:
    from state import MapState, rle_encode
    from ai_advisor import _rle_decode

    print("\n[state / RLE]")
    cells = [-1] * 100 + [0] * 50 + [100] * 10 + [-1] * 40
    encoded = rle_encode(cells)
    check("RLE compresses uniform runs", len(encoded) == 8, str(encoded))
    decoded = _rle_decode(encoded, len(cells))
    check("RLE round-trips exactly", decoded == cells)
    check("RLE of empty input is empty", rle_encode([]) == [])

    grid = MapState()
    check("empty map detected", grid.is_empty)
    grid.width, grid.height = 10, 10
    check("sized map not empty", not grid.is_empty)


def test_scan_binning() -> None:
    from ros_bridge import scan_to_ranges

    print("\n[scan -> LaserScan binning]")
    ranges = scan_to_ranges([0.0, 90.0, 180.0, 270.0],
                            [1000.0, 2000.0, 3000.0, 4000.0], bins=360)
    check("360 bins produced", len(ranges) == 360, str(len(ranges)))
    check("0 deg -> 1.0 m", abs(ranges[0] - 1.0) < 1e-6, str(ranges[0]))
    check("90 deg -> 2.0 m", abs(ranges[90] - 2.0) < 1e-6, str(ranges[90]))
    check("empty bin is inf", ranges[45] == float("inf"), str(ranges[45]))

    # Nearest-wins is the safety-critical property.
    ranges = scan_to_ranges([10.1, 10.4], [3000.0, 500.0], bins=360)
    check("nearest return wins within a bin", abs(ranges[10] - 0.5) < 1e-6,
          str(ranges[10]))

    ranges = scan_to_ranges([0.0, 1.0], [50.0, 9000.0], bins=360,
                            range_min_m=0.15, range_max_m=6.0)
    check("out-of-range samples discarded",
          ranges[0] == float("inf") and ranges[1] == float("inf"))

    ranges = scan_to_ranges([359.99], [1000.0], bins=360)
    check("angle at 360 boundary does not overflow the array",
          ranges[359] == 1.0, str(ranges[359]))


def test_map_summary() -> None:
    from ai_advisor import summarize_map
    from state import rle_encode

    print("\n[map summary / flood fill]")
    check("empty grid handled", summarize_map(0, 0, 0.05, [])["empty"] is True)

    # 20x20, two 6x6 free rooms separated by a wall of occupied cells.
    width = height = 20
    cells = [-1] * (width * height)
    for y in range(2, 8):
        for x in range(2, 8):
            cells[y * width + x] = 0
    for y in range(2, 8):
        for x in range(12, 18):
            cells[y * width + x] = 0
    for y in range(20):
        cells[y * width + 10] = 100

    summary = summarize_map(width, height, 0.20, rle_encode(cells))
    check("two rooms detected", summary["room_like_regions"] == 2,
          str(summary["room_like_regions"]))
    check("free cell count correct", summary["cells"]["free"] == 72,
          str(summary["cells"]["free"]))
    check("occupied cell count correct", summary["cells"]["occupied"] == 20,
          str(summary["cells"]["occupied"]))
    check("explored pct computed",
          summary["ratios"]["explored_pct"] == 23.0,
          str(summary["ratios"]["explored_pct"]))
    check("region areas reported in m2",
          summary["region_areas_m2"] == [1.44, 1.44],
          str(summary["region_areas_m2"]))


def test_ai_validation() -> None:
    from ai_advisor import _extract_json, _is_zero, _validate_suggestions

    print("\n[AI suggestion validation]")
    check("zero price string detected",
          _is_zero("0") and _is_zero("0.0") and _is_zero("0e0"))
    check("nonzero price rejected", not _is_zero("0.0000012") and not _is_zero(""))

    check("plain JSON parsed", _extract_json('{"a": 1}') == {"a": 1})
    check("fenced JSON parsed",
          _extract_json('```json\n{"a": 1}\n```') == {"a": 1})
    check("prose-wrapped JSON parsed",
          _extract_json('Sure! {"a": 1} hope that helps') == {"a": 1})
    check("unparseable returns None", _extract_json("no json here") is None)

    accepted, rejected = _validate_suggestions([
        {"parameter": "max_pwm_duty", "suggested_value": 180, "reasoning": "ok"},
        {"parameter": "max_pwm_duty", "suggested_value": 9999, "reasoning": "high"},
        {"parameter": "invented_param", "suggested_value": 5, "reasoning": "no"},
        {"parameter": "max_pwm_duty", "suggested_value": "fast", "reasoning": "no"},
    ])
    check("valid suggestion accepted", accepted[0]["suggested_value"] == 180,
          str(accepted[0] if accepted else None))
    check("out-of-range suggestion clamped, not dropped",
          len(accepted) == 2 and accepted[1]["suggested_value"] == 255
          and "clamped" in accepted[1]["note"], str(accepted))
    check("hallucinated parameter rejected",
          any(r["parameter"] == "invented_param" for r in rejected), str(rejected))
    check("non-numeric value rejected", len(rejected) == 2, str(rejected))
    check("non-list input handled", _validate_suggestions("nope") == ([], []))


def test_protocol_end_to_end() -> None:
    """Drive the real WebSocket endpoints with a fake Arduino and NodeMCU."""
    from fastapi.testclient import TestClient

    import main

    print("\n[WebSocket protocol end-to-end]")
    with TestClient(main.app) as client:
        with client.websocket_connect("/ws/motion") as arduino:
            arduino.send_json({
                "type": "hello", "device": "arduino_uno_r4", "fw": "1.0.0",
                "ip": "192.168.1.77",
                "config": {"max_pwm_duty": 200, "running": False},
            })
            resync = arduino.receive_json()
            check("Arduino gets a tuning re-sync on hello",
                  resync["type"] == "tuning_update", str(resync))
            check("re-sync uses firmware key names",
                  "max_linear_speed" in resync and "max_linear_speed_mm_s" not in resync,
                  str(resync))

            status = client.get("/api/status").json()
            check("Arduino shows connected", status["devices"]["arduino"]["connected"])
            check("boot state is stopped (§11.2)", status["running"] is False)

            with client.websocket_connect("/ws/app") as app_ws:
                # Drain the initial burst.
                topics = set()
                for _ in range(6):
                    topics.add(app_ws.receive_json()["topic"])
                check("browser gets schema + status + tuning on connect",
                      {"schema", "status", "tuning"} <= topics, str(topics))

                # Drive while stopped must be refused.
                app_ws.send_json({"type": "drive", "linear_mm_s": 200,
                                  "angular_mdeg_s": 0})
                reply = _await_topic(app_ws, "reply")
                check("drive refused while stopped",
                      reply["data"]["ok"] is False
                      and "stopped" in reply["data"]["msg"], str(reply))

                # Start.
                app_ws.send_json({"type": "control", "action": "start"})
                control = arduino.receive_json()
                check("start reaches the Arduino as §11.3 control",
                      control == {"type": "control", "action": "start"},
                      str(control))
                reply = _await_topic(app_ws, "reply")
                check("start acknowledged to the browser",
                      reply["data"]["ok"] is True, str(reply))

                status = client.get("/api/status").json()
                check("backend run-state is running", status["running"] is True)

                # Drive is now honoured and clamped to the tuning limit.
                app_ws.send_json({"type": "drive", "linear_mm_s": 5000,
                                  "angular_mdeg_s": 0})
                cmd = arduino.receive_json()
                check("cmd_vel sent to Arduino", cmd["type"] == "cmd_vel", str(cmd))
                check("drive clamped to max_linear_speed_mm_s (300)",
                      cmd["linear_mm_s"] == 300.0, str(cmd))

                # Tuning update: only changed fields, firmware key names.
                app_ws.send_json({
                    "type": "tuning_update",
                    "values": {"max_pwm_duty": 150, "max_linear_speed_mm_s": 250},
                })
                update = arduino.receive_json()
                check("tuning_update reaches Arduino",
                      update["type"] == "tuning_update", str(update))
                check("tuning_update carries only changed fields (§11.3)",
                      set(update) == {"type", "max_pwm_duty", "max_linear_speed"},
                      str(update))
                check("values are the clamped ones",
                      update["max_pwm_duty"] == 150
                      and update["max_linear_speed"] == 250, str(update))

                reply = _await_topic(app_ws, "reply")
                check("tuning_update acknowledged to the browser",
                      reply["data"]["ok"] is True, str(reply))

                tuning_state = client.get("/api/tuning").json()
                check("changed values marked unsaved (§11.4 UI indicator)",
                      set(tuning_state["dirty"])
                      == {"max_pwm_duty", "max_linear_speed_mm_s"},
                      str(tuning_state["dirty"]))

                # New limit takes effect immediately, no reflash.
                app_ws.send_json({"type": "drive", "linear_mm_s": 5000,
                                  "angular_mdeg_s": 0})
                cmd = arduino.receive_json()
                check("new speed cap applies with no reflash (§11)",
                      cmd["linear_mm_s"] == 250.0, str(cmd))

                # Motion calibration travels the same path. kick_duty is
                # deliberately over-range to prove the clamp happens before
                # the value reaches the motors.
                #
                # Note trim_right is NOT tested for clamping here: its cap is
                # 1.0, which is also its default, so an over-range request
                # clamps to the value already held and the §11.3 changed-only
                # filter correctly drops it from the wire. The clamp itself is
                # asserted in test_tuning_registry.
                app_ws.send_json({
                    "type": "tuning_update",
                    "values": {"trim_right": 0.92, "turn_boost": 1.8,
                               "kick_duty": 999},
                })
                calib = arduino.receive_json()
                check("calibration reaches Arduino under its own key names",
                      set(calib) == {"type", "trim_right", "turn_boost",
                                     "kick_duty"},
                      str(calib))
                check("kick_duty clamped to 255 before it reaches the motors",
                      calib["kick_duty"] == 255, str(calib))
                check("in-range calibration passes through unchanged",
                      calib["turn_boost"] == 1.8 and calib["trim_right"] == 0.92,
                      str(calib))
                _await_topic(app_ws, "reply")

                # Save as default.
                app_ws.send_json({"type": "tuning_save"})
                save = arduino.receive_json()
                check("tuning_save reaches Arduino",
                      save == {"type": "tuning_save"}, str(save))
                arduino.send_json({"type": "ack", "for": "tuning_save", "ok": True})
                _await_topic(app_ws, "reply")  # drain the save acknowledgement

                # E-stop.
                app_ws.send_json({"type": "control", "action": "estop"})
                estop = arduino.receive_json()
                check("e-stop sent as its own action",
                      estop == {"type": "control", "action": "estop"}, str(estop))
                _await_topic(app_ws, "reply")  # drain the e-stop acknowledgement
                status = client.get("/api/status").json()
                check("e-stop stops the bot and latches",
                      status["running"] is False
                      and status["estop_latched"] is True, str(status))

                # Odometry uplink. Two frames: travelled distance is the sum of
                # deltas between successive poses, so one frame alone is 0 mm.
                arduino.send_json({
                    "type": "odom", "t_ms": 1000, "x_mm": 0.0, "y_mm": 0.0,
                    "theta_rad": 0.0, "linear_mm_s": 0.0, "angular_mdeg_s": 0.0,
                    "ticks_l": 0, "ticks_r": 0, "duty_l": 0, "duty_r": 0,
                    "running": False, "collision": False, "rssi": -58,
                })
                arduino.send_json({
                    "type": "odom", "t_ms": 1100, "x_mm": 120.0, "y_mm": 35.0,
                    "theta_rad": 0.5, "linear_mm_s": 90.0, "angular_mdeg_s": 1500.0,
                    "ticks_l": 400, "ticks_r": 410, "duty_l": 90, "duty_r": 92,
                    "running": False, "collision": False, "rssi": -58,
                })
                status = _poll_until(
                    client, lambda s: s["odom"]["x_mm"] == 120.0
                )
                check("odometry lands in state", status["odom"]["x_mm"] == 120.0,
                      str(status["odom"]))
                check("battery omitted when no divider fitted (§6.1)",
                      "battery_v" not in status["odom"], str(status["odom"]))
                status = _poll_until(
                    client, lambda s: s["session"]["distance_travelled_mm"] > 0
                )
                check("pose trail accumulates travelled distance",
                      abs(status["session"]["distance_travelled_mm"] - 125.0) < 1.0,
                      str(status["session"]))

                # NodeMCU: scan uplink and its own tuning route.
                with client.websocket_connect("/ws/lidar") as nodemcu:
                    nodemcu.send_json({
                        "type": "hello", "device": "nodemcu_lidar", "fw": "1.0.0",
                        "config": {"lidar_min_range_mm": 150,
                                   "lidar_max_range_mm": 6000},
                    })
                    resync = nodemcu.receive_json()
                    check("NodeMCU gets its own tuning re-sync",
                          resync["type"] == "tuning_update"
                          and "lidar_min_range_mm" in resync
                          and "max_pwm_duty" not in resync, str(resync))

                    nodemcu.send_json({
                        "type": "scan", "seq": 7, "t_ms": 2000, "rev_ms": 180,
                        "n": 3,
                        "angles_deg": [0.0, 90.0, 180.0],
                        "dists_mm": [1500, 800, 2200],
                        "quality": [40, 45, 38],
                        "min_distance_mm": 800,
                    })
                    proximity = arduino.receive_json()
                    check("nearest LIDAR return forwarded to Arduino for "
                          "firmware collision-stop",
                          proximity == {"type": "proximity",
                                        "min_distance_mm": 800},
                          str(proximity))

                    scan = client.get("/api/scan").json()
                    check("scan stored with all three samples",
                          scan["n"] == 3 and scan["min_distance_mm"] == 800.0,
                          str(scan))

                    # Truncated frame must not desynchronise the arrays.
                    nodemcu.send_json({
                        "type": "scan", "seq": 8, "t_ms": 2200,
                        "angles_deg": [0.0, 90.0, 180.0],
                        "dists_mm": [1500, 800],
                    })
                    arduino.receive_json()  # proximity for seq 8
                    scan = client.get("/api/scan").json()
                    check("truncated scan frame truncates safely",
                          len(scan["angles_deg"]) == len(scan["dists_mm"]) == 2,
                          str(scan))

                    nodemcu.send_json({
                        "type": "log", "source": "lidar", "level": "warn",
                        "msg": "test log over wifi",
                    })
                    logs = _poll_logs(client, "test log over wifi")
                    check("NodeMCU logs arrive over WiFi (§6.3 / §3 note)",
                          logs is not None, "log not found")

                # Bad input handling.
                app_ws.send_json({"type": "nope"})
                reply = _await_topic(app_ws, "reply")
                check("unknown command reported",
                      reply["data"]["ok"] is False, str(reply))

                app_ws.send_json({"type": "nav_goal", "x_m": 5000, "y_m": 0})
                reply = _await_topic(app_ws, "reply")
                check("absurd nav goal rejected by bounds check",
                      reply["data"]["ok"] is False, str(reply))

        # Arduino gone: run-state must fall back to stopped.
        status = client.get("/api/status").json()
        check("disconnect forces stopped state",
              status["running"] is False
              and status["devices"]["arduino"]["connected"] is False,
              str(status["devices"]["arduino"]))

        health = client.get("/healthz").json()
        check("healthz responds", health["ok"] is True, str(health))


def _await_topic(ws, topic: str, limit: int = 40):
    """Read frames until one matches ``topic``."""
    for _ in range(limit):
        frame = ws.receive_json()
        if frame.get("topic") == topic:
            return frame
    raise AssertionError(f"topic {topic!r} not seen within {limit} frames")


def _poll_until(client, predicate, attempts: int = 40):
    import time

    status = {}
    for _ in range(attempts):
        status = client.get("/api/status").json()
        if predicate(status):
            return status
        time.sleep(0.02)
    return status


def _poll_logs(client, needle: str, attempts: int = 40):
    import time

    for _ in range(attempts):
        entries = client.get("/api/logs").json()["entries"]
        for entry in entries:
            if needle in entry.get("msg", ""):
                return entry
        time.sleep(0.02)
    return None


def main_() -> int:
    print("SLAM Bot backend smoke test")
    print("=" * 60)
    test_tuning_registry()
    test_state_and_rle()
    test_scan_binning()
    test_map_summary()
    test_ai_validation()
    test_protocol_end_to_end()

    print("\n" + "=" * 60)
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s)")
        for name in FAILURES:
            print(f"  - {name}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main_())
