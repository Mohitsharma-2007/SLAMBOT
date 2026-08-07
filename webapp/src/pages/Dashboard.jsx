/**
 * Dashboard — §6.1
 *
 * Connection status, live velocity/encoder readouts, the Start/Stop run-state
 * toggle, e-stop, and manual drive controls.
 *
 * Two spec details worth calling out in the code:
 *  - Start/Stop reflects backend state, never local state, so it survives a
 *    page refresh (§6.1) and cannot disagree with the firmware (§11.2).
 *  - Battery voltage is shown only when the Arduino actually reports it. §6.1
 *    says to omit it rather than fake a reading, so an absent divider produces
 *    an explicit "not fitted" note instead of a plausible-looking number.
 */

import { useCallback, useEffect, useRef, useState } from 'react'
import { useBot } from '../lib/store.jsx'

function fmt(value, digits = 0) {
  if (value === null || value === undefined || Number.isNaN(value)) return '--'
  return Number(value).toFixed(digits)
}

function duration(ms) {
  if (!ms || ms < 0) return '--'
  const total = Math.floor(ms / 1000)
  const h = Math.floor(total / 3600)
  const m = Math.floor((total % 3600) / 60)
  const s = total % 60
  if (h) return `${h}h ${m}m`
  if (m) return `${m}m ${s}s`
  return `${s}s`
}

function DeviceCard({ title, device, subtitle }) {
  const online = device?.connected
  const stale = online && device?.stale_ms != null && device.stale_ms > 3000
  return (
    <div className="stat">
      <div className="stat-label">{title}</div>
      <div className="row" style={{ gap: 7, marginTop: 2 }}>
        <span className={`dot ${online ? (stale ? 'warn' : 'ok') : 'err'}`} />
        <span style={{ fontWeight: 600 }}>
          {online ? (stale ? 'Stale' : 'Connected') : 'Disconnected'}
        </span>
      </div>
      <div className="stat-sub">
        {online ? (
          <>
            {device.ip ?? 'unknown ip'}
            {device.firmware ? ` · fw ${device.firmware}` : ''}
            {device.rssi != null ? ` · ${device.rssi} dBm` : ''}
            {device.stale_ms != null ? ` · ${device.stale_ms} ms ago` : ''}
          </>
        ) : (
          subtitle
        )}
      </div>
    </div>
  )
}

export default function Dashboard() {
  const {
    connected,
    status,
    tuning,
    running,
    estopLatched,
    controlMode,
    arduino,
    nodemcu,
    start,
    stop,
    estop,
    drive,
    setMode,
    resetSession,
    lastReply,
  } = useBot()

  const odom = status?.odom
  const session = status?.session
  const scanMeta = status?.scan_meta

  const maxLinear = tuning?.values?.max_linear_speed_mm_s ?? 300
  const maxAngular = tuning?.values?.max_angular_speed_mdeg_s ?? 60000

  const [speedPct, setSpeedPct] = useState(60)
  const heldKeys = useRef(new Set())
  const repeatRef = useRef(null)

  /**
   * Manual drive must be *repeated*, not sent once: the backend expires a
   * manual command after ~500 ms so a closed laptop lid or a crashed tab cannot
   * leave the robot driving. Holding a key therefore refreshes the command at
   * 5 Hz, comfortably inside that window.
   */
  const sendHeld = useCallback(() => {
    const keys = heldKeys.current
    const scale = speedPct / 100
    let linear = 0
    let angular = 0
    if (keys.has('forward')) linear += maxLinear * scale
    if (keys.has('back')) linear -= maxLinear * scale
    if (keys.has('left')) angular += maxAngular * scale
    if (keys.has('right')) angular -= maxAngular * scale
    drive(linear, angular)
  }, [drive, maxLinear, maxAngular, speedPct])

  // The repeat interval must call the *current* sendHeld, not the one that
  // existed when the key went down — otherwise moving the speed slider (or a
  // tuning push changing the caps) mid-hold keeps resending the stale value
  // until you release and press again.
  const sendHeldRef = useRef(sendHeld)
  useEffect(() => {
    sendHeldRef.current = sendHeld
  }, [sendHeld])

  const press = useCallback((direction) => {
    heldKeys.current.add(direction)
    sendHeldRef.current()
    if (!repeatRef.current) {
      repeatRef.current = setInterval(() => sendHeldRef.current(), 200)
    }
  }, [])

  const release = useCallback(
    (direction) => {
      heldKeys.current.delete(direction)
      if (heldKeys.current.size === 0) {
        if (repeatRef.current) {
          clearInterval(repeatRef.current)
          repeatRef.current = null
        }
        drive(0, 0)
      } else {
        sendHeldRef.current()
      }
    },
    [drive],
  )

  const releaseAll = useCallback(() => {
    heldKeys.current.clear()
    if (repeatRef.current) {
      clearInterval(repeatRef.current)
      repeatRef.current = null
    }
    drive(0, 0)
  }, [drive])

  // Keyboard driving: WASD / arrows, space for e-stop.
  useEffect(() => {
    const KEY_MAP = {
      ArrowUp: 'forward',
      ArrowDown: 'back',
      ArrowLeft: 'left',
      ArrowRight: 'right',
      w: 'forward',
      s: 'back',
      a: 'left',
      d: 'right',
      W: 'forward',
      S: 'back',
      A: 'left',
      D: 'right',
    }

    const onKeyDown = (event) => {
      if (event.target.tagName === 'INPUT' || event.target.tagName === 'TEXTAREA') {
        return
      }
      if (event.code === 'Space') {
        event.preventDefault()
        releaseAll()
        estop()
        return
      }
      const direction = KEY_MAP[event.key]
      if (!direction) return
      event.preventDefault()
      if (!heldKeys.current.has(direction)) press(direction)
    }

    const onKeyUp = (event) => {
      const direction = KEY_MAP[event.key]
      if (direction) release(direction)
    }

    // A page switch or lost focus must not leave a key stuck down.
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', releaseAll)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      window.removeEventListener('blur', releaseAll)
      if (repeatRef.current) clearInterval(repeatRef.current)
    }
  }, [press, release, releaseAll, estop])

  // Stop repeating the moment the run-state goes false.
  useEffect(() => {
    if (!running) releaseAll()
  }, [running, releaseAll])

  const driveDisabled = !connected || !running || controlMode !== 'manual'

  return (
    <>
      {estopLatched && (
        <div className="notice err">
          <strong>Emergency stop is latched.</strong>
          Motors are disabled. Clear the obstruction, then press Start to resume.
        </div>
      )}

      {!arduino?.connected && connected && (
        <div className="notice warn">
          <strong>Arduino not connected.</strong>
          The bot cannot move until the Uno R4 connects to{' '}
          <span className="mono">/ws/motion</span>. Check its WiFi credentials
          and the backend IP in <span className="mono">secrets.h</span>.
        </div>
      )}

      {/* --- run control ------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">Run control</div>
        <div className="row">
          <button
            className="success"
            onClick={start}
            disabled={!connected || running || !arduino?.connected}
            title="Enables the motors. Firmware boots stopped and stays stopped until this is pressed (§11.2)."
          >
            ▶ Start
          </button>
          <button
            className="danger"
            onClick={stop}
            disabled={!connected || !running}
            title="Disables motors and ignores all cmd_vel until Start is pressed again"
          >
            ■ Stop
          </button>
          <button className="estop" onClick={estop} disabled={!connected}>
            EMERGENCY STOP
          </button>

          <div className="spacer" />

          <label className="small dim">Control mode</label>
          <button
            className={`toggle sm ${controlMode === 'manual' ? 'on' : ''}`}
            onClick={() => setMode('manual')}
            disabled={!connected}
          >
            Manual
          </button>
          <button
            className={`toggle sm ${controlMode === 'nav2' ? 'on' : ''}`}
            onClick={() => setMode('nav2')}
            disabled={!connected}
            title="Hand velocity control to Nav2. Set goals on the Live Map page."
          >
            Nav2 autonomous
          </button>
        </div>
        <div className="param-help" style={{ marginTop: 9 }}>
          Stop is a firmware-level gate, not just a UI state: the Arduino zeroes
          both motors itself and also forces Stop on its own if no backend
          message arrives for 2 seconds (§11.2). Run-state lives on the backend,
          so it survives a page refresh.
        </div>
      </div>

      {/* --- devices ----------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">Connections</div>
        <div className="grid grid-4">
          <DeviceCard
            title="Arduino Uno R4 WiFi"
            device={arduino}
            subtitle="motion + odometry · /ws/motion"
          />
          <DeviceCard
            title="NodeMCU (LIDAR)"
            device={nodemcu}
            subtitle="scan relay · /ws/lidar"
          />
          <div className="stat">
            <div className="stat-label">ROS2 / SLAM</div>
            <div className="row" style={{ gap: 7, marginTop: 2 }}>
              <span className={`dot ${status?.ros?.available ? 'ok' : 'warn'}`} />
              <span style={{ fontWeight: 600 }}>
                {status?.ros?.available ? 'Active' : 'Not running'}
              </span>
            </div>
            <div className="stat-sub">
              {status?.ros?.nodes?.length
                ? status.ros.nodes.join(', ')
                : 'no nodes seen — /map unavailable'}
            </div>
          </div>
          <div className="stat">
            <div className="stat-label">Uptime</div>
            <div className="stat-value">{duration(status?.uptime_ms)}</div>
            <div className="stat-sub">
              {running ? `running ${duration(
                status?.run_started_at_ms ? Date.now() - status.run_started_at_ms : 0,
              )}` : 'idle'}
            </div>
          </div>
        </div>
      </div>

      {/* --- telemetry --------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">Live telemetry</div>
        <div className="grid grid-4">
          <div className="stat">
            <div className="stat-label">Linear velocity</div>
            <div className="stat-value">
              {fmt(odom?.linear_mm_s, 1)}
              <span className="stat-unit">mm/s</span>
            </div>
            <div className="stat-sub">cap {maxLinear} mm/s</div>
          </div>
          <div className="stat">
            <div className="stat-label">Angular velocity</div>
            <div className="stat-value">
              {fmt(odom ? odom.angular_mdeg_s / 1000 : null, 1)}
              <span className="stat-unit">°/s</span>
            </div>
            <div className="stat-sub">cap {(maxAngular / 1000).toFixed(0)} °/s</div>
          </div>
          <div className="stat">
            <div className="stat-label">Encoder ticks</div>
            <div className="stat-value">
              {odom ? `${odom.ticks_l} / ${odom.ticks_r}` : '--'}
            </div>
            <div className="stat-sub">left / right, cumulative</div>
          </div>
          <div className="stat">
            <div className="stat-label">Motor duty</div>
            <div className="stat-value">
              {odom ? `${odom.duty_l} / ${odom.duty_r}` : '--'}
            </div>
            <div className="stat-sub">
              of ±{tuning?.values?.max_pwm_duty ?? 200} cap
            </div>
          </div>

          <div className="stat">
            <div className="stat-label">Pose</div>
            <div className="stat-value" style={{ fontSize: 15 }}>
              {odom
                ? `${fmt(odom.x_mm, 0)}, ${fmt(odom.y_mm, 0)} mm`
                : '--'}
            </div>
            <div className="stat-sub">
              heading {fmt(odom?.theta_deg, 1)}°
            </div>
          </div>
          <div className="stat">
            <div className="stat-label">Nearest obstacle</div>
            <div className="stat-value">
              {scanMeta?.min_distance_mm != null
                ? fmt(scanMeta.min_distance_mm, 0)
                : '--'}
              <span className="stat-unit">mm</span>
            </div>
            <div className="stat-sub">
              stop below {tuning?.values?.collision_stop_distance_mm ?? 100} mm
            </div>
          </div>
          <div className="stat">
            <div className="stat-label">Scan rate</div>
            <div className="stat-value">
              {scanMeta?.rev_ms ? fmt(1000 / scanMeta.rev_ms, 1) : '--'}
              <span className="stat-unit">Hz</span>
            </div>
            <div className="stat-sub">{scanMeta?.n ?? 0} points/rev</div>
          </div>
          <div className="stat">
            <div className="stat-label">Battery</div>
            {odom?.battery_v != null ? (
              <>
                <div className="stat-value">
                  {fmt(odom.battery_v, 2)}
                  <span className="stat-unit">V</span>
                </div>
                <div className="stat-sub">
                  {odom.battery_v < 6.6
                    ? 'LOW — land the bot and charge'
                    : '2S LiPo, 6.6–8.4 V'}
                </div>
              </>
            ) : (
              <>
                <div className="stat-value faint" style={{ fontSize: 15 }}>
                  not fitted
                </div>
                <div className="stat-sub">
                  add a divider on A0 and set BATTERY_SENSE_ENABLED
                </div>
              </>
            )}
          </div>
        </div>

        {odom?.collision && (
          <div className="notice err" style={{ marginTop: 12, marginBottom: 0 }}>
            <strong>Collision stop active.</strong>
            An obstacle is within{' '}
            {tuning?.values?.collision_stop_distance_mm ?? 100} mm. Forward
            motion is blocked in firmware; reverse still works so you can back
            out.
          </div>
        )}
      </div>

      {/* --- manual drive ------------------------------------------------ */}
      <div className="grid grid-2">
        <div className="panel">
          <div className="panel-title">Manual drive</div>
          {driveDisabled && (
            <div className="notice warn">
              {!running
                ? 'Press Start to enable the motors.'
                : controlMode !== 'manual'
                  ? 'Control mode is Nav2 — switch to Manual to drive by hand.'
                  : 'Not connected.'}
            </div>
          )}
          <div className="row" style={{ alignItems: 'flex-start', gap: 20 }}>
            <div className="dpad">
              <div />
              <button
                onMouseDown={() => press('forward')}
                onMouseUp={() => release('forward')}
                onMouseLeave={() => release('forward')}
                onTouchStart={(e) => {
                  e.preventDefault()
                  press('forward')
                }}
                onTouchEnd={() => release('forward')}
                disabled={driveDisabled}
              >
                ▲
              </button>
              <div />
              <button
                onMouseDown={() => press('left')}
                onMouseUp={() => release('left')}
                onMouseLeave={() => release('left')}
                onTouchStart={(e) => {
                  e.preventDefault()
                  press('left')
                }}
                onTouchEnd={() => release('left')}
                disabled={driveDisabled}
              >
                ◀
              </button>
              <button
                className="stop-cell danger"
                onClick={() => {
                  releaseAll()
                  stop()
                }}
                disabled={!connected}
              >
                STOP
              </button>
              <button
                onMouseDown={() => press('right')}
                onMouseUp={() => release('right')}
                onMouseLeave={() => release('right')}
                onTouchStart={(e) => {
                  e.preventDefault()
                  press('right')
                }}
                onTouchEnd={() => release('right')}
                disabled={driveDisabled}
              >
                ▶
              </button>
              <div />
              <button
                onMouseDown={() => press('back')}
                onMouseUp={() => release('back')}
                onMouseLeave={() => release('back')}
                onTouchStart={(e) => {
                  e.preventDefault()
                  press('back')
                }}
                onTouchEnd={() => release('back')}
                disabled={driveDisabled}
              >
                ▼
              </button>
              <div />
            </div>

            <div style={{ flex: 1, minWidth: 180 }}>
              <label className="small dim">Speed: {speedPct}% of cap</label>
              <input
                type="range"
                min="10"
                max="100"
                step="5"
                value={speedPct}
                onChange={(e) => setSpeedPct(Number(e.target.value))}
              />
              <div className="param-help">
                Commanding{' '}
                <span className="mono">
                  {Math.round((maxLinear * speedPct) / 100)} mm/s
                </span>{' '}
                /{' '}
                <span className="mono">
                  {Math.round((maxAngular * speedPct) / 100 / 1000)} °/s
                </span>
                .
              </div>
              <div className="param-help" style={{ marginTop: 10 }}>
                <strong className="dim">Keyboard:</strong> WASD or arrow keys to
                drive, <span className="mono">Space</span> for emergency stop.
                Commands repeat while held — the backend expires them after
                500 ms so a frozen tab cannot leave the bot driving.
              </div>
            </div>
          </div>
        </div>

        <div className="panel">
          <div className="panel-title">
            Session
            <button className="sm" onClick={resetSession} disabled={!connected}>
              Reset counters
            </button>
          </div>
          <table className="data">
            <tbody>
              <tr>
                <td>Distance travelled</td>
                <td className="num">
                  {fmt(session?.distance_travelled_mm / 1000, 2)} m
                </td>
              </tr>
              <tr>
                <td>Collision events</td>
                <td className="num">{session?.collisions ?? 0}</td>
              </tr>
              <tr>
                <td>Possible stalls</td>
                <td className="num">{session?.stuck_events ?? 0}</td>
              </tr>
              <tr>
                <td>Watchdog / auto stops</td>
                <td className="num">{session?.watchdog_trips ?? 0}</td>
              </tr>
              <tr>
                <td>Log entries (warn / error)</td>
                <td className="num">
                  {status?.log_counts?.warn ?? 0} / {status?.log_counts?.error ?? 0}
                </td>
              </tr>
              <tr>
                <td>Map</td>
                <td className="num">
                  {status?.map_meta?.width
                    ? `${status.map_meta.width}×${status.map_meta.height} @ ${status.map_meta.resolution} m`
                    : 'none'}
                </td>
              </tr>
            </tbody>
          </table>
          {lastReply && Date.now() - lastReply.at < 6000 && (
            <div
              className={`notice ${lastReply.ok ? 'info' : 'warn'}`}
              style={{ marginTop: 12, marginBottom: 0 }}
            >
              {lastReply.msg}
            </div>
          )}
        </div>
      </div>
    </>
  )
}
