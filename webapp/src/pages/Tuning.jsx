/**
 * Tuning Panel — §6.5, driven by the §10 parameter table.
 *
 * The parameter list is *not* hardcoded here: it comes from the backend's
 * registry over the `schema` WebSocket topic. That means the UI cannot drift out
 * of sync with what the backend actually accepts, and a new parameter appears
 * here automatically once declared in backend/tuning.py.
 *
 * §11 behaviour this implements:
 *  - Every edit live-applies over WebSocket, reaching the MCU with no reflash
 *    and no reboot (§11.3). Slider drags are debounced so a drag sends a handful
 *    of updates rather than one per pixel.
 *  - "Save as default" persists to MCU flash (§11.4); until then the backend
 *    reports which values are dirty and each shows an "unsaved" badge, so the
 *    live-vs-persisted distinction is visible as the spec requires.
 *  - "Reset to default" clears both the live values and the saved config.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useBot } from '../lib/store.jsx'

const TARGET_LABEL = {
  arduino: 'Arduino',
  nodemcu: 'NodeMCU',
  ros: 'ROS2/Nav2',
}

function Param({ param, value, dirty, onChange }) {
  // Local text state so typing "1" on the way to "150" doesn't immediately
  // clamp and fight the cursor.
  const [text, setText] = useState(String(value ?? ''))
  const editing = useRef(false)

  useEffect(() => {
    if (!editing.current) setText(String(value ?? ''))
  }, [value])

  const commit = (raw) => {
    const parsed = Number(raw)
    if (!Number.isFinite(parsed)) {
      setText(String(value ?? ''))
      return
    }
    onChange(parsed)
  }

  const hasRange = param.min !== null && param.max !== null

  return (
    <div className="param">
      <div className="param-head">
        <span className="param-name">{param.label}</span>
        <span className="param-key">{param.name}</span>
        {dirty && <span className="badge unsaved">unsaved</span>}
        {param.measured && (
          <span className="badge measure" title="§10: measure this on your robot — the shipped value is a placeholder, not a safe default">
            measure
          </span>
        )}
        <div className="spacer" />
        {param.targets.map((target) => (
          <span key={target} className="badge target">
            {TARGET_LABEL[target] ?? target}
          </span>
        ))}
      </div>

      <div className="param-controls">
        {hasRange && (
          <input
            type="range"
            min={param.min}
            max={param.max}
            step={param.step ?? 1}
            value={Number(value ?? param.default)}
            onChange={(e) => onChange(Number(e.target.value))}
          />
        )}
        <input
          type="number"
          value={text}
          min={param.min ?? undefined}
          max={param.max ?? undefined}
          step={param.step ?? undefined}
          onFocus={() => {
            editing.current = true
          }}
          onChange={(e) => setText(e.target.value)}
          onBlur={(e) => {
            editing.current = false
            commit(e.target.value)
          }}
          onKeyDown={(e) => {
            if (e.key === 'Enter') {
              editing.current = false
              commit(e.currentTarget.value)
              e.currentTarget.blur()
            }
          }}
        />
        {param.unit && <span className="small faint mono">{param.unit}</span>}
      </div>

      {param.help && <div className="param-help">{param.help}</div>}
    </div>
  )
}

/** Angle-mask editor for lidar_angle_filter — a list, not a scalar. */
function AngleMaskParam({ value, onChange }) {
  const masks = Array.isArray(value) ? value : []

  const update = (index, key, raw) => {
    const next = masks.map((mask, i) =>
      i === index ? { ...mask, [key]: Number(raw) || 0 } : mask,
    )
    onChange(next)
  }

  return (
    <div className="param">
      <div className="param-head">
        <span className="param-name">Angle mask sectors</span>
        <span className="param-key">lidar_angle_filter</span>
        <div className="spacer" />
        <span className="badge target">NodeMCU</span>
      </div>

      {masks.length === 0 && (
        <div className="param-help" style={{ marginBottom: 8 }}>
          No sectors masked (the §10 default). Add one if part of your chassis
          blocks the beam and produces a permanent phantom obstacle.
        </div>
      )}

      {masks.map((mask, index) => (
        <div className="row" key={index} style={{ marginBottom: 6 }}>
          <span className="small dim">from</span>
          <input
            type="number"
            value={mask.start_deg ?? 0}
            min="0"
            max="360"
            style={{ width: 84 }}
            onChange={(e) => update(index, 'start_deg', e.target.value)}
          />
          <span className="small dim">to</span>
          <input
            type="number"
            value={mask.end_deg ?? 0}
            min="0"
            max="360"
            style={{ width: 84 }}
            onChange={(e) => update(index, 'end_deg', e.target.value)}
          />
          <span className="small faint">degrees</span>
          <button
            className="sm"
            onClick={() => onChange(masks.filter((_, i) => i !== index))}
          >
            Remove
          </button>
        </div>
      ))}

      <button
        className="sm"
        disabled={masks.length >= 4}
        onClick={() => onChange([...masks, { start_deg: 0, end_deg: 10 }])}
        title={masks.length >= 4 ? 'Firmware holds at most 4 masks' : undefined}
      >
        + Add sector
      </button>
      <div className="param-help">
        Sectors may wrap through zero (e.g. 350° → 10° masks the front 20°).
        Applied on the NodeMCU before samples are sent, so masked returns never
        reach SLAM at all.
      </div>
    </div>
  )
}

export default function Tuning() {
  const {
    schema,
    tuning,
    updateTuning,
    saveTuning,
    resetTuning,
    connected,
    arduino,
    nodemcu,
    lastReply,
    status,
    start,
    stop,
    drive,
  } = useBot()

  const [calState, setCalState] = useState({
    active: null, // 'left', 'right', 'ccw', null
    ticksStart: { l: 0, r: 0 },
    thetaStart: 0,
    elapsed: 0,
    ticksDelta: { l: 0, r: 0 },
    thetaDelta: 0,
    result: null,
    resultColor: 'info',
  })

  const runTest = useCallback((testType) => {
    if (!connected) return
    
    // 1. Arm robot if stopped
    if (!status?.running) {
      start()
    }
    
    const initialTicksL = status?.odom?.ticks_l ?? 0
    const initialTicksR = status?.odom?.ticks_r ?? 0
    const initialTheta = status?.odom?.theta_rad ?? 0
    
    setCalState({
      active: testType,
      ticksStart: { l: initialTicksL, r: initialTicksR },
      thetaStart: initialTheta,
      elapsed: 0,
      ticksDelta: { l: 0, r: 0 },
      thetaDelta: 0,
      result: 'Test in progress... Keep clear of wheels!',
      resultColor: 'info',
    })
    
    let count = 0
    const interval = setInterval(() => {
      count += 1
      
      let lin = 0
      let ang = 0
      
      if (testType === 'left') {
        // Run left wheel forward
        lin = 25
        ang = -19000
      } else if (testType === 'right') {
        // Run right wheel forward
        lin = 25
        ang = 19000
      } else if (testType === 'ccw') {
        // Spin CCW (left backward, right forward)
        lin = 0
        ang = 25000
      }
      
      drive(lin, ang)
      
      // Read current values
      const curL = status?.odom?.ticks_l ?? 0
      const curR = status?.odom?.ticks_r ?? 0
      const curTheta = status?.odom?.theta_rad ?? 0
      
      setCalState(prev => ({
        ...prev,
        elapsed: count * 100,
        ticksDelta: { l: curL - prev.ticksStart.l, r: curR - prev.ticksStart.r },
        thetaDelta: curTheta - prev.thetaStart,
      }))
      
      if (count >= 12) { // 1.2s done
        clearInterval(interval)
        drive(0, 0)
        
        setTimeout(() => {
          const finalL = status?.odom?.ticks_l ?? 0
          const finalR = status?.odom?.ticks_r ?? 0
          const finalTheta = status?.odom?.theta_rad ?? 0
          
          const deltaL = finalL - initialTicksL
          const deltaR = finalR - initialTicksR
          
          let dTheta = finalTheta - initialTheta
          while (dTheta > Math.PI) dTheta -= 2 * Math.PI
          while (dTheta < -Math.PI) dTheta += 2 * Math.PI
          const dThetaDeg = dTheta * 180 / Math.PI
          
          let resultText = ''
          let statusColor = 'info'
          
          if (testType === 'left') {
            if (deltaL > 30) {
              resultText = `Success: Left wheel counts positive (${deltaL} ticks). Left motor/encoder direction is correct!`
              statusColor = 'ok'
            } else if (deltaL < -30) {
              resultText = `Warning: Left wheel counts negative (${deltaL} ticks). Recommended: set "invert_left" parameter below to 1 to align feedback, or swap encoder A/B wires.`
              statusColor = 'warn'
            } else {
              resultText = `Error: Left wheel did not move enough (${deltaL} ticks). Verify motor power and configuration.`
              statusColor = 'err'
            }
          } else if (testType === 'right') {
            if (deltaR > 30) {
              resultText = `Success: Right wheel counts positive (${deltaR} ticks). Right motor/encoder direction is correct!`
              statusColor = 'ok'
            } else if (deltaR < -30) {
              resultText = `Warning: Right wheel counts negative (${deltaR} ticks). Recommended: set "invert_right" parameter below to 1 to align feedback, or swap encoder A/B wires.`
              statusColor = 'warn'
            } else {
              resultText = `Error: Right wheel did not move enough (${deltaR} ticks). Verify motor power and configuration.`
              statusColor = 'err'
            }
          } else if (testType === 'ccw') {
            if (dThetaDeg > 10) {
              resultText = `Success: Robot rotated CCW and Heading increased (+${dThetaDeg.toFixed(1)}°). Yaw integration is correctly aligned with ROS standard!`
              statusColor = 'ok'
            } else if (dThetaDeg < -10) {
              resultText = `Warning: Heading decreased (-${Math.abs(dThetaDeg).toFixed(1)}°). Yaw is integrating backwards. Recommended: swap encoder A/B channels or check wheel spacing configuration.`
              statusColor = 'warn'
            } else {
              resultText = `Error: Robot did not turn enough (${dThetaDeg.toFixed(1)}°). Verify wheel traction.`
              statusColor = 'err'
            }
          }
          
          setCalState(prev => ({
            ...prev,
            active: null,
            result: resultText,
            resultColor: statusColor,
          }))
        }, 100)
      }
    }, 100)
  }, [connected, status, start, drive])

  // Optimistic local values so sliders track the pointer while the round trip
  // to the backend completes.
  const [pending, setPending] = useState({})
  const debounceRef = useRef({})

  useEffect(() => {
    // Once the backend confirms a value, drop the optimistic copy.
    if (!tuning?.values) return
    setPending((prev) => {
      const next = { ...prev }
      let changed = false
      Object.keys(prev).forEach((key) => {
        if (JSON.stringify(tuning.values[key]) === JSON.stringify(prev[key])) {
          delete next[key]
          changed = true
        }
      })
      return changed ? next : prev
    })
  }, [tuning])

  const values = useMemo(
    () => ({ ...(tuning?.values ?? {}), ...pending }),
    [tuning, pending],
  )

  const dirtySet = useMemo(
    () => new Set(tuning?.dirty ?? []),
    [tuning],
  )

  const onChange = useCallback(
    (name, value) => {
      setPending((prev) => ({ ...prev, [name]: value }))
      // Debounce: a slider drag would otherwise emit an update per pixel, and
      // each one is a WiFi write to the MCU.
      clearTimeout(debounceRef.current[name])
      debounceRef.current[name] = setTimeout(() => {
        updateTuning({ [name]: value })
      }, 120)
    },
    [updateTuning],
  )

  useEffect(
    () => () => {
      Object.values(debounceRef.current).forEach(clearTimeout)
    },
    [],
  )

  const bySection = useMemo(() => {
    if (!schema?.params) return []
    const groups = new Map()
    schema.params.forEach((param) => {
      if (!groups.has(param.section)) groups.set(param.section, [])
      groups.get(param.section).push(param)
    })
    return (schema.sections ?? [...groups.keys()]).map((section) => ({
      section,
      params: groups.get(section) ?? [],
    }))
  }, [schema])

  if (!schema || !tuning) {
    return (
      <div className="panel">
        <div className="empty">
          {connected
            ? 'Loading parameter schema from the backend…'
            : 'Not connected to the backend.'}
        </div>
      </div>
    )
  }

  const dirtyCount = dirtySet.size

  return (
    <>
      <div className="panel">
        <div className="panel-title">Live tuning</div>
        <div className="row">
          <button
            className="primary"
            onClick={saveTuning}
            disabled={!connected || dirtyCount === 0}
            title="Writes the current values to EEPROM (Arduino) and LittleFS (NodeMCU) so they survive a power cycle"
          >
            Save as default{dirtyCount > 0 ? ` (${dirtyCount})` : ''}
          </button>
          <button
            className="danger"
            onClick={() => {
              if (
                window.confirm(
                  'Reset every parameter to firmware defaults and clear the saved config on both MCUs?',
                )
              ) {
                resetTuning()
              }
            }}
            disabled={!connected}
          >
            Reset all to defaults
          </button>
          <div className="spacer" />
          {dirtyCount > 0 ? (
            <span className="chip warn">
              {dirtyCount} unsaved change{dirtyCount === 1 ? '' : 's'}
            </span>
          ) : (
            <span className="chip ok">all values saved</span>
          )}
          {!arduino?.connected && (
            <span className="chip err">Arduino offline</span>
          )}
          {!nodemcu?.connected && <span className="chip err">NodeMCU offline</span>}
        </div>

        <div className="param-help" style={{ marginTop: 9 }}>
          Changes apply immediately over WiFi — the firmware patches its runtime
          config struct and the next control-loop iteration uses the new value,
          with no reflash and no reboot (§11.3). Unsaved values revert to
          firmware defaults on the next power cycle; press{' '}
          <strong>Save as default</strong> to persist them to MCU flash (§11.4).
          A parameter whose device is offline is stored on the backend and pushed
          automatically as soon as that device reconnects.
        </div>

        {lastReply && Date.now() - lastReply.at < 5000 && (
          <div
            className={`notice ${lastReply.ok ? 'info' : 'warn'}`}
            style={{ marginTop: 12, marginBottom: 0 }}
          >
            {lastReply.msg}
          </div>
        )}
      </div>

      <div className="panel" style={{ border: '1px solid var(--accent-dim)' }}>
        <div className="panel-title" style={{ color: 'var(--accent)' }}>
          Odometry Direction & Calibration Assistant
        </div>
        <div className="param-help" style={{ marginBottom: 12 }}>
          Align and verify your robot's coordinate systems with ROS (Forward motion increases X; Counter-Clockwise rotation increases Yaw angle θ).
        </div>
        
        {/* Live Values Grid */}
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(180px, 1fr))', gap: 10, marginBottom: 14 }}>
          <div className="stat">
            <div className="stat-label">Left Wheel Ticks</div>
            <div className="stat-value">{status?.odom?.ticks_l ?? 0}</div>
          </div>
          <div className="stat">
            <div className="stat-label">Right Wheel Ticks</div>
            <div className="stat-value">{status?.odom?.ticks_r ?? 0}</div>
          </div>
          <div className="stat">
            <div className="stat-label">Yaw Angle (θ)</div>
            <div className="stat-value">
              {status?.odom?.theta_rad ? (status.odom.theta_rad * 180 / Math.PI).toFixed(1) : '0.0'}
              <span className="stat-unit">°</span>
            </div>
          </div>
        </div>

        {/* Diagnostic Actions */}
        <div className="row" style={{ gap: 10, marginBottom: 14 }}>
          <button 
            onClick={() => runTest('left')} 
            disabled={calState.active !== null || !connected}
            className={calState.active === 'left' ? 'primary' : ''}
          >
            {calState.active === 'left' ? 'Testing Left...' : 'Test Left Motor'}
          </button>
          <button 
            onClick={() => runTest('right')} 
            disabled={calState.active !== null || !connected}
            className={calState.active === 'right' ? 'primary' : ''}
          >
            {calState.active === 'right' ? 'Testing Right...' : 'Test Right Motor'}
          </button>
          <button 
            onClick={() => runTest('ccw')} 
            disabled={calState.active !== null || !connected}
            className={calState.active === 'ccw' ? 'primary' : ''}
          >
            {calState.active === 'ccw' ? 'Testing Spin...' : 'Test CCW Rotation'}
          </button>
        </div>

        {/* Live Diagnostics Log / Status */}
        {calState.result && (
          <div className={`notice ${calState.resultColor || 'info'}`} style={{ marginTop: 10, marginBottom: 10 }}>
            <strong>Calibration Status:</strong>
            <div>{calState.result}</div>
            {calState.active && (
              <div style={{ marginTop: 6, fontSize: '11px', opacity: 0.8 }}>
                Progress: {calState.elapsed}ms | Left Ticks Δ: {calState.ticksDelta.l} | Right Ticks Δ: {calState.ticksDelta.r} | Yaw Δ: {(calState.thetaDelta * 180 / Math.PI).toFixed(1)}°
              </div>
            )}
          </div>
        )}

        {/* Quick Help Reference */}
        <div style={{ marginTop: 12, borderTop: '1px solid var(--border)', paddingTop: 10 }}>
          <div className="sub-title" style={{ fontSize: '11px', color: 'var(--text-dim)' }}>Manual Alignment Checklist</div>
          <ul className="rules" style={{ fontSize: '12px', marginTop: 4, paddingLeft: 16 }}>
            <li><strong>Roll Forward manually:</strong> Check that ticks count UP (both Left and Right). If they count down, the encoder leads for that wheel are swapped.</li>
            <li><strong>Rotate Left manually:</strong> Check that Yaw angle integrates positively. If it goes negative, the relative coordinate system is inverted.</li>
            <li><strong>Motor direction:</strong> If wheels drive backwards when commanded forward, toggle the "Invert Left Motor" or "Invert Right Motor" parameters below to 1.</li>
          </ul>
        </div>
      </div>

      {bySection.map(({ section, params }) => (
        <div className="panel" key={section}>
          <div className="panel-title">{section}</div>
          {params.map((param) =>
            param.kind === 'angle_mask' ? (
              <AngleMaskParam
                key={param.name}
                value={values[param.name]}
                onChange={(value) => onChange(param.name, value)}
              />
            ) : (
              <Param
                key={param.name}
                param={param}
                value={values[param.name]}
                dirty={dirtySet.has(param.name)}
                onChange={(value) => onChange(param.name, value)}
              />
            ),
          )}
        </div>
      ))}

      <div className="panel">
        <div className="panel-title">What still needs a reflash</div>
        <div className="param-help">
          Per §11.5: pin assignments, wiring-dependent constants (which physical
          pin is which encoder channel), and changes to the WebSocket message
          schema itself. Everything on this page does not — that is the whole
          point of the runtime config struct.
        </div>
      </div>
    </>
  )
}
