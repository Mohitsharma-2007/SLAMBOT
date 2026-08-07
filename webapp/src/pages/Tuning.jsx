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
  } = useBot()

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
