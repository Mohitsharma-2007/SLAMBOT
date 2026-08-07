/**
 * Logs & Debug — §6.3
 *
 * Live-tailing stream, filterable by source and severity, most recent at top.
 *
 * This page is also the answer to the §3 hardware constraint: the NodeMCU's USB
 * serial console is unusable at runtime because the RPLIDAR occupies the
 * hardware UART, so its diagnostics are shipped over WiFi and land here.
 */

import { useMemo, useState } from 'react'
import { useBot } from '../lib/store.jsx'

const SOURCES = ['lidar', 'motion', 'slam', 'nav', 'ai', 'system']
const LEVELS = ['info', 'warn', 'error']

function formatTime(ms) {
  const date = new Date(ms)
  return date.toLocaleTimeString(undefined, { hour12: false }) +
    '.' +
    String(date.getMilliseconds()).padStart(3, '0')
}

export default function Logs() {
  const { logs, status } = useBot()

  const [activeSources, setActiveSources] = useState(() => new Set(SOURCES))
  const [activeLevels, setActiveLevels] = useState(() => new Set(LEVELS))
  const [query, setQuery] = useState('')
  const [paused, setPaused] = useState(false)
  const [frozen, setFrozen] = useState([])

  const toggle = (set, value, setter) => {
    const next = new Set(set)
    if (next.has(value)) next.delete(value)
    else next.add(value)
    setter(next)
  }

  const source = paused ? frozen : logs

  const filtered = useMemo(() => {
    const needle = query.trim().toLowerCase()
    return source.filter((entry) => {
      if (!activeSources.has(entry.source)) return false
      if (!activeLevels.has(entry.level)) return false
      if (needle && !entry.msg.toLowerCase().includes(needle)) return false
      return true
    })
  }, [source, activeSources, activeLevels, query])

  const counts = useMemo(() => {
    const out = { info: 0, warn: 0, error: 0 }
    logs.forEach((entry) => {
      out[entry.level] = (out[entry.level] ?? 0) + 1
    })
    return out
  }, [logs])

  const copyVisible = () => {
    const text = filtered
      .slice()
      .reverse()
      .map(
        (entry) =>
          `${formatTime(entry.t_ms)} [${entry.source}/${entry.level}] ${entry.msg}`,
      )
      .join('\n')
    navigator.clipboard?.writeText(text)
  }

  return (
    <>
      <div className="panel">
        <div className="panel-title">
          Filters
          <span className="small faint">
            {filtered.length} of {logs.length} shown
          </span>
        </div>

        <div className="row" style={{ marginBottom: 10 }}>
          <span className="small dim" style={{ minWidth: 56 }}>
            Source
          </span>
          {SOURCES.map((name) => (
            <button
              key={name}
              className={`sm toggle ${activeSources.has(name) ? 'on' : ''}`}
              onClick={() => toggle(activeSources, name, setActiveSources)}
            >
              {name}
            </button>
          ))}
          <button
            className="sm"
            onClick={() =>
              setActiveSources(
                activeSources.size === SOURCES.length ? new Set() : new Set(SOURCES),
              )
            }
          >
            {activeSources.size === SOURCES.length ? 'none' : 'all'}
          </button>
        </div>

        <div className="row" style={{ marginBottom: 10 }}>
          <span className="small dim" style={{ minWidth: 56 }}>
            Severity
          </span>
          {LEVELS.map((level) => (
            <button
              key={level}
              className={`sm toggle ${activeLevels.has(level) ? 'on' : ''}`}
              onClick={() => toggle(activeLevels, level, setActiveLevels)}
            >
              {level} ({counts[level] ?? 0})
            </button>
          ))}
        </div>

        <div className="row">
          <input
            type="text"
            placeholder="Search messages…"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            style={{ flex: 1, minWidth: 180 }}
          />
          <button
            className={`sm toggle ${paused ? 'on' : ''}`}
            onClick={() => {
              if (!paused) setFrozen(logs)
              setPaused(!paused)
            }}
            title="Freeze the view so a fast stream doesn't scroll away what you're reading"
          >
            {paused ? '▶ Resume' : '⏸ Pause'}
          </button>
          <button className="sm" onClick={copyVisible}>
            Copy visible
          </button>
        </div>
      </div>

      <div className="panel">
        <div className="panel-title">
          Live log stream
          <span className="small faint">
            newest first
            {paused ? ' · PAUSED' : ''}
            {status?.devices?.nodemcu?.connected
              ? ''
              : ' · NodeMCU offline, no lidar logs'}
          </span>
        </div>

        {filtered.length === 0 ? (
          <div className="empty">
            {logs.length === 0
              ? 'No log entries yet. The backend logs its own startup, and both MCUs log over WiFi once connected.'
              : 'No entries match the current filters.'}
          </div>
        ) : (
          <div className="log-list">
            {filtered.map((entry) => (
              <div
                key={`${entry.seq}-${entry.t_ms}`}
                className={`log-line ${entry.level}`}
              >
                <span className="log-time">{formatTime(entry.t_ms)}</span>
                <span className="log-source">{entry.source}</span>
                <span className={`log-level ${entry.level}`}>{entry.level}</span>
                <span className="log-msg">{entry.msg}</span>
              </div>
            ))}
          </div>
        )}
      </div>
    </>
  )
}
