/**
 * Live backend connection + shared state for the whole app.
 *
 * One WebSocket to /ws/app carries every topic (§4). Every page reads from this
 * context rather than opening its own socket, so a six-page app still holds one
 * connection and the run-state can never diverge between tabs of the UI.
 *
 * Design notes:
 *  - Run-state (Start/Stop) is *never* stored as local UI state. It lives on the
 *    backend (§6.1: "state lives on the backend, not the browser tab") and this
 *    store only mirrors it. A page refresh re-reads it from the server.
 *  - High-rate topics (scan, status) are kept in refs and surfaced through a
 *    frame counter, so a 8 Hz scan does not trigger a React re-render tree for
 *    every revolution. Canvas pages read the ref directly in their draw loop.
 *  - Logs are capped client-side; the backend keeps the authoritative history.
 */

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react'

const MAX_CLIENT_LOGS = 1500

const BotContext = createContext(null)

function backendWsUrl(path) {
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${proto}//${window.location.host}${path}`
}

export function BotProvider({ children }) {
  const [connected, setConnected] = useState(false)
  const [status, setStatus] = useState(null)
  const [tuning, setTuning] = useState(null)
  const [schema, setSchema] = useState(null)
  const [logs, setLogs] = useState([])
  const [lastReply, setLastReply] = useState(null)
  const [mapVersion, setMapVersion] = useState(0)
  const [scanVersion, setScanVersion] = useState(0)

  // Bulk / high-rate payloads live in refs: canvases read them directly.
  const scanRef = useRef(null)
  const mapRef = useRef(null)
  const trailRef = useRef([])
  const planRef = useRef([])

  const wsRef = useRef(null)
  const reconnectRef = useRef(null)
  const mountedRef = useRef(true)

  const send = useCallback((payload) => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) return false
    ws.send(JSON.stringify(payload))
    return true
  }, [])

  useEffect(() => {
    mountedRef.current = true

    const connect = () => {
      if (!mountedRef.current) return
      const ws = new WebSocket(backendWsUrl('/ws/app'))
      wsRef.current = ws

      ws.onopen = () => {
        if (!mountedRef.current) return
        setConnected(true)
      }

      ws.onclose = () => {
        if (!mountedRef.current) return
        setConnected(false)
        // Steady 1.5 s retry. The robot's own firmware watchdog (§11.2) already
        // handles the safety side of a dropped link, so the UI only needs to
        // reconnect promptly, not aggressively.
        reconnectRef.current = setTimeout(connect, 1500)
      }

      ws.onerror = () => {
        // onclose always follows; nothing extra to do here.
      }

      ws.onmessage = (event) => {
        if (!mountedRef.current) return
        let frame
        try {
          frame = JSON.parse(event.data)
        } catch {
          return
        }
        const { topic, data } = frame
        switch (topic) {
          case 'status':
            setStatus(data)
            break
          case 'tuning':
            setTuning(data)
            break
          case 'schema':
            setSchema(data)
            break
          case 'scan':
            scanRef.current = data
            setScanVersion((v) => v + 1)
            break
          case 'map':
            mapRef.current = data
            setMapVersion((v) => v + 1)
            break
          case 'trail':
            trailRef.current = data.points ?? []
            break
          case 'plan':
            planRef.current = data.points ?? []
            setMapVersion((v) => v + 1)
            break
          case 'log':
            setLogs((prev) => {
              const next = [data, ...prev]
              return next.length > MAX_CLIENT_LOGS
                ? next.slice(0, MAX_CLIENT_LOGS)
                : next
            })
            break
          case 'log_history':
            // Backend sends oldest-first; the UI shows most recent at top (§6.3).
            setLogs((data ?? []).slice().reverse())
            break
          case 'reply':
            setLastReply({ ...data, at: Date.now() })
            break
          case 'drive_echo':
          case 'pong':
          case 'hello':
            break
          default:
            break
        }
      }
    }

    connect()

    return () => {
      mountedRef.current = false
      if (reconnectRef.current) clearTimeout(reconnectRef.current)
      const ws = wsRef.current
      if (ws) {
        ws.onclose = null
        ws.close()
      }
    }
  }, [])

  // -- commands ----------------------------------------------------------
  const start = useCallback(() => send({ type: 'control', action: 'start' }), [send])
  const stop = useCallback(() => send({ type: 'control', action: 'stop' }), [send])
  const estop = useCallback(() => send({ type: 'control', action: 'estop' }), [send])

  const drive = useCallback(
    (linearMmS, angularMdegS) =>
      send({
        type: 'drive',
        linear_mm_s: linearMmS,
        angular_mdeg_s: angularMdegS,
      }),
    [send],
  )

  const setMode = useCallback((mode) => send({ type: 'set_mode', mode }), [send])

  const updateTuning = useCallback(
    (values) => send({ type: 'tuning_update', values }),
    [send],
  )
  const saveTuning = useCallback(() => send({ type: 'tuning_save' }), [send])
  const resetTuning = useCallback(() => send({ type: 'tuning_reset' }), [send])

  const sendGoal = useCallback(
    (xM, yM, thetaRad = 0) =>
      send({ type: 'nav_goal', x_m: xM, y_m: yM, theta_rad: thetaRad }),
    [send],
  )

  const clearTrail = useCallback(() => send({ type: 'clear_trail' }), [send])
  const clearMap = useCallback(() => send({ type: 'clear_map' }), [send])
  const resetSession = useCallback(() => send({ type: 'reset_session' }), [send])

  const value = useMemo(
    () => ({
      connected,
      status,
      tuning,
      schema,
      logs,
      lastReply,
      scanRef,
      mapRef,
      trailRef,
      planRef,
      scanVersion,
      mapVersion,
      running: status?.running ?? false,
      estopLatched: status?.estop_latched ?? false,
      controlMode: status?.control_mode ?? 'manual',
      arduino: status?.devices?.arduino ?? null,
      nodemcu: status?.devices?.nodemcu ?? null,
      start,
      stop,
      estop,
      drive,
      setMode,
      updateTuning,
      saveTuning,
      resetTuning,
      sendGoal,
      clearTrail,
      clearMap,
      resetSession,
      send,
    }),
    [
      connected,
      status,
      tuning,
      schema,
      logs,
      lastReply,
      scanVersion,
      mapVersion,
      start,
      stop,
      estop,
      drive,
      setMode,
      updateTuning,
      saveTuning,
      resetTuning,
      sendGoal,
      clearTrail,
      clearMap,
      resetSession,
      send,
    ],
  )

  return <BotContext.Provider value={value}>{children}</BotContext.Provider>
}

export function useBot() {
  const context = useContext(BotContext)
  if (!context) throw new Error('useBot must be used inside <BotProvider>')
  return context
}

/** Fetch helper for the REST endpoints that aren't part of the WS stream. */
export async function api(path, options = {}) {
  const response = await fetch(`/api${path}`, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const text = await response.text()
  let payload = null
  if (text) {
    try {
      payload = JSON.parse(text)
    } catch {
      payload = { detail: text }
    }
  }
  if (!response.ok) {
    throw new Error(payload?.detail ?? `${response.status} ${response.statusText}`)
  }
  return payload
}
