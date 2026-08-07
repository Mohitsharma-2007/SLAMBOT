/**
 * Flash Center — §6.4
 *
 * Two genuinely different mechanisms, deliberately presented as such:
 *
 *  - Arduino Uno R4: compiled and uploaded by the *backend* via arduino-cli,
 *    with console output streamed live over /ws/flash. The board must be plugged
 *    into the machine running the backend, which the UI states outright.
 *
 *  - NodeMCU: flashed entirely *in the browser* over WebSerial with esptool-js.
 *    No backend involvement, so the board plugs into whatever device you're
 *    browsing from.
 */

import { useCallback, useEffect, useRef, useState } from 'react'
import { api, useBot } from '../lib/store.jsx'
import {
  connectEsp,
  fetchCompiledBinary,
  finishAndReset,
  flashBinary,
  webSerialSupported,
} from '../lib/esptool.js'

function Console({ lines }) {
  const ref = useRef(null)
  const pinnedRef = useRef(true)

  // Follow the tail, but stop fighting the user if they scroll up to read.
  useEffect(() => {
    const element = ref.current
    if (!element) return
    if (pinnedRef.current) element.scrollTop = element.scrollHeight
  }, [lines])

  return (
    <div
      className="console"
      ref={ref}
      onScroll={(e) => {
        const el = e.currentTarget
        pinnedRef.current = el.scrollHeight - el.scrollTop - el.clientHeight < 30
      }}
    >
      {lines.length === 0 ? (
        <span className="faint">Console output appears here.</span>
      ) : (
        lines.map((line, index) => (
          <div key={index} className={line.stream}>
            {line.text}
          </div>
        ))
      )}
    </div>
  )
}

export default function FlashCenter() {
  const { running, stop, connected } = useBot()

  const [probe, setProbe] = useState(null)
  const [probeError, setProbeError] = useState(null)
  const [ports, setPorts] = useState([])
  const [selectedPort, setSelectedPort] = useState('')

  const [arduinoLines, setArduinoLines] = useState([])
  const [arduinoBusy, setArduinoBusy] = useState(false)
  const [arduinoResult, setArduinoResult] = useState(null)

  const [espLines, setEspLines] = useState([])
  const [espBusy, setEspBusy] = useState(false)
  const [espProgress, setEspProgress] = useState(null)
  const [espFile, setEspFile] = useState(null)
  const [espError, setEspError] = useState(null)

  const wsRef = useRef(null)

  const appendArduino = useCallback((stream, text) => {
    setArduinoLines((prev) => [...prev.slice(-2000), { stream, text }])
  }, [])

  const appendEsp = useCallback((text) => {
    setEspLines((prev) => [...prev.slice(-2000), { stream: 'stdout', text }])
  }, [])

  // --- probe the backend's toolchain ------------------------------------
  const refreshProbe = useCallback(async () => {
    try {
      const info = await api('/flash/probe')
      setProbe(info)
      setProbeError(null)
      setPorts(info.ports ?? [])
      if (!selectedPort && info.ports?.length) {
        // Prefer a port arduino-cli identified as an UNO R4.
        const match = info.ports.find((p) =>
          /uno\s*r4|renesas/i.test(`${p.board} ${p.fqbn}`),
        )
        setSelectedPort((match ?? info.ports[0]).address)
      }
    } catch (error) {
      setProbeError(error.message)
    }
  }, [selectedPort])

  useEffect(() => {
    refreshProbe()
  }, [])

  // --- flash WebSocket ---------------------------------------------------
  useEffect(() => {
    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const ws = new WebSocket(`${proto}//${window.location.host}/ws/flash`)
    wsRef.current = ws

    ws.onmessage = (event) => {
      let msg
      try {
        msg = JSON.parse(event.data)
      } catch {
        return
      }
      switch (msg.type) {
        case 'ready':
          if (msg.probe) {
            setProbe(msg.probe)
            setPorts(msg.probe.ports ?? [])
          }
          break
        case 'output':
          appendArduino(msg.stream === 'stderr' ? 'stderr' : 'stdout', msg.line)
          break
        case 'started':
          setArduinoBusy(true)
          setArduinoResult(null)
          appendArduino('meta', `--- ${msg.upload ? 'compile + upload' : 'compile'} ${msg.target} ---`)
          break
        case 'finished':
          setArduinoBusy(false)
          setArduinoResult(msg)
          appendArduino('meta', `--- ${msg.ok ? 'SUCCESS' : 'FAILED'}: ${msg.detail} ---`)
          break
        case 'error':
          setArduinoBusy(false)
          appendArduino('stderr', msg.msg)
          break
        default:
          break
      }
    }

    ws.onclose = () => {
      setArduinoBusy(false)
    }

    return () => {
      ws.onclose = null
      ws.close()
    }
  }, [appendArduino])

  const flashArduino = useCallback(
    (upload) => {
      const ws = wsRef.current
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        appendArduino('stderr', 'flash socket is not connected')
        return
      }
      setArduinoLines([])
      ws.send(
        JSON.stringify({
          type: 'flash',
          target: 'arduino',
          port: selectedPort || null,
          upload,
        }),
      )
    },
    [selectedPort, appendArduino],
  )

  const compileNodemcu = useCallback(() => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) return
    setArduinoLines([])
    // upload:false — the backend compiles, the browser flashes (§6.4).
    ws.send(JSON.stringify({ type: 'flash', target: 'nodemcu', upload: false }))
  }, [])

  // --- browser-side ESP8266 flashing ------------------------------------
  const flashEsp = useCallback(
    async (source) => {
      setEspError(null)
      setEspLines([])
      setEspProgress(null)
      setEspBusy(true)

      let loader
      let transport
      try {
        let binary
        if (source === 'backend') {
          appendEsp('Fetching the backend-compiled .bin…')
          binary = await fetchCompiledBinary('nodemcu')
          appendEsp(`Got ${binary.byteLength} bytes.`)
        } else {
          if (!espFile) throw new Error('Choose a .bin file first.')
          binary = await espFile.arrayBuffer()
          appendEsp(`Using ${espFile.name} (${binary.byteLength} bytes).`)
        }

        appendEsp('Requesting serial port — pick the NodeMCU in the dialog…')
        const connection = await connectEsp(appendEsp)
        loader = connection.loader
        transport = connection.transport

        await flashBinary(loader, binary, setEspProgress, appendEsp)
        await finishAndReset(loader, transport, appendEsp)
        setEspProgress(100)
      } catch (error) {
        setEspError(error.message)
        appendEsp(`ERROR: ${error.message}`)
        if (transport) {
          try {
            await transport.disconnect()
          } catch {
            // Already closed.
          }
        }
      } finally {
        setEspBusy(false)
      }
    },
    [espFile, appendEsp],
  )

  const cliMissing = probe && !probe.cli_available
  const serialOk = webSerialSupported()

  return (
    <>
      {running && (
        <div className="notice err">
          <strong>The bot is running.</strong>
          Flashing resets the board and holds its serial port, which would leave
          the motors unattended mid-drive. Press{' '}
          <button className="sm danger" onClick={stop} style={{ marginLeft: 6 }}>
            Stop
          </button>{' '}
          first — the backend will refuse the flash otherwise.
        </div>
      )}

      <div className="grid grid-2">
        {/* ---------------- Arduino ---------------- */}
        <div className="panel">
          <div className="panel-title">
            Arduino Uno R4 WiFi
            <span className="small faint">via backend arduino-cli</span>
          </div>

          <div className="notice warn">
            <strong>The board must be plugged into the backend host.</strong>
            This flash runs on the machine serving this page
            {probe?.cli_path ? '' : ''}, not on your browsing device. If they are
            different computers, plug the Arduino into the backend machine (or use
            the Arduino IDE locally).
          </div>

          {probeError && (
            <div className="notice err">
              Could not probe the backend toolchain: {probeError}
            </div>
          )}

          {cliMissing && (
            <div className="notice err">
              <strong>arduino-cli not found on the backend.</strong>
              Install it and make sure it is on the PATH, or set{' '}
              <span className="mono">SLAM_ARDUINO_CLI</span> to its full path.
              Then install the core:{' '}
              <span className="mono">
                arduino-cli core install arduino:renesas_uno
              </span>
            </div>
          )}

          {probe && !probe.arduino_sketch_exists && (
            <div className="notice err">
              Sketch not found at{' '}
              <span className="mono">{probe.arduino_sketch}</span>
            </div>
          )}

          <div className="row" style={{ marginBottom: 10 }}>
            <label className="small dim">Serial port</label>
            <select
              value={selectedPort}
              onChange={(e) => setSelectedPort(e.target.value)}
              style={{ flex: 1, minWidth: 150 }}
            >
              <option value="">— select —</option>
              {ports.map((port) => (
                <option key={port.address} value={port.address}>
                  {port.address}
                  {port.board ? ` — ${port.board}` : ''}
                </option>
              ))}
            </select>
            <button className="sm" onClick={refreshProbe}>
              Rescan
            </button>
          </div>

          <div className="row" style={{ marginBottom: 10 }}>
            <button
              onClick={() => flashArduino(false)}
              disabled={arduinoBusy || cliMissing || running}
            >
              Compile only
            </button>
            <button
              className="primary"
              onClick={() => flashArduino(true)}
              disabled={arduinoBusy || cliMissing || !selectedPort || running}
            >
              {arduinoBusy ? 'Flashing…' : 'Compile + Upload'}
            </button>
            {arduinoResult && (
              <span className={`chip ${arduinoResult.ok ? 'ok' : 'err'}`}>
                {arduinoResult.ok ? 'success' : 'failed'}
              </span>
            )}
          </div>

          <div className="small faint mono" style={{ marginBottom: 8 }}>
            {probe?.cli_version ?? 'arduino-cli version unknown'} ·{' '}
            {probe?.arduino_fqbn}
          </div>

          <Console lines={arduinoLines} />
        </div>

        {/* ---------------- NodeMCU ---------------- */}
        <div className="panel">
          <div className="panel-title">
            NodeMCU (ESP8266)
            <span className="small faint">via browser WebSerial</span>
          </div>

          <div className="notice info">
            <strong>This one runs entirely in your browser.</strong>
            Plug the NodeMCU into <em>this</em> device, click Flash, and pick the
            port in the dialog. No backend involved — so it works even if the
            backend is on another machine.
          </div>

          {!serialOk && (
            <div className="notice err">
              <strong>WebSerial is unavailable in this browser.</strong>
              Use Chrome, Edge or Opera on desktop. Firefox and Safari do not
              implement WebSerial, and neither does any iOS browser. You can
              still flash with <span className="mono">esptool.py</span> or the
              Arduino IDE.
            </div>
          )}

          <div className="row" style={{ marginBottom: 10 }}>
            <button onClick={compileNodemcu} disabled={arduinoBusy || cliMissing}>
              Compile on backend
            </button>
            <button
              className="primary"
              onClick={() => flashEsp('backend')}
              disabled={!serialOk || espBusy}
              title="Fetch the backend-compiled .bin, then flash it over WebSerial"
            >
              {espBusy ? 'Flashing…' : 'Flash compiled .bin'}
            </button>
          </div>

          <div className="row" style={{ marginBottom: 10 }}>
            <label className="small dim">or choose a .bin</label>
            <input
              type="file"
              accept=".bin"
              onChange={(e) => setEspFile(e.target.files?.[0] ?? null)}
              className="small"
            />
            <button
              className="sm"
              onClick={() => flashEsp('file')}
              disabled={!serialOk || espBusy || !espFile}
            >
              Flash this file
            </button>
          </div>

          {espProgress !== null && (
            <div style={{ marginBottom: 10 }}>
              <div className="row small dim" style={{ marginBottom: 4 }}>
                <span>Writing flash</span>
                <div className="spacer" />
                <span className="mono">{espProgress}%</span>
              </div>
              <div
                style={{
                  height: 6,
                  background: 'var(--bg-input)',
                  borderRadius: 3,
                  overflow: 'hidden',
                }}
              >
                <div
                  style={{
                    height: '100%',
                    width: `${espProgress}%`,
                    background: espProgress === 100 ? 'var(--ok)' : 'var(--accent)',
                    transition: 'width 0.2s',
                  }}
                />
              </div>
            </div>
          )}

          {espError && <div className="notice err">{espError}</div>}

          <Console lines={espLines} />
        </div>
      </div>

      <div className="panel">
        <div className="panel-title">Before you rely on this page</div>
        <div className="param-help">
          §12 step 2: flash both boards once over USB the traditional way (Arduino
          IDE) to confirm the firmware works, <strong>before</strong> depending on
          the Flash Center. And per §12 step 9, once things are running you should
          rarely need this page at all — nearly every adjustment belongs in the
          Tuning Panel, which applies live with no reflash.
        </div>
        <div className="param-help" style={{ marginTop: 8 }}>
          Both sketches need a <span className="mono">secrets.h</span> next to
          them with your WiFi credentials and this backend's LAN IP. The backend
          refuses to compile without one rather than producing a binary that
          silently cannot connect.
        </div>
      </div>
    </>
  )
}
