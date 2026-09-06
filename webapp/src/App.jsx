/**
 * App shell: navigation, global connection state, and the always-visible
 * emergency stop.
 *
 * The e-stop lives in the top bar on every page, not just the Dashboard. If
 * something goes wrong while you're on the Tuning or Logs page, you should not
 * have to navigate anywhere to halt the robot.
 */

import { useState, useEffect } from 'react'
import { NavLink, Navigate, Route, Routes } from 'react-router-dom'
import { useBot } from './lib/store.jsx'
import Dashboard from './pages/Dashboard.jsx'
import LiveMap from './pages/LiveMap.jsx'
import Logs from './pages/Logs.jsx'
import FlashCenter from './pages/FlashCenter.jsx'
import Tuning from './pages/Tuning.jsx'
import AIAssistant from './pages/AIAssistant.jsx'
import HardwareGuide from './pages/HardwareGuide.jsx'
import Applications from './pages/Applications.jsx'

const PAGES = [
  { path: '/dashboard', label: 'Dashboard' },
  { path: '/map', label: 'Live Map' },
  { path: '/logs', label: 'Logs' },
  { path: '/flash', label: 'Flash Center' },
  { path: '/tuning', label: 'Tuning' },
  { path: '/ai', label: 'AI Assistant' },
  { path: '/guide', label: 'Hardware Guide' },
  { path: '/applications', label: 'Applications' },
]

export default function App() {
  const { connected, running, estopLatched, estop, arduino, nodemcu } = useBot()
  const [isFullscreen, setIsFullscreen] = useState(false)

  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(Boolean(document.fullscreenElement))
    }
    document.addEventListener('fullscreenchange', handleFullscreenChange)
    return () => document.removeEventListener('fullscreenchange', handleFullscreenChange)
  }, [])

  const toggleFullscreen = () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(() => {})
    } else {
      if (document.exitFullscreen) {
        document.exitFullscreen().catch(() => {})
      }
    }
  }

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand" style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <img
            src="/SLAMBOT.png"
            alt="SLAM Bot"
            style={{ width: '28px', height: '28px', borderRadius: '6px', filter: 'drop-shadow(0 0 6px rgba(0, 229, 255, 0.5))' }}
          />
          <div>SLAM<span>Bot</span></div>
        </div>

        <nav className="nav">
          {PAGES.map((page) => (
            <NavLink
              key={page.path}
              to={page.path}
              className={({ isActive }) => (isActive ? 'active' : '')}
            >
              {page.label}
            </NavLink>
          ))}
        </nav>

        <div className="topbar-right">
          <span
            className={`chip ${connected ? 'ok' : 'err'}`}
            title={
              connected
                ? 'Connected to the backend'
                : 'Backend unreachable — retrying every 1.5 s'
            }
          >
            <span className={`dot ${connected ? 'ok' : 'err'}`} />
            {connected ? 'Backend' : 'No backend'}
          </span>

          <span
            className={`chip ${arduino?.connected ? 'ok' : 'err'}`}
            title="Arduino Uno R4 WiFi — /ws/motion"
          >
            <span className={`dot ${arduino?.connected ? 'ok' : 'err'}`} />
            Arduino
          </span>

          <span
            className={`chip ${nodemcu?.connected ? 'ok' : 'err'}`}
            title="NodeMCU LIDAR relay — /ws/lidar"
          >
            <span className={`dot ${nodemcu?.connected ? 'ok' : 'err'}`} />
            LIDAR
          </span>

          <span
            className={`chip ${running ? 'ok' : estopLatched ? 'err' : 'warn'}`}
            title="Firmware-level run state (§11.2)"
          >
            {running ? 'RUNNING' : estopLatched ? 'E-STOPPED' : 'STOPPED'}
          </span>

          <button
            onClick={toggleFullscreen}
            style={{
              padding: '6px 12px',
              fontSize: '12px',
              fontWeight: '600',
              background: 'rgba(0, 229, 255, 0.12)',
              borderColor: 'var(--accent)',
              color: 'var(--accent)'
            }}
            title="Toggle Fullscreen Mode (F11)"
          >
            {isFullscreen ? 'Exit Full' : '⛶ Fullscreen'}
          </button>

          <button
            className="estop"
            onClick={estop}
            disabled={!connected}
            title="Hard stop: zeroes motors and latches the bot into Stopped"
          >
            E-STOP
          </button>
        </div>
      </header>

      <main className="content">
        {!connected && (
          <div className="notice err">
            <strong>Not connected to the backend.</strong>
            Start it with <span className="mono">uvicorn main:app --host 0.0.0.0
            --port 8000</span> from the <span className="mono">backend/</span>
            directory. Reconnecting automatically.
          </div>
        )}

        <Routes>
          <Route path="/" element={<Navigate to="/dashboard" replace />} />
          <Route path="/dashboard" element={<Dashboard />} />
          <Route path="/map" element={<LiveMap />} />
          <Route path="/logs" element={<Logs />} />
          <Route path="/flash" element={<FlashCenter />} />
          <Route path="/tuning" element={<Tuning />} />
          <Route path="/ai" element={<AIAssistant />} />
          <Route path="/guide" element={<HardwareGuide />} />
          <Route path="/applications" element={<Applications />} />
          <Route path="/guide" element={<HardwareGuide />} />
          <Route path="/applications" element={<Applications />} />
          <Route path="*" element={<Navigate to="/dashboard" replace />} />
        </Routes>
      </main>
    </div>
  )
}
