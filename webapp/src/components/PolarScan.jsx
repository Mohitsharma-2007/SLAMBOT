/**
 * Real-time polar plot of the current LIDAR revolution — §6.2's "shows distance
 * and angle" requirement.
 *
 * Renders from the scan ref in an animation frame loop rather than re-rendering
 * React on every revolution: at 5.5 Hz with ~400 points, going through the
 * virtual DOM would be wasteful and janky.
 *
 * Angle convention: 0° is straight ahead (up on screen) and angles increase
 * counter-clockwise, matching the ROS REP-103 right-hand convention the rest of
 * the stack uses. The RPLIDAR reports clockwise from its own zero mark, so the
 * sign is flipped here in one place rather than scattered through the app.
 */

import { useEffect, useRef, useState } from 'react'
import { useBot } from '../lib/store.jsx'

const RING_COLOR = '#243040'
const RING_LABEL = '#5c6b7d'

export default function PolarScan({ height = 420 }) {
  const { scanRef, tuning } = useBot()
  const canvasRef = useRef(null)
  const wrapRef = useRef(null)
  const [maxRangeMm, setMaxRangeMm] = useState(6000)
  const [autoRange, setAutoRange] = useState(true)
  const [showQuality, setShowQuality] = useState(false)
  const [stats, setStats] = useState({ n: 0, nearest: null, hz: null })

  const configuredMax = tuning?.values?.lidar_max_range_mm ?? 6000
  const collisionMm = tuning?.values?.collision_stop_distance_mm ?? 100

  useEffect(() => {
    if (autoRange) setMaxRangeMm(configuredMax)
  }, [autoRange, configuredMax])

  useEffect(() => {
    let frame
    const canvas = canvasRef.current
    if (!canvas) return

    const draw = () => {
      frame = requestAnimationFrame(draw)
      const scan = scanRef.current
      const wrap = wrapRef.current
      if (!wrap) return

      // Handle DPI and container resizes without a ResizeObserver dependency.
      const dpr = window.devicePixelRatio || 1
      const cssWidth = wrap.clientWidth
      const cssHeight = height
      if (
        canvas.width !== Math.round(cssWidth * dpr) ||
        canvas.height !== Math.round(cssHeight * dpr)
      ) {
        canvas.width = Math.round(cssWidth * dpr)
        canvas.height = Math.round(cssHeight * dpr)
        canvas.style.height = `${cssHeight}px`
      }

      const ctx = canvas.getContext('2d')
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.clearRect(0, 0, cssWidth, cssHeight)

      const cx = cssWidth / 2
      const cy = cssHeight / 2
      const radius = Math.min(cx, cy) - 24
      const scale = radius / maxRangeMm

      // --- range rings ---------------------------------------------------
      const ringStep = maxRangeMm <= 1500 ? 250 : maxRangeMm <= 3000 ? 500 : 1000
      ctx.font = '10px ui-monospace, monospace'
      ctx.textAlign = 'left'
      for (let r = ringStep; r <= maxRangeMm; r += ringStep) {
        ctx.beginPath()
        ctx.arc(cx, cy, r * scale, 0, Math.PI * 2)
        ctx.strokeStyle = RING_COLOR
        ctx.lineWidth = 1
        ctx.stroke()
        ctx.fillStyle = RING_LABEL
        ctx.fillText(
          r >= 1000 ? `${(r / 1000).toFixed(r % 1000 ? 1 : 0)} m` : `${r} mm`,
          cx + 3,
          cy - r * scale - 3,
        )
      }

      // --- collision-stop ring -------------------------------------------
      if (collisionMm * scale > 3) {
        ctx.beginPath()
        ctx.arc(cx, cy, collisionMm * scale, 0, Math.PI * 2)
        ctx.strokeStyle = '#f85149'
        ctx.setLineDash([3, 3])
        ctx.lineWidth = 1
        ctx.stroke()
        ctx.setLineDash([])
      }

      // --- cardinal spokes ------------------------------------------------
      ctx.strokeStyle = RING_COLOR
      ctx.textAlign = 'center'
      for (let deg = 0; deg < 360; deg += 45) {
        const rad = (deg - 90) * (Math.PI / 180)
        ctx.beginPath()
        ctx.moveTo(cx, cy)
        ctx.lineTo(cx + Math.cos(rad) * radius, cy + Math.sin(rad) * radius)
        ctx.stroke()
        ctx.fillStyle = RING_LABEL
        ctx.fillText(
          `${deg}°`,
          cx + Math.cos(rad) * (radius + 13),
          cy + Math.sin(rad) * (radius + 13) + 3,
        )
      }

      // --- robot marker + heading ----------------------------------------
      ctx.beginPath()
      ctx.arc(cx, cy, 5, 0, Math.PI * 2)
      ctx.fillStyle = '#4f9cf9'
      ctx.fill()
      ctx.beginPath()
      ctx.moveTo(cx, cy)
      ctx.lineTo(cx, cy - 18)
      ctx.strokeStyle = '#4f9cf9'
      ctx.lineWidth = 2
      ctx.stroke()

      if (!scan || !scan.angles_deg?.length) {
        ctx.fillStyle = '#6b7684'
        ctx.font = '13px system-ui, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('waiting for LIDAR data…', cx, cy + radius + 4)
        return
      }

      // --- scan points -----------------------------------------------------
      let nearest = Infinity
      const { angles_deg: angles, dists_mm: dists, quality } = scan
      for (let i = 0; i < angles.length; i += 1) {
        const distance = dists[i]
        if (distance > maxRangeMm) continue
        if (distance < nearest) nearest = distance

        // Negate for counter-clockwise, -90 so 0° points up.
        const rad = (-angles[i] - 90) * (Math.PI / 180)
        const x = cx + Math.cos(rad) * distance * scale
        const y = cy + Math.sin(rad) * distance * scale

        if (showQuality && quality?.length) {
          // Quality 0..63 -> dim to bright, so weak returns are visibly weak.
          const q = Math.max(0, Math.min(63, quality[i])) / 63
          ctx.fillStyle = `rgba(79, 156, 249, ${0.25 + 0.75 * q})`
        } else if (distance <= collisionMm) {
          ctx.fillStyle = '#f85149'
        } else if (distance < collisionMm * 3) {
          ctx.fillStyle = '#d29922'
        } else {
          ctx.fillStyle = '#3fb950'
        }

        ctx.beginPath()
        ctx.arc(x, y, 1.9, 0, Math.PI * 2)
        ctx.fill()
      }

      const hz = scan.rev_ms > 0 ? 1000 / scan.rev_ms : null
      // Only re-render React when a displayed number actually changed.
      setStats((prev) => {
        const next = {
          n: angles.length,
          nearest: Number.isFinite(nearest) ? Math.round(nearest) : null,
          hz: hz ? Math.round(hz * 10) / 10 : null,
        }
        return prev.n === next.n &&
          prev.nearest === next.nearest &&
          prev.hz === next.hz
          ? prev
          : next
      })
    }

    frame = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(frame)
  }, [scanRef, height, maxRangeMm, collisionMm, showQuality])

  return (
    <div>
      <div className="row" style={{ marginBottom: 10 }}>
        <label className="small dim">
          <input
            type="checkbox"
            checked={autoRange}
            onChange={(e) => setAutoRange(e.target.checked)}
          />{' '}
          Match tuning range
        </label>
        <label className="small dim">
          <input
            type="checkbox"
            checked={showQuality}
            onChange={(e) => setShowQuality(e.target.checked)}
          />{' '}
          Shade by signal quality
        </label>
        <div className="spacer" />
        <label className="small dim">Zoom</label>
        <select
          value={maxRangeMm}
          onChange={(e) => {
            setAutoRange(false)
            setMaxRangeMm(Number(e.target.value))
          }}
        >
          <option value={500}>0.5 m</option>
          <option value={1000}>1 m</option>
          <option value={2000}>2 m</option>
          <option value={4000}>4 m</option>
          <option value={6000}>6 m</option>
          <option value={12000}>12 m</option>
        </select>
      </div>

      <div className="canvas-wrap" ref={wrapRef}>
        <canvas ref={canvasRef} />
        <div className="canvas-overlay">
          {stats.n} points
          {stats.hz != null ? ` · ${stats.hz} Hz` : ''}
          {stats.nearest != null ? ` · nearest ${stats.nearest} mm` : ''}
        </div>
      </div>
    </div>
  )
}
