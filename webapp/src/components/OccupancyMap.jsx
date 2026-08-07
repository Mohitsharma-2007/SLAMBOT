/**
 * Occupancy grid renderer — §6.2.
 *
 * Draws /map from slam_toolbox, plus the robot pose, its trail, and Nav2's
 * current plan. Click to send a navigation goal.
 *
 * §6.2 is explicit that rviz2 must not be embedded in the browser; this is the
 * lightweight canvas renderer it asks for instead.
 *
 * Implementation notes:
 *  - The grid arrives run-length encoded (unexplored space dominates and
 *    compresses to nearly nothing). It is decoded into an ImageData once per new
 *    map, cached on an offscreen canvas, and then only blitted on pan/zoom — so
 *    dragging the view does not re-decode a 16-million-cell grid.
 *  - Map rows are drawn bottom-up: ROS occupancy grids have their origin at the
 *    bottom-left with +y up, while canvas y grows downward.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useBot } from '../lib/store.jsx'

function decodeToImageData(map) {
  const { width, height, rle } = map
  const image = new ImageData(width, height)
  const pixels = image.data

  let index = 0
  for (let i = 0; i < rle.length - 1; i += 2) {
    const value = rle[i]
    const count = rle[i + 1]

    let r
    let g
    let b
    if (value < 0) {
      // Unknown — mid grey, clearly distinct from both free and occupied.
      r = 32
      g = 38
      b = 46
    } else if (value < 25) {
      // Free space.
      r = 222
      g = 228
      b = 234
    } else if (value < 65) {
      // Uncertain: interpolate so partial evidence looks partial.
      const t = (value - 25) / 40
      r = Math.round(222 - t * 130)
      g = Math.round(228 - t * 140)
      b = Math.round(234 - t * 140)
    } else {
      // Occupied.
      r = 20
      g = 22
      b = 28
    }

    for (let n = 0; n < count && index < width * height; n += 1) {
      const offset = index * 4
      pixels[offset] = r
      pixels[offset + 1] = g
      pixels[offset + 2] = b
      pixels[offset + 3] = 255
      index += 1
    }
  }
  return image
}

export default function OccupancyMap({ height = 520 }) {
  const { mapRef, trailRef, planRef, status, mapVersion, sendGoal, controlMode } =
    useBot()

  const canvasRef = useRef(null)
  const wrapRef = useRef(null)
  const cacheRef = useRef({ stamp: -1, canvas: null, width: 0, height: 0 })

  const [view, setView] = useState({ zoom: 1, panX: 0, panY: 0 })
  const [autoFit, setAutoFit] = useState(true)
  const [showTrail, setShowTrail] = useState(true)
  const [goalHint, setGoalHint] = useState(null)
  const dragRef = useRef(null)

  /** Rebuild the offscreen bitmap only when a genuinely new map arrives. */
  const ensureCache = useCallback(() => {
    const map = mapRef.current
    if (!map || !map.width || !map.height) return null
    const cache = cacheRef.current
    if (cache.stamp === map.stamp_ms && cache.canvas) return cache

    const offscreen = document.createElement('canvas')
    offscreen.width = map.width
    offscreen.height = map.height
    offscreen.getContext('2d').putImageData(decodeToImageData(map), 0, 0)

    cacheRef.current = {
      stamp: map.stamp_ms,
      canvas: offscreen,
      width: map.width,
      height: map.height,
    }
    return cacheRef.current
  }, [mapRef])

  // World (metres) <-> screen (px) transform, recomputed per frame.
  const transformRef = useRef(null)

  useEffect(() => {
    let frame
    const draw = () => {
      frame = requestAnimationFrame(draw)
      const canvas = canvasRef.current
      const wrap = wrapRef.current
      if (!canvas || !wrap) return

      const dpr = window.devicePixelRatio || 1
      const cssWidth = wrap.clientWidth
      if (
        canvas.width !== Math.round(cssWidth * dpr) ||
        canvas.height !== Math.round(height * dpr)
      ) {
        canvas.width = Math.round(cssWidth * dpr)
        canvas.height = Math.round(height * dpr)
        canvas.style.height = `${height}px`
      }

      const ctx = canvas.getContext('2d')
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.fillStyle = '#0b0f14'
      ctx.fillRect(0, 0, cssWidth, height)

      const map = mapRef.current
      const cache = ensureCache()

      if (!map || !cache) {
        ctx.fillStyle = '#6b7684'
        ctx.font = '13px system-ui, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText(
          'no map yet — start slam_toolbox and drive the bot to build one',
          cssWidth / 2,
          height / 2,
        )
        transformRef.current = null
        return
      }

      const { resolution, origin_x: originX, origin_y: originY } = map
      const mapWidthM = map.width * resolution
      const mapHeightM = map.height * resolution

      // Fit-to-view scale, then apply the user's zoom/pan on top.
      const fitScale = Math.min(cssWidth / mapWidthM, height / mapHeightM) * 0.94
      const scale = fitScale * view.zoom
      const offsetX = (cssWidth - mapWidthM * scale) / 2 + view.panX
      const offsetY = (height - mapHeightM * scale) / 2 + view.panY

      const worldToScreen = (xM, yM) => [
        offsetX + (xM - originX) * scale,
        // Flip y: ROS grids have +y up, canvas has +y down.
        offsetY + (mapHeightM - (yM - originY)) * scale,
      ]
      const screenToWorld = (px, py) => [
        (px - offsetX) / scale + originX,
        mapHeightM - (py - offsetY) / scale + originY,
      ]
      transformRef.current = { worldToScreen, screenToWorld }

      // --- grid bitmap ----------------------------------------------------
      ctx.imageSmoothingEnabled = scale < 6
      ctx.drawImage(
        cache.canvas,
        offsetX,
        offsetY,
        mapWidthM * scale,
        mapHeightM * scale,
      )

      ctx.strokeStyle = '#2a3441'
      ctx.lineWidth = 1
      ctx.strokeRect(offsetX, offsetY, mapWidthM * scale, mapHeightM * scale)

      // --- pose trail -----------------------------------------------------
      const trail = trailRef.current
      if (showTrail && trail?.length > 1) {
        ctx.beginPath()
        trail.forEach(([xMm, yMm], i) => {
          const [px, py] = worldToScreen(xMm / 1000, yMm / 1000)
          if (i === 0) ctx.moveTo(px, py)
          else ctx.lineTo(px, py)
        })
        ctx.strokeStyle = 'rgba(79, 156, 249, 0.55)'
        ctx.lineWidth = 1.5
        ctx.stroke()
      }

      // --- Nav2 plan ------------------------------------------------------
      const plan = planRef.current
      if (plan?.length > 1) {
        ctx.beginPath()
        plan.forEach(([xM, yM], i) => {
          const [px, py] = worldToScreen(xM, yM)
          if (i === 0) ctx.moveTo(px, py)
          else ctx.lineTo(px, py)
        })
        ctx.strokeStyle = '#d29922'
        ctx.lineWidth = 2
        ctx.setLineDash([5, 4])
        ctx.stroke()
        ctx.setLineDash([])
      }

      // --- robot ----------------------------------------------------------
      const odom = status?.odom
      if (odom) {
        const [px, py] = worldToScreen(odom.x_mm / 1000, odom.y_mm / 1000)
        const theta = odom.theta_rad ?? 0
        const bodyRadius = Math.max(5, 0.1 * scale)

        ctx.beginPath()
        ctx.arc(px, py, bodyRadius, 0, Math.PI * 2)
        ctx.fillStyle = odom.collision
          ? 'rgba(248, 81, 73, 0.85)'
          : 'rgba(79, 156, 249, 0.85)'
        ctx.fill()
        ctx.strokeStyle = '#fff'
        ctx.lineWidth = 1.5
        ctx.stroke()

        // Heading indicator. Screen y is flipped, hence -sin.
        ctx.beginPath()
        ctx.moveTo(px, py)
        ctx.lineTo(
          px + Math.cos(theta) * bodyRadius * 2.1,
          py - Math.sin(theta) * bodyRadius * 2.1,
        )
        ctx.strokeStyle = '#fff'
        ctx.lineWidth = 2
        ctx.stroke()
      }

      // --- pending goal marker -------------------------------------------
      if (goalHint) {
        const [px, py] = worldToScreen(goalHint.x, goalHint.y)
        ctx.beginPath()
        ctx.arc(px, py, 7, 0, Math.PI * 2)
        ctx.strokeStyle = '#3fb950'
        ctx.lineWidth = 2
        ctx.stroke()
        ctx.beginPath()
        ctx.moveTo(px - 11, py)
        ctx.lineTo(px + 11, py)
        ctx.moveTo(px, py - 11)
        ctx.lineTo(px, py + 11)
        ctx.stroke()
      }
    }

    frame = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(frame)
  }, [
    mapRef,
    trailRef,
    planRef,
    status,
    height,
    view,
    showTrail,
    goalHint,
    ensureCache,
  ])

  // Auto-fit resets the view whenever a map with new dimensions arrives.
  const lastDims = useRef('')
  useEffect(() => {
    const map = mapRef.current
    if (!map || !autoFit) return
    const dims = `${map.width}x${map.height}`
    if (dims !== lastDims.current) {
      lastDims.current = dims
      setView({ zoom: 1, panX: 0, panY: 0 })
    }
  }, [mapVersion, autoFit, mapRef])

  // --- interaction --------------------------------------------------------
  const onWheel = useCallback((event) => {
    event.preventDefault()
    setAutoFit(false)
    setView((prev) => ({
      ...prev,
      zoom: Math.min(24, Math.max(0.4, prev.zoom * (event.deltaY < 0 ? 1.12 : 0.89))),
    }))
  }, [])

  const onPointerDown = useCallback((event) => {
    dragRef.current = {
      startX: event.clientX,
      startY: event.clientY,
      panX: view.panX,
      panY: view.panY,
      moved: false,
    }
    event.currentTarget.setPointerCapture?.(event.pointerId)
  }, [view.panX, view.panY])

  const onPointerMove = useCallback((event) => {
    const drag = dragRef.current
    if (!drag) return
    const dx = event.clientX - drag.startX
    const dy = event.clientY - drag.startY
    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) drag.moved = true
    setAutoFit(false)
    setView((prev) => ({ ...prev, panX: drag.panX + dx, panY: drag.panY + dy }))
  }, [])

  const onPointerUp = useCallback(
    (event) => {
      const drag = dragRef.current
      dragRef.current = null
      // A drag pans the view; only a genuine click sets a goal.
      if (!drag || drag.moved) return

      const transform = transformRef.current
      if (!transform) return
      const rect = event.currentTarget.getBoundingClientRect()
      const [xM, yM] = transform.screenToWorld(
        event.clientX - rect.left,
        event.clientY - rect.top,
      )
      setGoalHint({ x: xM, y: yM })
    },
    [],
  )

  const confirmGoal = useCallback(() => {
    if (!goalHint) return
    sendGoal(goalHint.x, goalHint.y, 0)
    setGoalHint(null)
  }, [goalHint, sendGoal])

  const mapInfo = useMemo(() => {
    const map = mapRef.current
    if (!map?.width) return null
    return {
      cells: `${map.width}×${map.height}`,
      metres: `${(map.width * map.resolution).toFixed(1)}×${(
        map.height * map.resolution
      ).toFixed(1)} m`,
      resolution: map.resolution,
    }
  }, [mapVersion, mapRef])

  return (
    <div>
      <div className="row" style={{ marginBottom: 10 }}>
        <label className="small dim">
          <input
            type="checkbox"
            checked={showTrail}
            onChange={(e) => setShowTrail(e.target.checked)}
          />{' '}
          Pose trail
        </label>
        <button
          className="sm"
          onClick={() => {
            setAutoFit(true)
            setView({ zoom: 1, panX: 0, panY: 0 })
          }}
        >
          Fit to view
        </button>
        <div className="spacer" />
        <span className="small faint mono">
          {mapInfo
            ? `${mapInfo.cells} cells · ${mapInfo.metres} · ${mapInfo.resolution} m/cell`
            : 'no map'}
        </span>
      </div>

      <div className="canvas-wrap" ref={wrapRef}>
        <canvas
          ref={canvasRef}
          onWheel={onWheel}
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={onPointerUp}
          style={{ cursor: dragRef.current ? 'grabbing' : 'crosshair' }}
        />
        <div className="canvas-overlay">
          drag to pan · scroll to zoom · click to pick a goal
        </div>
      </div>

      {goalHint && (
        <div className="notice info" style={{ marginTop: 12 }}>
          <strong>
            Goal at x={goalHint.x.toFixed(2)} m, y={goalHint.y.toFixed(2)} m
          </strong>
          <div className="row" style={{ marginTop: 8 }}>
            <button
              className="primary sm"
              onClick={confirmGoal}
              disabled={controlMode !== 'nav2'}
              title={
                controlMode !== 'nav2'
                  ? 'Switch to Nav2 mode on the Dashboard first'
                  : 'Send this goal to Nav2'
              }
            >
              Send to Nav2
            </button>
            <button className="sm" onClick={() => setGoalHint(null)}>
              Cancel
            </button>
            {controlMode !== 'nav2' && (
              <span className="small warn faint">
                Control mode is Manual — switch to Nav2 on the Dashboard to
                accept goals.
              </span>
            )}
          </div>
        </div>
      )}
    </div>
  )
}
