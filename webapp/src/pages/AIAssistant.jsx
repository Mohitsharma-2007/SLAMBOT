/**
 * AI Assistant — §6.6
 *
 * Three things, all backed by OpenRouter through the backend:
 *   1. "Analyze last session" -> structured tuning suggestions, applied to the
 *      Tuning Panel's values only after the operator clicks Apply. §6.6 requires
 *      the one-click apply to be explicit, never automatic.
 *   2. "Explain this map"     -> plain-language description of the occupancy grid.
 *   3. Chat panel             -> questions about the robot's state.
 *
 * The suggestions the backend returns are already validated against the §10
 * registry and clamped to each parameter's range, so a hallucinated parameter
 * name or an absurd value cannot reach the hardware. Where a value was clamped,
 * that is shown rather than hidden.
 */

import { useCallback, useEffect, useRef, useState } from 'react'
import { api, useBot } from '../lib/store.jsx'

function SuggestionCard({ suggestion, current, onApply, applied }) {
  const target = suggestion.suggested_value
  const delta =
    typeof current === 'number' && typeof target === 'number'
      ? target - current
      : null

  return (
    <div className="suggestion">
      <div className="suggestion-head">
        <span className="suggestion-param">{suggestion.parameter}</span>
        <span className="suggestion-delta dim">
          {current ?? '?'} → <strong>{target}</strong>
          {delta !== null && delta !== 0 && (
            <span className="faint">
              {' '}
              ({delta > 0 ? '+' : ''}
              {Number(delta.toFixed(3))})
            </span>
          )}
        </span>
        {suggestion.note && (
          <span className="badge unsaved" title="The model's raw value was outside this parameter's allowed range">
            {suggestion.note}
          </span>
        )}
        <div className="spacer" />
        <button
          className={applied ? 'sm success' : 'sm primary'}
          onClick={() => onApply(suggestion)}
          disabled={applied}
        >
          {applied ? 'Applied' : 'Apply'}
        </button>
      </div>
      {suggestion.reasoning && (
        <div className="suggestion-why">{suggestion.reasoning}</div>
      )}
    </div>
  )
}

export default function AIAssistant() {
  const { tuning, updateTuning, status, connected } = useBot()

  const [aiStatus, setAiStatus] = useState(null)
  const [analysis, setAnalysis] = useState(null)
  const [analysisBusy, setAnalysisBusy] = useState(false)
  const [applied, setApplied] = useState(() => new Set())

  const [mapDesc, setMapDesc] = useState(null)
  const [mapBusy, setMapBusy] = useState(false)

  const [messages, setMessages] = useState([])
  const [draft, setDraft] = useState('')
  const [chatBusy, setChatBusy] = useState(false)
  const chatRef = useRef(null)

  const [error, setError] = useState(null)

  const refreshAiStatus = useCallback(async () => {
    try {
      setAiStatus(await api('/ai/status'))
    } catch (err) {
      setError(err.message)
    }
  }, [])

  useEffect(() => {
    refreshAiStatus()
  }, [refreshAiStatus])

  useEffect(() => {
    const element = chatRef.current
    if (element) element.scrollTop = element.scrollHeight
  }, [messages])

  const analyzeSession = useCallback(async () => {
    setAnalysisBusy(true)
    setError(null)
    setApplied(new Set())
    try {
      const result = await api('/ai/analyze-session', { method: 'POST' })
      setAnalysis(result)
      if (!result.ok) setError(result.error)
    } catch (err) {
      setError(err.message)
    } finally {
      setAnalysisBusy(false)
      refreshAiStatus()
    }
  }, [refreshAiStatus])

  const describeMap = useCallback(async () => {
    setMapBusy(true)
    setError(null)
    try {
      const result = await api('/ai/describe-map', { method: 'POST' })
      setMapDesc(result)
      if (!result.ok) setError(result.error)
    } catch (err) {
      setError(err.message)
    } finally {
      setMapBusy(false)
      refreshAiStatus()
    }
  }, [refreshAiStatus])

  const applySuggestion = useCallback(
    (suggestion) => {
      // Writes into the Tuning Panel's live values — the same validated path a
      // manual slider edit takes, so it is clamped again backend-side.
      updateTuning({ [suggestion.parameter]: suggestion.suggested_value })
      setApplied((prev) => new Set(prev).add(suggestion.parameter))
    },
    [updateTuning],
  )

  const applyAll = useCallback(() => {
    const suggestions = analysis?.suggestions ?? []
    if (suggestions.length === 0) return
    const values = {}
    suggestions.forEach((s) => {
      values[s.parameter] = s.suggested_value
    })
    updateTuning(values)
    setApplied(new Set(suggestions.map((s) => s.parameter)))
  }, [analysis, updateTuning])

  const sendChat = useCallback(async () => {
    const text = draft.trim()
    if (!text || chatBusy) return
    setDraft('')
    setMessages((prev) => [...prev, { role: 'user', text }])
    setChatBusy(true)
    try {
      const result = await api('/ai/chat', {
        method: 'POST',
        body: JSON.stringify({ message: text }),
      })
      setMessages((prev) => [
        ...prev,
        result.ok
          ? { role: 'bot', text: result.reply }
          : { role: 'error', text: result.error ?? 'request failed' },
      ])
    } catch (err) {
      setMessages((prev) => [...prev, { role: 'error', text: err.message }])
    } finally {
      setChatBusy(false)
      refreshAiStatus()
    }
  }, [draft, chatBusy, refreshAiStatus])

  const notConfigured = aiStatus && !aiStatus.configured
  const hasMap = (status?.map_meta?.width ?? 0) > 0

  return (
    <>
      {notConfigured && (
        <div className="notice warn">
          <strong>No OpenRouter API key configured.</strong>
          Set <span className="mono">OPENROUTER_API_KEY</span> in the backend's
          environment and restart it. Everything else in the app works without
          it — only this page needs it.
        </div>
      )}

      {error && <div className="notice err">{error}</div>}

      <div className="panel">
        <div className="panel-title">
          Model
          <button className="sm" onClick={refreshAiStatus}>
            Refresh
          </button>
        </div>
        <div className="row">
          <span className={`chip ${aiStatus?.model ? 'ok' : 'warn'}`}>
            <span className={`dot ${aiStatus?.model ? 'ok' : 'warn'}`} />
            {aiStatus?.model ?? 'none resolved'}
          </span>
          {aiStatus?.free_model_count > 0 && (
            <span className="chip">
              {aiStatus.free_model_count} free models available
            </span>
          )}
          <button
            className="sm"
            onClick={async () => {
              try {
                await api('/ai/resolve-model', { method: 'POST' })
                refreshAiStatus()
              } catch (err) {
                setError(err.message)
              }
            }}
          >
            Re-resolve
          </button>
        </div>
        {aiStatus?.last_error && (
          <div className="param-help" style={{ color: 'var(--warn)' }}>
            {aiStatus.last_error}
          </div>
        )}
        <div className="param-help">
          Per §9 the model is never hardcoded: the backend queries OpenRouter's
          model list at startup, keeps only entries priced at zero, and picks the
          first available candidate — free-tier availability rotates, so it
          falls through a candidate list and re-resolves on demand.
        </div>
      </div>

      <div className="grid grid-2">
        {/* --- session analysis --- */}
        <div className="panel">
          <div className="panel-title">Analyze last session</div>
          <div className="row" style={{ marginBottom: 10 }}>
            <button
              className="primary"
              onClick={analyzeSession}
              disabled={analysisBusy || notConfigured || !connected}
            >
              {analysisBusy ? 'Analyzing…' : 'Analyze last session'}
            </button>
            {analysis?.suggestions?.length > 1 && (
              <button className="sm" onClick={applyAll}>
                Apply all {analysis.suggestions.length}
              </button>
            )}
          </div>

          <div className="param-help" style={{ marginBottom: 12 }}>
            Sends a <em>summary</em> — collision count, stall count, map
            coverage, distance travelled, current tuning — not raw sensor
            streams (§9.1). Suggestions land in the Tuning Panel only when you
            click Apply.
          </div>

          {analysis?.session_summary && (
            <div className="row small dim" style={{ marginBottom: 10, gap: 14 }}>
              <span>
                collisions:{' '}
                <span className="mono">{analysis.session_summary.collision_count}</span>
              </span>
              <span>
                stalls:{' '}
                <span className="mono">{analysis.session_summary.stuck_events}</span>
              </span>
              <span>
                coverage:{' '}
                <span className="mono">
                  {analysis.session_summary.map_coverage_pct ?? '--'}%
                </span>
              </span>
              <span>
                distance:{' '}
                <span className="mono">
                  {(analysis.session_summary.distance_travelled_mm / 1000).toFixed(2)} m
                </span>
              </span>
            </div>
          )}

          {analysis?.summary && (
            <div className="notice info" style={{ marginBottom: 12 }}>
              {analysis.summary}
            </div>
          )}

          {analysis?.suggestions?.map((suggestion) => (
            <SuggestionCard
              key={suggestion.parameter}
              suggestion={suggestion}
              current={tuning?.values?.[suggestion.parameter]}
              onApply={applySuggestion}
              applied={applied.has(suggestion.parameter)}
            />
          ))}

          {analysis?.ok && analysis.suggestions.length === 0 && (
            <div className="empty">
              The model suggested no changes — usually a sign there isn't enough
              session data yet. Drive a mapping run first.
            </div>
          )}

          {analysis?.rejected?.length > 0 && (
            <div className="notice warn" style={{ marginTop: 10 }}>
              <strong>
                {analysis.rejected.length} suggestion
                {analysis.rejected.length === 1 ? '' : 's'} rejected by validation
              </strong>
              {analysis.rejected.map((item, index) => (
                <div key={index} className="small mono">
                  {item.parameter}: {item.reason}
                </div>
              ))}
            </div>
          )}
        </div>

        {/* --- map description --- */}
        <div className="panel">
          <div className="panel-title">Explain this map</div>
          <div className="row" style={{ marginBottom: 10 }}>
            <button
              className="primary"
              onClick={describeMap}
              disabled={mapBusy || notConfigured || !hasMap}
              title={!hasMap ? 'No occupancy grid yet — run slam_toolbox first' : undefined}
            >
              {mapBusy ? 'Describing…' : 'Explain this map'}
            </button>
            {!hasMap && <span className="small faint">no map available yet</span>}
          </div>

          <div className="param-help" style={{ marginBottom: 12 }}>
            Room counting and area ratios are computed locally by flood-fill; the
            model only puts words to numbers the backend already calculated
            (§9.2).
          </div>

          {mapDesc?.map_summary && !mapDesc.map_summary.empty && (
            <table className="data" style={{ marginBottom: 12 }}>
              <tbody>
                <tr>
                  <td>Explored</td>
                  <td className="num">
                    {mapDesc.map_summary.ratios.explored_pct}%
                  </td>
                </tr>
                <tr>
                  <td>Free area</td>
                  <td className="num">{mapDesc.map_summary.free_area_m2} m²</td>
                </tr>
                <tr>
                  <td>Room-like regions</td>
                  <td className="num">{mapDesc.map_summary.room_like_regions}</td>
                </tr>
                <tr>
                  <td>Grid size</td>
                  <td className="num">
                    {mapDesc.map_summary.grid.width_m}×
                    {mapDesc.map_summary.grid.height_m} m
                  </td>
                </tr>
              </tbody>
            </table>
          )}

          {mapDesc?.description && (
            <div className="msg bot" style={{ maxWidth: '100%' }}>
              {mapDesc.description}
            </div>
          )}
        </div>
      </div>

      {/* --- chat --- */}
      <div className="panel">
        <div className="panel-title">Chat</div>
        <div className="chat" ref={chatRef}>
          {messages.length === 0 ? (
            <div className="empty">
              Ask about the robot's telemetry, SLAM behaviour, wiring or tuning.
              The assistant cannot drive the bot — motion only ever comes from
              the Dashboard controls or Nav2.
            </div>
          ) : (
            messages.map((message, index) => (
              <div
                key={index}
                className={`msg ${
                  message.role === 'user'
                    ? 'user'
                    : message.role === 'error'
                      ? 'error'
                      : 'bot'
                }`}
              >
                {message.text}
              </div>
            ))
          )}
          {chatBusy && <div className="msg bot faint">thinking…</div>}
        </div>

        <div className="row" style={{ marginTop: 12 }}>
          <input
            type="text"
            placeholder="e.g. why does the map drift when I turn?"
            value={draft}
            onChange={(e) => setDraft(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter') sendChat()
            }}
            disabled={notConfigured}
            style={{ flex: 1 }}
          />
          <button
            className="primary"
            onClick={sendChat}
            disabled={chatBusy || notConfigured || !draft.trim()}
          >
            Send
          </button>
        </div>
      </div>

      <div className="panel">
        <div className="panel-title">Out of scope for v1</div>
        <div className="param-help">
          §9.3 / §13 list natural-language goal setting as a stretch goal, and
          deliberately so: an LLM's output must never reach the motors without a
          bounds-checked intermediate step. If it is added later, the model
          should only extract intent, with a deterministic parser turning that
          into a validated Nav2 goal pose.
        </div>
      </div>
    </>
  )
}
