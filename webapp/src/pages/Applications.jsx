/**
 * Usage & Applications.
 *
 * Doubles as a slide deck. The same content renders two ways:
 *  - Document mode: everything stacked, scrollable, for reading and reference.
 *  - Presentation mode: one section per slide, arrow-key navigation, larger
 *    type, chrome hidden. Meant to be projected.
 *
 * Content is kept honest about what the hardware can actually do. A 2S-powered
 * N20 robot with a 6 m 2D lidar is genuinely useful for some things and
 * genuinely unsuitable for others, and a deck that oversells it would fall apart
 * under the first informed question. Each application therefore carries an
 * explicit readiness marker: what works today, what needs additions, and what
 * the platform cannot do.
 */

import { useCallback, useEffect, useState } from 'react'
import { useBot } from '../lib/store.jsx'

const READY = {
  now: { label: 'Works today', cls: 'ok' },
  addon: { label: 'Needs an add-on', cls: 'warn' },
  research: { label: 'Research direction', cls: '' },
}

/* ========================================================================== */
/* Content                                                                     */
/* ========================================================================== */

const CAPABILITIES = [
  {
    title: '360° mapping',
    detail: 'Builds a metric occupancy grid of an unknown space from scratch',
    stat: '6 m',
    statLabel: 'scan radius',
  },
  {
    title: 'Self-localisation',
    detail: 'Pose-graph SLAM with loop closure keeps position consistent',
    stat: '5 cm',
    statLabel: 'map resolution',
  },
  {
    title: 'Autonomous navigation',
    detail: 'A* global planning with dynamic-window local avoidance',
    stat: '0.3 m/s',
    statLabel: 'cruise speed',
  },
  {
    title: 'Remote operation',
    detail: 'Full control and live telemetry from any browser on the network',
    stat: '3',
    statLabel: 'safety layers',
  },
]

const SECTORS = [
  {
    icon: '🎓',
    name: 'Education & teaching',
    ready: 'now',
    lead:
      'The strongest fit. Every layer of a real robotics stack is present and legible at a scale a student can hold.',
    points: [
      'A complete SLAM + Nav2 pipeline students can read end to end, not a black box',
      'Live tuning makes cause and effect immediate — change the inflation radius, watch the path change',
      'The web UI removes the ROS learning cliff for a first lesson while leaving the real ROS graph underneath',
      'Cheap enough to build a classroom set; the whole BOM is hobby-grade',
    ],
    why:
      'Most teaching robots either hide the algorithms behind a toy API or demand a full ROS workstation per student. This sits in between.',
  },
  {
    icon: '🏭',
    name: 'Indoor inspection & monitoring',
    ready: 'addon',
    lead:
      'Repeatable patrols of a known indoor space, logging what changed since last time.',
    points: [
      'Server rooms and utility spaces: patrol a fixed route on a schedule',
      'Warehouse aisle occupancy — is a bay empty or blocked',
      'After-hours walkthroughs of offices and labs',
      'Change detection by differencing today\'s occupancy grid against a saved baseline',
    ],
    why:
      'The mapping and navigation are already there. What is missing is a camera and a scheduler — both additions rather than redesigns.',
    needs: 'A camera module, a docking charger, and scheduled-mission logic.',
  },
  {
    icon: '♿',
    name: 'Accessibility & assistive mapping',
    ready: 'addon',
    lead:
      'Survey a building and produce data about how navigable it actually is.',
    points: [
      'Measure real corridor and doorway widths from the occupancy grid',
      'Flag choke points too narrow for a wheelchair',
      'Generate indoor floor plans for buildings that have none',
      'Verify a route stays clear over repeated passes',
    ],
    why:
      'A 5 cm grid is precise enough for meaningful width measurements, and the flood-fill room detection already identifies enclosed regions and choke points.',
    needs: 'Export to a standard floor-plan format, and multi-session map merging.',
  },
  {
    icon: '🔬',
    name: 'Algorithm research platform',
    ready: 'now',
    lead:
      'A physical testbed for path planning and SLAM work, where sim-to-real gaps show up honestly.',
    points: [
      'Swap planners: SmacPlanner2D, NavFn, or a custom RRT*/D* Lite plugin',
      'Swap controllers: DWB today, TEB or MPPI as drop-in alternatives',
      'Benchmark against wheel-slip, encoder noise and WiFi latency that simulators smooth over',
      'Every run is logged with collision and stall counts for quantitative comparison',
    ],
    why:
      'Standard ROS2 interfaces throughout mean published Nav2 plugins work without modification.',
  },
  {
    icon: '🏆',
    name: 'Competitions & demonstrations',
    ready: 'now',
    lead:
      'Maze solving, autonomous navigation challenges, project exhibitions.',
    points: [
      'The live map projected on a screen makes an abstract algorithm visible to an audience',
      'Manual override is always one keypress away when a demo goes sideways',
      'Runs entirely on a local network — no internet dependency on the day',
      'This page doubles as the presentation itself',
    ],
    why:
      'The visualisation is the demo. Watching a map assemble in real time communicates more than any slide about pose graphs.',
  },
  {
    icon: '🚧',
    name: 'Hazardous-space survey',
    ready: 'research',
    lead:
      'Map somewhere a person should not walk into blind — with clear limits.',
    points: [
      'Crawl spaces, ducts and voids too tight or unsafe for entry',
      'Produce a metric map before committing a person to the space',
      'Teleoperated with live scan feedback beyond line of sight',
    ],
    why:
      'The platform can map an unknown enclosed space, which is the core requirement.',
    needs:
      'Realistically: better wheels or tracks, an IMU for tilt, and a wireless link with more range than consumer WiFi.',
    caution:
      'Not for anything genuinely dangerous. This is a 2S hobby robot with no sealing, no redundancy and no fail-safe recovery — treat it as a proof of concept, not equipment anyone should depend on.',
  },
]

const INTERACTIVE = [
  {
    title: 'Live map projection',
    detail:
      'Put the Live Map page on a projector while the robot drives. The occupancy grid assembling itself in real time is the single most effective explanation of SLAM available.',
    effort: 'Zero — works now',
  },
  {
    title: 'Click-to-navigate',
    detail:
      'Hand someone the tablet and let them click a point on the map. The robot plans and drives there. The gap between "click" and "arrives" is where planning becomes intuitive.',
    effort: 'Zero — works now',
  },
  {
    title: 'Tuning as a live experiment',
    detail:
      'Drop the inflation radius to 0 and watch the robot clip corners. Raise it and watch it refuse a doorway. Two slider moves teach more about costmaps than a lecture.',
    effort: 'Zero — works now',
  },
  {
    title: 'Break it on purpose',
    detail:
      'Set wheel_base_mm to a deliberately wrong value and watch the map rotate away from reality. Making odometry error visible is far more memorable than describing it.',
    effort: 'Zero — works now',
  },
  {
    title: 'Race the human',
    detail:
      'Time a person driving manually against Nav2 planning the same route. Nav2 usually loses on speed and wins on consistency — a good conversation starter about what autonomy is actually for.',
    effort: 'Zero — works now',
  },
  {
    title: 'Multi-robot swarm view',
    detail:
      'Multiple robots on one backend, one shared map, contributions merged. Turns a single-robot demo into a distributed-systems demo.',
    effort: 'Backend supports multiple sockets; needs map merging and namespacing',
  },
  {
    title: 'Voice interaction',
    detail:
      'Speech-to-text into the existing intent parser, spoken status back out. "Where are you?" answered from real pose data.',
    effort: 'Web Speech API in the browser plus the intent-extraction path',
  },
  {
    title: 'AR overlay',
    detail:
      'Point a phone at the room and see the robot\'s costmap and planned path overlaid on the real floor. Makes the invisible planning layer visible in place.',
    effort: 'WebXR plus a camera-pose calibration step',
  },
]

const AI_ROADMAP = [
  {
    phase: 'Already built',
    cls: 'ok',
    items: [
      {
        name: 'LLM tuning advisor',
        what:
          'Session statistics — collisions, stalls, coverage, distance — go to a model, which returns structured parameter suggestions with reasoning. Every suggestion is validated against the parameter registry and clamped to range before it can be applied.',
        value: 'Turns "it keeps hitting things" into a specific parameter change.',
      },
      {
        name: 'Map interpretation',
        what:
          'Flood-fill finds enclosed regions and computes areas locally; the model turns those numbers into a plain-language description of the space.',
        value: 'Makes an occupancy grid legible to someone who cannot read one.',
      },
      {
        name: 'Bounded-output safety',
        what:
          'No model output reaches a motor. Suggestions land in the tuning UI for human confirmation, and the firmware clamps independently regardless.',
        value: 'The pattern that makes everything below safe to add.',
      },
    ],
  },
  {
    phase: 'Near term — modest additions',
    cls: 'warn',
    items: [
      {
        name: 'Natural-language goals',
        what:
          'Parse "go to the far corner of the room on the left" into a coordinate. The model extracts intent only; a deterministic layer resolves it against the actual map and bounds-checks the result.',
        value: 'Removes the need to understand map coordinates.',
        note: 'Flagged as a stretch goal in the spec precisely because the intent/execution split has to be strict.',
      },
      {
        name: 'Semantic labelling',
        what:
          'A camera plus an object-detection model tags map regions: "kitchen", "doorway", "desk". Labels attach to grid coordinates.',
        value: 'Lets you say "go to the kitchen" instead of "go to (3.2, 1.8)".',
        note: 'Needs a camera — not in the current BOM.',
      },
      {
        name: 'Learned failure prediction',
        what:
          'Log every stall and collision with the local scan geometry, then train a small classifier to recognise the situations that precede them and slow down early.',
        value: 'Fewer stalls without hand-tuning thresholds for every floor surface.',
      },
      {
        name: 'Anomaly detection on patrol',
        what:
          'Difference the current occupancy grid against a saved baseline and report what moved. A model summarises the diff in words.',
        value: 'Turns repeated patrols into an actual monitoring product.',
      },
    ],
  },
  {
    phase: 'Ambitious — substantial work',
    cls: '',
    items: [
      {
        name: 'Learned local planner',
        what:
          'Replace or augment DWB with a policy trained in simulation and fine-tuned on real logs. Reinforcement learning happens off-board; only the trained policy runs at inference time.',
        value: 'Smoother motion in tight spaces than a hand-tuned cost function achieves.',
        note: 'The spec is explicit that on-device RL training is not feasible on a Cortex-M4 — training must be off-board.',
      },
      {
        name: 'Vision-language navigation',
        what:
          '"Find the red chair and stop next to it." A VLM grounds the description against a camera feed, and the existing planner handles the driving.',
        value: 'Goals described the way people actually describe places.',
      },
      {
        name: 'Predictive exploration',
        what:
          'Predict unmapped structure from partial observation — most indoor spaces are rectilinear and repetitive — and choose frontiers that maximise expected information gain.',
        value: 'Faster complete coverage than naive frontier exploration.',
      },
      {
        name: 'Self-calibration',
        what:
          'Estimate wheel_base_mm, wheel_diameter_mm and encoder_cpr automatically by comparing wheel odometry against scan-matching over a calibration drive.',
        value:
          'Removes the three manual measurements that most often ruin a first build.',
        note: 'Arguably the highest value-per-effort item on this list.',
      },
    ],
  },
]

const LIMITS = [
  ['2D only', 'A single-plane lidar cannot see a step, a table edge or an overhang. Anything below or above the scan plane is invisible.'],
  ['Indoor, flat floors', 'N20 motors and small wheels do not handle carpet transitions, thresholds or any incline worth mentioning.'],
  ['Tethered to WiFi', 'SLAM and planning run on the laptop. Out of WiFi range the watchdog stops the robot within 2 seconds — by design, but it is a hard operational limit.'],
  ['Runtime measured in tens of minutes', 'A 2S LiPo powering a spinning lidar plus two motors does not last an afternoon.'],
  ['No manipulation', 'It maps and navigates. It cannot pick anything up.'],
  ['Not safety-rated', 'Three independent stop layers make it safe to work near. That is not the same as certified for use around people who have not agreed to be near a robot.'],
]

/* ========================================================================== */
/* Presentation shell                                                          */
/* ========================================================================== */

function ReadyBadge({ ready }) {
  const meta = READY[ready]
  if (!meta) return null
  return <span className={`chip ${meta.cls}`}>{meta.label}</span>
}

export default function Applications() {
  const { status } = useBot()
  const [presenting, setPresenting] = useState(false)
  const [slide, setSlide] = useState(0)

  const slides = [
    'title',
    'capabilities',
    ...SECTORS.map((_, i) => `sector-${i}`),
    'interactive',
    ...AI_ROADMAP.map((_, i) => `ai-${i}`),
    'limits',
    'close',
  ]
  const total = slides.length

  const next = useCallback(() => setSlide((s) => Math.min(total - 1, s + 1)), [total])
  const prev = useCallback(() => setSlide((s) => Math.max(0, s - 1)), [])

  useEffect(() => {
    if (!presenting) return
    const onKey = (event) => {
      if (event.key === 'Escape') setPresenting(false)
      else if (['ArrowRight', 'ArrowDown', ' ', 'PageDown'].includes(event.key)) {
        event.preventDefault()
        next()
      } else if (['ArrowLeft', 'ArrowUp', 'PageUp'].includes(event.key)) {
        event.preventDefault()
        prev()
      } else if (event.key === 'Home') setSlide(0)
      else if (event.key === 'End') setSlide(total - 1)
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [presenting, next, prev, total])

  /* ---------------- slide renderers ---------------- */
  const current = slides[slide]

  const renderSlide = () => {
    if (current === 'title') {
      return (
        <div className="slide-center">
          <h1 className="slide-h1">
            SLAM<span style={{ color: 'var(--accent)' }}>Bot</span>
          </h1>
          <p className="slide-lead">
            Autonomous indoor mapping and navigation on hobby-grade hardware
          </p>
          <div className="row" style={{ justifyContent: 'center', gap: 12, marginTop: 24 }}>
            <span className="chip">RPLIDAR A1M8</span>
            <span className="chip">ROS2 · slam_toolbox · Nav2</span>
            <span className="chip">A* + DWB</span>
            <span className="chip">LLM tuning advisor</span>
          </div>
          <p className="dim" style={{ marginTop: 28, fontSize: 15 }}>
            Usage, applications, and where AI takes it next
          </p>
        </div>
      )
    }

    if (current === 'capabilities') {
      return (
        <>
          <h2 className="slide-h2">What it does</h2>
          <div className="grid grid-4" style={{ marginTop: 20 }}>
            {CAPABILITIES.map((cap) => (
              <div className="stat" key={cap.title}>
                <div className="stat-value" style={{ fontSize: 26, color: 'var(--accent)' }}>
                  {cap.stat}
                </div>
                <div className="stat-sub">{cap.statLabel}</div>
                <div style={{ fontWeight: 650, marginTop: 10, fontSize: 14 }}>
                  {cap.title}
                </div>
                <div className="small dim" style={{ marginTop: 4 }}>
                  {cap.detail}
                </div>
              </div>
            ))}
          </div>
          <div className="notice info" style={{ marginTop: 20 }}>
            <strong>The distinguishing feature: everything is live-tunable.</strong>
            Every parameter lives in a runtime struct on the microcontroller, not
            a compiled constant. Adjusting behaviour takes a slider move over
            WiFi, not a rebuild-and-reflash cycle — which is what makes the robot
            usable as a teaching and research instrument rather than a fixed
            appliance.
          </div>
        </>
      )
    }

    if (current?.startsWith('sector-')) {
      const sector = SECTORS[Number(current.split('-')[1])]
      return (
        <>
          <div className="row" style={{ gap: 14, alignItems: 'center' }}>
            <span style={{ fontSize: 38 }}>{sector.icon}</span>
            <h2 className="slide-h2" style={{ margin: 0 }}>{sector.name}</h2>
            <ReadyBadge ready={sector.ready} />
          </div>
          <p className="slide-lead" style={{ marginTop: 14 }}>{sector.lead}</p>
          <ul className="rules" style={{ fontSize: 15, marginTop: 18 }}>
            {sector.points.map((point) => (
              <li key={point}>{point}</li>
            ))}
          </ul>
          <div className="notice info" style={{ marginTop: 18 }}>
            <strong>Why this fits</strong>
            {sector.why}
          </div>
          {sector.needs && (
            <div className="notice warn">
              <strong>What it would need</strong>
              {sector.needs}
            </div>
          )}
          {sector.caution && (
            <div className="notice err">
              <strong>Honest caveat</strong>
              {sector.caution}
            </div>
          )}
        </>
      )
    }

    if (current === 'interactive') {
      return (
        <>
          <h2 className="slide-h2">Making it interactive</h2>
          <p className="slide-lead">
            Five of these need nothing but the robot you already have.
          </p>
          <div className="grid grid-2" style={{ marginTop: 18 }}>
            {INTERACTIVE.map((item) => (
              <div className="stat" key={item.title}>
                <div className="row" style={{ gap: 8, marginBottom: 6 }}>
                  <span style={{ fontWeight: 650, fontSize: 14 }}>{item.title}</span>
                </div>
                <div className="small dim">{item.detail}</div>
                <div
                  className="small"
                  style={{
                    marginTop: 8,
                    color: item.effort.startsWith('Zero')
                      ? 'var(--ok)'
                      : 'var(--text-faint)',
                  }}
                >
                  {item.effort}
                </div>
              </div>
            ))}
          </div>
        </>
      )
    }

    if (current?.startsWith('ai-')) {
      const group = AI_ROADMAP[Number(current.split('-')[1])]
      return (
        <>
          <div className="row" style={{ gap: 12, alignItems: 'center' }}>
            <h2 className="slide-h2" style={{ margin: 0 }}>Advancing it with AI</h2>
            <span className={`chip ${group.cls}`}>{group.phase}</span>
          </div>
          <div style={{ marginTop: 18 }}>
            {group.items.map((item) => (
              <div className="suggestion" key={item.name}>
                <div className="suggestion-head">
                  <span className="suggestion-param" style={{ fontSize: 14 }}>
                    {item.name}
                  </span>
                </div>
                <div style={{ fontSize: 14 }}>{item.what}</div>
                <div
                  className="small"
                  style={{ marginTop: 8, color: 'var(--ok)' }}
                >
                  → {item.value}
                </div>
                {item.note && (
                  <div className="small faint" style={{ marginTop: 6 }}>
                    Note: {item.note}
                  </div>
                )}
              </div>
            ))}
          </div>
        </>
      )
    }

    if (current === 'limits') {
      return (
        <>
          <h2 className="slide-h2">What it cannot do</h2>
          <p className="slide-lead">
            Worth stating plainly — every one of these is a consequence of
            deliberate cost and scope choices, not an oversight.
          </p>
          <div className="grid grid-2" style={{ marginTop: 18 }}>
            {LIMITS.map(([limit, why]) => (
              <div className="stat" key={limit}>
                <div style={{ fontWeight: 650, fontSize: 14, color: 'var(--warn)' }}>
                  {limit}
                </div>
                <div className="small dim" style={{ marginTop: 5 }}>{why}</div>
              </div>
            ))}
          </div>
        </>
      )
    }

    // close
    return (
      <div className="slide-center">
        <h2 className="slide-h1" style={{ fontSize: 34 }}>
          A complete stack, at a scale you can hold
        </h2>
        <div className="grid grid-3" style={{ marginTop: 26, textAlign: 'left' }}>
          <div className="stat">
            <div className="stat-label">Strongest today</div>
            <div style={{ fontWeight: 650, marginTop: 4 }}>
              Education, research, demonstration
            </div>
            <div className="small dim" style={{ marginTop: 6 }}>
              Every layer is present and readable, and live tuning makes cause
              and effect immediate.
            </div>
          </div>
          <div className="stat">
            <div className="stat-label">Nearest expansion</div>
            <div style={{ fontWeight: 650, marginTop: 4 }}>
              Add a camera
            </div>
            <div className="small dim" style={{ marginTop: 6 }}>
              Unlocks semantic labelling, inspection and vision-language
              navigation in one addition.
            </div>
          </div>
          <div className="stat">
            <div className="stat-label">Best value AI work</div>
            <div style={{ fontWeight: 650, marginTop: 4 }}>
              Self-calibration
            </div>
            <div className="small dim" style={{ marginTop: 6 }}>
              Removes the three manual measurements that most often ruin a first
              build.
            </div>
          </div>
        </div>
        <div className="notice info" style={{ marginTop: 26, textAlign: 'left' }}>
          <strong>The principle that makes the AI parts safe to extend</strong>
          No model output ever reaches a motor directly. Models suggest, propose
          and describe; deterministic code validates, clamps and executes. That
          boundary is what allows every item on the roadmap to be added without
          making the robot less trustworthy.
        </div>
      </div>
    )
  }

  /* ---------------- presentation mode ---------------- */
  if (presenting) {
    return (
      <div className="deck">
        <div className="deck-stage">{renderSlide()}</div>
        <div className="deck-bar">
          <button className="sm" onClick={() => setPresenting(false)}>
            ✕ Exit
          </button>
          <div className="spacer" />
          <button className="sm" onClick={prev} disabled={slide === 0}>
            ◀
          </button>
          <span className="mono small dim">
            {slide + 1} / {total}
          </span>
          <button className="sm" onClick={next} disabled={slide === total - 1}>
            ▶
          </button>
          <div className="spacer" />
          <span className="small faint">arrow keys · Esc to exit</span>
        </div>
        <div className="deck-progress">
          <div
            className="deck-progress-fill"
            style={{ width: `${((slide + 1) / total) * 100}%` }}
          />
        </div>
      </div>
    )
  }

  /* ---------------- document mode ---------------- */
  return (
    <>
      <div className="panel">
        <div className="panel-title">
          Usage &amp; applications
          <button
            className="primary sm"
            onClick={() => {
              setSlide(0)
              setPresenting(true)
            }}
          >
            ▶ Present ({total} slides)
          </button>
        </div>
        <p className="dim" style={{ marginTop: 0 }}>
          Where this robot is genuinely useful, how to make it interactive for an
          audience, and what AI adds next. Presentation mode turns this page into
          a slide deck — arrow keys to navigate, Esc to exit.
        </p>
        <div className="grid grid-4" style={{ marginTop: 14 }}>
          {CAPABILITIES.map((cap) => (
            <div className="stat" key={cap.title}>
              <div className="stat-value" style={{ color: 'var(--accent)' }}>
                {cap.stat}
              </div>
              <div className="stat-sub">{cap.statLabel}</div>
              <div style={{ fontWeight: 650, marginTop: 8, fontSize: 13 }}>
                {cap.title}
              </div>
              <div className="small dim" style={{ marginTop: 3 }}>
                {cap.detail}
              </div>
            </div>
          ))}
        </div>
        {status?.session?.distance_travelled_mm > 0 && (
          <div className="notice info" style={{ marginTop: 14, marginBottom: 0 }}>
            <strong>This robot's own record so far</strong>
            {(status.session.distance_travelled_mm / 1000).toFixed(1)} m driven ·{' '}
            {status.session.collisions} collision event
            {status.session.collisions === 1 ? '' : 's'} ·{' '}
            {status.map_meta?.width
              ? `${(status.map_meta.width * status.map_meta.resolution).toFixed(1)} × ${(
                  status.map_meta.height * status.map_meta.resolution
                ).toFixed(1)} m mapped`
              : 'no map yet'}
          </div>
        )}
      </div>

      {/* --- sectors --- */}
      <div className="panel">
        <div className="panel-title">Where it can be used</div>
        <p className="dim small" style={{ marginTop: 0 }}>
          Ordered by how well the current hardware actually fits, not by how
          impressive it sounds.
        </p>
        {SECTORS.map((sector) => (
          <div className="app-card" key={sector.name}>
            <div className="row" style={{ gap: 12, alignItems: 'center' }}>
              <span style={{ fontSize: 26 }}>{sector.icon}</span>
              <span style={{ fontWeight: 650, fontSize: 15 }}>{sector.name}</span>
              <ReadyBadge ready={sector.ready} />
            </div>
            <p className="dim" style={{ margin: '8px 0' }}>{sector.lead}</p>
            <ul className="rules small">
              {sector.points.map((point) => (
                <li key={point}>{point}</li>
              ))}
            </ul>
            <div className="param-help">
              <strong className="dim">Why this fits:</strong> {sector.why}
            </div>
            {sector.needs && (
              <div className="param-help" style={{ color: 'var(--warn)' }}>
                <strong>Needs:</strong> {sector.needs}
              </div>
            )}
            {sector.caution && (
              <div className="notice err" style={{ marginTop: 8, marginBottom: 0 }}>
                <strong>Honest caveat</strong>
                {sector.caution}
              </div>
            )}
          </div>
        ))}
      </div>

      {/* --- interactive --- */}
      <div className="panel">
        <div className="panel-title">
          Making it more interactive
          <span className="small faint">
            five of these need nothing new
          </span>
        </div>
        <div className="grid grid-2">
          {INTERACTIVE.map((item) => (
            <div className="stat" key={item.title}>
              <div style={{ fontWeight: 650, fontSize: 13.5 }}>{item.title}</div>
              <div className="small dim" style={{ marginTop: 5 }}>{item.detail}</div>
              <div
                className="small"
                style={{
                  marginTop: 8,
                  color: item.effort.startsWith('Zero') ? 'var(--ok)' : 'var(--text-faint)',
                }}
              >
                {item.effort}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* --- AI roadmap --- */}
      <div className="panel">
        <div className="panel-title">Advancing it with AI</div>
        <p className="dim small" style={{ marginTop: 0 }}>
          Grouped by how much work each actually takes. The first group is
          already running in this app.
        </p>
        {AI_ROADMAP.map((group) => (
          <div key={group.phase} style={{ marginBottom: 20 }}>
            <div className="row" style={{ marginBottom: 10 }}>
              <span className={`chip ${group.cls}`}>{group.phase}</span>
            </div>
            {group.items.map((item) => (
              <div className="suggestion" key={item.name}>
                <div className="suggestion-head">
                  <span className="suggestion-param">{item.name}</span>
                </div>
                <div className="small">{item.what}</div>
                <div className="small" style={{ marginTop: 7, color: 'var(--ok)' }}>
                  → {item.value}
                </div>
                {item.note && (
                  <div className="small faint" style={{ marginTop: 5 }}>
                    Note: {item.note}
                  </div>
                )}
              </div>
            ))}
          </div>
        ))}

        <div className="notice info">
          <strong>The boundary that keeps this safe</strong>
          No model output ever reaches a motor directly. Models suggest, propose
          and describe; deterministic code validates, clamps and executes. The
          tuning advisor already works this way — suggestions are checked against
          the parameter registry, clamped to range, and shown for confirmation
          before anything is applied, and the firmware clamps again regardless.
          Every item above can be added without weakening that.
        </div>
      </div>

      {/* --- limits --- */}
      <div className="panel">
        <div className="panel-title">Honest limitations</div>
        <p className="dim small" style={{ marginTop: 0 }}>
          Every one of these follows from a deliberate cost and scope decision.
          Knowing them is what separates a demo that survives questions from one
          that does not.
        </p>
        <div className="grid grid-2">
          {LIMITS.map(([limit, why]) => (
            <div className="stat" key={limit}>
              <div style={{ fontWeight: 650, fontSize: 13.5, color: 'var(--warn)' }}>
                {limit}
              </div>
              <div className="small dim" style={{ marginTop: 5 }}>{why}</div>
            </div>
          ))}
        </div>
      </div>
    </>
  )
}
