/**
 * Inline SVG wiring diagrams for the Hardware Guide.
 *
 * Hand-drawn SVG rather than an image asset for three reasons: it stays sharp at
 * any zoom, the wire colours come from the same CSS variables as the rest of the
 * UI (so light/dark stay consistent), and the pin labels are real text — which
 * means they're searchable, selectable and readable by a screen reader.
 *
 * Every wire carries a stroke pattern as well as a colour: dashed for the 5 V
 * rail, dotted for ground, solid for signal, thick solid for raw battery. Colour
 * alone would be unreadable for a colour-blind builder standing over a
 * breadboard, and a miswired power rail is the one mistake that destroys parts.
 */

const C = {
  vbat: '#f85149',      // raw 7.4–8.4 V
  v5: '#ff9f4a',        // regulated 5 V
  gnd: '#8b98a5',       // common ground
  signal: '#4f9cf9',    // digital / PWM
  uart: '#a371f7',      // serial
  encoder: '#3fb950',   // encoder channels
  board: '#1c232c',
  boardEdge: '#3d4a5c',
  text: '#e6edf3',
  dim: '#9aa7b4',
  faint: '#6b7684',
  warn: '#d29922',
  accent: '#4f9cf9',
}

const STROKE = {
  vbat: { stroke: C.vbat, strokeWidth: 3 },
  v5: { stroke: C.v5, strokeWidth: 2.4, strokeDasharray: '8 4' },
  gnd: { stroke: C.gnd, strokeWidth: 2, strokeDasharray: '2 3' },
  signal: { stroke: C.signal, strokeWidth: 1.8 },
  uart: { stroke: C.uart, strokeWidth: 1.8 },
  encoder: { stroke: C.encoder, strokeWidth: 1.8 },
}

function Board({ x, y, w, h, title, subtitle }) {
  return (
    <g>
      <rect
        x={x}
        y={y}
        width={w}
        height={h}
        rx="6"
        fill={C.board}
        stroke={C.boardEdge}
        strokeWidth="1.5"
      />
      <text x={x + w / 2} y={y + 19} textAnchor="middle" fill={C.text} fontSize="13" fontWeight="650">
        {title}
      </text>
      {subtitle && (
        <text x={x + w / 2} y={y + 34} textAnchor="middle" fill={C.faint} fontSize="10">
          {subtitle}
        </text>
      )}
    </g>
  )
}

function Pin({ x, y, label, align = 'left', color = C.dim }) {
  const dx = align === 'left' ? 9 : -9
  return (
    <g>
      <circle cx={x} cy={y} r="3" fill={color} />
      <text
        x={x + dx}
        y={y + 3.5}
        textAnchor={align === 'left' ? 'start' : 'end'}
        fill={C.dim}
        fontSize="10"
        fontFamily="ui-monospace, monospace"
      >
        {label}
      </text>
    </g>
  )
}

function Wire({ d, kind, label, labelX, labelY }) {
  return (
    <g>
      <path d={d} fill="none" strokeLinecap="round" strokeLinejoin="round" {...STROKE[kind]} />
      {label && (
        <text
          x={labelX}
          y={labelY}
          fill={C.faint}
          fontSize="9.5"
          fontFamily="ui-monospace, monospace"
          textAnchor="middle"
        >
          {label}
        </text>
      )}
    </g>
  )
}

export function WireLegend() {
  const items = [
    ['vbat', 'Raw battery 7.4–8.4 V'],
    ['v5', 'Regulated 5 V rail'],
    ['gnd', 'Common ground'],
    ['signal', 'Digital / PWM signal'],
    ['encoder', 'Encoder channel'],
    ['uart', 'UART serial'],
  ]
  return (
    <div className="row" style={{ gap: 16, marginBottom: 12, flexWrap: 'wrap' }}>
      {items.map(([kind, label]) => (
        <span key={kind} className="row small dim" style={{ gap: 6 }}>
          <svg width="30" height="10" aria-hidden="true">
            <line x1="1" y1="5" x2="29" y2="5" strokeLinecap="round" {...STROKE[kind]} />
          </svg>
          {label}
        </span>
      ))}
    </div>
  )
}

/* ========================================================================== */
/* Power distribution — one battery, one buck converter, one breadboard rail   */
/* ========================================================================== */
export function PowerDiagram() {
  // Consumers on the 5 V breadboard rail. Motors are deliberately NOT here —
  // they hang off DRV8833 VM on raw battery, because routing ~1.5 A of stall
  // current through a shared logic rail browns out every board on it.
  const rail = [
    { y: 120, name: 'Arduino Uno R4', pin: '5V pin', draw: '~50 mA' },
    { y: 175, name: 'NodeMCU ESP8266', pin: 'VIN', draw: '~80 mA peak' },
    { y: 230, name: 'RPLIDAR A1M8', pin: '5V', draw: '~400–500 mA' },
    { y: 285, name: 'DRV8833 VCC', pin: 'logic only', draw: '~2 mA' },
  ]

  return (
    <svg viewBox="0 0 880 470" style={{ width: '100%', height: 'auto' }} role="img"
      aria-label="One 2S LiPo feeds an LM2596 buck converter trimmed to 5 V, which supplies a breadboard rail powering the Arduino, NodeMCU, RPLIDAR and DRV8833 logic. The same battery also feeds DRV8833 VM directly for motor power.">

      <Board x={20} y={180} w={125} h={78} title="2S LiPo" subtitle="7.4 V nom / 8.4 V full" />
      <text x={152} y={206} fill={C.vbat} fontSize="11" fontFamily="ui-monospace, monospace">+</text>
      <text x={152} y={252} fill={C.gnd} fontSize="11" fontFamily="ui-monospace, monospace">−</text>
      <circle cx={145} cy={202} r="3.5" fill={C.vbat} />
      <circle cx={145} cy={246} r="3.5" fill={C.gnd} />

      {/* split node — the battery + goes two places */}
      <Wire d="M145 202 H 200" kind="vbat" />
      <circle cx={200} cy={202} r="5" fill={C.vbat} />
      <text x={200} y={192} textAnchor="middle" fill={C.vbat} fontSize="9" fontWeight="650">
        split
      </text>

      {/* ---- branch A: raw to DRV8833 VM (motor power) ---- */}
      <Wire d="M200 202 V 70 H 640" kind="vbat" />
      <text x={410} y={62} textAnchor="middle" fill={C.vbat} fontSize="10" fontWeight="650">
        raw 7.4–8.4 V straight to motor power — bypasses the buck entirely
      </text>
      <rect x={640} y={48} width={225} height={46} rx="5"
        fill={C.board} stroke={C.vbat} strokeWidth="1.8" />
      <text x={654} y={67} fill={C.text} fontSize="11.5" fontWeight="600">DRV8833 VM → motors</text>
      <text x={654} y={82} fill={C.vbat} fontSize="9.5" fontFamily="ui-monospace, monospace">
        up to ~1.5 A stall · 10.8 V max, so 8.4 V is fine
      </text>

      {/* ---- branch B: into the buck ---- */}
      <Wire d="M200 202 V 350 H 250" kind="vbat" />

      <Board x={250} y={315} w={145} h={78} title="LM2596" subtitle="trim to 5.00 V" />
      <text x={322} y={410} textAnchor="middle" fill={C.warn} fontSize="10" fontWeight="650">
        ⚠ set to 5.00 V before connecting anything
      </text>
      <text x={322} y={424} textAnchor="middle" fill={C.faint} fontSize="9">
        needs a heatsink at ~0.6 A load
      </text>

      {/* ---- breadboard rail ---- */}
      <Wire d="M395 354 H 470" kind="v5" />
      <rect x={470} y={95} width={54} height={285} rx="5"
        fill={C.board} stroke={C.v5} strokeWidth="1.6" strokeDasharray="6 3" />
      <text x={497} y={370} textAnchor="middle" fill={C.v5} fontSize="9.5" fontWeight="650">
        5 V
      </text>
      <text x={497} y={112} textAnchor="middle" fill={C.v5} fontSize="9.5" fontWeight="650">
        rail
      </text>
      <text x={497} y={240} textAnchor="middle" fill={C.dim} fontSize="10" fontWeight="600"
        transform="rotate(-90 497 240)">
        BREADBOARD
      </text>

      {/* consumers off the rail */}
      {rail.map((item) => (
        <g key={item.name}>
          <Wire d={`M524 ${item.y} H 640`} kind="v5" />
          <rect x={640} y={item.y - 21} width={225} height={42} rx="5"
            fill={C.board} stroke={C.boardEdge} strokeWidth="1.2" />
          <text x={654} y={item.y - 4} fill={C.text} fontSize="11.5" fontWeight="600">
            {item.name}
          </text>
          <text x={654} y={item.y + 11} fill={C.faint} fontSize="9.5"
            fontFamily="ui-monospace, monospace">
            {item.pin} · {item.draw}
          </text>
        </g>
      ))}

      {/* total draw callout */}
      <rect x={640} y={306} width={225} height={34} rx="5"
        fill="#2a2113" stroke={C.warn} strokeWidth="1.4" />
      <text x={654} y={321} fill={C.warn} fontSize="10.5" fontWeight="650">
        Rail total ≈ 0.55–0.65 A
      </text>
      <text x={654} y={334} fill={C.faint} fontSize="9">
        LM2596 rated 2–3 A — comfortable, but only without motors
      </text>

      {/* ---- ground star ---- */}
      <Wire d="M145 246 V 445 H 750" kind="gnd" />
      <circle cx={430} cy={445} r="5.5" fill={C.gnd} />
      <text x={430} y={464} textAnchor="middle" fill={C.dim} fontSize="10" fontWeight="600">
        common ground — battery −, buck GND, breadboard − rail, both MCUs, DRV8833, LIDAR
      </text>
      <Wire d="M750 445 V 94" kind="gnd" />
      <Wire d="M497 380 V 445" kind="gnd" />
    </svg>
  )
}

/* ========================================================================== */
/* What NOT to do — motors on the logic rail                                   */
/* ========================================================================== */
export function BadPowerDiagram() {
  return (
    <svg viewBox="0 0 860 250" style={{ width: '100%', height: 'auto' }} role="img"
      aria-label="Incorrect wiring: motors powered from the same 5 V breadboard rail as the Arduino, NodeMCU and LIDAR, causing brownouts.">
      <rect x={2} y={2} width={856} height={246} rx="8" fill="none"
        stroke={C.vbat} strokeWidth="2" strokeDasharray="8 5" />
      <text x={430} y={30} textAnchor="middle" fill={C.vbat} fontSize="13" fontWeight="700">
        ✕ DO NOT WIRE IT THIS WAY
      </text>

      <Board x={30} y={95} w={120} h={62} title="2S LiPo" subtitle="7.4 V" />
      <Wire d="M150 126 H 210" kind="vbat" />
      <Board x={210} y={95} w={125} h={62} title="LM2596" subtitle="5 V" />

      <Wire d="M335 126 H 400" kind="v5" />
      <rect x={400} y={70} width={46} height={112} rx="5" fill={C.board}
        stroke={C.v5} strokeWidth="1.5" strokeDasharray="6 3" />
      <text x={423} y={130} textAnchor="middle" fill={C.dim} fontSize="9" fontWeight="600"
        transform="rotate(-90 423 130)">BREADBOARD</text>

      {[
        { y: 82, name: 'Arduino', ok: true },
        { y: 110, name: 'NodeMCU', ok: true },
        { y: 138, name: 'RPLIDAR', ok: true },
        { y: 168, name: 'MOTORS ← the problem', ok: false },
      ].map((item) => (
        <g key={item.name}>
          <Wire d={`M446 ${item.y} H 560`} kind="v5" />
          <text x={568} y={item.y + 4} fill={item.ok ? C.dim : C.vbat}
            fontSize="11" fontWeight={item.ok ? 400 : 700}>
            {item.name}
          </text>
        </g>
      ))}

      <text x={430} y={212} textAnchor="middle" fill={C.vbat} fontSize="10.5" fontWeight="650">
        A motor stalling pulls ~1.5 A and drags the shared rail below 3 V
      </text>
      <text x={430} y={230} textAnchor="middle" fill={C.faint} fontSize="9.5">
        Result: Arduino and NodeMCU reboot mid-drive, LIDAR loses sync, robot runs away until the watchdog catches it
      </text>
    </svg>
  )
}

/* ========================================================================== */
/* Arduino ↔ DRV8833 ↔ motors — §3                                             */
/* ========================================================================== */
export function MotorDiagram() {
  const rows = [
    { pin: 'D6', label: 'AIN1', y: 96, kind: 'signal' },
    { pin: 'D11', label: 'AIN2', y: 124, kind: 'signal' },
    { pin: 'D10', label: 'BIN1', y: 152, kind: 'signal' },
    { pin: 'D9', label: 'BIN2', y: 180, kind: 'signal' },
    { pin: 'D8', label: 'nSLEEP', y: 208, kind: 'signal' },
  ]
  const encoders = [
    { pin: 'D2', label: 'Enc L A', y: 250 },
    { pin: 'D4', label: 'Enc L B', y: 274 },
    { pin: 'D3', label: 'Enc R A', y: 298 },
    { pin: 'D5', label: 'Enc R B', y: 322 },
  ]

  return (
    <svg viewBox="0 0 860 400" style={{ width: '100%', height: 'auto' }} role="img"
      aria-label="Arduino Uno R4 motor and encoder wiring to the DRV8833 and two N20 motors.">

      <Board x={30} y={70} w={170} h={280} title="Arduino Uno R4" subtitle="WiFi" />

      {rows.map((row) => (
        <g key={row.pin}>
          <Pin x={200} y={row.y} label={row.pin} align="right" color={C.signal} />
          <Wire d={`M200 ${row.y} H 380`} kind="signal" />
        </g>
      ))}

      {encoders.map((enc) => (
        <g key={enc.pin}>
          <Pin x={200} y={enc.y} label={enc.pin} align="right" color={C.encoder} />
          <Wire d={`M200 ${enc.y} H 330 V ${enc.y < 290 ? 372 : 386}`} kind="encoder" />
        </g>
      ))}

      <Board x={380} y={70} w={160} h={180} title="DRV8833" subtitle="dual H-bridge" />
      {rows.map((row) => (
        <Pin key={row.pin} x={380} y={row.y} label={row.label} align="left" color={C.signal} />
      ))}

      <text x={460} y={272} textAnchor="middle" fill={C.vbat} fontSize="9.5"
        fontFamily="ui-monospace, monospace">
        VM ← raw battery
      </text>
      <text x={460} y={286} textAnchor="middle" fill={C.v5} fontSize="9.5"
        fontFamily="ui-monospace, monospace">
        VCC ← 5 V rail
      </text>

      {/* motors */}
      {[
        { y: 110, name: 'Motor L', out: 'OUT1 / OUT2' },
        { y: 190, name: 'Motor R', out: 'OUT3 / OUT4' },
      ].map((motor) => (
        <g key={motor.name}>
          <Wire d={`M540 ${motor.y} H 640`} kind="signal" />
          <text x={588} y={motor.y - 7} textAnchor="middle" fill={C.faint} fontSize="9"
            fontFamily="ui-monospace, monospace">
            {motor.out}
          </text>
          <rect x={640} y={motor.y - 30} width={190} height={60} rx="6"
            fill={C.board} stroke={C.boardEdge} strokeWidth="1.5" />
          <text x={735} y={motor.y - 8} textAnchor="middle" fill={C.text} fontSize="12" fontWeight="600">
            {motor.name}
          </text>
          <text x={735} y={motor.y + 8} textAnchor="middle" fill={C.faint} fontSize="9.5">
            N20 6 V · quadrature encoder
          </text>
          <text x={735} y={motor.y + 22} textAnchor="middle" fill={C.encoder} fontSize="9">
            encoder: 5 V, GND, A, B
          </text>
        </g>
      ))}

      {/* encoder bus back to motors */}
      <Wire d="M330 372 H 735 V 232" kind="encoder" />
      <Wire d="M330 386 H 700" kind="encoder" />
      <text x={500} y={366} fill={C.encoder} fontSize="9.5" fontFamily="ui-monospace, monospace">
        encoder channels back to Arduino (interrupts on D2 / D3 only)
      </text>
    </svg>
  )
}

/* ========================================================================== */
/* NodeMCU ↔ RPLIDAR — §3                                                      */
/* ========================================================================== */
export function LidarDiagram() {
  return (
    <svg viewBox="0 0 860 380" style={{ width: '100%', height: 'auto' }} role="img"
      aria-label="NodeMCU ESP8266 connected to an RPLIDAR A1M8 over the hardware UART, with 7-pin core and motor connections explicitly shown.">

      <Board x={60} y={70} w={190} h={240} title="NodeMCU" subtitle="ESP8266 — LIDAR relay only" />
      <Pin x={250} y={110} label="GPIO3 (RX0)" align="right" color={C.uart} />
      <Pin x={250} y={145} label="GPIO1 (TX0)" align="right" color={C.uart} />
      <Pin x={250} y={180} label="VIN (5V)" align="right" color={C.v5} />
      <Pin x={250} y={205} label="GND" align="right" color={C.gnd} />

      {/* crossed UART — the classic mistake, so make the cross visually obvious */}
      <Wire d="M250 110 C 350 110, 400 145, 500 145" kind="uart" />
      <Wire d="M250 145 C 350 145, 400 110, 500 110" kind="uart" />
      <text x={375} y={92} textAnchor="middle" fill={C.uart} fontSize="10" fontWeight="650">
        TX → RX, RX → TX (crossed)
      </text>
      <text x={375} y={178} textAnchor="middle" fill={C.faint} fontSize="9.5"
        fontFamily="ui-monospace, monospace">
        115200 baud 8N1
      </text>

      {/* Core Power */}
      <Wire d="M250 180 H 500" kind="v5" />
      <Wire d="M250 205 H 500" kind="gnd" />

      {/* Motor Power & Control */}
      <Wire d="M460 180 V 235 H 500" kind="v5" />
      <Wire d="M460 235 V 285 H 500" kind="signal" /> 
      <circle cx={460} cy={180} r="3" fill={C.v5} />
      <circle cx={460} cy={235} r="3" fill={C.v5} />
      
      <Wire d="M480 205 V 260 H 500" kind="gnd" />
      <circle cx={480} cy={205} r="3" fill={C.gnd} />

      <Board x={500} y={70} w={200} h={240} title="RPLIDAR A1M8" subtitle="360° scanner (7 pins)" />
      
      {/* Core Interface */}
      <Pin x={500} y={110} label="TX (Core)" align="left" color={C.uart} />
      <Pin x={500} y={145} label="RX (Core)" align="left" color={C.uart} />
      <Pin x={500} y={180} label="5V (Core)" align="left" color={C.v5} />
      <Pin x={500} y={205} label="GND (Core)" align="left" color={C.gnd} />
      
      {/* Motor Interface */}
      <Pin x={500} y={235} label="5V (Motor)" align="left" color={C.v5} />
      <Pin x={500} y={260} label="GND (Motor)" align="left" color={C.gnd} />
      <Pin x={500} y={285} label="MOTOCTL (PWM)" align="left" color={C.signal} />

      <text x={380} y={231} textAnchor="middle" fill={C.faint} fontSize="9">
        Motor 5V + Core 5V on rail
      </text>
      <text x={380} y={256} textAnchor="middle" fill={C.faint} fontSize="9">
        Motor GND + Core GND on rail
      </text>
      <text x={370} y={281} textAnchor="middle" fill={C.faint} fontSize="9">
        MOTOCTL tied to 5V (always spin)
      </text>

      <text x={430} y={335} textAnchor="middle" fill={'#d29922'} fontSize="10.5" fontWeight="650">
        ⚠ The RPLIDAR occupies the hardware UART
      </text>
      <text x={430} y={352} textAnchor="middle" fill={C.faint} fontSize="9.5">
        USB serial debug is unusable at runtime — diagnostics go to the Logs page over WiFi instead
      </text>
      <text x={430} y={368} textAnchor="middle" fill={C.faint} fontSize="9.5">
        Unplug the LIDAR from GPIO1/GPIO3 before flashing over USB
      </text>
    </svg>
  )
}

/* ========================================================================== */
/* Whole-system block diagram — §4                                             */
/* ========================================================================== */
export function SystemDiagram() {
  const box = (x, y, w, h, title, sub, accent) => (
    <g key={title}>
      <rect x={x} y={y} width={w} height={h} rx="6" fill={C.board}
        stroke={accent ?? C.boardEdge} strokeWidth={accent ? 1.8 : 1.2} />
      <text x={x + w / 2} y={y + h / 2 - 3} textAnchor="middle" fill={C.text}
        fontSize="12" fontWeight="600">{title}</text>
      <text x={x + w / 2} y={y + h / 2 + 12} textAnchor="middle" fill={C.faint} fontSize="9.5">
        {sub}
      </text>
    </g>
  )

  const arrow = (d, label, lx, ly, color = C.signal) => (
    <g key={label}>
      <path d={d} fill="none" stroke={color} strokeWidth="1.8" markerEnd="url(#arrowhead)" />
      <text x={lx} y={ly} textAnchor="middle" fill={C.faint} fontSize="9"
        fontFamily="ui-monospace, monospace">{label}</text>
    </g>
  )

  return (
    <svg viewBox="0 0 860 420" style={{ width: '100%', height: 'auto' }} role="img"
      aria-label="System data flow from RPLIDAR through NodeMCU and the backend to ROS2, Nav2, and back to the Arduino and motors.">
      <defs>
        <marker id="arrowhead" markerWidth="8" markerHeight="6" refX="7" refY="3" orient="auto">
          <polygon points="0 0, 8 3, 0 6" fill={C.signal} />
        </marker>
      </defs>

      {box(20, 30, 130, 50, 'RPLIDAR A1M8', 'UART 115200')}
      {box(20, 130, 130, 50, 'NodeMCU', 'ESP8266')}
      {box(230, 80, 160, 60, 'FastAPI backend', 'laptop', C.accent)}
      {box(230, 200, 160, 50, 'slam_toolbox', '/map + TF')}
      {box(230, 290, 160, 50, 'Nav2', 'A* + DWB')}
      {box(480, 80, 150, 50, 'React web app', 'browser')}
      {box(480, 200, 150, 50, 'Arduino Uno R4', 'motion + odom')}
      {box(680, 200, 150, 50, 'DRV8833 → N20', 'motors')}

      {arrow('M85 80 V 130', 'UART', 108, 108, C.uart)}
      {arrow('M150 155 H 230 V 140', 'WiFi /ws/lidar', 190, 148)}
      {arrow('M310 140 V 200', '/scan', 330, 172)}
      {arrow('M310 250 V 290', '/map', 330, 274)}
      {arrow('M390 315 H 420 V 110 H 480', 'cmd_vel', 440, 176)}
      {arrow('M390 110 H 480', 'WebSocket', 435, 102)}
      {arrow('M390 100 C 430 100, 440 225, 480 225', 'WiFi /ws/motion', 448, 262)}
      {arrow('M630 225 H 680', 'PWM', 655, 218)}
      {arrow('M555 250 V 275 H 350 V 250', 'encoder odometry', 450, 289, C.encoder)}
    </svg>
  )
}
