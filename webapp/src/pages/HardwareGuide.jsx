/**
 * Hardware Connection & User Guide.
 *
 * Everything a builder needs with the robot on the bench in front of them:
 * the bill of materials, every pin connection as both a diagram and a table,
 * and the ordered bring-up procedure from bare boards to autonomous navigation.
 *
 * Two deliberate choices:
 *  - Live status is woven into the steps. The "start the backend" step shows
 *    whether the backend is actually reachable right now; the MCU steps show
 *    real connection state. A guide that can tell you whether the step worked
 *    beats one that just describes it.
 *  - Copy buttons on every command. People follow this with greasy fingers and
 *    a soldering iron nearby; retyping a uvicorn invocation is where typos come
 *    from.
 */

import { useCallback, useState } from 'react'
import { useBot } from '../lib/store.jsx'
import {
  BadPowerDiagram,
  LidarDiagram,
  MotorDiagram,
  PowerDiagram,
  SystemDiagram,
  WireLegend,
} from '../components/WiringDiagram.jsx'
import FullPinout from '../components/FullPinout.jsx'

function Copyable({ children, cmd }) {
  const [copied, setCopied] = useState(false)
  const text = cmd ?? (typeof children === 'string' ? children : '')

  const copy = useCallback(() => {
    navigator.clipboard?.writeText(text)
    setCopied(true)
    setTimeout(() => setCopied(false), 1400)
  }, [text])

  return (
    <div className="cmd">
      <code>{children}</code>
      <button className="sm" onClick={copy} title="Copy to clipboard">
        {copied ? '✓' : 'copy'}
      </button>
    </div>
  )
}

function Step({ n, title, status, children }) {
  return (
    <div className="step">
      <div className="step-n">{n}</div>
      <div className="step-body">
        <div className="step-title">
          {title}
          {status}
        </div>
        {children}
      </div>
    </div>
  )
}

function LiveChip({ ok, okText, badText, pending }) {
  if (pending) return <span className="chip">checking…</span>
  return (
    <span className={`chip ${ok ? 'ok' : 'warn'}`}>
      <span className={`dot ${ok ? 'ok' : 'warn'}`} />
      {ok ? okText : badText}
    </span>
  )
}

const BOM = [
  ['Battery', '2S LiPo, 7.4 V nominal / 8.4 V full charge', '1'],
  ['Buck converter', 'LM2596 (adjustable)', '1'],
  ['Motor driver', 'DRV8833 dual H-bridge', '1'],
  ['Motors', 'N20, 6 V, with quadrature encoders', '2'],
  ['LIDAR', 'RPLIDAR A1M8', '1'],
  ['Main controller', 'Arduino Uno R4 WiFi', '1'],
  ['Secondary controller', 'NodeMCU (ESP8266) — LIDAR relay only', '1'],
]

const ARDUINO_PINS = [
  ['D2', 'Motor L encoder CH-A', 'interrupt', 'encoder'],
  ['D4', 'Motor L encoder CH-B', 'read inside ISR', 'encoder'],
  ['D3', 'Motor R encoder CH-A', 'interrupt', 'encoder'],
  ['D5', 'Motor R encoder CH-B', 'read inside ISR', 'encoder'],
  ['D6', 'DRV8833 AIN1', 'PWM', 'signal'],
  ['D11', 'DRV8833 AIN2', 'PWM', 'signal'],
  ['D10', 'DRV8833 BIN1', 'PWM', 'signal'],
  ['D9', 'DRV8833 BIN2', 'PWM', 'signal'],
  ['D8', 'DRV8833 nSLEEP', 'driven HIGH at boot', 'signal'],
  ['5V', '5 V regulated rail', 'power', 'v5'],
  ['GND', 'Common ground', '—', 'gnd'],
  ['(onboard WiFi)', 'Backend /ws/motion', 'commands + odometry', 'wifi'],
]

const NODEMCU_PINS = [
  ['GPIO3 (RX0)', 'RPLIDAR TX', 'hardware UART, 115200', 'uart'],
  ['GPIO1 (TX0)', 'RPLIDAR RX', 'hardware UART', 'uart'],
  ['VIN / 5V', '5 V regulated rail', 'power', 'v5'],
  ['GND', 'Common ground', '—', 'gnd'],
  ['(onboard WiFi)', 'Backend /ws/lidar', 'scan data out', 'wifi'],
]

const DRV_PINS = [
  ['VM', 'Raw battery 7.4–8.4 V', 'within the 10.8 V max — no separate trim rail'],
  ['VCC', '5 V regulated rail', 'logic supply'],
  ['GND', 'Common ground', 'star point'],
  ['OUT1 / OUT2', 'Motor L terminals', ''],
  ['OUT3 / OUT4', 'Motor R terminals', ''],
  ['nSLEEP', 'Arduino D8', 'must be HIGH or the driver stays asleep'],
]

export default function HardwareGuide() {
  const { connected, arduino, nodemcu, status, running, tuning } = useBot()
  const rosUp = status?.ros?.available
  const hasMap = (status?.map_meta?.width ?? 0) > 0
  const [showSpecPinmap, setShowSpecPinmap] = useState(false)

  return (
    <>
      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">Hardware connection &amp; user guide</div>
        <p className="dim" style={{ marginTop: 0 }}>
          Everything needed to go from a box of parts to an autonomously mapping
          robot. Work through the sections in order — the wiring must be right
          and the 5 V rail verified before any board is powered.
        </p>
        <div className="row">
          <LiveChip ok={connected} okText="Backend reachable" badText="Backend offline" />
          <LiveChip ok={arduino?.connected} okText="Arduino online" badText="Arduino offline" />
          <LiveChip ok={nodemcu?.connected} okText="NodeMCU online" badText="NodeMCU offline" />
          <LiveChip ok={rosUp} okText="ROS2 active" badText="ROS2 not running" />
          <LiveChip ok={hasMap} okText="Map building" badText="No map yet" />
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">1 · Bill of materials</div>
        <div className="scroll-x">
          <table className="data">
            <thead>
              <tr>
                <th>Component</th>
                <th>Specification</th>
                <th>Qty</th>
              </tr>
            </thead>
            <tbody>
              {BOM.map(([name, spec, qty]) => (
                <tr key={name}>
                  <td style={{ fontWeight: 600 }}>{name}</td>
                  <td className="dim">{spec}</td>
                  <td className="num">{qty}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
        <div className="param-help" style={{ marginTop: 10 }}>
          Also needed but not strictly electronics: a chassis with a caster,
          wheels to fit the N20 shafts, a multimeter (non-negotiable — see step
          2), and a USB cable for each board.
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">2 · Power architecture — one battery, one buck</div>

        <div className="notice err">
          <strong>Do not power the motors from the 5 V breadboard rail.</strong>
          This is the one change that matters. Two N20 motors pull roughly 1.5 A
          between them at stall, and a starting motor is briefly a near short
          circuit. Put that on the same rail as the Arduino, NodeMCU and LIDAR
          and the rail collapses every time you accelerate: both boards reboot
          mid-drive, the LIDAR loses UART sync, and the robot keeps rolling until
          the watchdog catches it two seconds later. <strong>Motor power comes
          straight from the battery into DRV8833 VM.</strong> Nothing extra to
          buy — you already have that wire.
        </div>

        <div className="diagram" style={{ marginBottom: 14 }}>
          <BadPowerDiagram />
        </div>

        <div className="notice info">
          <strong>The rest of your plan is right.</strong>
          One battery, one LM2596 trimmed to 5 V, feeding a breadboard rail that
          powers the Arduino, NodeMCU and LIDAR — that works, and it is exactly
          what the diagram below shows. The DRV8833 still needs its logic pin
          (VCC) on that 5 V rail; it is only the motor supply (VM) that bypasses
          the buck.
        </div>

        <WireLegend />
        <div className="diagram">
          <PowerDiagram />
        </div>

        <div className="grid grid-2" style={{ marginTop: 14 }}>
          <div>
            <div className="sub-title">The four power rules</div>
            <ul className="rules">
              <li>
                <strong>Battery + splits two ways.</strong> One wire to the buck
                input, one wire straight to DRV8833 VM. That single split is the
                whole design.
              </li>
              <li>
                <strong>DRV8833 VM ← raw battery</strong> (7.4–8.4 V). The
                driver's limit is 10.8 V, so a 2S pack is within spec — no
                regulation needed, and the motors get their current from the
                battery directly where it belongs.
              </li>
              <li>
                <strong>DRV8833 VCC ← 5 V rail.</strong> VM is motor power, VCC
                is logic power. Two different pins doing two different jobs;
                wiring VCC to the battery will damage the driver.
              </li>
              <li>
                <strong>One star ground.</strong> Battery −, buck OUT−,
                breadboard − rail, both MCU GNDs, DRV8833 GND, LIDAR GND and both
                encoder GNDs all common. Skip this and the UART and encoders both
                misbehave in ways that look like broken hardware.
              </li>
            </ul>
          </div>
          <div>
            <div className="sub-title">Does one LM2596 cope?</div>
            <div className="scroll-x">
              <table className="data">
                <tbody>
                  <tr>
                    <td>RPLIDAR A1M8</td>
                    <td className="num">~450 mA</td>
                  </tr>
                  <tr>
                    <td>NodeMCU (WiFi peaks)</td>
                    <td className="num">~80 mA</td>
                  </tr>
                  <tr>
                    <td>Arduino Uno R4</td>
                    <td className="num">~50 mA</td>
                  </tr>
                  <tr>
                    <td>Encoders ×2 + DRV8833 logic</td>
                    <td className="num">~40 mA</td>
                  </tr>
                  <tr style={{ fontWeight: 700 }}>
                    <td>Total on the 5 V rail</td>
                    <td className="num">~620 mA</td>
                  </tr>
                </tbody>
              </table>
            </div>
            <p className="dim small" style={{ marginTop: 8 }}>
              An LM2596 module is rated 2–3 A, so ~0.6 A is comfortable —{' '}
              <em>as long as the motors are not on it</em>. Add ~1.5 A of motor
              current and you are at the module's thermal limit with no margin,
              which is the other half of why motors go on the battery.
            </p>
            <div className="notice warn" style={{ marginBottom: 0 }}>
              <strong>Fit a heatsink.</strong>
              At 0.6 A the LM2596 dissipates around 1.5 W. The bare chip gets hot
              enough to thermally throttle in an enclosed chassis, and a
              throttling regulator browns out your boards just as effectively as
              an overloaded one.
            </div>
          </div>
        </div>

        <div className="notice err" style={{ marginTop: 14 }}>
          <strong>Trim the LM2596 to 5.00 V ±0.1 V and confirm with a multimeter
          BEFORE connecting any board.</strong>
          A converter fresh out of the bag can output anything up to its input
          voltage. Battery → buck only, nothing else attached. Probe OUT+ to
          OUT−, turn the pot (multi-turn — about 15 turns, no click at the end)
          until the meter reads 5.00 V, then power down and wire everything else.
          An untrimmed converter passing 7.4 V kills the Arduino, the NodeMCU and
          the LIDAR simultaneously.
        </div>

        <div className="grid grid-2" style={{ marginTop: 14 }}>
          <div className="stat">
            <div className="stat-label">Motor over-voltage is handled in firmware</div>
            <div className="stat-value">
              {tuning?.values?.max_pwm_duty ?? 200}
              <span className="stat-unit">/ 255</span>
            </div>
            <div className="stat-sub">
              ≈{' '}
              {(((tuning?.values?.max_pwm_duty ?? 200) / 255) * 8.4).toFixed(1)} V
              average across a 6 V motor at a full 8.4 V pack
            </div>
            <p className="dim small" style={{ marginTop: 8, marginBottom: 0 }}>
              Your 6 V motors sit on a 7.4–8.4 V rail, and that is fine because
              the PWM duty cap limits the average voltage. It is a live-tunable
              runtime value on the Tuning page, not a compiled constant — raising
              it above ~200 means continuously over-volting the motors.
            </p>
          </div>
          <div className="stat">
            <div className="stat-label">Breadboard caveat</div>
            <p className="dim small" style={{ marginTop: 6, marginBottom: 8 }}>
              Breadboard contacts are good for roughly 1 A and their springs
              loosen with use. At 0.6 A across a single pair of rail contacts you
              are fine, but:
            </p>
            <ul className="rules small" style={{ marginBottom: 0 }}>
              <li>
                Feed the rail from the buck at <strong>one point</strong>, and if
                your board has split rails, bridge both halves.
              </li>
              <li>
                Put a <strong>470–1000 µF electrolytic across the 5 V rail</strong>{' '}
                if you have one spare. It absorbs the LIDAR's spin-up surge.
              </li>
              <li>
                Never run motor current through the breadboard — DRV8833 VM and
                the motor outputs should be direct wires.
              </li>
            </ul>
          </div>
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">3 · Arduino Uno R4 → DRV8833 → motors</div>

        <div className="notice warn">
          <strong>This pin map differs from the written spec, deliberately.</strong>
          On the UNO R4 WiFi hardware PWM exists only on D3, D5, D6, D9, D10 and
          D11. The spec's §3 table puts <code>AIN2</code> on D7 and{' '}
          <code>BIN1</code> on D8, which are digital-only. A DRV8833 has no
          enable pin, so in IN/IN mode the second input is what sets speed in
          reverse — with the original mapping each motor would drive
          proportionally forwards and full-speed-only backwards. Two motor lines
          moved and <code>nSLEEP</code> (which needs no PWM) took D8.
        </div>

        <div className="diagram">
          <MotorDiagram />
        </div>

        <div className="row" style={{ margin: '12px 0' }}>
          <button
            className={`sm toggle ${showSpecPinmap ? 'on' : ''}`}
            onClick={() => setShowSpecPinmap(!showSpecPinmap)}
          >
            {showSpecPinmap ? 'Hide' : 'Show'} original spec pin map
          </button>
          <span className="small faint">
            only if your harness is already crimped to it
          </span>
        </div>

        {showSpecPinmap && (
          <div className="notice">
            <strong>Using the original §3 pin map</strong>
            Uncomment <code>#define USE_SPEC_PINMAP</code> at the top of{' '}
            <code>arduino_uno_r4.ino</code> and reflash. Wiring becomes AIN1→D6,
            AIN2→D7, BIN1→D8, BIN2→D9, nSLEEP→D10, encoders on D2–D5. Motion
            works, but reverse is bang-bang: full speed or nothing.
          </div>
        )}

        <div className="scroll-x">
          <table className="data">
            <thead>
              <tr>
                <th>Arduino pin</th>
                <th>Connects to</th>
                <th>Purpose</th>
              </tr>
            </thead>
            <tbody>
              {ARDUINO_PINS.map(([pin, to, purpose, kind]) => (
                <tr key={pin}>
                  <td>
                    <span className={`pin-tag ${kind}`}>{pin}</span>
                  </td>
                  <td>{to}</td>
                  <td className="dim small">{purpose}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        <div className="sub-title" style={{ marginTop: 16 }}>DRV8833 connections</div>
        <div className="scroll-x">
          <table className="data">
            <thead>
              <tr>
                <th>DRV8833 pin</th>
                <th>Connects to</th>
                <th>Note</th>
              </tr>
            </thead>
            <tbody>
              {DRV_PINS.map(([pin, to, note]) => (
                <tr key={pin}>
                  <td>
                    <span className="pin-tag signal">{pin}</span>
                  </td>
                  <td>{to}</td>
                  <td className="dim small">{note}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        <div className="param-help" style={{ marginTop: 12 }}>
          <strong>Encoder wiring:</strong> each N20 encoder needs four wires —
          5 V, GND, CH-A and CH-B. Only CH-A carries an interrupt (D2 and D3 are
          the two pins guaranteed interrupt-capable on this board); CH-B is read
          inside the interrupt handler to determine direction. If a wheel counts
          backwards when driven forwards, swap that motor's CH-A and CH-B.
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">4 · NodeMCU → RPLIDAR A1M8</div>

        <div className="notice warn">
          <strong>The LIDAR connects to the NodeMCU only — not to the Arduino as
          well.</strong>
          A UART is a point-to-point link: two devices, one pair of wires. Wiring
          the LIDAR's TX to both boards means two receivers on one line and,
          worse, two transmitters that will fight each other and can damage a
          pin. There is also no spare hardware UART on the Uno R4 — D0/D1 are the
          USB-serial port, so taking them costs you uploads and serial debugging.
          The NodeMCU exists in this build specifically to be the LIDAR's
          dedicated UART host; it reads the scans and forwards them over WiFi, so
          the Arduino still gets the obstacle data — just via the backend rather
          than by wire.
        </div>

        <div className="diagram">
          <LidarDiagram />
        </div>

        <div className="scroll-x" style={{ marginTop: 12 }}>
          <table className="data">
            <thead>
              <tr>
                <th>NodeMCU pin</th>
                <th>Connects to</th>
                <th>Purpose</th>
              </tr>
            </thead>
            <tbody>
              {NODEMCU_PINS.map(([pin, to, purpose, kind]) => (
                <tr key={pin}>
                  <td>
                    <span className={`pin-tag ${kind}`}>{pin}</span>
                  </td>
                  <td>{to}</td>
                  <td className="dim small">{purpose}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        <div className="notice warn" style={{ marginTop: 12 }}>
          <strong>Two things that catch everyone here.</strong>
          <div style={{ marginTop: 6 }}>
            <strong>1. TX goes to RX.</strong> LIDAR TX → NodeMCU GPIO3 (RX0),
            NodeMCU GPIO1 (TX0) → LIDAR RX. Straight-through wiring gives you a
            silent LIDAR and no error message.
          </div>
          <div style={{ marginTop: 6 }}>
            <strong>2. Unplug the LIDAR before flashing over USB.</strong> The
            RPLIDAR sits on the same hardware UART as the USB-serial bridge, so
            an upload will either fail or be corrupted by scan data. For the
            same reason the serial monitor is useless at runtime — that is why
            the NodeMCU logs over WiFi to the{' '}
            <strong>Logs</strong> page instead.
          </div>
        </div>

        <div className="param-help">
          The RPLIDAR A1M8 uses a 7-pin connection: 4 pins for the core interface (5V, GND, TX, RX) and 3 pins for the motor (5V, GND, MOTOCTL). Wire both 5V pins to the breadboard 5V rail, and both GND pins to the common ground. Tie the MOTOCTL pin to 5V as well to spin the motor continuously. The LIDAR draws a real amount of current at spin-up (~400-500 mA); if the 5 V rail sags the NodeMCU can brown out. The firmware re-issues the SCAN command automatically if samples stop for 3 seconds.
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">
          5 · Complete pinout — every pin on every component
        </div>
        <p className="dim small" style={{ marginTop: 0 }}>
          Every pin is listed, including the ones you are not using, so you can
          tell a genuinely spare pin from one that was left out of the table.
          Rows marked <span className="pin-tag gnd">avoid</span> in the notes are
          pins that exist but should stay clear — boot-mode straps and the USB
          serial port.
        </p>
        <FullPinout />

        <div className="notice info">
          <strong>Wire count check.</strong>
          If you have wired all of this correctly you should have: 2 wires
          battery to split point, 2 to the buck, 2 buck to breadboard, 2 raw to
          DRV8833 VM, 4 motor winding wires, 8 encoder wires (4 per motor), 5
          Arduino-to-DRV8833 signal wires, 4 LIDAR wires, and a 5 V + GND pair to
          each of the Arduino, NodeMCU and DRV8833 VCC. Anything left over means
          something is missing.
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">6 · How the pieces talk to each other</div>
        <div className="diagram">
          <SystemDiagram />
        </div>
        <div className="param-help" style={{ marginTop: 10 }}>
          Both MCUs are WebSocket <em>clients</em> of the backend — they connect
          out to it, so the backend needs a fixed LAN IP that you put in each
          sketch's <code>secrets.h</code>. One backend process is simultaneously
          the MCU server, the browser server, a ROS2 node, the arduino-cli
          runner and the OpenRouter client.
        </div>
      </div>

      {/* ================================================================ */}
      <div className="panel">
        <div className="panel-title">7 · Movement test first (no LIDAR, no backend)</div>
        <p className="dim small" style={{ marginTop: 0 }}>
          Before wiring the NodeMCU and LIDAR, prove the drive half of the robot
          works on its own. <code>firmware/movement_test/</code> is a standalone
          sketch: the Arduino joins your phone hotspot, serves its own control
          page, and you drive it from a browser. No backend, no ROS, no
          WebSocket server, nothing else to go wrong.
        </p>

        <div className="notice" style={{ marginBottom: 12 }}>
          <strong>Wheels off the ground for the first run.</strong> A reversed
          motor is the single most common bring-up error and it is much easier
          to watch a wheel spin the wrong way than to chase a robot.
        </div>

        <Step n="A" title="Set the hotspot to 2.4 GHz">
          <p className="dim small">
            The Uno R4's radio is 2.4 GHz only and will not see a 5 GHz
            network. On Android: <em>Hotspot → AP Band → 2.4 GHz</em>. On
            iPhone: <em>Personal Hotspot → Maximise Compatibility → ON</em>.
          </p>
          <Copyable>cp firmware/movement_test/secrets.h.example firmware/movement_test/secrets.h</Copyable>
          <p className="dim small" style={{ marginBottom: 0 }}>
            Edit it with your hotspot name and password. Unlike the production
            firmware there is no <code>BACKEND_HOST</code> — the Arduino is the
            server here.
          </p>
        </Step>

        <Step n="B" title="Flash it and read the serial monitor">
          <p className="dim small">
            Open <code>firmware/movement_test/movement_test.ino</code> in the
            Arduino IDE, select <em>Arduino UNO R4 WiFi</em>, upload. Then open
            Serial Monitor at <strong>115200</strong>. The board prints the URL
            to open:
          </p>
          <Copyable>Connected.{'\n'}  Open this in your browser:  http://192.168.x.x</Copyable>
          <p className="dim small" style={{ marginBottom: 0 }}>
            If it prints <code>WiFi FAILED</code>, the hotspot is on 5 GHz or
            the credentials are wrong. The motors stay disabled either way.
          </p>
        </Step>

        <Step n="C" title="Power it from the battery, not USB">
          <p className="dim small">
            Unplug USB, then connect the battery. <strong>Do not run both at
            once</strong> once the buck converter feeds the Arduino's 5V pin —
            you would be back-feeding the USB host's regulator from the buck.
            Reconnect USB only for reflashing, with the battery disconnected.
          </p>
        </Step>

        <Step n="D" title="Drive it">
          <p className="dim small">
            Open that URL on your phone or laptop (same hotspot). Press{' '}
            <strong>ARM</strong>, then hold an arrow. Arrow keys and WASD work
            on a laptop; space is an emergency stop. The encoder counts under
            the pad should move — if one stays at zero, that encoder is
            miswired.
          </p>
          <div className="rules" style={{ marginBottom: 0 }}>
            <div><strong>Both wheels backwards</strong> — swap the battery leads at the DRV8833? No: set both <code>INVERT_LEFT</code> and <code>INVERT_RIGHT</code> to <code>true</code> at the top of the sketch.</div>
            <div><strong>One wheel backwards</strong> — flip just that side's <code>INVERT_</code> flag. Do not re-solder; the flag exists precisely for this.</div>
            <div><strong>Turns the wrong way</strong> — your left and right motors are swapped at the driver. Flip both <code>INVERT_</code> flags back and swap OUT1/OUT2 with OUT3/OUT4 instead.</div>
            <div><strong>Buzzes but doesn't move</strong> — raise <code>START_DUTY</code> from 70 in steps of 10. That is the stiction threshold and it varies between motors.</div>
            <div><strong>Boards reboot when driving</strong> — the DRV8833 <code>VCC</code> is on the 5 V rail instead of raw battery. Section 2 covers this.</div>
          </div>
        </Step>

        <div className="notice" style={{ marginTop: 12, marginBottom: 0 }}>
          <strong>Speed is capped in firmware.</strong> Duty is limited to 120/255,
          so on a full 8.4 V pack the motors see ≈4.0 V average — well under
          their 6 V rating and deliberately slow. The production firmware raises
          this to 200 (≈6.6 V). Once this test passes, flash{' '}
          <code>arduino_uno_r4.ino</code> for the real system.
        </div>
      </div>

      {/* ================================================================ */}
      <div className="panel">
        <div className="panel-title">8 · Full bring-up procedure</div>
        <p className="dim small" style={{ marginTop: 0 }}>
          Follow these in order. Each step's live indicator tells you whether it
          actually worked before you move on.
        </p>

        <Step
          n="1"
          title="Verify the 5 V rail"
        >
          <p className="dim small">
            Battery → LM2596 only, nothing else connected. Probe the converter
            output and trim the pot until the meter reads 5.00 V ±0.1 V. Power
            down before wiring anything else.
          </p>
        </Step>

        <Step n="2" title="Fill in WiFi credentials">
          <p className="dim small">
            Copy each template and edit it. <code>secrets.h</code> is gitignored
            so credentials never get committed.
          </p>
          <Copyable>cp firmware/arduino_uno_r4/secrets.h.example firmware/arduino_uno_r4/secrets.h</Copyable>
          <Copyable>cp firmware/nodemcu_lidar/secrets.h.example firmware/nodemcu_lidar/secrets.h</Copyable>
          <div className="notice" style={{ marginTop: 8, marginBottom: 0 }}>
            <code>BACKEND_HOST</code> must be the <strong>LAN IP</strong> of the
            machine running the backend — from the MCU's point of view,{' '}
            <code>localhost</code> means the MCU itself. Find it with{' '}
            <code>ipconfig</code> (Windows) or <code>ip addr</code> (Linux). The
            ESP8266 is 2.4 GHz only, and most routers isolate a guest SSID from
            the main network.
          </div>
        </Step>

        <Step n="3" title="Flash both boards over USB, the traditional way">
          <p className="dim small">
            Do this once with the Arduino IDE before relying on the Flash
            Center — you want to know the firmware itself works before adding a
            web layer on top of it.
          </p>
          <div className="grid grid-2" style={{ gap: 12 }}>
            <div>
              <div className="sub-title">Arduino Uno R4 WiFi</div>
              <ul className="rules small">
                <li>Board manager: <code>Arduino UNO R4 Boards</code></li>
                <li>Libraries: <code>ArduinoHttpClient</code>, <code>ArduinoJson</code> v7</li>
                <li>Open <code>firmware/arduino_uno_r4/arduino_uno_r4.ino</code>, select the port, Upload</li>
              </ul>
            </div>
            <div>
              <div className="sub-title">NodeMCU (ESP8266)</div>
              <ul className="rules small">
                <li>Board manager URL: <code>http://arduino.esp8266.com/stable/package_esp8266com_index.json</code></li>
                <li>Libraries: <code>WebSockets</code> (Markus Sattler), <code>ArduinoJson</code> v7</li>
                <li><strong>Disconnect the LIDAR from GPIO1/GPIO3 first</strong></li>
              </ul>
            </div>
          </div>
        </Step>

        <Step
          n="4"
          title="Start the backend"
          status={<LiveChip ok={connected} okText="reachable" badText="not reachable" />}
        >
          <p className="dim small">
            First run only — create the virtual environment and install
            dependencies:
          </p>
          <Copyable>python -m venv .venv</Copyable>
          <Copyable>.venv/bin/pip install -r backend/requirements.txt</Copyable>
          <p className="dim small" style={{ marginTop: 10 }}>
            Then start it. <code>--host 0.0.0.0</code> matters: the MCUs connect
            in over the LAN, so binding to localhost would make the robot
            unreachable.
          </p>
          <Copyable>cd backend &amp;&amp; uvicorn main:app --host 0.0.0.0 --port 8000</Copyable>
          <p className="dim small" style={{ marginTop: 8 }}>
            Windows PowerShell:
          </p>
          <Copyable>cd backend; ..\.venv\Scripts\uvicorn.exe main:app --host 0.0.0.0 --port 8000</Copyable>
          <div className="param-help">
            Allow the port through the host firewall, or the MCUs will connect
            to nothing. <code>rclpy</code> is not in requirements.txt on purpose
            — it isn't pip-installable and comes from your ROS2 install. The
            backend runs fine without it.
          </div>
        </Step>

        <Step
          n="5"
          title="Confirm both MCUs connect over WebSocket"
          status={
            <LiveChip
              ok={arduino?.connected && nodemcu?.connected}
              okText="both connected"
              badText={
                arduino?.connected
                  ? 'NodeMCU missing'
                  : nodemcu?.connected
                    ? 'Arduino missing'
                    : 'neither connected'
              }
            />
          }
        >
          <p className="dim small">
            Power the robot. Within a few seconds both dots in the top bar
            should turn green. The MCUs reconnect automatically every 2 s, so
            you can start the backend after the robot and it will still find it.
          </p>
          <div className="scroll-x">
            <table className="data">
              <thead>
                <tr>
                  <th>Endpoint</th>
                  <th>Client</th>
                  <th>Carries</th>
                  <th>Live</th>
                </tr>
              </thead>
              <tbody>
                <tr>
                  <td><code>/ws/motion</code></td>
                  <td>Arduino Uno R4</td>
                  <td className="dim small">cmd_vel, control, tuning ⇄ odometry</td>
                  <td>
                    <span className={`dot ${arduino?.connected ? 'ok' : 'err'}`} />
                  </td>
                </tr>
                <tr>
                  <td><code>/ws/lidar</code></td>
                  <td>NodeMCU</td>
                  <td className="dim small">tuning ⇄ scan revolutions</td>
                  <td>
                    <span className={`dot ${nodemcu?.connected ? 'ok' : 'err'}`} />
                  </td>
                </tr>
                <tr>
                  <td><code>/ws/app</code></td>
                  <td>This browser</td>
                  <td className="dim small">status, scan, map, logs, commands</td>
                  <td>
                    <span className={`dot ${connected ? 'ok' : 'err'}`} />
                  </td>
                </tr>
                <tr>
                  <td><code>/ws/flash</code></td>
                  <td>Flash Center</td>
                  <td className="dim small">live compile/upload console</td>
                  <td><span className="dot" /></td>
                </tr>
              </tbody>
            </table>
          </div>
          <div className="param-help">
            Nothing connecting? Check <code>BACKEND_HOST</code> in{' '}
            <code>secrets.h</code>, confirm the backend is bound to{' '}
            <code>0.0.0.0</code>, and check the firewall. Both the robot and the
            backend must be on the same subnet.
          </div>
        </Step>

        <Step
          n="6"
          title="Start ROS2 (mapping and autonomy)"
          status={<LiveChip ok={rosUp} okText="active" badText="not running" />}
        >
          <p className="dim small">
            Linux with ROS2 Humble or newer, plus <code>slam_toolbox</code>,{' '}
            <code>nav2_bringup</code> and <code>nav2_smac_planner</code>. Build
            once:
          </p>
          <Copyable>cd ros2_ws &amp;&amp; colcon build --symlink-install</Copyable>
          <Copyable>source install/setup.bash</Copyable>
          <p className="dim small" style={{ marginTop: 10 }}>
            Then launch slam_toolbox + Nav2:
          </p>
          <Copyable>ros2 launch slam_bot_bringup bringup.launch.py</Copyable>
          <p className="dim small" style={{ marginTop: 10 }}>
            Mapping only, no autonomous navigation:
          </p>
          <Copyable>ros2 launch slam_bot_bringup bringup.launch.py use_nav2:=false</Copyable>
          <p className="dim small" style={{ marginTop: 10 }}>
            If your LIDAR is not exactly at the robot's centre, pass the measured
            offset (§7 defaults to 0,0,0):
          </p>
          <Copyable>ros2 launch slam_bot_bringup bringup.launch.py laser_offset_x:=0.04 laser_offset_z:=0.06</Copyable>
          <div className="param-help">
            Verify data is flowing with <code>ros2 topic hz /scan</code>. Empty
            means the backend isn't publishing — so either the bridge is down or
            the NodeMCU is offline. Everything except <code>/map</code> and Nav2
            goals works without ROS2 at all.
          </div>
        </Step>

        <Step n="7" title="Start the web app">
          <p className="dim small">Development server with hot reload:</p>
          <Copyable>cd webapp &amp;&amp; npm install &amp;&amp; npm run dev</Copyable>
          <p className="dim small" style={{ marginTop: 10 }}>
            Or build once and let the backend serve it from a single origin (no
            proxy, works from a phone on the same network):
          </p>
          <Copyable>cd webapp &amp;&amp; npm run build</Copyable>
          <div className="param-help">
            Dev server runs on <code>:5173</code> and proxies <code>/api</code>{' '}
            and <code>/ws</code> to <code>:8000</code>. The built version is
            served by the backend at <code>:8000</code> directly.
          </div>
        </Step>

        <Step
          n="8"
          title="Verify the safety gates — do not skip this"
          status={
            <span className={`chip ${running ? 'warn' : 'ok'}`}>
              currently {running ? 'RUNNING' : 'stopped'}
            </span>
          }
        >
          <div className="notice warn">
            <strong>Put the robot on a stand with its wheels off the ground for
            this.</strong>
          </div>
          <ol className="rules small">
            <li>
              Press <strong>Start</strong> on the Dashboard, then drive with the
              arrow keys — the wheels should turn.
            </li>
            <li>
              Press <strong>Stop</strong>. The wheels must halt immediately and
              stay halted even while you keep pressing the drive keys.
            </li>
            <li>
              Press <strong>Start</strong> again, get the wheels moving, then{' '}
              <strong>cut WiFi power to the robot</strong> (or kill the backend).
              The wheels must stop on their own within 2 seconds — that is the
              firmware watchdog, and it is what protects you when the link dies
              mid-run.
            </li>
            <li>
              Hold something in front of the LIDAR closer than{' '}
              <code>{tuning?.values?.collision_stop_distance_mm ?? 100} mm</code>{' '}
              and confirm forward motion is blocked while reverse still works.
            </li>
          </ol>
          <div className="param-help">
            If any of these fail, stop and fix it before putting the robot on the
            floor. The browser is never a safety layer — all three gates live in
            firmware precisely so they survive a crashed backend or a dead WiFi
            link.
          </div>
        </Step>

        <Step n="9" title="Measure your three odometry values">
          <p className="dim small">
            The shipped values are placeholders. Odometry will drift and the map
            will smear until you replace them with real measurements. They are
            flagged with a <span className="badge measure">measure</span> badge
            on the Tuning page.
          </p>
          <div className="scroll-x">
            <table className="data">
              <thead>
                <tr>
                  <th>Parameter</th>
                  <th>Current</th>
                  <th>How to measure</th>
                </tr>
              </thead>
              <tbody>
                <tr>
                  <td><code>encoder_cpr</code></td>
                  <td className="num">{tuning?.values?.encoder_cpr ?? '—'}</td>
                  <td className="dim small">
                    Mark a wheel, roll exactly 10 full turns by hand, read the
                    tick delta on the Dashboard, divide by 10. Includes gearbox
                    ratio and the firmware's 2× decode.
                  </td>
                </tr>
                <tr>
                  <td><code>wheel_diameter_mm</code></td>
                  <td className="num">{tuning?.values?.wheel_diameter_mm ?? '—'}</td>
                  <td className="dim small">
                    Calipers, with the robot's weight on the wheel — tyres
                    compress under load.
                  </td>
                </tr>
                <tr>
                  <td><code>wheel_base_mm</code></td>
                  <td className="num">{tuning?.values?.wheel_base_mm ?? '—'}</td>
                  <td className="dim small">
                    Centre-to-centre between the two wheels' contact patches.
                    Errors here show up as rotational drift — the map rotates
                    away from reality on every turn.
                  </td>
                </tr>
              </tbody>
            </table>
          </div>
          <div className="param-help">
            Enter them on the Tuning page, verify by driving a measured 1 m
            straight line and checking the reported pose, then press{' '}
            <strong>Save as default</strong> to write them to MCU flash.
          </div>
        </Step>

        <Step n="10" title="Drive manually, then hand over to Nav2">
          <p className="dim small">
            Drive the whole space by hand first and watch the map build on the
            Live Map page. Only once manual driving and collision-stop are
            trustworthy should you switch control mode to{' '}
            <strong>Nav2 autonomous</strong> and click a goal on the map.
          </p>
          <div className="param-help">
            From here nearly every adjustment belongs on the Tuning page, which
            applies live over WiFi with no reflash. You should rarely need the
            Flash Center again.
          </div>
        </Step>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">9 · Troubleshooting</div>
        <div className="scroll-x">
          <table className="data">
            <thead>
              <tr>
                <th>Symptom</th>
                <th>Most likely cause</th>
              </tr>
            </thead>
            <tbody>
              {[
                ['Boards reboot the moment motors start',
                  'Motor current is on the 5 V logic rail. Move DRV8833 VM to the raw battery. This is the single most common wiring mistake on a single-buck build.'],
                ['LIDAR loses sync when driving',
                  'Same cause — rail sag. Also add a 470–1000 µF capacitor across the 5 V rail to absorb the LIDAR spin-up surge.'],
                ['Buck converter gets very hot',
                  'Either the motors are on it, or it has no heatsink. At ~0.6 A of logic load it needs one; with motors added it will thermally throttle and brown out your boards.'],
                ['MCU never connects',
                  'BACKEND_HOST is not the backend\'s LAN IP, backend not bound to 0.0.0.0, host firewall blocking :8000, or the ESP8266 is on a 5 GHz / guest SSID it cannot reach.'],
                ['No LIDAR data at all',
                  'TX/RX not crossed. Check LIDAR TX → GPIO3 and GPIO1 → LIDAR RX. Also confirm the LIDAR motor is actually spinning.'],
                ['LIDAR drops out intermittently',
                  'Brown-out on the shared 5 V rail at spin-up. The firmware re-issues SCAN after 3 s and logs a warning — check the Logs page.'],
                ['Motors buzz but do not turn',
                  'max_pwm_duty too low to overcome stiction, or nSLEEP is not HIGH. Verify nSLEEP is on D8 (or D10 with the spec pin map).'],
                ['One motor runs backwards',
                  'Swap that motor\'s two output wires at the DRV8833.'],
                ['Robot arcs when told to go straight',
                  'Encoder direction inverted on one side — swap that encoder\'s CH-A and CH-B.'],
                ['Map smears or rotates away on turns',
                  'wheel_base_mm is wrong. Measure it properly; this is the single most common cause of bad SLAM on a home-built base.'],
                ['Map drifts on straight runs',
                  'wheel_diameter_mm or encoder_cpr is wrong. Drive a measured 1 m and compare against the reported pose.'],
                ['Start button refuses',
                  'The Arduino is not connected — the backend will not enable motors it cannot command. An e-stop latch also blocks it until you press Start again.'],
                ['Bot stops every ~2 seconds',
                  'Watchdog tripping from a flaky WiFi link. Check RSSI on the Dashboard; the firmware halts on purpose when the link is unreliable.'],
                ['arduino-cli not found',
                  'Install it, then arduino-cli core install arduino:renesas_uno. Set SLAM_ARDUINO_CLI to the full path if it is not on PATH.'],
                ['NodeMCU flash button does nothing',
                  'WebSerial only exists in Chrome, Edge and Opera on desktop. Firefox, Safari and all iOS browsers do not implement it.'],
                ['No /map appears',
                  'ROS2 or slam_toolbox is not running. Check ros2 topic hz /scan — if empty, the bridge is down or the NodeMCU is offline.'],
              ].map(([symptom, cause]) => (
                <tr key={symptom}>
                  <td style={{ fontWeight: 600, minWidth: 200 }}>{symptom}</td>
                  <td className="dim small">{cause}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>

      {/* ---------------------------------------------------------------- */}
      <div className="panel">
        <div className="panel-title">10 · What needs a reflash, and what doesn't</div>
        <div className="grid grid-2">
          <div>
            <div className="sub-title" style={{ color: 'var(--ok)' }}>
              Live over WiFi — no reflash
            </div>
            <ul className="rules small">
              <li>Every parameter on the Tuning page</li>
              <li>Speed caps, PWM duty cap, collision distance</li>
              <li>LIDAR range filtering and angle masks</li>
              <li>Odometry geometry and PID gains</li>
              <li>Nav2 inflation radius and velocity limits</li>
              <li>Start / Stop / e-stop</li>
            </ul>
          </div>
          <div>
            <div className="sub-title" style={{ color: 'var(--warn)' }}>
              Requires a reflash
            </div>
            <ul className="rules small">
              <li>Pin assignments</li>
              <li>Wiring-dependent constants (which pin is which encoder channel)</li>
              <li>WebSocket message schema changes</li>
              <li>WiFi credentials or backend IP (<code>secrets.h</code>)</li>
            </ul>
          </div>
        </div>
        <div className="param-help" style={{ marginTop: 10 }}>
          This split is the whole point of the runtime config struct: every
          tunable value lives in RAM on the MCU and is patched in place by a{' '}
          <code>tuning_update</code> message, taking effect on the very next
          control-loop iteration.
        </div>
      </div>
    </>
  )
}
