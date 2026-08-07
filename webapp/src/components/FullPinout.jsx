/**
 * Complete pin-by-pin reference for every component.
 *
 * Lists *every* pin on each board, not only the ones in use — because "what is
 * D7 doing?" is a question you ask while staring at a half-wired robot, and an
 * incomplete table cannot answer it. Unused pins are marked as free so you know
 * they are genuinely spare rather than accidentally omitted.
 *
 * Rows are colour-coded by signal class and additionally carry a text status,
 * so the table stays usable in greyscale or for a colour-blind builder.
 */

const USED = 'used'
const FREE = 'free'
const AVOID = 'avoid'

function PinTable({ title, subtitle, columns, rows }) {
  return (
    <div style={{ marginBottom: 22 }}>
      <div className="sub-title" style={{ fontSize: 13, color: 'var(--text)' }}>
        {title}
      </div>
      {subtitle && (
        <div className="small faint" style={{ marginBottom: 8 }}>{subtitle}</div>
      )}
      <div className="scroll-x">
        <table className="data pinout">
          <thead>
            <tr>
              {columns.map((col) => (
                <th key={col}>{col}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            {rows.map((row, i) => (
              <tr key={i} className={`pin-row ${row.status}`}>
                <td>
                  <span className={`pin-tag ${row.kind ?? 'gnd'}`}>{row.pin}</span>
                </td>
                <td style={{ fontWeight: row.status === USED ? 600 : 400 }}>
                  {row.to}
                </td>
                <td className="dim small">{row.note}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  )
}

const COLS = ['Pin', 'Connects to', 'Notes']

/* -------------------------------------------------------------------------- */
const ARDUINO = [
  { pin: 'D0 / RX', to: '— free', note: 'USB serial. Leave clear for uploads and debug printing.', kind: 'uart', status: AVOID },
  { pin: 'D1 / TX', to: '— free', note: 'USB serial. Same reason.', kind: 'uart', status: AVOID },
  { pin: 'D2', to: 'Motor L encoder CH-A', note: 'Hardware interrupt. Yellow wire on most N20 encoders.', kind: 'encoder', status: USED },
  { pin: 'D3', to: 'Motor R encoder CH-A', note: 'Hardware interrupt. The second of only two guaranteed IRQ pins.', kind: 'encoder', status: USED },
  { pin: 'D4', to: 'Motor L encoder CH-B', note: 'Read inside the ISR to get direction. No interrupt needed.', kind: 'encoder', status: USED },
  { pin: 'D5', to: 'Motor R encoder CH-B', note: 'Read inside the ISR.', kind: 'encoder', status: USED },
  { pin: 'D6', to: 'DRV8833 AIN1', note: 'PWM — left motor forward.', kind: 'signal', status: USED },
  { pin: 'D7', to: '— free', note: 'Not a PWM pin on the R4, which is why AIN2 is not here.', kind: 'signal', status: FREE },
  { pin: 'D8', to: 'DRV8833 nSLEEP', note: 'Plain digital HIGH. Driver stays asleep if this floats.', kind: 'signal', status: USED },
  { pin: 'D9', to: 'DRV8833 BIN2', note: 'PWM — right motor reverse.', kind: 'signal', status: USED },
  { pin: 'D10', to: 'DRV8833 BIN1', note: 'PWM — right motor forward.', kind: 'signal', status: USED },
  { pin: 'D11', to: 'DRV8833 AIN2', note: 'PWM — left motor reverse.', kind: 'signal', status: USED },
  { pin: 'D12', to: '— free', note: 'Spare. Good spot for a status LED or a bump switch.', kind: 'signal', status: FREE },
  { pin: 'D13', to: '— free', note: 'Onboard LED is on this pin.', kind: 'signal', status: FREE },
  { pin: 'A0', to: '— free (optional battery sense)', note: 'Fit a divider here and set BATTERY_SENSE_ENABLED to read pack voltage.', kind: 'signal', status: FREE },
  { pin: 'A1–A5', to: '— free', note: 'Spare analog inputs.', kind: 'signal', status: FREE },
  { pin: '5V', to: 'Breadboard 5 V rail', note: 'INPUT here — the buck powers the board. Do not also plug in USB while battery-powered.', kind: 'v5', status: USED },
  { pin: 'GND', to: 'Breadboard − rail', note: 'At least one; use two if you have spare rows.', kind: 'gnd', status: USED },
  { pin: 'VIN', to: '— leave empty', note: 'Do NOT feed 7.4 V here as well as 5 V into the 5V pin.', kind: 'vbat', status: AVOID },
  { pin: '3.3V', to: '— free', note: 'Output. Nothing in this build needs it.', kind: 'v5', status: FREE },
  { pin: 'onboard WiFi', to: 'Backend /ws/motion', note: 'Commands in, odometry out. Set in secrets.h.', kind: 'wifi', status: USED },
]

const NODEMCU = [
  { pin: 'GPIO3 / RX0 (D9)', to: 'RPLIDAR TX', note: 'Hardware UART receive, 115200 8N1. This is where scan data arrives.', kind: 'uart', status: USED },
  { pin: 'GPIO1 / TX0 (D10)', to: 'RPLIDAR RX', note: 'Hardware UART transmit — sends the SCAN / STOP commands.', kind: 'uart', status: USED },
  { pin: 'VIN', to: 'Breadboard 5 V rail', note: 'Onboard regulator drops this to 3.3 V. Feed 5 V here, never 3.3 V.', kind: 'v5', status: USED },
  { pin: 'GND', to: 'Breadboard − rail', note: 'Must share ground with the LIDAR or UART will not work.', kind: 'gnd', status: USED },
  { pin: '3V3', to: '— free', note: 'Output. Not used here.', kind: 'v5', status: FREE },
  { pin: 'D0 (GPIO16)', to: '— free', note: 'No interrupt support. Wake pin.', kind: 'signal', status: FREE },
  { pin: 'D1 (GPIO5)', to: '— free', note: 'Spare. Best free pin if you add I²C later.', kind: 'signal', status: FREE },
  { pin: 'D2 (GPIO4)', to: '— free', note: 'Spare.', kind: 'signal', status: FREE },
  { pin: 'D3 (GPIO0)', to: '— leave clear', note: 'Boot mode pin. Pulling it low at reset enters flash mode.', kind: 'signal', status: AVOID },
  { pin: 'D4 (GPIO2)', to: '— leave clear', note: 'Boot mode pin, also the onboard LED. Must be HIGH at boot.', kind: 'signal', status: AVOID },
  { pin: 'D5–D7', to: '— free', note: 'SPI pins, spare in this build.', kind: 'signal', status: FREE },
  { pin: 'D8 (GPIO15)', to: '— leave clear', note: 'Boot mode pin. Must be LOW at boot.', kind: 'signal', status: AVOID },
  { pin: 'A0', to: '— free', note: 'Single ADC, 0–1 V range on a bare ESP8266 (0–3.3 V on most NodeMCU boards).', kind: 'signal', status: FREE },
  { pin: 'onboard WiFi', to: 'Backend /ws/lidar', note: 'Filtered scan revolutions out, tuning in.', kind: 'wifi', status: USED },
]

const DRV8833 = [
  { pin: 'VM', to: 'Battery + (raw 7.4–8.4 V)', note: 'MOTOR power. Straight from the battery, NOT from the buck.', kind: 'vbat', status: USED },
  { pin: 'VCC', to: 'Breadboard 5 V rail', note: 'LOGIC power only, a couple of mA. Different pin, different job.', kind: 'v5', status: USED },
  { pin: 'GND', to: 'Breadboard − rail', note: 'Both GND pins if the board has two.', kind: 'gnd', status: USED },
  { pin: 'AIN1', to: 'Arduino D6', note: 'PWM.', kind: 'signal', status: USED },
  { pin: 'AIN2', to: 'Arduino D11', note: 'PWM. Sets reverse speed for the left motor.', kind: 'signal', status: USED },
  { pin: 'BIN1', to: 'Arduino D10', note: 'PWM.', kind: 'signal', status: USED },
  { pin: 'BIN2', to: 'Arduino D9', note: 'PWM. Sets reverse speed for the right motor.', kind: 'signal', status: USED },
  { pin: 'nSLEEP', to: 'Arduino D8', note: 'HIGH = awake. Some breakout boards tie this high already — check yours.', kind: 'signal', status: USED },
  { pin: 'nFAULT', to: '— free', note: 'Open-drain fault output. Optional: wire to a spare pin to detect over-current.', kind: 'signal', status: FREE },
  { pin: 'AOUT1 / AOUT2', to: 'Left motor terminals', note: 'Swap these two if the wheel spins the wrong way.', kind: 'vbat', status: USED },
  { pin: 'BOUT1 / BOUT2', to: 'Right motor terminals', note: 'Swap to reverse.', kind: 'vbat', status: USED },
]

const MOTOR = [
  { pin: 'M1 (motor +)', to: 'DRV8833 AOUT1 / BOUT1', note: 'Motor winding. Carries the full motor current.', kind: 'vbat', status: USED },
  { pin: 'M2 (motor −)', to: 'DRV8833 AOUT2 / BOUT2', note: 'Motor winding.', kind: 'vbat', status: USED },
  { pin: 'VCC (encoder)', to: 'Breadboard 5 V rail', note: 'Encoder logic supply — separate from the motor winding. Usually the red wire.', kind: 'v5', status: USED },
  { pin: 'GND (encoder)', to: 'Breadboard − rail', note: 'Usually black.', kind: 'gnd', status: USED },
  { pin: 'C1 / A', to: 'Arduino D2 (left) / D3 (right)', note: 'Hall channel A. Usually yellow.', kind: 'encoder', status: USED },
  { pin: 'C2 / B', to: 'Arduino D4 (left) / D5 (right)', note: 'Hall channel B. Usually green. Swap A and B to flip counting direction.', kind: 'encoder', status: USED },
]

// The A1M8 has 7 pins across TWO separate connectors. They are not
// interchangeable: the 4-pin core socket carries the UART and logic supply,
// the 3-pin motor socket carries the spindle supply. Each has its own GND.
const LIDAR = [
  // ---- 4-pin CORE connector (the scanner electronics + UART) ----
  { pin: 'CORE 1 — V5.0', to: 'Breadboard 5 V rail', note: 'Logic supply for the scanner head. ~100 mA. Regulated 5 V only, never raw battery.', kind: 'v5', status: USED },
  { pin: 'CORE 2 — TX', to: 'NodeMCU GPIO3 (RX0 / D9)', note: 'Scan data out of the LIDAR. Crossed: LIDAR TX goes to NodeMCU RX. 115200 8N1.', kind: 'uart', status: USED },
  { pin: 'CORE 3 — RX', to: 'NodeMCU GPIO1 (TX0 / D10)', note: 'Commands into the LIDAR (SCAN / STOP / RESET). Crossed: LIDAR RX to NodeMCU TX.', kind: 'uart', status: USED },
  { pin: 'CORE 4 — GND', to: 'Breadboard − rail', note: 'Signal ground. The UART will not work without this shared with the NodeMCU.', kind: 'gnd', status: USED },
  // ---- 3-pin MOTOR connector (the spindle that spins the head) ----
  { pin: 'MOTOR 1 — VMOTO', to: 'Breadboard 5 V rail', note: 'Spindle motor supply. The single biggest load on the rail: ~350–450 mA running, higher at spin-up.', kind: 'v5', status: USED },
  { pin: 'MOTOR 2 — GND', to: 'Breadboard − rail', note: 'Spindle ground. Separate wire from the core GND, same rail.', kind: 'gnd', status: USED },
  { pin: 'MOTOR 3 — MOTOCTL', to: 'Breadboard 5 V rail', note: 'Spindle enable / PWM speed input. Tied high to 5 V = spins continuously at full speed. This is what your diagram does and it is correct.', kind: 'signal', status: USED },
]

const BUCK = [
  { pin: 'IN+', to: 'Battery + (7.4 V)', note: 'Through a switch and ideally a 3 A fuse.', kind: 'vbat', status: USED },
  { pin: 'IN−', to: 'Battery −', note: '', kind: 'gnd', status: USED },
  { pin: 'OUT+', to: 'Breadboard + rail', note: 'Trim to 5.00 V with the pot BEFORE connecting this wire.', kind: 'v5', status: USED },
  { pin: 'OUT−', to: 'Breadboard − rail', note: 'Ties the whole ground system together.', kind: 'gnd', status: USED },
  { pin: 'Trim pot', to: '— adjustment screw', note: 'Multi-turn. Roughly 15 turns end to end; it does not click when it stops.', kind: 'signal', status: USED },
]

export default function FullPinout() {
  return (
    <>
      <PinTable
        title="Arduino Uno R4 WiFi"
        subtitle="Main controller — motor PWM, encoder counting, odometry, WiFi link to the backend"
        columns={COLS}
        rows={ARDUINO}
      />
      <PinTable
        title="NodeMCU (ESP8266)"
        subtitle="LIDAR relay only — reads the RPLIDAR UART and forwards filtered scans over WiFi"
        columns={COLS}
        rows={NODEMCU}
      />
      <PinTable
        title="DRV8833 dual H-bridge"
        subtitle="Motor driver — note that VM and VCC come from two different sources"
        columns={COLS}
        rows={DRV8833}
      />
      <PinTable
        title="N20 6 V motor with quadrature encoder (×2)"
        subtitle="Six wires per motor: two for the winding, four for the encoder"
        columns={COLS}
        rows={MOTOR}
      />
      <PinTable
        title="RPLIDAR A1M8 — 7 pins across 2 connectors"
        subtitle="4-pin core socket (UART + logic) and 3-pin motor socket (spindle). Connects to the NodeMCU only — the Arduino's UART is needed for USB uploads."
        columns={COLS}
        rows={LIDAR}
      />
      <PinTable
        title="LM2596 buck converter"
        subtitle="The only regulator in the system. Powers logic; motors bypass it."
        columns={COLS}
        rows={BUCK}
      />
    </>
  )
}
