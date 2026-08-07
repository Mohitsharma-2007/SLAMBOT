/**
 * Browser-based ESP8266 flashing via WebSerial + esptool-js — §6.4.
 *
 * This path involves no backend at all: the user plugs the NodeMCU into the
 * machine running the *browser*, grants serial access, and the .bin is written
 * from JavaScript. That's what makes it work when the browsing device and the
 * backend host are different machines.
 *
 * esptool-js is loaded dynamically so a browser without WebSerial (Firefox,
 * Safari, any iOS browser) doesn't pay for the bundle and gets a clear message
 * instead of a cryptic failure.
 */

export function webSerialSupported() {
  return typeof navigator !== 'undefined' && 'serial' in navigator
}

/**
 * NodeMCU v2 (ESP-12E) has 4 MB of flash. DIO at 40 MHz is the conservative
 * mode that works on every clone; QIO would be faster but bricks some boards.
 */
export const FLASH_DEFAULTS = {
  baudrate: 460800,
  flashSize: 'keep',
  flashMode: 'keep',
  flashFreq: 'keep',
  eraseAll: false,
  compress: true,
  address: 0x0000,
}

class Transport {
  // Placeholder so the dynamic import's shape is obvious at the call site.
}

/**
 * Connect to a serial device the user picks, and return an esptool loader.
 *
 * @param {(message: string) => void} log
 * @param {number} baudrate
 */
export async function connectEsp(log, baudrate = FLASH_DEFAULTS.baudrate) {
  if (!webSerialSupported()) {
    throw new Error(
      'This browser does not support WebSerial. Use Chrome, Edge or Opera on ' +
        'desktop — Firefox and Safari do not implement it. (You can still flash ' +
        'the NodeMCU with esptool.py or the Arduino IDE.)',
    )
  }

  const { ESPLoader, Transport: SerialTransport } = await import('esptool-js')

  const port = await navigator.serial.requestPort()
  const transport = new SerialTransport(port)

  const terminal = {
    clean() {},
    writeLine(data) {
      log(String(data))
    },
    write(data) {
      log(String(data))
    },
  }

  const loader = new ESPLoader({
    transport,
    baudrate,
    terminal,
    romBaudrate: 115200,
  })

  const chip = await loader.main()
  log(`Detected chip: ${chip}`)

  return { loader, transport, chip }
}

/**
 * Write one binary at an offset, reporting progress as a 0..100 percentage.
 *
 * @param {object} loader          from connectEsp
 * @param {ArrayBuffer} binary     firmware image
 * @param {(pct: number) => void} onProgress
 * @param {(message: string) => void} log
 * @param {object} options
 */
export async function flashBinary(
  loader,
  binary,
  onProgress,
  log,
  options = {},
) {
  const {
    address = FLASH_DEFAULTS.address,
    eraseAll = FLASH_DEFAULTS.eraseAll,
    compress = FLASH_DEFAULTS.compress,
  } = options

  // esptool-js takes the image as a binary *string*, not an ArrayBuffer.
  const bytes = new Uint8Array(binary)
  let data = ''
  const CHUNK = 0x8000
  for (let i = 0; i < bytes.length; i += CHUNK) {
    data += String.fromCharCode.apply(
      null,
      bytes.subarray(i, Math.min(i + CHUNK, bytes.length)),
    )
  }

  log(`Writing ${bytes.length} bytes at 0x${address.toString(16)}…`)

  await loader.writeFlash({
    fileArray: [{ data, address }],
    flashSize: FLASH_DEFAULTS.flashSize,
    flashMode: FLASH_DEFAULTS.flashMode,
    flashFreq: FLASH_DEFAULTS.flashFreq,
    eraseAll,
    compress,
    reportProgress: (_fileIndex, written, total) => {
      if (total > 0) onProgress(Math.round((written / total) * 100))
    },
  })

  log('Write complete.')
}

/** Reset the board so the new firmware runs, then release the port. */
export async function finishAndReset(loader, transport, log) {
  try {
    await loader.after()
    log('Board reset — new firmware is running.')
  } catch (error) {
    log(`Reset failed (harmless; power-cycle the board): ${error.message}`)
  }
  try {
    await transport.disconnect()
  } catch {
    // Port already gone.
  }
}

/** Fetch the backend-compiled .bin so the user need not hunt for a file. */
export async function fetchCompiledBinary(target = 'nodemcu') {
  const response = await fetch(`/api/flash/artifact/${target}`)
  if (!response.ok) {
    let detail = `${response.status} ${response.statusText}`
    try {
      const payload = await response.json()
      if (payload?.detail) detail = payload.detail
    } catch {
      // Non-JSON error body.
    }
    throw new Error(detail)
  }
  return response.arrayBuffer()
}

export { Transport }
