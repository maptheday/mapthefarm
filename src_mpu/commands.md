# ESP MPU Reference

## Commands

### Full rebuild (use when changing HTML/CSS/JS)
```bash
rm data/*.gz && gzip -k data/index.html data/style.css data/script.js && pio run --target erase && pio run --target uploadfs && pio run --target upload && pio device monitor
```

### Firmware only (use when changing main.cpp)
```bash
pio run --target upload && pio device monitor
```

---

## How flash memory works

The ESP32's flash chip is just a long strip of bytes with numbered addresses, from `0x00000000` to `0x00FFFFFF` (16MB). Every byte has an address. Nothing more.

The **partition table** (`default_16MB.csv`) is a map that divides that strip into named regions:

| Name | Address | What it stores |
|------|---------|----------------|
| `nvs` | `0x9000` | WiFi credentials, settings |
| `otadata` | `0xe000` | Which firmware to boot |
| `app0` / `app1` | `0x10000` | Your compiled firmware |
| `spiffs` | `0xC90000` | Your HTML/CSS/JS files |
| `coredump` | `0xFF0000` | Crash logs |

If your board definition says 8MB but your chip is 16MB, the partition addresses don't match — the firmware looks for files at `0xC90000` but `uploadfs` wrote them to `0x670000`. LittleFS finds garbage and reports corruption.

---

## What uploadfs does

`uploadfs` takes everything in your `data/` folder, packs it into a binary image, and writes it to flash at the `spiffs` partition address. It's like copying files to a USB drive, except you have to copy them to exactly the right address or nothing works.

---

## What LittleFS does

Raw flash is just bytes — the chip has no concept of files or folders. LittleFS is a filesystem (like NTFS on Windows) that organizes those bytes into named files with directories. It's designed for microcontrollers because it's crash-safe and efficient on small flash chips.

`uploadfs` writes the bytes. LittleFS is what makes those bytes readable as `index.html`, `style.css`, etc.

---

## Why we gzip the files

`ESPAsyncWebServer` automatically looks for `.gz` versions of files first (e.g. `index.html.gz` before `index.html`). If it finds a `.gz` it serves it compressed, which is faster over WiFi. If you don't gzip, it logs errors looking for `.gz` files — the page still loads but you get noise in the serial monitor.

Always `rm data/*.gz` before re-gzipping so stale compressed files don't get uploaded.


----


pio run                    # compile firmware

-------

### Find the ESP32 serial port (macOS)

Unplug and reconnect the ESP32, then list the serial devices:

```bash
ls /dev/cu.*
pio device list
```

Use the port that appears when the ESP32 is connected, usually a name such as
`/dev/cu.usbmodem14201`. Prefer `/dev/cu.*` on macOS. If no ESP32 port appears,
check the USB cable, board connection, and USB permissions.

Set the port once in the shell so the upload and HIL commands use the same
device. Replace the example value whenever macOS assigns a different port:

```bash
export ESP_PORT=/dev/cu.usbmodem14201

pio run -e wokwi_sim --target clean
pio run -e wokwi_sim --target upload --upload-port "$ESP_PORT"

python3 simulate/hil_runner.py \
  --port "$ESP_PORT" \
  --scenario simulate/scenarios/edge_geofence_breach.py
```

The `wokwi_sim` environment has a default port in `platformio.ini`, but the
`--upload-port` option above overrides it when the port changes. To inspect the
current default, run:

```bash
grep -E '^(monitor_port|upload_port)' platformio.ini
```

HIL tests do not build or flash the firmware. If the runner connects but fails
before takeoff, upload the current `wokwi_sim` firmware first, then rerun the
scenario. A failed upload can leave an older firmware image on the board.