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