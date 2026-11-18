# ESP8266 Wardriving
A compilation of scripts and resources for wardriving on the [ESP8266 WiFi microcontroller](https://www.espressif.com/en/products/socs/esp8266), including data visualization tools in Jupyter Notebook.

Documentation on this page is still in development.

## Components
Any ESP8266-based board should work with the basic required components, but the D1 mini form factor is highly recommended since using modules (such as for SD logging + battery management) can be done using plug-and-play hardware in a small footprint. All linked components are D1 mini compatible. 

**Required Components:**
| Component | Purpose |
| --- | --- |
| ESP8266 | Gather WiFi data & control hardware modules
| SD Reader | Store data that we can analyze with WiGLE / Python |
| GPS Module | Grab geolocation data + timestamp |

**Optional Components:**
| Component | Purpose | 
| --- | --- |
| 128x64 OLED | Get a visual display of WiFi data being captured |
| LiPo Battery | Power your ESP8266 for portable applications |
| Battery Module | Manage power for your ESP8266 w/ a LiPo battery |
| 100K Ω Resistor | Optional to read in battery level w/ the D1 Mini |
   
## Hardware Setup

**SD Reader Module** 
| SD Reader Pin | ESP8266 GPIO | D1 Mini Pin |
| --- | --- | --- |
| MISO | GPIO12 | D6 |
| MOSI | GPIO13 | D7 |
| SCK | GPIO14 | D5 |
| CS | GPIO15 | D8 |

**GPS Module (firmware lama — `ESP8266-Wardriving.ino`)** 
| GPS Pin | ESP8266 GPIO | D1 Mini Pin |
| --- | --- | --- |
| TX | GPIO2 | D4 |
| RX | GPIO0 | D3 |

**GPS Module (firmware cepat — `wardriving_fast.ino`)**

Firmware v2 memakai **hardware UART Serial1** (bukan SoftwareSerial) agar byte NMEA tidak pernah hilang dan CPU bebas untuk scanning:

| GPS Pin | ESP8266 GPIO | D1 Mini Pin |
| --- | --- | --- |
| TX | GPIO4 (Serial1 RX) | D2 |
| RX (opsional, utk konfigurasi UBX) | GPIO3 (Serial1 TX) | D0 |

> Catatan: karena GPS pindah ke D2/D0, wiring OLED I2C pada shield standar
> (SDA=D1, SCL=D2) perlu disesuaikan — gunakan pin SCL lain atau set
> `USE_OLED 0` pada sketch bila tidak memakai layar.

## Firmware Cepat: `wardriving_fast.ino`

Dikembangkan dari `ESP8266-Wardriving.ino` dengan target **menangkap sinyal WiFi & GPS lebih cepat dan efisien**. Output tetap kompatibel WiGLE CSV (header identik), sehingga seluruh script visualisasi lama dapat dipakai tanpa perubahan.

**WiFi — channel-hopping sniffer (pengganti `WiFi.scanNetworks()` blocking):**
- Radio berpindah channel 1→13 setiap `DWELL_MS` (default 130 ms → satu sweep penuh ±1,7 detik). Versi lama terjebak big-scan blocking 1–3 detik per putaran.
- Frame *Beacon* / *Probe-Response* diparsing langsung di RAM: SSID, BSSID, RSSI, dan encryption (IE RSN/WPA + capability bit) real-time — termasuk jaringan hidden.
- Promiscuous filter dibatasi ke frame management saja (`WIFI_PROMIS_FILTER_MASK_MGMT`) sehingga CPU tidak terbebani data frame.
- Dedup hash-table in-RAM dengan window 30 detik (`DEDUP_WINDOW_MS`): MAC yang sama tidak ditulis berulang tiap beacon → file jauh lebih kecil dan SD lebih awet.

**Penyimpanan — penyebab #1 kelambatan versi lama diperbaiki:**
- Versi lama melakukan `SD.open()` + `SD.close()` **per jaringan per scan**. Versi baru membuka satu handle file permanen dengan ring-buffer statis di RAM; flush terjadwal sekali per detik (`FLUSH_MS`).
- Formatting CSV memakai `snprintf_P`/`dtostrf` (format string di PROGMEM) — tanpa operator `+ String` yang mahal dan memfragmentasi heap.

**GPS — feed non-blocking:**
- `smartDelay()` yang memblokir dihapus total; parser TinyGPS++ disuapi setiap iterasi loop sehingga posisi selalu segar saat sebuah beacon dicatat.
- Opsional: set `GPS_UBX_5HZ 1` untuk mengirim UBX-CFG-RATE (checksum Fletcher-8 dihitung runtime) ke modul u-blox → rate navigasi naik ke 5 Hz (posisi 5× lebih sering).

**Parameter tunable di bagian atas sketch:**
| Konstanta | Default | Fungsi |
| --- | --- | --- |
| `DWELL_MS` | 130 | Waktu tinggal per channel (kecil = sweep lebih cepat) |
| `FLUSH_MS` | 1000 | Interval flush buffer ke SD |
| `DEDUP_WINDOW_MS` | 30000 | Jarak waktu minimum pencatatan ulang satu MAC |
| `GPS_BAUD_HW` | 9600 | Baud hardware UART GPS (sesuaikan bila modul sudah di-reconfig) |
| `USE_OLED` / `WEP_FLASH` | 1 | Tampilan & kedip peringatan WEP |

## Data Visualization Scripts

Data Visualization Tutorial: https://www.youtube.com/watch?v=pFHUPs51CRQ
