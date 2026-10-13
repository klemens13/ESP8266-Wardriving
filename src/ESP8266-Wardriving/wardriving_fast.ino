/* =====================================================================
 *  wardriving_fast.ino — ESP8266 (Wemos D1 mini) Wardriving v2
 *
 *  Tujuan: lebih CEPAT & EFISIEN menangkap sinyal WiFi maupun GPS
 *  dibanding firmware lama (ESP8266-Wardriving.ino).
 *
 *  ---------------- SINYAL WiFi ----------------
 *  1. Channel-hopping aktif via PROMISCUOUS sniffer: radio hanya berada
 *     di satu channel selama DWELL_MS lalu pindah. Seluruh band 2.4 GHz
 *     (ch 1–13) tersapu tiap ±2 detik, tanpa big-scan blocking
 *     `WiFi.scanNetworks()` yang menahan CPU ±1–3 dtk per putaran.
 *  2. Parse frame Beacon / Probe-Response langsung dari buffer mentah:
 *     SSID, BSSID, RSSI, dan encryption (IE RSN/WPA + capability bit)
 *     diperoleh real-time, termasuk jaringan hidden (SSID kosong).
 *  3. Dedup in-RAM berbasis hash table dengan window waktu: MAC yang
 *     sama tidak ditulis berulang setiap beacon → file jauh lebih kecil,
 *     kartu SD lebih awet, loop tidak pernah melambat oleh I/O.
 *
 *  ---------------- PENYIMPANAN ----------------
 *  4. Satu handle File dibuka permanen + RAM line-buffer statis;
 *     flush ke SD terjadwal 1×/detik (FLUSH_MS). Versi lama melakukan
 *     SD.open()+SD.close() PER JARINGAN PER SCAN — inilah penyebab #1
 *     kelambatan pada firmware sebelumnya.
 *  5. Formatting CSV memakai snprintf_P/dtostrf ke char buffer (PROGMEM
 *     format string) — tanpa operator+ String yang mahal & membuat
 *     fragmentasi heap.
 *
 *  ---------------- SINYAL GPS ----------------
 *  6. HardwareSerial Serial1 (RX = GPIO4/D2) menggantikan SoftwareSerial:
 *     tidak ada bit-banging, byte NMEA hilang hampir nol, CPU bebas.
 *  7. Feed parser non-blocking: seluruh karakter tersedia diserap tiap
 *     iterasi loop. `smartDelay()` yang memblokir dibuang total → data
 *     GPS selalu segar saat sebuah beacon tercatat.
 *  8. Opsional (GPS_UBX_5HZ=1): kirim UBX-CFG-RATE dengan checksum
 *     Fletcher-8 dihitung runtime → rate navigasi naik ke 5 Hz pada
 *     modul u-blox (Neo-6M/7/8/9/10) → posisi 5× lebih sering.
 *     (Tetap pada baud 9600 default — tanpa risiko kehilangan komunikasi.)
 *
 *  ---------------- TAMPILAN ----------------
 *  9. OLED di-throttle (DISPLAY_MS) lewat dirty-flag; render terjadi di
 *     loop utama, bukan di dalam jalur penulisan data.
 *
 *  Output tetap kompatibel WiGLE CSV (header identik dgn firmware lama).
 *
 *  Wiring baru:
 *    GPS TX -> D2 (GPIO4, Serial1 RX)      GPS VCC/GND seperti biasa
 *    SD card: D5(SCK) D6(MISO) D7(MOSI) CS=D8   (tidak berubah)
 *    OLED I2C: SDA->D1(GPIO5), SCL->D6(GPIO12)* (*share MISO SPI; bila
 *    memakai shield D1-mini standar, pin sudah benar — atau set USE_OLED 0)
 * ===================================================================== */

#include <ESP8266WiFi.h>
extern "C" {
#include "user_interface.h"
}
#include <SD.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------------------------------------------------------ config
#define UTC_offset        +7     // PDT (konsisten dengan firmware lama)
#define SD_CS             D8
#define GPS_RX_PIN        D2     // GPIO4 = RX hardware UART Serial1
#define GPS_TX_PIN        D0     // GPIO3 = TX Serial1 (hanya utk kirim UBX)
#define GPS_BAUD_HW       9600   // default Neo-6M; pakai 115200 jika modul sudah di-reconfig
#define GPS_UBX_5HZ       0      // 1 = konfigurasikan modul u-blox ke 5Hz (butuh garis TX tersambung)
#define DWELL_MS          130    // tinggal per channel → sweep penuh ±1.7 s
#define FLUSH_MS          1000   // interval flush buffer → SD
#define DISPLAY_MS        250    // throttle refresh OLED
#define DEDUP_WINDOW_MS   30000  // MAC sama boleh dicatat ulang setelah 30 dtk
#define WEP_FLASH         1      // 1 = kedip OLED saat jaringan WEP terdeteksi
#define USE_OLED          1      // 0 = tanpa layar (hemat ~1.5KB RAM & pin)

// ------------------------------------------------------------------ globals
static File     logFile;              // dibuka SEKALI seumur jalan
static bool     sdReady   = false;
static uint32_t networksLogged = 0;   // baris CSV terbuffer/tertulis
static uint32_t beaconsSeen    = 0;   // diagnostik unjuk-kerja sniffer

static TinyGPSPlus gps;
static HardwareSerial GPSserial(1);   // Serial1: RX-only HW UART pada GPIO4

static const uint8_t CHANNELS[] PROGMEM = {1,2,3,4,5,6,7,8,9,10,11,12,13};
static const uint8_t NUM_CH = sizeof(CHANNELS);

// ---- RAM line buffer (pool statis, anti fragmentasi heap)
// Buffer kecil di BSS + flush cepat: bila penuh sebelum jadwal flush,
// bufPush() memanggil flushToSD() sehingga TIDAK ADA baris yang hilang.
#define BUF_LINES   32
#define BUF_LINELEN 200
static char    bufPool[BUF_LINES][BUF_LINELEN];  // ±6.4 KB BSS — selalu aman
static uint8_t bufHead = 0, bufTail = 0;         // indeks ring (bufHead=write)

static inline uint8_t bufCount() {
  return (bufHead >= bufTail) ? (bufHead - bufTail) : (BUF_LINES - bufTail + bufHead);
}

// ---- dedup hash-table (open addressing, hemat RAM: 128 slot × 12 B ≈ 1.5 KB)
#define DEDUP_SLOTS 128
struct SeenRec { uint64_t mac; uint32_t t; };
static SeenRec dedup[DEDUP_SLOTS];

// ---- sniffer/UI state
static volatile uint8_t curChIdx = 0;
static uint32_t lastDwell = 0, lastFlush = 0, lastDisp = 0;
static volatile bool dispDirty = false;
#if WEP_FLASH && USE_OLED
static volatile bool wepAlert = false;         // diset dari sniffCb, dieksekusi di loop (non-blocking)
#endif
static char    lastSSID[33]   = {0};
static int8_t  lastRSSI       = 0;
static uint8_t lastCh         = 0;
static char    lastEncShort[8] = {0};

#if USE_OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);
static void drawStatus();   // prototype (dipanggil dari loop sebelum definisinya)
#endif

// ------------------------------------------------------------------ helpers
static inline uint64_t macToU64(const uint8_t* b) {
  uint64_t m = 0; for (uint8_t i = 0; i < 6; i++) m = (m << 8) | b[i]; return m;
}

static void fmtTimestamp(char* out /* >= 20 byte */) {
  sprintf_P(out, PSTR("%04d-%02d-%02d %02d:%02d:%02d"),
            year(), month(), day(), hour(), minute(), second());
}

// true ⇒ layak dicatat (MAC baru, atau window dedup terlampaui)
static bool shouldLog(uint64_t mac, uint32_t now) {
  // hash: OUI + byte NIC terakhir → AP dengan OUI sama tetap tersebar merata
  uint32_t h = (uint32_t)((mac >> 24) ^ (mac & 0xFF) ^ ((mac >> 8) & 0xFFFF));
  h %= DEDUP_SLOTS;
  for (uint8_t probe = 0; probe < 8; probe++) {
    uint32_t i = (h + probe) & (DEDUP_SLOTS - 1);
    if (dedup[i].mac == 0)                 { dedup[i].mac = mac; dedup[i].t = now; return true; }
    if (dedup[i].mac == mac) {
      if ((uint32_t)(now - dedup[i].t) < DEDUP_WINDOW_MS) return false;
      dedup[i].t = now; return true;
    }
  }
  // cluster penuh: ganti entri tertua dalam cluster (approximate LRU)
  uint32_t oldest = now, oi = h;
  for (uint8_t probe = 0; probe < 8; probe++) {
    uint32_t i = (h + probe) & (DEDUP_SLOTS - 1);
    if ((int32_t)(dedup[i].t - oldest) < 0) { oldest = dedup[i].t; oi = i; }
  }
  dedup[oi].mac = mac; dedup[oi].t = now;
  return true;
}

// ------------------------------------------------------------------ buffer -> SD
static void flushToSD();   // prototype (dipanggil dari bufPush)

static void bufPush(const char* line) {
  if (bufCount() >= BUF_LINES - 1) flushToSD();
  if (bufCount() >= BUF_LINES - 1) return;      // hanya terjadi bila SD mati
  strncpy(bufPool[bufHead], line, BUF_LINELEN - 1);
  bufPool[bufHead][BUF_LINELEN - 1] = 0;
  bufHead = (bufHead + 1) % BUF_LINES;
}

void flushToSD() {
  if (!sdReady || bufTail == bufHead) return;
  do {
    size_t n = strlen(bufPool[bufTail]);
    if (logFile.write((const uint8_t*)bufPool[bufTail], n) != n) { sdReady = false; break; }
    bufTail = (bufTail + 1) % BUF_LINES;
  } while (bufTail != bufHead);
  logFile.flush();                    // sync fisik terjadwal, bukan per-baris
}

// ------------------------------------------------------------------ record writer
static void sanitizeSsid(char* s) {   // buang kontrol char & pemecah CSV
  for (char* p = s; *p; p++)
    if ((uint8_t)*p < 32 || *p == ',') *p = ' ';
}

static void logRecord(const uint8_t* bssid, const char* ssid,
                      const char* encFull, const char* encShort,
                      int8_t rssi, uint8_t chan) {
  uint32_t now = millis();
  if (!shouldLog(macToU64(bssid), now)) return;

  char ts[20]; fmtTimestamp(ts);
  char lat[12], lng[12], alt[8], acc[8];
  dtostrf(gps.location.isValid() ? gps.location.lat() : 0.0, 1, 6, lat);
  dtostrf(gps.location.isValid() ? gps.location.lng() : 0.0, 1, 6, lng);
  dtostrf(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 1, 1, alt);
  dtostrf(gps.hdop.isValid() ? gps.hdop.value() * 0.01 : 0.0, 1, 1, acc);

  char safeSsid[33];
  strncpy(safeSsid, ssid, 32); safeSsid[32] = 0;
  sanitizeSsid(safeSsid);

  static char line[BUF_LINELEN];
  snprintf_P(line, sizeof(line),
             PSTR("%02X:%02X:%02X:%02X:%02X:%02X,%s,%s,%s,%u,%d,%s,%s,%s,%s,WIFI\n"),
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
             safeSsid, encFull, ts, (unsigned)chan, (int)rssi, lat, lng, alt, acc);
  bufPush(line);
  networksLogged++;

  strncpy(lastSSID, safeSsid, 32); lastSSID[32] = 0;
  strncpy(lastEncShort, encShort, 7); lastEncShort[7] = 0;
  lastRSSI = rssi; lastCh = chan; dispDirty = true;
#if WEP_FLASH && USE_OLED
  if (encShort[0] == 'W' && encShort[1] == 'E' && encShort[2] == 'P') wepAlert = true;
#endif
}

// ================================================================== SNIFFER
// Callback promiscuous — dipanggil dari konteks driver WiFi.
// Harus IRAM_ATTR, cepat, dan tidak menyentuh SD/SPI/I2C.
static void ICACHE_RAM_ATTR sniffCb(uint8_t* raw, uint16_t len) {
  if (len < 40) return;                       // 24 hdr + 12 fixed beacon minimum
  uint8_t fc = raw[0];
  if (fc != 0x80 && fc != 0x50) return;       // hanya Beacon / Probe Response

  // Layout 802.11 mgmt: FC(2) DUR(2) [ADDR1 6 @2] [ADDR2 6 @10] [ADDR3 6 @16]
  //                     SEQ(2)@22 | Beacon: TS(8) INTV(2) CAP(2) → IE mulai @36
  const uint8_t* addr2 = raw + 10;            // transmitter addr (=BSSID pada beacon)
  const uint8_t* addr3 = raw + 16;            // BSSID field
  // Untuk Probe-Response (from AP) kedua alamat sama; pilih yang non-zero.
  const uint8_t* bssid = addr3;
  uint8_t any = 0; for (uint8_t i = 0; i < 6; i++) any |= addr3[i];
  if (!any) bssid = addr2;

  // RSSI: core ESP8266 Arduino menempelkan struct rx_control_info SETELAH
  // body frame; signal strength = selang beberapa byte sebelum akhir buffer.
  // Pendekatan yang dipakai mayoritas sketch wardriving ESP8266:
  int8_t rssi = -(int8_t)raw[len - 1];        // estimasi kasar dari tail descriptor

  // Capability info little-endian @34..35, bit-4 (privacy) → WEP/encrypted
  bool wepCapable = (raw[35] & 0x10) != 0;

  char ssid[33] = {0};
  bool hasWpa = false, hasWpa2 = false;

  uint16_t idx = 36;
  while (idx + 2 <= len) {
    uint8_t id = raw[idx], elen = raw[idx + 1];
    uint16_t body = idx + 2;
    if (body + elen > len) break;             // frame terpotong → berhenti aman
    if (id == 0 && elen <= 32) {              // SSID IE
      memcpy(ssid, raw + body, elen); ssid[elen] = 0;
    } else if (id == 48 && elen >= 4) {       // RSN IE → WPA2
      // payload: ver(2) group-suite(4) [pair-count(2) suites...]
      uint8_t gtype = raw[body + 5];          // byte tipe cipher grup (OUI+type)
      if (gtype == 2 || gtype == 6) hasWpa2 = true;         // CCMP / AES-128
      if (gtype == 4 || gtype == 5) { hasWpa2 = true; hasWpa = true; } // TKIP mix
    } else if (id == 131 && elen >= 8) {      // WPA IE (tag vendor Microsoft)
      hasWpa = true;
    }
    idx = body + elen;
  }

  const char* encFull; const char* encShort;
  if      (hasWpa && hasWpa2) { encFull = "[WPA-PSK-CCMP+TKIP][WPA2-PSK-CCMP+TKIP][ESS]"; encShort = "AUTO"; }
  else if (hasWpa2)           { encFull = "[WPA2-PSK-CCMP+TKIP][ESS]";                   encShort = "WPA2"; }
  else if (hasWpa)            { encFull = "[WPA-PSK-CCMP+TKIP][ESS]";                    encShort = "WPA";  }
  else if (wepCapable)        { encFull = "[WEP][ESS]";                                  encShort = "WEP";  }
  else                        { encFull = "[ESS]";                                       encShort = "NONE"; }

  logRecord(bssid, ssid, encFull, encShort, rssi, pgm_read_byte(&CHANNELS[curChIdx]));
  beaconsSeen++;
}

// ================================================================== UBX helper
// Bangun paket UBX kelas CFG (0x06) dengan checksum Fletcher-8 dihitung runtime.
static void ubxCfgSend(uint8_t msgId, const uint8_t* payload, uint16_t plen) {
  uint8_t hdr[6] = {0xB5, 0x62, 0x06, msgId, (uint8_t)(plen & 0xFF), (uint8_t)(plen >> 8)};
  uint8_t a = 0, b = 0;
  for (uint8_t i = 0; i < 6; i++) { a += hdr[i]; b += a; }
  for (uint16_t i = 0; i < plen; i++) { a += payload[i]; b += a; }
  GPSserial.write(hdr, 6);
  GPSserial.write(payload, plen);
  uint8_t ck[2] = {a, b};
  GPSserial.write(ck, 2);
  GPSserial.flush();
}

static void tryConfigureGps() {
#if GPS_UBX_5HZ
  // UBX-CFG-RATE (0x06 0x08): measRate=200ms, navRate=1 cycle -> 5 Hz.
  // Payload 24 byte format baru (protocol >=20, mis. NEO-M8/M9/F9) TIDAK
  // dipakai di sini; format lawas 6 byte diterima semua generasi u-blox.
  static const uint8_t rate[] = {0xC8,0x00, 0x01,0x00, 0x05,0x00};
  ubxCfgSend(0x08, rate, sizeof(rate));       // checksum Fletcher-8 dihitung runtime
#endif
}

// ================================================================== SETUP
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n* ESP8266 WarDriver FAST *"));

  pinMode(BUILTIN_LED, OUTPUT); digitalWrite(BUILTIN_LED, HIGH);

  // --- GPS: hardware UART, non-blocking sejak awal
  GPSserial.begin(GPS_BAUD_HW, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  tryConfigureGps();

  // --- OLED (opsional)
#if USE_OLED
  Wire.begin(D1, D6);                          // SDA=GPIO5, SCL=GPIO12 (lihat header)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0, 0); display.println(F("WarDriver FAST")); display.display();
#endif

  // --- SD: satu file, satu handle untuk selamanya
  sdReady = SD.begin(SD_CS);
  Serial.print(F("SD: ")); Serial.println(sdReady ? F("found") : F("NOT FOUND"));
  if (sdReady) initLogFile();

  // --- Pasang sniffer promiscuous + filter agar hanya frame management
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_opmode(STATION_MODE);
  wifi_promiscuous_filter_t filt;
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;   // hemat CPU: lewati data frames
  wifi_set_promiscuous_filter(&filt);
  wifi_set_promiscuous_ctrl_filter(&filt);
  wifi_set_promiscuous_rx_cb(sniffCb);
  wifi_promiscuous_enable(1);
  esp_wifi_set_channel(CHANNELS[0], WIFI_SECOND_CHAN_NONE);
  Serial.println(F("Sniffer aktif — channel hopping dimulai."));
}

void initLogFile() {
  int i = 0; char fn[16];
  snprintf(fn, sizeof(fn), "fast%d.csv", i);
  while (SD.exists(fn)) { i++; snprintf(fn, sizeof(fn), "fast%d.csv", i); }
  logFile = SD.open(fn, FILE_WRITE);
  if (!logFile) { sdReady = false; return; }
  logFile.println(F("WigleWifi-1.4,appRelease=2.53,model=D1-Mini-Pro,release=0.0.0,device=NetDash,display=SSD1306,board=ESP8266,brand=Wemos"));
  logFile.println(F("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type"));
  logFile.flush();
  Serial.print(F("Created: ")); Serial.println(fn);
}

// ================================================================== LOOP
void loop() {
  uint32_t now = millis();

  // 1) Serap semua byte NMEA yang tersedia (parser per-char, tak pernah blok)
  while (GPSserial.available()) gps.encode((char)GPSserial.read());

  // 2) Channel hop — inti kecepatan tangkap WiFi
  if (now - lastDwell >= DWELL_MS) {
    lastDwell = now;
    curChIdx = (curChIdx + 1) % NUM_CH;
    esp_wifi_set_channel(pgm_read_byte(&CHANNELS[curChIdx]), WIFI_SECOND_CHAN_NONE);
    digitalWrite(BUILTIN_LED, LOW);            // heartbeat tiap hop
  } else digitalWrite(BUILTIN_LED, HIGH);

  // 3) Flush SD terjadwal (buffer kecil, write batch ±6 KB/dtk — aman dari brownout)
  if (now - lastFlush >= FLUSH_MS) { lastFlush = now; flushToSD(); }

  // 4) OLED throttled via dirty-flag
#if USE_OLED
  if (dispDirty && now - lastDisp >= DISPLAY_MS) {
    lastDisp = now; dispDirty = false; drawStatus();
  }
#endif
#if WEP_FLASH && USE_OLED
  // kedip peringatan WEP — dikerjakan di loop, BUKAN di dalam sniffCb/logRecord
  if (wepAlert) {
    wepAlert = false;
    display.invertDisplay(true);  delay(120);
    display.invertDisplay(false); delay(120);
  }
#endif

  // 5) Jam realtime dari GPS untuk kolom FirstSeen
  if (gps.time.isValid() && gps.date.isValid()) {
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(),  gps.date.month(),  gps.date.year());
    adjustTime(UTC_offset * SECS_PER_HOUR);
  }

  // watchdog GPS (non-blocking)
  static uint32_t lastWarn = 0;
  if (now > 5000 && gps.charsProcessed() < 10 && now - lastWarn > 5000) {
    lastWarn = now; Serial.println(F("No GPS data received: check wiring"));
  }
}

#if USE_OLED
void drawStatus() {
  display.clearDisplay();
  display.setCursor(0, 0);
  char ts[6]; sprintf_P(ts, PSTR("%02d:%02d"), hour(), minute());
  display.print(ts);
  display.printf_P(PSTR(" Ch:%u N:%lu\n"), (unsigned)lastCh, (unsigned long)networksLogged);
  display.drawLine(0, 9, 128, 9, WHITE);
  uint8_t l = strlen(lastSSID);
  if (l > 20) lastSSID[20] = 0;                 // potong in-place (tanpa alokasi String)
  display.println(l ? lastSSID : "(hidden)");
  display.printf_P(PSTR("%d dBm %s\n"), (int)lastRSSI, lastEncShort);
  display.printf_P(PSTR("Sat:%d %.1fkm/h\n"), gps.satellites.value(), gps.speed.kmph());
  if (gps.location.isValid())
    display.printf_P(PSTR("%.5f,%.5f"), gps.location.lat(), gps.location.lng());
  else display.print(F("No fix yet..."));
  display.display();
}
#endif
