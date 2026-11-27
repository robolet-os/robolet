// robolet test firmware for ESP32-S3 (N16R8): board check, YDLIDAR X4 check, on-chip compute benchmark,
// optional motor-base link check. Run ./test.sh, or open the serial monitor at 115200 and type a command letter + Enter.
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include "config.h"
#include "x4_parser.h"
#include "bench_core.h"
#include "x4_sim.h"
#include "nav_core.h"
#include "mission_core.h"
#include "nav_nodes.h"
#include "mission_nodes.h"
#include "real_nodes.h"
#include "bridge.h"
#include "hal/usb_serial_jtag_ll.h"
#include <esp_system.h>
#include <esp_attr.h>

#define FW_VERSION "robolet-fw 0.16 (2026-11-27)"
#ifndef PIO_PLATFORM_VERSION
#define PIO_PLATFORM_VERSION "unknown"
#endif

// Console on both USB ports: the chip's own USB port (Serial, USB CDC) and the USB-UART port (Serial0).
// Output goes to both and input is taken from either, so it does not matter which port the cable is in.
class DualConsole : public Stream {
public:
  void begin(unsigned long baud) {
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxBufferSize(16384);   // keep the core's 100 ms TX timeout: with 0, lines could be dropped silently
    Serial.setRxBufferSize(16384);   // Gazebo runs: about 4 KB from the PC per cycle; the USB driver drops what does not fit
#endif
    Serial.begin(baud);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial0.setRxBufferSize(8192);
    Serial0.setTxBufferSize(4096);
    Serial0.begin(baud);
#endif
  }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial0.write(b, n);
#endif
    return n;
  }
  int available() override {
    int n = Serial.available();
#if ARDUINO_USB_CDC_ON_BOOT
    n += Serial0.available();
#endif
    return n;
  }
  int last_port = 0;                // port of the last byte read: 0 chip USB, 1 USB-UART (the PC link answers there)
  int read() override {
    if (Serial.available()) { last_port = 0; return Serial.read(); }
#if ARDUINO_USB_CDC_ON_BOOT
    if (Serial0.available()) { last_port = 1; return Serial0.read(); }
#endif
    return -1;
  }
  int peek() override {
    if (Serial.available()) return Serial.peek();
#if ARDUINO_USB_CDC_ON_BOOT
    if (Serial0.available()) return Serial0.peek();
#endif
    return -1;
  }
  void flush() override { Serial.flush(); }
};
static DualConsole Con;

static String line_in; static bool line_junk = false;
static bool read_line(String& out) {               // non-blocking line reader for the console
  while (Con.available()) {
    char c = (char)Con.read();
    if (c == '\r') continue;
    if (c == '\n') {                              // a line with binary bytes (link frames left over from a PC tool): ignored
      bool junk = line_junk; out = line_in; line_in = ""; line_junk = false; out.trim();
      if (junk) continue;
      return true;
    }
    if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) line_junk = true;
    if (line_in.length() < 64) line_in += c;
  }
  return false;
}

// =============================================================================== restart diagnostics
// The last stage of a mission (world, seed, cycle, ...) is kept in RTC memory, which survives a restart (but not a power
// cycle). After an unexpected restart, the board check prints why the chip restarted and that stage.
RTC_NOINIT_ATTR static char rtc_stage[96];
RTC_NOINIT_ATTR static uint32_t rtc_magic;
static const uint32_t RTC_MAGIC = 0xC0DE5AFEu;
static void stage_note(const char* s) { strncpy(rtc_stage, s, sizeof rtc_stage - 1); rtc_stage[sizeof rtc_stage - 1] = 0; rtc_magic = RTC_MAGIC; }
static const char* reset_name(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power on"; case ESP_RST_EXT: return "reset pin"; case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "CRASH (exception or abort)"; case ESP_RST_INT_WDT: return "CRASH (interrupt watchdog)";
    case ESP_RST_TASK_WDT: return "CRASH (task watchdog)"; case ESP_RST_WDT: return "CRASH (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "deep sleep"; case ESP_RST_BROWNOUT: return "BROWNOUT (power supply)"; case ESP_RST_SDIO: return "SDIO";
    default: return "unknown";
  }
}

// =============================================================================== board check
static void test_board() {
  Con.println("\n=== [b] board check ===");
  Con.printf("firmware %s, built %s %s | PlatformIO espressif32 %s, Arduino core %d.%d.%d, ESP-IDF %s\n", FW_VERSION, __DATE__, __TIME__,
             PIO_PLATFORM_VERSION, ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH, esp_get_idf_version());
  Con.printf("chip %s rev %d, %d cores, CPU %u MHz\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
  esp_reset_reason_t rr = esp_reset_reason();
  Con.printf("last restart: %s", reset_name(rr));
  if (rr != ESP_RST_POWERON && rtc_magic == RTC_MAGIC) Con.printf(" | last stage before it: %s", rtc_stage);
  Con.println();
  if (rr == ESP_RST_POWERON) rtc_magic = 0;
  uint32_t flash = ESP.getFlashChipSize(), ps = psramFound() ? ESP.getPsramSize() : 0;
  Con.printf("flash %u MB | PSRAM %s, %u KB (free %u KB) | internal heap free %u KB, largest block %u KB\n",
                flash >> 20, psramFound() ? "found" : "NOT FOUND", ps >> 10, ESP.getFreePsram() >> 10,
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >> 10, heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >> 10);
  bool ok = flash >= (15u << 20) && ps >= (7u << 20);
  if (ps) {                                          // write / read back 4 MB of PSRAM with a pattern
    const uint32_t N = (4u << 20) / 4;
    uint32_t* m = (uint32_t*)heap_caps_malloc(N * 4, MALLOC_CAP_SPIRAM);
    if (!m) { Con.println("PSRAM 4 MB allocation FAILED"); ok = false; }
    else {
      int64_t t0 = esp_timer_get_time();
      for (uint32_t i = 0; i < N; i++) m[i] = i * 2654435761u;
      int64_t t1 = esp_timer_get_time(); uint32_t bad = 0;
      for (uint32_t i = 0; i < N; i++) if (m[i] != i * 2654435761u) bad++;
      int64_t t2 = esp_timer_get_time();
      heap_caps_free(m);
      Con.printf("PSRAM 4 MB pattern test: %u errors, write %.1f MB/s, read+check %.1f MB/s\n",
                    bad, 4.0 / ((t1 - t0) / 1e6), 4.0 / ((t2 - t1) / 1e6));
      if (bad) ok = false;
    }
  }
  Con.printf("RESULT board: %s (expects flash >= 16 MB and PSRAM >= 8 MB for N16R8)\n", ok ? "PASS" : "CHECK");
}

// =============================================================================== lidar
HardwareSerial LidarSerial(1);
struct ScanStats {
  static const int MAXP = 1200;
  X4Point cur[MAXP]; int ncur = 0;              // points of the revolution being collected
  X4Point last[MAXP]; int nlast = 0;            // last complete revolution
  uint32_t scans = 0, pts_total = 0, pts_valid = 0;
  uint32_t t_first = 0, t_last = 0;
  volatile bool new_scan = false;
} SCN;

static void on_packet(const X4Point* p, int n, bool start, void*) {
  if (start) {                                   // zero packet: previous revolution is complete
    if (SCN.ncur > 100) {
      memcpy(SCN.last, SCN.cur, sizeof(X4Point) * SCN.ncur); SCN.nlast = SCN.ncur;
      uint32_t now = millis(); if (SCN.scans == 0) SCN.t_first = now; SCN.t_last = now; SCN.scans++;
      SCN.pts_total += SCN.ncur; for (int i = 0; i < SCN.ncur; i++) if (SCN.cur[i].dist_mm > 0) SCN.pts_valid++;
      SCN.new_scan = true;
    }
    SCN.ncur = 0;
  }
  for (int i = 0; i < n && SCN.ncur < ScanStats::MAXP; i++) SCN.cur[SCN.ncur++] = p[i];
}
static X4Parser parser(on_packet, nullptr);
static uint32_t lidar_bytes = 0;

static void lidar_pins_init() {
  if (LIDAR_M_EN_PIN >= 0) { pinMode(LIDAR_M_EN_PIN, OUTPUT); digitalWrite(LIDAR_M_EN_PIN, LOW); }
  if (LIDAR_DEV_EN_PIN >= 0) { pinMode(LIDAR_DEV_EN_PIN, OUTPUT); digitalWrite(LIDAR_DEV_EN_PIN, LOW); }
  if (LIDAR_M_SCTP_PIN >= 0) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(LIDAR_M_SCTP_PIN, 10000, 8); ledcWrite(LIDAR_M_SCTP_PIN, 255);
#else
    ledcSetup(0, 10000, 8); ledcAttachPin(LIDAR_M_SCTP_PIN, 0); ledcWrite(0, 255);
#endif
  }
  LidarSerial.setRxBufferSize(8192);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
}
static void sctp_duty(int pct) {
  if (LIDAR_M_SCTP_PIN < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LIDAR_M_SCTP_PIN, pct * 255 / 100);
#else
  ledcWrite(0, pct * 255 / 100);
#endif
}
static void lidar_start() {
  if (LIDAR_DEV_EN_PIN >= 0) digitalWrite(LIDAR_DEV_EN_PIN, HIGH);
  sctp_duty(LIDAR_M_SCTP_DUTY);
  if (LIDAR_M_EN_PIN >= 0) digitalWrite(LIDAR_M_EN_PIN, HIGH);
  delay(300);
  const uint8_t stop[2] = {0xA5, 0x65}, start[2] = {0xA5, 0x60};
  LidarSerial.write(stop, 2); delay(50);
  while (LidarSerial.available()) LidarSerial.read();
  parser.reset(); parser.packets_ok = parser.packets_bad_cs = parser.bytes_skipped = 0;
  SCN.scans = SCN.pts_total = SCN.pts_valid = 0; SCN.ncur = SCN.nlast = 0; lidar_bytes = 0;
  LidarSerial.write(start, 2);
}
static void lidar_stop() {
  const uint8_t stop[2] = {0xA5, 0x65};
  LidarSerial.write(stop, 2);
  if (LIDAR_M_EN_PIN >= 0) digitalWrite(LIDAR_M_EN_PIN, LOW);
  sctp_duty(100);
}
static void lidar_poll() {
  uint8_t buf[256];
  int n;
  while ((n = LidarSerial.available()) > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = LidarSerial.readBytes(buf, n); lidar_bytes += n; parser.feed(buf, n);
  }
}
// nearest valid range within +-tol degrees of a lidar angle (X4 angles are clockwise from the lidar's 0 mark)
static float range_at(float deg, float tol = 4.0f) {
  float best = 0;
  for (int i = 0; i < SCN.nlast; i++) {
    float d = fabsf(SCN.last[i].angle_deg - deg); if (d > 180) d = 360 - d;
    if (d <= tol && SCN.last[i].dist_mm > 0 && (best == 0 || SCN.last[i].dist_mm < best)) best = SCN.last[i].dist_mm;
  }
  return best;
}
static void print_scan_line() {
  int valid = 0; float nd = 1e9, na = 0;
  for (int i = 0; i < SCN.nlast; i++) if (SCN.last[i].dist_mm > 0) { valid++; if (SCN.last[i].dist_mm < nd) { nd = SCN.last[i].dist_mm; na = SCN.last[i].angle_deg; } }
  Con.printf("scan %u: %d pts (%d valid) | nearest %.0f mm at %.1f deg | 0deg %.0f, 90deg %.0f, 180deg %.0f, 270deg %.0f mm\n",
                SCN.scans, SCN.nlast, valid, valid ? nd : 0.0f, na, range_at(0), range_at(90), range_at(180), range_at(270));
}

static void test_lidar(uint32_t seconds) {
  Con.printf("\n=== [l] YDLIDAR X4 check, %u s (RX=%d TX=%d M_EN=%d DEV_EN=%d M_SCTP=%d) ===\n",
                seconds, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_M_EN_PIN, LIDAR_DEV_EN_PIN, LIDAR_M_SCTP_PIN);
  lidar_start();
  uint32_t t0 = millis(), next = t0 + 1000;
  while (millis() - t0 < seconds * 1000) {
    lidar_poll();
    if (millis() >= next) { next += 1000; Con.printf("  t=%2us bytes %u packets ok %u bad %u scans %u\n", (millis() - t0) / 1000, lidar_bytes, parser.packets_ok, parser.packets_bad_cs, SCN.scans); }
    delay(2);
  }
  lidar_stop();
  float hz = SCN.scans > 1 ? (SCN.scans - 1) * 1000.0f / (SCN.t_last - SCN.t_first) : 0;
  float bad = parser.packets_ok + parser.packets_bad_cs ? 100.0f * parser.packets_bad_cs / (parser.packets_ok + parser.packets_bad_cs) : 0;
  float valid = SCN.pts_total ? 100.0f * SCN.pts_valid / SCN.pts_total : 0;
  Con.printf("bytes %u | packets ok %u, checksum errors %u (%.1f %%), skipped bytes %u\n", lidar_bytes, parser.packets_ok, parser.packets_bad_cs, bad, parser.bytes_skipped);
  Con.printf("revolutions %u, %.2f Hz | points per revolution %.0f | valid points %.1f %%\n", SCN.scans, hz, SCN.scans ? (float)SCN.pts_total / SCN.scans : 0.0f, valid);
  if (SCN.nlast) print_scan_line();
  bool ok = SCN.scans >= 3 && hz > 4 && hz < 14 && bad < 5 && valid > 30;
  Con.printf("RESULT lidar: %s\n", ok ? "PASS" : "FAIL");
  if (lidar_bytes == 0)
    Con.println("  no bytes: check X4 Tx -> LIDAR_RX_PIN, X4 Rx -> LIDAR_TX_PIN, common GND, 5 V supply able to give 0.5 A, DEV_EN/M_EN high");
  else if (parser.packets_ok == 0)
    Con.println("  bytes but no valid packets: check baud 128000 and that Tx/Rx are not swapped");
  else if (SCN.scans < 3)
    Con.println("  packets but no revolutions: motor not turning? check M_EN high and M_SCTP duty (lower = faster)");
}

static void stream_lidar() {
  Con.println("\n=== [s] lidar stream: one line per revolution, press Enter to stop ===");
  lidar_start();
  while (!Con.available()) { lidar_poll(); if (SCN.new_scan) { SCN.new_scan = false; print_scan_line(); } delay(2); }
  while (Con.available()) Con.read();
  lidar_stop();
}

static void dump_scan() {
  Con.println("\n=== [p] one revolution as CSV (angle_deg,dist_mm) ===");
  lidar_start();
  uint32_t t0 = millis();
  while (SCN.scans < 3 && millis() - t0 < 5000) { lidar_poll(); delay(2); }
  lidar_stop();
  if (!SCN.nlast) { Con.println("no complete revolution received (run [l] for diagnosis)"); return; }
  Con.println("BEGIN_SCAN");
  for (int i = 0; i < SCN.nlast; i++) Con.printf("%.2f,%.1f\n", SCN.last[i].angle_deg, SCN.last[i].dist_mm);
  Con.println("END_SCAN");
}

// =============================================================================== compute benchmark
static uint8_t* alloc_cap(size_t n, uint32_t cap) { return (uint8_t*)heap_caps_malloc(n, cap | MALLOC_CAP_8BIT); }

struct BenchOut { double csm_fast = 0, amcl = 0, amcl_opt = 0, ray = 0, ray_opt = 0; };

static void bench_one(const char* where, uint32_t cap, BenchOut* out) {
  using namespace bc;
  uint8_t *occ = alloc_cap(CELLS, cap), *dist = alloc_cap(CELLS, cap), *lik = alloc_cap(CELLS, cap), *cost = alloc_cap(CELLS, cap);
  float* rg = (float*)heap_caps_malloc(720 * 4, MALLOC_CAP_INTERNAL);
  float* rg2 = (float*)heap_caps_malloc(720 * 4, MALLOC_CAP_INTERNAL);
  float* px = (float*)heap_caps_malloc(720 * 4, MALLOC_CAP_INTERNAL);
  float* py = (float*)heap_caps_malloc(720 * 4, MALLOC_CAP_INTERNAL);
  int32_t* idx = (int32_t*)heap_caps_malloc(720 * 4, MALLOC_CAP_INTERNAL);
  Pose* parts = (Pose*)heap_caps_malloc(600 * sizeof(Pose), MALLOC_CAP_INTERNAL);
  float* w = (float*)heap_caps_malloc(600 * 4, MALLOC_CAP_INTERNAL);
  float* w2 = (float*)heap_caps_malloc(600 * 4, MALLOC_CAP_INTERNAL);
  AmclTable* tab = (AmclTable*)heap_caps_malloc(sizeof(AmclTable), MALLOC_CAP_INTERNAL);
  if (!occ || !dist || !lik || !cost || !rg || !rg2 || !px || !py || !idx || !parts || !w || !w2 || !tab) { Con.printf("[%s] allocation failed\n", where); goto done; }
  {
    const int REP = 5;
    uint32_t mhz = ESP.getCpuFreqMHz();
    int64_t t0 = esp_timer_get_time();
    build_room(occ); distance_field(occ, dist); likelihood_field(dist, lik); costmap(dist, cost);
    int64_t t1 = esp_timer_get_time();
    Con.printf("[%s] map build (distance+likelihood+costmap, %d cells): %.2f ms\n", where, CELLS, (t1 - t0) / 1000.0);

    Pose truth = {2.0f, 2.0f, 0.3f};
    t0 = esp_timer_get_time();
    int rl = raycast(occ, truth.x, truth.y, truth.th, 720, rg);
    t1 = esp_timer_get_time();
    raycast_opt(occ, truth.x, truth.y, truth.th, 720, rg2);
    int64_t t2 = esp_timer_get_time();
    int diff = 0; for (int i = 0; i < 720; i++) if (fabsf(rg[i] - rg2[i]) > 1e-4f) diff++;
    out->ray = (t1 - t0) / 1000.0; out->ray_opt = (t2 - t1) / 1000.0;
    Con.printf("[%s] ray cast 720 beams: v1 %.2f ms -> v2 %.2f ms (%d lookups, %.0f -> %.0f cycles/lookup), %d beams differ\n",
               where, out->ray, out->ray_opt, rl, out->ray * 1000 * mhz / rl, out->ray_opt * 1000 * mhz / rl, diff);

    int n = scan_points(rg, 720, px, py);
    Pose guess = {2.05f, 1.95f, 0.33f};
    MatchResult a{}, b{};
    t0 = esp_timer_get_time();
    for (int r = 0; r < REP; r++) a = csm_naive(lik, px, py, n, guess);
    t1 = esp_timer_get_time();
    for (int r = 0; r < REP; r++) b = csm_fast(lik, px, py, n, guess, idx);
    t2 = esp_timer_get_time();
    double cn = (t1 - t0) / 1000.0 / REP, cf = (t2 - t1) / 1000.0 / REP;
    bool okn = fabsf(a.best.x - 2.0f) < 0.03f && fabsf(a.best.y - 2.0f) < 0.03f && fabsf(a.best.th - 0.3f) < 0.016f;
    bool okf = fabsf(b.best.x - 2.0f) < 0.03f && fabsf(b.best.y - 2.0f) < 0.03f && fabsf(b.best.th - 0.3f) < 0.016f;
    Con.printf("[%s] scan match naive 125 poses x %d pts: %.2f ms, %.1f cycles/lookup, pose %s\n", where, n, cn, cn * 1000 * mhz / a.lookups, okn ? "correct" : "WRONG");
    Con.printf("[%s] scan match fast  125 poses x %d pts: %.2f ms, %.1f cycles/lookup, pose %s\n", where, n, cf, cf * 1000 * mhz / b.lookups, okf ? "correct" : "WRONG");
    out->csm_fast = cf;

    srand(2);
    for (int p = 0; p < 600; p++) parts[p] = {truth.x + ((rand() % 41) - 20) * 0.01f, truth.y + ((rand() % 41) - 20) * 0.01f, truth.th + ((rand() % 21) - 10) * 0.01f};
    amcl_table(*tab);
    uint32_t al = 0;
    t0 = esp_timer_get_time();
    for (int r = 0; r < REP; r++) al = amcl_weights(lik, px, py, n, parts, 600, 60, w);
    t1 = esp_timer_get_time();
    for (int r = 0; r < REP; r++) amcl_weights_opt(lik, px, py, n, parts, 600, 60, *tab, w2);
    t2 = esp_timer_get_time();
    float dw = 0; int b1 = 0, b2 = 0;
    for (int p = 0; p < 600; p++) { float d = fabsf(w[p] - w2[p]); if (d > dw) dw = d; if (w[p] > w[b1]) b1 = p; if (w2[p] > w2[b2]) b2 = p; }
    out->amcl = (t1 - t0) / 1000.0 / REP; out->amcl_opt = (t2 - t1) / 1000.0 / REP;
    Con.printf("[%s] AMCL 600 particles x 60 beams: v1 %.2f ms -> v2 %.2f ms (%.0f -> %.0f cycles/lookup), best particle %s, max weight diff %.4f\n",
               where, out->amcl, out->amcl_opt, out->amcl * 1000 * mhz / al, out->amcl_opt * 1000 * mhz / al, b1 == b2 ? "same" : "DIFFERENT", dw);
  }
done:
  heap_caps_free(occ); heap_caps_free(dist); heap_caps_free(lik); heap_caps_free(cost);
  heap_caps_free(rg); heap_caps_free(rg2); heap_caps_free(px); heap_caps_free(py); heap_caps_free(idx);
  heap_caps_free(parts); heap_caps_free(w); heap_caps_free(w2); heap_caps_free(tab);
}

static void test_compute() {
  using namespace bc;
  Con.printf("\n=== [c] compute benchmark v2 (8 x 6 m map, 5 cm cells, CPU %u MHz, one core) ===\n", ESP.getCpuFreqMHz());
  BenchOut bi, bp;
  bench_one("SRAM ", MALLOC_CAP_INTERNAL, &bi);
  bench_one("PSRAM", MALLOC_CAP_SPIRAM, &bp);
  // A* (work arrays 12 B/cell = 384 KB -> PSRAM)
  uint8_t *occ = alloc_cap(CELLS, MALLOC_CAP_SPIRAM), *dist = alloc_cap(CELLS, MALLOC_CAP_SPIRAM), *cost = alloc_cap(CELLS, MALLOC_CAP_SPIRAM);
  AStarWork wk = {(float*)heap_caps_malloc(CELLS * 4, MALLOC_CAP_SPIRAM), (uint16_t*)heap_caps_malloc(CELLS * 2, MALLOC_CAP_SPIRAM),
                  (uint16_t*)heap_caps_malloc(CELLS * 2, MALLOC_CAP_SPIRAM), alloc_cap(CELLS, MALLOC_CAP_SPIRAM)};
  float* fbuf = (float*)heap_caps_malloc(CELLS * 4, MALLOC_CAP_SPIRAM);
  double as_ms = 0, as2_ms = 0;
  if (occ && dist && cost && wk.g && wk.heap && wk.pos && wk.state && fbuf) {
    build_room(occ); distance_field(occ, dist); costmap(dist, cost);
    int s = cy(1.0f) * W + cx(1.0f), g = cy(5.0f) * W + cx(7.0f);
    int64_t t0 = esp_timer_get_time();
    AStarResult r = astar(cost, s, g, wk);
    int64_t t1 = esp_timer_get_time();
    AStarResult r2 = astar_opt(cost, s, g, wk, fbuf);
    int64_t t2 = esp_timer_get_time();
    as_ms = (t1 - t0) / 1000.0; as2_ms = (t2 - t1) / 1000.0;
    Con.printf("[PSRAM] A* (1,1)->(7,5): v1 %.2f ms -> v2 %.2f ms | length v1 %.2f m, v2 %.2f m (%s) | expanded %d / %d cells\n",
               as_ms, as2_ms, r.length_m, r2.length_m, (r.found && r2.found && fabsf(r.length_m - r2.length_m) < 0.01f) ? "same" : "DIFFERENT", r.expanded, r2.expanded);
  } else Con.println("A* allocation failed");
  heap_caps_free(occ); heap_caps_free(dist); heap_caps_free(cost); heap_caps_free(fbuf);
  heap_caps_free(wk.g); heap_caps_free(wk.heap); heap_caps_free(wk.pos); heap_caps_free(wk.state);
  // Steady-state load with the simulator's rates: scan match at 7 scans/s, AMCL 2 updates/s, A* 1 plan/s
  double v1_i = (7 * bi.csm_fast + 2 * bi.amcl + as_ms) / 10.0, v1_p = (7 * bp.csm_fast + 2 * bp.amcl + as_ms) / 10.0;
  double v2_i = (7 * bi.csm_fast + 2 * bi.amcl_opt + as2_ms) / 10.0, v2_p = (7 * bp.csm_fast + 2 * bp.amcl_opt + as2_ms) / 10.0;
  Con.printf("load (7 scan matches/s + 2 AMCL/s + 1 A*/s), %% of one core: v1 SRAM %.1f / PSRAM %.1f  ->  v2 SRAM %.1f / PSRAM %.1f\n", v1_i, v1_p, v2_i, v2_p);
  Con.printf("  v2 breakdown (PSRAM): scan match %.1f %%, AMCL %.1f %%, A* %.1f %%\n", 7 * bp.csm_fast / 10.0, 2 * bp.amcl_opt / 10.0, as2_ms / 10.0);
  Con.println("(tools/chip_budget.py estimated about 12 % for the S3 by counting operations)");
  Con.println("RESULT compute: done (all lines must say 'pose correct', 'same', and 0 beams differ)");
}

// =============================================================================== virtual lidar
// [v] The chip generates the byte stream of a YDLIDAR X4 from the simulated room (src/x4_sim.h) for a robot
// moving along a short path, receives it through the same parser as the real sensor and tracks the pose with
// scan matching. With a jumper wire from LIDAR_TX_PIN to LIDAR_RX_PIN the bytes go through the real UART at
// 128000 bps; without it they are handed to the parser inside the chip.
static bool uart_loopback_present() {
  while (LidarSerial.available()) LidarSerial.read();
  const uint8_t probe[8] = {0x13, 0x57, 0x9B, 0xDF, 0x02, 0x46, 0x8A, 0xCE};
  LidarSerial.write(probe, 8); LidarSerial.flush();
  uint8_t got[8]; int n = 0; uint32_t t = millis();
  while (millis() - t < 50 && n < 8) { if (LidarSerial.available()) got[n++] = (uint8_t)LidarSerial.read(); }
  return n == 8 && memcmp(got, probe, 8) == 0;
}

static void test_virtual_lidar() {
  using namespace bc;
  const int N = 30;
  Con.println("\n=== [v] virtual lidar: simulated X4 byte stream -> parser -> scan matching (30 revolutions) ===");
  uint8_t *occ = alloc_cap(CELLS, MALLOC_CAP_SPIRAM), *dist = alloc_cap(CELLS, MALLOC_CAP_SPIRAM), *lik = alloc_cap(CELLS, MALLOC_CAP_INTERNAL);
  uint8_t* buf = alloc_cap(x4sim::REV_BYTES, MALLOC_CAP_INTERNAL);
  float* px = (float*)heap_caps_malloc(800 * 4, MALLOC_CAP_INTERNAL);
  float* py = (float*)heap_caps_malloc(800 * 4, MALLOC_CAP_INTERNAL);
  int32_t* idx = (int32_t*)heap_caps_malloc(800 * 4, MALLOC_CAP_INTERNAL);
  if (!occ || !dist || !lik || !buf || !px || !py || !idx) { Con.println("allocation failed"); Con.println("RESULT vlidar: FAIL"); goto done; }
  {
    build_room(occ); distance_field(occ, dist); likelihood_field(dist, lik);
    bool loop = uart_loopback_present();
    Con.printf("transport: %s\n", loop ? "UART loopback found (GPIO TX -> RX jumper): bytes go through the real UART at 128000 bps"
                                       : "no jumper found: bytes are handed to the parser inside the chip (connect TX->RX for the UART test)");
    parser.reset(); parser.packets_ok = parser.packets_bad_cs = parser.bytes_skipped = 0;
    SCN.scans = SCN.pts_total = SCN.pts_valid = 0; SCN.ncur = SCN.nlast = 0; SCN.new_scan = false; lidar_bytes = 0;
    x4sim::Rng rng{12345};
    Pose est = {1.5f + 0.04f, 1.5f - 0.03f, 0.02f};          // start guess deliberately off by 5 cm / 0.02 rad
    int matched = 0; float max_e = 0, max_eth = 0, max_wall = 0;
    double gen_ms = 0, match_ms = 0; uint32_t bytes_sent = 0;
    int64_t t_start = esp_timer_get_time();
    for (int k = 0; k <= N; k++) {
      Pose truth = {1.5f + 0.03f * k, 1.5f + 0.015f * k, 0.02f * k};
      int64_t g0 = esp_timer_get_time();
      int nb = x4sim::revolution(occ, truth, buf, rng);
      gen_ms += (esp_timer_get_time() - g0) / 1000.0;
      bytes_sent += nb;
      if (loop) {
        for (int off = 0; off < nb; off += 64) { int m = nb - off < 64 ? nb - off : 64; LidarSerial.write(buf + off, m); lidar_poll(); }
        LidarSerial.flush();
        uint32_t quiet = millis();                            // read until the line has been quiet for 5 ms
        while (millis() - quiet < 5) { uint32_t b0 = lidar_bytes; lidar_poll(); if (lidar_bytes != b0) quiet = millis(); }
      } else {
        parser.feed(buf, nb); lidar_bytes += nb;
      }
      if (SCN.new_scan && k > 0) {                            // zero packet of revolution k completed revolution k-1
        SCN.new_scan = false;
        Pose t = {1.5f + 0.03f * (k - 1), 1.5f + 0.015f * (k - 1), 0.02f * (k - 1)};
        int64_t m0 = esp_timer_get_time();
        int n = x4sim::to_robot_frame(SCN.last, SCN.nlast, px, py, 800);
        MatchResult r = csm_fast(lik, px, py, n, est, idx);
        match_ms += (esp_timer_get_time() - m0) / 1000.0;
        est = r.best;
        float wall = x4sim::mean_wall_distance(dist, t, px, py, n);
        float e = hypotf(est.x - t.x, est.y - t.y), eth = fabsf(est.th - t.th);
        if (e > max_e) max_e = e;
        if (eth > max_eth) max_eth = eth;
        if (wall > max_wall) max_wall = wall;
        matched++;
        if (k <= 2 || k == N)
          Con.printf("  rev %2d: %d pts (%d valid) | points-to-wall %.3f m | estimate (%.2f, %.2f, %.3f) truth (%.2f, %.2f, %.3f) error %.3f m %.3f rad\n",
                     k - 1, SCN.nlast, n, wall, est.x, est.y, est.th, t.x, t.y, t.th, e, eth);
      }
    }
    double total_s = (esp_timer_get_time() - t_start) / 1e6;
    Con.printf("bytes sent %u, received %u | packets ok %u, checksum errors %u, skipped bytes %u\n", bytes_sent, lidar_bytes, parser.packets_ok, parser.packets_bad_cs, parser.bytes_skipped);
    Con.printf("revolutions matched %d/%d | max points-to-wall %.3f m | max tracking error %.3f m, %.3f rad (scan-match grid 0.05 m / 0.03 rad)\n",
               matched, N, max_wall, max_e, max_eth);
    Con.printf("time per revolution: generate %.1f ms, convert+match %.1f ms | whole run %.2f s = %.1f revolutions/s%s\n",
               gen_ms / (N + 1), matched ? match_ms / matched : 0.0, total_s, matched / total_s, loop ? " (UART-limited, a real X4 sends about 7/s)" : "");
    bool ok = matched == N && parser.packets_bad_cs == 0 && max_wall < 0.03f && max_e < 0.06f && max_eth < 0.035f;
    Con.printf("RESULT vlidar: %s (%s)\n", ok ? "PASS" : "FAIL", loop ? "through UART" : "inside the chip");
  }
done:
  heap_caps_free(occ); heap_caps_free(dist); heap_caps_free(lik); heap_caps_free(buf);
  heap_caps_free(px); heap_caps_free(py); heap_caps_free(idx);
}

// =============================================================================== virtual driving
// [n] The whole navsim.py pipeline on the chip: SLAM mapping tour, then AMCL + A* + pure pursuit to 4 goals on the
// SLAM map, with the virtual X4 as sensor. Runs in its own task with a 32 KB stack.
//   n      nodes talking through topics with ROS 2 names and message types (src/nav_nodes.h), then the node/topic list
//   n old  the same algorithms in one loop (src/nav_core.h nav::run); gives the same results
static int64_t nav_now_us() { return esp_timer_get_time(); }
// =============================================================================== PC link (bridge.h, link.h)
// "bridge" at the end of n / k / K: a bridge node sends topics to the PC tool (tools/robolet.py) on the port the
// command came from, and takes its requests; log lines then go through the same byte stream. "rt": one cycle per
// lidar revolution in real time (1/7 s) instead of as fast as the chip can. "hold" (k only): after the mission's legs the
// run goes on (robot and nodes alive) and takes goals from the PC (tools/ros2.py) until the PC calls /mission/stop.
static wire::Ring link_tx, link_rx;
static wire::Sender link_text;                       // log lines while the link is on (same writer task as the frames)
static uint8_t* link_work = nullptr;
static volatile bool link_on = false, link_running = false;
static int link_port = 0; static bool link_rt = false; static int64_t link_t0 = 0;
static char link_hello[120];
// Chip USB port: the bytes go straight into the 64-byte USB FIFO, written here with the count the hardware takes.
// (The Arduino 2.0.17 USB serial driver copies up to 64 bytes from its buffer into the FIFO in an interrupt and
// ignores how many the FIFO took, so under load some bytes are lost: 922 of 5855 frames arrived broken in the
// 2026-11-26 chip test, while 99 % of the bytes arrived.) The driver is still used for reading and for text
// outside a bridge run. USB-UART port: Serial0 (no such problem), only as much as its buffer takes.
// If the PC reads too slowly, the ring fills up and whole frames are dropped and counted (STATS).
static uint64_t link_bytes_out = 0, link_bytes_in = 0; static uint32_t link_short_writes = 0, link_wait_ms = 0;
// A full 64-byte packet does not end a USB transfer: the PC's driver keeps it until a shorter packet comes. When the
// chip then waits for the PC (Gazebo runs), the last frame would stay in the PC's driver (seen 2026-11-26: the run
// stopped after 221 and after 21 cycles). So after a full packet with nothing behind it, an empty packet follows.
static bool link_zlp_due = false;
static size_t link_port_write(const uint8_t* p, size_t n) {
  if (link_port == 0) {
    if (!usb_serial_jtag_ll_txfifo_writable()) return 0;
    size_t w = usb_serial_jtag_ll_write_txfifo(p, n);
    usb_serial_jtag_ll_txfifo_flush();                 // send this packet
    link_zlp_due = w == 64;
    return w;
  }
#if ARDUINO_USB_CDC_ON_BOOT
  return Serial0.write(p, n);
#else
  return 0;
#endif
}
static int link_port_room() {
  if (link_port == 0) return 512;                     // the FIFO itself says how much it takes
#if ARDUINO_USB_CDC_ON_BOOT
  return Serial0.availableForWrite();
#else
  return 0;
#endif
}
static int link_port_read() {
#if ARDUINO_USB_CDC_ON_BOOT
  return link_port == 0 ? Serial.read() : Serial0.read();
#else
  return Serial.read();
#endif
}
static void link_task(void*) {                       // core 0: ring -> USB, USB -> ring
  static uint8_t tmp[512]; size_t off = 0, len = 0;
  link_bytes_out = 0; link_bytes_in = 0; link_short_writes = 0; link_wait_ms = 0;
  int64_t last_rest = esp_timer_get_time(), last_moved = last_rest;
  while (link_on || link_tx.used() || off < len) {
    if (!link_on && esp_timer_get_time() - last_moved > 2000000) break;   // run over and nobody reads: give up
    if (esp_timer_get_time() - last_rest > 2000) { vTaskDelay(1); last_rest = esp_timer_get_time(); }   // let the idle task run
    bool moved = false;
    if (off == len) { off = 0; len = 0; }
    if (len == 0) {
      int room = link_port_room();
      if (room > 0) len = link_tx.get(tmp, (size_t)room < sizeof tmp ? (size_t)room : sizeof tmp);
    }
    if (off < len) {
      size_t w = link_port_write(tmp + off, len - off);
      if (link_port != 0 && w < len - off) link_short_writes++;   // (on the chip USB port every packet is 64 bytes)
      off += w; link_bytes_out += w; moved = w > 0;
      if (moved) last_moved = esp_timer_get_time();
    }
    int k = 0;
    while (link_rx.space() > 0 && k < 512) {
      int c = link_port_read();
      if (c < 0) break;
      uint8_t b = (uint8_t)c; link_rx.put(&b, 1); k++;
    }
    link_bytes_in += k;
    if (link_zlp_due && off == len && !link_tx.used() && usb_serial_jtag_ll_txfifo_writable()) {
      usb_serial_jtag_ll_txfifo_flush(); link_zlp_due = false;   // empty packet: the PC gets the last full one now
    }
    if (!moved && !k) {
      if (off < len && link_port == 0) { esp_rom_delay_us(20); continue; }   // the PC picks up a packet within ~1 ms
      vTaskDelay(1); last_rest = esp_timer_get_time(); if (off < len || link_tx.used()) link_wait_ms++;
    }
  }
  link_running = false;
  vTaskDelete(nullptr);
}
// "gz" (k only): Gazebo on the PC plays the robot and the world (tools/gzbridge.py); the bridge is on, and every cycle
// first waits for the PC's scan and odometry of that cycle (/gz/step). Nothing for 5 s (60 s before the first cycle,
// while the PC starts Gazebo): the run stops (watchdog).
static bridge::Bridge* link_bridge = nullptr;
static void nav_log(const char* s);
static bool gz_wait(int64_t tick) {
  int64_t t0 = esp_timer_get_time(), limit = tick == 0 ? 60000000 : 5000000, nudge = t0 + 500000;
  for (;;) {
    link_bridge->pump();
    if (esp_timer_get_time() > nudge) { link_bridge->send_stats(); nudge += 500000; }   // a few bytes push anything left out
    bus::Topic* st = bus::find_topic("/gz/step");
    if (st && st->seq > 0 && ((const std_msgs::msg::Int32*)st->slot(st->seq - 1))->data >= tick) return true;
    if (esp_timer_get_time() - t0 > limit) {
      char line[200];
      snprintf(line, sizeof line, "[k]   gz: no data for cycle %d from the PC (last /gz/step %d; from the PC: %llu bytes, %u frames, %u damaged)",
               (int)tick, st && st->seq ? ((const std_msgs::msg::Int32*)st->slot(st->seq - 1))->data : -1,
               (unsigned long long)link_bytes_in, (unsigned)link_bridge->rx_frames, (unsigned)link_bridge->parser.bad);
      nav_log(line);
      return false;
    }
    vTaskDelay(1);
  }
}
static bool link_start(const String& cmd) {
  const bool gz = cmd.indexOf(" gz") > 0;
  if (cmd.indexOf("bridge") < 0 && !gz) return true;
  const size_t TX = 256 * 1024, RX = 16 * 1024, WORK = 64 * 1024;
  link_tx.b = (uint8_t*)heap_caps_malloc(TX, MALLOC_CAP_SPIRAM); link_tx.cap = TX; link_tx.head = 0; link_tx.tail = 0;
  link_rx.b = (uint8_t*)heap_caps_malloc(RX, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); link_rx.cap = RX; link_rx.head = 0; link_rx.tail = 0;
  link_work = (uint8_t*)heap_caps_malloc(3 * WORK, MALLOC_CAP_SPIRAM);
  if (!link_tx.b || !link_rx.b || !link_work) {
    Con.println("bridge: no memory"); heap_caps_free(link_tx.b); heap_caps_free(link_rx.b); heap_caps_free(link_work);
    link_tx.b = link_rx.b = link_work = nullptr; return false;
  }
  link_rt = cmd.indexOf(" rt") > 0; link_port = Con.last_port; link_text.ring = &link_tx;
  navn::run_hooks().hold = cmd.indexOf(" hold") > 0;
  snprintf(link_hello, sizeof link_hello, "%s | %s", FW_VERSION, cmd.c_str());
  navn::run_hooks().extra_node = []() -> bus::NodeBase* {
    link_t0 = esp_timer_get_time();
    return link_bridge = new bridge::Bridge(&link_tx, &link_rx, link_work, WORK, link_hello); };
  if (gz) { navn::run_hooks().gz = true; navn::run_hooks().before_tick = gz_wait; link_rt = false; }   // the PC sets the pace
  navn::run_hooks().after_tick = [](int64_t tick) {
    if (!link_rt) return;
    int64_t due = link_t0 + tick * 142857;           // 1/7 s per cycle
    while (esp_timer_get_time() < due) vTaskDelay(1);
  };
  Con.printf("bridge on (%s port%s%s): the PC tool takes over this port until the run ends\n", link_port ? "USB-UART" : "chip USB",
             link_rt ? ", real time" : "", gz ? ", Gazebo on the PC plays the robot" : "");
  Con.flush(); delay(100);
  link_on = true; link_running = true;
  xTaskCreatePinnedToCore(link_task, "link", 4096, nullptr, 2, nullptr, 0);
  return true;
}
static void link_stop() {
  if (!link_on) return;
  link_on = false;
  while (link_running) delay(10);                    // the ring is sent out first
  navn::run_hooks() = navn::RunHooks(); link_bridge = nullptr;
  delay(200);
  Con.printf("link: %llu bytes to the PC, %u partial writes (finished later), %u ms waiting for the port; %llu bytes from the PC\n",
             (unsigned long long)link_bytes_out, (unsigned)link_short_writes, (unsigned)link_wait_ms, (unsigned long long)link_bytes_in);
  heap_caps_free(link_tx.b); heap_caps_free(link_rx.b); heap_caps_free(link_work);
  link_tx.b = link_rx.b = link_work = nullptr;
}
static void nav_log(const char* s) {
  if (link_on) {                                     // wait up to 0.5 s for room (nobody reading the port: dropped)
    for (int i = 0; i < 500 && link_text.ring->space() < strlen(s) + 1 && link_running; i++) vTaskDelay(1);
    link_text.text(s);
  }
  else Con.println(s);
}
static nav::Result nav_result;
static volatile bool nav_done = false, nav_old = false;
static nav::Work nav_work;
static void* bus_alloc(size_t n) { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }   // topic slots and node buffers
static void* bus_alloc_fast(size_t n) {                          // likelihood fields: internal RAM if there is room
  void* p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return p ? p : heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
}
static void bus_release(void* p) { heap_caps_free(p); }

static void nav_task(void*) {
  nav::Hooks h = {nav_now_us, nav_log};
  bus::hooks().alloc = bus_alloc; bus::hooks().alloc_fast = bus_alloc_fast; bus::hooks().release = bus_release;
  nav_result = nav_old ? nav::run(nav_work, h) : navn::run_nodes(nav_work, h, 0, nav_log);
  nav_done = true;
  vTaskDelete(nullptr);
}

static void* nav_bufs[32]; static int nav_nbufs = 0;
static void* nav_take(void* p) { nav_bufs[nav_nbufs++] = p; return p; }
static void nav_free() { for (int i = 0; i < nav_nbufs; i++) heap_caps_free(nav_bufs[i]); nav_nbufs = 0; }
static bool nav_alloc() {
  using namespace nav;
  const uint32_t CAP_PS = MALLOC_CAP_SPIRAM, CAP_IN = MALLOC_CAP_INTERNAL;
  Work& w = nav_work; nav_nbufs = 0;
  w.occ_truth = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_PS)); w.dist_truth = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_PS));
  w.lo = (int8_t*)nav_take(alloc_cap(CELLS, CAP_PS)); w.occ = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_PS));
  w.dist = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_PS)); w.lik = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_IN)); w.cost = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_PS));
  w.g = (float*)nav_take(heap_caps_malloc(CELLS * 4, CAP_PS)); w.f = (float*)nav_take(heap_caps_malloc(CELLS * 4, CAP_PS));
  w.heap = (uint16_t*)nav_take(heap_caps_malloc(CELLS * 2, CAP_PS)); w.pos = (uint16_t*)nav_take(heap_caps_malloc(CELLS * 2, CAP_PS));
  w.state = (uint8_t*)nav_take(alloc_cap(CELLS, CAP_PS));
  w.path_x = (float*)nav_take(heap_caps_malloc(PATH_CAP * 4, CAP_PS)); w.path_y = (float*)nav_take(heap_caps_malloc(PATH_CAP * 4, CAP_PS));
  w.parts = (Pose*)nav_take(heap_caps_malloc(NP * sizeof(Pose), CAP_IN)); w.parts2 = (Pose*)nav_take(heap_caps_malloc(NP * sizeof(Pose), CAP_IN));
  w.pw = (float*)nav_take(heap_caps_malloc(NP * 4, CAP_IN)); w.rev = (uint8_t*)nav_take(alloc_cap(x4sim::REV_BYTES, CAP_IN));
  w.px = (float*)nav_take(heap_caps_malloc(MAXPTS * 4, CAP_IN)); w.py = (float*)nav_take(heap_caps_malloc(MAXPTS * 4, CAP_IN));
  w.idx = (int32_t*)nav_take(heap_caps_malloc(MAXPTS * 4, CAP_IN));
  for (int i = 0; i < nav_nbufs; i++) if (!nav_bufs[i]) return false;
  return true;
}

static void test_virtual_drive(const String& cmd) {
  using namespace nav;
  nav_old = cmd.indexOf(" old") > 0;
  if (cmd.indexOf(" gz") > 0) { Con.println("gz: only for missions (k w m s o gz)"); Con.println("RESULT nav: FAIL"); return; }
  Con.printf("\n=== [n] virtual driving: SLAM mapping, then AMCL + A* + pure pursuit to 4 goals (about 1-2 minutes), %s ===\n",
             nav_old ? "one loop" : "nodes and topics");
  size_t ps0 = heap_caps_get_free_size(MALLOC_CAP_SPIRAM), in0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  if (!link_start(cmd)) { Con.println("RESULT nav: FAIL"); return; }   // after the banner and the memory count
  if (!nav_alloc()) { Con.println("allocation failed"); Con.println("RESULT nav: FAIL"); nav_free(); link_stop(); return; }
  nav_done = false;
  int64_t t0 = esp_timer_get_time();
  xTaskCreatePinnedToCore(nav_task, "nav", 32768, nullptr, 1, nullptr, 1);
  while (!nav_done) delay(50);
  link_stop();
  const Result& r = nav_result;
  Con.printf("whole run on the chip: %.1f s (of which the virtual lidar itself takes about 47 ms per revolution)\n", (esp_timer_get_time() - t0) / 1e6);
  nav_free();
  delay(200);                                                    // let the long node/topic list leave the USB buffer
  Con.printf("memory after the run (everything freed): PSRAM %u KB free (before %u KB), internal %u KB free (before %u KB)\n",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), (unsigned)(ps0 / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)(in0 / 1024));
  Con.printf("RESULT nav: %s (SLAM error < 0.10 m, map wall error < 0.05 m, all goals within 0.25 m, localization error < 0.20 m, no contacts) | load: mapping %.1f %%, navigation %.1f %% of one core\n",
             r.pass ? "PASS" : "FAIL", r.cpu_load_map, r.cpu_load_nav);
}

// =============================================================================== lost-robot mission
// [k w m s o]  one mission: world w (0 room, 1 s_corridor, 2 s_corridor_lm, 3 doors, 4 room_tilted), mode m (0 AMCL only, 1 spin), seed s,
//              options o (1 kidnap, 2 unmapped box, 3 both; default 3)
// [K n m o]    batch: all 5 worlds x seeds 0..n-1, mode m (default 1 = spin), options o (default 3), summary per world
// Both run the nodes of src/mission_nodes.h (the first mission also prints the node/topic list and the time per node);
// with "old" at the end (k 3 1 0 3 old) the single loop of src/mission_core.h runs instead (same results).
static mis::MWork mis_work;
static volatile bool mis_done = false, mis_old = false;
static int mis_w = 1, mis_m = 1, mis_s = 0, mis_o = 3, mis_batch = 0;
static int mis_count[mis::N_WORLDS][mis::OUT_N];   // [world][outcome]

static void mis_task(void*) {
  nav::Hooks h = {nav_now_us, nav_log};
  bus::hooks().alloc = bus_alloc; bus::hooks().alloc_fast = bus_alloc_fast; bus::hooks().release = bus_release;
  memset(mis_count, 0, sizeof(mis_count));
  int w0 = mis_batch ? 0 : mis_w, w1 = mis_batch ? mis::N_WORLDS - 1 : mis_w, s0 = mis_batch ? 0 : mis_s, s1 = mis_batch ? mis_batch - 1 : mis_s;
  for (int wd = w0; wd <= w1; wd++) for (int sd = s0; sd <= s1; sd++) {
    int64_t t0 = esp_timer_get_time();
    mis::MResult r = mis_old ? mis::run_mission(mis_work, h, wd, mis_m, (uint32_t)sd, mis_o)
                             : misn::run_mission_nodes(mis_work, h, wd, mis_m, (uint32_t)sd, mis_o, (wd == w0 && sd == s0) ? nav_log : nullptr);
    mis::print_result(h, r);
    char line[48]; snprintf(line, sizeof line, "  (chip time %.0f s)", (esp_timer_get_time() - t0) / 1e6); nav_log(line);
    mis_count[wd][r.outcome_code]++;
  }
  mis_done = true;
  vTaskDelete(nullptr);
}

// the mission's buffers (after nav_alloc); false: out of memory
static bool mis_buffers() {
  mis_work.w = &nav_work;
  navn::run_hooks().stage = stage_note;                // where the run is, kept over a restart (the board check prints it)
  mis_work.qx = (float*)nav_take(heap_caps_malloc(nav::MAXPTS * 4, MALLOC_CAP_INTERNAL));
  mis_work.qy = (float*)nav_take(heap_caps_malloc(nav::MAXPTS * 4, MALLOC_CAP_INTERNAL));
  mis_work.cand = (mis::Cand*)nav_take(heap_caps_malloc(mis::CAND_CAP * sizeof(mis::Cand), MALLOC_CAP_SPIRAM));
  mis_work.ex = (float*)nav_take(heap_caps_malloc(mis::MAXHYP * (nav::MAXPTS / 2) * 4, MALLOC_CAP_SPIRAM));
  mis_work.meas = (float*)nav_take(heap_caps_malloc((nav::MAXPTS / 2) * 4, MALLOC_CAP_INTERNAL));
  mis_work.errs = (float*)nav_take(heap_caps_malloc(mis::ERRS_CAP * 4, MALLOC_CAP_SPIRAM));
  mis_work.live = (uint8_t*)nav_take(alloc_cap(nav::CELLS, MALLOC_CAP_SPIRAM)); mis_work.live_dist = (uint8_t*)nav_take(alloc_cap(nav::CELLS, MALLOC_CAP_SPIRAM));
  mis_work.occ_plan = (uint8_t*)nav_take(alloc_cap(nav::CELLS, MALLOC_CAP_SPIRAM)); mis_work.dist_plan = (uint8_t*)nav_take(alloc_cap(nav::CELLS, MALLOC_CAP_SPIRAM));
  mis_work.cost_plan = (uint8_t*)nav_take(alloc_cap(nav::CELLS, MALLOC_CAP_SPIRAM));
  for (int i = 0; i < nav_nbufs; i++) if (!nav_bufs[i]) return false;
  return true;
}

static void test_mission(const String& cmd) {
  bool batch = cmd[0] == 'K';
  mis_old = cmd.indexOf(" old") > 0;                 // (" old": not the "old" in "hold")
  int a = -1, b = -1, c = -1, d = -1;
  sscanf(cmd.c_str() + 1, "%d %d %d %d", &a, &b, &c, &d);
  if (cmd.indexOf(" gz") > 0 && (batch || mis_old)) { Con.println("gz: only for one mission with the nodes (k w m s o gz)"); Con.println("RESULT mission: FAIL"); return; }
  if (batch) { mis_batch = a > 0 ? a : 4; mis_m = b >= 0 ? b : 1; mis_o = c > 0 ? c : 3; }
  else { mis_batch = 0; mis_w = a >= 0 && a < mis::N_WORLDS ? a : 1; mis_m = b >= 0 ? b : 1; mis_s = c >= 0 ? c : 0; mis_o = d > 0 ? d : 3; }
  if (batch) Con.printf("\n=== [K] missions: 5 worlds x %d seeds, mode %s, options %d (1 kidnap, 2 unmapped box) (several minutes per mission), %s ===\n", mis_batch, mis::mode_name(mis_m), mis_o, mis_old ? "one loop" : "nodes and topics");
  else Con.printf("\n=== [k] mission: world %s, mode %s, seed %d, options %d (1 kidnap, 2 unmapped box) (a few minutes), %s ===\n", mis::world_name(mis_w), mis::mode_name(mis_m), mis_s, mis_o, mis_old ? "one loop" : "nodes and topics");
  size_t ps0 = heap_caps_get_free_size(MALLOC_CAP_SPIRAM), in0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  if (!link_start(cmd)) { Con.println("RESULT mission: FAIL"); return; }   // after the banner and the memory count
  if (!nav_alloc()) { Con.println("allocation failed"); Con.println("RESULT mission: FAIL"); nav_free(); link_stop(); return; }
  if (!mis_buffers()) { Con.println("allocation failed"); Con.println("RESULT mission: FAIL"); nav_free(); link_stop(); return; }
  mis_done = false;
  int64_t t0 = esp_timer_get_time();
  xTaskCreatePinnedToCore(mis_task, "mission", 32768, nullptr, 1, nullptr, 1);
  while (!mis_done) delay(100);
  link_stop();
  int wrong = 0;
  for (int wd = 0; wd < mis::N_WORLDS; wd++) {
    int n = 0; for (int o = 0; o < mis::OUT_N; o++) n += mis_count[wd][o];
    if (!n) continue;
    Con.printf("SUMMARY %s %s options %d: %d runs | correct %d, stopped safely %d, incomplete %d, wrong-pose driving %d, contact %d\n",
               mis::world_name(wd), mis::mode_name(mis_m), mis_o, n, mis_count[wd][mis::OUT_CORRECT], mis_count[wd][mis::OUT_STOPPED_SAFELY],
               mis_count[wd][mis::OUT_INCOMPLETE], mis_count[wd][mis::OUT_WRONG_POSE], mis_count[wd][mis::OUT_CONTACT]);
    wrong += mis_count[wd][mis::OUT_WRONG_POSE] + mis_count[wd][mis::OUT_CONTACT];
  }
  double total = (esp_timer_get_time() - t0) / 1e6;
  nav_free();
  delay(200);
  Con.printf("memory after the run (everything freed): PSRAM %u KB free (before %u KB), internal %u KB free (before %u KB)\n",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), (unsigned)(ps0 / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)(in0 / 1024));
  Con.printf("RESULT mission: %s (%d runs with wrong-pose driving or contacts) | total chip time %.0f s\n", wrong ? "SEE ABOVE" : "PASS", wrong, total);
}

// =============================================================================== motor base link
#if BASE_LINK_ENABLED
HardwareSerial BaseSerial(2);
static void base_begin() {
  static bool begun = false;
  if (!begun) { BaseSerial.setRxBufferSize(4096); BaseSerial.begin(BASE_BAUD, SERIAL_8N1, BASE_RX_PIN, BASE_TX_PIN); begun = true; }
}
// all complete "L,R" lines that came in since the last call: keeps the last one; false if none
static bool base_poll_counts(long& l, long& r) {
  static char line[32]; static int n = 0; static bool synced = false; bool got = false;
  while (BaseSerial.available()) {
    char c = (char)BaseSerial.read();
    if (c == '\n') {
      line[n] = 0;
      if (synced) { char* k = strchr(line, ','); if (k && k != line) { l = atol(line); r = atol(k + 1); got = true; } }
      synced = true; n = 0;                                      // (the first line may be cut)
    } else if (c != '\r' && n < (int)sizeof(line) - 1) line[n++] = c;
  }
  return got;
}
static bool base_read_counts(long& l, long& r, uint32_t ms) {   // the last "L,R" line seen within ms
  uint32_t t = millis(); bool got = false; long a, b;
  while (millis() - t < ms) { if (base_poll_counts(a, b)) { l = a; r = b; got = true; } delay(2); }
  return got;
}
#endif

// [m] motor base check; [m go]: without asking (test.sh base). Lift the wheels off the ground first.
static void test_base(const String& cmd) {
  Con.println("\n=== [m] motor base link check ===");
#if !BASE_LINK_ENABLED
  Con.println("disabled: set BASE_LINK_ENABLED 1 and the pins in src/config.h (or use the jarabot environment), rebuild, upload");
  Con.println("RESULT base: FAIL");
#else
  base_begin();
  long l0 = 0, r0 = 0, l1 = 0, r1 = 0;
  BaseSerial.print("pwm,1500,1500\n");
  if (!base_read_counts(l0, r0, 500)) { Con.println("no encoder lines from the base (check wiring: Nano TX -> divider -> GPIO 47, GPIO 21 -> Nano RX, common GND, Nano USB unplugged, baud 57600)"); Con.println("RESULT base: FAIL"); return; }
  Con.printf("encoder counts now L=%ld R=%ld\n", l0, r0);
  bool go = cmd.indexOf("go") > 0;
  if (!go) {
    Con.println("Lift the wheels off the ground, then type yes + Enter within 15 s to run the wheels forward for 1 s.");
    String ans; uint32_t t = millis();
    while (millis() - t < 15000) { if (read_line(ans)) { go = (ans == "yes"); break; } delay(10); }
    if (!go) { Con.println("skipped"); return; }
  } else { Con.println("wheels forward for 1 s in 3 s (lift the wheels off the ground)"); delay(3000); }
  char c[32]; snprintf(c, sizeof(c), "pwm,1500,%d\n", BASE_TEST_FWD);
  base_read_counts(l0, r0, 200);
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) { BaseSerial.print(c); delay(50); }
  for (int i = 0; i < 5; i++) { BaseSerial.print("pwm,1500,1500\n"); delay(20); }
  base_read_counts(l1, r1, 500);
  for (int i = 0; i < 3; i++) { BaseSerial.print("pwm,1500,1500\n"); delay(20); }
  long dl = l1 - l0, dr = r1 - r0;
  Con.printf("after 1 s forward (pwm %d): L %ld -> %ld (%+ld), R %ld -> %ld (%+ld)\n", BASE_TEST_FWD, l0, l1, dl, r0, r1, dr);
  if (dl && dr) Con.printf("  = L %.3f m, R %.3f m with wheel radius 0.035 m and 1200 counts per turn%s\n",
                           dl * 2 * 3.14159 * 0.035 / 1200, dr * 2 * 3.14159 * 0.035 / 1200,
                           (dl > 0) != (dr > 0) ? " | signs differ: set left_encoder_sign or right_encoder_sign of /base_driver to -1" :
                           (dl < 0 ? " | both negative: set both encoder signs of /base_driver to -1" : ""));
  bool ok = labs(dl) > 0 && labs(dr) > 0;
  Con.printf("RESULT base: %s (both encoders must change)\n", ok ? "PASS" : "FAIL");
#endif
}

// =============================================================================== real robot
// [r ... bridge]  the robot-side nodes on the real lidar and motor base (src/real_nodes.h). Started from the PC:
//   python3 tools/ros2.py daemon --chip "r"      (+ /usr/bin/python3 tools/ros2_bridge.py for rviz2 and the keyboard)
static realn::RevFramer real_framer;
static uint8_t* real_rev = nullptr;
static volatile bool real_done = false;
static float real_duty = LIDAR_M_SCTP_DUTY;
static uint32_t real_t_last = 0;
// waits for the next complete revolution (at most 0.5 s); keeps the lidar at 7 Hz by its motor duty
static int real_lidar_rev(uint8_t* out, int cap) {
  uint32_t t0 = millis(); uint8_t buf[256];
  while (millis() - t0 < 500) {
    int n = LidarSerial.available();
    if (n > 0) { if (n > (int)sizeof(buf)) n = sizeof(buf); n = LidarSerial.readBytes(buf, n); real_framer.feed(buf, n); }
    int k = real_framer.take(out, cap);
    if (k > 0) {
      uint32_t now = millis();
#if LIDAR_SPEED_CONTROL
      if (real_t_last && LIDAR_M_SCTP_PIN >= 0) {                // lower duty = faster
        float period = now - real_t_last, err = (period - 142.857f) / 142.857f;
        if (fabsf(err) < 0.5f) { real_duty -= 4.0f * err; real_duty = fmaxf(20.0f, fminf(95.0f, real_duty)); sctp_duty((int)lroundf(real_duty)); }
      }
#endif
      real_t_last = now;
      return k;
    }
    if (n <= 0) vTaskDelay(1);
  }
  return 0;
}
// relay mode ([r relay]): the lidar and the motor controller stay on the USB ports of a Raspberry Pi or PC, and
// tools/relay.py there passes their bytes through the link: /relay/lidar_rx and /relay/base_rx in (std_msgs/UInt8MultiArray),
// /relay/base_tx out. Everything else runs on the chip as with the wires.
static bool real_relay = false;
static realn::CountLines relay_counts;
static std_msgs::msg::UInt8MultiArray relay_msg;
static bus::Publisher<std_msgs::msg::UInt8MultiArray> relay_pub;
static bool relay_hook(const char* topic, const uint8_t* c, size_t n) {
  bool lid = strcmp(topic, "/relay/lidar_rx") == 0, base = strcmp(topic, "/relay/base_rx") == 0;
  if (!lid && !base) return false;
  cdr::Reader r(c, n); if (!r.header()) return true;
  cdr_read(r, relay_msg); if (!r.ok) return true;
  if (lid) real_framer.feed(relay_msg.data.data, (int)relay_msg.data.size);
  else relay_counts.feed(relay_msg.data.data, (int)relay_msg.data.size);
  return true;
}
static int relay_lidar_rev(uint8_t* out, int cap) {           // waits for a revolution from the relay (at most 0.5 s)
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    if (link_bridge) link_bridge->pump();
    int k = real_framer.take(out, cap);
    if (k > 0) return k;
    vTaskDelay(1);
  }
  return 0;
}
static void real_task(void*) {
  nav::Hooks h = {nav_now_us, nav_log};
  bus::hooks().alloc = bus_alloc; bus::hooks().alloc_fast = bus_alloc_fast; bus::hooks().release = bus_release;
  if (real_relay) { relay_pub.t = bus::get_topic<std_msgs::msg::UInt8MultiArray>("/relay/base_tx", 4, false); relay_pub.t->pubs++; }
  realn::run_real(mis_work, h, real_rev, realn::RevFramer::CAP, 7LL * 3600 * 4, nullptr);
  real_done = true;
  vTaskDelete(nullptr);
}
static void test_real(const String& cmd) {
  real_relay = cmd.indexOf(" relay") > 0;
  if (real_relay) Con.printf("\n=== [r relay] real robot: lidar and motor base through tools/relay.py on the PC / Raspberry Pi (mount x %.3f m, yaw %.0f deg) ===\n",
                             (double)LIDAR_MOUNT_X, (double)LIDAR_MOUNT_YAW_DEG);
  else Con.printf("\n=== [r] real robot: lidar (RX=%d, M_CTR/M_SCTP=%d, mount x %.3f m, yaw %.0f deg), motor base %s ===\n",
             LIDAR_RX_PIN, LIDAR_M_SCTP_PIN, (double)LIDAR_MOUNT_X, (double)LIDAR_MOUNT_YAW_DEG, BASE_LINK_ENABLED ? "on" : "off (lidar only: carry or push the robot)");
  if (real_relay && cmd.indexOf("bridge") < 0) { Con.println("relay needs the link: start it from the PC with  python3 tools/ros2.py daemon --chip \"r relay\""); Con.println("RESULT real: FAIL"); return; }
  if (cmd.indexOf("bridge") < 0) Con.println("(no bridge: start it from the PC with  python3 tools/ros2.py daemon --chip \"r\"  to drive and send goals)");
  size_t ps0 = heap_caps_get_free_size(MALLOC_CAP_SPIRAM), in0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  if (!link_start(cmd)) { Con.println("RESULT real: FAIL"); return; }
  navn::run_hooks().after_tick = nullptr;                        // the lidar sets the pace (no "rt" waiting)
  if (!nav_alloc() || !mis_buffers() || !(real_rev = (uint8_t*)nav_take(heap_caps_malloc(realn::RevFramer::CAP, MALLOC_CAP_INTERNAL)))) {
    Con.println("allocation failed"); Con.println("RESULT real: FAIL"); nav_free(); link_stop(); return; }
  navn::lidar_mount().x = LIDAR_MOUNT_X; navn::lidar_mount().yaw_deg = LIDAR_MOUNT_YAW_DEG;
  real_framer.reset(); real_t_last = 0; real_duty = LIDAR_M_SCTP_DUTY;
  if (real_relay) {
    relay_counts = realn::CountLines();
    bridge::publish_hook() = relay_hook;
    realn::io().lidar_rev = relay_lidar_rev;
    realn::io().base_counts = [](long& l, long& r) { return relay_counts.take(l, r); };
    realn::io().base_write = [](const char* s) {
      std_msgs::msg::UInt8MultiArray& m = relay_pub.loan(); size_t k = strlen(s);
      memcpy(m.data.data, s, k); m.data.size = (uint32_t)k; relay_pub.publish(); };
  } else {
    realn::io().lidar_rev = real_lidar_rev;
#if BASE_LINK_ENABLED
    base_begin();
    realn::io().base_write = [](const char* s) { BaseSerial.print(s); };
    realn::io().base_counts = [](long& l, long& r) { return base_poll_counts(l, r); };
#else
    realn::io().base_write = nullptr; realn::io().base_counts = nullptr;
#endif
    lidar_start();
  }
  real_done = false;
  xTaskCreatePinnedToCore(real_task, "real", 32768, nullptr, 1, nullptr, 1);
  while (!real_done) delay(100);
  if (!real_relay) {
    lidar_stop();
#if BASE_LINK_ENABLED
    for (int i = 0; i < 5; i++) { BaseSerial.print("pwm,1500,1500\n"); delay(20); }
#endif
  }
  bridge::publish_hook() = nullptr; relay_pub.t = nullptr;
  link_stop();
  realn::io() = realn::RealIO(); navn::lidar_mount() = navn::LidarMount();
  Con.printf("lidar: %u revolutions, %u damaged packets, motor duty at the end %.0f %%\n", real_framer.revs, real_framer.bad, (double)real_duty);
  nav_free(); real_rev = nullptr;
  delay(200);
  Con.printf("memory after the run (everything freed): PSRAM %u KB free (before %u KB), internal %u KB free (before %u KB)\n",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), (unsigned)(ps0 / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)(in0 / 1024));
  Con.println("RESULT real: done");
}

// =============================================================================== menu
static void menu() {
  Con.println("\ncommands: b board | l lidar 10 s | s lidar stream | p print one scan CSV | c compute benchmark | v virtual lidar | n virtual driving (n old: one loop) | k w m s o mission | K n m o batch (n/k/K ... old: one loop, ... bridge [rt] [hold]: topics to the PC tool, k ... gz: robot in Gazebo, tools/gzbridge.py) | m motor base (m go: without asking) | r real robot (from the PC: tools/ros2.py daemon --chip r) | a all (b, c, l) | h help");
}

void setup() {
  Con.begin(CONSOLE_BAUD);
  delay(1500);
  Con.println("\n" FW_VERSION ", built " __DATE__ " " __TIME__);
  lidar_pins_init();
  test_board();
  menu();
}

void loop() {
  String cmd;
  if (!read_line(cmd) || cmd.length() == 0) { delay(10); return; }
  switch (cmd[0]) {
    case 'b': test_board(); break;
    case 'l': test_lidar(10); break;
    case 's': stream_lidar(); break;
    case 'p': dump_scan(); break;
    case 'c': test_compute(); break;
    case 'v': test_virtual_lidar(); break;
    case 'n': test_virtual_drive(cmd); break;
    case 'k': case 'K': test_mission(cmd); break;
    case 'm': test_base(cmd); break;
    case 'r': test_real(cmd); break;
    case 'a': test_board(); test_compute(); test_lidar(10); break;
    default: break;
  }
  menu();
}
