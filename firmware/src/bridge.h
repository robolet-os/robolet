// Bridge node: topics to and from the PC over the serial link (link.h), as ROS 2 CDR bytes.
//   start : HELLO (firmware text) + ADVERTISE for every topic (id, name, type).
//   PC -> chip: SUBSCRIBE name [max rate]   the bridge forwards that topic (at most that rate, maps run-length coded)
//               UNSUBSCRIBE name, LIST (all ADVERTISE frames again),
//               PUBLISH name type CDR       publishes on that topic in the chip (made if it does not exist yet)
//               CALL id name CDR            calls a service (ros.h); REPLY id code CDR comes back (code 0 = answered,
//                                           -1 = no such service, -2 = bad request). An action's get_result answers when
//                                           the goal ends. SERVICE frames (name, type) list the services, at the start
//                                           and on LIST. NODE frames (name, published topics, subscribed topics,
//                                           space-separated) list the nodes, at the start and on LIST.
//   A forwarded topic without a rate limit goes out message by message (all of them, e.g. /rosout); with a rate
//   limit, the newest message when the period is over.
//   every second: STATS (frames sent / dropped because the link was full / bad frames received); text as LOG frames.
// The bridge only reads the topics it forwards, so the robot nodes give the same results with and without it.
// It is created after the other nodes, so it runs last in each cycle and sends what they published in that cycle.
#pragma once
#include "bus.h"
#include "link.h"
#include "ros2_msgs.h"
#include "ros.h"

namespace bridge {

// make a topic of a type given by name (a PUBLISH from the PC for a topic the chip does not have yet)
inline bus::Topic* topic_by_type(const char* name, const char* type) {
  bus::Topic* t = bus::find_topic(name);
  if (t) return strcmp(t->type, type) == 0 ? t : nullptr;
#define X(T) if (strcmp(type, T::TYPE) == 0) return bus::get_topic<T>(name, 1, false);
  ROS2_MSG_TYPES(X)
#undef X
  return nullptr;
}

// a PUBLISH from the PC goes here first; true: taken (not published on the bus). Used by the real robot's relay mode
// (bytes of the lidar and the motor controller, tools/relay.py).
inline std::function<bool(const char* topic, const uint8_t* cdr, size_t n)>& publish_hook() {
  static std::function<bool(const char*, const uint8_t*, size_t)> f; return f;
}

struct Bridge : bus::NodeBase {
  wire::Sender tx; wire::Ring* rx; wire::Parser parser;
  uint8_t *buf, *rle; size_t cap;
  struct Fwd { bus::Topic* t; bus::SubRaw* s; int64_t period_ns, last_ns; const void* msg; };
  Fwd fwd[bus::MAX_SUBS]; int nf = 0;
  int64_t next_stats = 0; uint32_t rx_frames = 0, rx_bad_publish = 0;   // frames from the PC; PUBLISHes that did not fit
  // tx_ring / rx_ring: filled and emptied by the I/O side (a task on the chip, the test on a PC);
  // work: 3 buffers of `buf_cap` bytes (serialized message, run-length code, received frame)
  Bridge(wire::Ring* tx_ring, wire::Ring* rx_ring, uint8_t* work, size_t buf_cap, const char* hello)
      : NodeBase("bridge"), rx(rx_ring), parser(work + 2 * buf_cap, buf_cap, 16384), buf(work), rle(work + buf_cap), cap(buf_cap) {
    tx.ring = tx_ring;
    tx.frame(wire::HELLO, 0, (const uint8_t*)hello, strlen(hello));
    advertise_all(); advertise_services(); advertise_nodes();
    ros::remote_reply() = [this](uint32_t id, const uint8_t* cdr, size_t n) { reply(id, 0, cdr, n); };
    create_timer(0, [this]() { cycle(); });
  }
  ~Bridge() { ros::remote_reply() = nullptr; }
  void advertise_services() {
    ros::Registry& r = ros::registry();
    for (int i = 0; i < r.n; i++) {
      size_t a = strlen(r.s[i]->name) + 1, b = strlen(r.s[i]->type) + 1;
      memcpy(buf, r.s[i]->name, a); memcpy(buf + a, r.s[i]->type, b);
      tx.frame(wire::SERVICE, (uint16_t)i, buf, a + b);
    }
  }
  // NODE frame: name \0 publishers \0 subscriptions \0 (topic names separated by spaces)
  void advertise_nodes() {
    bus::Graph& g = bus::graph();
    for (int i = 0; i < g.nnodes; i++) {
      bus::NodeBase* nd = g.nodes[i]; size_t p = 0;
      auto put = [&](const char* s) { size_t k = strlen(s); if (p + k + 2 < cap) { memcpy(buf + p, s, k); p += k; } };
      put(nd->name); buf[p++] = 0;
      for (int k = 0; k < nd->npubs; k++) { if (k) put(" "); put(nd->pubs[k]->name); }
      buf[p++] = 0;
      for (int k = 0; k < nd->nsubs; k++) { if (k) put(" "); put(nd->subs[k]->t->name); }
      buf[p++] = 0;
      tx.frame(wire::NODE, (uint16_t)i, buf, p);
    }
  }
  void reply(uint32_t id, int32_t code, const uint8_t* cdr, size_t n) {
    if (8 + n > cap) { n = 0; code = -2; }
    memcpy(rle, &id, 4); memcpy(rle + 4, &code, 4); if (n) memcpy(rle + 8, cdr, n);
    tx.frame(wire::REPLY, 0, rle, 8 + n);
  }
  int id_of(bus::Topic* t) { bus::Graph& g = bus::graph(); for (int i = 0; i < g.ntopics; i++) if (g.topics[i] == t) return i; return -1; }
  void advertise(bus::Topic* t) {
    size_t a = strlen(t->name) + 1, b = strlen(t->type) + 1;
    memcpy(buf, t->name, a); memcpy(buf + a, t->type, b);
    tx.frame(wire::ADVERTISE, (uint16_t)id_of(t), buf, a + b);
  }
  void advertise_all() { bus::Graph& g = bus::graph(); for (int i = 0; i < g.ntopics; i++) advertise(g.topics[i]); }
  void subscribe(const char* name, uint16_t rate10) {
    bus::Topic* t = bus::find_topic(name);
    if (!t) { char line[80]; snprintf(line, sizeof line, "bridge: no topic %s", name); tx.text(line); return; }
    int64_t period = rate10 ? (int64_t)(1e10 / rate10) : 0;
    for (int i = 0; i < nf; i++) if (fwd[i].t == t) { fwd[i].period_ns = period; return; }
    if (nf >= bus::MAX_SUBS) { tx.text("bridge: too many forwarded topics"); return; }
    Fwd& f = fwd[nf]; f.t = t; f.period_ns = period; f.last_ns = INT64_MIN / 2; f.msg = nullptr;
    int k = nf++;
    f.s = create_raw_subscription(t, [this, k](const void* m) { got(k, m); });
    advertise(t);
  }
  void unsubscribe(const char* name) {
    for (int i = 0; i < nf; i++) if (strcmp(fwd[i].t->name, name) == 0) {
      remove_subscription(fwd[i].s);
      for (int k = i; k + 1 < nf; k++) fwd[k] = fwd[k + 1];
      nf--;
      for (int k = 0; k < nf; k++) { int kk = k; fwd[k].s->cb = [this, kk](const void* m) { got(kk, m); }; }
      return;
    }
  }
  void on_frame() {
    rx_frames++;
    const char* p = (const char*)parser.payload; size_t n = parser.len;
    size_t a = strnlen(p, n);
    if (parser.kind == wire::SUBSCRIBE && a + 3 <= n) subscribe(p, (uint16_t)(parser.payload[a + 1] | (parser.payload[a + 2] << 8)));
    else if (parser.kind == wire::UNSUBSCRIBE && a < n) unsubscribe(p);
    else if (parser.kind == wire::LIST) { advertise_all(); advertise_services(); advertise_nodes(); }
    else if (parser.kind == wire::CALL && n >= 5) {             // u32 id, name, CDR request
      uint32_t id; memcpy(&id, parser.payload, 4);
      const char* nm = (const char*)parser.payload + 4; size_t a = strnlen(nm, n - 4);
      if (4 + a >= n) { reply(id, -2, nullptr, 0); return; }
      ros::ServiceBase* sv = ros::find_service(nm);
      if (!sv) { reply(id, -1, nullptr, 0); return; }
      int k = sv->call_cdr(parser.payload + 4 + a + 1, n - 4 - a - 1, buf, cap, id);
      if (k > 0) reply(id, 0, buf, (size_t)k); else if (k < 0) reply(id, -2, nullptr, 0);   // 0: answered later
    }
    else if (parser.kind == wire::PUBLISH && a < n) {
      const char* type = p + a + 1; size_t b = strnlen(type, n - a - 1);
      if (a + 1 + b >= n) { rx_bad_publish++; return; }
      if (publish_hook() && publish_hook()(p, parser.payload + a + 1 + b + 1, n - (a + 1 + b + 1))) return;
      bool known = bus::find_topic(p) != nullptr;
      bus::Topic* t = topic_by_type(p, type);
      if (t && !known) advertise(t);
      const uint8_t* c = parser.payload + a + 1 + b + 1;
      if (!t || !bus::publish_cdr(t, c, n - (a + 1 + b + 1))) rx_bad_publish++;
    }
  }
  // received frames -> requests and PUBLISHes (also called between cycles while the run waits for the PC, gz mode)
  void pump() {
    uint8_t tmp[256]; size_t k;
    while ((k = rx->get(tmp, sizeof tmp)) > 0)
      for (size_t off = 0; off < k;) { off += parser.put(tmp + off, k - off); while (parser.next()) on_frame(); }
  }
  void got(int k, const void* m) { if (fwd[k].period_ns == 0) send(fwd[k], m, bus::now_ns()); else fwd[k].msg = m; }
  void send(Fwd& f, const void* msg, int64_t now) {
    size_t n = f.t->ser(msg, buf, cap);
    if (!n) { tx.dropped++; return; }
    uint8_t flags = 0; const uint8_t* out = buf;
    if (n > 2048) { size_t r = wire::rle_encode(buf, n, rle, cap); if (r) { out = rle; n = r; flags = wire::F_RLE; } }
    if (tx.frame(wire::DATA, (uint16_t)id_of(f.t), out, n, flags)) f.last_ns = now;
  }
  void cycle() {
    pump();
    int64_t now = bus::now_ns();
    for (int i = 0; i < nf; i++) {
      Fwd& f = fwd[i];
      if (!f.msg || now - f.last_ns < f.period_ns) continue;
      const void* m = f.msg; f.msg = nullptr; send(f, m, now);
    }
    if (now >= next_stats) { next_stats = now + 1000000000LL; send_stats(); }
  }
  void send_stats() {
    uint32_t st[6] = {tx.sent, tx.dropped, parser.bad, rx_frames, rx_bad_publish, (uint32_t)tx.ring->used()};
    tx.frame(wire::STATS, 0, (const uint8_t*)st, sizeof st);
  }
  void log(const char* line) { tx.text(line); }
};

}  // namespace bridge
