// In-chip publish/subscribe with ROS 2 names and message types (no DDS, no network).
//   topic        : a name ("/scan"), a ROS 2 message type (sensor_msgs/msg/LaserScan) and a ring of `depth` message
//                  slots allocated once (PSRAM on the chip). Publishing writes the next slot; nothing is allocated per
//                  message. A latched topic gives its last message to a subscriber created later (ROS 2
//                  transient_local, as used for /map).
//   node         : named set of publishers, subscriptions and timers.
//   executor     : spin_once() visits the nodes in the order they were created; for each node it delivers the new
//                  messages of its subscriptions (in the order the subscriptions were created), then runs its due
//                  timers. One executor in one task, so a run is repeatable: the same inputs give the same outputs.
//   introspection: node and topic lists with types, publisher/subscriber counts, message counts and rates
//                  (graph_print), the information `ros2 node list` / `ros2 topic list -v` / `ros2 topic hz` show.
//   wire         : every topic can write and read its messages as ROS 2 CDR bytes (Topic::ser / deser, cdr.h), and a
//                  node can subscribe to or publish on a topic by name without knowing the type (bridge.h uses this).
// Services, actions, parameters and lifecycle states: ros.h (built on this file). Not here: several executors in
// several tasks (locking).
// Pure C++: the same file runs on a PC (test_host/) and on the chip.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <new>
#include <functional>
#include "cdr.h"

namespace bus {

struct Hooks {
  void* (*alloc)(size_t) = malloc;      // message slots and other large buffers (the firmware sets a PSRAM allocator)
  void* (*alloc_fast)(size_t) = malloc; // small buffers read in inner loops (the firmware: internal RAM, else PSRAM)
  void (*release)(void*) = free;        // frees memory from either
  int64_t (*now_ns)() = nullptr;        // clock for stamps, timers and rates (simulation: simulated time)
  int64_t (*cpu_us)() = nullptr;        // real clock to measure how long each node's callbacks take
};
inline Hooks& hooks() { static Hooks h; return h; }
inline int64_t now_ns() { return hooks().now_ns ? hooks().now_ns() : 0; }

static const int MAX_TOPICS = 72, MAX_NODES = 20, MAX_SUBS = 24, MAX_PUBS = 16, MAX_TIMERS = 4, NAME_LEN = 48;

struct Topic {
  char name[NAME_LEN];
  const char* type;
  size_t size; int depth; bool latched;
  uint8_t* slots;
  void (*destroy)(void*);                // runs the message destructor before the slots are released
  size_t (*ser)(const void* msg, uint8_t* buf, size_t cap);    // ROS 2 CDR bytes (with header); 0 if they do not fit
  bool (*deser)(void* msg, const uint8_t* buf, size_t n);      // false: not this type / does not fit
  uint32_t seq = 0;                      // number of messages published so far
  int pubs = 0, subs = 0;
  int64_t first_ns = -1, last_ns = 0;
  void* slot(uint32_t s) { return slots + (size_t)(s % (uint32_t)depth) * size; }
};

struct Graph {
  Topic* topics[MAX_TOPICS]; int ntopics = 0;
  struct NodeBase* nodes[MAX_NODES]; int nnodes = 0;
};
inline Graph& graph() { static Graph g; return g; }

inline Topic* find_topic(const char* name) {
  Graph& g = graph();
  for (int i = 0; i < g.ntopics; i++) if (strcmp(g.topics[i]->name, name) == 0) return g.topics[i];
  return nullptr;
}

// Get or create a topic of message type T. A second user with a different type is a programming error.
template <typename T>
inline Topic* get_topic(const char* name, int depth, bool latched) {
  Topic* t = find_topic(name);
  if (t) {
    if (strcmp(t->type, T::TYPE) != 0) { printf("bus: topic %s is %s, not %s\n", name, t->type, T::TYPE); abort(); }
    if (depth > t->depth) { printf("bus: topic %s created with depth %d, asked %d\n", name, t->depth, depth); }
    t->latched = t->latched || latched;
    return t;
  }
  Graph& g = graph();
  if (g.ntopics >= MAX_TOPICS) { printf("bus: too many topics\n"); abort(); }
  t = new Topic();
  strncpy(t->name, name, NAME_LEN - 1); t->name[NAME_LEN - 1] = 0;
  t->type = T::TYPE; t->size = sizeof(T); t->depth = depth < 1 ? 1 : depth; t->latched = latched;
  t->slots = (uint8_t*)hooks().alloc(t->size * t->depth);
  if (!t->slots) { printf("bus: no memory for topic %s (%u bytes)\n", name, (unsigned)(t->size * t->depth)); abort(); }
  for (int i = 0; i < t->depth; i++) new (t->slots + i * t->size) T();
  t->destroy = [](void* p) { ((T*)p)->~T(); };
  t->ser = [](const void* m, uint8_t* b, size_t cap) -> size_t {
    cdr::Writer w(b, cap); w.header(); cdr_write(w, *(const T*)m); return w.ok ? w.pos : 0; };
  t->deser = [](void* m, const uint8_t* b, size_t n) -> bool {
    cdr::Reader r(b, n); if (!r.header()) return false; cdr_read(r, *(T*)m); return r.ok && r.pos == n; };
  g.topics[g.ntopics++] = t;
  return t;
}

inline void mark_published(Topic* t) {
  int64_t now = now_ns();
  if (t->first_ns < 0) t->first_ns = now;
  t->last_ns = now; t->seq++;
}
// publish on a topic found by name, from CDR bytes (a message from the PC); false if the bytes are not that type
inline bool publish_cdr(Topic* t, const uint8_t* b, size_t n) {
  void* slot = t->slot(t->seq);
  if (!t->deser(slot, b, n)) return false;
  mark_published(t); return true;
}

// Publisher: loan() gives the next slot to fill in place (no copy, nothing on the stack), publish() makes it visible.
template <typename T>
struct Publisher {
  Topic* t = nullptr;
  T& loan() { return *(T*)t->slot(t->seq); }
  void publish() { mark_published(t); }
  void publish(const T& m) { loan() = m; publish(); }
  const char* topic_name() const { return t->name; }
};

struct SubBase {
  Topic* t = nullptr; uint32_t next = 0; uint32_t lost = 0;
  virtual void deliver() = 0;
  virtual ~SubBase() {}
};
// Subscription: the callback gets every message published since the last delivery (at most `depth` of them; older
// ones count as lost). The message stays valid only during the callback.
template <typename T>
struct Subscription : SubBase {
  std::function<void(const T&)> cb;
  const T* last = nullptr;               // last delivered message (valid until the slot is reused)
  void deliver() override {
    uint32_t s = t->seq;
    if (s - next > (uint32_t)t->depth) { lost += s - next - t->depth; next = s - t->depth; }
    for (; next != s; next++) { last = (const T*)t->slot(next); if (cb) cb(*last); }
  }
};

// subscription to a topic of any type (the callback gets a pointer to the message)
struct SubRaw : SubBase {
  std::function<void(const void*)> cb;
  void deliver() override {
    uint32_t s = t->seq;
    if (s - next > (uint32_t)t->depth) { lost += s - next - t->depth; next = s - t->depth; }
    for (; next != s; next++) if (cb) cb(t->slot(next));
  }
};

struct Timer { int64_t period_ns, next_ns; std::function<void()> cb; };

struct NodeBase {
  char name[NAME_LEN];
  SubBase* subs[MAX_SUBS]; int nsubs = 0;
  Topic* pubs[MAX_PUBS]; int npubs = 0;
  Timer timers[MAX_TIMERS]; int ntimers = 0;
  double busy_us = 0; uint32_t calls = 0;   // real time spent in this node's callbacks (hooks().cpu_us)
  bool running = true;                      // false: the executor skips the node (a lifecycle node not "active")
  explicit NodeBase(const char* n) {
    strncpy(name, n, NAME_LEN - 1); name[NAME_LEN - 1] = 0;
    Graph& g = graph(); if (g.nnodes >= MAX_NODES) { printf("bus: too many nodes\n"); abort(); }
    g.nodes[g.nnodes++] = this;
  }
  virtual ~NodeBase() {
    for (int i = 0; i < nsubs; i++) delete subs[i];
    Graph& g = graph();                                   // leave the graph
    for (int i = 0; i < g.nnodes; i++) if (g.nodes[i] == this) { for (int k = i; k + 1 < g.nnodes; k++) g.nodes[k] = g.nodes[k + 1]; g.nnodes--; break; }
  }
  template <typename T> Publisher<T> create_publisher(const char* topic, int depth = 1, bool latched = false) {
    Publisher<T> p; p.t = get_topic<T>(topic, depth, latched); p.t->pubs++;
    if (npubs < MAX_PUBS) pubs[npubs++] = p.t;
    return p;
  }
  template <typename T> Subscription<T>* create_subscription(const char* topic, std::function<void(const T&)> cb, int depth = 1) {
    if (nsubs >= MAX_SUBS) { printf("bus: node %s has too many subscriptions\n", name); abort(); }
    Subscription<T>* s = new Subscription<T>();
    s->t = get_topic<T>(topic, depth, false); s->t->subs++; s->cb = cb;
    s->next = (s->t->latched && s->t->seq > 0) ? s->t->seq - 1 : s->t->seq;   // latched: get the last message
    subs[nsubs++] = s;
    return s;
  }
  SubRaw* create_raw_subscription(Topic* t, std::function<void(const void*)> cb) {
    if (nsubs >= MAX_SUBS) { printf("bus: node %s has too many subscriptions\n", name); abort(); }
    SubRaw* s = new SubRaw(); s->t = t; t->subs++; s->cb = cb;
    s->next = (t->latched && t->seq > 0) ? t->seq - 1 : t->seq;
    subs[nsubs++] = s;
    return s;
  }
  void remove_subscription(SubBase* s) {
    for (int i = 0; i < nsubs; i++) if (subs[i] == s) {
      s->t->subs--; delete s;
      for (int k = i; k + 1 < nsubs; k++) subs[k] = subs[k + 1];
      nsubs--; return;
    }
  }
  void create_timer(int64_t period_ns, std::function<void()> cb) {
    if (ntimers >= MAX_TIMERS) { printf("bus: node %s has too many timers\n", name); abort(); }
    timers[ntimers++] = {period_ns, now_ns(), cb};
  }
};

// the node whose callbacks are running (nullptr outside spin_once), e.g. for the name on /rosout
inline NodeBase*& current_node() { static NodeBase* n = nullptr; return n; }

// One pass over all nodes (see the header comment for the order).
inline void spin_once() {
  Graph& g = graph();
  int64_t (*cpu)() = hooks().cpu_us;
  for (int n = 0; n < g.nnodes; n++) {
    NodeBase* nd = g.nodes[n];
    if (!nd->running) continue;               // its messages wait (up to the subscription depth) until it is active
    int64_t c0 = cpu ? cpu() : 0; bool ran = false; current_node() = nd;
    for (int i = 0; i < nd->nsubs; i++) {
      SubBase* s = nd->subs[i];
      if (s->t->seq != s->next) { s->deliver(); ran = true; }
    }
    int64_t now = now_ns();
    for (int i = 0; i < nd->ntimers; i++) {
      Timer& tm = nd->timers[i];
      if (now >= tm.next_ns) { tm.next_ns += tm.period_ns; if (tm.next_ns <= now) tm.next_ns = now + tm.period_ns; tm.cb(); ran = true; }
    }
    if (ran) { nd->calls++; if (cpu) nd->busy_us += (double)(cpu() - c0); }
  }
  current_node() = nullptr;
}

// Release all topics. The nodes belong to whoever created them and must be deleted (or go out of scope) first.
inline void reset() {
  Graph& g = graph();
  if (g.nnodes) printf("bus: reset with %d nodes still alive\n", g.nnodes);
  for (int i = 0; i < g.ntopics; i++) {
    Topic* t = g.topics[i];
    for (int k = 0; k < t->depth; k++) t->destroy(t->slots + k * t->size);
    hooks().release(t->slots); delete t;
  }
  g.ntopics = 0;
}

// `ros2 node list` + `ros2 topic list -v` + rates, written line by line to out()
inline void graph_print(void (*out)(const char*)) {
  Graph& g = graph(); char line[200];
  snprintf(line, sizeof line, "nodes (%d):", g.nnodes); out(line);
  for (int n = 0; n < g.nnodes; n++) {
    NodeBase* nd = g.nodes[n];
    const int L = (int)sizeof line;                       // (snprintf returns the length it wanted: keep p inside the line)
    auto add = [&](int p, const char* fmt, const char* s) { if (p < L - 1) p += snprintf(line + p, L - p, fmt, s); return p < L - 1 ? p : L - 1; };
    int p = add(0, "  /%s  pub:", nd->name);
    for (int i = 0; i < nd->npubs; i++) p = add(p, " %s", nd->pubs[i]->name);
    p = add(p, "  sub:%s", "");
    for (int i = 0; i < nd->nsubs; i++) p = add(p, " %s", nd->subs[i]->t->name);
    out(line);
  }
  snprintf(line, sizeof line, "topics (%d):", g.ntopics); out(line);
  for (int i = 0; i < g.ntopics; i++) {
    Topic* t = g.topics[i];
    double span = (t->last_ns - t->first_ns) * 1e-9, hz = (t->seq > 1 && span > 0) ? (t->seq - 1) / span : 0;
    snprintf(line, sizeof line, "  %-26s %-40s %d pub %d sub | %6u msgs %6.2f Hz | %u B x %d%s", t->name, t->type, t->pubs, t->subs,
             (unsigned)t->seq, hz, (unsigned)t->size, t->depth, t->latched ? " latched" : "");
    out(line);
  }
}

// time per node: `ros2 run` has no direct equivalent; this is what `top` would show per node on Linux
inline void load_print(void (*out)(const char*), double seconds_simulated) {
  Graph& g = graph(); char line[160];
  for (int n = 0; n < g.nnodes; n++) {
    NodeBase* nd = g.nodes[n];
    snprintf(line, sizeof line, "  /%-18s %7u runs  %9.1f ms  %5.2f %% of one core", nd->name, (unsigned)nd->calls, nd->busy_us / 1000.0,
             seconds_simulated > 0 ? nd->busy_us / (seconds_simulated * 1e6) * 100.0 : 0.0);
    out(line);
  }
}

}  // namespace bus
