// ROS 2 services, parameters, lifecycle states and actions on the in-chip bus (bus.h), with the ROS 2 interface types
// (rcl_interfaces, lifecycle_msgs, action_msgs, std_srvs, nav2_msgs from msg/) and the ROS 2 names:
//   service    : name + srv type + handler. Inside the chip a call runs the handler at once (one executor, no threads,
//                so a run is repeatable). From the PC (bridge.h CALL frame) the request comes as CDR bytes and the
//                response goes back as CDR (REPLY frame); a handler may also answer later (an action's get_result).
//   parameters : declare_parameter(name, default, description, min, max) on a ros::Node; the node serves
//                /<node>/get_parameters, /<node>/set_parameters, /<node>/list_parameters (rcl_interfaces), so
//                /<node>/describe_parameters, /<node>/get_parameter_types, /<node>/set_parameters_atomically
//                (rcl_interfaces), so `ros2 param get / set / list / describe`-style tools work. Values are checked
//                against type, range and read-only; every change is published on /parameter_events. Every node has
//                the read-only parameter use_sim_time (true when the clock is the simulation's, see /clock).
//   lifecycle  : a managed node starts "unconfigured"; configure -> "inactive", activate -> "active", and back with
//                deactivate / cleanup, shutdown -> "finalized". The executor runs only active nodes. Each managed node
//                serves /<node>/change_state, /<node>/get_state, /<node>/get_available_transitions (lifecycle_msgs) and
//                publishes /<node>/transition_event (two events per transition, as rclcpp: primary state -> transition
//                state -> primary state). LifecycleManager does what
//                nav2_lifecycle_manager does: starts the nodes in order (configure all, then activate all) and serves
//                ~/manage_nodes (nav2_msgs/srv/ManageLifecycleNodes) and ~/is_active (std_srvs/srv/Trigger).
//   actions    : as rclcpp_action: services /<name>/_action/send_goal, cancel_goal, get_result and topics
//                /<name>/_action/feedback, /<name>/_action/status. One goal at a time; a new goal ends the running
//                one (aborted, "preempted"), as Nav2's single-goal servers do. A client in the chip gets the result
//                through a callback as soon as the server ends the goal.
//   /rosout    : rosout() publishes a log line as rcl_interfaces/Log with the name of the node that is running.
// Pure C++: the same file runs on a PC (test_host/) and on the chip.
#pragma once
#include "bus.h"
#include "ros2_msgs.h"
#include <math.h>

namespace ros {
using bus::NodeBase;
static const int MAX_SERVICES = 192, MAX_ACTIONS = 8, NAME = 64, MAX_PARAMS = 16, MAX_NODE_SERVICES = 20;

// true while the clock (bus::now_ns) is a simulation's: the value of every node's use_sim_time parameter
inline bool& sim_time() { static bool b = false; return b; }

// /rosout: one log line as rcl_interfaces/Log (rosout_topic() makes the topic; again after each bus::reset())
inline bus::Topic* rosout_topic() {
  bus::Topic* t = bus::find_topic("/rosout");
  if (!t) { t = bus::get_topic<rcl_interfaces::msg::Log>("/rosout", 8, false); t->pubs++; }
  return t;
}
inline void rosout(uint8_t level, const char* text, const char* node = nullptr) {
  bus::Topic* t = rosout_topic();
  rcl_interfaces::msg::Log& m = *(rcl_interfaces::msg::Log*)t->slot(t->seq);
  int64_t ns = bus::now_ns(); m.stamp.sec = (int32_t)(ns / 1000000000); m.stamp.nanosec = (uint32_t)(ns % 1000000000);
  bus::NodeBase* cur = bus::current_node();
  m.level = level; m.name.set(node ? node : cur ? cur->name : "robolet"); m.msg.set(text);
  m.file.set(""); m.function.set(""); m.line = 0;
  bus::mark_published(t);
}

// ------------------------------------------------------------------------------------------------------------ services
struct ServiceBase {
  char name[NAME] = {0}; const char* type = ""; NodeBase* node = nullptr; uint32_t calls = 0;
  // a call from the PC: CDR request in, CDR response out. >0: bytes written; 0: the answer comes later (see
  // remote_reply()); <0: bad request
  virtual int call_cdr(const uint8_t* in, size_t n, uint8_t* out, size_t cap, uint32_t call_id) = 0;
  virtual ~ServiceBase();
};
struct Registry { ServiceBase* s[MAX_SERVICES]; int n = 0; };
inline Registry& registry() { static Registry r; return r; }
inline ServiceBase* find_service(const char* name) {
  Registry& r = registry();
  for (int i = 0; i < r.n; i++) if (strcmp(r.s[i]->name, name) == 0) return r.s[i];
  return nullptr;
}
inline void add_service(ServiceBase* s) {
  Registry& r = registry();
  if (r.n >= MAX_SERVICES || find_service(s->name)) { printf("ros: cannot add service %s\n", s->name); abort(); }
  r.s[r.n++] = s;
}
inline ServiceBase::~ServiceBase() {
  Registry& r = registry();
  for (int i = 0; i < r.n; i++) if (r.s[i] == this) { for (int k = i; k + 1 < r.n; k++) r.s[k] = r.s[k + 1]; r.n--; break; }
}
// an answer that comes after the call (set by the bridge): call id, CDR response
inline std::function<void(uint32_t, const uint8_t*, size_t)>& remote_reply() { static std::function<void(uint32_t, const uint8_t*, size_t)> f; return f; }

template <typename T> inline size_t to_cdr(const T& m, uint8_t* out, size_t cap) {
  cdr::Writer w(out, cap); w.header(); cdr_write(w, m); return w.ok ? w.pos : 0;
}
template <typename T> inline T* alloc_msg() { return new (bus::hooks().alloc(sizeof(T))) T(); }
template <typename T> inline void free_msg(T* p) { if (p) { p->~T(); bus::hooks().release(p); } }
template <typename T> inline void clear_msg(T* p) { p->~T(); new (p) T(); }   // no large temporary on the stack

template <typename Srv> struct Service : ServiceBase {
  using Req = typename Srv::Request; using Res = typename Srv::Response;
  // fills res; returns false when it answers later (calls from the PC only)
  std::function<bool(const Req&, Res&, uint32_t call_id)> cb;
  Req* rq = nullptr; Res* rs = nullptr;                          // for calls from the PC, allocated on the first one
  ~Service() { free_msg(rq); free_msg(rs); }
  bool call(const Req& q, Res& r) { calls++; clear_msg(&r); return cb(q, r, 0); }   // a call inside the chip
  int call_cdr(const uint8_t* in, size_t n, uint8_t* out, size_t cap, uint32_t call_id) override {
    if (!rq) { rq = alloc_msg<Req>(); rs = alloc_msg<Res>(); }
    clear_msg(rq); clear_msg(rs);
    cdr::Reader r(in, n); if (!r.header()) return -1;
    cdr_read(r, *rq); if (!r.ok || r.pos != n) return -1;
    calls++;
    if (!cb(*rq, *rs, call_id)) return 0;
    size_t k = to_cdr(*rs, out, cap); return k ? (int)k : -1;
  }
};
template <typename Srv> struct Client {
  char name[NAME] = {0}; Service<Srv>* s = nullptr;
  bool ready() { return find(); }
  bool find() {
    if (s) return true;
    ServiceBase* b = find_service(name);
    if (!b || strcmp(b->type, Srv::TYPE) != 0) return false;
    s = (Service<Srv>*)b; return true;
  }
  bool call(const typename Srv::Request& q, typename Srv::Response& r) { return find() && s->call(q, r); }
};

// ---------------------------------------------------------------------------------------------------------- parameters
using rcl_interfaces::msg::ParameterType;
struct Param {
  char name[48] = {0}; const char* description = ""; uint8_t type = ParameterType::PARAMETER_NOT_SET; bool read_only = false;
  bool b = false; int64_t i = 0; double d = 0; char s[48] = {0}; double lo = -INFINITY, hi = INFINITY;
  const char* const* sarr = nullptr; int nsarr = 0;              // string array (read only, owned by the node)
  void to_msg(rcl_interfaces::msg::ParameterValue& v) const {
    v.type = type; v.bool_value = b; v.integer_value = i; v.double_value = d; v.string_value.set(s);
    v.string_array_value.size = 0;
    for (int k = 0; k < nsarr && k < (int)v.string_array_value.capacity; k++) { v.string_array_value[k].set(sarr[k]); v.string_array_value.size = k + 1; }
  }
};

// a node with parameters, services and (if managed) a lifecycle state
struct Node : NodeBase {
  using State = lifecycle_msgs::msg::State;
  using Transition = lifecycle_msgs::msg::Transition;
  ServiceBase* srvs[MAX_NODE_SERVICES]; int nsrvs = 0;
  Param* params = nullptr; int nparams = 0;
  bool managed; uint8_t lstate;
  bus::Publisher<rcl_interfaces::msg::ParameterEvent> pe_pub;
  bus::Publisher<lifecycle_msgs::msg::TransitionEvent> te_pub;
  // optional check before a parameter changes (the new value is in p): false + reason refuses it
  std::function<bool(const Param& p, const char*& reason)> on_set_parameter;

  explicit Node(const char* name, bool managed_ = false) : NodeBase(name), managed(managed_),
      lstate(managed_ ? State::PRIMARY_STATE_UNCONFIGURED : State::PRIMARY_STATE_ACTIVE) {
    running = !managed;
    params = (Param*)bus::hooks().alloc(sizeof(Param) * MAX_PARAMS);
    for (int k = 0; k < MAX_PARAMS; k++) new (&params[k]) Param();
    declare_parameter("use_sim_time", sim_time(), "the clock is the simulation's (/clock)", true);
    pe_pub = create_publisher<rcl_interfaces::msg::ParameterEvent>("/parameter_events");
    parameter_services();
    if (managed) {
      char b[NAME]; full_name("~/transition_event", b); te_pub = create_publisher<lifecycle_msgs::msg::TransitionEvent>(b, 4);
      lifecycle_services();
    }
  }
  virtual ~Node() {
    for (int k = 0; k < nsrvs; k++) delete srvs[k];
    for (int k = 0; k < MAX_PARAMS; k++) params[k].~Param();
    bus::hooks().release(params);
  }
  // "~/x" -> "/<node>/x"; "/x" stays
  void full_name(const char* n, char* out) {
    if (n[0] == '~') snprintf(out, NAME, "/%s%s", name, n + 1); else snprintf(out, NAME, "%s", n);
  }
  template <typename Srv> Service<Srv>* create_service(const char* n, std::function<bool(const typename Srv::Request&, typename Srv::Response&, uint32_t)> cb) {
    if (nsrvs >= MAX_NODE_SERVICES) { printf("ros: node %s has too many services\n", name); abort(); }
    Service<Srv>* s = new Service<Srv>(); full_name(n, s->name); s->type = Srv::TYPE; s->node = this; s->cb = cb;
    add_service(s); srvs[nsrvs++] = s; return s;
  }
  template <typename Srv> Client<Srv> create_client(const char* n) { Client<Srv> c; full_name(n, c.name); return c; }

  // ---- parameters
  Param& declare(const char* n, uint8_t type, const char* desc, bool read_only) {
    if (nparams >= MAX_PARAMS) { printf("ros: node %s has too many parameters\n", name); abort(); }
    Param& p = params[nparams++]; snprintf(p.name, sizeof p.name, "%s", n); p.type = type; p.description = desc; p.read_only = read_only;
    return p;
  }
  double& declare_parameter(const char* n, double def, const char* desc, double lo = -INFINITY, double hi = INFINITY, bool ro = false) {
    Param& p = declare(n, ParameterType::PARAMETER_DOUBLE, desc, ro); p.d = def; p.lo = lo; p.hi = hi; return p.d; }
  int64_t& declare_parameter(const char* n, int64_t def, const char* desc, double lo = -INFINITY, double hi = INFINITY, bool ro = false) {
    Param& p = declare(n, ParameterType::PARAMETER_INTEGER, desc, ro); p.i = def; p.lo = lo; p.hi = hi; return p.i; }
  bool& declare_parameter(const char* n, bool def, const char* desc, bool ro = false) {
    Param& p = declare(n, ParameterType::PARAMETER_BOOL, desc, ro); p.b = def; return p.b; }
  char* declare_parameter(const char* n, const char* def, const char* desc, bool ro = false) {
    Param& p = declare(n, ParameterType::PARAMETER_STRING, desc, ro); snprintf(p.s, sizeof p.s, "%s", def); return p.s; }
  void declare_parameter(const char* n, const char* const* list, int count, const char* desc) {
    Param& p = declare(n, ParameterType::PARAMETER_STRING_ARRAY, desc, true); p.sarr = list; p.nsarr = count; }
  Param* find_param(const char* n) { for (int k = 0; k < nparams; k++) if (strcmp(params[k].name, n) == 0) return &params[k]; return nullptr; }
  // one value from set_parameters: checks, then stores it
  bool set_param(const rcl_interfaces::msg::Parameter& in, const char*& reason) {
    Param q; Param* p = check_param(in, q, reason);
    if (!p) return false;
    p->b = q.b; p->i = q.i; p->d = q.d; memcpy(p->s, q.s, sizeof p->s);
    return true;
  }
  // the declared parameter and its new value in q, or nullptr + reason
  Param* check_param(const rcl_interfaces::msg::Parameter& in, Param& q, const char*& reason) {
    Param* p = find_param(in.name.c_str());
    if (!p) { reason = "parameter not declared"; return nullptr; }
    if (p->read_only) { reason = "parameter is read only"; return nullptr; }
    const rcl_interfaces::msg::ParameterValue& v = in.value;
    q = *p;
    if (p->type == ParameterType::PARAMETER_DOUBLE && v.type == ParameterType::PARAMETER_DOUBLE) q.d = v.double_value;
    else if (p->type == ParameterType::PARAMETER_DOUBLE && v.type == ParameterType::PARAMETER_INTEGER) q.d = (double)v.integer_value;
    else if (p->type == ParameterType::PARAMETER_INTEGER && v.type == ParameterType::PARAMETER_INTEGER) q.i = v.integer_value;
    else if (p->type == ParameterType::PARAMETER_BOOL && v.type == ParameterType::PARAMETER_BOOL) q.b = v.bool_value;
    else if (p->type == ParameterType::PARAMETER_STRING && v.type == ParameterType::PARAMETER_STRING) snprintf(q.s, sizeof q.s, "%s", v.string_value.c_str());
    else { reason = "wrong parameter type"; return nullptr; }
    double x = q.type == ParameterType::PARAMETER_DOUBLE ? q.d : (double)q.i;
    if ((q.type == ParameterType::PARAMETER_DOUBLE || q.type == ParameterType::PARAMETER_INTEGER) && (x < q.lo || x > q.hi)) {
      reason = "value out of range"; return nullptr; }
    if (on_set_parameter && !on_set_parameter(q, reason)) return nullptr;
    return p;
  }
  // /parameter_events: the parameters named in `changed` (that changed)
  void parameter_event(const Param* const* changed, int n) {
    if (n <= 0) return;
    rcl_interfaces::msg::ParameterEvent& e = pe_pub.loan();
    int64_t ns = bus::now_ns(); e.stamp.sec = (int32_t)(ns / 1000000000); e.stamp.nanosec = (uint32_t)(ns % 1000000000);
    char b[NAME]; snprintf(b, sizeof b, "/%s", name); e.node.set(b);
    e.new_parameters.size = 0; e.deleted_parameters.size = 0; e.changed_parameters.size = 0;
    for (int k = 0; k < n && k < (int)e.changed_parameters.capacity; k++) {
      e.changed_parameters[k].name.set(changed[k]->name); changed[k]->to_msg(e.changed_parameters[k].value); e.changed_parameters.size = k + 1;
    }
    pe_pub.publish();
  }
  void parameter_services() {
    using namespace rcl_interfaces::srv;
    create_service<GetParameters>("~/get_parameters", [this](const GetParameters_Request& q, GetParameters_Response& r, uint32_t) {
      for (uint32_t k = 0; k < q.names.size && k < r.values.capacity; k++) {
        Param* p = find_param(q.names[k].c_str());
        if (p) p->to_msg(r.values[k]); else r.values[k].type = ParameterType::PARAMETER_NOT_SET;
        r.values.size = k + 1;
      }
      return true; });
    create_service<SetParameters>("~/set_parameters", [this](const SetParameters_Request& q, SetParameters_Response& r, uint32_t) {
      const Param* changed[8]; int nc = 0;
      for (uint32_t k = 0; k < q.parameters.size && k < r.results.capacity; k++) {
        const char* reason = "";
        r.results[k].successful = set_param(q.parameters[k], reason); r.results[k].reason.set(reason); r.results.size = k + 1;
        if (r.results[k].successful && nc < 8) changed[nc++] = find_param(q.parameters[k].name.c_str());
      }
      parameter_event(changed, nc);
      return true; });
    create_service<SetParametersAtomically>("~/set_parameters_atomically", [this](const SetParametersAtomically_Request& q, SetParametersAtomically_Response& r, uint32_t) {
      Param vals[8]; Param* ps[8]; const Param* changed[8]; int n = 0; const char* reason = "";
      for (uint32_t k = 0; k < q.parameters.size; k++) {          // all must pass, else none is set
        if (n >= 8) { reason = "too many parameters"; n = -1; break; }
        ps[n] = check_param(q.parameters[k], vals[n], reason);
        if (!ps[n]) { n = -1; break; }
        n++;
      }
      r.result.successful = n >= 0; r.result.reason.set(reason);
      for (int k = 0; k < n; k++) { Param* p = ps[k]; p->b = vals[k].b; p->i = vals[k].i; p->d = vals[k].d; memcpy(p->s, vals[k].s, sizeof p->s); changed[k] = p; }
      parameter_event(changed, n);
      return true; });
    create_service<DescribeParameters>("~/describe_parameters", [this](const DescribeParameters_Request& q, DescribeParameters_Response& r, uint32_t) {
      for (uint32_t k = 0; k < q.names.size && k < r.descriptors.capacity; k++) {
        rcl_interfaces::msg::ParameterDescriptor& d = r.descriptors[k]; r.descriptors.size = k + 1;
        d.name.set(q.names[k].c_str());
        Param* p = find_param(q.names[k].c_str());
        if (!p) { d.type = ParameterType::PARAMETER_NOT_SET; continue; }
        d.type = p->type; d.description.set(p->description); d.read_only = p->read_only;
        if (p->type == ParameterType::PARAMETER_DOUBLE && (isfinite(p->lo) || isfinite(p->hi))) {
          d.floating_point_range.size = 1; d.floating_point_range[0].from_value = p->lo; d.floating_point_range[0].to_value = p->hi; }
        if (p->type == ParameterType::PARAMETER_INTEGER && (isfinite(p->lo) || isfinite(p->hi))) {
          d.integer_range.size = 1;
          d.integer_range[0].from_value = isfinite(p->lo) ? (int64_t)p->lo : INT64_MIN; d.integer_range[0].to_value = isfinite(p->hi) ? (int64_t)p->hi : INT64_MAX; }
      }
      return true; });
    create_service<GetParameterTypes>("~/get_parameter_types", [this](const GetParameterTypes_Request& q, GetParameterTypes_Response& r, uint32_t) {
      for (uint32_t k = 0; k < q.names.size && k < r.types.capacity; k++) {
        Param* p = find_param(q.names[k].c_str()); r.types[k] = p ? p->type : (uint8_t)ParameterType::PARAMETER_NOT_SET; r.types.size = k + 1;
      }
      return true; });
    create_service<ListParameters>("~/list_parameters", [this](const ListParameters_Request& q, ListParameters_Response& r, uint32_t) {
      for (int k = 0; k < nparams && r.result.names.size < r.result.names.capacity; k++) {
        bool match = q.prefixes.size == 0;
        for (uint32_t j = 0; j < q.prefixes.size && !match; j++) match = strncmp(params[k].name, q.prefixes[j].c_str(), strlen(q.prefixes[j].c_str())) == 0;
        if (match) r.result.names[r.result.names.size++].set(params[k].name);
      }
      return true; });
  }

  // ---- lifecycle (managed nodes)
  virtual bool on_configure() { return true; }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }
  virtual bool on_cleanup() { return true; }
  virtual bool on_shutdown() { return true; }
  static const char* state_label(uint8_t s) {
    switch (s) {
      case State::PRIMARY_STATE_UNCONFIGURED: return "unconfigured";
      case State::PRIMARY_STATE_INACTIVE: return "inactive";
      case State::PRIMARY_STATE_ACTIVE: return "active";
      case State::PRIMARY_STATE_FINALIZED: return "finalized";
      default: return "unknown";
    }
  }
  // one transition of the ROS 2 lifecycle state machine; false if not allowed here or the callback failed
  bool transition(uint8_t t) {
    uint8_t s = lstate, next = s; bool ok = false;
    switch (t) {
      case Transition::TRANSITION_CONFIGURE: if (s == State::PRIMARY_STATE_UNCONFIGURED) { ok = on_configure(); next = State::PRIMARY_STATE_INACTIVE; } break;
      case Transition::TRANSITION_CLEANUP: if (s == State::PRIMARY_STATE_INACTIVE) { ok = on_cleanup(); next = State::PRIMARY_STATE_UNCONFIGURED; } break;
      case Transition::TRANSITION_ACTIVATE: if (s == State::PRIMARY_STATE_INACTIVE) { ok = on_activate(); next = State::PRIMARY_STATE_ACTIVE; } break;
      case Transition::TRANSITION_DEACTIVATE: if (s == State::PRIMARY_STATE_ACTIVE) { ok = on_deactivate(); next = State::PRIMARY_STATE_INACTIVE; } break;
      case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN: if (s == State::PRIMARY_STATE_UNCONFIGURED) { ok = on_shutdown(); next = State::PRIMARY_STATE_FINALIZED; } break;
      case Transition::TRANSITION_INACTIVE_SHUTDOWN: if (s == State::PRIMARY_STATE_INACTIVE) { ok = on_shutdown(); next = State::PRIMARY_STATE_FINALIZED; } break;
      case Transition::TRANSITION_ACTIVE_SHUTDOWN: if (s == State::PRIMARY_STATE_ACTIVE) { on_deactivate(); ok = on_shutdown(); next = State::PRIMARY_STATE_FINALIZED; } break;
      default: break;
    }
    if (ok) {
      uint8_t mid = transition_state(t);                         // as rclcpp: two events, through the transition state
      transition_event(t, transition_label(t), s, mid);
      transition_event(success_id(t), "transition_success", mid, next);
      lstate = next; running = lstate == State::PRIMARY_STATE_ACTIVE;
    }
    return ok;
  }
  static const char* transition_label(uint8_t t) {
    switch (t) {
      case Transition::TRANSITION_CONFIGURE: return "configure";
      case Transition::TRANSITION_CLEANUP: return "cleanup";
      case Transition::TRANSITION_ACTIVATE: return "activate";
      case Transition::TRANSITION_DEACTIVATE: return "deactivate";
      case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN: case Transition::TRANSITION_INACTIVE_SHUTDOWN:
      case Transition::TRANSITION_ACTIVE_SHUTDOWN: return "shutdown";
      default: return "unknown";
    }
  }
  static uint8_t transition_state(uint8_t t) {                   // lifecycle_msgs/State TRANSITION_STATE_*
    switch (t) {
      case Transition::TRANSITION_CONFIGURE: return State::TRANSITION_STATE_CONFIGURING;
      case Transition::TRANSITION_CLEANUP: return State::TRANSITION_STATE_CLEANINGUP;
      case Transition::TRANSITION_ACTIVATE: return State::TRANSITION_STATE_ACTIVATING;
      case Transition::TRANSITION_DEACTIVATE: return State::TRANSITION_STATE_DEACTIVATING;
      default: return State::TRANSITION_STATE_SHUTTINGDOWN;
    }
  }
  static uint8_t success_id(uint8_t t) {                         // lifecycle_msgs/Transition TRANSITION_ON_*_SUCCESS
    switch (t) {
      case Transition::TRANSITION_CONFIGURE: return 10;
      case Transition::TRANSITION_CLEANUP: return 20;
      case Transition::TRANSITION_ACTIVATE: return 40;
      case Transition::TRANSITION_DEACTIVATE: return 50;
      default: return 30;
    }
  }
  static const char* any_label(uint8_t s) {
    switch (s) {
      case State::TRANSITION_STATE_CONFIGURING: return "configuring";
      case State::TRANSITION_STATE_CLEANINGUP: return "cleaningup";
      case State::TRANSITION_STATE_SHUTTINGDOWN: return "shuttingdown";
      case State::TRANSITION_STATE_ACTIVATING: return "activating";
      case State::TRANSITION_STATE_DEACTIVATING: return "deactivating";
      default: return state_label(s);
    }
  }
  void transition_event(uint8_t id, const char* label, uint8_t from, uint8_t to) {
    lifecycle_msgs::msg::TransitionEvent& e = te_pub.loan();
    e.timestamp = (uint64_t)bus::now_ns(); e.transition.id = id; e.transition.label.set(label);
    e.start_state.id = from; e.start_state.label.set(any_label(from)); e.goal_state.id = to; e.goal_state.label.set(any_label(to));
    te_pub.publish();
  }
  // the transitions allowed from the current state (as `ros2 lifecycle list <node>`)
  int available(uint8_t* out) const {
    switch (lstate) {
      case State::PRIMARY_STATE_UNCONFIGURED: out[0] = Transition::TRANSITION_CONFIGURE; out[1] = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN; return 2;
      case State::PRIMARY_STATE_INACTIVE: out[0] = Transition::TRANSITION_CLEANUP; out[1] = Transition::TRANSITION_ACTIVATE;
                                          out[2] = Transition::TRANSITION_INACTIVE_SHUTDOWN; return 3;
      case State::PRIMARY_STATE_ACTIVE: out[0] = Transition::TRANSITION_DEACTIVATE; out[1] = Transition::TRANSITION_ACTIVE_SHUTDOWN; return 2;
      default: return 0;
    }
  }
  void lifecycle_services() {
    using namespace lifecycle_msgs::srv;
    create_service<ChangeState>("~/change_state", [this](const ChangeState_Request& q, ChangeState_Response& r, uint32_t) {
      r.success = transition(q.transition.id); return true; });
    create_service<GetState>("~/get_state", [this](const GetState_Request&, GetState_Response& r, uint32_t) {
      r.current_state.id = lstate; r.current_state.label.set(state_label(lstate)); return true; });
    create_service<GetAvailableTransitions>("~/get_available_transitions", [this](const GetAvailableTransitions_Request&, GetAvailableTransitions_Response& r, uint32_t) {
      uint8_t t[4]; int n = available(t);
      for (int k = 0; k < n && k < (int)r.available_transitions.capacity; k++) {
        lifecycle_msgs::msg::TransitionDescription& d = r.available_transitions[k]; r.available_transitions.size = k + 1;
        d.transition.id = t[k]; d.transition.label.set(transition_label(t[k]));
        d.start_state.id = lstate; d.start_state.label.set(state_label(lstate));
        d.goal_state.id = transition_state(t[k]); d.goal_state.label.set(any_label(transition_state(t[k])));
      }
      return true; });
  }
};

// ------------------------------------------------------------------------------------------------- lifecycle manager
// As nav2_lifecycle_manager: node_names in start order; startup = configure all, then activate all.
struct LifecycleManager : Node {
  const char* names[16]; int n = 0; bool& autostart; bool started = false;
  LifecycleManager(const char* name, std::initializer_list<const char*> nodes)
      : Node(name), autostart(declare_parameter("autostart", true, "start the nodes when the manager starts", true)) {
    for (const char* s : nodes) if (n < 16) names[n++] = s;
    declare_parameter("node_names", names, n, "managed nodes, in start order");
    using nav2_msgs::srv::ManageLifecycleNodes;
    create_service<ManageLifecycleNodes>("~/manage_nodes", [this](const nav2_msgs::srv::ManageLifecycleNodes_Request& q, nav2_msgs::srv::ManageLifecycleNodes_Response& r, uint32_t) {
      r.success = manage(q.command); return true; });
    create_service<std_srvs::srv::Trigger>("~/is_active", [this](const std_srvs::srv::Trigger_Request&, std_srvs::srv::Trigger_Response& r, uint32_t) {
      r.success = all_in(State::PRIMARY_STATE_ACTIVE); r.message.set(r.success ? "all nodes active" : "not all nodes active"); return true; });
  }
  bool change(const char* node, uint8_t t) {                     // through the node's own change_state service
    char srv[NAME]; snprintf(srv, sizeof srv, "/%s/change_state", node);
    Client<lifecycle_msgs::srv::ChangeState> c; snprintf(c.name, sizeof c.name, "%s", srv);
    lifecycle_msgs::srv::ChangeState_Request q; lifecycle_msgs::srv::ChangeState_Response r; q.transition.id = t;
    return c.call(q, r) && r.success;
  }
  uint8_t state_of(const char* node) {
    char srv[NAME]; snprintf(srv, sizeof srv, "/%s/get_state", node);
    Client<lifecycle_msgs::srv::GetState> c; snprintf(c.name, sizeof c.name, "%s", srv);
    lifecycle_msgs::srv::GetState_Request q; lifecycle_msgs::srv::GetState_Response r;
    return c.call(q, r) ? r.current_state.id : State::PRIMARY_STATE_UNKNOWN;
  }
  bool all_in(uint8_t s) { for (int k = 0; k < n; k++) if (state_of(names[k]) != s) return false; return true; }
  bool forward(uint8_t t, uint8_t from) { bool ok = true; for (int k = 0; k < n; k++) if (state_of(names[k]) == from) ok = change(names[k], t) && ok; return ok; }
  bool backward(uint8_t t, uint8_t from) { bool ok = true; for (int k = n - 1; k >= 0; k--) if (state_of(names[k]) == from) ok = change(names[k], t) && ok; return ok; }
  bool manage(uint8_t cmd) {
    using M = nav2_msgs::srv::ManageLifecycleNodes_Request;
    switch (cmd) {
      case M::STARTUP: return forward(Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_UNCONFIGURED) &&
                              forward(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_INACTIVE);
      case M::CONFIGURE: return forward(Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_UNCONFIGURED);
      case M::PAUSE: return backward(Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_ACTIVE);
      case M::RESUME: return forward(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_INACTIVE);
      case M::CLEANUP: return backward(Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_INACTIVE);
      case M::RESET: return backward(Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_ACTIVE) &&
                            backward(Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_INACTIVE);
      case M::SHUTDOWN: manage(M::RESET); return backward(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, State::PRIMARY_STATE_UNCONFIGURED);
      default: return false;
    }
  }
  bool startup() { started = true; return manage(nav2_msgs::srv::ManageLifecycleNodes_Request::STARTUP); }
};

// ------------------------------------------------------------------------------------------------------------- actions
using action_msgs::msg::GoalStatus;
struct ActionEntry { char name[NAME]; const char* type; void* server; };
struct ActionRegistry { ActionEntry a[MAX_ACTIONS]; int n = 0; };
inline ActionRegistry& actions() { static ActionRegistry r; return r; }

template <typename A> struct ActionServer {
  using Goal = typename A::Goal; using Result = typename A::Result; using Feedback = typename A::Feedback;
  using FbMsg = typename A::FeedbackMessage; using SendGoal = typename A::SendGoal; using GetResult = typename A::GetResult;
  Node* node; char name[40];                                    // (+ "/_action/cancel_goal" fits a service name)
  Service<SendGoal>* s_send; Service<action_msgs::srv::CancelGoal>* s_cancel; Service<GetResult>* s_result;
  bus::Publisher<FbMsg> fb_pub; bus::Publisher<action_msgs::msg::GoalStatusArray> st_pub;
  // the server's side
  std::function<bool(const Goal&)> handle_goal;                  // accept the goal? (default: yes)
  std::function<void(const Goal&)> handle_accepted;              // start working on it (status is "executing")
  std::function<bool()> handle_cancel;                           // accept a cancel request? then call canceled() later
  // the goal
  uint8_t uuid[16] = {0}; int8_t status = GoalStatus::STATUS_UNKNOWN; int64_t stamp_ns = 0; uint32_t goals = 0;
  Goal* goal; Result* result;
  std::function<void(int8_t status, const Result&)> local_done;  // the client in the chip (see ActionClient)
  uint32_t remote_wait[4]; int nwait = 0;                         // get_result calls from the PC, answered at the end
  uint8_t reply_buf[512];
  static const int KEEP = 4;                                     // ended goals whose result can still be asked for
  struct Ended { uint8_t uuid[16]; int8_t status; Result* result; } ended[KEEP]; int nended = 0;

  ActionServer(Node* n, const char* action_name) : node(n), goal(alloc_msg<Goal>()), result(alloc_msg<Result>()) {
    for (int k = 0; k < KEEP; k++) { ended[k].result = alloc_msg<Result>(); ended[k].status = GoalStatus::STATUS_UNKNOWN; memset(ended[k].uuid, 0, 16); }
    snprintf(name, sizeof name, "%s", action_name);
    char b[NAME];
    snprintf(b, sizeof b, "%s/_action/send_goal", name);
    s_send = node->create_service<SendGoal>(b, [this](const typename SendGoal::Request& q, typename SendGoal::Response& r, uint32_t) {
      r.accepted = accept(q.goal_id.uuid, q.goal, nullptr); r.stamp.sec = (int32_t)(stamp_ns / 1000000000); r.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000);
      return true; });
    snprintf(b, sizeof b, "%s/_action/cancel_goal", name);
    s_cancel = node->create_service<action_msgs::srv::CancelGoal>(b, [this](const action_msgs::srv::CancelGoal_Request& q, action_msgs::srv::CancelGoal_Response& r, uint32_t) {
      r.return_code = cancel(q.goal_info.goal_id.uuid);
      if (r.return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE) { r.goals_canceling.size = 1; memcpy(r.goals_canceling[0].goal_id.uuid, uuid, 16); }
      return true; });
    snprintf(b, sizeof b, "%s/_action/get_result", name);
    s_result = node->create_service<GetResult>(b, [this](const typename GetResult::Request& q, typename GetResult::Response& r, uint32_t call_id) {
      if (memcmp(q.goal_id.uuid, uuid, 16) != 0) {               // an earlier goal?
        r.status = GoalStatus::STATUS_UNKNOWN;
        for (int k = 0; k < nended; k++) if (memcmp(q.goal_id.uuid, ended[k].uuid, 16) == 0) { r.status = ended[k].status; r.result = *ended[k].result; }
        return true;
      }
      if (running()) { if (call_id && nwait < 4) { remote_wait[nwait++] = call_id; return false; } r.status = status; return true; }
      r.status = status; r.result = *result; return true; });
    snprintf(b, sizeof b, "%s/_action/feedback", name); fb_pub = node->create_publisher<FbMsg>(b);
    snprintf(b, sizeof b, "%s/_action/status", name); st_pub = node->create_publisher<action_msgs::msg::GoalStatusArray>(b, 1, true);
    ActionRegistry& r = actions();
    if (r.n < MAX_ACTIONS) { snprintf(r.a[r.n].name, NAME, "%s", name); r.a[r.n].type = A::TYPE; r.a[r.n].server = this; r.n++; }
  }
  ~ActionServer() {
    ActionRegistry& r = actions();
    for (int i = 0; i < r.n; i++) if (r.a[i].server == this) { for (int k = i; k + 1 < r.n; k++) r.a[k] = r.a[k + 1]; r.n--; break; }
    free_msg(goal); free_msg(result);
    for (int k = 0; k < KEEP; k++) free_msg(ended[k].result);
  }
  bool running() const { return status == GoalStatus::STATUS_ACCEPTED || status == GoalStatus::STATUS_EXECUTING || status == GoalStatus::STATUS_CANCELING; }
  void publish_status() {
    action_msgs::msg::GoalStatusArray& a = st_pub.loan(); a.status_list.size = 1;
    memcpy(a.status_list[0].goal_info.goal_id.uuid, uuid, 16);
    a.status_list[0].goal_info.stamp.sec = (int32_t)(stamp_ns / 1000000000); a.status_list[0].goal_info.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000);
    a.status_list[0].status = status; st_pub.publish();
  }
  // a new goal (from send_goal, or from a client in the chip with its result callback)
  bool accept(const uint8_t* id, const Goal& g, std::function<void(int8_t, const Result&)> done) {
    if (handle_goal && !handle_goal(g)) return false;
    if (running()) { clear_msg(result); result->error_msg.set("preempted by a new goal"); finish(GoalStatus::STATUS_ABORTED); }
    memcpy(uuid, id, 16); *goal = g; clear_msg(result); stamp_ns = bus::now_ns(); goals++;
    local_done = done; nwait = 0;
    status = GoalStatus::STATUS_EXECUTING; publish_status();
    if (handle_accepted) handle_accepted(*goal);
    return true;
  }
  int8_t cancel(const uint8_t* id) {
    using C = action_msgs::srv::CancelGoal_Response;
    if (memcmp(id, uuid, 16) != 0 && !(id[0] == 0 && memcmp(id, id + 1, 15) == 0)) return C::ERROR_UNKNOWN_GOAL_ID;   // zero id: the current goal
    if (!running()) return C::ERROR_GOAL_TERMINATED;
    if (handle_cancel && !handle_cancel()) return C::ERROR_REJECTED;
    status = GoalStatus::STATUS_CANCELING; publish_status(); return C::ERROR_NONE;
  }
  bool is_canceling() const { return status == GoalStatus::STATUS_CANCELING; }
  void publish_feedback(const Feedback& f) { FbMsg& m = fb_pub.loan(); memcpy(m.goal_id.uuid, uuid, 16); m.feedback = f; fb_pub.publish(); }
  void succeed(const Result& r) { *result = r; finish(GoalStatus::STATUS_SUCCEEDED); }
  void abort(const Result& r) { *result = r; finish(GoalStatus::STATUS_ABORTED); }
  void canceled(const Result& r) { *result = r; finish(GoalStatus::STATUS_CANCELED); }
  void finish(int8_t st) {
    status = st; publish_status();
    Ended& e = ended[(goals - 1) % KEEP]; memcpy(e.uuid, uuid, 16); e.status = st; *e.result = *result;
    if (nended < KEEP) nended++;
    if (nwait && remote_reply()) {                               // get_result calls from the PC that waited
      typename GetResult::Response* r = alloc_msg<typename GetResult::Response>(); r->status = st; r->result = *result;
      size_t n = to_cdr(*r, reply_buf, sizeof reply_buf); free_msg(r);
      for (int k = 0; k < nwait; k++) remote_reply()(remote_wait[k], reply_buf, n);
    }
    nwait = 0;
    auto cb = local_done; local_done = nullptr;
    if (cb) cb(st, *result);                                     // may send the next goal at once
  }
};

// a client in the chip: the goal goes straight to the server; the result comes through the callback as soon as the
// server ends the goal (while the server runs, in the same cycle)
template <typename A> struct ActionClient {
  char name[NAME] = {0}; ActionServer<A>* server = nullptr; uint32_t next_id = 1;
  bool find() {
    if (server) return true;
    ActionRegistry& r = actions();
    for (int i = 0; i < r.n; i++) if (strcmp(r.a[i].name, name) == 0 && strcmp(r.a[i].type, A::TYPE) == 0) { server = (ActionServer<A>*)r.a[i].server; return true; }
    return false;
  }
  bool send_goal(const typename A::Goal& g, std::function<void(int8_t status, const typename A::Result&)> done) {
    if (!find()) return false;
    uint8_t id[16] = {0}; uint32_t k = next_id++; memcpy(id, &k, 4); id[15] = 0xC1;   // goal id from this client
    return server->accept(id, g, done);
  }
  bool cancel() { return find() && server->cancel(server->uuid) == action_msgs::srv::CancelGoal_Response::ERROR_NONE; }
};

// `ros2 service list -t` + `ros2 action list -t`, line by line
inline void graph_print(void (*out)(const char*)) {
  char line[160]; Registry& r = registry();
  snprintf(line, sizeof line, "services (%d):", r.n); out(line);
  for (int i = 0; i < r.n; i++) { snprintf(line, sizeof line, "  %-44s %s%s", r.s[i]->name, r.s[i]->type, r.s[i]->calls ? "" : ""); out(line); }
  ActionRegistry& a = actions();
  snprintf(line, sizeof line, "actions (%d):", a.n); out(line);
  for (int i = 0; i < a.n; i++) { snprintf(line, sizeof line, "  %-44s %s", a.a[i].name, a.a[i].type); out(line); }
}

}  // namespace ros
