#!/usr/bin/env python3
"""Generate fixed-size C++ structs from ROS 2 .msg files.

    python3 tools/gen_msgs.py            (from the firmware folder) -> src/ros2_msgs.h

Each <pkg>/<Type>.msg becomes struct <pkg>::msg::<Type> with the same field names, types and order as in ROS 2
(like the C++ types of rosidl, but without dynamic memory):
  bool, byte, char, int8..uint64, float32, float64 -> bool, uint8_t, uint8_t, int8_t..uint64_t, float, double
  string        -> msgcore::Str<N>     (N from capacities.txt, "string" line, or a per-field line)
  T[]           -> msgcore::Seq<T, N>  (N from capacities.txt; .size is the used length)
  T[<=N]        -> msgcore::Seq<T, N>
  T[N]          -> T name[N]
  constants     -> static constexpr members
  default value -> member initialiser
Services (<pkg>/<Name>.srv) and actions (<pkg>/<Name>.action) become the same structs rosidl makes:
  <pkg>::srv::<Name>_Request / _Response and struct <Name> { using Request; using Response; }
  <pkg>::action::<Name>_Goal / _Result / _Feedback, the service and topic types of the action
  (<Name>_SendGoal_Request {goal_id, goal}, _SendGoal_Response {accepted, stamp}, _GetResult_Request {goal_id},
  _GetResult_Response {status, result}, _FeedbackMessage {goal_id, feedback}) and struct <Name> with all of them.
  A struct without fields gets "uint8 structure_needs_at_least_one_member", as in rosidl (so the CDR bytes match).
Every struct has  static constexpr const char* TYPE = "<pkg>/msg/<Type>" (or srv, action), and next to it
  cdr_write(cdr::Writer&, const T&) / cdr_read(cdr::Reader&, T&): ROS 2 CDR bytes (src/cdr.h), so a message can leave
  the chip in the same form as on a ROS 2 network (bridge to a PC, later micro-ROS).
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MSG_DIR = os.path.join(ROOT, "msg")
OUT = os.path.join(ROOT, "src", "ros2_msgs.h")

PRIM = {"bool": "bool", "byte": "uint8_t", "char": "uint8_t", "int8": "int8_t", "uint8": "uint8_t", "int16": "int16_t",
        "uint16": "uint16_t", "int32": "int32_t", "uint32": "uint32_t", "int64": "int64_t", "uint64": "uint64_t",
        "float32": "float", "float64": "double"}


def load_caps():
    caps, string_default = {}, 24
    for line in open(os.path.join(MSG_DIR, "capacities.txt")):
        line = line.split("#")[0].strip()
        if not line:
            continue
        key, val = line.split()
        if key == "string":
            string_default = int(val)
        else:
            caps[key] = int(val)
    return caps, string_default


def parse(pkg, name, path, lines=None, kind="msg"):
    fields, consts = [], []
    for raw in (lines if lines is not None else open(path)):
        line = raw.split("#")[0].strip()
        if not line:
            continue
        m = re.match(r"^(\S+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(\S+)$", line)
        if m:
            consts.append((m.group(1), m.group(2), m.group(3)))
            continue
        parts = line.split()
        ftype, fname = parts[0], parts[1]
        default = parts[2] if len(parts) > 2 else None
        arr = None
        am = re.match(r"^(.+)\[(<=)?(\d*)\]$", ftype)
        if am:
            ftype = am.group(1)
            if am.group(3) == "":
                arr = ("unbounded", None)
            elif am.group(2):
                arr = ("bounded", int(am.group(3)))
            else:
                arr = ("fixed", int(am.group(3)))
        if ftype not in PRIM and ftype != "string":
            if "/" not in ftype:
                ftype = ("std_msgs/Header" if ftype == "Header" else pkg + "/" + ftype)
            elif ftype.count("/") == 2 and ftype.split("/")[1] == "msg":
                ftype = ftype.split("/")[0] + "/" + ftype.split("/")[2]
        fields.append({"type": ftype, "name": fname, "arr": arr, "default": default})
    if not fields:                                       # rosidl: an empty structure still has one member
        fields.append({"type": "uint8", "name": "structure_needs_at_least_one_member", "arr": None, "default": None})
    return {"pkg": pkg, "name": name, "fields": fields, "consts": consts, "kind": kind}


def field(t, n):
    return {"type": t, "name": n, "arr": None, "default": None}


def split_sections(path):
    secs, cur = [], []
    for raw in open(path):
        if raw.strip() == "---":
            secs.append(cur); cur = []
        else:
            cur.append(raw)
    return secs + [cur]


def load_interface(pkg, fname, path, msgs, wrappers):
    """one .msg / .srv / .action file -> entries in msgs (key -> parsed struct) and wrapper structs"""
    base, ext = os.path.splitext(fname)
    if ext == ".msg":
        msgs[pkg + "/" + base] = parse(pkg, base, path)
    elif ext == ".srv":
        rq, rs = split_sections(path)
        msgs[pkg + "/srv/" + base + "_Request"] = parse(pkg, base + "_Request", path, rq, "srv")
        msgs[pkg + "/srv/" + base + "_Response"] = parse(pkg, base + "_Response", path, rs, "srv")
        wrappers.append(("srv", pkg, base, [("Request", base + "_Request"), ("Response", base + "_Response")]))
    elif ext == ".action":
        g, r, f = split_sections(path)
        A = pkg + "/action/" + base
        msgs[A + "_Goal"] = parse(pkg, base + "_Goal", path, g, "action")
        msgs[A + "_Result"] = parse(pkg, base + "_Result", path, r, "action")
        msgs[A + "_Feedback"] = parse(pkg, base + "_Feedback", path, f, "action")
        def synth(name, flds):
            msgs[A + "_" + name] = {"pkg": pkg, "name": base + "_" + name, "fields": flds, "consts": [], "kind": "action"}
        synth("SendGoal_Request", [field("unique_identifier_msgs/UUID", "goal_id"), field(A + "_Goal", "goal")])
        synth("SendGoal_Response", [field("bool", "accepted"), field("builtin_interfaces/Time", "stamp")])
        synth("GetResult_Request", [field("unique_identifier_msgs/UUID", "goal_id")])
        synth("GetResult_Response", [field("int8", "status"), field(A + "_Result", "result")])
        synth("FeedbackMessage", [field("unique_identifier_msgs/UUID", "goal_id"), field(A + "_Feedback", "feedback")])
        wrappers.append(("action_srv", pkg, base + "_SendGoal", [("Request", base + "_SendGoal_Request"), ("Response", base + "_SendGoal_Response")]))
        wrappers.append(("action_srv", pkg, base + "_GetResult", [("Request", base + "_GetResult_Request"), ("Response", base + "_GetResult_Response")]))
        wrappers.append(("action", pkg, base, [("Goal", base + "_Goal"), ("Result", base + "_Result"), ("Feedback", base + "_Feedback"),
                                               ("SendGoal", base + "_SendGoal"), ("GetResult", base + "_GetResult"),
                                               ("FeedbackMessage", base + "_FeedbackMessage")]))


def cpp_type(ftype, caps, key, sdef):
    if ftype in PRIM:
        return PRIM[ftype]
    if ftype == "string":
        return "msgcore::Str<%d>" % caps.get(key + ":string", sdef)
    parts = ftype.split("/")
    if len(parts) == 3:
        return "%s::%s::%s" % tuple(parts)
    return "%s::msg::%s" % (parts[0], parts[1])


def cdr_functions(m, msgs):
    """cdr_write / cdr_read for one message type (field order = .msg order)."""
    T = m["name"]
    wr = ["inline void cdr_write(cdr::Writer& w, const %s& m) {" % T]
    rd = ["inline void cdr_read(cdr::Reader& r, %s& m) {" % T]
    if not m["fields"]:
        wr.append("  (void)w; (void)m;"); rd.append("  (void)r; (void)m;")
    for fl in m["fields"]:
        f, t, arr = fl["name"], fl["type"], fl["arr"]
        prim, nested = t in PRIM, t in msgs
        if arr is None:
            if prim:
                wr.append("  w.put(m.%s);" % f); rd.append("  r.get(m.%s);" % f)
            elif t == "string":
                wr.append("  w.str(m.%s.data);" % f); rd.append("  r.str(m.%s.data, sizeof m.%s.data);" % (f, f))
            else:
                wr.append("  cdr_write(w, m.%s);" % f); rd.append("  cdr_read(r, m.%s);" % f)
        elif arr[0] == "fixed":
            n = arr[1]
            if prim:
                wr.append("  w.array(m.%s, %d);" % (f, n)); rd.append("  r.array(m.%s, %d);" % (f, n))
            elif nested:
                wr.append("  for (int i = 0; i < %d; i++) cdr_write(w, m.%s[i]);" % (n, f))
                rd.append("  for (int i = 0; i < %d; i++) cdr_read(r, m.%s[i]);" % (n, f))
            else:
                sys.exit("string arrays are not supported (%s.%s)" % (T, f))
        else:
            wr.append("  w.put(m.%s.size);" % f)
            rd.append("  m.%s.size = r.count(m.%s.capacity);" % (f, f))
            if prim:
                wr.append("  w.array(m.%s.data, m.%s.size);" % (f, f)); rd.append("  r.array(m.%s.data, m.%s.size);" % (f, f))
            elif nested:
                wr.append("  for (uint32_t i = 0; i < m.%s.size; i++) cdr_write(w, m.%s[i]);" % (f, f))
                rd.append("  for (uint32_t i = 0; i < m.%s.size; i++) cdr_read(r, m.%s[i]);" % (f, f))
            else:                                        # string[]
                wr.append("  for (uint32_t i = 0; i < m.%s.size; i++) w.str(m.%s[i].data);" % (f, f))
                rd.append("  for (uint32_t i = 0; i < m.%s.size; i++) r.str(m.%s[i].data, sizeof m.%s[i].data);" % (f, f, f))
    return wr + ["}"] + rd + ["}"]


def main():
    caps, sdef = load_caps()
    msgs, wrappers = {}, []
    for pkg in sorted(os.listdir(MSG_DIR)):
        d = os.path.join(MSG_DIR, pkg)
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith((".msg", ".srv", ".action")):
                load_interface(pkg, f, os.path.join(d, f), msgs, wrappers)
    # dependency order
    order, seen = [], set()

    def visit(k):
        if k in seen:
            return
        seen.add(k)
        for fl in msgs[k]["fields"]:
            if fl["type"] in msgs:
                visit(fl["type"])
            elif fl["type"] not in PRIM and fl["type"] != "string":
                sys.exit("unknown type %s in %s" % (fl["type"], k))
        order.append(k)
    for k in sorted(msgs):
        visit(k)

    out = ["// Generated by tools/gen_msgs.py from msg/ -- do not edit by hand.",
           "// ROS 2 message types with the same field names, types and order as the ROS 2 interface packages, but with",
           "// fixed capacities (msg/capacities.txt) so that no memory is allocated per message.",
           "#pragma once", "#include <stdint.h>", "#include <string.h>", "#include \"cdr.h\"", "",
           "namespace msgcore {",
           "template <typename T, uint32_t N> struct Seq {          // T[] with a fixed capacity; size = used length",
           "  static constexpr uint32_t capacity = N;",
           "  uint32_t size = 0;",
           "  T data[N];",
           "  T& operator[](uint32_t i) { return data[i]; }",
           "  const T& operator[](uint32_t i) const { return data[i]; }",
           "};",
           "template <uint32_t N> struct Str {                      // string with a fixed capacity (N - 1 characters)",
           "  char data[N] = {0};",
           "  void set(const char* s) { strncpy(data, s, N - 1); data[N - 1] = 0; }",
           "  const char* c_str() const { return data; }",
           "};",
           "}  // namespace msgcore", ""]
    for k in order:
        m = msgs[k]
        kind = m.get("kind", "msg")
        out.append("namespace %s { namespace %s {" % (m["pkg"], kind))
        out.append("struct %s {" % m["name"])
        out.append('  static constexpr const char* TYPE = "%s/%s/%s";' % (m["pkg"], kind, m["name"]))
        for ct, cn, cv in m["consts"]:
            out.append("  static constexpr %s %s = %s;" % (PRIM[ct], cn, cv))
        for fl in m["fields"]:
            key = k + "." + fl["name"]
            t = cpp_type(fl["type"], caps, key, sdef)
            init = ""
            if fl["default"] is not None and fl["arr"] is None:
                init = " = %s" % fl["default"]
            elif fl["arr"] is None and fl["type"] in PRIM:
                init = " = 0" if fl["type"] != "bool" else " = false"
            if fl["arr"] is None:
                out.append("  %s %s%s;" % (t, fl["name"], init))
            elif fl["arr"][0] == "fixed":
                out.append("  %s %s[%d] = {};" % (t, fl["name"], fl["arr"][1]))
            else:
                n = fl["arr"][1] if fl["arr"][0] == "bounded" else caps.get(key)
                if n is None:
                    sys.exit("no capacity for %s (add it to msg/capacities.txt)" % key)
                out.append("  msgcore::Seq<%s, %d> %s;" % (t, n, fl["name"]))
        out.append("};")
        out += cdr_functions(m, msgs)
        out.append("}}  // namespace %s::%s" % (m["pkg"], kind))
        out.append("")
    for wkind, pkg, name, members in wrappers:              # service / action types: the types that belong together
        ns = "srv" if wkind == "srv" else "action"
        out.append("namespace %s { namespace %s {" % (pkg, ns))
        out.append("struct %s {" % name)
        out.append('  static constexpr const char* TYPE = "%s/%s/%s";' % (pkg, ns, name))
        for alias, target in members:
            out.append("  using %s = %s;" % (alias, target))
        out.append("};")
        out.append("}}  // namespace %s::%s" % (pkg, ns))
        out.append("")
    out.append("// every type, for tables: #define X(T) ...  ROS2_MSG_TYPES(X)")
    out.append("#define ROS2_MSG_TYPES(X) " + " ".join("X(%s::%s::%s)" % (msgs[k]["pkg"], msgs[k].get("kind", "msg"), msgs[k]["name"]) for k in order))
    out.append("")
    open(OUT, "w").write("\n".join(out))
    print("wrote %s: %d message types" % (os.path.relpath(OUT, ROOT), len(order)))


if __name__ == "__main__":
    main()
