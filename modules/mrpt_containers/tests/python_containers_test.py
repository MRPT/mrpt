#!/usr/bin/env python3
"""Unit tests for mrpt.containers.YAML Python bindings."""
import sys
import tempfile
import os

try:
    from mrpt.containers import YAML
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.containers bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.containers import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

# ---------------------------------------------------------------------------
print("YAML — construction")
y_empty = YAML()
check("default empty",       y_empty.empty())
check("default isNullNode",  y_empty.isNullNode())

src = "key: 42\nname: Alice\npi: 3.14\nflag: true\n"
y = YAML.from_string(src)
check("parse from string, not empty", not y.empty())
check("parse from string, isMap",     y.isMap())

# ---------------------------------------------------------------------------
print("\nYAML — type queries")
check("isMap True",       y.isMap())
check("isScalar False",   not y.isScalar())
check("isSequence False", not y.isSequence())
check("isNullNode False", not y.isNullNode())

# ---------------------------------------------------------------------------
print("\nYAML — has / __contains__ / keys")
check("has('key')",           y.has("key"))
check("has('missing') False", not y.has("missing"))
check("'key' in y",           "key" in y)
check("'missing' not in y",   "missing" not in y)
ks = y.keys()
check("keys returns list",    isinstance(ks, list))
check("all keys present",     set(ks) == {"key", "name", "pi", "flag"})

# ---------------------------------------------------------------------------
print("\nYAML — size / len")
check("size() == 4",  y.size() == 4)
check("len(y) == 4",  len(y) == 4)
check("empty() False", not y.empty())

# ---------------------------------------------------------------------------
print("\nYAML — __getitem__ + typed extractors")
check("['key'].as_int()",   y["key"].as_int()   == 42)
check("['name'].as_str()",  y["name"].as_str()  == "Alice")
check("['pi'].as_float()",  abs(y["pi"].as_float() - 3.14) < 1e-9)
check("['flag'].as_bool()", y["flag"].as_bool() == True)

try:
    _ = y["missing"]
    check("KeyError on missing key", False, "no exception raised")
except KeyError:
    check("KeyError on missing key", True)

# ---------------------------------------------------------------------------
print("\nYAML — getOrDefault helpers")
check("get_str hits",         y.get_str("name")           == "Alice")
check("get_str default",      y.get_str("x", "def")       == "def")
check("get_float hits",       abs(y.get_float("pi") - 3.14) < 1e-9)
check("get_float default",    y.get_float("x", 1.5)       == 1.5)
check("get_int hits",         y.get_int("key")            == 42)
check("get_int default",      y.get_int("x", 7)           == 7)
check("get_bool hits",        y.get_bool("flag")          == True)
check("get_bool default",     y.get_bool("x", False)      == False)

# ---------------------------------------------------------------------------
print("\nYAML — __setitem__ + round-trip")
y2 = YAML.from_string("a: 1\n")
y2["b"] = YAML.from_string("hello")  # scalar
check("setitem b",        y2.has("b"))

# ---------------------------------------------------------------------------
print("\nYAML — iteration (__iter__)")
keys_via_iter = list(y)
check("__iter__ yields all keys", set(keys_via_iter) == {"key", "name", "pi", "flag"})

# Iterating a non-map raises TypeError
try:
    scalar = YAML.from_string("42")
    _ = list(scalar)
    check("TypeError on non-map iter", False, "no exception")
except TypeError:
    check("TypeError on non-map iter", True)

# ---------------------------------------------------------------------------
print("\nYAML — nested maps")
nested_src = "robot:\n  name: R2D2\n  speed: 3.14\n"
yn = YAML.from_string(nested_src)
robot = yn["robot"]
check("nested isMap",       robot.isMap())
check("nested keys",        set(robot.keys()) == {"name", "speed"})
check("nested str value",   robot["name"].as_str() == "R2D2")
check("nested float value", abs(robot["speed"].as_float() - 3.14) < 1e-9)

# ---------------------------------------------------------------------------
print("\nYAML — sequence nodes")
seq_src = "nums:\n  - 1\n  - 2\n  - 3\n"
ys = YAML.from_string(seq_src)
nums = ys["nums"]
check("sequence isSequence", nums.isSequence())
check("sequence size == 3",  nums.size() == 3)

# push_back on sequence — node must be initialized as a sequence first
y_seq = YAML.from_string("[]")
y_seq.push_back(10.0)
y_seq.push_back(20.0)
check("push_back numeric size", y_seq.size() == 2)

y_str_seq = YAML.from_string("[]")
y_str_seq.push_back_str("hello")
y_str_seq.push_back_str("world")
check("push_back_str size", y_str_seq.size() == 2)

# ---------------------------------------------------------------------------
print("\nYAML — to_string / save_to_file / from_file")
txt = y.to_string()
check("to_string returns str", isinstance(txt, str))
check("to_string has key",     "key" in txt)
check("to_string has Alice",   "Alice" in txt)

with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
    fname = f.name
try:
    y.save_to_file(fname)
    y_loaded = YAML.from_file(fname)
    check("from_file not empty",      not y_loaded.empty())
    check("from_file get_str",        y_loaded.get_str("name") == "Alice")
    check("from_file get_int",        y_loaded.get_int("key")  == 42)
finally:
    os.unlink(fname)

try:
    YAML().save_to_file("/nonexistent_dir/x.yaml")
    check("save_to_file bad path raises", False)
except RuntimeError:
    check("save_to_file bad path raises", True)

# ---------------------------------------------------------------------------
print("\nYAML — JSON input (autodetected)")
json_src = '{"x": 1, "y": 2}'
yj = YAML.from_string(json_src)
check("JSON parse isMap",  yj.isMap())
check("JSON x == 1",       yj.get_int("x") == 1)
check("JSON y == 2",       yj.get_int("y") == 2)

# ---------------------------------------------------------------------------
print("\nYAML — clear / empty after clear")
yc = YAML.from_string("a: 1\n")
check("before clear not empty", not yc.empty())
yc.clear()
check("after clear is empty",   yc.empty())

# ---------------------------------------------------------------------------
print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
