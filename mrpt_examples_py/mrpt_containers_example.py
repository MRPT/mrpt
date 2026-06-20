#!/usr/bin/env python3
"""
mrpt_containers_example.py — mrpt.containers.YAML

Demonstrates the MRPT YAML class and shows side-by-side equivalents for
users migrating from yaml-cpp (pyyaml) or PyYAML.

  mrpt.containers.YAML  vs  PyYAML / ruamel-yaml
  ─────────────────────────────────────────────
  Both parse YAML and JSON. Key differences:

  • mrpt.YAML is a C++ object; child access via [] returns another YAML node,
    not a native Python dict/list.  Call .as_str() / .as_int() / .as_float() /
    .as_bool() on a leaf to get a Python scalar.
    (PyYAML returns plain Python dicts/lists/scalars directly.)

  • mrpt.YAML also supports getOrDefault("key", fallback) for safe access.

  • mrpt.YAML integrates with MRPT's CConfigFileBase system, allowing the
    same YAML files to drive C++ algorithms from Python.

  • Sequence nodes must be created via from_string("[]") before push_back;
    there is no integer index operator.
"""

from mrpt.containers import YAML

YAML_SRC = """\
robot:
  name: R2D2
  speed: 3.14
  active: true
  sensors:
    - lidar
    - camera
  pose:
    x: 1.0
    y: 2.5
    theta: 0.785
"""

# ── Parsing ──────────────────────────────────────────────────────────────────
print("── Parsing ─────────────────────────────────")
doc = YAML.from_string(YAML_SRC)

# PyYAML equivalent:
#   import yaml
#   doc = yaml.safe_load(YAML_SRC)           # returns a plain Python dict
#
# mrpt.YAML returns a YAML node (not a plain dict):
print(f"type: {type(doc)}")          # mrpt.containers._bindings.YAML
print(f"isMap: {doc.isMap()}")       # True
print(f"keys: {doc.keys()}")         # ['robot']

# ── Accessing nested values ───────────────────────────────────────────────────
print("\n── Accessing values ────────────────────────")
robot = doc["robot"]

# PyYAML:  name = doc["robot"]["name"]           -> plain str
# mrpt:    name = doc["robot"]["name"].as_str()  -> str via explicit cast
name   = robot["name"].as_str()
speed  = robot["speed"].as_float()
active = robot["active"].as_bool()

print(f"name:   {name!r}")    # 'R2D2'
print(f"speed:  {speed}")     # 3.14
print(f"active: {active}")    # True

assert name == "R2D2"
assert abs(speed - 3.14) < 1e-9
assert active is True

# ── Safe access with defaults ─────────────────────────────────────────────────
print("\n── Safe access (getOrDefault) ───────────────")
# PyYAML:  doc["robot"].get("battery", 100)   -> Python dict .get()
# mrpt:    robot.get_int("battery", 100)       -> typed helper
battery = robot.get_int("battery", 100)
print(f"battery (default 100): {battery}")   # 100
assert battery == 100

# ── Membership test ───────────────────────────────────────────────────────────
print("\n── 'in' operator ────────────────────────────")
# PyYAML:  "name" in doc["robot"]
# mrpt:    "name" in robot   (same syntax, supported via __contains__)
print(f"'name' in robot:    {'name' in robot}")       # True
print(f"'battery' in robot: {'battery' in robot}")    # False

# ── Iteration over map keys ───────────────────────────────────────────────────
print("\n── Iteration ────────────────────────────────")
# PyYAML:  for k, v in doc["robot"].items(): ...
# mrpt:    for k in robot: ...  (yields keys; access values with robot[k])
pose = robot["pose"]
print("pose fields:")
for key in pose:
    print(f"  {key}: {pose[key].as_float():.3f}")

# ── Sequences ────────────────────────────────────────────────────────────────
print("\n── Sequences ────────────────────────────────")
sensors = robot["sensors"]
print(f"isSequence: {sensors.isSequence()}")
print(f"size: {sensors.size()}")
# mrpt has no integer index operator; inspect via to_string or size:
print(f"sensors:\n{sensors.to_string().strip()}")

# Building a sequence from scratch — must start from "[]":
# PyYAML:  lst = [1.0, 2.0, 3.0]             -> plain Python list
# mrpt:    start from an empty sequence node, then push_back
nums = YAML.from_string("[]")
nums.push_back(1.0)
nums.push_back(2.0)
nums.push_back(3.0)
print(f"built numeric sequence size: {nums.size()}")  # 3
assert nums.size() == 3

strs = YAML.from_string("[]")
strs.push_back_str("alpha")
strs.push_back_str("beta")
print(f"string sequence: {strs.to_string().strip()}")

# ── Building a map programmatically ──────────────────────────────────────────
print("\n── Building maps ────────────────────────────")
# PyYAML:  d = {"x": 1.0, "y": 2.0}; yaml.dump(d)
# mrpt:    use __setitem__ with YAML scalar nodes
cfg = YAML()
cfg["x"]     = YAML.from_string("1.0")
cfg["y"]     = YAML.from_string("2.0")
cfg["label"] = YAML.from_string("origin")
print(f"cfg keys:    {cfg.keys()}")
print(f"cfg['label']: {cfg['label'].as_str()}")
assert cfg["label"].as_str() == "origin"

# ── Serialization ─────────────────────────────────────────────────────────────
print("\n── Serialization ────────────────────────────")
# PyYAML:  yaml.dump(doc)
# mrpt:    doc.to_string()
txt = robot["pose"].to_string()
print(f"pose as YAML:\n{txt}")
assert "x" in txt and "y" in txt

# save / load round-trip
import tempfile, os
with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
    fname = f.name
try:
    doc.save_to_file(fname)
    doc2 = YAML.from_file(fname)
    assert doc2["robot"]["name"].as_str() == "R2D2"
    print(f"round-trip OK (saved to {os.path.basename(fname)})")
finally:
    os.unlink(fname)

# ── JSON is also accepted ─────────────────────────────────────────────────────
print("\n── JSON input (auto-detected) ───────────────")
# PyYAML does NOT parse JSON by default; mrpt.YAML auto-detects it:
jdoc = YAML.from_string('{"width": 640, "height": 480}')
print(f"width: {jdoc.get_int('width')}, height: {jdoc.get_int('height')}")
assert jdoc.get_int("width") == 640

print("\nAll checks ✓")
