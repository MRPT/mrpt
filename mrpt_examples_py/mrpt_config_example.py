#!/usr/bin/env python3
"""
mrpt_config_example.py — configuration files with mrpt.config.

Demonstrates:
  - CConfigFileMemory: in-memory INI-style configuration (no file I/O)
  - CConfigFile: file-backed configuration (written to a temp file)
  - read_double / read_int / read_string / read_bool: typed getters
  - write: typed setters
  - sections / keys: enumeration
  - setContentFromYAML / getContentAsYAML: YAML round-trip
  - config_parser: pre-processing of config text
"""

import os
import tempfile
from mrpt.config import CConfigFile, CConfigFileMemory, config_parser

# ── CConfigFileMemory — no file I/O ─────────────────────────────────────────
print("── CConfigFileMemory ───────────────────────")
ini = """
[General]
AppName = MRPTDemo
Version = 3.0
Debug   = true

[Camera]
FocalLength = 1200.5
Width  = 640
Height = 480
"""
cfg = CConfigFileMemory(ini)

app  = cfg.read_string("General", "AppName", "")
ver  = cfg.read_double("General", "Version", 0.0)
dbg  = cfg.read_bool  ("General", "Debug", False)
fl   = cfg.read_double("Camera",  "FocalLength", 0.0)
w    = cfg.read_int   ("Camera",  "Width", 0)

print(f"AppName      = {app!r}")
print(f"Version      = {ver}")
print(f"Debug        = {dbg}")
print(f"FocalLength  = {fl}")
print(f"Width        = {w}")

assert app == "MRPTDemo"
assert abs(ver - 3.0) < 1e-9
assert dbg is True
assert abs(fl - 1200.5) < 1e-6
assert w == 640
print("  typed reads ✓")

# Section / key enumeration
sections = cfg.getAllSections()
print(f"\nSections: {sections}")
assert "General" in sections and "Camera" in sections

keys_gen = cfg.getAllKeys("General")
print(f"Keys in [General]: {keys_gen}")
assert "AppName" in keys_gen

assert cfg.sectionExists("Camera")
assert cfg.keyExists("General", "Version")
assert not cfg.sectionExists("Lidar")
print("  sectionExists / keyExists ✓")

# Modify a value and read it back
cfg.write("Camera", "Width", 1280)
w2 = cfg.read_int("Camera", "Width", 0)
print(f"\nAfter write, Width = {w2}")
assert w2 == 1280
print("  write ✓")

# YAML round-trip
yaml_str = cfg.getContentAsYAML()
print(f"\ngetContentAsYAML() (excerpt):\n{yaml_str[:200]}")

cfg2 = CConfigFileMemory()
cfg2.setContentFromYAML(yaml_str)
assert cfg2.read_string("General", "AppName", "") == "MRPTDemo"
print("  YAML round-trip ✓")

# clear()
cfg.clear()
assert not cfg.sectionExists("General")
print("  clear() ✓")

# ── CConfigFile — file-backed ────────────────────────────────────────────────
print("\n── CConfigFile (temp file) ─────────────────")
tmp_fd, tmp_path = tempfile.mkstemp(suffix=".ini", prefix="mrpt_cfg_test_")
os.close(tmp_fd)
try:
    fc = CConfigFile(tmp_path)
    fc.write("Sensor", "Type", "Lidar")
    fc.write("Sensor", "Range", 100.0)
    fc.writeNow()           # flush to disk immediately

    # Re-open and read back
    fc2 = CConfigFile(tmp_path)
    stype = fc2.read_string("Sensor", "Type", "")
    srange = fc2.read_double("Sensor", "Range", 0.0)
    print(f"Read back: Type={stype!r}, Range={srange}")
    assert stype == "Lidar"
    assert abs(srange - 100.0) < 1e-9
    print("  CConfigFile write + read ✓")
    print(f"  (temp file: {tmp_path})")
finally:
    os.unlink(tmp_path)

# ── config_parser ────────────────────────────────────────────────────────────
print("\n── config_parser ───────────────────────────")
# config_parser substitutes ${VAR} with values defined via @define.
raw = "@define MY_VAR 42\n[Section]\nvalue = ${MY_VAR}\n"
parsed = config_parser(raw)
print(f"config_parser result:\n{parsed}")
assert "42" in parsed
print("  config_parser ran without error ✓")

print("\nAll checks ✓")
