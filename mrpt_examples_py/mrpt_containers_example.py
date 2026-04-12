#!/usr/bin/env python3
"""
mrpt_containers_example.py — YAML config containers with mrpt.containers.

Demonstrates:
  - mrpt.containers.YAML: parse, query, modify, serialise YAML documents.

Note: for full INI-style config files (with sections/keys and default values)
see mrpt.config (CConfigFileMemory / CConfigFile).
"""

from mrpt.containers import YAML

# ---------------------------------------------------------------------------
# Parse YAML from a string
# ---------------------------------------------------------------------------
src = """
robot:
  name: R2D2
  wheels: 2
  speed_limit: 1.5
sensors:
  - lidar
  - camera
"""
doc = YAML.from_string(src)
print("Parsed YAML:")
print(doc)

# Basic API
print(f"\nhas('robot')  = {doc.has('robot')}")
print(f"has('foobar') = {doc.has('foobar')}")
print(f"size()        = {doc.size()}")
print(f"empty()       = {doc.empty()}")

# ---------------------------------------------------------------------------
# Round-trip: to_string → re-parse
# ---------------------------------------------------------------------------
txt = doc.to_string()
doc2 = YAML.from_string(txt)
print(f"\nRound-trip OK: size={doc2.size()}  ✓")

# ---------------------------------------------------------------------------
# Empty document
# ---------------------------------------------------------------------------
empty = YAML()
print(f"\nEmpty YAML: empty()={empty.empty()}, size={empty.size()}")
empty.clear()
print("  clear() on empty — no error ✓")
