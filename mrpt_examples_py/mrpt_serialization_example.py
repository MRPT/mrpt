#!/usr/bin/env python3
"""
mrpt_serialization_example.py — binary serialisation with mrpt.serialization.

Demonstrates:
  - objectToBytes / bytesToObject: round-trip binary serialisation via Python bytes
  - CSerializable base class
  - Uses mrpt.poses.CPose3D as a concrete serialisable object
"""

from mrpt.serialization import objectToBytes, bytesToObject
from mrpt.poses import CPose3D

# ---------------------------------------------------------------------------
# Create a pose and serialise it to a Python bytes object
# ---------------------------------------------------------------------------
pose = CPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
print(f"Original pose: {pose}")

raw = objectToBytes(pose)
print(f"Serialised to {len(raw)} bytes")
assert isinstance(raw, bytes)
assert len(raw) > 0

# ---------------------------------------------------------------------------
# Deserialise back and verify
# ---------------------------------------------------------------------------
pose2 = bytesToObject(raw)
print(f"Deserialised: {pose2}")

# The returned object is a shared_ptr<CSerializable>; cast via GetRuntimeClass
cls_name = pose2.GetRuntimeClass().className
print(f"  Runtime class: {cls_name}")
assert "CPose3D" in cls_name, f"Expected CPose3D, got {cls_name}"
print("  Round-trip OK ✓")
