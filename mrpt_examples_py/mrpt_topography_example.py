#!/usr/bin/env python3
"""
mrpt_topography_example.py — geodetic coordinate conversions with mrpt.topography.

Demonstrates:
  - TCoords: degrees/minutes/seconds representation
  - TGeodeticCoords: WGS84 (lat, lon, height) point
  - geodeticToGeocentric_WGS84: WGS84 → ECEF (Earth-Centred, Earth-Fixed)
  - geocentricToGeodetic: ECEF → WGS84 round-trip
  - geodeticToENU_WGS84: WGS84 → local East-North-Up (ENU) frame
"""

from mrpt.topography import (
    TCoords,
    TGeodeticCoords,
    geodeticToGeocentric_WGS84,
    geocentricToGeodetic,
    geodeticToENU_WGS84,
)

# ---------------------------------------------------------------------------
# TCoords — degrees/minutes/seconds helper
# ---------------------------------------------------------------------------
c = TCoords(40, 24, 51.2)          # 40° 24' 51.2"
print(f"TCoords: {c}")
print(f"  decimal: {c.getDecimalValue():.6f}°")
deg, mins, sec = c.getDegMinSec()
print(f"  DMS: {deg}° {mins}' {sec:.2f}\"")

# ---------------------------------------------------------------------------
# TGeodeticCoords — WGS84 position
# ---------------------------------------------------------------------------
# Malaga, Spain (approx.)
malaga = TGeodeticCoords(36.7213, -4.4214, 10.0)   # lat_deg, lon_deg, height_m
print(f"\nWGS84 point: {malaga}")

# ---------------------------------------------------------------------------
# WGS84 → ECEF (geocentric)
# ---------------------------------------------------------------------------
ecef = geodeticToGeocentric_WGS84(malaga)
print(f"\nECEF (ECEF/geocentric, metres):")
print(f"  x={ecef.x:.2f}  y={ecef.y:.2f}  z={ecef.z:.2f}")

# ---------------------------------------------------------------------------
# ECEF → WGS84 round-trip
# ---------------------------------------------------------------------------
back = geocentricToGeodetic(ecef)
print(f"\nRound-trip WGS84: lat={back.lat.getDecimalValue():.6f}°  "
      f"lon={back.lon.getDecimalValue():.6f}°  h={back.height:.2f} m")
assert abs(back.lat.getDecimalValue() - 36.7213) < 1e-5
assert abs(back.lon.getDecimalValue() - -4.4214) < 1e-5
print("  Round-trip OK ✓")

# ---------------------------------------------------------------------------
# geodeticToENU_WGS84 — local East-North-Up frame
# ---------------------------------------------------------------------------
origin = TGeodeticCoords(36.7213, -4.4214, 10.0)
point  = TGeodeticCoords(36.7213 + 0.001, -4.4214 + 0.001, 20.0)   # ~111 m N, ~90 m E

enu = geodeticToENU_WGS84(point, origin)
print(f"\nENU displacement from origin:")
print(f"  East={enu.x:.2f} m  North={enu.y:.2f} m  Up={enu.z:.2f} m")
