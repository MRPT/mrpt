#!/usr/bin/env python3
"""Smoke tests for mrpt.topography Python bindings."""
import sys, math

try:
    from mrpt.topography import (
        TGeodeticCoords, geodeticToENU_WGS84,
        geodeticToGeocentric_WGS84, geocentricToGeodetic,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.topography bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.topography import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("TGeodeticCoords")
origin = TGeodeticCoords(37.0, -6.0, 10.0)
check("lat", abs(origin.lat.decimal_value - 37.0) < 1e-9)
check("lon", abs(origin.lon.decimal_value - (-6.0)) < 1e-9)
check("height", abs(origin.height - 10.0) < 1e-9)

print("geodeticToENU_WGS84")
point = TGeodeticCoords(37.001, -6.0, 10.0)
enu = geodeticToENU_WGS84(point, origin)
# ~0.001 deg latitude ≈ 111 m north
check("ENU north ~111m", abs(enu.x - 0.0) < 10.0)   # x is East
check("ENU east ~0", 80.0 < enu.y < 130.0, f"y(north)={enu.y:.1f} m")

print("geodeticToGeocentric round-trip")
ecef = geodeticToGeocentric_WGS84(origin)
check("ECEF x non-zero", abs(ecef.x) > 1e5)
back = geocentricToGeodetic(ecef)
check("round-trip lat", abs(back.lat.decimal_value - 37.0) < 1e-5)
check("round-trip lon", abs(back.lon.decimal_value - (-6.0)) < 1e-5)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
