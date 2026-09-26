#!/usr/bin/env python3
"""Smoke tests for mrpt.topography Python bindings."""
import sys, math

try:
    from mrpt.topography import (
        TGeodeticCoords, geodeticToENU_WGS84, ENUToGeodetic_WGS84,
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

print("ENUToGeodetic_WGS84 round-trip")
back_pt = ENUToGeodetic_WGS84(enu, origin)
check("ENU->geodetic lat", abs(back_pt.lat.decimal_value - 37.001) < 1e-9)
check("ENU->geodetic lon", abs(back_pt.lon.decimal_value - (-6.0)) < 1e-9)
check("ENU->geodetic height", abs(back_pt.height - 10.0) < 1e-4)

print("geodeticToGeocentric round-trip")
ecef = geodeticToGeocentric_WGS84(origin)
check("ECEF x non-zero", abs(ecef.x) > 1e5)
back = geocentricToGeodetic(ecef)
check("round-trip lat", abs(back.lat.decimal_value - 37.0) < 1e-5)
check("round-trip lon", abs(back.lon.decimal_value - (-6.0)) < 1e-5)

print("UTM")
import mrpt.topography as topo
madrid = TGeodeticCoords(40.3154333, -3.4857166, 50.0)
utm, zone, band = topo.geodeticToUTM(madrid)
check("UTM x", abs(utm.x - 458731) < 1.0, f"got {utm.x}")
check("UTM y", abs(utm.y - 4462881) < 1.0, f"got {utm.y}")
check("UTM zone/band", zone == 30 and band == "T", f"got {zone}{band}")
back_utm = topo.UTMToGeodetic(utm, zone, "N")
check("UTM round-trip lat", abs(back_utm.lat.decimal_value - 40.3154333) < 1e-6)
check("UTM round-trip lon", abs(back_utm.lon.decimal_value - (-3.4857166)) < 1e-6)
check("UTM round-trip height", abs(back_utm.height - 50.0) < 1e-9)

print("TEllipsoid")
wgs84 = topo.TEllipsoid.Ellipsoid_WGS84()
check("WGS84 semiaxis", abs(wgs84.sa - 6378137.0) < 1e-6 and wgs84.name == "WGS84")
grs80 = topo.TEllipsoid.Ellipsoid_GRS80()
ecef_grs80 = topo.geodeticToGeocentric(origin, grs80)
check("GRS80 ~ WGS84", abs(ecef_grs80.x - ecef.x) < 1e-3, f"{ecef_grs80.x} vs {ecef.x}")
back_grs80 = geocentricToGeodetic(ecef_grs80, grs80)
check("GRS80 round-trip", abs(back_grs80.lat.decimal_value - 37.0) < 1e-6)

print("ENU_axes_from_WGS84")
axes = topo.ENU_axes_from_WGS84(origin)
check("ENU frame origin is ECEF point", abs(axes.x - ecef.x) < 1e-3, f"{axes.x} vs {ecef.x}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
