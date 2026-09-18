#!/usr/bin/env python3
#                    _
#                   | |    Mobile Robot Programming Toolkit (MRPT)
#  _ __ ___  _ __ _ | |_
# | '_ ` _ \| '__| '_ \ __|         https://www.mrpt.org/
# | | | | | | |  | |_) | |_
# |_| |_| |_|_|  | .__/ \__|    https://github.com/MRPT/mrpt/
#                | |
#                |_|
#
# Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
# See: https://www.mrpt.org/Authors - All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Regenerate src/embedded_icon_font.cpp from the Material Symbols font.

The upstream variable font is ~11 MB and covers ~3600 icons, far too large to
embed. This script instantiates it at a single weight/style and subsets it to
the icons listed in ICONS below, which brings it down to a few tens of KB.

To add an icon: append its ICON_MS_* suffix to ICONS and re-run. Requires
fonttools and a C++ compiler.

Usage:
    python3 scripts/regenerate_icon_font.py
"""

import re
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
ICONS_HEADER = PKG / "3rdparty" / "IconFontCppHeaders" / "IconsMaterialSymbols.h"
B2C_SRC = PKG / "3rdparty" / "imgui" / "misc" / "fonts" / "binary_to_compressed_c.cpp"
OUT_CPP = PKG / "src" / "embedded_icon_font.cpp"

FONT_URL = (
    "https://github.com/google/material-design-icons/raw/master/variablefont/"
    "MaterialSymbolsOutlined%5BFILL,GRAD,opsz,wght%5D.ttf"
)

# Fixed instance of the variable font. wght=400 matches ImGui's default UI text.
AXIS_LIMITS = {"FILL": 0, "GRAD": 0, "opsz": 24, "wght": 400}

# Curated set: the actions the six GUI apps need on toolbars and menus.
# Extend as the port progresses; every addition costs ~0.3 KB.
ICONS = [
    # File
    "FOLDER_OPEN", "SAVE", "SAVE_AS", "FILE_OPEN", "DESCRIPTION", "CLOSE",
    "DELETE", "CONTENT_COPY", "CONTENT_PASTE", "CONTENT_CUT", "PRINT",
    # Playback / simulation
    "PLAY_ARROW", "PAUSE", "STOP", "SKIP_NEXT", "SKIP_PREVIOUS",
    "FAST_FORWARD", "FAST_REWIND", "REPLAY", "LOOP", "SPEED",
    # Edit
    "UNDO", "REDO", "EDIT", "ADD", "REMOVE", "SEARCH", "FILTER_ALT",
    "CHECK", "CLEAR", "REFRESH",
    # View / camera
    "ZOOM_IN", "ZOOM_OUT", "FIT_SCREEN", "VISIBILITY", "VISIBILITY_OFF",
    "GRID_ON", "GRID_OFF", "LAYERS", "PHOTO_CAMERA", "VIDEOCAM",
    "CENTER_FOCUS_STRONG", "OPEN_IN_FULL", "CLOSE_FULLSCREEN",
    # Navigation / robotics
    "NAVIGATION", "MY_LOCATION", "LOCATION_ON", "ROUTE", "EXPLORE",
    "SENSORS", "RADAR", "PRECISION_MANUFACTURING", "SETTINGS_INPUT_ANTENNA",
    "STRAIGHTEN", "STRAIGHT",
    # Data / plots
    "SHOW_CHART", "BAR_CHART", "SCATTER_PLOT", "TIMELINE", "TABLE_CHART",
    "DATA_OBJECT", "MAP", "TERRAIN",
    # App chrome
    "SETTINGS", "TUNE", "MENU", "MORE_VERT", "HELP", "INFO", "WARNING",
    "ERROR", "BUG_REPORT", "TERMINAL", "DASHBOARD", "VIEW_SIDEBAR",
    "DOCK_TO_LEFT", "DOCK_TO_RIGHT", "DOCK_TO_BOTTOM",
    "ARROW_BACK", "ARROW_FORWARD", "ARROW_UPWARD", "ARROW_DOWNWARD",
    "EXPAND_MORE", "EXPAND_LESS", "CHEVRON_LEFT", "CHEVRON_RIGHT",
    # Battery: the 0..6 bar level indicators plus the notable states.
    "BATTERY_0_BAR", "BATTERY_1_BAR", "BATTERY_2_BAR", "BATTERY_3_BAR",
    "BATTERY_4_BAR", "BATTERY_5_BAR", "BATTERY_6_BAR", "BATTERY_FULL",
    "BATTERY_CHARGING_FULL", "BATTERY_ALERT", "BATTERY_LOW",
    "BATTERY_VERY_LOW", "BATTERY_ERROR", "BATTERY_UNKNOWN", "BATTERY_SAVER",
    # Status / hardware
    "MEMORY", "BOLT", "ACCESS_TIME", "KEYBOARD", "MOUSE",
    "INPUT", "PERM_DATA_SETTING", "SETTINGS_APPLICATIONS", "BUILD",
    # Geometry / rendering
    "3D_ROTATION", "BLUR_ON", "LENS_BLUR", "GRADIENT", "CAMERA_ALT",
    # Routing
    "ALT_ROUTE", "FORK_RIGHT", "ADS_CLICK",
    # Documents / annotations
    "ARTICLE", "COMMENT", "FACT_CHECK", "DONE", "DOWNLOAD_FOR_OFFLINE",
    "ADD_CHART", "PLUS_ONE",
    # Misc
    "ROCKET_LAUNCH", "SCHOOL", "SCIENCE",
]


def parse_codepoints() -> dict:
    """Map ICON_MS_<NAME> -> codepoint int, from IconFontCppHeaders."""
    pattern = re.compile(r"#define\s+ICON_MS_(\S+)\s+\"[^\"]+\"\s*//\s*U\+([0-9a-fA-F]+)")
    out = {}
    for line in ICONS_HEADER.read_text(encoding="utf-8").splitlines():
        m = pattern.search(line)
        if m:
            out[m.group(1)] = int(m.group(2), 16)
    return out


def main() -> int:
    if not ICONS_HEADER.exists():
        sys.exit(f"Missing {ICONS_HEADER}; run: git submodule update --init --recursive")

    codepoints = parse_codepoints()
    missing = [n for n in ICONS if n not in codepoints]
    if missing:
        sys.exit(f"Unknown icon name(s) in ICONS: {', '.join(missing)}")

    wanted = sorted({codepoints[n] for n in ICONS})
    print(f"> {len(ICONS)} icons -> {len(wanted)} unique codepoints")

    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        src_ttf = tmp / "variable.ttf"
        print(f"> Downloading {FONT_URL}")
        urllib.request.urlretrieve(FONT_URL, src_ttf)
        print(f"  {src_ttf.stat().st_size / 1e6:.1f} MB")

        # Instantiate the variable font at a single fixed style.
        inst_ttf = tmp / "instance.ttf"
        subprocess.run(
            [sys.executable, "-m", "fontTools.varLib.instancer", str(src_ttf)]
            + [f"{k}={v}" for k, v in AXIS_LIMITS.items()]
            + ["-o", str(inst_ttf)],
            check=True, stdout=subprocess.DEVNULL,
        )

        # Subset to the wanted glyphs only.
        sub_ttf = tmp / "subset.ttf"
        subprocess.run(
            [sys.executable, "-m", "fontTools.subset", str(inst_ttf),
             "--output-file=" + str(sub_ttf),
             "--unicodes=" + ",".join(f"U+{cp:04X}" for cp in wanted),
             "--drop-tables+=DSIG",
             "--name-IDs=*", "--recalc-bounds"],
            check=True, stdout=subprocess.DEVNULL,
        )
        print(f"> Subset font: {sub_ttf.stat().st_size / 1024:.1f} KB")

        # Build imgui's binary_to_compressed_c and emit the array.
        b2c = tmp / "binary_to_compressed_c"
        subprocess.run(["c++", "-O2", "-o", str(b2c), str(B2C_SRC)], check=True)
        res = subprocess.run(
            [str(b2c), "-nocompress", "-u8", str(sub_ttf), "MrptIconFont"],
            check=True, capture_output=True, text=True,
        )
        array_src = res.stdout

    header = f"""/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \\| '__| '_ \\| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \\__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

/* GENERATED FILE - do not edit by hand.
 * Regenerate with: python3 scripts/regenerate_icon_font.py
 *
 * Contents: a subset of Google's Material Symbols Outlined, instantiated at
 * {AXIS_LIMITS}, covering {len(wanted)} icons.
 * Font licensed under Apache-2.0; see LICENSES/material-symbols-Apache-2.0.txt
 */

#include <mrpt/imgui_vendor/icon_font.h>

// clang-format off
namespace
{{
{array_src}}}  // namespace
// clang-format on

namespace mrpt::imgui_vendor
{{
const void* iconFontData() {{ return MrptIconFont_data; }}
unsigned int iconFontDataSize() {{ return MrptIconFont_size; }}
}}  // namespace mrpt::imgui_vendor
"""
    OUT_CPP.write_text(header, encoding="utf-8")
    print(f"> Wrote {OUT_CPP} ({OUT_CPP.stat().st_size / 1024:.1f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
