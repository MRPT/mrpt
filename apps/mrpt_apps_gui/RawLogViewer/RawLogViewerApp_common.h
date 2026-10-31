/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

// Constants shared by the RawLogViewerApp source files.

#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/icons.h>

#include <vector>

namespace rlv
{
constexpr const char* APP_NAME = "RawLogViewer";
constexpr const char* CFG_SECT = "RawLogViewer";
constexpr size_t MAX_RECENT_FILES = 10;

constexpr const char* PANEL_TREE = ICON_MS_ACCOUNT_TREE " Rawlog###Rawlog";
constexpr const char* PANEL_SELECTED = ICON_MS_DATA_OBJECT " Selected object###Selected";
constexpr const char* PANEL_INFO = ICON_MS_INFO " Rawlog information###Info";
constexpr const char* PANEL_CONSOLE = ICON_MS_TERMINAL " Console###Console";
constexpr const char* PANEL_VIEW = ICON_MS_TUNE " 3D view###View";
constexpr const char* PANEL_EDIT = ICON_MS_EDIT " Edit rawlog###Edit";
constexpr const char* PANEL_SENSOR_PARAMS = ICON_MS_SENSORS " Sensor parameters###SensorParams";
constexpr const char* PANEL_MOTION_MODEL = ICON_MS_NAVIGATION " Motion model###MotionModel";
constexpr const char* PANEL_RAWMAP = ICON_MS_MAP " Raw map###RawMap";
constexpr const char* PANEL_EXPORT = ICON_MS_FILE_EXPORT " Export to text###Export";

// Tags to know what a file dialog result is for:
constexpr const char* TAG_OPEN = "open";
constexpr const char* TAG_LOAD_PART = "load_part";
constexpr const char* TAG_SAVE = "save";
constexpr const char* TAG_IMAGES_DIR = "images_dir";
constexpr const char* TAG_EXPORT_TXT = "export_txt";
constexpr const char* TAG_RAWMAP_TXT = "rawmap_txt";
constexpr const char* TAG_RAWMAP_3D = "rawmap_3d";
constexpr const char* TAG_RAWMAP_PATH = "rawmap_path";
constexpr const char* TAG_BATCH_LOAD = "batch_load";
constexpr const char* TAG_BATCH_SAVE = "batch_save";

inline const std::vector<mrpt::imgui::FileDialogFilter> RAWLOG_FILTERS = {
    {"Rawlog files", {"rawlog", "rawlog.gz", "rawlog.zst"}},
};
inline const std::vector<mrpt::imgui::FileDialogFilter> TXT_FILTERS = {
    {"Text files", {"txt"}},
};
inline const std::vector<mrpt::imgui::FileDialogFilter> INI_FILTERS = {
    {"Config files", {"ini", "cfg"}},
};
inline const std::vector<mrpt::imgui::FileDialogFilter> SCENE_FILTERS = {
    {"3D scenes", {"3Dscene"}},
};

constexpr const char* KEYBOARD_HELP =
    "Keyboard:\n"
    "  Left / Right          : Previous / next entry\n"
    "  Page Up / Page Down   : Jump backward / forward\n"
    "  Home / End            : First / last entry\n"
    "  Space                 : Play / pause\n"
    "  Ctrl+O                : Open a rawlog\n"
    "  Ctrl+S                : Save the rawlog as...\n"
    "  F1                    : About\n"
    "\n"
    "Mouse (over the 3D view):\n"
    "  Left button + drag    : Orbit the camera\n"
    "  Right/middle + drag   : Pan\n"
    "  Wheel                 : Zoom in / out\n";

constexpr const char* TIPS =
    "- Double-click a sensor in the 'Rawlog information' panel (or use Sensors > Add sensor "
    "panel) to open a panel that follows that sensor along the timeline. Add as many as you "
    "need, and dock them anywhere: the layout is remembered.\n\n"
    "- The timeline (bottom) shows one row of ticks per sensor. Click or drag on it to seek; "
    "Space plays and pauses, and the arrow keys step entry by entry.\n\n"
    "- Rawlogs with externally-stored images need the directory of those images. It is "
    "detected automatically, but it can be overridden in the 'Rawlog information' panel.\n\n"
    "- The 3D view shows the selected entry at the robot pose from odometry, and optionally "
    "the odometry and GPS paths, and the map built with Tools > Raw map.\n\n"
    "- Batch edits on files (cut, filter by label, convert formats, export, ...) are better "
    "done with the 'rawlog-edit' command line tool: it is faster and does not need to load "
    "the whole dataset in memory. See 'rawlog-edit --help'.\n\n"
    "- Edits in this viewer modify the rawlog in memory: use File > Save as... to keep them.";

constexpr const char* DEFAULT_RAWMAP_CONFIG =
    "// Maps to build: see mrpt::maps::TSetOfMetricMapInitializers\n"
    "[map]\n"
    "pointsMap_count=1\n"
    "\n"
    "[map_pointsMap_00_insertOpts]\n"
    "minDistBetweenLaserPoints=0.05\n"
    "also_interpolate=false\n";

constexpr const char* DEFAULT_BATCH_POSE_CONFIG =
    "// One section per sensor. Angles in degrees.\n"
    "// calib_params=[fx fy cx cy] and distort_params=[k1 k2 p1 p2] are optional,\n"
    "// for monocular cameras only.\n"
    "[OneCamera]\n"
    "sensorLabel = CAMERA1\n"
    "pose_x = 0\n"
    "pose_y = 0\n"
    "pose_z = 0.5\n"
    "pose_yaw = 0\n"
    "pose_pitch = 0\n"
    "pose_roll = 0\n"
    "\n"
    "[AnotherSensor]\n"
    "sensorLabel = LASER1\n"
    "pose_x = 0.2\n"
    "pose_y = 0\n"
    "pose_z = 0.3\n"
    "pose_yaw = 0\n"
    "pose_pitch = 0\n"
    "pose_roll = 0\n";

}  // namespace rlv
