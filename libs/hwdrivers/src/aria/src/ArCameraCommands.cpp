/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaUtil.h"
#include "ArCameraCommands.h"


AREXPORT const char *ArCameraCommands::GET_CAMERA_DATA = "getCameraData";
AREXPORT const char *ArCameraCommands::GET_CAMERA_INFO = "getCameraInfo";

AREXPORT const char *ArCameraCommands::GET_PICTURE  = "getPicture";
AREXPORT const char *ArCameraCommands::GET_SNAPSHOT = "getSnapshot";

AREXPORT const char *ArCameraCommands::GET_DISPLAY = "getDisplay";


AREXPORT const char *ArCameraCommands::SET_CAMERA_ABS = "setCameraAbs";
AREXPORT const char *ArCameraCommands::SET_CAMERA_REL = "setCameraRel";
AREXPORT const char *ArCameraCommands::SET_CAMERA_PCT = "setCameraPct";

AREXPORT const char *ArCameraCommands::GET_CAMERA_MODE_LIST = "getCameraModeList";
AREXPORT const char *ArCameraCommands::CAMERA_MODE_UPDATED = "cameraModeUpdated";
AREXPORT const char *ArCameraCommands::SET_CAMERA_MODE = "setCameraMode";

AREXPORT const char *ArCameraCommands::RESET_CAMERA = "resetCamera";
// TODO Make these the original packet names?  Move to ArNetworking?

AREXPORT const char *ArCameraCommands::GET_CAMERA_DATA_INT = "getCameraDataInt";
AREXPORT const char *ArCameraCommands::GET_CAMERA_INFO_INT = "getCameraInfoInt";
AREXPORT const char *ArCameraCommands::SET_CAMERA_ABS_INT = "setCameraAbsInt";
AREXPORT const char *ArCameraCommands::SET_CAMERA_REL_INT = "setCameraRelInt";
AREXPORT const char *ArCameraCommands::SET_CAMERA_PCT_INT = "setCameraPctInt";
AREXPORT const char *ArCameraCommands::GET_VIDEO = "getVideo";

AREXPORT const char *ArCameraCommands::GET_SNAPSHOT_PLAIN = "getSnapshotPlain";

