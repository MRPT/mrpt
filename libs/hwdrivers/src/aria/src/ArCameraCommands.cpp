/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

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

