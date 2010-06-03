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

#ifndef ARIA_H
#define ARIA_H

#include "ariaOSDef.h"
#include "ariaTypedefs.h"
#include "ArSerialConnection.h"
#include "ArTcpConnection.h"
#include "ArSimpleConnector.h"
#include "ArLogFileConnection.h"
#include "ArLog.h"
#include "ArRobotPacket.h"
#include "ArRobotPacketSender.h"
#include "ArRobotPacketReceiver.h"
#include "ArRobotConfigPacketReader.h"
#include "ArRobotTypes.h"
#include "ariaUtil.h"
#include "ArArgumentBuilder.h"
#include "ArArgumentParser.h"
#include "ArFileParser.h"
#include "ArConfig.h"
#include "ArConfigArg.h"
#include "ArConfigGroup.h"
#include "ArRobot.h"
#include "ArCommands.h"
#include "ArJoyHandler.h"
#include "ArSyncTask.h"
#include "ArTaskState.h"
#include "ariaInternal.h"
#include "ArSonarDevice.h"
#include "ArPriorityResolver.h"
#include "ArAction.h"
#include "ArActionGroup.h"
#include "ArActionGroups.h"
#include "ArActionDeceleratingLimiter.h"
#include "ArActionLimiterForwards.h"
#include "ArActionLimiterBackwards.h"
#include "ArActionLimiterTableSensor.h"
#include "ArActionBumpers.h"
#include "ArActionIRs.h"
#include "ArActionStallRecover.h"
#include "ArActionAvoidFront.h"
#include "ArActionAvoidSide.h"
#include "ArActionConstantVelocity.h"
#include "ArActionInput.h"
#include "ArActionRobotJoydrive.h"
#include "ArActionJoydrive.h"
#include "ArActionKeydrive.h"
#include "ArActionTriangleDriveTo.h"
#include "ArActionTurn.h"
#include "ArActionRatioInput.h"
#include "ArActionStop.h"
#include "ArActionGoto.h"
#include "ArModule.h"
#include "ArModuleLoader.h"
#include "ArRecurrentTask.h"
#include "ArInterpolation.h"
#include "ArGripper.h"
#include "ArSonyPTZ.h"
#include "ArAMPTU.h"
#include "ArP2Arm.h"
#include "ArACTS.h"
#include "ArSick.h"
#include "ArSickLogger.h"
#include "ArIrrfDevice.h"
#include "ArKeyHandler.h"
#include "ArDPPTU.h"
#include "ArVCC4.h"
#include "ArMode.h"
#include "ArModes.h"
#include "ArNetServer.h"
#include "ArSignalHandler.h"
#include "ArAnalogGyro.h"
#include "ArMap.h"
#include "ArLineFinder.h"
#include "ArBumpers.h"
#include "ArIRs.h"
#include "ArDrawingData.h"
#include "ArForbiddenRangeDevice.h"
#include "ArTCM2.h"
#if !defined(WIN32) && !defined(SWIG)
#include "ArVersalogicIO.h"
#endif
#include "ArActionGotoStraight.h"
#include "ArDataLogger.h"
#include "ArRobotJoyHandler.h"
#include "ArRatioInputKeydrive.h"
#include "ArRatioInputJoydrive.h"
#include "ArRatioInputRobotJoydrive.h"
#include "ArActionMovementParameters.h"
#include "ArSoundPlayer.h"
#include "ArSoundsQueue.h"
#include "ArCameraCollection.h"
#include "ArCameraCommands.h"
#include "ArStringInfoGroup.h"
#include "ArSonarAutoDisabler.h"
#include "ArActionDriveDistance.h"
#include "ArLaserReflectorDevice.h"
#include "ArRobotConfig.h"

#endif // ARIA_H
