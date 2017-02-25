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

#include "ArCameraCollection.h"

AREXPORT ArCameraCollection::ArCameraCollection() :
  myMutex(),
	myCameraToInfoMap()
{
} // end ctor

AREXPORT ArCameraCollection::~ArCameraCollection()
{
  ArUtil::deleteSetPairs(myCameraToInfoMap.begin(),
                         myCameraToInfoMap.end());
  myCameraToInfoMap.clear();

} // end dtor

AREXPORT bool ArCameraCollection::addCamera(const char *cameraName,
											                      const char *cameraType,
											                      const char *displayName,
											                      const char *displayType)
{
  if (cameraName == NULL) {
    return false;
  }

  lock();
  CameraInfo *info = findCameraInfo(cameraName);
  if (info != NULL) {
    unlock();
    return false;
  }

  info = new CameraInfo();

  info->myCameraName = cameraName;
  info->myCameraType = cameraType;
  info->myDisplayName = ((displayName != NULL) ? displayName : cameraName);
  info->myDisplayType = ((displayType != NULL) ? displayType : cameraType);

  myCameraToInfoMap[cameraName] = info;

  unlock();
  return true;

} // end method addCamera


AREXPORT bool ArCameraCollection::removeCamera(const char *cameraName)
{
  if (cameraName == NULL) {
    return false;
  }
  lock();

  std::map<std::string, CameraInfo*>::iterator iter =
                                            myCameraToInfoMap.find(cameraName);
  if (iter == myCameraToInfoMap.end()) {
    unlock();
    return false;
  }

  delete iter->second;
  iter->second = NULL;

  myCameraToInfoMap.erase(iter);

  setModified();

  unlock();

  return true;

} // end method removeCamera


AREXPORT bool ArCameraCollection::addCameraCommand(const char *cameraName,
												                           const char *command,
												                           const char *cameraCommandName,
                                                   int requestInterval)
{
  if (command == NULL) {
    return false;
  }
  lock();

  CameraInfo *cameraInfo = findCameraInfo(cameraName);
  if (cameraInfo == NULL) {
    unlock();
    return false;
  }

  CommandInfo *info = findCommandInfo(cameraName, command);
  if (info != NULL) {
    unlock();
    return false;
  }

  info = new CommandInfo();
  info->myCommand = command;
  info->myCameraCommandName = cameraCommandName;
  info->myRequestInterval = requestInterval;


  cameraInfo->myCommandToInfoMap[command] = info;

  setModified();
  unlock();

  return true;

} // end method addCameraCommand


AREXPORT bool ArCameraCollection::removeCameraCommand(const char *cameraName,
													                            const char *command)
{
  lock();

  CameraInfo *info = findCameraInfo(cameraName);
  if (info == NULL) {
    unlock();
    return false;
  }

  std::map<std::string, CommandInfo*>::iterator iter =
                                        info->myCommandToInfoMap.find(command);

  if (iter == info->myCommandToInfoMap.end()) {
    unlock();
    return false;
  }

  delete iter->second;
  iter->second = NULL;

  info->myCommandToInfoMap.erase(iter);

  setModified();
  unlock();

  return true;

} // end method removeCameraCommand


AREXPORT bool ArCameraCollection::addParameter(const char *cameraName,
                                              ArCameraParameterSource *source,
                                              const ArConfigArg &param)
{
  lock();
  CameraInfo *camInfo = findCameraInfo(cameraName);
  if ((camInfo == NULL) || (param.getName() == NULL)) {
    unlock();
    return false;
  }

  ParamInfo *info = findParamInfo(cameraName, param.getName());
  if (info != NULL) {
    unlock();
    return false;
  }

  info = new ParamInfo();
  info->mySource = source;
  info->myParam = param;

  camInfo->myParamToInfoMap[param.getName()] = info;

  unlock();
  return true;

} // end method addParameter


AREXPORT bool ArCameraCollection::removeParameter(const char *cameraName,
                                                  const char *paramName)
{
  lock();
  CameraInfo *camInfo = findCameraInfo(cameraName);
  if ((camInfo == NULL) || (paramName == NULL)) {
    unlock();
    return false;
  }

  std::map<std::string, ParamInfo*>::iterator iter =
        camInfo->myParamToInfoMap.find(paramName);

  if (iter == camInfo->myParamToInfoMap.end()) {
    unlock();
    return false;
  }

  camInfo->myParamToInfoMap.erase(iter);

  unlock();
  return true;

} // end method removeParameter


AREXPORT void ArCameraCollection::getCameraNames(std::list<std::string> &outList)
{
  lock();
  outList.clear();

  for (std::map<std::string, CameraInfo*>::iterator iter = myCameraToInfoMap.begin();
       iter != myCameraToInfoMap.end();
       iter++) {
    outList.push_back(iter->first);

  } // end for each map entry

  unlock();

} // end method getCameraNames


AREXPORT const char *ArCameraCollection::getCameraType(const char *cameraName)
{
  const char *type = NULL;

  lock();

  CameraInfo *info = findCameraInfo(cameraName);
  if (info != NULL) {
    type = info->myCameraType.c_str();
  }
  unlock();

  return type;

} // end method getCameraType


AREXPORT const char *ArCameraCollection::getDisplayName(const char *cameraName)
{
  const char *displayName = NULL;

  lock();

  CameraInfo *info = findCameraInfo(cameraName);
  if (info != NULL) {
    displayName = info->myDisplayName.c_str();
  }
  unlock();

  return displayName;

} // end method getDisplayName


AREXPORT const char *ArCameraCollection::getDisplayType(const char *cameraName)
{
  const char *displayType = NULL;

  lock();

  CameraInfo *info = findCameraInfo(cameraName);
  if (info != NULL) {
    displayType = info->myDisplayType.c_str();
  }
  unlock();

  return displayType;

} // end method getDisplayType


AREXPORT void ArCameraCollection::getCameraCommands(const char *cameraName,
                                                    std::list<std::string> &outList)
{
  lock();
  outList.clear();

  CameraInfo *info = findCameraInfo(cameraName);
  if (info != NULL) {
    for (std::map<std::string, CommandInfo*>::iterator iter = info->myCommandToInfoMap.begin();
        iter != info->myCommandToInfoMap.end();
        iter++) {
      outList.push_back(iter->first);

    } // end for each map entry
  } // end if info found

  unlock();

} // end method getCameraCommands


AREXPORT const char *ArCameraCollection::getCommandName(const char *cameraName,
														                            const char *command)
{
  const char *cameraCommandName = NULL;

  lock();

  CommandInfo *info = findCommandInfo(cameraName, command);
  if (info != NULL) {
    cameraCommandName = info->myCameraCommandName.c_str();
  } // end if info found

  unlock();

  return cameraCommandName;

} // end method getCommandName


AREXPORT int ArCameraCollection::getRequestInterval(const char *cameraName,
														                        const char *command)
{
  int interval = -1;

  lock();

  CommandInfo *info = findCommandInfo(cameraName, command);
  if (info != NULL) {
    interval = info->myRequestInterval;
  } // end if info found

  unlock();

  return interval;

} // end method getRequestInterval


AREXPORT void ArCameraCollection::getParameterNames
                                      (const char *cameraName,
                                       std::list<std::string> &outList)
{
  lock();
  outList.clear();

  CameraInfo *info = findCameraInfo(cameraName);
  if (info != NULL) {
    for (std::map<std::string, ParamInfo*>::iterator iter = info->myParamToInfoMap.begin();
         iter != info->myParamToInfoMap.end();
         iter++) {
      outList.push_back(iter->first);

    } // end for each map entry
  } // end if info found

  unlock();

} // end method getParameterNames

AREXPORT bool ArCameraCollection::getParameter(const char *cameraName,
                                               const char *parameterName,
                                               ArConfigArg &paramOut)
{
  lock();

  ParamInfo *info = findParamInfo(cameraName, parameterName);
  if (info == NULL) {
    unlock();
    return false;
  }
  paramOut = info->myParam;
  unlock();

  return true;

} // end method getParameter



AREXPORT bool ArCameraCollection::setParameter(const char *cameraName,
                                               const ArConfigArg &param)
{
  lock();

  ParamInfo *info = findParamInfo(cameraName, param.getName());
  if (info == NULL) {
    unlock();
    return false;
  }
  info->myParam = param;

  if (info->mySource != NULL) {
    info->mySource->setParameter(param);
  }

  unlock();
  return true;

} // end method changeParameter


AREXPORT bool ArCameraCollection::exists(const char *cameraName)
{
  lock();

  CameraInfo *info = findCameraInfo(cameraName);

  unlock();

  return (info != NULL);

} // end method exists


AREXPORT bool ArCameraCollection::exists(const char *cameraName,
				         const char *command)
{
  if ((cameraName == NULL) || (command == NULL)) {
    return false;
  }

  lock();

  CommandInfo *info = findCommandInfo(cameraName, command);

  unlock();

  return (info != NULL);

} // end method exists


AREXPORT bool ArCameraCollection::parameterExists(const char *cameraName,
					          const char *paramName)
{
  if ((cameraName == NULL) || (paramName == NULL)) {
    return false;
  }
  lock();

  ParamInfo  *info = findParamInfo(cameraName, paramName);

  unlock();

  return (info != NULL);

}


AREXPORT void ArCameraCollection::startUpdate()
{
  lock();
  myIsUpdatesEnabled = false;
  unlock();

} // end method startUpdate


AREXPORT void ArCameraCollection::endUpdate()
{
  lock();
  myIsUpdatesEnabled = true;
  // Now that updates have been re-enabled, check whether the collection was
  // modified and notify any listeners.
  notifyModifiedListeners();
  unlock();

} // end method endUpdate


AREXPORT bool ArCameraCollection::addModifiedCB(ArFunctor *functor,
                                                ArListPos::Pos position)
{
  if (functor == NULL) {
    return false;
  }

  bool isSuccess = true;
  lock();

  switch (position) {
  case ArListPos::FIRST:
    myModifiedCBList.push_front(functor);
    break;

  case ArListPos::LAST:
    myModifiedCBList.push_back(functor);
    break;

  default:
    isSuccess = false;
    ArLog::log(ArLog::Terse,
               "ArCameraCollection::addModifiedCB: Unrecognized position = %i.",
               position);
    break;
  }

  unlock();

  return isSuccess;

} // end method addModifiedCB


AREXPORT bool ArCameraCollection::removeModifiedCB(ArFunctor *functor)
{
  if (functor == NULL) {
    return false;
  }

  lock();
  // Could/should add a check to make sure functor is really there in order
  // to return a more accurate status... (But this probably isn't really
  // an issue since functors aren't removed all that often.)
  myModifiedCBList.remove(functor);
  unlock();

  return true;

} // end method removeModifiedCB


ArCameraCollection::CameraInfo *ArCameraCollection::findCameraInfo(const char *cameraName)
{
  if (cameraName == NULL) {
    return NULL;
  }

  CameraInfo *info = NULL;

  std::map<std::string, CameraInfo*>::iterator iter =
                                            myCameraToInfoMap.find(cameraName);

  if (iter != myCameraToInfoMap.end()) {
    info = iter->second;
  }

  return info;

} // end method findCameraInfo


ArCameraCollection::CommandInfo *ArCameraCollection::findCommandInfo
                                                        (const char *cameraName,
                                                         const char *commandName)
{
  CameraInfo *cameraInfo = findCameraInfo(cameraName);
  if (cameraInfo == NULL) {
    return NULL;
  }

  CommandInfo *info = NULL;

  std::map<std::string, CommandInfo*>::iterator iter =
                                     cameraInfo->myCommandToInfoMap.find(commandName);

  if (iter != cameraInfo->myCommandToInfoMap.end()) {
    info = iter->second;
  }

  return info;

} // end method findCommandInfo

ArCameraCollection::ParamInfo *ArCameraCollection::findParamInfo
                                                      (const char *cameraName,
                                                       const char *paramName)
{
  CameraInfo *cameraInfo = findCameraInfo(cameraName);
  if (cameraInfo == NULL) {
    return NULL;
  }

  ParamInfo *info = NULL;

  std::map<std::string, ParamInfo*>::iterator iter =
                                     cameraInfo->myParamToInfoMap.find(paramName);

  if (iter != cameraInfo->myParamToInfoMap.end()) {
    info = iter->second;
  }

  return info;

} // end method findParamInfo



void ArCameraCollection::setModified()
{
  myIsModified = true;
  notifyModifiedListeners();
}


void ArCameraCollection::notifyModifiedListeners()
{
  if (!myIsUpdatesEnabled || !myIsModified) {
    return;
  }


  myIsModified = false;

} // end method notifyModifiedListeners


ArCameraCollection::CameraInfo::CameraInfo() :
    myCameraName(),
    myCameraType(),
    myDisplayName(),
    myDisplayType(),
    myCommandToInfoMap(),
    myParamToInfoMap()
{
}

ArCameraCollection::CameraInfo::~CameraInfo()
{
  ArUtil::deleteSetPairs(myCommandToInfoMap.begin(),
                         myCommandToInfoMap.end());
  myCommandToInfoMap.clear();
}


ArCameraCollection::CommandInfo::CommandInfo() :
  myCommand(),
  myCameraCommandName(),
  myRequestInterval(-1)
{
}

ArCameraCollection::CommandInfo::~CommandInfo()
{
}


ArCameraCollection::ParamInfo::ParamInfo() :
  mySource(NULL),
  myParam()
{
}

ArCameraCollection::ParamInfo::~ParamInfo()
{
}


