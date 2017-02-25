/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARCAMERACOLLECTION_H
#define ARCAMERACOLLECTION_H

#include <list>
#include <map>
#include <string>

#include "ArConfigArg.h"
#include "ArFunctor.h"
#include "ArMutex.h"

class ArCameraCollectionItem;
class ArCameraParameterSource;

/// Maintains information about all of the robot's cameras.
/**
 * ArCameraCollection is a repository for information about each of the 
 * cameras that are installed on the robot.  It enables clients to adjust
 * to robots with varying camera configurations.
 * <p>
 * Three main types of information are maintained for each camera:
 * <ul>
 * <li> Overview Information:  This includes the name and type of the camera,
 * along with text strings suitable for display.  Note that each camera
 * must be assigned a unique name.  Furthermore, the overview information
 * must be added to the collection before any of the other types of 
 * information are added.</li>
 *
 * <li> Command Information: Each camera may respond to any number
 * of generic commands (such as pan/tilt/zoom, or get camera picture).
 * The command information defines which commands are supported for 
 * each camera, and also defines a unique "camera command name" for 
 * each generic command. (The "camera command name", for example, 
 * may be used as a network packet name.  It must be unique across 
 * <em>all</em> of the robot's cameras.)  Commands which are commonly
 * supported are defined in ArCameraCommands, but additional ones may 
 * be added.</li>
 *
 * <li> Parameter Information: Parameters, or settings, may be defined
 * for each camera.  A framework has been built into the collection to
 * allow clients to view/edit the parameters.  Changes are relayed to
 * the originator (source) of the parameter, which is responsible for
 * actually adjusting the camera hardware. </li>
 * </ul>
 * <p>
 * Callbacks may be installed on the collection to be notified whenever
 * the collection is modified.  This notification only occurs when
 * cameras, commands, or parameters are added or removed. (The editing
 * of parameters is merely passed to the parameter source.)
 * <p>
 * ArCameraCollection is thread-safe.  
**/
class ArCameraCollection
{
public:
	
  /// Constructor
  AREXPORT ArCameraCollection();

  /// Destructor
	AREXPORT virtual ~ArCameraCollection();
	
  // ---------------------------------------------------------------------------

  /// Adds a new camera to the collection.
  /**
   * @param cameraName the char * name of the camera; must be unique 
   * and non-NULL
   * @param cameraType the char * type of the camera (e.g. VCC4 or Omni)
   * @param displayName the char * string to be displayed for the 
   * camera name; if NULL, then the cameraName is used instead
   * @param displayType the char * string to be displayed for the 
   * camera type; if NULL, then the cameraType is used instead
   * @return bool true if the camera was successfully added; false,
   * otherwise.
  **/
  AREXPORT virtual bool addCamera(const char *cameraName,
                                  const char *cameraType,
                                  const char *displayName,
                                  const char *displayType);

  /// Removes the specified camera from the collection.
  /**
   * @param cameraName the char * name of the camera to be removed
   * @return bool true if the camera was successfully removed; false, 
   * otherwise.
  **/
  AREXPORT virtual bool removeCamera(const char *cameraName);

  /// Adds the specified command to the camera.
  /**
   * @param cameraName the char * name of the camera to which the 
   * command is to be added.  The camera must have already been 
   * installed in the collection via the addCamera() method.
   * @param command the char * identifier of the command that is being 
   * added.  Common commands (currently recognized by MobileEyes)
   * are defined in ArCameraCommands.
   * @param cameraCommandName the corresponding char * command 
   * (i.e. network packet) name that is actually handled by the 
   * camera.  The cameraCommandName must be non-NULL and unique
   * across all robot cameras.
   * @param requestInterval the int number of milliseconds between
   * client requests for repeating commands; if -1, then the 
   * command is not repeating.
   * @return bool true if the command was successfully added;
   * false if an error occurred.
  **/
  AREXPORT virtual bool addCameraCommand(const char *cameraName,
                                         const char *command,
                                         const char *cameraCommandName,
                                         int requestInterval = -1);


  /// Removes the specified command from the camera.
  /**
   * @param cameraName the char * name of the camera from which the 
   * command is to be removed.  
   * @param command the char * identifier of the command that is being 
   * removed.  
   * @return bool true if the command was successfully removed;
   * false if an error occurred.
  **/
  AREXPORT virtual bool removeCameraCommand(const char *cameraName,
                                            const char *command);


  /// Adds the specified parameter to the camera.
  /**
   * @param cameraName the char * name of the camera to which the 
   * parameter is to be added.  The camera must have already been 
   * installed in the collection via the addCamera() method.
   * @param source the ArCameraParameterSource * that is to be notified
   * when the parameter value is changed; if NULL, then no notification
   * @param param the ArConfigArg parameter to be added; the parameter
   * name must be unique for this camera.  (Parameter names may be 
   * reused across different cameras though.)
   * @return bool true if the parameter was successfully added to the 
   * camera; false if an error occurred
  **/
  AREXPORT virtual bool addParameter(const char *cameraName,
                                     ArCameraParameterSource *source,
                                     const ArConfigArg &param);


  /// Removes the specified parameter from the camera.
  /**
   * @param cameraName the char * name of the camera from which the 
   * parameter is to be removed.  
   * @param paramName the char * name of the parameter to be removed
   * @return bool true if the parameter was successfully removed from the 
   * camera; false if an error occurred
  **/
  AREXPORT virtual bool removeParameter(const char *cameraName,
                                        const char *paramName);

  // ---------------------------------------------------------------------------

  /// Returns the names of the cameras that are in the collection.
  /**
   * @param outList the std::list<std::string> into which the names are output;
   * any previous contents of the outList are cleared.
  **/
  AREXPORT virtual void getCameraNames(std::list<std::string> &outList);


  /// Returns the type of the specified camera.
  /**
   * @param cameraName the unique char * name of the camera
   * @return char * the type of the specified camera; NULL if the camera was 
   * not found in the collection
  **/
  AREXPORT virtual const char *getCameraType(const char *cameraName);

  /// Returns the display name of the specified camera.
  /**
   * @param cameraName the unique char * name of the camera
   * @return char * the string to be displayed as the name of the specified camera; 
   * NULL if the camera was not found in the collection
  **/
  AREXPORT virtual const char *getDisplayName(const char *cameraName);

  /// Returns the display type of the specified camera.
  /**
   * @param cameraName the unique char * name of the camera
   * @return char * the string to be displayed as the type of the specified camera; 
   * NULL if the camera was not found in the collection
  **/
  AREXPORT virtual const char *getDisplayType(const char *cameraName);

 // ---------------------------------------------------------------------------

  /// Returns the generic commands that are supported by the specified camera.
  /**
   * @param cameraName the unique char * name of the camera
   * @param outList the std::list<std::string> into which the commands are output;
   * any previous contents of the outList are cleared.
  **/
  AREXPORT virtual void getCameraCommands(const char *cameraName,
                                          std::list<std::string> &outList);

  /// Returns the specific camera command (/ network packet) name for the generic command.
  /**
   * @param cameraName the unique char * name of the camera
   * @param command the char * name of the generic command to be retrieved
   * @return char * the unique command (or network packet) name for the generic
   * command on the specified camera; NULL, if the camera does not support the 
   * generic command
  **/
  AREXPORT virtual const char *getCommandName(const char *cameraName,
                                              const char *command);

  /// Returns the default request interval for the specified camera command.
  /**
   * @param cameraName the unique char * name of the camera
   * @param command the char * name of the generic command 
   * @return int the default number of milliseconds between command requests;
   * if -1, then the command is not for refreshing data
  **/
  AREXPORT virtual int getRequestInterval(const char *cameraName,
                                          const char *command);

  // ---------------------------------------------------------------------------

  /// Returns the names of the parameters for the specified camera.
  /**
   * @param cameraName the unique char * name of the camera
   * @param outList the std::list<std::string> into which the parameter names are 
   * output; any previous contents of the outList are cleared.
  **/
  AREXPORT virtual void getParameterNames(const char *cameraName,
                                          std::list<std::string> &outList);

  /// Returns the specified camera parameter.
  /**
   * @param cameraName the unique char * name of the camera
   * @param parameterName the unique char * name of the parameter to be retrieved
   * @param paramOut the ArConfigArg into which the parameter is copied
   * @return bool true if the parameter was successfully found; false, otherwise.
  **/
  AREXPORT virtual bool getParameter(const char *cameraName,
                                     const char *parameterName,
                                     ArConfigArg &paramOut);
  
  /// Updates the specified camera parameter.
  /**
   * @param cameraName the unique char * name of the camera
   * @param param the ArConfigArg to be set; the parameter must have been
   * previously added to the camera with the addParameter() method
   * @return bool true if the parameter was found and set; false, otherwise.
  **/
  AREXPORT virtual bool setParameter(const char *cameraName,
                                     const ArConfigArg &param);
 
  // ---------------------------------------------------------------------------

  /// Returns whether the specified camera is contained in the collection.
  AREXPORT virtual bool exists(const char *cameraName);

  /// Returns whether the specified command is defined for a particular camera.
  AREXPORT virtual bool exists(const char *cameraName,
                               const char *command);

  /// Returns whether the specified parameter has been defined for a particular camera.
  AREXPORT virtual bool parameterExists(const char *cameraName,
		  		        const char *paramName);

  // ---------------------------------------------------------------------------

  /// Adds a callback to be invoked when the camera collection has been modified.
  /**
   * @param functor the ArFunctor * to be invoked when the collection has been
   * modified; must be non-NULL
   * @param position the ArListPos::Pos at which to put the callback
   * (beginning or end)
   * @return bool true if the callback was successfully added; false, otherwise.
  **/
  AREXPORT virtual bool addModifiedCB(ArFunctor *functor,
                                      ArListPos::Pos position = ArListPos::LAST);

  /// Removes a callback from the modified notification list.
  /**
   * @param functor the ArFunctor * to be removed from the notification list
   * @return bool true if the callback was successfully removed; false, otherwise.
  **/
  AREXPORT virtual bool removeModifiedCB(ArFunctor *functor);


  /// Starts an update to the collection.
  /**
   * This method may be used when multiple changes are being made to the collection.
   * While an update is in progress, the modified callbacks will not be invoked.
   * The endUpdate() method should be called after all of the changes are complete
   * (and then the modified callbacks <em>will</em> be invoked).
  **/
  AREXPORT virtual void startUpdate();

  /// Ends an update to the collection.
  /**
   * A call to startUpdate() must eventually be followed by a call to endUpdate().
  **/
  AREXPORT virtual void endUpdate();


  // ---------------------------------------------------------------------------

  /// Lock the collection
  /*AREXPORT*/ int lock() {
    return (myMutex.lock()); 
  }

  /// Try to lock the collection without blocking
  /*AREXPORT*/ int tryLock() {
    return(myMutex.tryLock());
  }

  /// Unlock the collection
  /*AREXPORT*/ int unlock() {
    return(myMutex.unlock());
  }


// -----------------------------------------------------------------------------
protected:

  /// Information regarding a particular camera command.
  struct CommandInfo {

    /// Generic name of the command.
    std::string myCommand;
    /// Unique name of the corresponding command (or packet) for this camera.
    std::string myCameraCommandName;
    /// Default number of milliseconds between command requests.
    int myRequestInterval;

    /// Constructor
    CommandInfo();
    /// Destructor
    ~CommandInfo();

  }; // end struct CommandInfo


  /// Information regarding a particular camera parameter.
  struct ParamInfo {

    /// Source of the parameter (to be notified when the parameter changes)
    ArCameraParameterSource *mySource;
    /// The parameter
    ArConfigArg myParam;
  
    /// Constructor
    ParamInfo();
    /// Destructor
    ~ParamInfo();

  }; // end struct ParamInfo


  /// Information regarding a single camera.
  struct CameraInfo {

    /// Unique name of the camera
    std::string myCameraName;
    /// Type of the camera
    std::string myCameraType;
    /// String displayed for the name of the camera
    std::string myDisplayName;
    /// String displayed for the type of the camera
    std::string myDisplayType;
    /// Map of generic command names to the specific camera command info
    std::map<std::string, CommandInfo*> myCommandToInfoMap;
    /// Map of parameter names to the related info (including the actual parameter value)
    std::map<std::string, ParamInfo*> myParamToInfoMap;

    /// Constructor
    CameraInfo();
    /// Destructor
    ~CameraInfo();

  }; // end struct CameraInfo


  /// Returns a pointer to the CameraInfo for the specified camera.
  CameraInfo *findCameraInfo(const char *cameraName);

  /// Returns a pointer to the CommandInfo for the specified generic command.
  CommandInfo *findCommandInfo(const char *cameraName,
                               const char *commandName);

  /// Returns a pointer to the ParamInfo for the specified parameter.
  ParamInfo *findParamInfo(const char *cameraName,
                           const char *paramName);

  /// Sets an indication that the collection has been modified.
  void setModified();

  /// Invokes each of the callbacks when the collection has been modified.
  void notifyModifiedListeners();

private:
  /// Disabled copy ctor
  ArCameraCollection(const ArCameraCollection &);
  /// Disabled assignment operator
	ArCameraCollection &operator=(const ArCameraCollection &);

protected:

  /// Mutex for multi-threaded access
  ArMutex myMutex;
  /// Map of camera names to the associated camera information
  std::map<std::string, CameraInfo*> myCameraToInfoMap;
  /// Whether updates are currently enabled
  bool myIsUpdatesEnabled;
  /// Whether the collection has been modified (since the last notification)
  bool myIsModified;
  /// List of callbacks to be notified when the collection is modified
  std::list<ArFunctor *> myModifiedCBList;

}; // end class ArCameraCollection

// -----------------------------------------------------------------------------

/// Interface for items that add information to the camera collection.
/**
 * ArCameraCollectionItem is a simple interface whose primary purpose is to 
 * identify classes that support some aspect of a camera's functionality.
 * It defines two methods: one to identify the associated camera, and one
 * that adds the information about the supported functionality to the collection.
 * (Note that the addToCameraCollection() method is not automatically invoked.
 * Its only purpose in life is to suggest consistency between different items.)
**/
class ArCameraCollectionItem 
{
public:

  /// Constructor
  /*AREXPORT*/ ArCameraCollectionItem() {};
  /// Destructor
  /*AREXPORT*/ virtual ~ArCameraCollectionItem() {};

  /// Returns the name of the camera handled by this item.
  AREXPORT virtual const char *getCameraName() = 0;

  /// Adds this item to the given camera collection.
  AREXPORT virtual void addToCameraCollection(ArCameraCollection &collection) = 0;

}; // end class ArCameraCollectionItem

// -----------------------------------------------------------------------------

/// Interface for collection items that also access the camera's parameters.
/**
 * ArCameraParameterSource is a special collection item that provides the ability
 * to read and modify some of the camera's parameters.  In general, the 
 * addToCameraCollection() method should add the parameters to the collection.
 * The collection will then invoke the getParameter() and setParameter() methods as
 * callers make changes to the parameters; the ArCameraParameterSource is reponsible 
 * for propagating the changes to the camera hardware.
**/
class ArCameraParameterSource : public ArCameraCollectionItem
{
public:

  /// Constructor
  /*AREXPORT*/ ArCameraParameterSource() {};
  /// Destructor
  /*AREXPORT*/ ~ArCameraParameterSource() {};

  /// Gets the specified camera parameter.
  AREXPORT virtual bool getParameter(const char *paramName,
                                     ArConfigArg &paramOut) = 0;

  /// Sets the given camera parameter.
  AREXPORT virtual bool setParameter(const ArConfigArg &param) = 0;

}; // end class ArCameraParameterSource


#endif // ARCAMERACOLLECTION_H
