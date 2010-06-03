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


        /* SWIG 1.3 Wrapper Interface Definition for Aria */

#ifdef SWIGPYTHON
#warning Defining ARIA wrapper interface for Python

/* We need the module declared as "AriaPy" here so that other modules
   (like ArNetworking or ARNL) that use this wrapper will know what
   the resulting Python module and wrapper library are called. 
*/
%module(directors="1", docstring="Python wrapper library for Aria") AriaPy

#else
# ifdef SWIGJAVA
# warning Defining ARIA wrapper interface for Java

%module(directors="1", docstring="Java wrapper library for Aria") AriaJava

# else
# warning Defining ARIA wrapper interface for something other that Java or Python.

%module(directors="1", docstring="Wrapper library for Aria") Aria

# endif
#endif

%feature("autodoc", "1");


%header %{
 /* ******** %header ********** */
%}
%wrapper %{
 /* ******** %wrapper ********** */
%}


%{
#include "Aria.h"
#include "ArSoundPlayer.h"
#include "ArSpeech.h"
#include "ArSoundsQueue.h"
#include "wrapper_ExtraClasses.h"
%}
%warnfilter(451) ArUtil;


/* Give names to some vector template types: */
%include "std_vector.i"
%template(ArPoseWithTimeVector) std::vector<ArPoseWithTime>;
%template(ArSensorReadingVector) std::vector<ArSensorReading>;



/* Enable director classes (subclasses) for ArAction.
   Other classes in Aria that can be subclasses, but very rarely are,
   include ArResolver, ArBasePacket, ArDeviceConnection, ArRangeDevice,
   ArActionGroup, ArMode; maybe others. 
   You could add %feature("director") directives for those classes
   and regenerate the wrapper libraries if you want to use them.
   (They are omitted since making a class a director adds lots
   of code to the wrapper library.)
   ArASyncTask is also often subclassed, but threading in Python is
   kind of hard to get working right, at least as of Python 2.3, 
   especially going through Swig to do it, so providing ArASyncTask is 
   postponed.
*/
%feature("director") ArAction;

/* Supply an alternate setRobot() to avoid infinite recursion
   between the Python/Java subclass and Swig's director method.
*/
%extend ArAction {
  void setActionRobot(ArRobot* robot)
  {
    self->ArAction::setRobot(robot);
  }
}



/** Functors: **/

%{
#include "wrapper_Functors.h"
%}

/* In python, use typemaps to convert function objects to ArFunctor subclasses */

#ifdef SWIGPYTHON

%typemap(in) ArFunctor* {
  $1 = new ArPyFunctor($input); // XXX Memory leak. How to free?
}

%typecheck(SWIG_TYPECHECK_POINTER) ArFunctor* {
  $1 = PyCallable_Check($input);
}


%typemap(in) ArRetFunctor<bool>* {
  $1 = new ArPyRetFunctor_Bool($input); // XXX Memory leak. How to free it?
}


%typecheck(SWIG_TYPECHECK_POINTER) ArRetFunctor<bool>* {
  $1 = PyCallable_Check($input);
}

#endif // ifdef SWIGPYTHON 




/* In Java, enable directors so you can subclass ArFunctors */
#ifdef SWIGJAVA
%feature("director") ArFunctor;
//%feature("director") ArRetFunctor<bool>;
//%template(ArRetFunctor_Bool) ArRetFunctor<bool>;
#endif



/* Rename or ignore things that can cause problems for SWIG: */

%rename (removePendingItemsWithType) ArSoundsQueue::removePendingItems(const
%char*, ItemType);
%rename (removePendingItemsByPriority) ArSoundsQueue::removePendingItems(int);
%rename (removePendingItemsByPriorityWithType) ArSoundsQueue::removePendingItems(int, ItemType);
%rename (removePendingItemsByType) ArSoundsQueue::removePendingItems(ItemType);
%rename (nextItemByType) ArSoundsQueue::nextItem(ItemType);
%rename (nextItemByPriority) ArSoundsQueue::nextItem(int);
%rename (nextItemByTypeAndPriority) ArSoundsQueue::nextItem(ItemType, int);
%ignore ArActionTriangleDriveTo::getData;

// arconfig cannot target pointers in the wrapped languages:
%ignore ArConfigArg::ArConfigArg(const char*, int*, const char*, int, int);
%ignore ArConfigArg::ArConfigArg(const char*, short*, const char*, int, int);
%ignore ArConfigArg::ArConfigArg(const char*, unsigned short*, const char*, int, int);
%ignore ArConfigArg::ArConfigArg(const char*, unsigned char*, const char*, int, int);
%ignore ArConfigArg::ArConfigArg(const char*, double*, const char*, double, double);
%ignore ArConfigArg::ArConfigArg(const char*, bool*, const char*);
 

// Rename reserved words in Python:
#ifdef SWIGPYTHON
%rename(NoLog) ArLog::None;
%rename(printQueue) ArRingQueue::print;
#endif

// In Java and Python, you can easily concatenate strings and primitive types,
// so we can just refer to logPlain() as log(), and ignore the varargs log().
%ignore ArLog::log;
%rename (log) ArLog::logPlain;


// java.lang.Object has a final method called wait().
#ifdef SWIGJAVA
%rename(waitFor) ArCondition::wait;
#endif


/* Typemaps to make ARIA classes more accessible and work better in various
 * target languages:
 */

/* In python, use a standard list of strings for argc and argv: */

#ifdef SWIGPYTHON

// TODO: ArArgumentParser can modify the argv list, these changes
// need to be reflected in the Python or Java list after the C++ function
// returns (if possible). Not sure how to do that yet.

%typemap(in) (int *argc, char **argv) {
    int i;
    if (!PyList_Check($input)) {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }
    int tmpArgc = PyList_Size($input);
    tmpArgc = PyList_Size($input);
    $2 = (char **) malloc((tmpArgc+1)*sizeof(char *));
    for (i = 0; i < tmpArgc; i++) {
        PyObject *s = PyList_GetItem($input,i);
        if (!PyString_Check(s)) {
            free($2);
            PyErr_SetString(PyExc_ValueError, "Arguments must be strings");
            return NULL;
        }
        $2[i] = PyString_AsString(s);
    }
    $2[i] = 0;
    // Allocate a new int to hold the size, since some classes retain the pointer
    // and try to use it after this wrapped function returns
    int *newArgc = (int*) malloc(sizeof(int));
    *newArgc = tmpArgc;
    $1 = newArgc;
}

%typecheck(SWIG_TYPECHECK_POINTER) (int *argc, char **argv) {
  $1 = PyList_Check($input);
}

#endif //SIWGPYTHON

/* In Java, use a String[] for argc and argv (e.g. the argv[] parameter in main())  */

#ifdef SWIGJAVA

%typemap(in) (int *argc, char **argv) (jint size){
  size = jenv->GetArrayLength((jarray)$input);
  int tmpArgc = size;
  int i;
  $2 = (char**)malloc( (size+1) * sizeof(char*) );
  for(i = 0; i < size; i++) {
    jstring js = (jstring) jenv->GetObjectArrayElement((jobjectArray)$input, i);
    const char *cs = jenv->GetStringUTFChars(js, 0);
    $2[i] = (char*)malloc(strlen((cs)+1) * sizeof(const char));
    strcpy($2[i], cs);
    jenv->ReleaseStringUTFChars(js, cs);
    jenv->DeleteLocalRef(js);
  }
  $2[i] = 0;
  int *newArgc = (int*) malloc(sizeof(int));
  *newArgc = tmpArgc;
  $1 = newArgc;
}


/* XXX TODO? or will the jni and jtype typemaps below deal with it? :
%typecheck(java,SWIG_TYPECHECK_POINTER) (int *, char **) {
  // TODO 
}
*/


%typemap(jni) (int *argc, char **argv) "jobjectArray"
%typemap(jtype) (int *argc, char **argv) "java.lang.String[]"
%typemap(jstype) (int *argc, char **argv) "java.lang.String[]"
%typemap(javain) (int *argc, char **argv) "$javainput"
%typemap(javaout) (int *argc, char **argv) { return $jnicall; }



#endif // SWIGJAVA




/* Include files with class declarations to wrap: */

// dont include ArVersalogicIO.h or ArRobotTypes.h
%include "ArBasePacket.h"
%include "ArPTZ.h"
%include "ArThread.h"
%include "ArASyncTask.h"
%include "ArRangeDevice.h"
%include "ArRangeDeviceThreaded.h"
%include "ArResolver.h"
%include "ArAction.h"
%include "ArActionAvoidFront.h"
%include "ArActionAvoidSide.h"
%include "ArActionBumpers.h"
%include "ArActionColorFollow.h"
%include "ArActionConstantVelocity.h"
%include "ArActionDeceleratingLimiter.h"
%include "ArActionDesired.h"
%include "ArActionGoto.h"
%include "ArActionGotoStraight.h"
%include "ArActionGroup.h"
%include "ArActionGroups.h"
%include "ArActionInput.h"
%include "ArActionIRs.h"
%include "ArActionJoydrive.h"
%include "ArActionKeydrive.h"
%include "ArActionLimiterBackwards.h"
%include "ArActionLimiterForwards.h"
%include "ArActionLimiterTableSensor.h"
%include "ArActionMovementParameters.h"
%include "ArActionRatioInput.h"
%include "ArActionRobotJoydrive.h"
%include "ArActionStallRecover.h"
%include "ArActionStop.h"
%include "ArActionTriangleDriveTo.h"
%include "ArActionTurn.h"
%include "ArACTS.h"
%include "ArAMPTU.h"
%include "ArAnalogGyro.h"
%include "ArArg.h"
%include "ArArgumentBuilder.h"
%include "ArArgumentParser.h"
%include "ArBumpers.h"
%include "ArCommands.h"
%include "ArCondition.h"
%include "ArConfigArg.h"
%include "ArConfigGroup.h"
%include "ArConfig.h"
%include "ArDataLogger.h"
%include "ArDeviceConnection.h"
%include "ArDPPTU.h"
%include "ArDrawingData.h"
%include "ArExport.h"
%include "ArFileParser.h"
%include "ArForbiddenRangeDevice.h"
%include "ArFunctorASyncTask.h"
%include "ArFunctor.h"
%include "ArGripper.h"
%include "ariaInternal.h"
%include "ariaOSDef.h"
%include "ariaTypedefs.h"
%include "ariaUtil.h"
%include "ArInterpolation.h"
%include "ArIrrfDevice.h"
%include "ArIRs.h"
%include "ArJoyHandler.h"
%include "ArKeyHandler.h"
%include "ArLineFinder.h"
%include "ArLogFileConnection.h"
%include "ArLog.h"
%include "ArMap.h"
%include "ArMode.h"
%include "ArModes.h"
%include "ArModule.h"
%include "ArModuleLoader.h"
%include "ArMutex.h"
%include "ArNetServer.h"
%include "ArP2Arm.h"
%include "ArPriorityResolver.h"
%include "ArRangeBuffer.h"
%include "ArRatioInputJoydrive.h"
%include "ArRatioInputKeydrive.h"
%include "ArRatioInputRobotJoydrive.h"
%include "ArRecurrentTask.h"
%include "ArRingQueue.h"
%include "ArRobotConfigPacketReader.h"
%include "ArRobot.h"
%include "ArRobotJoyHandler.h"
%include "ArRobotPacket.h"
%include "ArRobotPacketReceiver.h"
%include "ArRobotPacketSender.h"
%include "ArRobotParams.h"
%include "ArSensorReading.h"
%include "ArSerialConnection.h"
%include "ArSick.h"
%include "ArSickLogger.h"
%include "ArSickPacket.h"
%include "ArSickPacketReceiver.h"
%include "ArSignalHandler.h"
%include "ArSimpleConnector.h"
%include "ArSocket.h"
%include "ArSonarDevice.h"
%include "ArSonyPTZ.h"
%include "ArSoundPlayer.h"
%include "ArSoundsQueue.h"
%include "ArSpeech.h"
%include "ArSyncLoop.h"
%include "ArSyncTask.h"
%include "ArTaskState.h"
%include "ArTCM2.h"
%include "ArTcpConnection.h"
%include "ArTransform.h"
%include "ArVCC4.h"

%include "wrapper_ExtraClasses.h"





/* Extensions to make ARIA classes more convenient in various target languages: */

// Extension to print an ArPose nicely in Python:
#ifdef SWIGPYTHON
%extend ArPose {
   char* __str__() {
      static char tmp[256];
      snprintf(tmp, 256, "(X:%.4f, Y:%.4f, T:%.4f)", self->getX(), self->getY(), self->getTh());
      return &tmp[0];
   }
}
#endif

// Extension that allows you to access pose components as member attributes
// rather than using accessor methods.  We do this by providing "dummy" member
// variables, then overloading Swigs internal accessors:
#ifdef SWIGPYTHON
%extend ArPose {
  double x, y, th;
}
%{
  const double ArPose_x_get(ArPose* p) {
    return (const double) p->getX();
  }
  void ArPose_x_set(ArPose* p, double x) {
    p->setX(x);
  }
  const double ArPose_y_get(ArPose* p) {
    return (const double) p->getY();
  }
  void ArPose_y_set(ArPose* p, double y) {
    p->setY(y);
  }
  const double ArPose_th_get(ArPose* p) {
    return (const double) p->getTh();
  }
  void ArPose_th_set(ArPose* p, double th) {
    p->setTh(th);
  }
%}
#endif



/* TODO:
    * Make ArConfig and ArConfigSection have dictionary access operators in
    * Python
*/




/* TODO: typemap to check if a pointer returned by a method is NULL, and
   throw an exception. (Many methods in ARIA return NULL if an object
   is not found.)  Maybe only do this for Java since Python has the "None"
   object value?
*/

