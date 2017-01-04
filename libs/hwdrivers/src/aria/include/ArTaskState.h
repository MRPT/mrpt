/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARTASKSTATE_H
#define ARTASKSTATE_H

/// Class with the different states a task can be in
/** 
    These are the defined states, if the state is anything other than is 
    defined here that is annotated (not running) the process will be run.
    No one should have any of their own states less than the USER_START 
    state.  People's own states should start at USER_START or at
    USER_START plus a constant (so they can have different sets of states).
*/
class ArTaskState
{
public:
  enum State 
  {
    INIT = 0,  ///< Initialized (running)
    RESUME,    ///< Resumed after being suspended (running)
    ACTIVE,    ///< Active (running)
    SUSPEND,   ///< Suspended (not running) 
    SUCCESS,   ///< Succeeded and done (not running)
    FAILURE,    ///< Failed and done (not running)
    USER_START = 20 ///< This is where the user states should start (they will all be run)
  };

};


#endif
