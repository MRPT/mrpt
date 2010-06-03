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
