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


#ifndef ARNOVATELGPS_H
#define ARNOVATELGPS_H


#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include "ariaUtil.h"
#include "ArGPS.h"

/** @brief GPS subclass to support the NovAtel ProPak G2 and other NovAtel
 * "OEM4" devices.
 *  
 *  This subclass overrides connect() to send initialization commands specific
 *  to NovAtel GPS devices.
 *  On connect, it initiazes SBAS (satellite-based augmentation) to
 *  "auto", which uses WAAS, EGNOS, or MSAS if available,
 *  and then requests NMEA messages to be interpreted by ArGPS.
 *
 *  It does not initialize corrections sent by your own DGPS fixed
 *  base station over radio, or the wide-area L-Band services such as OmniStar or 
 *  the Canada-wide DGPS (supported by the NovaTel ProPak LBplus). 
 *  To configure DGPS, you must connect to the GPS with a terminal
 *  program (e.g. "minicom" in Linux), configure the device for
 *  your specific service, region and radio settings, and save 
 *  with the "saveconfig" command. See the GPS setup notes 
 *  at http://robots.mobilerobots.com/tech_notes/GPS_Setup_Notes.txt
 *  and the NovAtel GPS Reference Manual Volume 1, Sections 4.3 and 4.5 for
 *  ("Transmitting and Receiving Corrections" and "Enabling L-Band Positioning")
 *  for more information on doing this.
 *
 */
class ArNovatelGPS : public virtual ArGPS {
public:
  AREXPORT virtual bool connect();
  AREXPORT virtual ~ArNovatelGPS();
};

#endif 


