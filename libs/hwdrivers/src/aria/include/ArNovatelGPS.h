/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


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


