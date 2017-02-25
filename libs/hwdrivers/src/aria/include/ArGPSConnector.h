/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef ARGPSCONNECTOR_H
#define ARGPSCONNECTOR_H

#include <string>
#include <vector>

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"
#include "ArGPS.h"

class ArDeviceConnection;

/** 
 *  @brief Factory for creating ArGPS objects based on command-line
 *  parameters.
 *
 *  Use createGPS() to create a GPS object.
 *
 *  @note The device connection object created by 
 *  ArGPSConnector is destroyed  when ArGPSConnector is 
 *  destroyed. Therefore, you must not destroy an ArGPSConnector
 *  while its associated ArGPS is in use.
 *
 *  The following command line parameters are used:
 *  <dl>
 *    <dt>gpsPort</dt> 
 *      <dd>Create a serial device connection to use, using the given port. 
 *      Default is COM2 (/dev/ttyS1 on Linux)</dd>
 *    <dt>gpsBaud</dt> 
 *      <dd>Set the serial device connection to the given baud rate. 
 *      Default is 9600.</dd>
 *    <dt>gpsType</dt> 
 *      <dd>Set the GPS device type (determine initialization
 *      commands sent). Valid values are  "standard" or "novatelg2". 
 *
 *  </dl>
 *
 *  @todo Don't delete the device connection, let ArGPS do that? Set a flag to
 *  tell ArGPS to do it?
*/

class ArGPSConnector {
public:
  AREXPORT ArGPSConnector(ArArgumentParser* argParser);
  AREXPORT ~ArGPSConnector();

  /** Gets command line arguments */
  AREXPORT bool parseArgs();

  /** Log argument option information */
  AREXPORT void logArgs();

  /** Create a new GPS object (may be an ArGPS subclass based on device type)
   * and a device connection for that GPS.
   * See ArGPS for instructions on using it.
   *
   * @return NULL if there was an error creating a GPS object or an error
   * creating and opening its device connection. Otherwise, return the new GPS
   * object.  
   */
  AREXPORT ArGPS* createGPS();

  /** Keep trying to connect to the GPS, possibly adjusting connection
   * parameters.  */
  AREXPORT bool connectGPS(ArGPS *gps);

protected:

  /** @brief Device type identifiers */
  typedef enum {
      /// For a standard NMEA GPS device accessible using ArGPS 
      Standard, 
      /// For a Novatel device accessible using ArNovatelGPS 
      Novatel
  } GPSType;

  ArDeviceConnection *myDeviceCon;
  ArArgumentParser *myArgParser;
  ArRetFunctorC<bool, ArGPSConnector> myParseArgsCallback;
  ArFunctorC<ArGPSConnector> myLogArgsCallback;
  int myBaud;
  const char *myPort;
  const char *myTCPHost;
  int myTCPPort;
  GPSType myDeviceType;
};


#endif  // ifdef ARGPSCONNECTOR_H


