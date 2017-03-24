/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef ARGPS_H
#define ARGPS_H

#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include "ariaUtil.h"
#include <string>
#include <vector>

class ArDeviceConnection; // for pointer in ArGPS

/** @brief GPS Device Interface 
 *
 *  Connects to GPS device over a serial port or other device connection and reads data.
 *  Supports GPS devices sending standard NMEA format data 
 *  (specifically the GPRMC, GPGGA, GPGSA, GPGRME, and optionally PGRMZ, PGRME, 
 *  and HCHDG messages). 
 *  If your GPS device supports several data formats or modes, select
 *  NMEA output in its configuration.
 *
 *  The preferred method of creating and setting up a new ArGPS object is to use
 *  ArGPSConnector, which creates an instance of ArGPS or a subclass, and
 *  creates and opens its device connection, based on command-line parameters.
 *  (To manually create an ArGPS object, create an ArDeviceConnection instance
 *  and call setDeviceConnection(), then open that device connection and call
 *  connect() or blockingConnect().)
 *
 *  For either method, to get new data from the GPS, must call read() periodically, 
 *  ideally at a rate equal to or faster than your GPS sends data (usually one second). 
 *  read() returns flags indicating whether it received partial data from the
 *  GPS, a complete "message" of data that it used to update stored data, or an
 *  error.
 *  You can use an ArRobot synchronized task callback functor to call read(), or you can run a 
 *  loop in a new thread.  If you are calling read() from a loop in a new thread, 
 *  the loop ought to include a call to ArUtil::sleep() for at least several hundred 
 *  miliseconds to avoid starving other threads, since read() will return
 *  immediately if there is no data to read rather than blocking.
 *
 *  @sa @ref gpsExample.cpp
 *
 *  This class is not thread safe. Stored data is updated by read(), so you can
 *  use an ArMutex to lock around data accessors (the getXXX methods) and also
 *  around calls to read().
 *
 *  @note ArGPS only provides access to the data reported by a GPS. The position
 *  reported by a GPS is in degrees on the surface of the earth, not in the
 *  cartesian coordinate system used by the robot odometry or ArMap, and Aria does
 *  not yet provide mechanisms for registering and projecting maps geographically or
 *  transforming positions between the two coordinate systems.
 *
 */
class ArGPS {

public:
    AREXPORT ArGPS();
    AREXPORT virtual ~ArGPS();

    /** @brief Set device connection to use */
    /*AREXPORT*/ void setDeviceConnection(ArDeviceConnection* deviceConn) { myDevice = deviceConn; }

    /** @brief Return device connection in use (or NULL if none) */
    /*AREXPORT*/ ArDeviceConnection* getDeviceConnection(ArDeviceConnection* deviceConn) const { return myDevice; }


    /** @brief Check that the device connection is open, and get ready to read data from the GPS. 
     *  If a non-standard device type was set, then device-specific
     *  initialization commands may be sent.
     *
     *  @return false if there is no device connection or the device connection
     *  is not open, or if there is an error sending device initialization
     *  commands. Otherwise, return true.
     *
     */
    AREXPORT virtual bool connect();

    /** @brief Call connect(), then keep attempting to read data. 
     * If no valid data is
     * received and parsed (resulting in a stored data update) within the timeout period, return false. 
     * @param connectTimeout If no data is received from the GPS within this
     * number of miliseconds, return false.
     * @return see connect(). In addition, return false if no data is received
     * within the timeout period.
     **/
    AREXPORT bool blockingConnect(unsigned long connectTimeout = 8000);

    
    /** @brief Flags to indicates what the read() method did. 
     *  i.e. If nothing was done, then the
     *  result will be 0. To check a read() return result @a result to see if data was updated, use
     *  (result & ReadUpdated). To check if there was an error, use (result &
     *  ReadError). 
     */
    typedef enum { 
      ReadFinished = 0,
      ReadError = 1, 
      ReadData = 2,
      ReadUpdated = 4
    } ReadFlags;

    /** @brief Read some data from the device connection, and update stored data as complete messages are received. 
     * Return when no more data is available. 
     * @return ReadError if there was as error reading from the device connection,
     * Data if some data was read, ReadNoData if there was no data to read, or
     * ReadUpdated if data was read and the end of a message was encountered,
     * triggering any updates of stored data.
     */
    AREXPORT virtual int read();

    /** @brief Set whether checksum sent with NMEA messages is ignored */
    /*AREXPORT*/ void setIgnoreChecksum(bool ignore) { ignoreChecksum = ignore; }

    /** @brief Log last received data using ArLog. */
    AREXPORT void logData() const;

    /** Print basic navigation data on one line to standard output, with no newline at end. */
    AREXPORT void printData() const;

    /** @group Data accessors
     * @brief Access the last received data from the GPS */
    // @{
 
    typedef enum {  
        NoFix, BadFix, GPSFix, DGPSFix, PPSFix, 
        RTKinFix, FloatRTKinFix, DeadReckFix, 
        ManualFix, SimulatedFix, UnknownFixType 
     } FixType;

    /*AREXPORT*/ FixType getFixType() const { return myFixType; }
    AREXPORT const char* getFixTypeName() const;

    /*AREXPORT*/ bool havePosition() const { return myHavePosition; }
    /*AREXPORT*/ bool haveLatitude() const { return myHavePosition; }
    /*AREXPORT*/ bool haveLongitude() const { return myHavePosition; }

    /** @return latitude in decimal degrees */
    /*AREXPORT*/ double getLatitude() const { return myLatitude; }

    /** @return longitude in decimal degrees */
    /*AREXPORT*/ double getLongitude() const { return myLongitude; }

    /** @return copy of an ArTime object set to the time that ArGPS received latitude and longitude data. */
    /*AREXPORT*/ ArTime getTimeReceivedPosition() const { return myTimeGotPosition; }

    /*AREXPORT*/ bool haveSpeed() const { return myHaveSpeed; }

    /** @return GPS' measured speed converted to meters per second */
    /*AREXPORT*/ double getSpeed() const { return mySpeed; }

    /*AREXPORT*/ unsigned short getGPSPositionTimestamp() const { return myGPSPositionTimestamp; }

    /*AREXPORT*/ unsigned short getNumSatellitesTracked() const { return myNumSatellitesTracked; }
    /*AREXPORT*/ bool haveDGPSStation() const { return myHaveDGPSStation; }
    /*AREXPORT*/ unsigned short getDGPSStationID() const { return myDGPSStationID; }

    /** @return whether GPS provided a distance error estimation (only some Garmins do) */
    /*AREXPORT*/ bool havePositionError() const { return myHavePositionError; }
    /** GPS device's error estimation in meters */
    /*AREXPORT*/ double getPositionError() const { return myPositionError; }
    /** @return whether GPS provided an altitude error estimation (only some Garmins do) */
    /*AREXPORT*/ bool haveVerticalPositionError() const { return myHaveVerticalPositionError; }
    /// GPS device's error estimation in meters
    /*AREXPORT*/ double getVerticalPositionError() const { return myVerticalPositionError; }

    /// Have a compass heading value
    /*AREXPORT*/ bool haveCompassHeading() const { return myHaveCompassHeading; }
    /// Heading from magnetic north
    /*AREXPORT*/ double getCompassHeading() const { return myCompassHeading; }

    /*AREXPORT*/ bool haveAltitude() const { return myHaveAltitude; }
    /*AREXPORT*/ double getAltitude() const { return myAltitude; }

    /*AREXPORT*/ bool haveHDOP() const { return myHaveHDOP; }
    /*AREXPORT*/ double getHDOP() const { return myHDOP; }
    /*AREXPORT*/ bool haveVDOP() const { return myHaveVDOP; }
    /*AREXPORT*/ double getVDOP() const { return myVDOP; }
    /*AREXPORT*/ bool havePDOP() const { return myHavePDOP; }
    /*AREXPORT*/ double getPDOP() const { return myPDOP; }
    //@}

protected:
    /* Last data received */
    double myLatitude;
    double myLongitude;
    bool myHavePosition;
    ArTime myTimeGotPosition;   // Time we got the data
    double mySpeed;
    bool myHaveSpeed;
    unsigned short myGPSPositionTimestamp;   // Timestamp provided by GPS device
    FixType myFixType;
    unsigned short myNumSatellitesTracked;
    double myAltitude;
    bool myHaveAltitude;
    unsigned short myDGPSStationID;
    bool myHaveDGPSStation;
    double myPositionError;
    bool myHavePositionError;
    double myVerticalPositionError;
    bool myHaveVerticalPositionError;
    double myCompassHeading;
    bool myHaveCompassHeading;
    bool myHaveHDOP;
    double myHDOP;
    bool myHaveVDOP;
    double myVDOP;
    bool myHavePDOP;
    double myPDOP;

    /* Read a doubleing point number out of a std::string, if possible.
     * @return true if the string was nonempty and @a target was modified.
     */
    bool readFloatFromString(std::string& str, double* target, double(*convf)(double) = NULL);

    /* Read an unsigned short integer out of a std::string, if possible.
     * @return true if the string was nonempty and @a target was modified.
     */
    bool readUShortFromString(std::string& str, unsigned short* target, unsigned short (*convf)(unsigned short) = NULL);


    /* Read a double from a member of a vector of strings, if it exists. */
    bool readFloatFromStringVec(std::vector<std::string>* vec, size_t i, double* target, double (*convf)(double) = NULL);

    /* Read a double from a member of a vector of strings, if it exists. */
    bool readUShortFromStringVec(std::vector<std::string>* vec, size_t i, unsigned short* target, unsigned short (*convf)(unsigned short) = NULL);

    /* Convert DDDMM.MMMM to decimal degrees */
    static double gpsDegminToDegrees(double degmin);

    /* Convert US nautical knots to meters/sec */
    static double knotsToMPS(double knots);
 
    /* Convert meters/sec to miles/hour */
    static double mpsToMph(double mps) { return mps * 2.23693629; }

    /* Convert meters to US feet */
    static double metersToFeet(double m) { return m * 3.2808399; }

    /* Convert US feet  to meters */
    static double feetToMeters(double f) { return f / 3.2808399; }
    

    /* Connection info */
    ArDeviceConnection *myDevice;
    bool myCreatedOwnDeviceCon;
    ArRetFunctorC<bool, ArGPS> myParseArgsCallback; 
    ArArgumentParser* myArgParser;
    
    /* NMEA message, divided into parts.  */
    typedef std::vector<std::string> MessageVector;

    /* NMEA message handler type. 
     * XXX Is the reference OK for ArFunctor?  
     */
    typedef ArFunctor1C<ArGPS, MessageVector*> Handler;

    /* Map of message identifiers to handler functors */
    typedef std::map<std::string, Handler*> HandlerMap;

    /* NMEA message handlers used by ArGPS */
    HandlerMap myHandlers;

    void handleGPRMC(MessageVector* message);
    Handler myGPRMCHandler;

    void handleGPGGA(MessageVector* message);
    Handler myGPGGAHandler;

    void handlePGRME(MessageVector* message);
    Handler myPGRMEHandler;

    void handlePGRMZ(MessageVector* message);
    Handler myPGRMZHandler;

    void handleHCHDG(MessageVector* message);
    Handler myHCHDGHandler;

    void handleGPGSA(MessageVector* message);
    Handler myGPGSAHandler;

    /*  NMEA scanner state.
     *  There are possabilities for opmitization here, such 
     *  as just storing the read data in a buffer and handling
     *  each field as it is found in the buffer, or building
     *  a list of char* for each field pointing into the buffer
     *  instead of copying each field into a std::string in the
     *  currentMessage vector, etc. etc.
     */
    const unsigned short MaxNumFields;
    const unsigned short MaxFieldSize; // bytes
    bool ignoreChecksum;

    MessageVector currentMessage;
    std::string currentField;
    char checksumBuf[3];
    short checksumBufOffset;
    bool inChecksum;
    bool inMessage;
    char currentChecksum;
    bool gotCR;

      // update state
    void beginMessage();
    void endMessage();
    void nextField();
    void beginChecksum();

};

#endif // ifdef ARGPS_H
