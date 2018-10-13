/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef _CMT3_H_2006_04_14
#define _CMT3_H_2006_04_14

#ifndef _CMT_MONOLITHIC
#include "cmt2.h"
#include "cmtpacket.h"
#endif

namespace xsens
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Support  classes
///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt3
////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief High-level communication class.

	The class uses CMT level 2, but does not inherit from it. If software needs
   to access
	the level 2 components, it needs to be done through the getCmt2s() and
   getCmt2f() functions.

	All device identification is done through device Ids in CMT level 3.
*/
class Cmt3
{
	//// data & structures ////
   protected:
	/** The (optional) CMT level 2 serial object that this class operates on. */
	Cmt2s m_serial;
	/** The (optional) CMT level 2 logfile object that this class operates on.
	 */
	Cmt2f m_logFile;
	/** ms per sample = 1000 / sample frequency. */
	double m_rtcMsPerSample;
	// double m_sampleFrequency;		//!< The sample frequency of the port,
	// computed from sampling period and output skip factor.
	/** The sample period of the port. */
	uint16_t m_period;
	/** The skip factor of the port. */
	uint16_t m_skip;
	/** The start of the RTC counter, the time of arrival of sample 0. */
	TimeStamp m_rtcStart;
	/** The long sample counter (normal counter wraps at 64k). */
	uint32_t m_rtcCount;
	/** The last received sample counter, used to determine wrap-around. */
	CmtMtTimeStamp m_rtcLastSc;
	/** The baudrate that was last set to be used by the port. */
	uint32_t m_baudrate;
	/** The config mode timeout. */
	uint32_t m_timeoutConf;
	/** The measurement mode timeout. */
	uint32_t m_timeoutMeas;
	/** The last result of an operation. */
	mutable XsensResultValue m_lastResult;
	/** The index of the first formatting item. */
	uint16_t m_firstItem;
	/** The number of times a goto config is attempted before the function
	 * fails. */
	uint16_t m_gotoConfigTries;
	/** Keeps track of whether the connected device is measuring or being
	 * configured. */
	bool m_measuring;
	/** Automatically scan for device details during open. Default is true
	 * (recommended). */
	bool m_detailedScan;
	/** Indicates whether to read from the log file or from the serial port. */
	bool m_readFromFile;
	/** Indicates if the rtc is initialised or not. */
	bool m_rtcInitialized;
	/** Indicates whether to write all received messages to the logfile or not,
	 * automatically set to true by createLogFile. */
	bool m_logging;
	/** Contains the last error reported by hardware. */
	XsensResultValue m_lastHwError;
	/** Contains the Device ID of the device that caused the last hardware
	 * error. */
	CmtDeviceId m_lastHwErrorDeviceId;

   public:
	/** Indicates if the RTC should be computed or not (to save CPU time). */
	bool m_useRtc;
	/** Cached eMTS data. */
	void* m_eMtsData[CMT_MAX_DEVICES_PER_PORT];

   protected:
	/** The configuration of the connected devices. */
	CmtDeviceConfiguration m_config;

	//// functions
	//////////////////////////////////////////////////////////////////////////////////////
   protected:
	/** This object cannot be copied, so this function is not implemented. */
	Cmt3(const Cmt3& ref);
	/** Internal function to compute the RTC value. */
	void fillRtc(Packet* pack);
	/** \brief Find a device Id in the list and return its busId. \details
	 * CmtDeviceId 0= busId 0= broadcast. A CMT_BID_INVALID value is also
	 * possible and indicates that the devId was not found. */
	uint8_t getBusIdInternal(const CmtDeviceId devId) const;

   public:
	/** Default constructor, initializes all members to their default values. */
	Cmt3();
	/** Destructor, de-initializes, frees memory allocated for buffers, etc. */
	~Cmt3();

	/** \brief Close the communication port. \details This function places the
	 * device in configuration mode and closes the communication port, ending
	 * all further communication with the device. */
	XsensResultValue closePort(bool gotoConfigFirst = true);
	/** \brief Get an Xbus Master's battery level. \details The battery level is
	 * a value between 0 and 255 that indicates the voltage of the batteries.
	 * The scale is not linear and the values should not be used as an absolute
	 * voltage. The amount of time remaining for measurement given any battery
	 * level greatly depends on the type of batteries used, the number of
	 * sensors attached to the Xbus Master and the used output options. */
	XsensResultValue getBatteryLevel(uint8_t& level);
	/** \brief Get the baudrate used by a port. \details This function returns
	 * the baud rate at which the port is currently connected. The function will
	 * return an error when no port is connected. \see getSerialBaudrate
	 * setSerialBaudrate setBaudrate */
	XsensResultValue getBaudrate(uint32_t& baudrate);
	/** \brief Get the state of the bluetooth communication. \details This
	 * function tells whether the Bluetooth connection of the Xbus Master is on
	 * (1) or off (0). \note This function is only valid in configuration mode.
	 */
	XsensResultValue getBluetoothState(bool& enabled);
	/** \brief Retrieve the BusId of a device. \details The function checks its
	 * internal list for a match of deviceId. If it is found, the corresponding
	 * BusId is returned in busId. Otherwise, a 0 is placed in busId and
	 * XRV_NOTFOUND result is returned. */
	XsensResultValue getBusId(
		uint8_t& busId, const CmtDeviceId deviceId = CMT_DID_MASTER) const;
	/** \brief Get the state of the Xbus power. \details This function tells
	 * whether the Xbus of the connected Xbus Master is currently switched on
	 * (1) or not (0). When it is switched off, the attached MT devices have no
	 * power and communication with them is not possible. Before going to
	 * measurement mode, use setBusPowerState to restore power. \see
	 * setBusPowerState \note This function is only valid in configuration mode.
	 */
	XsensResultValue getBusPowerState(bool& enabled);

	/** \brief Return a reference to the embedded Cmt2f (logfile) object.
	 * \details Any manipulation of the object should be done through Cmt3.
	 * Cmt3's integrity is not guaranteed if the Cmt2f object is manipulated
	 * directly. \see refreshCache */
	Cmt2f* getCmt2f(void);
	/** \brief Return a reference to the embedded Cmt2s (comm port) object.
	 * \details Any manipulation of the object should be done through Cmt3.
	 * Cmt3's integrity is not guaranteed if the Cmt2s object is manipulated
	 * directly. \see refreshCache */
	Cmt2s* getCmt2s(void);

	/** \brief Get device configuration. \details This function retrieves the
	 * complete device configuration of a single device. \note This information
	 * is cached by Cmt3, so it is available in measurement mode. */
	XsensResultValue getConfiguration(CmtDeviceConfiguration& configuration);
	/** \brief Retrieve data size. \details This function retrieves the number
	 * of bytes that are in a data message as sent by the given device. */
	XsensResultValue getDataLength(
		uint32_t& length, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Retrieve total device count. \details This function retrieves the
	 * total number of connected (master + slave) devices or 0 if not connected.
	 */
	uint32_t getDeviceCount(void) const;
	/** \brief Retrieve the DeviceId of a device given its Bus ID. \details This
	 * function retrieves the DeviceId for the device with the given Bus ID.
	 * When no devices are connected, a 0 ID is supplied. */
	XsensResultValue getDeviceId(
		const uint8_t busId, CmtDeviceId& deviceId) const;
	/** \brief Return device mode. \details This function retrieves the
	 * output-related settings of the device, such as the sample rate and output
	 * settings. \see setDeviceMode \note This function actually reads the
	 * device mode from the cached configuration, so it is available in
	 * measurement mode. \see refreshCache */
	XsensResultValue getDeviceMode(
		CmtDeviceMode& mode, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Return device mode2. \details This function retrieves the
	 * output-related settings of the device, such as the period, skip factor
	 * and output settings. \see setDeviceMode \note This function actually
	 * reads the device mode from the cached configuration, so it is available
	 * in measurement mode. \see refreshCache */
	XsensResultValue getDeviceMode2(
		CmtDeviceMode2& mode, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Retrieve the eMts data of the specified sensor(s). \details This
	 * function can be used to read proprietary data from one or more Motion
	 * Trackers. This data is required by higher level functions in combination
	 * with Configuration data to convert Raw data into Calibrated and
	 * Orientation data. The eMTs data is quite large, but it is cached. The
	 * first request should be done in configuration mode, but following
	 * requests can be done in measurement mode. When requesting eMTS data for a
	 * single sensor, the buffer should be at least CMT_EMTS_SIZE bytes long.
	 * When using CMT_DID_BROADCAST, the eMTS data of all connected sensors is
	 * placed into the buffer sequentially. In the latter case, the buffer
	 * should be able to hold at least sensorcount * CMT_EMTS_SIZE bytes. */
	XsensResultValue getEMtsData(
		void* buffer, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Return the error mode of the device. \details This function
	 * returns the error mode of the device. The error mode determines how the
	 * device handles errors. See the low-level communication documentation for
	 * more details. \see setErrorMode \note This function is only valid in
	 * configuration mode. */
	XsensResultValue getErrorMode(
		uint16_t& mode, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Return Firmware revision. \details This function retrieves the
	 * firmware version that is currently installed in the device. \note This
	 * function is only valid in configuration mode. */
	XsensResultValue getFirmwareRevision(
		CmtVersion& revision, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Return the heading offset. \details This function retrieves the
	 * heading offset in radians used by the device. The valid range is -pi to
	 * +pi. The heading offset is used as a final correction on the output
	 * orientation. \see setHeading getMagneticDeclination
	 * setMagneticDeclination \note This function is only valid in configuration
	 * mode. */
	XsensResultValue getHeading(
		double& heading, const CmtDeviceId deviceId = CMT_DID_MASTER);

	/** \brief Return the error code of the last user function call. */
	XsensResultValue getLastResult(void) const { return m_lastResult; }
	/** \brief Return the last Hardware error code. \details This function
	 * returns the XsensResultValue of the last problem reported by hardware (if
	 * any). Hardware problems are all 'error' messages returned by a sensor.
	 * \param did If any problems were found, the responsible device ID will be
	 * returned in this parameter. \see clearHwError */
	XsensResultValue getHwError(CmtDeviceId& did) const
	{
		did = m_lastHwErrorDeviceId;
		return m_lastHwError;
	}
	/** \brief Reset the hardware error code. \details Use this function to
	 * reset the hardware error code reported by getHwError. \see getHwError */
	void clearHwError(void)
	{
		m_lastHwErrorDeviceId = 0;
		m_lastHwError = XRV_OK;
	}

	/** \brief Return the location ID of a sensor. \details This function
	 * retrieves the location ID stored in the device. \see setLocationId \note
	 * This function is only valid in configuration mode. */
	XsensResultValue getLocationId(
		uint16_t& locationId, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Retrieve the read position of the log file. \details This
	 * function will return the current read position in the open log file in
	 * bytes from the start. \note The read and write positions of log files are
	 * completely independent of each other. \remarks To change the read
	 * position, either use resetLogFileReadPos or manipulate the log file
	 * through getCmt2f. \see resetLogFileReadPos getCmt2f */
	XsensResultValue getLogFileReadPosition(CmtFilePos& pos);

	/** \brief Retrieve the size of the open log file in bytes. */
	XsensResultValue getLogFileSize(CmtFilePos& size);
	/** \brief Retrieve the name of the open log file or an empty string if no
	 * logfile is open */
	XsensResultValue getLogFileName(char* filename);
	/** \brief Retrieve the name of the open log file or an empty string if no
	 * logfile is open */
	XsensResultValue getLogFileName(wchar_t* filename);
	/** \brief Return the stored magnetic declination. \details This function
	 * retrieves the stored local magnetic declination in radians. The valid
	 * range is -pi to +pi. The magnetic declination is used in the sensor
	 * fusion process to determine the output orientation. \see getHeading
	 * setHeading setMagneticDeclination \note This function is only valid in
	 * configuration mode. */
	XsensResultValue getMagneticDeclination(
		double& declination, const CmtDeviceId deviceId = CMT_DID_MASTER);
	CmtDeviceId getMasterId(void);  //! \brief Return the device Id of the first
	//! device (master). \details \note The
	//! deviceId is read from the cached
	//! configuration data, so it is also
	//! available in measurement mode.

	/** \brief Retrieve number of MT devices. \details This function will return
	 * the number of connected MT devices. Effectively, this returns the device
	 * count minus any Xbus Masters. */
	uint16_t getMtCount(void) const;
	XsensResultValue getMtDeviceId(const uint8_t index, CmtDeviceId& deviceId)
		const;  //! \brief Return the device Id of an MT device. \details This
	//! function returns the ID of the index'th MT (non-Xbus Master)
	//! device connected to this object. \note The deviceId is read
	//! from the cached configuration data, so it is also available
	//! in measurement mode.
	/** \brief Return the port number that this object is connected to. \details
	 * If CMT is reading from a log file, an error will be returned. */
	XsensResultValue getPortNr(uint8_t& port) const;
	/** \brief Return product code. \details This function retrieves the product
	 * code of the given device. \param productCode The buffer that will store
	 * the product code. This buffer should be at least 20 bytes. \remarks The
	 * product code is NOT necessarily NULL-terminated. If it is less than 20
	 * characters it will be, but if the product code is 20 characters it won't
	 * be. The recommended method is to create a buffer that is 21 bytes long
	 * and set the last byte to 0 before calling this function. \note This
	 * function is only valid in configuration mode. */
	XsensResultValue getProductCode(
		char* productCode, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Return current sample frequency. \details This function
	 * determines the sample frequency of the device from the cached sample rate
	 * and skip factor and returns it. For devices connected to an Xbus Master,
	 * the values used by the Xbus Master are returned. \see setDeviceMode
	 * getDeviceMode */
	uint16_t getSampleFrequency(void);
	/** \brief Return the baud rate used for serial communication. \details This
	 * function retrieves the baud rate that is used when the device is
	 * connected by a serial connection. In most cases this is the same as
	 * getBaudrate, but when an Xbus Master is connected by a Bluetooth
	 * connection, it doesn't have to be. \see setSerialBaudrate setBaudrate
	 * getBaudrate \note This function is only valid in configuration mode. */
	XsensResultValue getSerialBaudrate(uint32_t& baudrate);
	/** \brief Retrieve the inbound synchronization settings of the master MT
	 * device. \details This function retrieves the current inbound
	 * synchronization settings of the master MT device (sync mode, skip factor
	 * and offset). This function does not work for Xbus Masters and should not
	 * be used for sensors connected to an Xbus Master. \see setSyncInSettings
	 * getSyncOutSettings setSyncOutSettings getSyncMode setSyncMode \note This
	 * function is only valid in configuration mode. */
	XsensResultValue getSyncInSettings(CmtSyncInSettings& settings);
	/** \brief Retrieve the inbound synchronization mode of an MT device.
	 * \details This function retrieves the current inbound synchronization mode
	 * of the MT device. This function does not work for Xbus Masters and should
	 * not be used for sensors connected to an Xbus Master. \see setSyncInMode
	 * setSyncInSettings getSyncInSettings getSyncInSkipFactor
	 * setSyncInSkipFactor getSyncInOffset setSyncInOffset getSyncOutSettings
	 * setSyncOutSettings getSyncMode setSyncMode */
	XsensResultValue getSyncInMode(uint16_t& mode);
	/** \brief Retrieve the inbound synchronization skip factor of an MT device.
	 * \details This function retrieves the current inbound synchronization skip
	 * factor of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * setSyncInSkipFactor setSyncInSettings getSyncInSettings getSyncInOffset
	 * setSyncInOffset getSyncOutSettings setSyncOutSettings getSyncMode
	 * setSyncMode */
	XsensResultValue getSyncInSkipFactor(uint16_t& skipFactor);
	/** \brief Retrieve the inbound synchronization offset of an MT device.
	 * \details This function retrieves the current inbound synchronization
	 * offset of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * setSyncInOffset setSyncInSettings getSyncInSettings getSyncInSkipFactor
	 * setSyncInSkipFactor getSyncOutSettings setSyncOutSettings getSyncMode
	 * setSyncMode */
	XsensResultValue getSyncInOffset(uint32_t& offset);
	/** \brief Retrieve the sync mode of the Xbus Master. \details This function
	 * requests the current synchronization mode used by the specified Xbus
	 * Master. This function is not valid for MT devices. \see setSyncMode
	 * getSyncInSettings setSyncInSettings getSyncOutSettings setSyncOutSettings
	 * \note This function is only valid in configuration mode. */
	XsensResultValue getSyncMode(uint8_t& mode);
	/** \brief Retrieve the outbound synchronization settings of the master MT
	 * device. \details This function retrieves the current outbound
	 * synchronization settings of the MT device (sync mode, skip factor, offset
	 * and pulse width). This function does not work for Xbus Masters and should
	 * not be used for sensors connected to an Xbus Master. \see
	 * setSyncOutSettings getSyncInSettings setSyncInSettings getSyncMode
	 * setSyncMode \note This function is only valid in configuration mode. */
	XsensResultValue getSyncOutSettings(CmtSyncOutSettings& settings);
	/** \brief Retrieve the outbound synchronization mode of an MT device.
	 * \details This function retrieves the current outbound synchronization
	 * mode of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * setSyncOutMode setSyncOutSettings getSyncOutSettings getSyncOutPulseWidth
	 * setSyncOutPulseWidth getSyncOutSkipFactor setSyncOutSkipFactor
	 * getSyncOutOffset setSyncOutOffset getSyncInSettings setSyncInSettings
	 * getSyncMode setSyncMode */
	XsensResultValue getSyncOutMode(uint16_t& mode);
	/** \brief Retrieve the outbound synchronization pulse width of an MT
	 * device. \details This function retrieves the current outbound
	 * synchronization pulse width of the MT device. This function does not work
	 * for Xbus Masters and should not be used for sensors connected to an Xbus
	 * Master. \see setSyncOutPulseWidth setSyncOutSettings getSyncOutSettings
	 * setSyncOutMode getSyncOutMode getSyncOutSkipFactor setSyncOutSkipFactor
	 * getSyncOutOffset setSyncOutOffset getSyncInSettings setSyncInSettings
	 * getSyncMode setSyncMode */
	XsensResultValue getSyncOutPulseWidth(uint32_t& pulseWidth);
	/** \brief Retrieve the outbound synchronization skip factor of an MT
	 * device. \details This function retrieves the current outbound
	 * synchronization skip factor of the MT device. This function does not work
	 * for Xbus Masters and should not be used for sensors connected to an Xbus
	 * Master. \see stSyncOutSkipFactor setSyncOutSettings getSyncOutSettings
	 * setSyncOutMode getSyncOutMode getSyncOutPulseWidth setSyncOutPulseWidth
	 * getSyncOutOffset setSyncOutOffset getSyncInSettings setSyncInSettings
	 * getSyncMode setSyncMode */
	XsensResultValue getSyncOutSkipFactor(uint16_t& skipFactor);
	/** \brief Retrieve the outbound synchronization offset of an MT device.
	 * \details This function retrieves the current outbound synchronization
	 * offset of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * setSyncOutOffset setSyncOutSettings getSyncOutSettings setSyncOutMode
	 * getSyncOutMode getSyncOutPulseWidth setSyncOutPulseWidth
	 * getSyncOutSkipFactor setSyncOutSkipFactor getSyncInSettings
	 * setSyncInSettings getSyncMode setSyncMode */
	XsensResultValue getSyncOutOffset(uint32_t& offset);
	/** \brief Return the configuration mode timeout. \details When in config
	 * mode, the system uses a different message timeout than in measurement
	 * mode, since configuration messages can take longer to process than
	 * measurement mode messages. This function can be used to determine the
	 * current config mode timeout in ms. \see setTimeoutConfig
	 * getTimeoutMeasurement setTimeoutMeasurement */
	uint32_t getTimeoutConfig(void) const;
	/** \brief Return the measurement mode timeout. \details When in measurement
	 * mode, the system uses a different message timeout than in config mode,
	 * since measurement mode messages should be faster to process than config
	 * mode messages. This function can be used to determine the current
	 * measurement mode timeout in ms. \see setTimeoutMeasurement
	 * getTimeoutConfig setTimeoutConfig */
	uint32_t getTimeoutMeasurement(void) const;
	/** \brief Return the UTC time of the last received sample. \details This
	 * function is only valid for MTi-G sensors. In measurement mode it will
	 * retrieve the UTC time of the last received sample. In Config mode, it
	 * will retrieve the most recent UTC time. In config mode, the time returned
	 * will be requested diectly from the GPS subsystem, with several layers of
	 * messaging between the original source and the host. In measurement mode,
	 * the UTC time is requested from the GPS subsystem periodically and
	 * estimated (with a very high precision) for the remaining samples. So in
	 * config mode, the time will probably jitter more than in measurement mode.
	 */
	XsensResultValue getUtcTime(CmtUtcTime& utc, const CmtDeviceId deviceId);
	/** \brief Return the dual-output mode of the XM. \details This function
	 * retrieves the dual mode output mode of the Xbus Master. The dual output
	 * mode of the Xbus Master defines whether it will send data on the serial
	 * connection (at the same baud rate as the bluetooth connection) when it is
	 * connected via Bluetooth. When set to 0, the serial communication is
	 * disabled, data is sent over the serial bus. \note When dual-mode is
	 * enabled, the Xbus Master will NOT receive command messages on the serial
	 * bus. \see setXmOutputMode */
	XsensResultValue getXmOutputMode(uint8_t& mode);
	/** \brief Place all connected devices into Configuration Mode. \details
	 * This function places the sensors in configuration mode. \note This
	 * function has no effect when reading from a log file */
	XsensResultValue gotoConfig(void);
	/** \brief Place all connected devices into Measurement Mode. \details This
	 * function places the sensors in measurement mode. \note This function has
	 * no effect when reading from a log file */
	XsensResultValue gotoMeasurement(void);
	/** \brief Perform an initBus request. \details See the low-level
	 * documentation for more information on the InitBus message. \see
	 * refreshCache setBusPowerState */
	XsensResultValue initBus(void);
	/** \brief Return whether the Cmt3 object is writing to a log file or not */
	bool isLogging(void) const { return m_logging; }
	/** \brief Return whether the communication port is open or not. */
	bool isPortOpen(void) const { return (m_serial.isOpen()); }
	/** \brief Return whether the main device is an Xbus Master or not. */
	bool isXm(void) const;

	XsensResultValue openPort(
		const char* portName,
		const uint32_t baudRate =
			CMT_DEFAULT_BAUD_RATE);  //! \brief Open a communication channel to
//! the given COM port number. \details
//! This function is first passed through
//! to the Cmt2s object. Then, the device
//! settings are retrieved and stored
//! locally. This function automatically
//! places the device(s) in config mode,
//! using gotoConfig. \see gotoConfig
//! closePort
#ifdef _WIN32
	XsensResultValue openPort(
		const uint32_t portNumber,
		const uint32_t baudRate =
			CMT_DEFAULT_BAUD_RATE);  //! \brief Open a communication channel to
//! the given COM port number. \details
//! This function is first passed through
//! to the Cmt2s object. Then, the device
//! settings are retrieved and stored
//! locally. This function automatically
//! places the device(s) in config mode,
//! using gotoConfig. \see gotoConfig
//! closePort
#endif
	/** \brief Peek(take a look) at the message ID of the next message. \details
	 * This function can only be used when reading from a log file. It will find
	 * the next message in the file and place its message ID in the messageId
	 * parameter. Afterwards, the read position of the file will be restored.
	 * \remarks This function is mostly useful when dealing with a file that has
	 * more than just data messages. By using the peek function, it is possible
	 * to decide whether a readDataPacket should be called or for example
	 * getBatteryLevel or getGpsStatus. When the peek function is not used and
	 * for example getBatteryLevel is called, all messages between the current
	 * read position and the first battery level message in the file will be
	 * skipped. An alternative would be to get the read position, call the
	 * desired function (get battery level) and restore the read position, but
	 * then the moment at which the (battery level, UTC time, satellite info,
	 * etc) data becomes available will not be known. */
	XsensResultValue peekLogMessageId(uint8_t& messageId);
	/** \brief Retrieve a data message. \details This function will attempt to
	 * read a data message from the open port or from the log file, depending on
	 * the system state. When acceptOther is set to true, the first received
	 * message will be returned. If a data message is successfully read, XRV_OK
	 * will be returned. If another message is read, XRV_OTHER will be returned
	 * and the received message will be placed in the Packet. Otherwise, an
	 * appropriate error will be returned. */
	XsensResultValue readDataPacket(Packet* pack, bool acceptOther = false);
	XsensResultValue requestData(Packet* pack);  //!\brief Request a data
	//! message and wait for it to
	//! arrive. \details This
	//! function is only useful when
	//! the skip factor is set to
	//! 0xFFFF.
	/** \brief Reset all connected sensors. */
	XsensResultValue reset(void);
	/** \brief Perform an orientation reset on a device. \details This function
	 * performs an orientation reset. See the MT documentation for more
	 * information about Orientation resets. \note If you wish to save the
	 * setting to the device, perform the CMT_RESETORIENTATION_STORE operation
	 * while in Config mode after having performed the appropriate orientation
	 * reset in measurement mode. */
	XsensResultValue resetOrientation(
		const CmtResetMethod method,
		const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Restore the device to factory default settings. \details This
	 * function completely restores the selected device to the default settings
	 * (115k2 baud rate, 100Hz sample frequency, factory defined scenarios).
	 * \note This function is only available in configuration mode. */
	XsensResultValue restoreFactoryDefaults(
		const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the baudrate and possibly reconnect. \details Use this
	 * function to change the baudrate of the device. This actually tries to
	 * change the baud rate of the current connection. When reconnect is set to
	 * true, the device receives a Reset message and the port is reopened at the
	 * new baudrate. Otherwise, make sure to perform a Reset manually, since the
	 * new baudrate will not be set until a Reset has been performed. \note To
	 * change the baudrate of the serial connection while connected via
	 * Bluetooth, use setSerialBaudrate. \see getBaudrate setSerialBaudrate
	 * getSerialBaudrate \note This function is only available in configuration
	 * mode. */
	XsensResultValue setBaudrate(
		const uint32_t baudrate, bool reconnect = true);
	/** \brief Set the Bluetooth state of the Xbus Master. \details This
	 * function sets the state of the bluetooth communication to on or off.
	 * \note This function is only available in configuration mode. */
	XsensResultValue setBluetoothState(const bool enabled);
	/** \brief Switch the Xbus Master bus power on or off. \details Use this
	 * function to switch the Xbus Master Xbus power on or off. \remarks This
	 * function can be used to save a lot of power when not measuring, while
	 * still keeping a connection to the system. However, there is a relatively
	 * long startup time when restoring power as the sensors are reinitialized.
	 * \note You will need to perform an initBus and possibly a refreshCache
	 * call after switching the power back on with this function. \note This
	 * function is only available in configuration mode. \see initBus
	 * refreshCache */
	XsensResultValue setBusPowerState(const bool enabled);
	/** \brief Set the complete output mode of a device. \details This function
	 * updates the complete output mode of the specified device. It only updates
	 * values that are different than those reported by the device unless force
	 * is set to true. The function will automatically update only the part of
	 * the device mode that is relevant for the device. So it is possible to
	 * configure all devices, including an Xbus Master with the same mode (only
	 * the Xbus Master will update its period, while the Motion Trackers will
	 * update their output mode and settings). \note This function is only
	 * available in configuration mode. \see getDeviceMode getDeviceMode2
	 * setDeviceMode2 */
	XsensResultValue setDeviceMode(
		const CmtDeviceMode& mode, bool force,
		const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the complete output mode2 of a device. \details This function
	 * updates the complete output mode of the specified device. It only updates
	 * values that are different than those reported by the device unless force
	 * is set to true. The function will automatically update only the part of
	 * the device mode that is relevant for the device. So it is possible to
	 * configure all devices, including an Xbus Master with the same mode (only
	 * the Xbus Master will update its period, while the Motion Trackers will
	 * update their output mode and settings). \note This function is only
	 * available in configuration mode. \see getDeviceMode2 getDeviceMode
	 * setDeviceMode */
	XsensResultValue setDeviceMode2(
		const CmtDeviceMode2& mode, bool force,
		const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the error mode of the device. \details This function sets the
	 * error mode of the device. The error mode determines how the device
	 * handles errors. See the low-level communication documentation for more
	 * details. \note This function is only available in configuration mode.
	 * \see getErrorMode */
	XsensResultValue setErrorMode(const uint16_t mode);
	/** \brief Set the number of times the gotoConfig function will attempt a
	 * gotoConfig before failing. \details This is especially useful when using
	 * RS485 communication or when for some reason the communication lines are
	 * not reliable. */
	XsensResultValue setGotoConfigTries(const uint16_t tries);
	/** \brief Set the heading offset. \details This function sets the heading
	 * offset in radians used by the device. The valid range is -pi to +pi. The
	 * heading offset is used as a final correction on the output orientation.
	 * \note This function is only available in configuration mode. \see
	 * getHeading getMagneticDeclination setMagneticDeclination */
	XsensResultValue setHeading(
		const double heading, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the location ID of a sensor. \details This function sets the
	 * location ID stored in the device. \note This function is only available
	 * in configuration mode. \see getLocationId */
	XsensResultValue setLocationId(
		uint16_t locationId, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the stored magnetic declination. \details This function sets
	 * the stored local magnetic declination in radians. The valid range is -pi
	 * to +pi. The magnetic declination is used in the sensor fusion process to
	 * determine the output orientation. \note This function is only available
	 * in configuration mode. \see getHeading setHeading getMagneticDeclination
	 */
	XsensResultValue setMagneticDeclination(
		const double declination, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the inbound synchronization settings of an MT device.
	 * \details This function sets the current inbound synchronization settings
	 * of the MT device (sync mode, skip factor and offset). This function does
	 * not work for Xbus Masters and should not be used for sensors connected to
	 * an Xbus Master. \note This function is only available in configuration
	 * mode. \see getSyncInSettings getSyncOutSettings setSyncOutSettings
	 * getSyncMode setSyncMode */
	XsensResultValue setSyncInSettings(const CmtSyncInSettings& settings);
	/** \brief Set the inbound synchronization mode of an MT device. \details
	 * This function sets the current inbound synchronization mode of the MT
	 * device. This function does not work for Xbus Masters and should not be
	 * used for sensors connected to an Xbus Master. \see getSyncInMode
	 * setSyncInSettings getSyncInSettings getSyncInSkipFactor
	 * setSyncInSkipFactor getSyncInOffset setSyncInOffset getSyncOutSettings
	 * setSyncOutSettings getSyncMode setSyncMode */
	XsensResultValue setSyncInMode(const uint16_t mode);
	/** \brief Set the inbound synchronization skip factor of an MT device.
	 * \details This function sets the current inbound synchronization skip
	 * factor of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * getSyncInSkipFactor setSyncInSettings getSyncInSettings getSyncInOffset
	 * setSyncInOffset getSyncOutSettings setSyncOutSettings getSyncMode
	 * setSyncMode */
	XsensResultValue setSyncInSkipFactor(const uint16_t skipFactor);
	/** \brief Set the inbound synchronization offset of an MT device. \details
	 * This function sets the current inbound synchronization offset of the MT
	 * device. This function does not work for Xbus Masters and should not be
	 * used for sensors connected to an Xbus Master. \see getSyncInOffset
	 * setSyncInSettings getSyncInSettings getSyncInSkipFactor
	 * setSyncInSkipFactor getSyncOutSettings setSyncOutSettings getSyncMode
	 * setSyncMode */
	XsensResultValue setSyncInOffset(const uint32_t offset);
	/** \brief Set the sync mode of the Xbus Master. \details This function sets
	 * the current synchronization mode used by the specified Xbus Master. This
	 * function is not valid for MT devices. \note This function is only
	 * available in configuration mode. \see getSyncMode getSyncInSettings
	 * setSyncInSettings getSyncOutSettings setSyncOutSettings */
	XsensResultValue setSyncMode(const uint8_t mode);
	/** \brief Set the outbound synchronization settings of an MT device.
	 * \details This function sets the current outbound synchronization settings
	 * of the MT device (sync mode, skip factor, offset and pulse width). This
	 * function does not work for Xbus Masters and should not be used for
	 * sensors connected to an Xbus Master. \note This function is only
	 * available in configuration mode. \see setSyncOutSettings
	 * getSyncInSettings setSyncInSettings getSyncMode setSyncMode */
	XsensResultValue setSyncOutSettings(const CmtSyncOutSettings& settings);
	/** \brief Set the outbound synchronization mode of an MT device. \details
	 * This function sets the current outbound synchronization mode of the MT
	 * device. This function does not work for Xbus Masters and should not be
	 * used for sensors connected to an Xbus Master. \see getSyncOutMode
	 * setSyncOutSettings getSyncOutSettings getSyncOutPulseWidth
	 * setSyncOutPulseWidth getSyncOutSkipFactor setSyncOutSkipFactor
	 * getSyncOutOffset setSyncOutOffset getSyncInSettings setSyncInSettings
	 * getSyncMode setSyncMode */
	XsensResultValue setSyncOutMode(const uint16_t mode);
	/** \brief Set the outbound synchronization pulse width of an MT device.
	 * \details This function sets the current outbound synchronization pulse
	 * width of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * getSyncOutPulseWidth setSyncOutSettings getSyncOutSettings setSyncOutMode
	 * getSyncOutMode getSyncOutSkipFactor setSyncOutSkipFactor getSyncOutOffset
	 * setSyncOutOffset getSyncInSettings setSyncInSettings getSyncMode
	 * setSyncMode */
	XsensResultValue setSyncOutPulseWidth(const uint32_t pulseWidth);
	/** \brief Set the outbound synchronization skip factor of an MT device.
	 * \details This function sets the current outbound synchronization skip
	 * factor of the MT device. This function does not work for Xbus Masters and
	 * should not be used for sensors connected to an Xbus Master. \see
	 * setSyncOutSkipFactor setSyncOutSettings getSyncOutSettings setSyncOutMode
	 * getSyncOutMode getSyncOutPulseWidth setSyncOutPulseWidth getSyncOutOffset
	 * setSyncOutOffset getSyncInSettings setSyncInSettings getSyncMode
	 * setSyncMode */
	XsensResultValue setSyncOutSkipFactor(const uint16_t skipFactor);
	/** \brief Set the outbound synchronization offset of an MT device. \details
	 * This function sets the current outbound synchronization offset of the MT
	 * device. This function does not work for Xbus Masters and should not be
	 * used for sensors connected to an Xbus Master. \see getSyncOutOffset
	 * setSyncOutSettings getSyncOutSettings setSyncOutMode getSyncOutMode
	 * getSyncOutPulseWidth setSyncOutPulseWidth getSyncOutSkipFactor
	 * setSyncOutSkipFactor getSyncInSettings setSyncInSettings getSyncMode
	 * setSyncMode */
	XsensResultValue setSyncOutOffset(const uint32_t offset);
	/** \brief Set the default timeout value to use in blocking operations on
	 * the communication port. */
	XsensResultValue setTimeout(const uint32_t ms)
	{
		return (m_lastResult = m_serial.setTimeout(ms));
	}
	/** \brief Set the configuration mode timeout. \details When in config mode,
	 * the system uses a different message timeout than in measurement mode,
	 * since configuration messages can take longer to process than measurement
	 * mode messages. This function can be used to set the config mode timeout
	 * in ms. \note This function is only available in configuration mode. \see
	 * getTimeoutConfig getTimeoutMeasurement setTimeoutMeasurement setTimeout
	 */
	XsensResultValue setTimeoutConfig(
		const uint32_t timeout = CMT3_DEFAULT_TIMEOUT_CONF);
	/** \brief Set the measurement mode timeout. \details When in measurement
	 * mode, the system uses a different message timeout than in config mode,
	 * since measurement mode messages should be faster to process than config
	 * mode messages. This function can be used to set the measurement mode
	 * timeout in ms. \note This function is only available in configuration
	 * mode. \see getTimeoutMeasurement getTimeoutConfig setTimeoutConfig
	 * setTimeout */
	XsensResultValue setTimeoutMeasurement(
		const uint32_t timeout = CMT3_DEFAULT_TIMEOUT_MEAS);
	/** \brief Set the dual-output mode of the XM. \details This function sets
	 * the dual mode output mode of the Xbus Master. The dual output mode of the
	 * Xbus Master defines whether it will send data on the serial connection
	 * (at the same baud rate as the bluetooth connection) when it is connected
	 * via Bluetooth. When set to 0, the serial communication is disabled, data
	 * is sent over the serial bus. \note When dual-mode is enabled, the Xbus
	 * Master will NOT receive commands on the serial bus. \note This function
	 * is only available in configuration mode. \see getXmDualOutputMode */
	XsensResultValue setXmOutputMode(const uint8_t mode);
	/** \brief Switch the connected Xbus Master. \details This function tell the
	 * connected Xbus Master to power down. This differs from setBusPowerState,
	 * because this function actually powers down the Xbus Master itself, while
	 * setBusPowerState only powers down the bus controlled by the Xbus Master.
	 * After This function is called, the Xbus Master must be manually switched
	 * on to make it operational again. \see setBusPowerState */
	XsensResultValue setXmPowerOff(void);
	/** Update device information stored on host PC. \details Some device
	 * information is cached on the host PC for faster access. The Cmt3
	 * automatically tries to keep the cache up to date. But when unexpected
	 * things happen, such as custom messages that change the settings, or a
	 * power-cycle, then the user may need to tell Cmt3 to update its cache.
	 * When both a file and a port are open, the file parameter can be used to
	 * indicate what data source to read from (true is file, false is port).
	 * \note This function is only available in configuration mode. */
	XsensResultValue refreshCache(const bool file = false);
	/** \brief Wait for a data message to arrive. \details The function waits
	 * for a data message to arrive or until a timeout occurs. The function will
	 * also accept error messages. */
	XsensResultValue waitForDataMessage(Packet* pack);
	/** \brief Create a log file for incoming messages. \details This function
	 * creates a log file for recording incoming messages. The Cmt3 object must
	 * be connected to a communication port or the function will fail. \param
	 * filename The name of the file to use as a log file. A fully qualified
	 * (local) path+filename is recommended. \param startLogging When set to
	 * true (default), Cmt3 will immediately start logging incoming messages to
	 * the file, otherwise setLogMode must be called first before logging is
	 * started. \remarks If a writable file of the same name already exists in
	 * the target location, it will be overwritten without warning. \see
	 * closeLogFile isLogFileOpen openLogFile setLogMode \note This function is
	 * available in configuration AND in measurement mode. */
	XsensResultValue createLogFile(
		const char* filename, bool startLogging = false);
	/** \brief Create a log file for incoming messages. \details This function
	 * creates a log file for recording incoming messages. The Cmt3 object must
	 * be connected to a communication port or the function will fail. \param
	 * filename The name of the file to use as a log file. A fully qualified
	 * (local) path+filename is recommended. \param startLogging When set to
	 * true (default), Cmt3 will immediately start logging incoming messages to
	 * the file, otherwise setLogMode must be called first before logging is
	 * started. \remarks If a writable file of the same name already exists in
	 * the target location, it will be overwritten without warning. \see
	 * closeLogFile isLogFileOpen openLogFile setLogMode \note This function is
	 * available in configuration AND in measurement mode. */
	XsensResultValue createLogFile(
		const wchar_t* filename, bool startLogging = false);
	/** \brief Close an open log file. \details This function closes the logfile
	 * if it was open. \param del When set to true, the file will be deleted
	 * after closing if it is not read-only. This is mostly useful for temporary
	 * log-files.	\see createLogFile isLogFileOpen openLogFile setLogMode */
	XsensResultValue closeLogFile(bool del = false);
	/** \brief Return whether or not(true or false) the supplied file is open.
	 * \details When the filename parameter is nullptr or empty "", the function
	 * will simply return whether a log file is open. Otherwise, the supplied
	 * name will be checked against the name of the open log file as well.
	 * \return true if the log file is open, false if it isn't \see closeLogFile
	 * createLogFile openLogFile setLogMode */
	bool isLogFileOpen(const char* filename) const;
	/** \brief Return whether or not(true or false) the supplied file is open.
	 * \details When the filename parameter is nullptr or empty "", the function
	 * will simply return whether a log file is open. Otherwise, the supplied
	 * name will be checked against the name of the open log file as well.
	 * \return true if the log file is open, false if it isn't \see closeLogFile
	 * createLogFile openLogFile setLogMode */
	bool isLogFileOpen(const wchar_t* filename) const;
	/** \brief Open a log file for input. \details This function opens the
	 * supplied log file for reading. The function will fail if a serial
	 * connection is currently open. \param filename The name of the file to
	 * open. It is recommended to use a fully qualified path+filename. \note
	 * This function is only available in configuration mode. \see closeLogFile
	 * createLogFile isLogFileOpen setLogMode */
	XsensResultValue openLogFile(const char* filename);
	/** \brief Open a log file for input. \details This function opens the
	 * supplied log file for reading. The function will fail if a serial
	 * connection is currently open. \param filename The name of the file to
	 * open. It is recommended to use a fully qualified path+filename. \note
	 * This function is only available in configuration mode. \see closeLogFile
	 * createLogFile isLogFileOpen setLogMode */
	XsensResultValue openLogFile(const wchar_t* filename);
	/** \brief Set whether to read from comm port or file. \details While it is
	 * not possible to open a port or file while the other is open, it is
	 * possible to create a log file while a port is open. In some rare cases it
	 * may be required to log data and read it back while the port remains open
	 * and then continue logging in the same file. This is not recommended.
	 * Instead, log to a file, close it and then open it in another Cmt3 object.
	 * However, if you must do this, then this function can be used to specify
	 * that you want to switch between reading from the log file (true) or port
	 * (false). */
	XsensResultValue setDataSource(bool readFromFile);
	/** \brief Set the logging mode. \details This function sets the logging
	 * mode to enabled (true) or disabled (false). When enabled, all received
	 * messages are logged to the file. \remarks Some information that is cached
	 * on the host PC will generate faked messages in the log file. Eg.
	 * getConfiguration and some others. */
	XsensResultValue setLogMode(bool active);
	/** \brief Restart reading from the start of the open log file. \details
	 * This function resets the read position to the start of the open log file.
	 * Only the read position is affected, the write position remains the same.
	 * \see openLogFile */
	XsensResultValue resetLogFileReadPos(void);
	XsensResultValue writeMessageToLogFile(
		const Message& msg);  //! \brief Write the specified message to the open
	//! log file. \details This function can be used
	//! to add custom messages to a log file. \note A
	//! typical log file will only contain acknowledge
	//! messages, not request messages. \see
	//! createLogFile
	/** \brief Retrieve a list of the available scenarios. \details Use this
	 * function to retrieve a list of all scenarios available in a Motion
	 * Tracker. See the supplied documentation for more information about
	 * scenarios. \param scenarios A buffer for storing the available scenarios.
	 * The supplied buffer should be able to contain at least 6 scenarios, but
	 * for future devices a larger buffer is advised. Use
	 * CMT_MAX_SCENARIOS_IN_MT+1. The list is terminated by a scenario that has
	 * type 0. \remarks The type of any of the returned scenarios can be safely
	 * supplied to the setScenario function. \see getScenario setScenario \note
	 * This function is only available in configuration mode. */
	XsensResultValue getAvailableScenarios(
		CmtScenario* scenarios, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Get the currently active scenario from a Motion Tracker. \details
	 * This function retrieves the scenario currently used by the specified
	 * device when outputting orientation and/or position data. \see
	 * getAvailableScenarios setScenario \note This function is only available
	 * in configuration mode. */
	XsensResultValue getScenario(
		uint8_t& scenarioType, uint8_t& scenarioVersion,
		const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Specify the scenario to use in the sensor. \details This function
	 * specifies the scenario that the sensor should use for converting raw data
	 * into orientation and/or position data. This must be one of the scenarios
	 * in the list supplied by getAvailableScenarios. \see getAvailableScenarios
	 * getScenario \note This function is only available in configuration mode.
	 */
	XsensResultValue setScenario(
		const uint8_t scenarioType,
		const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Retrieve the currently used magnitude of the gravity vector.
	 * \details The magnitude of the gravity vector is used to determine
	 * absolute acceleration from measured acceleration. \see
	 * setGravityMagnitude \note This function is only available in
	 * configuration mode. */
	XsensResultValue getGravityMagnitude(
		double& magnitude, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the currently used magnitude of the gravity vector. \details
	 * The magnitude of the gravity vector is used to determine absolute
	 * acceleration from measured acceleration. Use this function to set tit to
	 * a custom value in m/s2. The default value of 9.812687357684514m/s2 should
	 * be fine for most places on earth, but in some cases the gravity may be
	 * drastically different from standard Earth gravity (eg. space, deep
	 * subterranean/submarine, polar regions, gravitational anomalies. See also
	 * <A
	 * HREF="http://www.abc.net.au/science/news/stories/s911917.htm">http://www.abc.net.au/science/news/stories/s911917.htm</A>
	 * and <A
	 * HREF="http://en.wikipedia.org/wiki/Earth%27s_gravity#Comparative_gravities_in_various_cities_around_the_world">http://en.wikipedia.org/wiki/Earth%27s_gravity#Comparative_gravities_in_various_cities_around_the_world</A>)
	 * \note Correct orientation output cannot be guaranteed when an incorrect
	 * value is supplied for the gravity. Changing this value is at your own
	 * risk. \see getGravityMagnitude \remarks In future versions, the MTi-G may
	 * be able to estimate the local gravity magnitude on its own. \note This
	 * function is only available in configuration mode. */
	XsensResultValue setGravityMagnitude(
		const double magnitude, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Get the currently used GPS lever arm. \details Use this function
	 * to retrieve the vector currently used as the GPS lever arm in meters. The
	 * GPS lever arm is the relative position of the GPS antenna to the MTi-G
	 * unit. The arm is specified in the object coordinate system. See the
	 * manual for more information on coordinate systems, alignment resets and
	 * the lever arm. \see setGpsLeverArm \note This function is only available
	 * in configuration mode. \note This function is only available for MTi-G
	 * devices. */
	XsensResultValue getGpsLeverArm(
		CmtVector& arm, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Request the status of the GPS satellites in orbit. \details This
	 * function requests the GPS satellite status information from the GPS
	 * subsystem. In config mode, this information is requested from the GPS
	 * subsystem immediately, which can cause a relatively long delay (250ms)
	 * before a reply is received. In measurement mode, the satellite status is
	 * regularly polled internally and the latest status is returned immediately
	 * when this function is called. */
	XsensResultValue getGpsStatus(
		CmtGpsStatus& status, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Set the currently used GPS lever arm. \details Use this function
	 * to set the vector currently used as the GPS lever arm. The GPS lever arm
	 * is the relative position of the GPS antenna to the MTi-G unit. The arm is
	 * specified in the object coordinate system and in meters. See the manual
	 * for more information on coordinate systems, alignment resets and the
	 * lever arm. \see getGpsLeverArm \note This function is only available in
	 * configuration mode. \note This function is only available for MTi-G
	 * devices. */
	XsensResultValue setGpsLeverArm(
		const CmtVector& arm, const CmtDeviceId deviceId = CMT_DID_MASTER);
	/** \brief Store important components of the XKF filter state to
	 * non-volatile memory. \details This function allows you to store some
	 * critical components of the internal XKF filter state to non-volatile
	 * memory. The stored settings will be used to initialize the filter
	 * whenever the sensor is switched to Measurement mode. \note This function
	 * is only available in Config mode. */
	XsensResultValue storeXkfState(const CmtDeviceId deviceId = CMT_DID_MASTER);
};

}  // namespace xsens

#endif  // _CMT3_H_2006_04_14
