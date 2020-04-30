
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSDEVICECONFIGURATION_H
#define XSDEVICECONFIGURATION_H

#include "xscontrollerconfig.h"
#include <xstypes/pstdint.h>
#include <xstypes/xsbusid.h>
#include <xstypes/xstypedefs.h>
#ifdef __cplusplus
#include <xstypes/xsexception.h>
#endif

struct MtwInfo;
struct XsDeviceConfiguration;
struct XsMessage;
struct XsDeviceId;

#ifdef __cplusplus
extern "C" {
#else
#define XSDEVICEINFO_INITIALIZER = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define XSDEVICECONFIGURATION_INITIALIZER = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

XDA_DLL_API void XsDeviceConfiguration_construct(struct XsDeviceConfiguration* thisPtr);
XDA_DLL_API void XsDeviceConfiguration_assign(struct XsDeviceConfiguration* thisPtr, XsSize numberOfDevices, const struct XsDeviceConfiguration* src);
XDA_DLL_API void XsDeviceConfiguration_destruct(struct XsDeviceConfiguration* thisPtr);
XDA_DLL_API void XsDeviceConfiguration_copy(struct XsDeviceConfiguration* copy, struct XsDeviceConfiguration const* src);
XDA_DLL_API int  XsDeviceConfiguration_empty(const struct XsDeviceConfiguration* thisPtr);
XDA_DLL_API void XsDeviceConfiguration_readFromMessage(struct XsDeviceConfiguration* thisPtr, const struct XsMessage* msg);
XDA_DLL_API void XsDeviceConfiguration_writeToMessage(const struct XsDeviceConfiguration* thisPtr, struct XsMessage* msg);
XDA_DLL_API XsSize XsDeviceConfiguration_findDevice(const struct XsDeviceConfiguration* thisPtr, const struct XsDeviceId* deviceId);

#ifdef __cplusplus
} // extern "C"
#endif

/*! \brief %Device information for MT devices in an XsDeviceConfiguration. */
struct XsMtDeviceConfiguration {
	uint64_t	m_deviceId;				/*!< \brief This device ID */
	uint8_t		m_reserved[8];			/*!< \brief Reserved space */
	uint16_t	m_filterProfile;		/*!< \brief The currently chosen filter profile */
	uint8_t		m_fwRevMajor;			/*!< \brief The major version of the firmware */
	uint8_t		m_fwRevMinor;			/*!< \brief The minor version of the firmware */
	uint8_t		m_fwRevRevision;		/*!< \brief The revision version of the firmware */
	char		m_filterType;			/*!< \brief The filter type */
	uint8_t		m_filterMajor;			/*!< \brief The filter major version */
	uint8_t		m_filterMinor;			/*!< \brief The filter minor version */
};

typedef struct XsMtDeviceConfiguration XsMtDeviceConfiguration;

/*! \brief Device information for the main device in an XsDeviceConfiguration. */
struct XsMasterDeviceConfiguration {
	uint64_t		m_masterDeviceId;	/*!< \brief The master device ID */
	uint16_t		m_samplingPeriod;	/*!< \brief The sampling period */
	uint16_t		m_outputSkipFactor;	/*!< \brief The output skip factor */
	uint8_t			m_reserved1[8];		/*!< \brief Reserved space */
	uint8_t			m_date[8];			/*!< \brief The date */
	uint8_t			m_time[8];			/*!< \brief The time */
	uint8_t			m_productCode[20];	/*!< \brief The master product code */
	uint8_t			m_reserved2[44];	/*!< \brief Reserved space */
};
typedef struct XsMasterDeviceConfiguration XsMasterDeviceConfiguration;

#ifdef __cplusplus
/*! \class XsDeviceConfigurationException
	Exception class thrown when an exception occured inside the XsDeviceConfiguration
*/
class XsDeviceConfigurationException : public XsException {
public:
	XsDeviceConfigurationException() : XsException("Invalid device configuration") {}
};
#endif

/*! \brief Structure containing a full device configuration as returned by the ReqConfig message. */
struct XsDeviceConfiguration {
#ifdef __cplusplus
	/*! \brief Constructor

		\param numberOfDevs : The number of devices for which memory should be allocated in the XsDeviceConfiguration

		\sa XsDeviceConfiguration_construct
	*/
	explicit XsDeviceConfiguration(uint16_t numberOfDevs = 0)
		: m_numberOfDevices(0)
		, m_deviceInfo(0)
	{
		memset(this, 0, sizeof(XsDeviceConfiguration));
		if (numberOfDevs)
			XsDeviceConfiguration_assign(this, numberOfDevs, 0);
	}

	/*! \brief Copy constructor
		\param other the object to copy
		\sa XsDeviceConfiguration_copy
		*/
	XsDeviceConfiguration(const XsDeviceConfiguration& other)
		: m_numberOfDevices(0)
		, m_deviceInfo(0)
	{
		memset(this, 0, sizeof(XsDeviceConfiguration));
		XsDeviceConfiguration_copy(this, &other);
	}

	/*! \brief Assign \a other to this device configuaration

		\param other the object to copy

		\returns a const reference to this object
		\sa XsDeviceConfiguration_copy
		*/
	inline const XsDeviceConfiguration& operator = (const XsDeviceConfiguration& other)
	{
		if (this != &other)
			XsDeviceConfiguration_copy(this, &other);
		return *this;
	}

	/*! \brief Destructor \sa XsDeviceConfiguration_destruct */
	inline ~XsDeviceConfiguration()
	{
		XsDeviceConfiguration_destruct(this);
	}

	/*! \brief Clears and frees data \sa XsDeviceConfiguration_destruct */
	inline void clear()
	{
		XsDeviceConfiguration_destruct(this);
	}

	/*! \brief Test if this object is empty
		\returns true if this object is empty
	*/
	inline bool empty() const
	{
		return m_numberOfDevices == 0;
	}

	/*! \brief \copybrief XsDeviceConfiguration_readFromMessage

		\param msg the message to read the device configuration from

		\sa XsDeviceConfiguration_readFromMessage
		*/
	inline void readFromMessage(const XsMessage &msg)
	{
		XsDeviceConfiguration_readFromMessage(this, &msg);
	}

	/*! \brief \copybrief XsDeviceConfiguration_writeToMessage

		\param msg the message to write the device configuration to

		\sa XsDeviceConfiguration_writeToMessage
		*/
	inline void writeToMessage(XsMessage& msg) const
	{
		XsDeviceConfiguration_writeToMessage(this, &msg);
	}

	/*! \brief Return true if this contains device info for \a deviceId

		\param deviceId the device ID to find in this configuration
		\returns true if the device is present in the configuration
	*/
	inline bool containsDevice(const XsDeviceId& deviceId)
	{
		return XsDeviceConfiguration_findDevice(this, &deviceId) != 0;
	}

	/*! \brief The deviceInfo for the \a deviceId
		\param deviceId the device ID to identify with
		\returns the deviceInfo structure for \a deviceId
	*/
	inline XsMtDeviceConfiguration const& deviceInfo(const XsDeviceId& deviceId) const
	{
		XsSize busId = XsDeviceConfiguration_findDevice(this, &deviceId);
		if (busId == 0)
			throw XsDeviceConfigurationException();

		return deviceInfo(busId);
	}

	/*! \brief The deviceInfo for the \a deviceId
		\param deviceId the device ID to identify with
		\returns the deviceInfo structure for \a deviceId
	*/
	inline XsMtDeviceConfiguration& deviceInfo(const XsDeviceId& deviceId)
	{
		XsSize busId = XsDeviceConfiguration_findDevice(this, &deviceId);
		if (busId == 0)
			throw XsDeviceConfigurationException();

		return deviceInfo(busId);
	}

	/*! \brief The device info for the device at \a busId

		\param busId the bus ID of the device for which to return data for

		\returns a reference to the device configuration for the device at \a busId
	*/
	inline XsMtDeviceConfiguration& deviceInfo(XsSize busId)
	{
		if (!m_numberOfDevices)
			throw XsDeviceConfigurationException();

		if (busId == XS_BID_MASTER)
			return m_deviceInfo[0];

		if (busId > m_numberOfDevices)
			throw XsDeviceConfigurationException();

		return m_deviceInfo[busId-1];
	}

	/*! \brief The device info for the device at \a busId

		\param busId the bus ID of the device for which to return data for

		\returns a const reference to the device configuration for the device at \a busId
	*/
	inline const XsMtDeviceConfiguration& deviceInfo(XsSize busId) const
	{
		if (!m_numberOfDevices)
			throw XsDeviceConfigurationException();

		if (busId == XS_BID_MASTER)
			return m_deviceInfo[0];

		if (busId > m_numberOfDevices)
			throw XsDeviceConfigurationException();

		return m_deviceInfo[busId-1];
	}


	/*! \brief The device info for the master device

		\returns a reference to the device configuration for the master device
	*/
	inline XsMasterDeviceConfiguration& masterInfo()
	{
		return m_masterInfo;
	}

	/*! \brief The device info for the master device

		\returns a const reference to the device configuration for the master device
	*/
	inline const XsMasterDeviceConfiguration& masterInfo() const
	{
		return m_masterInfo;
	}

	/*! \brief Set the number of devices to \a count
		\param count the new number of devices to allocate for
		*/
	inline void setNumberOfDevices(XsSize count)
	{
		XsDeviceConfiguration_assign(this, count, 0);
	}

	/*! \brief The current number of devices
		\returns the number of devices
		*/
	inline XsSize numberOfDevices() const
	{
		return (XsSize) m_numberOfDevices;
	}

	/*! \brief \copybrief numberOfDevices
		\copydetails numberOfDevices
	*/
	inline XsSize deviceCount() const
	{
		return numberOfDevices();
	}

private:
//! \protectedsection
#endif
	XsMasterDeviceConfiguration m_masterInfo;	//!< \brief The master info
	const uint16_t	m_numberOfDevices;		//!< \brief The currently allocated number of devices
	XsMtDeviceConfiguration* const m_deviceInfo;	//!< \brief The list of device infos
};

typedef struct XsDeviceConfiguration XsDeviceConfiguration;

#endif
