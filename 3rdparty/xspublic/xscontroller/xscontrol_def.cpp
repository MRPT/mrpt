
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

#include "xscontrol_def.h"
#include "xdacommunicatorfactory.h"
#include <xstypes/xsportinfo.h>
#include <xstypes/xsportinfoarray.h>
#include "xsdeviceconfiguration.h"
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xsdatapacket.h>

#include "proxycommunicator.h"
#include "restorecommunication.h"

#include <xstypes/xsversion.h>

#include "mtix0device.h"
#include "mtix00device.h"
#include "mtigdevice.h"
#include <xstypes/xsdid.h>			// xddid and xsens_private for imar device id checks
#include "xsdeviceptrarray.h"
#include <xstypes/xssyncsetting.h>
#include <xstypes/xsmath.h>
#include <xstypes/xsdeviceidarray.h>
#include <set>
#include <memory>
#include <xstypes/xssyncsettingarray.h>
#include "broadcastdevice.h"
#include "idfetchhelpers.h"

using namespace xsens;
using namespace XsMath;

/*!
	\class XsControl
	\brief High level Motion Tracker (MT) management class

	CMT version 2 and higher do not use the explicit error codes that CMT version 1 used. Most functions
	return a boolean indicating success, a null-object, empty lists or nothing at all. In some cases more
	in-depth knowledge of the last error is required. For these occasions, use lastResult() or lastResultText()
	to find out what exactly went wrong.

	\note This object cannot be copied. The copy constructor has been disabled.
	\sa cinterface For the C interface functions.
*/

/*! \fn XsControl::clearCallbackHandlers(bool chain = true)
	\brief Clear the callback handler list
	\param chain Whether to clear the callback handlers of all connected devices as well (true, default)
	or just the callback handlers of the %XsControl object (false)
*/

/*! \fn XsControl::addCallbackHandler(XsCallbackPlainC* cb, bool chain = true)
	\brief Add a callback handler to the list
	\param cb The handler to add to the list.
	\param chain When set to true (default) the callback is added to connected devices as well
	\note NULL and duplicate handlers are ignored, but chaining is still done.
*/

/*! \fn XsControl::removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true)
	\brief Remove a handler from the list
	\param cb The handler to remove from the list.
	\param chain When set to true (default) the callback is added to connected devices as well
	\note If \a cb is not found in the list or if \a cb is NULL, the list is not changed, but
			chaining is still done.
*/

/*! \fn XsControl::broadcast() const
	\brief Returns the broadcast device
	\details The broadcast device can be used to apply an operation to all connected devices at once (if
	they support it)
	\returns An XsDevice pointer representing the broadcast device
*/

/*! \brief Construct a new Xsens Device API control object.
	\details Construct a new Xsens Device API control object that can be used to open ports, files, etc.
	\return The newly constructed XsControl object
*/
XsControl::XsControl()
	: m_useFakeMessages(true)
	, m_synchronousDataReport(false)
	, m_lastHwError(XRV_OK)
	, m_lastHwErrorDeviceId(0)
	, m_recording(false)
	, m_broadcaster(0)
	, m_optionsEnable(XSO_Calibrate | XSO_Orientation)
	, m_optionsDisable(XSO_None)
	, m_latLonAlt(XsVector())
	, m_deviceFactory(new DeviceFactory)
	, m_communicatorFactory(new XdaCommunicatorFactory)
	, m_restoreCommunication(nullptr)
{
	m_communicatorFactory->registerCommunicatorTypes();
	m_deviceFactory->registerDevices();

	m_broadcaster = new BroadcastDevice(this);
	m_restoreCommunication = new RestoreCommunication(this);
}


/*! \brief Destroy this XsControl object.
	\details All connected devices are put in config mode. All serial ports and files are subsequently closed.
	\sa close()
*/
XsControl::~XsControl()
{
	try {
		close();
		delete m_broadcaster;
		delete m_restoreCommunication;
	} catch(...)
	{}

	delete m_deviceFactory;
	delete m_communicatorFactory;
	m_broadcaster = nullptr;
	m_restoreCommunication = nullptr;
}

/*! \brief Get a descriptive text for the given \a resultCode.
	\param resultCode The result code to translate
	\returns The \a resultCode translated into an %XsString
*/
XsString XsControl::resultText(XsResultValue resultCode)
{
	return XsResultValue_toString(resultCode);
}

/*! \brief Close all ports and files.
	\details All devices are put in config mode before the serial port is closed.
*/
void XsControl::close()
{
	JLDEBUGG("");
	XSEXITLOGD(gJournal);

	if (!m_broadcaster->isReadingFromFile())
	{
		m_broadcaster->gotoConfig();
	}

	std::vector<XsDevice*> localList = m_deviceList;
	for (std::vector<XsDevice*>::iterator it = localList.begin(); it != localList.end(); ++it)
	{
		removeChainedManager(*it);

		// prepareForTermination is called automatically when the reference is removed
		(*it)->prepareForTermination();
		(*it)->removeRef();
	}
	m_deviceList.clear();

	m_lastHwError = XRV_OK;
	m_lastHwErrorDeviceId = 0;

	m_lastResult = XRV_OK;
}

#ifndef XSENS_NO_PORT_NUMBERS
/*! \brief Close the serial port with the given \a portNr.
	\details All connected devices are put in config mode before the port is closed.
	\param portNr The COM port number of the port that should be closed
	\note This function is only available on Windows systems
*/
void XsControl::closePort(int portNr)
{
	JLDEBUGG(portNr);
	XSEXITLOGD(gJournal);

	closePort(XsPortInfo(portNr).portName());
}
#endif

/*! \brief Close the device port with the given XsDevice
	\details All connected devices are put in config mode before the port is closed.
	\param device the XsDevice obtained with the device() function
	\sa device()
*/
void XsControl::closePort(XsDevice *device)
{
	JLDEBUGG(device);
	XSEXITLOGD(gJournal);

	for (uint16_t i = 0;i < m_deviceList.size();++i)
	{
		if (device == m_deviceList[i])
		{
			m_deviceList.erase(m_deviceList.begin() + i);
			removeChainedManager(device);
			device->prepareForTermination();
			device->removeRef();
		}
	}
}

/*! \brief Close the serial port with the given \a portname.
	\details All connected devices are put in config mode before the port is closed.
	\param portname the name of the port to close (e.g. COM1 on Windows, /dev/ttyUSB0 on Linux)
*/
void XsControl::closePort(const XsString& portname)
{
	JLDEBUGG(portname.toStdString());
	XSEXITLOGD(gJournal);

	// find appropriate port
	LockReadWrite portLock(&m_portMutex);
	portLock.lock(true);

	for (uint16_t i = 0;i < m_deviceList.size();++i)
	{
		if (portname == m_deviceList[i]->portName())
			closePort(m_deviceList[i]);
	}
}

/*! \brief Close the port that is used for communication with the given \a deviceId
	\param deviceId The device ID to clos eth port for. When 0, the first available port is closed.
	\note When the port hosts multiple devices, this function will make all devices connected to the port
	invalid.
*/
void XsControl::closePort(const XsDeviceId& deviceId)
{
	XsDevice* inf = findDevice(deviceId);
	if (inf)
		closePort(inf);
}

/*! \brief Close the serial port that matches \a portinfo.
	\details All connected devices are put in config mode before the port is closed.
	\param portinfo A port information structure that contains the name of the port to close
	\sa closePort(const XsString&)
*/
void XsControl::closePort(const XsPortInfo& portinfo)
{
	closePort(portinfo.portName());
}

/*! \brief Clear the inbound data buffers of all devices
	\details
	\copydetails XsDevice::flushInputBuffers
*/
void XsControl::flushInputBuffers()
{
	JLDEBUGG("");
	XSEXITLOGD(gJournal);

	for (uint32_t i = 0; i < m_deviceList.size(); i++) // loop over all devices,
	{
		XsDevice *dev = m_deviceList[i];
		dev->flushInputBuffers(); // flush data buffer of each device.
	}
}

/*! \brief Get the number of connected devices.
	\returns The number of connected devices
*/
int XsControl::deviceCount() const
{
	int count = 0;
	for (uint32_t i = 0; i < m_deviceList.size(); i++)
	{
		XsDevice *dev = m_deviceList[i];
		count += 1 + (int) dev->childCount();
	}
	m_lastResult = XRV_OK;
	return count;
}

/*!
	\brief Get the device IDs of all the connected devices.
	\returns Vector containing the device IDs of all the connected devices.
*/
std::vector<XsDeviceId> XsControl::deviceIds() const
{
	LockReadWrite portLock(&m_portMutex, LS_Read);

	std::vector<XsDeviceId> result;
	for (uint32_t index = 0; index < m_deviceList.size(); ++index)
	{
		XsDevice const* dev = m_deviceList[index];
		result.push_back(XsDeviceId(dev->deviceId()));
		auto childrn = dev->children();
		for (auto child : childrn)
			if (child)
				result.push_back(XsDeviceId(child->deviceId()));
	}
	return result;
}

/*!
	\brief Get the device of the device on the given \a locationId.

	If the location ID is not found, the lastResult value is set and the
	function returns a nullptr.

	\param locationId the location ID of the device we're looking for

	\returns a pointer to the device for \a locationId

	\sa lastResult()
*/
XsDevice* XsControl::getDeviceFromLocationId(uint16_t locationId) const
{
	JLDEBUGG((int) locationId);
	XSEXITLOGD(gJournal);
	for (uint16_t i = 0; i < m_deviceList.size(); ++i) {
		XsDevice *d = m_deviceList[i]->getDeviceFromLocationId(locationId);
		if (!d)
			continue;
		m_lastResult = XRV_OK;
		return d;
	}
	m_lastResult = XRV_NOTFOUND;
	return nullptr;
}

/*!	\brief Clear the last hardware error.
	\sa lastHardwareError();
*/
void XsControl::clearHardwareError()
{
	m_lastHwErrorDeviceId = 0;
	m_lastHwError = XRV_OK;
}

/*!	\brief Get the last hardware error code
	\returns The last hardware error
	\sa lastResult()
	\sa resultText()
*/
XsResultValue XsControl::lastHardwareError() const
{
	return m_lastHwError;
}

/*! \returns The device ID that caused the last hardware error.
*/
XsDeviceId XsControl::lastHardwareErrorDeviceId() const
{
	return m_lastHwErrorDeviceId;
}

/*!	\brief Get the result value of the last operation.
	\details The result values are codes that describe a failure in more detail.
	\returns the last known error code
	\sa resultText(XsResultValue), lastResultText()
*/
XsResultValue XsControl::lastResult() const
{
	return m_lastResult.lastResult();
}

/*!	\brief Get the accompanying error text for the value returned by lastResult()
	\details This is more than a convenience function for
	\code
		XsString lastResultText = XsControl::resultText(xscontrol->lastResult());
	\endcode
	It may provide situation-specific information instead.
	\returns a human readable error description
	\sa resultText(XsResultValue), lastResult()
*/
XsString XsControl::lastResultText() const
{
	return m_lastResult.lastResultText();
}

/*!	\brief Get the number of main devices.
	\returns the number of main devices
	\sa mainDeviceIds(), mainDeviceId(const XsDeviceId&)
*/
int XsControl::mainDeviceCount() const
{
//	m_lastResult = XRV_OK;
	return (int) m_deviceList.size();
}

/*!
	\brief Get the number of connected MTs.

	\returns the number of MTs, including both main and child devices.

	\sa mtDeviceIds()
*/
/* We assume motion trackers cannot have children and if a device is a child device, it is a motion tracker */
int XsControl::mtCount() const
{
	int count = 0;
	for (uint32_t i = 0; i < m_deviceList.size(); i++)
	{
		XsDevice *dev = m_deviceList[i];
		if (dev->isMotionTracker())
			count++;
	}
	m_lastResult = XRV_OK;
	return count;
}

/*!
	\brief Get the device IDs of the available main devices.

	Main devices are the devices communicating with the serial port, typically Bodypacks,
	Awinda Stations and stand-alone MTis or MTxs.
	\returns a std::vector with the device IDs.
*/
std::vector<XsDeviceId> XsControl::mainDeviceIds() const
{
	LockReadWrite portLock(&m_portMutex);
	portLock.lock(false);

	m_lastResult = XRV_OK;
	std::vector<XsDeviceId> ids;
	ids.reserve(m_deviceList.size());
	for (uint32_t index = 0; index < m_deviceList.size(); ++index)
		ids.push_back(m_deviceList[index]->deviceId());
	return ids;
}

/*!
	\brief Get the device IDs of the available MTs.
	\returns A std::vector with the device IDs.
	\sa mtCount
	\internal
*/
std::vector<XsDeviceId> XsControl::mtDeviceIds() const
{
	m_lastResult = XRV_OK;

	LockReadWrite portLock(&m_portMutex);
	portLock.lock(false);

	std::vector<XsDeviceId> result;
	for (uint32_t i = 0;i < m_deviceList.size();++i)
	{
		XsDevice const *main = m_deviceList.at(i);
		if (main->isMotionTracker())
			result.push_back(main->deviceId());
	}
	return result;
}

/*! \brief Place all sensors connected through a serial port into Configuration Mode.

  This function is called before close() in the destructor of the class.
  /sa close()

  \internal
  The function places the sensors in configuration mode in the appropriate order as they are sorted by sortBySync.
*/
void XsControl::gotoConfig(void)
{
	JLDEBUGG("");
	XSEXITLOGD(gJournal);

	m_broadcaster->gotoConfig();
}

/*!
	\brief Place all sensors connected through a serial port into Measurement Mode.

	The function places the sensors in measurement mode in the appropriate order
	as they are sorted by sortBySync.
*/
void XsControl::gotoMeasurement()
{
	JLDEBUGG("");
	XSEXITLOGD(gJournal);

	m_broadcaster->gotoMeasurement();
}

/*! \brief Starts restore communication procedure.
	\details Restores the communication settings to the default factory settings.
	\note Works with RS422 and legacy products only.
	\param portName the name of port to which device is connected.
	\returns XRV_OK if restore communication procedure was successfull.
*/
XsResultValue XsControl::startRestoreCommunication(const XsString & portName)
{
	return m_restoreCommunication->start(portName);
}

/*! \brief Stops restore communication procedure.
*/
void XsControl::stopRestoreCommunication()
{
	m_restoreCommunication->stop();
}

/*! \brief Test if the given \a deviceId is docked

	Only wireless devices can be regarded as docked.

	\param deviceId the ID of the device to investigate

	\returns true if the device is docked, false otherwise
*/
bool XsControl::isDeviceDocked(const XsDeviceId& deviceId) const
{
	(void)deviceId;
	return false;
}

/*!
	\brief Test if the given \a deviceId is an MTw and if it is wirelessly connected.
	\details If the device ID is not found, the function returns false and the lastResult value is set.

	\param deviceId the ID of the device to investigate

	\returns true if the device is wirelessly connected, false otherwise
*/
bool XsControl::isDeviceWireless(const XsDeviceId& deviceId) const
{
	(void)deviceId;
	return false;
}

/*! \cond XS_INTERNAL */
/*! \brief Creates and adds a device as master. The new master will be owned by this.
	\param[in] communicator the Communicator for the created device.
	\note communicator is a pointer to a dynamically allocated Communicator.
	The created device will take ownership of the communicator.
	\returns the newly created device.
*/
XsDevice* XsControl::addMasterDevice(Communicator* communicator)
{
	XsDevice* result = nullptr;
	XsDevice* dev = m_deviceFactory->createMasterDevice(communicator);
	if (dev != nullptr)
	{
		setPersistentSettings(dev);
		addChainedManager(dev);
		m_deviceList.push_back(dev);
		result = dev;
	}
	return result;
}
/*! \endcond */

/*!	\brief Open the log file with the given \a filename.
  \returns True is the file was opened successfully. False if an error was encountered.

	\param filename the name of the file to open

	\returns true on success, false on failure

  \sa lastResult(), loadLogFile(), logFileName()
*/
bool XsControl::openLogFile(const XsString& filename)
{
	JLDEBUGG(filename.toStdString());
	XSEXITLOGD(gJournal);

	LockReadWrite portLock(&m_portMutex);
	portLock.lock(true);

	// try opening the file as a regular mtb file
	auto object = Communicator::createUniquePtr(m_communicatorFactory->create(filename));

	copyCallbackHandlersTo(object.get());
	//object->addCallbackHandler(d);
	if (!object->openLogFile(filename))
	{
		m_lastResult = object->lastResult();
		return false;
	}

	// note that addMasterDevice ALWAYS takes ownership of communicator
	XsDevice* dev = addMasterDevice(object.release());
	if (!dev)
		return false;

	dev->resetLogFileReadPosition();	// this will and should call reinitializeProcessors();

	m_lastResult = XRV_OK;
	return true;

}

#ifndef XSENS_NO_PORT_NUMBERS
/*! \brief Open a communication channel to the given COM \a portNr.

	This is a convenience overload for openPort(const XsString&, XsBaudRate, bool).
	This function is available on Microsoft Windows only due to the ambiguous nature of port numbers on other platforms.

	\param baudrate The baudrate used on the port.
	\param portNr The port number.
	\param timeout The maximum number of ms to try to put the device in config mode before giving up, if 0 the default value is used
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices

	\returns true if the port was opened successfully, false otherwise

	\sa openPort(const XsString&, XsBaudRate, uint32_t, bool) \sa lastResult()
*/
bool XsControl::openPort(int portNr, XsBaudRate baudrate, uint32_t timeout, bool detectRs485)
{
	JLDEBUGG("port " << (int32_t) portNr << " baudrate " << baudrate << " timeout " << timeout << " detectRs485 " << (int32_t)detectRs485);
	XSEXITLOGD(gJournal);

	return openPort(XsPortInfo(portNr, baudrate), timeout, detectRs485);
}
#endif

/*!
	\brief Open a communication channel on serial port with the given \a portname.

	If opening the port is successful, the connected devices are available through the XsControl interface.

	The expected value for \a portname on Microsoft Windows platforms is "COMx" where x is the port number.

	\param baudrate The baudrate used on the port.
	\param portname The name of the port.
	\param timeout The maximum number of ms to try to put the device in config mode before giving up, if 0 the default value is used
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices

	\returns true on success, false otherwise

	\sa openPort(int, XsBaudRate, uint32_t, bool)
*/
bool XsControl::openPort(const XsString &portname, XsBaudRate baudrate, uint32_t timeout, bool detectRs485)
{
	XsPortInfo localPortInfo = XsPortInfo(portname, baudrate);
	uint16_t vid = vidFromString(portname.toStdString());
	uint16_t pid = pidFromString(portname.toStdString());
	localPortInfo.setVidPid(vid, pid);
	return openPort(localPortInfo, timeout, detectRs485);
}

/*! \cond XS_INTERNAL */
/*! \brief Finalize opening the port

  Takes ownership of the passed Communicator.
*/
bool XsControl::finalizeOpenPort(Communicator *communicator, XsPortInfo &portinfo, uint32_t timeout, bool detectRs485)
{
	XSEXITLOGD(gJournal);
	if (!communicator)
	{
		m_lastResult = XRV_INVALIDPARAM;
		return false;
	}

	auto serialPort = Communicator::createUniquePtr(communicator);

	copyCallbackHandlersTo(serialPort.get());

	if (timeout)
		serialPort->setGotoConfigTimeout(timeout);

	bool retval = serialPort->openPort(portinfo, OPS_Full, detectRs485);
	if (serialPort->masterDeviceId().isValid())
		portinfo.setDeviceId(serialPort->masterDeviceId());

	if (!retval)
	{
		m_lastResult.set(serialPort->lastResult(), serialPort->lastResultText());
		return false;
	}

	// note that addMasterDevice ALWAYS takes ownership of communicator
	if (!addMasterDevice(serialPort.release()))
		return false;

	return true;
}
/*! \endcond */

/*!	\brief Open a communication channel using the details in the supplied %XsPortInfo structure
	\param portinfo Contains the details of the port to open. The \a portinfo may be updated with a detected deviceid
	\param timeout The maximum number of ms to try to put the device in config mode before giving up, if 0 the default value is used
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices. Only necessary if \a portInfo does not contain a device ID of an RS485 device
	\returns true on success, false otherwise
	\sa openPort(const XsString &, XsBaudRate, uint32_t)
*/
bool XsControl::openPort(XsPortInfo& portinfo, uint32_t timeout, bool detectRs485)
{
	JLDEBUGG("port " << portinfo << " timeout " << timeout << " detectRs485 " << (int32_t)detectRs485);
	XSEXITLOGD(gJournal);

	Communicator *xs3info = findXbusInterface(portinfo);
	if (xs3info)
	{
		if (xs3info->masterDeviceId().isValid() && !portinfo.deviceId().isValid())
			portinfo.setDeviceId(xs3info->masterDeviceId());
		m_lastResult = XRV_ALREADYOPEN;
		return true;
	}

	return finalizeOpenPort(m_communicatorFactory->create(portinfo), portinfo, timeout, detectRs485);
}

/*!	\brief Open a communication channel using the details in the supplied %XsPortInfo structure using the supplied credentials
	\param portinfo Contains the details of the connection to open. The device ID in the structure may be updated by this function.
	\param id The user ID to be supplied for the authentication
	\param key The key to be supplied for the authentication
	\param timeout The maximum number of ms to try to put the device in config mode before giving up, if 0 the default value is used
	\returns true on success, false otherwise
	\sa openPort(const XsPortInfo &, uint32_t, bool)
	\sa openPort(const XsString &, XsBaudRate, uint32_t)
*/
bool XsControl::openPortWithCredentials(XsPortInfo& portinfo, XsString const& id, XsString const& key, uint32_t timeout)
{
	JLDEBUGG("port " << portinfo << " id " << id << " key " << key << " timeout " << timeout);
	XSEXITLOGD(gJournal);

	Communicator *xs3info = findXbusInterface(portinfo);
	if (xs3info)
	{
		if (xs3info->masterDeviceId().isValid())
			portinfo.setDeviceId(xs3info->masterDeviceId());
		m_lastResult = XRV_ALREADYOPEN;
		return true;
	}

	xs3info = m_communicatorFactory->create(portinfo);
	xs3info->setCredentials(id, key);
	return finalizeOpenPort(xs3info, portinfo, timeout, false);
}

/*! \brief Open a custom communication channel
	\param channelId: User-provided identifier for the custom channel. Supplying the same channel Id more than once gives a XV_ALREADYOPEN result (\sa getLastResult)
	\param channelLatency: The worst-case round-trip delay in milliseconds induced by the custom channel. XDA will add this latency to its communication timeout values
	\param detectRs485 Enable more extended scan to detect rs485 devices
	\returns true if the port was successfully opened.
	\sa closeCustomPort
	\sa transmissionReceived
*/
bool XsControl::openCustomPort(int channelId, uint32_t channelLatency, bool detectRs485)
{
	if (m_proxyChannels.find(channelId) != m_proxyChannels.end())
	{
		// We are assuming that the device is there in this case
		m_lastResult = XRV_ALREADYOPEN;
		return true;
	}

	ProxyCommunicator* proxy = new ProxyCommunicator(channelId, channelLatency);
	m_proxyChannels[channelId] = proxy;

	XsPortInfo portInfo = ProxyCommunicator::createPortInfo(channelId);
	if (!finalizeOpenPort(proxy, portInfo, 0, detectRs485))
	{
		//Please note that finalizeOpenPort takes ownership of the Communicator pointer and will clean it up on error
		m_proxyChannels.erase(channelId);
		return false;
	}

	return true;
}

/*! \brief Returns the port information for a custom communication channel
	\param channelId: The user-provided identifier of the channel
	\returns The port info for the given channel
	\sa openCustomPort
*/
XsPortInfo XsControl::customPortInfo(int channelId) const
{
	if (m_proxyChannels.find(channelId) == m_proxyChannels.end())
		return XsPortInfo();

	return m_proxyChannels.at(channelId)->portInfo();
}

/*! \brief Closes a custom communication channel
	\note When closing a custom channel the device will not be switched to config mode. This must be done manually \sa XsDevice::gotoConfig
	\param channelId: The user-provided identifier of the channel to close. This identifier must match with the one used to open the port
	\sa openCustomPort
*/
void XsControl::closeCustomPort(int channelId)
{
	if (m_proxyChannels.find(channelId) == m_proxyChannels.end())
		return;

	ProxyCommunicator* p = m_proxyChannels[channelId];
	XsDevice* dev = findDevice(p->masterDeviceId());
	dev->setGotoConfigOnClose(false);
	m_proxyChannels.erase(channelId);
	closePort(p->masterDeviceId());
}

/*! \brief Feed data coming back from an Xsens device over a custom channel into XDA
	\note For correct operation of XDA it is key that the transmissionReceived function is called from a separate thread
	\param channelId: The user-provided identifier of the custom channel
	\param data: The data to feed back into XDA
*/
void XsControl::transmissionReceived(int channelId, const XsByteArray& data)
{
	if (m_proxyChannels.find(channelId) == m_proxyChannels.end())
		return;
	m_proxyChannels[channelId]->handleReceivedData(data);
}

/*!
	\brief Open a communication channel on serial port with the given \a portname.

	If opening the port is successful, the connected devices are available through the XsControl interface.

	The expected value for \a portname on Microsoft Windows platforms is "COMx" where x is the port number.

	\param baudrate The baudrate used on the port.
	\param portname The name of the port.
	\param imarType The type of iMAR device that is used
	\param timeout The maximum number of ms to try to put the device in config mode before giving up, if 0 the default value is used

	\returns true on success, false otherwise

	\sa openPort(int, XsBaudRate, bool)
*/
bool XsControl::openImarPort_internal(const XsString &, XsBaudRate , int , uint32_t )
{
	return false;
}

/*!
	\brief Get the device ID of the dock device for the given \a deviceId.

	This function returns the ID of the docking station that deviceId is plugged into.
	If the docking station itself is not an open port in this XsControl or the device is not
	plugged into a docking station, the function will return a 0 id.

	\param deviceId the ID of the device to find the dock parent for

	\returns the ID of the device that has \a deviceId docked

	\sa isDeviceDocked
*/
XsDeviceId XsControl::dockDeviceId(const XsDeviceId& deviceId) const
{
	JLDEBUGG(deviceId.toString().toStdString());

	LockReadWrite portLock(&m_portMutex);
	portLock.lock(false);

	m_lastResult = XRV_OK;
	if (!deviceId.isMtw())
		return XsDeviceId();

	XsDevice *dev = findDevice(deviceId);
	if (!dev) {
		m_lastResult = XRV_INVALIDID;
		return XsDeviceId();
	}

	for (uint32_t index = 0; index < m_deviceList.size(); ++index)
	{
		if (m_deviceList[index]->deviceIsDocked(dev))
			return m_deviceList[index]->deviceId();
	}

	return XsDeviceId();
}

/*!	\brief Sets the current GNSS position of the system
	\details This function will update the Latitude, Longitude and Altitude of the system and all
	connected devices. This differs from broadcast()->setInitialPositionLLA() in that the setting is persistent
	for the XsControl and will be applied to devices connected after the setting has been made.
	Note: this XDA data type is the setting initialPositionLLA, which is set by setInitialPositionLLA.
	It's value is therefore static. Use LatitudeLongitude to retrieve the live position data from the MTi.
	\param lla A vector containing the desired Latitude, Longitude and Altitude
	\returns true since the function always succeeds, the boolean return value is for consistency in
	the interface.
*/
bool XsControl::setInitialPositionLLA(const XsVector& lla)
{
	m_latLonAlt = lla;
	m_broadcaster->setInitialPositionLLA(lla);
	return true;
}

/*! \brief Returns the XsDevice interface object associated with the supplied \a deviceId
	\param deviceId The ID of the device to return, 0 to return the first available main device
	\returns The XsDevice attached to the \a deviceId or 0 if the device was not found
	\sa broadcast()
*/
XsDevice* XsControl::device(const XsDeviceId& deviceId) const
{
	return findDevice(deviceId);
}

/*! \brief Returns all main XsDevice interface objects
	\returns A list containing pointers to the main device XsDevice objects
*/
XsDevicePtrArray XsControl::mainDevices() const
{
	XsDevicePtrArray rv;
	for (std::vector<XsDevice*>::const_iterator it = m_deviceList.begin(); it != m_deviceList.end(); ++it)
		rv.push_back(*it);
	return rv;
}

/*! \brief Returns the broadcast device
	\details The broadcast device can be used to apply an operation to all connected devices at once (if
	they support it)
	\returns An XsDevice pointer representing the broadcast device
*/
 XsDevice* XsControl::broadcast() const
{
	return m_broadcaster;
}

/*! \cond XS_INTERNAL */
/*! \brief Check if there are devices in a recording state and update m_recording accordingly
*/
void XsControl::updateRecordingState()
{
	for (size_t i = 0 ; i < m_deviceList.size(); ++i)
		if (m_deviceList[i]->isRecording())
			m_recording = true;

	m_recording = false;
}

/*! \brief Find the device info of the supplied \a deviceId and return a pointer to the item in the list

	\param deviceId the device ID to look for

	\returns the device matching \a deviceId, NULL otherwise. On failure lastResult() is set.
*/
XsDevice* XsControl::findDevice(const XsDeviceId& deviceId) const
{
	if (m_deviceList.empty()) {
		m_lastResult = XRV_NOFILEORPORTOPEN;
		return NULL;
	}

	m_lastResult = XRV_OK;

	if (deviceId.toInt() == 0)
		return m_deviceList[0];	// simply return the first item in the list

	for (size_t i = 0; i < m_deviceList.size(); i++)
	{
		XsDevice *dev = m_deviceList[i];

		if (dev == 0) {
			break;
		}

		if (dev->deviceId() == deviceId)
			return dev;

		dev = dev->findDevice(deviceId);
		if (dev)
			return dev;
	}

	m_lastResult = XRV_INVALIDID;
	return NULL;
}

/*! \brief Close an existing device. Because it was detected somewhere else.
	\param deviceId the device ID to look for
*/
void XsControl::removeExistingDevice(XsDeviceId const & deviceId)
{
	XsDevice *dev  = findDevice(deviceId);
	if (!dev)
		return;

	if (dev->isMasterDevice())
		closePort(deviceId);
}

/*! \brief Searches for the XBUS interface in a device object
	\param deviceId The ID of device object to search in
	\returns The found communication interface
*/
Communicator* XsControl::findXbusInterface(const XsDeviceId &deviceId) const
{
	XsDevice *dev = findDevice(deviceId);
	if (!dev)
		return NULL;

	return dev->communicator();
}

/*! \brief Searches for the XBUS interface in a port information object
	\param portInfo The port information object to search in
	\returns The found communication interface
*/
Communicator* XsControl::findXbusInterface(const XsPortInfo &portInfo) const
{
	return findXbusInterface(portInfo.portName());
}

/*! \brief Searches for the XBUS interface in a port
	\param portName The name of port to search in
	\returns The found communication interface
*/
Communicator* XsControl::findXbusInterface(const XsString &portName) const
{
	for (size_t i = 0; i < m_deviceList.size(); ++i)
	{
		assert(m_deviceList[i]->communicator());
		if (portName == m_deviceList[i]->communicator()->portInfo().portName())
			return m_deviceList[i]->communicator();
	}
	return NULL;
}
/*! \endcond */

/*!	\brief Load filter profile definitions from a settings file with the given \a filename

	To use the filtering properly, XDA requires filter settings usually stored in a
	scenarios.xsb file. XSB is the Xsens Settings Binary format. This function allows you to
	specify the full path + filename to use instead of the default filter profiles embedded in the dll.

	\param filename The full path+filename to use for loading the parameters. When empty, the default
					filter profiles are loaded. On Linux this is "./scenarios.xsb", on Windows it is read
					from the dll resource.

	\returns true on success, false on failure
*/
bool XsControl::loadFilterProfiles(const XsString& )
{
	return false;
}

/*! \cond XS_INTERNAL */
/*! \brief Close a serial communication port by \a index
	\param i The index of a device from a device list
*/
void XsControl::closePortByIndex(uint32_t i)
{
	JLDEBUGG(i);
//	XSEXITLOGD(gJournal);

	XsDevice *dev = m_deviceList[i];
	closePort(dev);
}
/*! \endcond */

/*! \brief Return the currently enabled options
	\return The options that are set to be enabled
	\sa setOptions \sa disabledOptions
*/
XsOption XsControl::enabledOptions() const
{
	return m_optionsEnable;
}

/*! \brief Return the currently explicitly disabled options
	\return The options that are set to be explicitly disabled
	\sa setOptions \sa enabledOptions
*/
XsOption XsControl::disabledOptions() const
{
	return m_optionsDisable;
}

/*! \brief Peristently enable or disable options
	\details These options are used to specify whether XDA should compute certain kinds of data from
	available other data and what data-retention policy to use. On a system with limited
	resources it may be useful to limit the processing and data retention done by XDA. By default XDA will
	do all processing it can do, but retain as little data as possible.
	This function remembers the setting and applies it to new devices when they are created as well
	as broadcasting it to existing devices.
	In case of conflict, \a enable supersedes \a disable.
	\param enable A logically OR'ed combination of XsOptions to enable
	\param disable A logically OR'ed combination of XsOptions to disable
	\note While XsDevice uses these options in an additional manner, remembering whatever was enabled/disabled before,
	calling the XsControl version replaces all its remembered enable/disable values.
	\sa setOptionsForce
*/
void XsControl::setOptions(XsOption enable, XsOption disable)
{
	m_optionsEnable = XsOption_purify(enable);
	m_optionsDisable = disable;
	m_broadcaster->setOptions(m_optionsEnable, m_optionsDisable);
}

/*! \brief Peristently enable or disable options
	\details These options are used to specify whether XDA should compute certain kinds of data from
	available other data and what data-retention policy to use. On a system with limited
	resources it may be useful to limit the processing and data retention done by XDA. By default XDA will
	do all processing it can do, but retain as little data as possible.
	This function remembers the setting and applies it to new devices when they are created as well
	as broadcasting it to existing devices.
	Any non-enabled options are explicitly disabled.
	\param enabled A logically OR'ed combination of the desired enabled XsOptions
	\note Contrary to \a setOptions this function will do a hard override of all options of the child devices as it will
	assume that any non-enabled option should be specifically disabled.
	\sa setOptions
*/
void XsControl::setOptionsForce(XsOption enabled)
{
	m_optionsEnable = XsOption_purify(enabled);
	m_optionsDisable = ((~m_optionsEnable) & XSO_All);
	m_broadcaster->setOptions(m_optionsEnable, m_optionsDisable);
}

/*! \cond XS_INTERNAL */
/*! \brief Write the persistent settings to a (newly created) device
	\details These settings include whether to perform filtering, explicit initialPositionLLA values, etc
	\param dev The device to write the settings to
*/
void XsControl::setPersistentSettings(XsDevice* dev)
{
	dev->setOptions(m_optionsEnable, m_optionsDisable);

	if (!m_latLonAlt.empty())
		dev->setInitialPositionLLA(m_latLonAlt);
}
/*! \endcond */

#ifndef XDA_PRIVATE_BUILD
#include "xscontrol_public.h"
#else
#include "xscontrolex.h"
#endif
XsControl* XsControl::construct()
{
	return new XsControlEx;
}
#include "xscontrol_def.h"
