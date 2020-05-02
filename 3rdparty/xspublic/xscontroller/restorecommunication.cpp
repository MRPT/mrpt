
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

#include "restorecommunication.h"
#include "xscontrol_def.h"
#include "serialinterface.h"
#include <xstypes/xsportinfo.h>


/*! \class RestoreCommunication
	\brief Performs restore communication procedure with a provided COM port.
	\details When started it will repeatedly send the reset byte (0xDE) at 115k2 until a specific answer is received.
	\note The procedure is executed on a separate thread and call a callback on completion/error.
*/

/*! \brief Constructor.
	\details Uses XsControl object to call a callback and creates a serial interface.
	\param control A pointer to XsControl object.
*/
RestoreCommunication::RestoreCommunication(XsControl* control)
	: m_control(control)
	, m_serialInterface(new SerialInterface())
	, m_isRestoring(false)
{
}

/*! \brief Destructor.
	\details Stops restoring the communication before destruction.
*/
RestoreCommunication::~RestoreCommunication()
{
	stop();
}

/*! \brief Starts the restore communication procedure.
	\details Tries to open COM port and starts a thread for the execution.
	\param portName A string with a port name.
	\returns XRV_OK if the procedure started successfully
*/
XsResultValue RestoreCommunication::start(const XsString &portName)
{
	m_portName = portName;
	XsPortInfo portInfo(portName, XBR_115k2);
	XsResultValue result;
	if (!m_isRestoring)
	{
		result = openComPort(portInfo);
		if (result == XRV_OK)
		{
			if (!startThread("RestoreCommunication"))
				result = XRV_ERROR;
			else
				m_isRestoring = true;
		}
	}
	else
	{
		result = XRV_ERROR;
		stop();
	}
	return result;
}

/*! \brief Stops the restore communication procedure.
	\details Stops the thread and closes a serial interface.
*/
void RestoreCommunication::stop()
{
	m_isRestoring = false;
	m_serialInterface->close();
}

/*! \brief Inner function for an execution on a thread.
	\returns 0.
*/
int32_t RestoreCommunication::innerFunction(void)
{
	m_serialInterface->setTimeout(0);

	bool success = false;
	const size_t readDataSize = 4;
	unsigned char resetByte = (unsigned char)0xDE; // reset byte
	unsigned char preambleBuf[1] = { 0 };
	unsigned char readBuf[readDataSize] = { 0, 0, 0, 0 };


	XsByteArray resetByteArray(1, &resetByte);
	XsByteArray preambleArray(1, &preambleBuf[0]);
	XsByteArray readArray(readDataSize, &readBuf[0]);

	while (!success && m_isRestoring)
	{
		XsResultValue writeResult = m_serialInterface->writeData(resetByteArray);
		XsResultValue readResult = m_serialInterface->readData(1, preambleArray);

		if (writeResult == XRV_OK && readResult == XRV_OK)
		{
			if (preambleArray[0] == XS_PREAMBLE)
				success = true;
		}

		XsTime::udelay(100);
	}

	bool stoppedByUser = !success;

	if (!stoppedByUser)
	{
		m_serialInterface->setTimeout(1000);
		m_serialInterface->readData(readDataSize, readArray);
		bool restoreOk = (preambleArray[0] == XS_PREAMBLE && readArray[0] == 0xFF && readArray[1] == 0x3E && readArray[2] == 0x00 && readArray[3] == 0xC3);

		XsResultValue result = restoreOk ? XRV_OK : XRV_RESTORE_COMMUNICATION_FAILED;
		m_control->onRestoreCommunication(&m_portName, result);
	}
	else
	{
		m_control->onRestoreCommunication(&m_portName, XRV_RESTORE_COMMUNICATION_STOPPED);
	}

	m_isRestoring = false;
	stopThread();
	return 0;
}

/*! \brief Tries to open COM port.
	\details If the port was open it will retry.
	\param portInfo A port info to open.
	\return XRV_OK if was successful.
*/
XsResultValue RestoreCommunication::openComPort(const XsPortInfo &portInfo)
{
	XsResultValue result = m_serialInterface->open(portInfo);
	if (result == XRV_ALREADYOPEN)
	{
		result = m_serialInterface->close();
		if (result == XRV_OK)
			result = m_serialInterface->open(portInfo);
	}
	return result;
}
