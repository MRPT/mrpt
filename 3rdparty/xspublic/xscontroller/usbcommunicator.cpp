
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

#include "usbcommunicator.h"
#include "usbinterface.h"

/*! \class UsbCommunicator
	\brief A class that uses USB to communicate
*/

/*! \brief Construct and returns new USB communicator
	\returns The new USB communicator
*/
Communicator *UsbCommunicator::construct()
{
	return new UsbCommunicator;
}

/*! \brief Default constructor
*/
UsbCommunicator::UsbCommunicator()
	: m_usbInterface(nullptr)
{
}

/*! \brief Default destructor
*/
UsbCommunicator::~UsbCommunicator()
{
}

/*! \copybrief Communicator::gotoConfig
*/
XsResultValue UsbCommunicator::gotoConfig(bool detectRs485)
{
	XsResultValue r = SerialPortCommunicator::gotoConfig(detectRs485);

	if (r == XRV_OK)
	{
		m_usbInterface->setRawIo(false);
		m_usbInterface->setTimeout(0);
	}
	return r;
}

/*! \copybrief Communicator::gotoMeasurement
*/
XsResultValue UsbCommunicator::gotoMeasurement()
{
	XsResultValue r = SerialPortCommunicator::gotoMeasurement();
	if (r == XRV_OK)
	{
		m_usbInterface->setRawIo(true);
		m_usbInterface->setTimeout(2000);
	}
	return r;
}

/*! \brief Creates a stream interface
	\param pi The port to use
	\returns The shared pointer to a stream interface
*/
std::shared_ptr<StreamInterface> UsbCommunicator::createStreamInterface(const XsPortInfo &pi)
{
	assert(pi.isUsb());
	m_usbInterface = new UsbInterface();
	std::shared_ptr<StreamInterface> stream(m_usbInterface,
			[&](StreamInterface *intf)
			{
				m_usbInterface = nullptr;
				delete intf;
			}
		);

	setLastResult(m_usbInterface->open(pi));

	return stream;
}
