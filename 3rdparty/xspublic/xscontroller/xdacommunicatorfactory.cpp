
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

#include "xdacommunicatorfactory.h"
#include "mtbfilecommunicator.h"
#include "serialportcommunicator.h"
#include "usbcommunicator.h"

namespace CommunicatorType {
	static const CommunicatorFactory::CommunicatorTypeId UNKNOWN    = 0;
	static const CommunicatorFactory::CommunicatorTypeId MTBFILE    = 1;
	static const CommunicatorFactory::CommunicatorTypeId USB        = 2;
	static const CommunicatorFactory::CommunicatorTypeId SERIALPORT = 3;
}

/*! \class XdaCommunicatorFactory
	\brief XDA communication factory
*/

/*! \brief Construct this factory */
XdaCommunicatorFactory::XdaCommunicatorFactory()
{
}

/*! \copydoc CommunicatorFactory::filenameToCommunicatorId */
XdaCommunicatorFactory::CommunicatorTypeId XdaCommunicatorFactory::filenameToCommunicatorId(const XsString &) const
{
	/* Everything is expected to be an mtb file for now.
	   It is very wel possible that we're going to have to check for
	   other weird stuff as well, such as COM1 or /dev/ttyUSB0,
	   which would state a case for just a string-based approach.
	*/
	return CommunicatorType::MTBFILE;
}

/*! \copydoc CommunicatorFactory::portInfoToCommunicatorId */
XdaCommunicatorFactory::CommunicatorTypeId XdaCommunicatorFactory::portInfoToCommunicatorId(const XsPortInfo &portInfo) const
{
	for (auto it = constructors().begin(); it != constructors().end(); ++it)
		if (it->second.second && it->second.second(portInfo))
			return it->first;

	return CommunicatorType::UNKNOWN;
}

namespace {
	/*! \returns True if a \a portInfo is USB port
		\param portInfo The port info to check
	*/
	bool isUsb(const XsPortInfo &portInfo)
	{
		return portInfo.isUsb();
	}

	/*! \returns True if a \a portInfo is a serial port
		\param portInfo The port info to check
	*/
	bool isSerialPort(const XsPortInfo &portInfo)
	{
		return !portInfo.isUsb() && !portInfo.isNetwork();
	}
}

/*! \brief Register the communicator types */
void XdaCommunicatorFactory::registerCommunicatorTypes()
{
	(void)registerType(CommunicatorType::MTBFILE, &MtbFileCommunicator::construct, nullptr);
	(void)registerType(CommunicatorType::SERIALPORT, &SerialPortCommunicator::construct, &isSerialPort);
	(void)registerType(CommunicatorType::USB, &UsbCommunicator::construct, &isUsb);
}
