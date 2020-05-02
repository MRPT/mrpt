
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

#ifndef DEVICETYPES_H
#define DEVICETYPES_H

#include "devicefactory.h"

namespace DeviceType {
	static const DeviceFactory::DeviceTypeId BODYPACK				= 5;
	static const DeviceFactory::DeviceTypeId AWINDA2STATION			= 6;
	static const DeviceFactory::DeviceTypeId AWINDA2DONGLE			= 7;
	static const DeviceFactory::DeviceTypeId AWINDA2OEM				= 8;
	static const DeviceFactory::DeviceTypeId SYNCSTATION			= 9;
	static const DeviceFactory::DeviceTypeId MTI_X					= 20;
	static const DeviceFactory::DeviceTypeId MTI_X0					= 21;
	static const DeviceFactory::DeviceTypeId MTI_X00				= 22;
	static const DeviceFactory::DeviceTypeId MTIG					= 23;
	static const DeviceFactory::DeviceTypeId MTI_7					= 24;
	static const DeviceFactory::DeviceTypeId MTI_6X0				= 25;
	static const DeviceFactory::DeviceTypeId MTX2					= 30;
	static const DeviceFactory::DeviceTypeId GLOVE					= 50;
	static const DeviceFactory::DeviceTypeId MTW2					= 80;
	static const DeviceFactory::DeviceTypeId IMARIFOG				= 100;
	static const DeviceFactory::DeviceTypeId IMARFSAS				= 101;
	static const DeviceFactory::DeviceTypeId ABMCLOCKMASTER			= 200;
	static const DeviceFactory::DeviceTypeId HILDEVICE				= 300;
}

#endif
