
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

#ifndef IDFETCHHELPERS_H
#define IDFETCHHELPERS_H

#include <regex>
#include <xstypes/xsdeviceid.h>
#include <algorithm>

/* Helper functions for fetching of device/vendor/product ids from text strings */

/* Get the first match for _regex_ within _string_ */
static inline std::string searchOne(std::string const& string, std::string const& regex)
{
	std::smatch sm;
	std::regex_search(string, sm, std::regex(regex));
	if (sm.size() < 2)
		return std::string();
	return sm[1];
}

/* Fetch the device ID from the given device path */
static inline XsDeviceId deviceIdFromDevPath(std::string const& devpath)
{
	XsDeviceId deviceId;
	std::string id = searchOne(devpath, "([0-9A-Fa-f]*)$");
	if (!id.empty())
		deviceId.fromString(XsString(id));
	return deviceId;
}

/* Fetch the vendor ID from the given string */
static inline uint16_t vidFromString(std::string const& string)
{
	try
	{
		std::string regex = string;
		std::transform(regex.begin(), regex.end(), regex.begin(), ::toupper);
		auto t1 = searchOne(regex, "VID_([0-9a-fA-F]{4}).*PID_.*");
		if (t1.empty())	// prevent unnecessary exceptions
			return 0;
		return static_cast<uint16_t>(std::stoi(t1, nullptr, 16));
	}
	catch (std::invalid_argument &)
	{
		return 0;
	}
}

/* Fetch the product ID from the given string */
static inline uint16_t pidFromString(std::string const& string)
{
	try
	{
		std::string regex = string;
		std::transform(regex.begin(), regex.end(), regex.begin(), ::toupper);
		auto t1 = searchOne(regex, "VID_.*PID_([0-9a-fA-F]{4})");
		if (t1.empty())	// prevent unnecessary exceptions
			return 0;
		return static_cast<uint16_t>(std::stoi(t1, nullptr, 16));
	}
	catch (std::invalid_argument &)
	{
		return 0;
	}
}

#endif
