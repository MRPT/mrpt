
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

#include "xssensorranges.h"
#include <cassert>

extern "C" {

/* Find the A in hardware info of the product code

Returns a pointer to the A.
*/
static const char *findHardwareType(const char *productCode)
{
	if (findHardwareManufacturer(productCode) != HMT_MT)
		return nullptr;

	const char *A = strchr(productCode, 'A');
	if (!A)
		return nullptr;
	const char *G = strchr(A, 'G');
	if (!G)
		return nullptr;
	assert(G - A <= 3);
	return A;
}

/*! \brief Return the hardware manufacturer from \a productCode
*/
HardwareManufacturerType findHardwareManufacturerC(const XsString* productCode)
{
	if (strstr(productCode->c_str(), "MT") != nullptr)
		return HMT_MT;

	return HMT_None;
}

/*! \brief Return the hardware type from \a productCode
*/
void findHardwareTypeC(const XsString* productCode, XsString* resultValue)
{
	if (!resultValue)
		return;
	const char *hwt = findHardwareType(productCode->c_str());
	if (!hwt)
		resultValue->clear();
	else
		*resultValue = hwt;
}

/* Return the accelerometer range field */
static char accelerometerRangeField(const char *productCode)
{
	const char *hwi = findHardwareType(productCode);
	if (!hwi)
		return 0;
	return *(hwi + 1);
}

/* Return the gyroscope range field */
static char gyroscopeRangeField(const char *productCode)
{
	const char *hwi = findHardwareType(productCode);
	if (!hwi)
		return 0;
	const char *G = strchr(hwi, 'G');
	if (!G)
		return 0;
	return *(G + 1);
}

/*! \brief The accelerometer range from product code \a productCode
*/
double accelerometerRangeC(const XsString *productCode, int32_t hwVersionMajor)
{
	switch (findHardwareManufacturerC(productCode))
	{
	case HardwareManufacturerType::HMT_MT:
		switch(accelerometerRangeField(productCode->c_str()))
		{
		case '1': return   100.0;
		case '2': return    20.0;
		case '3': return    17.0;
		case '5': return    50.0;
		case '6': return    60.0;
		case '7': return   160.0;
		case '8':
		{
			if (hwVersionMajor < 3)
				return 180.0;
			else
				return 200.0;
		}
		default: return 10000.0;
		}

	default:
		return 10000.0;
	}
}

/*! \brief The actual accelerometer range from product code \a productCode

This is a measured value and possibly larger than what accelerometerRange() returns.
*/
double actualAccelerometerRangeC(const XsString *productCode, int32_t hwVersionMajor)
{
	switch (findHardwareManufacturerC(productCode))
	{
	case HardwareManufacturerType::HMT_MT:
		switch(accelerometerRangeField(productCode->c_str()))
		{
		case '1': return   100.0;
		case '2': return    20.0;
		case '3': return    17.0;
		case '5': return    50.0;
		case '6': return    60.0;
		case '7': return   160.0;
		case '8':
		{
			if (hwVersionMajor < 3)
				return 180.0;
			else
				return 200.0;
		}
		default: return 10000.0;
		}

	default:
		return 10000.0;
	}
}

/*! \brief The gyroscope range from product code \a productCode
*/
double gyroscopeRangeC(const XsString *productCode)
{
	switch (findHardwareManufacturerC(productCode))
	{
	case HardwareManufacturerType::HMT_MT:
		switch(gyroscopeRangeField(productCode->c_str()))
		{
		case '0': return  1000.0;
		case '1': return   150.0;
		case '2': return  1200.0;
		case '3': return   300.0;
		case '4': return   450.0;
		case '5': return  2500.0;
		case '6': return  1800.0;
		case '9': return   900.0;
		default: return 10000.0;
		}

	default:
		return 10000.0;
	}
}

/*! \brief The actual gyroscope range from product code \a productCode

This is a measured value and possibly larger than what gyroscopeRange() returns.
*/
double actualGyroscopeRangeC(const XsString *productCode)
{
	switch (findHardwareManufacturerC(productCode))
	{
	case HardwareManufacturerType::HMT_MT:
		switch(gyroscopeRangeField(productCode->c_str()))
		{
		case '0': return  1000.0;
		case '1': return   180.0;
		case '2': return  1700.0;
		case '3': return   420.0;
		case '4': return   450.0;
		case '5': return  2500.0;
		case '6': return  2000.0;
		case '9': return  1080.0;
		default: return 10000.0;
		}

	default:
		return 10000.0;
	}
}

}
