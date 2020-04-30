
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

#ifndef XSPRESSURE_H
#define XSPRESSURE_H

#include "pstdint.h"

/*! \brief Pressure data.
	\details Contains the pressure data and the pressure age
*/
struct XsPressure {
#ifdef __cplusplus
	/*! \brief Create an XsPressure
		\param pressure the pressure
		\param age the pressure age
	*/
	explicit XsPressure(double pressure = 0, uint8_t age = 0) :
		m_pressure(pressure),
		m_pressureAge(age)
	{
	}

	/*! \brief Create a new XsPressure as copy from \a other
		\param other the pressure carrier to copy from
	*/
	inline XsPressure(XsPressure const& other) :
		m_pressure(other.m_pressure),
		m_pressureAge(other.m_pressureAge)
	{
	}

	/*! \brief Copy the data from \a other
		\param other the pressure carrier to copy from
		\return this
	*/
	inline XsPressure const & operator=(XsPressure const& other)
	{
		m_pressure = other.m_pressure;
		m_pressureAge = other.m_pressureAge;
		return *this;
	}

	/*! \brief Return true if this is equal to \a other
		\param other the pressure carrier to compare against
		\return true if both XsPressures are equal
	*/
	inline bool operator==(XsPressure const& other) const
	{
		return other.m_pressure == m_pressure && other.m_pressureAge == m_pressureAge;
	}
#endif
	double		m_pressure;		//!< Pressure in Pascal
	uint8_t		m_pressureAge;	//!< Age of pressure data in samples
};
typedef struct XsPressure XsPressure;

#endif
