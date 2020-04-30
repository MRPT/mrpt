
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

#ifndef XSDEVICEPARAMETER_H
#define XSDEVICEPARAMETER_H

#include "xsdeviceparameteridentifier.h"
#include <xstypes/pstdint.h>

#ifdef __cplusplus
extern "C" {
struct XsDeviceParameter;
} // extern "C"
#endif

/*!
 *	\class XsDeviceParameter
 *	\brief Class to set and retrieve parameters from a XsDevice object.
 */
struct XsDeviceParameter
{
#ifdef __cplusplus
	//!	\brief Constructor, initializes the object with a parameter \a id.
	explicit XsDeviceParameter(XsDeviceParameterIdentifier id)
		: m_id(id)
		, m_value(0)
	{
	}

	//!	\brief Constructor, initializes the object with a parameter \a id and desired \a value.
	explicit XsDeviceParameter(XsDeviceParameterIdentifier id, int value)
		: m_id(id)
		, m_value(static_cast<uint32_t>(value))
	{
	}

	//!	\brief Returns the current parameter identifier.
	XsDeviceParameterIdentifier id() const
	{
		return m_id;
	}

	/*!
	 *	\brief Returns the stored parameter value.
	 *	\returns The parameter value in the desired type.
	 */
	template<typename T>
	T getValue() const;


	/*!
	 *	\brief Sets the parameter value.
	 *	\param value: the desired parameter value.
	 */
	template<typename T>
	void setValue(T value);

private:
#endif
	XsDeviceParameterIdentifier m_id;
	int m_value;
};


#ifdef __cplusplus
template<>
//! \copydoc XsDeviceParameter::getValue
inline bool XsDeviceParameter::getValue<bool>() const
{
	return m_value > 0;
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline uint8_t XsDeviceParameter::getValue<uint8_t>() const
{
	return static_cast<uint8_t>(m_value);
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline uint16_t XsDeviceParameter::getValue<uint16_t>() const
{
	return static_cast<uint16_t>(m_value);
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline uint32_t XsDeviceParameter::getValue<uint32_t>() const
{
	return static_cast<uint32_t>(m_value);
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline int XsDeviceParameter::getValue<int>() const
{
	return m_value;
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<bool>(bool value)
{
	m_value = value ? 1 :0;
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<uint8_t>(uint8_t value)
{
	m_value = static_cast<int>(value);
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<uint16_t>(uint16_t value)
{
	m_value = static_cast<int>(value);
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<uint32_t>(uint32_t value)
{
	m_value = static_cast<int>(value);
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<int>(int value)
{
	m_value = value;
}
#endif

typedef struct XsDeviceParameter XsDeviceParameter;

#endif
