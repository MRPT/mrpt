
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

#ifndef MESSAGESERIALIZER_H
#define MESSAGESERIALIZER_H

#include <xstypes/xsmessage.h>
#include <xstypes/xsdataidentifier.h>
#include <xstypes/xscandataidentifier.h>
#include <xstypes/xscanframeformat.h>

struct XsDeviceId;
struct XsOutputConfigurationArray;
struct XsOutputConfiguration;
struct XsCanOutputConfigurationArray;
struct XsCanOutputConfiguration;

class MessageSerializer
{
public:
	MessageSerializer(XsMessage &message, XsSize offset = 0);
	virtual ~MessageSerializer();

	MessageSerializer &operator<<(XsDataIdentifier value);
	MessageSerializer &operator<<(XsCanDataIdentifier value);
	MessageSerializer &operator<<(XsCanFrameFormat value);
	MessageSerializer &operator<<(const XsDeviceId &id);
	MessageSerializer &operator<<(uint8_t value);
	MessageSerializer &operator<<(uint16_t value);
	MessageSerializer &operator<<(uint32_t value);
	MessageSerializer &operator<<(uint64_t value);

	/*! \brief Output stream operator that adds a int64_t to the stream
		\param value The value to add to the stream
		\returns A reference to this object
	*/
	inline MessageSerializer &operator<<(int64_t value)
	{
		return (operator<<((uint64_t)value));
	}

	/*! \brief Output stream operator that adds a int32_t to the stream
		\param value The value to add to the stream
		\returns A reference to this object
	*/
	inline MessageSerializer &operator<<(int32_t value)
	{
		return (operator<<((uint32_t)value));
	}

	/*! \brief Output stream operator that adds a int16_t to the stream
		\param value The value to add to the stream
		\returns A reference to this object
	*/
	inline MessageSerializer &operator<<(int16_t value)
	{
		return (operator<<((uint16_t)value));
	}

	/*! \brief Output stream operator that adds a int8_t to the stream
		\param value The value to add to the stream
		\returns A reference to this object
	*/
	inline MessageSerializer &operator<<(int8_t value)
	{
		return (operator<<((uint8_t)value));
	}

	MessageSerializer &operator<<(const XsOutputConfigurationArray &config);
	MessageSerializer &operator<<(const XsOutputConfiguration &cfg);

	MessageSerializer &operator<<(const XsCanOutputConfigurationArray &config);
	MessageSerializer &operator<<(const XsCanOutputConfiguration &cfg);

	/*! \returns The current index */
	inline XsSize index() const { return m_index; }

	void append(const uint8_t *data, XsSize size);
	void finalize();
private:
	XsMessage &m_message;
	XsSize m_index;
};

class MessageDeserializer
{
public:
	MessageDeserializer(const XsMessage &message, XsSize offset = 0);
	virtual ~MessageDeserializer();

	MessageDeserializer &operator>>(XsDataIdentifier &value);
	MessageDeserializer &operator>>(XsCanDataIdentifier &value);
	MessageDeserializer &operator>>(XsCanFrameFormat &value);
	MessageDeserializer &operator>>(XsDeviceId &id);
	MessageDeserializer &operator>>(uint8_t &value);
	MessageDeserializer &operator>>(int8_t &value);
	MessageDeserializer &operator>>(uint16_t &value);
	MessageDeserializer &operator>>(uint32_t &value);
	MessageDeserializer &operator>>(uint64_t &value);

	/*! \brief Input stream operator that takes a int64_t from the stream
		\param value Reference in which the read value is stored
		\returns A reference to this object
	*/
	inline MessageDeserializer &operator>>(int64_t &value)
	{
		return (operator>>((uint64_t&)value));
	}

	/*! \brief Input stream operator that takes a int32_t from the stream
		\param value Reference in which the read value is stored
		\returns A reference to this object
	*/
	inline MessageDeserializer &operator>>(int32_t &value)
	{
		return (operator>>((uint32_t&)value));
	}

	/*! \brief Input stream operator that takes a int16_t from the stream
		\param value Reference in which the read value is stored
		\returns A reference to this object
	*/
	inline MessageDeserializer &operator>>(int16_t &value)
	{
		return (operator>>((uint16_t&)value));
	}

	bool atEnd() const;

	MessageDeserializer &operator>>(XsOutputConfigurationArray &config);
	MessageDeserializer &operator>>(XsOutputConfiguration &cfg);

	MessageDeserializer &operator>>(XsCanOutputConfigurationArray &config);
	MessageDeserializer &operator>>(XsCanOutputConfiguration &cfg);

	//! \returns The current message
	const XsMessage& message() { return m_message; }

	//! \returns The current index
	inline XsSize index() const { return m_index; }

private:
	const XsMessage &m_message;
	XsSize m_index;
};

#endif
