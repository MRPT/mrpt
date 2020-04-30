
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

#include "messageserializer.h"
#include <xstypes/xsdeviceid.h>
#include <xstypes/xsoutputconfiguration.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xscanoutputconfiguration.h>
#include <xstypes/xscanoutputconfigurationarray.h>


/*! \class MessageSerializer
	\brief A class that does the message serialization
*/

/*!	\brief Default constructor
*/
MessageSerializer::MessageSerializer(XsMessage &msg, XsSize offset) :
	m_message(msg),
	m_index(offset)
{

}

/*! \brief Destructor
*/
MessageSerializer::~MessageSerializer()
{
	finalize();
}

/*! \brief Output stream operator that adds a XsDataIdentifier to the stream
	\param id The data identifier
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(XsDataIdentifier id)
{
	return operator<<((uint16_t)id);
}

/*! \brief Output stream operator that adds a XsCanDataIdentifier to the stream
	\param id The CAN data identifier
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(XsCanDataIdentifier id)
{
	return operator<<((uint8_t)id);
}

/*! \brief Output stream operator that adds a XsCanIdLenght to the stream
	\param idl The CAN Id lenght enum value
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(XsCanFrameFormat idl)
{
	return operator<<((uint8_t)((idl==XCFF_11Bit_Identifier)?0:1));
}

/*! \brief Output stream operator that adds a XsDeviceId to the stream
	\param id The device ID
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(const XsDeviceId &id)
{
	return operator<<(id.legacyDeviceId());
}

/*! \brief Output stream operator that adds a uint8_t to the stream
	\param value The value to add to the stream
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(uint8_t value)
{
	m_message.setDataByte(value, m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Output stream operator that adds a uint16_t to the stream
	\param value The value to add to the stream
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(uint16_t value)
{
	m_message.setDataShort(value, m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Output stream operator that adds a uint32_t to the stream
	\param value The value to add to the stream
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(uint32_t value)
{
	m_message.setDataLong(value, m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Output stream operator that adds a uint64_t to the stream
	\param value The value to add to the stream
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(uint64_t value)
{
	m_message.setDataLongLong(value, m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Output stream operator that adds a XsOutputConfigurationArray to the stream
	\param config The output configuration array
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(const XsOutputConfigurationArray &config)
{
	if (config.size() == 0)
		return (*this << (XsOutputConfiguration(XDI_None, 0)));

	for (auto &cfg : config)
		*this << cfg;
	return *this;
}

/*! \brief Output stream operator that adds a XsOutputConfiguration to the stream
	\param cfg The output configuration
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(const XsOutputConfiguration &cfg)
{
	return (*this << cfg.m_dataIdentifier << cfg.m_frequency);
}

/*! \brief Output stream operator that adds a XsOutputConfigurationArray to the stream
	\param config The output configuration array
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(const XsCanOutputConfigurationArray &config)
{
	if (config.size() == 0)
		return (*this << (XsCanOutputConfiguration(XCFF_11Bit_Identifier, XCDI_Invalid, 0, 0)));

	for (auto &cfg : config)
		*this << cfg;
	return *this;
}

/*! \brief Output stream operator that adds a XsCanOutputConfiguration to the stream
	\param cfg The output configuration
	\returns A reference to this object
*/
MessageSerializer &MessageSerializer::operator<<(const XsCanOutputConfiguration &cfg)
{
	return (*this << cfg.m_dataIdentifier << cfg.m_frameFormat << cfg.m_id << cfg.m_frequency);
}


/*! \brief Appends the data to the message
	\param data The value to add to the message
	\param size The size of this value
*/
void MessageSerializer::append(const uint8_t *data, XsSize size)
{
	m_message.setDataBuffer(data, size, m_index);
	m_index += size;
}

/*! \brief Finalizes the message serialization
*/
void MessageSerializer::finalize()
{
	m_message.resizeData(m_index);
}


/*! \class MessageDeserializer
	\brief A class that does the message deserialization
*/

/*!	\brief Default constructor
*/
MessageDeserializer::MessageDeserializer(const XsMessage &msg, XsSize offset) :
	m_message(msg),
	m_index(offset)
{

}

/*! \brief Destructor
*/
MessageDeserializer::~MessageDeserializer()
{
}

/*! \brief Input stream operator that takes a XsDataIdentifier from the stream
	\param value Reference in which the data identifier is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsDataIdentifier &value)
{
	uint16_t v;
	operator>>(v);
	value = static_cast<XsDataIdentifier>(v);
	return *this;
}

/*! \brief Input stream operator that takes a XsCanDataIdentifier from the stream
	\param value Reference in which the CAN data identifier is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsCanDataIdentifier &value)
{
	uint8_t v;
	operator>>(v);
	value = static_cast<XsCanDataIdentifier>(v);
	return *this;
}

/*! \brief Input stream operator that takes a XsCanIdLength from the stream
	\param value Reference in which the CAN ID length is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsCanFrameFormat &value)
{
	uint8_t v;
	operator>>(v);
	value = static_cast<XsCanFrameFormat>(v);
	return *this;
}

/*! \brief Input stream operator that takes a XsDeviceId from the stream
	\param value Reference in which the device ID is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsDeviceId &value)
{
	uint32_t v;
	operator>>(v);
	value = XsDeviceId(v);
	return *this;
}

/*! \brief Input stream operator that takes a uint8_t from the stream
	\param value Reference in which the read value is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(uint8_t &value)
{
	value = m_message.getDataByte(m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Input stream operator that takes a int8_t from the stream
	\param value Reference in which the read value is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(int8_t &value)
{
	value = (int8_t)m_message.getDataByte(m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Input stream operator that takes a uint16_t from the stream
	\param value Reference in which the read value is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(uint16_t &value)
{
	value = m_message.getDataShort(m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Input stream operator that takes a uint32_t from the stream
	\param value Reference in which the read value is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(uint32_t &value)
{
	value = m_message.getDataLong(m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Input stream operator that takes a uint64_t from the stream
	\param value Reference in which the read value is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(uint64_t &value)
{
	value = m_message.getDataLongLong(m_index);
	m_index += sizeof(value);
	return *this;
}

/*! \brief Input stream operator that takes a XsOutputConfigurationArray from the stream
	\param config Reference in which the output configuration array is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsOutputConfigurationArray &config)
{
	config.clear();
	while (!atEnd())
	{
		XsOutputConfiguration cfg;
		*this >> cfg;
		config.push_back(cfg);
	}
	return *this;
}

/*! \brief Input stream operator that takes a XsOutputConfiguration from the stream
	\param cfg Reference in which the output configuration is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsOutputConfiguration &cfg)
{
	return (*this >> cfg.m_dataIdentifier >> cfg.m_frequency);
}

/*! \brief Input stream operator that takes a XsCanOutputConfigurationArray from the stream
	\param config Reference in which the CAN output configuration array is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsCanOutputConfigurationArray &config)
{
	config.clear();
	while (!atEnd())
	{
		XsCanOutputConfiguration cfg;
		*this >> cfg;
		config.push_back(cfg);
	}
	return *this;
}

/*! \brief Input stream operator that takes a XsCanOutputConfiguration from the stream
	\param cfg Reference in which the CAN output configuration is stored
	\returns A reference to this object
*/
MessageDeserializer &MessageDeserializer::operator>>(XsCanOutputConfiguration &cfg)
{
	return (*this >> cfg.m_dataIdentifier >> cfg.m_frameFormat >> cfg.m_id >> cfg.m_frequency);
}

/*! \brief Checks if we are at the end of message.
	\returns true if we are at the end.
*/
bool MessageDeserializer::atEnd() const
{
	assert(m_index <= m_message.getDataSize());
	return m_index == m_message.getDataSize();
}
