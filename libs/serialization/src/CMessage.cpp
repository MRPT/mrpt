/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "serialization-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/archiveFrom_std_streams.h>
#include <sstream>
#include <cstring>  // memcpy()

using namespace mrpt::serialization;

void CMessage::serializeObject(const CSerializable* obj)
{
	MRPT_START
	std::stringstream auxStream;
	auto arch = mrpt::serialization::archiveFrom<std::iostream>(auxStream);

	// Dump the object in the memory stream:
	arch.WriteObject(obj);

	// Copy data to message:
	const auto& data = auxStream.str();
	content.resize(data.size());
	memcpy(
		&content[0],  // Dest
		&data[0],  // Src
		content.size());

	MRPT_END
}

/*---------------------------------------------------------------
				deserializeIntoExistingObject
 ---------------------------------------------------------------*/
void CMessage::deserializeIntoExistingObject(CSerializable* obj)
{
	MRPT_START
	std::stringstream auxStream;
	auto arch = mrpt::serialization::archiveFrom<std::iostream>(auxStream);

	// Copy data into the stream:
	arch.WriteBuffer(&content[0], content.size());
	auxStream.seekg(0);

	// Try to parse data into existing object:
	arch.ReadObject(obj);

	MRPT_END
}

/*---------------------------------------------------------------
				deserializeIntoNewObject
 ---------------------------------------------------------------*/
void CMessage::deserializeIntoNewObject(CSerializable::Ptr& obj)
{
	MRPT_START
	std::stringstream auxStream;
	auto arch = mrpt::serialization::archiveFrom<std::iostream>(auxStream);

	// Copy data into the stream:
	if (!content.empty())
	{
		arch.WriteBuffer(&content[0], content.size());
		auxStream.seekg(0);

		// Try to parse data into a new object:
		obj = arch.ReadObject();
	}
	else
		obj.reset();

	MRPT_END
}

/*---------------------------------------------------------------
				setContentFromString
 ---------------------------------------------------------------*/
void CMessage::setContentFromString(const std::string& str)
{
	content.resize(str.size());
	if (content.size() > 0) memcpy(&content[0], str.c_str(), str.size());
}

/*---------------------------------------------------------------
				getContentAsString
 ---------------------------------------------------------------*/
void CMessage::getContentAsString(std::string& str)
{
	str.resize(content.size());
	if (content.size() > 0) memcpy(&str[0], &content[0], str.size());
}
