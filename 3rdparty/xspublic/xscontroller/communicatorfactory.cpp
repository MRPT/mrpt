
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

#include "communicatorfactory.h"

/*! \brief Create the factory
*/
CommunicatorFactory::CommunicatorFactory()
{
}

/*! \brief Destory the factory
*/
CommunicatorFactory::~CommunicatorFactory()
{
}

/*! \brief Register a communicator type with the factory
*/
bool CommunicatorFactory::registerType(CommunicatorTypeId typeId, CommunicatorConstructFunc constructFunc, PortInfoMatchFunc matchFunc)
{
	return m_constructors.insert(ConstructorsMap::value_type(typeId, std::make_pair(constructFunc, matchFunc))).second;
}

/*! \brief Create a communicator based on a port info
*/
Communicator *CommunicatorFactory::create(const XsPortInfo &portInfo) const
{
	CommunicatorTypeId id = portInfoToCommunicatorId(portInfo);
	return construct(id);
}

/*! \brief Create a communicator based on a filename
*/
Communicator *CommunicatorFactory::create(const XsString &filename) const
{
	CommunicatorTypeId id = filenameToCommunicatorId(filename);
	return construct(id);
}

/*! \brief Construct a communicator based on \a typeId
*/
Communicator *CommunicatorFactory::construct(CommunicatorTypeId typeId) const
{
	Communicator *c = nullptr;
	if (typeId != CommunicatorType::INVALID)
	{
		ConstructorsMap::const_iterator i = m_constructors.find(typeId);
		if (i != m_constructors.end())
			c = i->second.first();
	}
	return c;
}
