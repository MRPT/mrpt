
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

#ifndef COMMUNICATORFACTORY_H
#define COMMUNICATORFACTORY_H

#include "compat.h"

#include <map>
#include <memory>

struct Communicator;
struct XsString;
struct XsPortInfo;

/*! \class CommunicatorFactory
	\brief A Factory for the communicators
*/
class CommunicatorFactory
{
public:
	/*! \brief Create a new CommunicatorFactory and register its basic types
		\return A unique pointer to the created factory
	*/
	template <typename T>
	static std::unique_ptr<T> createFactory()
	{
		auto factory = std::make_unique<T>();
		factory->registerCommunicatorTypes();
		return factory;
	}

	CommunicatorFactory();
	virtual ~CommunicatorFactory();

	//! The typedef of the communicator type ID
	typedef unsigned int CommunicatorTypeId;

	//! The typedef of the communicator constructor function
	typedef Communicator* (*CommunicatorConstructFunc)();

	//! The typedef of the port info match function
	typedef bool (*PortInfoMatchFunc)(const XsPortInfo &);

	Communicator *create(const XsPortInfo &portInfo) const;
	Communicator *create(const XsString &filename) const;

	bool registerType(CommunicatorTypeId typeId, CommunicatorConstructFunc constructFunc, PortInfoMatchFunc matchFunc);

	/*! \brief Match a filename to a communicator
		\param filename A name of file
		\returns A communicator type ID
	*/
	virtual CommunicatorTypeId filenameToCommunicatorId(const XsString &filename) const = 0;

	/*! \brief Match a XsPortInfo to a communicator
		\param portInfo An information about the port
		\returns A communicator type ID
	*/
	virtual CommunicatorTypeId portInfoToCommunicatorId(const XsPortInfo &portInfo) const = 0;

	/*! \brief Registrates communicator types */
	virtual void registerCommunicatorTypes() {}

protected:
	//! The typedef of a map for a communicator type ID and constructors map
	typedef std::map<CommunicatorTypeId, std::pair<CommunicatorConstructFunc, PortInfoMatchFunc>> ConstructorsMap;

	//! \returns A constant reference to the constructors map
	ConstructorsMap const& constructors() const { return m_constructors; }
	virtual Communicator *construct(CommunicatorTypeId communicator) const;

private:
	ConstructorsMap m_constructors;
};

namespace CommunicatorType {
	static const CommunicatorFactory::CommunicatorTypeId INVALID = 0;
}

#endif
