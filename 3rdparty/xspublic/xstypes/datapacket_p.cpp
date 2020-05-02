
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

#include "datapacket_p.h"

/*! \cond XS_INTERNAL */

/*! \class DataPacketPrivate
	\brief Internal administration for contained data of XsDataPacket class.
	\details This is only the part that can be stored in an XsMessage, so no TOA and computed packet IDs
*/

volatile std::atomic_int DataPacketPrivate::m_created(0);
volatile std::atomic_int DataPacketPrivate::m_destroyed(0);

/*! \brief Copy constructor */
DataPacketPrivate::DataPacketPrivate(DataPacketPrivate const& p)
	: MapType()		// start with clean map
	, m_refCount(1)	// this is a new object so the ref count is 1
{
	++m_created;
	*this = p;		// does NOT manipulate the ref count
}

/*! \brief Destructor */
DataPacketPrivate::~DataPacketPrivate()
{
	++m_destroyed;
	try
	{
		clear();
	}
	catch(...)
	{
	}
}

/*! \brief Assignment operator
	\param p The item to copy from
	\note This does NOT manipulate the ref count and it shouldn't.
	\return A reference to this
*/
DataPacketPrivate& DataPacketPrivate::operator = (const DataPacketPrivate& p)
{
	if (this != &p)
	{
		clear();
		for (auto i : p)
			insert(i.first, i.second->clone());
	}
	return *this;
}

/*! \brief Clear the contents */
void DataPacketPrivate::clear()
{
	for (auto it : *this)
		delete it.second;
	MapType::clear();
}

/*! \brief Find the item matching \a id
	\details This function will return an iterator to the item that matches \a id or end() if it could not find it.
	The function uses a loose comparison to ensure entry ambiguity. As a result, the returned iterator may have a
	slightly different id than the supplied one.
	\param id The item to search for.
	\return An iterator to the requested item or end()
*/
MapType::const_iterator DataPacketPrivate::find(XsDataIdentifier id) const
{
	return MapType::find(id & XDI_FullTypeMask);
}

/*! \brief Add or overwrite the item with \a id
	\details The function will create a new item or overwite the current item, properly cleaning up existing data
	if necessary.
	\param id The id to attach to the data.
	\param var The data to store.
	\return An internal iterator pointing to the inserted or updated item.
*/
MapType::iterator DataPacketPrivate::insert(XsDataIdentifier id, XsDataPacket_Private::Variant* var)
{
	id = id & XDI_FullTypeMask;
	auto it = MapType::lower_bound(id);
	if (it != end() && it->first == id)
	{
		delete it->second;
		it->second = var;
		return it;
	}
	else
		return MapType::insert(it, std::make_pair(id & XDI_FullTypeMask, var));
}

/*! \brief Remove the item with \a id if it exists, cleaning up associated data */
void DataPacketPrivate::erase(XsDataIdentifier id)
{
	auto it = find(id);
	if (it != end())
		erase(it);
}

/*! \brief Remove the item at \a it, cleaning up associated data */
void DataPacketPrivate::erase(MapType::const_iterator const& it)
{
	delete it->second;
	MapType::erase(it);
}

/*! \brief Merge \a other into this
	\details The function will copy all items from \a other into this. Existing items will only be overwritten if
	\a overwrite is set to true. This function does not detect conflicts between closely related but IDs (such as
	frame range and packet counter). This is left to the caller.
	\param other The other data to merge into this packet
	\param overwrite When set to true, the contents of \a other will overwrite existing data. When set to false, only new items will be added.
*/
void DataPacketPrivate::merge(DataPacketPrivate const& other, bool overwrite)
{
	if (overwrite)
		for (auto i : other)
			insert(i.first, i.second->clone());
	else
	{
		for (auto i : other)
		{
			auto j = find(i.first);
			if (j == end())
				insert(i.first, i.second->clone());
		}
	}
}

/*! \brief Returns the difference between created and destroyed DataPacketPrivate objects, for debugging purposes only */
int DataPacketPrivate::creationDiff()
{
	return m_created.load() - m_destroyed.load();
}

/*! \endcond XS_INTERNAL */
