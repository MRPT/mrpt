/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <xsens/xsdataformat.h>
#include "packetfixeddata.h"

#ifdef LOG_PACKET
#	include "xslog.h"
#	define PACKETLOG	XSENSLOG
#else
#	define PACKETLOG(...)
#endif

/*! \brief Default constructor, creates an empty (invalid) object
*/
PacketFixedData::PacketFixedData()
	: m_infoList(NULL)
	, m_formatList(NULL)
	, m_idList(NULL)
	, m_xm(false)
	, m_itemCount(0)
{
	PACKETLOG("%s creating default %p\n", __FUNCTION__, this);
}

/*! \brief Sized constructor, creates an object with room for \a count device's worth of data
	\details The constructor sets the xbus flag to false
	\param count The number of devices whose metadata is stored in the object
*/
PacketFixedData::PacketFixedData(uint16_t count)
	: m_infoList(NULL)
	, m_formatList(NULL)
	, m_idList(NULL)
	, m_xm(false)
	, m_itemCount(count)
{
	PACKETLOG("%s creating %p with %d items\n", __FUNCTION__, this, count);
	m_formatList = new XsDataFormat[m_itemCount];
	m_infoList = new PacketInfo[m_itemCount];
	m_idList = new XsDeviceId[m_itemCount];
}

/*! \brief Copy constructor
	\param p The object to copy the contents from
*/
PacketFixedData::PacketFixedData(const PacketFixedData& p)
	: m_infoList(NULL)
	, m_formatList(NULL)
	, m_idList(NULL)
	, m_xm(false)
	, m_itemCount(0)
{
	PACKETLOG("%s creating %p from %p\n", __FUNCTION__, this, &p);
	*this = p;
	PACKETLOG("%s done creating %p\n", __FUNCTION__, this);
}

/*! \brief Destructor
*/
PacketFixedData::~PacketFixedData()
{
	PACKETLOG("%s %p\n", __FUNCTION__, this);
	m_itemCount = 0;
	delete[] m_formatList;
	delete[] m_infoList;
	delete[] m_idList;
	PACKETLOG("%s %p exit\n", __FUNCTION__, this);
}

/*! \brief Assignment operator, copies contents from \a data
	\param data The object to copy from
*/
void PacketFixedData::operator = (const PacketFixedData& data)
{
	if (this == &data)
		return;

	PACKETLOG("%s copy from %p to %p\n", __FUNCTION__, &data, this);

	delete[] m_formatList;
	delete[] m_idList;
	delete[] m_infoList;
	m_formatList = NULL;
	m_idList = NULL;
	m_infoList = NULL;

	m_itemCount = data.m_itemCount;
	m_formatList = new XsDataFormat[data.m_itemCount];
	m_idList = new XsDeviceId[data.m_itemCount];
	m_infoList = new PacketInfo[data.m_itemCount];

	for (uint16_t i = 0; i < data.m_itemCount; ++i)
	{
		m_formatList[i] = data.m_formatList[i];
		m_infoList[i] = data.m_infoList[i];
		m_idList[i] = data.m_idList[i];
	}
	m_xm = data.m_xm;

	PACKETLOG("%s exit\n", __FUNCTION__);
}

