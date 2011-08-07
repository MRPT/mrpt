/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>  // Only for precomp. headers, include all libmrpt-core headers.


#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::poses;
using namespace std;

#include <mrpt/utils/metaprogramming.h>
using namespace mrpt::utils::metaprogramming;

IMPLEMENTS_SERIALIZABLE(CSimpleMap, CSerializable,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CSimpleMap::CSimpleMap() : m_posesObsPairs()
{
}

/*---------------------------------------------------------------
					Copy
  ---------------------------------------------------------------*/
CSimpleMap::CSimpleMap( const CSimpleMap &o )  :
	m_posesObsPairs( o.m_posesObsPairs )
{
	for_each( m_posesObsPairs.begin(), m_posesObsPairs.end(), ObjectPairMakeUnique() );
}

/*---------------------------------------------------------------
					Copy
  ---------------------------------------------------------------*/
CSimpleMap & CSimpleMap::operator = ( const CSimpleMap& o)
{
	MRPT_START

	//TPosePDFSensFramePair	pair;

	if (this == &o) return *this;		// It may be used sometimes

	m_posesObsPairs = o.m_posesObsPairs;
	for_each( m_posesObsPairs.begin(), m_posesObsPairs.end(), ObjectPairMakeUnique() );

	return *this;

	MRPT_END
}


/*---------------------------------------------------------------
						size
  ---------------------------------------------------------------*/
size_t CSimpleMap::size() const
{
	return m_posesObsPairs.size();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CSimpleMap::clear()
{
	m_posesObsPairs.clear();
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CSimpleMap::~CSimpleMap()
{
	clear();
}

/*---------------------------------------------------------------
							get const
  ---------------------------------------------------------------*/
void  CSimpleMap::get(
		size_t	        index,
		CPose3DPDFPtr &out_posePDF,
		CSensoryFramePtr &out_SF ) const
{
	if (index>=m_posesObsPairs.size())
		THROW_EXCEPTION("Index out of bounds");

	out_posePDF	= m_posesObsPairs[index].first;
	out_SF		= m_posesObsPairs[index].second;
}

/*---------------------------------------------------------------
						remove
  ---------------------------------------------------------------*/
void  CSimpleMap::remove(size_t index)
{
	MRPT_START

	if (index>=m_posesObsPairs.size())
		THROW_EXCEPTION("Index out of bounds");

	m_posesObsPairs.erase( m_posesObsPairs.begin() + index );

	MRPT_END
}


/*---------------------------------------------------------------
						set
  ---------------------------------------------------------------*/
void  CSimpleMap::set(
	size_t	index,
	const CPose3DPDFPtr &in_posePDF,
	const CSensoryFramePtr & in_SF )
{
	MRPT_START

	if (index>=m_posesObsPairs.size())
		THROW_EXCEPTION("Index out of bounds");

	if (in_posePDF) m_posesObsPairs[index].first = in_posePDF;
	if (in_SF) 		m_posesObsPairs[index].second = in_SF;

	MRPT_END
}

/*---------------------------------------------------------------
						set 2D
  ---------------------------------------------------------------*/
void  CSimpleMap::set(
	size_t	index,
	const CPosePDFPtr &in_posePDF,
	const CSensoryFramePtr &in_SF )
{
	MRPT_START

	if (index>=m_posesObsPairs.size())
		THROW_EXCEPTION("Index out of bounds");

	if (in_posePDF) 	m_posesObsPairs[index].first = CPose3DPDFPtr( CPose3DPDF::createFrom2D( *in_posePDF ) );
	if (in_SF) 			m_posesObsPairs[index].second = in_SF;

	MRPT_END
}


/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void  CSimpleMap::insert( const CPose3DPDF *in_posePDF, const CSensoryFramePtr &in_SF )
{
	MRPT_START

	TPosePDFSensFramePair	pair;

	pair.second  = in_SF;
	pair.first	 = CPose3DPDFPtr( static_cast<CPose3DPDF*>(in_posePDF->duplicate()) );

	m_posesObsPairs.push_back( pair );

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void  CSimpleMap::insert(
	const CPose3DPDFPtr &in_posePDF,
	const CSensoryFramePtr &in_SF )
{
	MRPT_START

	TPosePDFSensFramePair	pair;

	pair.second  = in_SF;
	pair.first	 = in_posePDF;

	m_posesObsPairs.push_back( pair );

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void  CSimpleMap::insert( const CPose3DPDF *in_posePDF, const CSensoryFrame &in_SF )
{
	MRPT_START

	TPosePDFSensFramePair	pair;

	pair.second  = CSensoryFramePtr( new CSensoryFrame(in_SF) );
	pair.first	 = CPose3DPDFPtr( static_cast<CPose3DPDF*>(in_posePDF->duplicate()) );

	m_posesObsPairs.push_back( pair );

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void  CSimpleMap::insert( const CPosePDF *in_posePDF, const CSensoryFrame &in_SF )
{
	MRPT_START

	TPosePDFSensFramePair	pair;

	pair.second  = CSensoryFramePtr( new CSensoryFrame(in_SF) );
	pair.first	 = CPose3DPDFPtr( static_cast<CPose3DPDF*>(in_posePDF->duplicate()) );

	m_posesObsPairs.push_back( pair );

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void  CSimpleMap::insert( const CPosePDF *in_posePDF, const CSensoryFramePtr &in_SF )
{
	MRPT_START

	TPosePDFSensFramePair	pair;

	pair.second  = in_SF;
	pair.first	 = CPose3DPDFPtr( static_cast<CPose3DPDF*>(in_posePDF->duplicate()) );

	m_posesObsPairs.push_back( pair );

	MRPT_END
}

/*---------------------------------------------------------------
						insert  2D
  ---------------------------------------------------------------*/
void  CSimpleMap::insert(
	const CPosePDFPtr &in_posePDF,
	const CSensoryFramePtr &in_SF )
{
	insert( CPose3DPDFPtr( CPose3DPDF::createFrom2D( *in_posePDF ) ) ,in_SF);
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CSimpleMap::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t		i,n;
		n = m_posesObsPairs.size();
		out << n;
		for (i=0;i<n;i++)
			out << *m_posesObsPairs[i].first << *m_posesObsPairs[i].second;
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CSimpleMap::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 1:
		{
			uint32_t	i,n;
			clear();
			in >> n;
			m_posesObsPairs.resize(n);
			for (i=0;i<n;i++)
				in >> m_posesObsPairs[i].first >> m_posesObsPairs[i].second;
		} break;
	case 0:
		{
			// There are 2D poses PDF instead of 3D: transform them:
			uint32_t	i,n;
			clear();
			in >> n;
			m_posesObsPairs.resize(n);
			for (i=0;i<n;i++)
			{
				CPosePDFPtr aux2Dpose;
				in >> aux2Dpose >> m_posesObsPairs[i].second;
				m_posesObsPairs[i].first = CPose3DPDFPtr( CPose3DPDF::createFrom2D( *aux2Dpose ) );
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
					changeCoordinatesOrigin
  ---------------------------------------------------------------*/
void CSimpleMap::changeCoordinatesOrigin( const CPose3D  &newOrigin )
{
	for (TPosePDFSensFramePairList::iterator it=m_posesObsPairs.begin(); it!=m_posesObsPairs.end(); ++it)
		it->first->changeCoordinatesReference(newOrigin);
}

/** Save this object to a .simplemap binary file (compressed with gzip)
* \sa loadFromFile
* \return false on any error.
*/
bool CSimpleMap::saveToFile(const std::string &filName) const
{
	try
	{
		mrpt::utils::CFileGZOutputStream  f(filName);
		f << *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/** Load the contents of this object from a .simplemap binary file (possibly compressed with gzip)
* \sa saveToFile
* \return false on any error.
*/
bool CSimpleMap::loadFromFile(const std::string &filName)
{
	try
	{
		mrpt::utils::CFileGZInputStream  f(filName);
		f >> *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}
