/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
