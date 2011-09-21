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

#include <mrpt/maps.h>  // Precompiled header

#include <mrpt/slam/CWeightedPointsMap.h>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CWeightedPointsMap, CPointsMap,mrpt::slam)


/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CWeightedPointsMap::CWeightedPointsMap()
{
	reserve( 400 );
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CWeightedPointsMap::~CWeightedPointsMap()
{
}

/*---------------------------------------------------------------
				reserve & resize methods
 ---------------------------------------------------------------*/
void CWeightedPointsMap::reserve(size_t newLength)
{
	x.reserve( newLength );
	y.reserve( newLength );
	z.reserve( newLength );
	pointWeight.reserve(newLength);
}

// Resizes all point buffers so they can hold the given number of points: newly created points are set to default values,
//  and old contents are not changed.
void CWeightedPointsMap::resize(size_t newLength)
{
	x.resize( newLength, 0 );
	y.resize( newLength, 0 );
	z.resize( newLength, 0 );
	pointWeight.resize(newLength, 1);
}

// Resizes all point buffers so they can hold the given number of points, *erasing* all previous contents
//  and leaving all points to default values.
void CWeightedPointsMap::setSize(size_t newLength)
{
	x.assign( newLength, 0);
	y.assign( newLength, 0);
	z.assign( newLength, 0);
	pointWeight.assign( newLength, 1 );
}

void  CWeightedPointsMap::setPointFast(size_t index,float x,float y,float z)
{
	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = z;
	// this->pointWeight: Unmodified
	// mark_as_modified(); -> Fast
}

void  CWeightedPointsMap::insertPointFast( float x, float y, float z )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
	this->pointWeight.push_back(1);
	// mark_as_modified(); -> Fast
}

/*---------------------------------------------------------------
						Copy constructor
  ---------------------------------------------------------------*/
void  CWeightedPointsMap::copyFrom(const CPointsMap &obj)
{
	CPointsMap::base_copyFrom(obj);  // This also does a ::resize(N) of all data fields.

	const CWeightedPointsMap *pW = dynamic_cast<const CWeightedPointsMap*>(&obj);
	if (pW)
	{
		pointWeight = pW->pointWeight;
	}
}

/*---------------------------------------------------------------
						addFrom_classSpecific
 ---------------------------------------------------------------*/
void  CWeightedPointsMap::addFrom_classSpecific(const CPointsMap &anotherMap, const size_t nPreviousPoints)
{
	const size_t nOther = anotherMap.size();

	// Specific data for this class:
	const CWeightedPointsMap * anotheMap_w = dynamic_cast<const CWeightedPointsMap *>(&anotherMap);

	if (anotheMap_w)
	{
		for (size_t i=0,j=nPreviousPoints;i<nOther;i++, j++)
			pointWeight[j] = anotheMap_w->pointWeight[i];
	}
}

/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CWeightedPointsMap::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t n = x.size();

		// First, write the number of points:
		out << n;

		if (n>0)
		{
			out.WriteBufferFixEndianness(&x[0],n);
			out.WriteBufferFixEndianness(&y[0],n);
			out.WriteBufferFixEndianness(&z[0],n);
			out.WriteBufferFixEndianness(&pointWeight[0],n);
		}

		// options saved too
		out	<< insertionOptions.minDistBetweenLaserPoints
			<< insertionOptions.addToExistingPointsMap
			<< insertionOptions.also_interpolate
			<< insertionOptions.disableDeletion
			<< insertionOptions.fuseWithExisting
			<< insertionOptions.isPlanarMap
			<< insertionOptions.maxDistForInterpolatePoints;

		// Insertion as 3D:
		out << m_disableSaveAs3DObject;
		out << insertionOptions.horizontalTolerance;

		likelihoodOptions.writeToStream(out);
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CWeightedPointsMap::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			mark_as_modified();

			// Read the number of points:
			uint32_t n;
			in >> n;

			x.resize(n);
			y.resize(n);
			z.resize(n);
			pointWeight.resize(n);


			if (n>0)
			{
				in.ReadBufferFixEndianness(&x[0],n);
				in.ReadBufferFixEndianness(&y[0],n);
				in.ReadBufferFixEndianness(&z[0],n);
				in.ReadBufferFixEndianness(&pointWeight[0],n);
			}

			in 	>> insertionOptions.minDistBetweenLaserPoints
				>> insertionOptions.addToExistingPointsMap
				>> insertionOptions.also_interpolate
				>> insertionOptions.disableDeletion
				>> insertionOptions.fuseWithExisting
				>> insertionOptions.isPlanarMap
				>> insertionOptions.maxDistForInterpolatePoints
				>> m_disableSaveAs3DObject;

			in >> insertionOptions.horizontalTolerance;

			likelihoodOptions.readFromStream(in);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					Clear
  ---------------------------------------------------------------*/
void  CWeightedPointsMap::internal_clear()
{
	// This swap() thing is the only way to really deallocate the memory.
	vector_strong_clear(x);
	vector_strong_clear(y);
	vector_strong_clear(z);
	vector_strong_clear(pointWeight);

	mark_as_modified();
}

namespace mrpt {
	namespace slam {
		namespace detail {
			using mrpt::slam::CWeightedPointsMap;

			template <> struct pointmap_traits<CWeightedPointsMap>
			{
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called only once before inserting points - this is the place to reserve memory in lric for extra working variables. */
				inline static void  internal_loadFromRangeScan2D_init(CWeightedPointsMap &me, mrpt::slam::CPointsMap::TLaserRange2DInsertContext & lric)  {  }
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data */
				inline static void  internal_loadFromRangeScan2D_prepareOneRange(CWeightedPointsMap &me, const float gx,const float gy, const float gz, mrpt::slam::CPointsMap::TLaserRange2DInsertContext & lric )  {  }
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called after each "{x,y,z}.push_back(...);" */
				inline static void  internal_loadFromRangeScan2D_postPushBack(CWeightedPointsMap &me, mrpt::slam::CPointsMap::TLaserRange2DInsertContext & lric)  { me.pointWeight.push_back(1); }

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called only once before inserting points - this is the place to reserve memory in lric for extra working variables. */
				inline static void  internal_loadFromRangeScan3D_init(CWeightedPointsMap &me, mrpt::slam::CPointsMap::TLaserRange3DInsertContext & lric) {  }
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data */
				inline static void  internal_loadFromRangeScan3D_prepareOneRange(CWeightedPointsMap &me, const float gx,const float gy, const float gz, mrpt::slam::CPointsMap::TLaserRange3DInsertContext & lric )  {  }
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called after each "{x,y,z}.push_back(...);" */
				inline static void  internal_loadFromRangeScan3D_postPushBack(CWeightedPointsMap &me, mrpt::slam::CPointsMap::TLaserRange3DInsertContext & lric)  { me.pointWeight.push_back(1); }
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data, at the end */
				inline static void  internal_loadFromRangeScan3D_postOneRange(CWeightedPointsMap &me, mrpt::slam::CPointsMap::TLaserRange3DInsertContext & lric )  {  }
			};
		}
	}
}


/** See CPointsMap::loadFromRangeScan() */
void  CWeightedPointsMap::loadFromRangeScan(
		const CObservation2DRangeScan &rangeScan,
		const CPose3D				  *robotPose)
{
	mrpt::slam::detail::loadFromRangeImpl<CWeightedPointsMap>::templ_loadFromRangeScan(*this,rangeScan,robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void  CWeightedPointsMap::loadFromRangeScan(
		const CObservation3DRangeScan &rangeScan,
		const CPose3D				  *robotPose)
{
	mrpt::slam::detail::loadFromRangeImpl<CWeightedPointsMap>::templ_loadFromRangeScan(*this,rangeScan,robotPose);
}


// ================================ PLY files import & export virtual methods ================================

/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
void CWeightedPointsMap::PLY_import_set_vertex_count(const size_t N)
{
	this->setSize(N);
}
