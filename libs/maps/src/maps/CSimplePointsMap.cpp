/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CStream.h>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CSimplePointsMap,pointsMap", mrpt::maps::CSimplePointsMap)

CSimplePointsMap::TMapDefinition::TMapDefinition()
{
}

void CSimplePointsMap::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
	likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix+string("_likelihoodOpts") );
}

void CSimplePointsMap::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CSimplePointsMap::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CSimplePointsMap::TMapDefinition &def = *dynamic_cast<const CSimplePointsMap::TMapDefinition*>(&_def);
	CSimplePointsMap *obj = new CSimplePointsMap();
	obj->insertionOptions  = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========


IMPLEMENTS_SERIALIZABLE(CSimplePointsMap, CPointsMap,mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CSimplePointsMap::CSimplePointsMap()
{
	reserve( 400 );
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CSimplePointsMap::~CSimplePointsMap()
{
}

/*---------------------------------------------------------------
				reserve & resize methods
 ---------------------------------------------------------------*/
void CSimplePointsMap::reserve(size_t newLength)
{
	newLength = mrpt::utils::length2length4N(newLength);

	x.reserve( newLength );
	y.reserve( newLength );
	z.reserve( newLength );
}

// Resizes all point buffers so they can hold the given number of points: newly created points are set to default values,
//  and old contents are not changed.
void CSimplePointsMap::resize(size_t newLength)
{
	this->reserve(newLength); // to ensure 4N capacity
	x.resize( newLength, 0 );
	y.resize( newLength, 0 );
	z.resize( newLength, 0 );
	mark_as_modified();
}

// Resizes all point buffers so they can hold the given number of points, *erasing* all previous contents
//  and leaving all points to default values.
void CSimplePointsMap::setSize(size_t newLength)
{
	this->reserve(newLength); // to ensure 4N capacity
	x.assign( newLength, 0);
	y.assign( newLength, 0);
	z.assign( newLength, 0);
	mark_as_modified();
}


/*---------------------------------------------------------------
						Copy constructor
  ---------------------------------------------------------------*/
void  CSimplePointsMap::copyFrom(const CPointsMap &obj)
{
	CPointsMap::base_copyFrom(obj);  // This also does a ::resize(N) of all data fields.
}


/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSimplePointsMap::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 9;
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
		}
		out << genericMapParams;  // v9

		insertionOptions.writeToStream(out); // version 9: insert options are saved with its own method:
		likelihoodOptions.writeToStream(out); // Added in version 5:
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CSimplePointsMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 8:
	case 9:
		{
			mark_as_modified();

			// Read the number of points:
			uint32_t n;
			in >> n;

			this->resize(n);

			if (n>0)
			{
				in.ReadBufferFixEndianness(&x[0],n);
				in.ReadBufferFixEndianness(&y[0],n);
				in.ReadBufferFixEndianness(&z[0],n);
			}
			if (version>=9)
				in >> genericMapParams;
			else 
			{
				bool disableSaveAs3DObject;
				in >> disableSaveAs3DObject;
				genericMapParams.enableSaveAs3DObject = !disableSaveAs3DObject;
			}

			insertionOptions.readFromStream(in);
			likelihoodOptions.readFromStream(in);
		} break;

	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		{
			mark_as_modified();

			// Read the number of points:
			uint32_t n;
			in >> n;

			this->resize(n);

			if (n>0)
			{
				in.ReadBufferFixEndianness(&x[0],n);
				in.ReadBufferFixEndianness(&y[0],n);
				in.ReadBufferFixEndianness(&z[0],n);

				// Version 1: weights are also stored:
				// Version 4: Type becomes long int -> uint32_t for portability!!
				if (version>=1)
				{
					if (version>=4)
					{
						if (version>=7)
						{
							// Weights were removed from this class in v7 (MRPT 0.9.5),
							//  so nothing else to do.
						}
						else
						{
							// Go on with old serialization format, but discard weights:
							std::vector<uint32_t>  dummy_pointWeight(n);
							in.ReadBufferFixEndianness(&dummy_pointWeight[0],n);
						}
					}
					else
					{
						std::vector<uint32_t>  dummy_pointWeight(n);
						in.ReadBufferFixEndianness(&dummy_pointWeight[0],n);
					}
				}
			}

			if (version>=2)
			{
				// version 2: options saved too
				in 	>> insertionOptions.minDistBetweenLaserPoints
					>> insertionOptions.addToExistingPointsMap
					>> insertionOptions.also_interpolate
					>> insertionOptions.disableDeletion
					>> insertionOptions.fuseWithExisting
					>> insertionOptions.isPlanarMap;

				if (version<6)
				{
					bool old_matchStaticPointsOnly;
					in >> old_matchStaticPointsOnly;
				}

				in >> insertionOptions.maxDistForInterpolatePoints;

				{
					bool disableSaveAs3DObject;
					in >> disableSaveAs3DObject;
					genericMapParams.enableSaveAs3DObject = !disableSaveAs3DObject;
				}
			}

			if (version>=3)
			{
				in >> insertionOptions.horizontalTolerance;
			}

			if (version>=5) // version 5: added likelihoodOptions
				likelihoodOptions.readFromStream(in);

			if (version>=8) // version 8: added insertInvalidPoints
				in >> insertionOptions.insertInvalidPoints;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					Clear
  ---------------------------------------------------------------*/
void  CSimplePointsMap::internal_clear()
{
	// This swap() thing is the only way to really deallocate the memory.
	vector_strong_clear(x);
	vector_strong_clear(y);
	vector_strong_clear(z);

	mark_as_modified();
}

void  CSimplePointsMap::setPointFast(size_t index,float x,float y,float z)
{
	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = z;
}

void  CSimplePointsMap::insertPointFast( float x, float y, float z )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
}


namespace mrpt {
	namespace maps {
		namespace detail {
			using mrpt::maps::CSimplePointsMap;

			template <> struct pointmap_traits<CSimplePointsMap>
			{

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called only once before inserting points - this is the place to reserve memory in lric for extra working variables. */
				inline static void  internal_loadFromRangeScan2D_init(CSimplePointsMap &me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext & lric) {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(lric);
				}
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data */
				inline static void  internal_loadFromRangeScan2D_prepareOneRange(CSimplePointsMap &me, const float gx,const float gy, const float gz, mrpt::maps::CPointsMap::TLaserRange2DInsertContext & lric ) {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(gx);
					MRPT_UNUSED_PARAM(gy);
					MRPT_UNUSED_PARAM(gz);
					MRPT_UNUSED_PARAM(lric);
				}
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called after each "{x,y,z}.push_back(...);" */
				inline static void  internal_loadFromRangeScan2D_postPushBack(CSimplePointsMap &me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext & lric)  {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(lric);
				}

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called only once before inserting points - this is the place to reserve memory in lric for extra working variables. */
				inline static void  internal_loadFromRangeScan3D_init(CSimplePointsMap &me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric) {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(lric);
				}
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data */
				inline static void  internal_loadFromRangeScan3D_prepareOneRange(CSimplePointsMap &me, const float gx,const float gy, const float gz, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric )  {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(gx);
					MRPT_UNUSED_PARAM(gy);
					MRPT_UNUSED_PARAM(gz);
					MRPT_UNUSED_PARAM(lric);
				}
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called after each "{x,y,z}.push_back(...);" */
				inline static void  internal_loadFromRangeScan3D_postPushBack(CSimplePointsMap &me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric)  {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(lric);
				}
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data, at the end */
				inline static void  internal_loadFromRangeScan3D_postOneRange(CSimplePointsMap &me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric )  {
					MRPT_UNUSED_PARAM(me);
					MRPT_UNUSED_PARAM(lric);
				}
			};
		}
	}
}

/** See CPointsMap::loadFromRangeScan() */
void  CSimplePointsMap::loadFromRangeScan(
		const CObservation2DRangeScan &rangeScan,
		const CPose3D				  *robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CSimplePointsMap>::templ_loadFromRangeScan(*this,rangeScan,robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void  CSimplePointsMap::loadFromRangeScan(
		const CObservation3DRangeScan &rangeScan,
		const CPose3D				  *robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CSimplePointsMap>::templ_loadFromRangeScan(*this,rangeScan,robotPose);
}

// ================================ PLY files import & export virtual methods ================================

/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
void CSimplePointsMap::PLY_import_set_vertex_count(const size_t N)
{
	this->setSize(N);
}

