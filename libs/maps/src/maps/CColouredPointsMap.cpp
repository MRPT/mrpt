/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/utils/CStream.h>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;


//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CColouredPointsMap,colourPointsMap", mrpt::maps::CColouredPointsMap)

CColouredPointsMap::TMapDefinition::TMapDefinition()
{
}

void CColouredPointsMap::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
	likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix+string("_likelihoodOpts") );
	colourOpts.loadFromConfigFile(source, sectionNamePrefix+string("_colorOpts") );
}

void CColouredPointsMap::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
	this->colourOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CColouredPointsMap::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CColouredPointsMap::TMapDefinition &def = *dynamic_cast<const CColouredPointsMap::TMapDefinition*>(&_def);
	CColouredPointsMap *obj = new CColouredPointsMap();
	obj->insertionOptions  = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	obj->colorScheme = def.colourOpts;
	return obj;
}
//  =========== End of Map definition Block =========


IMPLEMENTS_SERIALIZABLE(CColouredPointsMap, CPointsMap,mrpt::maps)


#if MRPT_HAS_PCL
#   include <pcl/io/pcd_io.h>
#   include <pcl/point_types.h>
//#   include <pcl/registration/icp.h>
#endif

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CColouredPointsMap::CColouredPointsMap()
{
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CColouredPointsMap::~CColouredPointsMap()
{
}

/*---------------------------------------------------------------
				reserve & resize methods
 ---------------------------------------------------------------*/
void CColouredPointsMap::reserve(size_t newLength)
{
	newLength = mrpt::utils::length2length4N(newLength);

	x.reserve( newLength );
	y.reserve( newLength );
	z.reserve( newLength );
	m_color_R.reserve( newLength );
	m_color_G.reserve( newLength );
	m_color_B.reserve( newLength );
}

// Resizes all point buffers so they can hold the given number of points: newly created points are set to default values,
//  and old contents are not changed.
void CColouredPointsMap::resize(size_t newLength)
{
	this->reserve(newLength); // to ensure 4N capacity

	x.resize( newLength, 0 );
	y.resize( newLength, 0 );
	z.resize( newLength, 0 );
	m_color_R.resize( newLength, 1 );
	m_color_G.resize( newLength, 1 );
	m_color_B.resize( newLength, 1 );
	mark_as_modified();
}

// Resizes all point buffers so they can hold the given number of points, *erasing* all previous contents
//  and leaving all points to default values.
void CColouredPointsMap::setSize(size_t newLength)
{
	this->reserve(newLength); // to ensure 4N capacity

	x.assign( newLength, 0);
	y.assign( newLength, 0);
	z.assign( newLength, 0);
	m_color_R.assign( newLength, 1 );
	m_color_G.assign( newLength, 1 );
	m_color_B.assign( newLength, 1 );
	mark_as_modified();
}


/*---------------------------------------------------------------
						Copy constructor
  ---------------------------------------------------------------*/
void  CColouredPointsMap::copyFrom(const CPointsMap &obj)
{
	CPointsMap::base_copyFrom(obj);  // This also does a ::resize(N) of all data fields.

	const CColouredPointsMap *pCol = dynamic_cast<const CColouredPointsMap *>(&obj);
	if (pCol)
	{
		m_color_R = pCol->m_color_R;
		m_color_G = pCol->m_color_G;
		m_color_B = pCol->m_color_B;
		//m_min_dist = pCol->m_min_dist;
	}

}




/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CColouredPointsMap::writeToStream(mrpt::utils::CStream &out, int *version) const
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
		out << m_color_R << m_color_G << m_color_B; // added in v4

		out << genericMapParams; // v9
		insertionOptions.writeToStream(out); // version 9?: insert options are saved with its own method
		likelihoodOptions.writeToStream(out); // Added in version 5
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CColouredPointsMap::readFromStream(mrpt::utils::CStream &in, int version)
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
			in >> m_color_R >> m_color_G >> m_color_B;

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

			if (version>=4)  // Color data
			{
				in >> m_color_R >> m_color_G >> m_color_B;
				if (version>=7)
				{
					// Removed: in >> m_min_dist;
				}
				else
				{
					std::vector<float>	dummy_dist;
					in >> dummy_dist;
				}
			}
			else
			{
				m_color_R.assign(x.size(),1.0f);
				m_color_G.assign(x.size(),1.0f);
				m_color_B.assign(x.size(),1.0f);
				//m_min_dist.assign(x.size(),2000.0f);
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
void  CColouredPointsMap::internal_clear()
{
	// This swap() thing is the only way to really deallocate the memory.
	vector_strong_clear(x);
	vector_strong_clear(y);
	vector_strong_clear(z);

	vector_strong_clear(m_color_R);
	vector_strong_clear(m_color_G);
	vector_strong_clear(m_color_B);

	mark_as_modified();
}


/** Changes a given point from map. First index is 0.
 * \exception Throws std::exception on index out of bound.
 */
void  CColouredPointsMap::setPoint(size_t index,float x, float y, float z, float R, float G, float B)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");
	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = z;
	this->m_color_R[index]=R;
	this->m_color_G[index]=G;
	this->m_color_B[index]=B;
	mark_as_modified();
}

/** Changes just the color of a given point from the map. First index is 0.
 * \exception Throws std::exception on index out of bound.
 */
void  CColouredPointsMap::setPointColor(size_t index,float R, float G, float B)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");
	this->m_color_R[index]=R;
	this->m_color_G[index]=G;
	this->m_color_B[index]=B;
	// mark_as_modified();  // No need to rebuild KD-trees, etc...
}

void  CColouredPointsMap::insertPointFast( float x, float y, float z )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
	m_color_R.push_back(1);
	m_color_G.push_back(1);
	m_color_B.push_back(1);

	// mark_as_modified(); -> Fast
}

void  CColouredPointsMap::insertPoint( float x, float y, float z, float R, float G, float B )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
	m_color_R.push_back(R);
	m_color_G.push_back(G);
	m_color_B.push_back(B);

	mark_as_modified();
}

/*---------------------------------------------------------------
					getAs3DObject
 ---------------------------------------------------------------*/
void CColouredPointsMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	ASSERT_(outObj);

	if (!genericMapParams.enableSaveAs3DObject) return;

	opengl::CPointCloudColouredPtr  obj = opengl::CPointCloudColoured::Create();

	obj->loadFromPointsMap(this);
	obj->setColor(1,1,1,  1.0);

	obj->setPointSize( mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE );

	outObj->insert(obj);
}

/*---------------------------------------------------------------
					TColourOptions
 ---------------------------------------------------------------*/
CColouredPointsMap::TColourOptions::TColourOptions() :
	scheme(cmFromHeightRelativeToSensor),
	z_min(-10),
	z_max(10),
	d_max(5)
{
}

/*---------------------------------------------------------------
					TColourOptions
 ---------------------------------------------------------------*/
void CColouredPointsMap::TColourOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &source,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_CAST( scheme, int, TColouringMethod,   source,section )
	MRPT_LOAD_CONFIG_VAR(z_min, float,		source,section)
	MRPT_LOAD_CONFIG_VAR(z_max, float,		source,section)
	MRPT_LOAD_CONFIG_VAR(d_max, float,		source,section)
}

/*---------------------------------------------------------------
					TColourOptions
 ---------------------------------------------------------------*/
void  CColouredPointsMap::TColourOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CColouredPointsMap::TColourOptions] ------------ \n\n");

	out.printf("scheme                                  = %u\n",	(unsigned)scheme);
	out.printf("z_min                                   = %f\n",	z_min );
	out.printf("z_max                                   = %f\n",	z_max );
	out.printf("d_max                                   = %f\n",	d_max );
}

/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
unsigned long CColouredPointsMap::getPoint( size_t index, float &x, float &y, float &z) const
{
	if (index >= this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	return 1; // Weight
}

/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
void  CColouredPointsMap::getPoint( size_t index, float &x, float &y, float &z, float &R, float &G, float &B ) const
{
	if (index >= this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	R = m_color_R[index];
	G = m_color_G[index];
	B = m_color_B[index];
}

/** Retrieves a point color (colors range is [0,1])
  */
void  CColouredPointsMap::getPointColor( size_t index, float &R, float &G, float &B ) const
{
	if (index >= this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	R = m_color_R[index];
	G = m_color_G[index];
	B = m_color_B[index];
}

// This function is duplicated from "mrpt::vision::pinhole::projectPoint_with_distortion", to avoid
//  a dependency on mrpt-vision.
void aux_projectPoint_with_distortion(
	const mrpt::math::TPoint3D  &P,
	const mrpt::utils::TCamera  &params,
	mrpt::utils::TPixelCoordf  &pixel,
	bool accept_points_behind
	)
{
	MRPT_UNUSED_PARAM(accept_points_behind);
	// Pinhole model:
	const double x = P.x/P.z;
	const double y = P.y/P.z;

	// Radial distortion:
	const double r2 = square(x)+square(y);
	const double r4 = square(r2);
	const double r6 = r2*r4;

	pixel.x = params.cx() + params.fx() *(  x*(1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6) + 2*params.dist[2]*x*y+params.dist[3]*(r2+2*square(x))  );
	pixel.y = params.cy() + params.fy() *(  y*(1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6) + 2*params.dist[3]*x*y+params.dist[2]*(r2+2*square(y))  );
}


/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
bool CColouredPointsMap::colourFromObservation( const CObservationImage &obs, const CPose3D &robotPose )
{
	// Check if image is not grayscale
	ASSERT_( obs.image.isColor() );

	CPose3D cameraPoseR;					// Get Camera Pose on the Robot(B) (CPose3D)
	CPose3D cameraPoseW;					// World Camera Pose

	obs.getSensorPose( cameraPoseR );
	cameraPoseW = robotPose + cameraPoseR;	// Camera Global Coordinates

	// Image Information
	unsigned int imgW = obs.image.getWidth();
	unsigned int imgH = obs.image.getHeight();

	// Projection related variables
	std::vector<TPixelCoordf>			projectedPoints;	// The set of projected points in the image
	std::vector<TPixelCoordf>::iterator itProPoints;		// Iterator for projectedPoints
	std::vector<size_t>					p_idx;
	std::vector<float>					p_dist;
	std::vector<unsigned int>			p_proj;

	// Get the N closest points
	kdTreeNClosestPoint2DIdx( cameraPoseW.x(), cameraPoseW.y(),					// query point
							   200000,										// number of points to search
							   p_idx, p_dist );								// indexes and distances of the returned points

	// Fill p3D vector
	for( size_t k = 0; k < p_idx.size(); k++ )
	{
		float d = sqrt(p_dist[k]);
		size_t idx = p_idx[k];
		if( d < colorScheme.d_max ) //  && d < m_min_dist[idx] )
		{
			TPixelCoordf px;
			aux_projectPoint_with_distortion(TPoint3D( x[idx], y[idx], z[idx] ),  obs.cameraParams, px, true);
			projectedPoints.push_back(px);
			p_proj.push_back(k);
		} // end if
	} // end for

	// Get channel order
	unsigned int chR,chG,chB;
	if( obs.image.getChannelsOrder()[0] == 'B' ) { chR = 2; chG = 1;	chB = 0; }
	else { chR = 0; chG = 1; chB = 2; }

	unsigned int n_proj = 0;
	const float factor = 1.0f/255;	// Normalize pixels:

	// Get the colour of the projected points
	size_t k;
	for( itProPoints = projectedPoints.begin(), k = 0;
		 itProPoints != projectedPoints.end();
		 ++itProPoints, ++k )
	{
		// Only get the points projected inside the image
		if( itProPoints->x >= 0 && itProPoints->x < imgW && itProPoints->y > 0 && itProPoints->y < imgH )
		{
			unsigned int ii = p_idx[p_proj[k]];
			uint8_t * p = obs.image( (unsigned int)itProPoints->x, (unsigned int)itProPoints->y );

			m_color_R[ii]	= p[chR]*factor; // R
			m_color_G[ii]	= p[chG]*factor; // G
			m_color_B[ii]	= p[chB]*factor; // B
			// m_min_dist[ii]	= p_dist[p_proj[k]];

			n_proj++;
		}
	} // end for

	return true;
} // end colourFromObservation


void CColouredPointsMap::resetPointsMinDist( float defValue )
{
	MRPT_UNUSED_PARAM(defValue);
	// m_min_dist.assign(x.size(),defValue);
}


bool  CColouredPointsMap::save3D_and_colour_to_text_file(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return false;

	for (unsigned int i=0;i<x.size();i++)
		os::fprintf(f,"%f %f %f %d %d %d\n",x[i],y[i],z[i],(uint8_t)(255*m_color_R[i]),(uint8_t)(255*m_color_G[i]),(uint8_t)(255*m_color_B[i]));
	//	os::fprintf(f,"%f %f %f %f %f %f %f\n",x[i],y[i],z[i],m_color_R[i],m_color_G[i],m_color_B[i],m_min_dist[i]);

	os::fclose(f);
	return true;

}

// ================================ PLY files import & export virtual methods ================================

/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
void CColouredPointsMap::PLY_import_set_vertex_count(const size_t N)
{
	this->setSize(N);
}

/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CColouredPointsMap::PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color)
{
	if (pt_color)
			this->setPoint(idx,pt.x,pt.y,pt.z,pt_color->R,pt_color->G,pt_color->B);
	else	this->setPoint(idx,pt.x,pt.y,pt.z);
}

/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point.
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CColouredPointsMap::PLY_export_get_vertex(
	const size_t idx,
	mrpt::math::TPoint3Df &pt,
	bool &pt_has_color,
	mrpt::utils::TColorf &pt_color) const
{
	pt_has_color=true;

	pt.x = x[idx];
	pt.y = y[idx];
	pt.z = z[idx];

	pt_color.R = m_color_R[idx];
	pt_color.G = m_color_G[idx];
	pt_color.B = m_color_B[idx];
}


/*---------------------------------------------------------------
						addFrom_classSpecific
 ---------------------------------------------------------------*/
void  CColouredPointsMap::addFrom_classSpecific(const CPointsMap &anotherMap, const size_t nPreviousPoints)
{
	const size_t nOther = anotherMap.size();

	// Specific data for this class:
	const CColouredPointsMap * anotheMap_col = dynamic_cast<const CColouredPointsMap *>(&anotherMap);

	if (anotheMap_col)
	{
		for (size_t i=0,j=nPreviousPoints;i<nOther;i++, j++)
		{
			m_color_R[j] = anotheMap_col->m_color_R[i];
			m_color_G[j] = anotheMap_col->m_color_G[i];
			m_color_B[j] = anotheMap_col->m_color_B[i];
		}
	}
}

/** Save the point cloud as a PCL PCD file, in either ASCII or binary format \return false on any error */
bool CColouredPointsMap::savePCDFile(const std::string &filename, bool save_as_binary) const
{
#if MRPT_HAS_PCL
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	const size_t nThis = this->size();

	// Fill in the cloud data
	cloud.width    = nThis;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	const float f = 255.f;

	union myaux_t
	{
		uint8_t rgb[4];
		float   f;
	} aux_val;

	for (size_t i = 0; i < nThis; ++i)
	{
		cloud.points[i].x =this->x[i];
		cloud.points[i].y =this->y[i];
		cloud.points[i].z =this->z[i];

		aux_val.rgb[0]=static_cast<uint8_t>( this->m_color_B[i] * f);
		aux_val.rgb[1]=static_cast<uint8_t>( this->m_color_G[i] * f);
		aux_val.rgb[2]=static_cast<uint8_t>( this->m_color_R[i] * f);

		cloud.points[i].rgb = aux_val.f;
	}

	return 0 == pcl::io::savePCDFile(filename, cloud, save_as_binary);

#else
	MRPT_UNUSED_PARAM(filename); MRPT_UNUSED_PARAM(save_as_binary);
	THROW_EXCEPTION("Operation not available: MRPT was built without PCL")
#endif
}

namespace mrpt {
	namespace maps {
		namespace detail {
			using mrpt::maps::CColouredPointsMap;

			template <> struct pointmap_traits<CColouredPointsMap>
			{
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called only once before inserting points - this is the place to reserve memory in lric for extra working variables. */
				inline static void  internal_loadFromRangeScan2D_init(CColouredPointsMap &me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext & lric)
				{
					// Vars:
					//  [0] -> pR
					//  [1] -> pG
					//  [2] -> pB
					//  [3] -> Az_1_color
					lric.fVars.resize(4);

					ASSERT_NOT_EQUAL_(me.colorScheme.z_max,me.colorScheme.z_min)
					lric.fVars[3] = 1.0/(me.colorScheme.z_max-me.colorScheme.z_min);
				}

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data */
				inline static void  internal_loadFromRangeScan2D_prepareOneRange(CColouredPointsMap &me, const float gx,const float gy, const float gz, mrpt::maps::CPointsMap::TLaserRange2DInsertContext & lric )
				{
					MRPT_UNUSED_PARAM(gx); MRPT_UNUSED_PARAM(gy);
					// Relative height of the point wrt the sensor:
					const float rel_z = gz - lric.HM.get_unsafe(2,3); // m23;

					// Variable renaming:
					float & pR = lric.fVars[0];
					float & pG = lric.fVars[1];
					float & pB = lric.fVars[2];
					const float &Az_1_color = lric.fVars[3];

					// Compute color:
					switch( me.colorScheme.scheme )
					{
					//case cmFromHeightRelativeToSensor:
					case CColouredPointsMap::cmFromHeightRelativeToSensorJet:
					case CColouredPointsMap::cmFromHeightRelativeToSensorGray:
						{
							float q = (rel_z-me.colorScheme.z_min)*Az_1_color;
							if (q<0) q=0; else if (q>1) q=1;

							if (me.colorScheme.scheme==CColouredPointsMap::cmFromHeightRelativeToSensorGray)
							{
								pR=pG=pB=q;
							}
							else
							{
								mrpt::utils::jet2rgb(q, pR,pG,pB);
							}
						}
						break;
					case CColouredPointsMap::cmFromIntensityImage:
						{
							// In 2D scans we don't have this channel.
							pR=pG=pB=1.0;
						}
						break;
					default:
						THROW_EXCEPTION("Unknown color scheme");
					}

				}
				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called after each "{x,y,z}.push_back(...);" */
				inline static void  internal_loadFromRangeScan2D_postPushBack(CColouredPointsMap &me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext & lric)
				{
					float & pR = lric.fVars[0];
					float & pG = lric.fVars[1];
					float & pB = lric.fVars[2];
					me.m_color_R.push_back(pR);
					me.m_color_G.push_back(pG);
					me.m_color_B.push_back(pB);
					//m_min_dist.push_back(1e4);

				}

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called only once before inserting points - this is the place to reserve memory in lric for extra working variables. */
				inline static void  internal_loadFromRangeScan3D_init(CColouredPointsMap &me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric)
				{
					// Vars:
					//  [0] -> pR
					//  [1] -> pG
					//  [2] -> pB
					//  [3] -> Az_1_color
					//  [4] -> K_8u
					//  [5] -> cx
					//  [6] -> cy
					//  [7] -> fx
					//  [8] -> fy
					lric.fVars.resize(9);
					float &cx = lric.fVars[5];
					float &cy = lric.fVars[6];
					float &fx = lric.fVars[7];
					float &fy = lric.fVars[8];

					// unsigned int vars:
					//  [0] -> imgW
					//  [1] -> imgH
					//  [2] -> img_idx_x
					//  [3] -> img_idx_y
					lric.uVars.resize(4);
					unsigned int & imgW = lric.uVars[0];
					unsigned int & imgH = lric.uVars[1];
					unsigned int & img_idx_x = lric.uVars[2];
					unsigned int & img_idx_y = lric.uVars[3];

					// bool vars:
					//  [0] -> hasValidIntensityImage
					//  [1] -> hasColorIntensityImg
					//  [2] -> simple_3d_to_color_relation
					lric.bVars.resize(3);
					uint8_t & hasValidIntensityImage = lric.bVars[0];
					uint8_t & hasColorIntensityImg   = lric.bVars[1];
					uint8_t & simple_3d_to_color_relation = lric.bVars[2];

					ASSERT_NOT_EQUAL_(me.colorScheme.z_max,me.colorScheme.z_min)
					lric.fVars[3] = 1.0/(me.colorScheme.z_max-me.colorScheme.z_min);  // Az_1_color = ...
					lric.fVars[4] = 1.0f/255; // K_8u

					hasValidIntensityImage = false;
					imgW=0;
					imgH=0;

					if (lric.rangeScan.hasIntensityImage)
					{
						// assure the size matches:
						if (lric.rangeScan.points3D_x.size() == lric.rangeScan.intensityImage.getWidth() * lric.rangeScan.intensityImage.getHeight() )
						{
							hasValidIntensityImage = true;
							imgW = lric.rangeScan.intensityImage.getWidth();
							imgH = lric.rangeScan.intensityImage.getHeight();
						}
					}


					hasColorIntensityImg = hasValidIntensityImage && lric.rangeScan.intensityImage.isColor();

					// running indices for the image pixels for the gray levels:
					// If both range & intensity images coincide (e.g. SwissRanger), then we can just
					// assign 3D points to image pixels one-to-one, but that's not the case if
					//
					simple_3d_to_color_relation = (std::abs(lric.rangeScan.relativePoseIntensityWRTDepth.norm())<1e-5);
					img_idx_x = 0;
					img_idx_y = 0;

					// Will be used just if simple_3d_to_color_relation=false
					cx = lric.rangeScan.cameraParamsIntensity.cx();
					cy = lric.rangeScan.cameraParamsIntensity.cy();
					fx = lric.rangeScan.cameraParamsIntensity.fx();
					fy = lric.rangeScan.cameraParamsIntensity.fy();

				}

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data */
				inline static void  internal_loadFromRangeScan3D_prepareOneRange(CColouredPointsMap &me, const float gx,const float gy, const float gz, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric )
				{
					MRPT_UNUSED_PARAM(gx); MRPT_UNUSED_PARAM(gy);
					// Rename variables:
					float &pR = lric.fVars[0];
					float &pG = lric.fVars[1];
					float &pB = lric.fVars[2];
					float &Az_1_color = lric.fVars[3];
					float &K_8u = lric.fVars[4];
					float &cx = lric.fVars[5];
					float &cy = lric.fVars[6];
					float &fx = lric.fVars[7];
					float &fy = lric.fVars[8];

					unsigned int & imgW = lric.uVars[0];
					unsigned int & imgH = lric.uVars[1];
					unsigned int & img_idx_x = lric.uVars[2];
					unsigned int & img_idx_y = lric.uVars[3];

					const uint8_t & hasValidIntensityImage = lric.bVars[0];
					const uint8_t & hasColorIntensityImg   = lric.bVars[1];
					const uint8_t & simple_3d_to_color_relation = lric.bVars[2];

					// Relative height of the point wrt the sensor:
					const float rel_z = gz - lric.HM.get_unsafe(2,3); // m23;

					// Compute color:
					switch( me.colorScheme.scheme )
					{
					//case cmFromHeightRelativeToSensor:
					case CColouredPointsMap::cmFromHeightRelativeToSensorJet:
					case CColouredPointsMap::cmFromHeightRelativeToSensorGray:
						{
							float q = (rel_z-me.colorScheme.z_min)*Az_1_color;
							if (q<0) q=0; else if (q>1) q=1;

							if (me.colorScheme.scheme==CColouredPointsMap::cmFromHeightRelativeToSensorGray)
							{
								pR=pG=pB=q;
							}
							else
							{
								mrpt::utils::jet2rgb(q, pR,pG,pB);
							}
						}
						break;
					case CColouredPointsMap::cmFromIntensityImage:
						{
							// Do we have to project the 3D point into the image plane??
							bool hasValidColor = false;
							if (simple_3d_to_color_relation)
							{
								hasValidColor=true;
							}
							else
							{
								// Do projection:
								TPoint3D  pt; // pt_wrt_colorcam;
								lric.rangeScan.relativePoseIntensityWRTDepth.inverseComposePoint(
									lric.scan_x,lric.scan_y,lric.scan_z,
									pt.x,pt.y,pt.z);

								// Project to image plane:
								if (pt.z)
								{
									img_idx_x = cx + fx * pt.x/pt.z;
									img_idx_y = cy + fy * pt.y/pt.z;

									hasValidColor=
										img_idx_x<imgW &&   // img_idx_x>=0  isn't needed for unsigned.
										img_idx_y<imgH;
								}
							}

							if (hasValidColor && hasColorIntensityImg)
							{
								const uint8_t *c= lric.rangeScan.intensityImage.get_unsafe(img_idx_x, img_idx_y, 0);
								pR= c[2] * K_8u;
								pG= c[1] * K_8u;
								pB= c[0] * K_8u;
							}
							else
							if (hasValidColor && hasValidIntensityImage)
							{
								uint8_t c= *lric.rangeScan.intensityImage.get_unsafe(img_idx_x, img_idx_y, 0);
								pR=pG=pB= c * K_8u;
							}
							else
							{
								pR=pG=pB=1.0;
							}
						}
						break;
					default:
						THROW_EXCEPTION("Unknown color scheme");
					}

				}

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called after each "{x,y,z}.push_back(...);" */
				inline static void  internal_loadFromRangeScan3D_postPushBack(CColouredPointsMap &me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric)
				{
					float &pR = lric.fVars[0];
					float &pG = lric.fVars[1];
					float &pB = lric.fVars[2];

					me.m_color_R.push_back(pR);
					me.m_color_G.push_back(pG);
					me.m_color_B.push_back(pB);

					//m_min_dist.push_back(1e4);

				}

				/** Helper method fot the generic implementation of CPointsMap::loadFromRangeScan(), to be called once per range data, at the end */
				inline static void  internal_loadFromRangeScan3D_postOneRange(CColouredPointsMap &me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext & lric )
				{
					MRPT_UNUSED_PARAM(me);
					unsigned int & imgW = lric.uVars[0];
					unsigned int & img_idx_x = lric.uVars[2];
					unsigned int & img_idx_y = lric.uVars[3];

					const uint8_t & hasValidIntensityImage = lric.bVars[0];
					const uint8_t & simple_3d_to_color_relation = lric.bVars[2];

					// Advance the color pointer (just for simple cases, e.g. SwissRanger, not for Kinect)
					if (simple_3d_to_color_relation && hasValidIntensityImage)
					{
						if (++img_idx_x>=imgW)
						{
							img_idx_y++;
							img_idx_x=0;
						}
					}
				}
			};
		}
	}
}



/** See CPointsMap::loadFromRangeScan() */
void  CColouredPointsMap::loadFromRangeScan(
		const CObservation2DRangeScan &rangeScan,
		const CPose3D				  *robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CColouredPointsMap>::templ_loadFromRangeScan(*this,rangeScan,robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void  CColouredPointsMap::loadFromRangeScan(
		const CObservation3DRangeScan &rangeScan,
		const CPose3D				  *robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CColouredPointsMap>::templ_loadFromRangeScan(*this,rangeScan,robotPose);
}

