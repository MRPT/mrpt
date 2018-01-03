/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/CArchive.h>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CColouredPointsMap,colourPointsMap", mrpt::maps::CColouredPointsMap)

CColouredPointsMap::TMapDefinition::TMapDefinition() {}
void CColouredPointsMap::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
	colourOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_colorOpts"));
}

void CColouredPointsMap::TMapDefinition::dumpToTextStream_map_specific(
	mrpt::utils::CStream& out) const
{
	this->insertionOpts.dumpToTextStream(std::ostream& out) const
{
	out.printf(
		"\n----------- [CColouredPointsMap::TColourOptions] ------------ \n\n");

	out.printf(
		"scheme                                  = %u\n", (unsigned)scheme);
	out.printf("z_min                                   = %f\n", z_min);
	out.printf("z_max                                   = %f\n", z_max);
	out.printf("d_max                                   = %f\n", d_max);
}

/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
unsigned long CColouredPointsMap::getPoint(
	size_t index, float& x, float& y, float& z) const
{
	if (index >= this->x.size()) THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	return 1;  // Weight
}

/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
void CColouredPointsMap::getPoint(
	size_t index, float& x, float& y, float& z, float& R, float& G,
	float& B) const
{
	if (index >= this->x.size()) THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	R = m_color_R[index];
	G = m_color_G[index];
	B = m_color_B[index];
}

/** Retrieves a point color (colors range is [0,1])
  */
void CColouredPointsMap::getPointColor(
	size_t index, float& R, float& G, float& B) const
{
	if (index >= this->x.size()) THROW_EXCEPTION("Index out of bounds");

	R = m_color_R[index];
	G = m_color_G[index];
	B = m_color_B[index];
}

// This function is duplicated from
// "mrpt::vision::pinhole::projectPoint_with_distortion", to avoid
//  a dependency on mrpt-vision.
void aux_projectPoint_with_distortion(
	const mrpt::math::TPoint3D& P, const mrpt::img::TCamera& params,
	mrpt::img::TPixelCoordf& pixel, bool accept_points_behind)
{
	MRPT_UNUSED_PARAM(accept_points_behind);
	// Pinhole model:
	const double x = P.x / P.z;
	const double y = P.y / P.z;

	// Radial distortion:
	const double r2 = square(x) + square(y);
	const double r4 = square(r2);
	const double r6 = r2 * r4;

	pixel.x = params.cx() +
			  params.fx() * (x * (1 + params.dist[0] * r2 +
								  params.dist[1] * r4 + params.dist[4] * r6) +
							 2 * params.dist[2] * x * y +
							 params.dist[3] * (r2 + 2 * square(x)));
	pixel.y = params.cy() +
			  params.fy() * (y * (1 + params.dist[0] * r2 +
								  params.dist[1] * r4 + params.dist[4] * r6) +
							 2 * params.dist[3] * x * y +
							 params.dist[2] * (r2 + 2 * square(y)));
}

/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
bool CColouredPointsMap::colourFromObservation(
	const CObservationImage& obs, const CPose3D& robotPose)
{
	// Check if image is not grayscale
	ASSERT_(obs.image.isColor());

	CPose3D cameraPoseR;  // Get Camera Pose on the Robot(B) (CPose3D)
	CPose3D cameraPoseW;  // World Camera Pose

	obs.getSensorPose(cameraPoseR);
	cameraPoseW = robotPose + cameraPoseR;  // Camera Global Coordinates

	// Image Information
	unsigned int imgW = obs.image.getWidth();
	unsigned int imgH = obs.image.getHeight();

	// Projection related variables
	std::vector<TPixelCoordf>
		projectedPoints;  // The set of projected points in the image
	std::vector<TPixelCoordf>::iterator
		itProPoints;  // Iterator for projectedPoints
	std::vector<size_t> p_idx;
	std::vector<float> p_dist;
	std::vector<unsigned int> p_proj;

	// Get the N closest points
	kdTreeNClosestPoint2DIdx(
		cameraPoseW.x(), cameraPoseW.y(),  // query point
		200000,  // number of points to search
		p_idx, p_dist);  // indexes and distances of the returned points

	// Fill p3D vector
	for (size_t k = 0; k < p_idx.size(); k++)
	{
		float d = sqrt(p_dist[k]);
		size_t idx = p_idx[k];
		if (d < colorScheme.d_max)  //  && d < m_min_dist[idx] )
		{
			TPixelCoordf px;
			aux_projectPoint_with_distortion(
				TPoint3D(x[idx], y[idx], z[idx]), obs.cameraParams, px, true);
			projectedPoints.push_back(px);
			p_proj.push_back(k);
		}  // end if
	}  // end for

	// Get channel order
	unsigned int chR, chG, chB;
	if (obs.image.getChannelsOrder()[0] == 'B')
	{
		chR = 2;
		chG = 1;
		chB = 0;
	}
	else
	{
		chR = 0;
		chG = 1;
		chB = 2;
	}

	unsigned int n_proj = 0;
	const float factor = 1.0f / 255;  // Normalize pixels:

	// Get the colour of the projected points
	size_t k;
	for (itProPoints = projectedPoints.begin(), k = 0;
		 itProPoints != projectedPoints.end(); ++itProPoints, ++k)
	{
		// Only get the points projected inside the image
		if (itProPoints->x >= 0 && itProPoints->x < imgW &&
			itProPoints->y > 0 && itProPoints->y < imgH)
		{
			unsigned int ii = p_idx[p_proj[k]];
			uint8_t* p = obs.image(
				(unsigned int)itProPoints->x, (unsigned int)itProPoints->y);

			m_color_R[ii] = p[chR] * factor;  // R
			m_color_G[ii] = p[chG] * factor;  // G
			m_color_B[ii] = p[chB] * factor;  // B
			// m_min_dist[ii]	= p_dist[p_proj[k]];

			n_proj++;
		}
	}  // end for

	return true;
}  // end colourFromObservation

void CColouredPointsMap::resetPointsMinDist(float defValue)
{
	MRPT_UNUSED_PARAM(defValue);
	// m_min_dist.assign(x.size(),defValue);
}

bool CColouredPointsMap::save3D_and_colour_to_text_file(
	const std::string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	for (unsigned int i = 0; i < x.size(); i++)
		os::fprintf(
			f, "%f %f %f %d %d %d\n", x[i], y[i], z[i],
			(uint8_t)(255 * m_color_R[i]), (uint8_t)(255 * m_color_G[i]),
			(uint8_t)(255 * m_color_B[i]));
	//	os::fprintf(f,"%f %f %f %f %f %f
	//%f\n",x[i],y[i],z[i],m_color_R[i],m_color_G[i],m_color_B[i],m_min_dist[i]);

	os::fclose(f);
	return true;
}

// ================================ PLY files import & export virtual methods
// ================================

/** In a base class, reserve memory to prepare subsequent calls to
 * PLY_import_set_vertex */
void CColouredPointsMap::PLY_import_set_vertex_count(const size_t N)
{
	this->setSize(N);
}

/** In a base class, will be called after PLY_import_set_vertex_count() once for
 * each loaded point.
  *  \param pt_color Will be nullptr if the loaded file does not provide color
 * info.
  */
void CColouredPointsMap::PLY_import_set_vertex(
	const size_t idx, const mrpt::math::TPoint3Df& pt,
	const mrpt::img::TColorf* pt_color)
{
	if (pt_color)
		this->setPoint(
			idx, pt.x, pt.y, pt.z, pt_color->R, pt_color->G, pt_color->B);
	else
		this->setPoint(idx, pt.x, pt.y, pt.z);
}

/** In a base class, will be called after PLY_export_get_vertex_count() once for
 * each exported point.
  *  \param pt_color Will be nullptr if the loaded file does not provide color
 * info.
  */
void CColouredPointsMap::PLY_export_get_vertex(
	const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
	mrpt::img::TColorf& pt_color) const
{
	pt_has_color = true;

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
void CColouredPointsMap::addFrom_classSpecific(
	const CPointsMap& anotherMap, const size_t nPreviousPoints)
{
	const size_t nOther = anotherMap.size();

	// Specific data for this class:
	const CColouredPointsMap* anotheMap_col =
		dynamic_cast<const CColouredPointsMap*>(&anotherMap);

	if (anotheMap_col)
	{
		for (size_t i = 0, j = nPreviousPoints; i < nOther; i++, j++)
		{
			m_color_R[j] = anotheMap_col->m_color_R[i];
			m_color_G[j] = anotheMap_col->m_color_G[i];
			m_color_B[j] = anotheMap_col->m_color_B[i];
		}
	}
}

/** Save the point cloud as a PCL PCD file, in either ASCII or binary format
 * \return false on any error */
bool CColouredPointsMap::savePCDFile(
	const std::string& filename, bool save_as_binary) const
{
#if MRPT_HAS_PCL
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	const size_t nThis = this->size();

	// Fill in the cloud data
	cloud.width = nThis;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	const float f = 255.f;

	union myaux_t {
		uint8_t rgb[4];
		float f;
	} aux_val;

	for (size_t i = 0; i < nThis; ++i)
	{
		cloud.points[i].x = this->x[i];
		cloud.points[i].y = this->y[i];
		cloud.points[i].z = this->z[i];

		aux_val.rgb[0] = static_cast<uint8_t>(this->m_color_B[i] * f);
		aux_val.rgb[1] = static_cast<uint8_t>(this->m_color_G[i] * f);
		aux_val.rgb[2] = static_cast<uint8_t>(this->m_color_R[i] * f);

		cloud.points[i].rgb = aux_val.f;
	}

	return 0 == pcl::io::savePCDFile(filename, cloud, save_as_binary);

#else
	MRPT_UNUSED_PARAM(filename);
	MRPT_UNUSED_PARAM(save_as_binary);
	THROW_EXCEPTION("Operation not available: MRPT was built without PCL");
#endif
}

namespace mrpt
{
namespace maps
{
namespace detail
{
using mrpt::maps::CColouredPointsMap;

template <>
struct pointmap_traits<CColouredPointsMap>
{
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called only once before inserting
	 * points - this is the place to reserve memory in lric for extra working
	 * variables. */
	inline static void internal_loadFromRangeScan2D_init(
		CColouredPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		// Vars:
		//  [0] -> pR
		//  [1] -> pG
		//  [2] -> pB
		//  [3] -> Az_1_color
		lric.fVars.resize(4);

		ASSERT_NOT_EQUAL_(me.colorScheme.z_max, me.colorScheme.z_min)
		lric.fVars[3] = 1.0 / (me.colorScheme.z_max - me.colorScheme.z_min);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data */
	inline static void internal_loadFromRangeScan2D_prepareOneRange(
		CColouredPointsMap& me, const float gx, const float gy, const float gz,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(gx);
		MRPT_UNUSED_PARAM(gy);
		// Relative height of the point wrt the sensor:
		const float rel_z = gz - lric.HM.get_unsafe(2, 3);  // m23;

		// Variable renaming:
		float& pR = lric.fVars[0];
		float& pG = lric.fVars[1];
		float& pB = lric.fVars[2];
		const float& Az_1_color = lric.fVars[3];

		// Compute color:
		switch (me.colorScheme.scheme)
		{
			// case cmFromHeightRelativeToSensor:
			case CColouredPointsMap::cmFromHeightRelativeToSensorJet:
			case CColouredPointsMap::cmFromHeightRelativeToSensorGray:
			{
				float q = (rel_z - me.colorScheme.z_min) * Az_1_color;
				if (q < 0)
					q = 0;
				else if (q > 1)
					q = 1;

				if (me.colorScheme.scheme ==
					CColouredPointsMap::cmFromHeightRelativeToSensorGray)
				{
					pR = pG = pB = q;
				}
				else
				{
					mrpt::utils::jet2rgb(q, pR, pG, pB);
				}
			}
			break;
			case CColouredPointsMap::cmFromIntensityImage:
			{
				// In 2D scans we don't have this channel.
				pR = pG = pB = 1.0;
			}
			break;
			default:
				THROW_EXCEPTION("Unknown color scheme");
		}
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called after each
	 * "{x,y,z}.push_back(...);" */
	inline static void internal_loadFromRangeScan2D_postPushBack(
		CColouredPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		float& pR = lric.fVars[0];
		float& pG = lric.fVars[1];
		float& pB = lric.fVars[2];
		me.m_color_R.push_back(pR);
		me.m_color_G.push_back(pG);
		me.m_color_B.push_back(pB);
		// m_min_dist.push_back(1e4);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called only once before inserting
	 * points - this is the place to reserve memory in lric for extra working
	 * variables. */
	inline static void internal_loadFromRangeScan3D_init(
		CColouredPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
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
		float& cx = lric.fVars[5];
		float& cy = lric.fVars[6];
		float& fx = lric.fVars[7];
		float& fy = lric.fVars[8];

		// unsigned int vars:
		//  [0] -> imgW
		//  [1] -> imgH
		//  [2] -> img_idx_x
		//  [3] -> img_idx_y
		lric.uVars.resize(4);
		unsigned int& imgW = lric.uVars[0];
		unsigned int& imgH = lric.uVars[1];
		unsigned int& img_idx_x = lric.uVars[2];
		unsigned int& img_idx_y = lric.uVars[3];

		// bool vars:
		//  [0] -> hasValidIntensityImage
		//  [1] -> hasColorIntensityImg
		//  [2] -> simple_3d_to_color_relation
		lric.bVars.resize(3);
		uint8_t& hasValidIntensityImage = lric.bVars[0];
		uint8_t& hasColorIntensityImg = lric.bVars[1];
		uint8_t& simple_3d_to_color_relation = lric.bVars[2];

		ASSERT_NOT_EQUAL_(me.colorScheme.z_max, me.colorScheme.z_min)
		lric.fVars[3] = 1.0 / (me.colorScheme.z_max -
							   me.colorScheme.z_min);  // Az_1_color = ...
		lric.fVars[4] = 1.0f / 255;  // K_8u

		hasValidIntensityImage = false;
		imgW = 0;
		imgH = 0;

		if (lric.rangeScan.hasIntensityImage)
		{
			// assure the size matches:
			if (lric.rangeScan.points3D_x.size() ==
				lric.rangeScan.intensityImage.getWidth() *
					lric.rangeScan.intensityImage.getHeight())
			{
				hasValidIntensityImage = true;
				imgW = lric.rangeScan.intensityImage.getWidth();
				imgH = lric.rangeScan.intensityImage.getHeight();
			}
		}

		hasColorIntensityImg =
			hasValidIntensityImage && lric.rangeScan.intensityImage.isColor();

		// running indices for the image pixels for the gray levels:
		// If both range & intensity images coincide (e.g. SwissRanger), then we
		// can just
		// assign 3D points to image pixels one-to-one, but that's not the case
		// if
		//
		simple_3d_to_color_relation =
			(std::abs(lric.rangeScan.relativePoseIntensityWRTDepth.norm()) <
			 1e-5);
		img_idx_x = 0;
		img_idx_y = 0;

		// Will be used just if simple_3d_to_color_relation=false
		cx = lric.rangeScan.cameraParamsIntensity.cx();
		cy = lric.rangeScan.cameraParamsIntensity.cy();
		fx = lric.rangeScan.cameraParamsIntensity.fx();
		fy = lric.rangeScan.cameraParamsIntensity.fy();
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data */
	inline static void internal_loadFromRangeScan3D_prepareOneRange(
		CColouredPointsMap& me, const float gx, const float gy, const float gz,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(gx);
		MRPT_UNUSED_PARAM(gy);
		// Rename variables:
		float& pR = lric.fVars[0];
		float& pG = lric.fVars[1];
		float& pB = lric.fVars[2];
		float& Az_1_color = lric.fVars[3];
		float& K_8u = lric.fVars[4];
		float& cx = lric.fVars[5];
		float& cy = lric.fVars[6];
		float& fx = lric.fVars[7];
		float& fy = lric.fVars[8];

		unsigned int& imgW = lric.uVars[0];
		unsigned int& imgH = lric.uVars[1];
		unsigned int& img_idx_x = lric.uVars[2];
		unsigned int& img_idx_y = lric.uVars[3];

		const uint8_t& hasValidIntensityImage = lric.bVars[0];
		const uint8_t& hasColorIntensityImg = lric.bVars[1];
		const uint8_t& simple_3d_to_color_relation = lric.bVars[2];

		// Relative height of the point wrt the sensor:
		const float rel_z = gz - lric.HM.get_unsafe(2, 3);  // m23;

		// Compute color:
		switch (me.colorScheme.scheme)
		{
			// case cmFromHeightRelativeToSensor:
			case CColouredPointsMap::cmFromHeightRelativeToSensorJet:
			case CColouredPointsMap::cmFromHeightRelativeToSensorGray:
			{
				float q = (rel_z - me.colorScheme.z_min) * Az_1_color;
				if (q < 0)
					q = 0;
				else if (q > 1)
					q = 1;

				if (me.colorScheme.scheme ==
					CColouredPointsMap::cmFromHeightRelativeToSensorGray)
				{
					pR = pG = pB = q;
				}
				else
				{
					mrpt::utils::jet2rgb(q, pR, pG, pB);
				}
			}
			break;
			case CColouredPointsMap::cmFromIntensityImage:
			{
				// Do we have to project the 3D point into the image plane??
				bool hasValidColor = false;
				if (simple_3d_to_color_relation)
				{
					hasValidColor = true;
				}
				else
				{
					// Do projection:
					TPoint3D pt;  // pt_wrt_colorcam;
					lric.rangeScan.relativePoseIntensityWRTDepth
						.inverseComposePoint(
							lric.scan_x, lric.scan_y, lric.scan_z, pt.x, pt.y,
							pt.z);

					// Project to image plane:
					if (pt.z)
					{
						img_idx_x = cx + fx * pt.x / pt.z;
						img_idx_y = cy + fy * pt.y / pt.z;

						hasValidColor = img_idx_x < imgW &&  // img_idx_x>=0
										// isn't needed for
										// unsigned.
										img_idx_y < imgH;
					}
				}

				if (hasValidColor && hasColorIntensityImg)
				{
					const uint8_t* c = lric.rangeScan.intensityImage.get_unsafe(
						img_idx_x, img_idx_y, 0);
					pR = c[2] * K_8u;
					pG = c[1] * K_8u;
					pB = c[0] * K_8u;
				}
				else if (hasValidColor && hasValidIntensityImage)
				{
					uint8_t c = *lric.rangeScan.intensityImage.get_unsafe(
						img_idx_x, img_idx_y, 0);
					pR = pG = pB = c * K_8u;
				}
				else
				{
					pR = pG = pB = 1.0;
				}
			}
			break;
			default:
				THROW_EXCEPTION("Unknown color scheme");
		}
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called after each
	 * "{x,y,z}.push_back(...);" */
	inline static void internal_loadFromRangeScan3D_postPushBack(
		CColouredPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		float& pR = lric.fVars[0];
		float& pG = lric.fVars[1];
		float& pB = lric.fVars[2];

		me.m_color_R.push_back(pR);
		me.m_color_G.push_back(pG);
		me.m_color_B.push_back(pB);

		// m_min_dist.push_back(1e4);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data, at the
	 * end */
	inline static void internal_loadFromRangeScan3D_postOneRange(
		CColouredPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(me);
		unsigned int& imgW = lric.uVars[0];
		unsigned int& img_idx_x = lric.uVars[2];
		unsigned int& img_idx_y = lric.uVars[3];

		const uint8_t& hasValidIntensityImage = lric.bVars[0];
		const uint8_t& simple_3d_to_color_relation = lric.bVars[2];

		// Advance the color pointer (just for simple cases, e.g. SwissRanger,
		// not for Kinect)
		if (simple_3d_to_color_relation && hasValidIntensityImage)
		{
			if (++img_idx_x >= imgW)
			{
				img_idx_y++;
				img_idx_x = 0;
			}
		}
	}
};
}
}
}

/** See CPointsMap::loadFromRangeScan() */
void CColouredPointsMap::loadFromRangeScan(
	const CObservation2DRangeScan& rangeScan, const CPose3D* robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CColouredPointsMap>::
		templ_loadFromRangeScan(*this, rangeScan, robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void CColouredPointsMap::loadFromRangeScan(
	const CObservation3DRangeScan& rangeScan, const CPose3D* robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CColouredPointsMap>::
		templ_loadFromRangeScan(*this, rangeScan, robotPose);
}
