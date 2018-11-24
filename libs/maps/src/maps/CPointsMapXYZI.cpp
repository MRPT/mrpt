/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          http://www.mrpt.org/                          |
|                                                                        |
| Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
| See: http://www.mrpt.org/Authors - All rights reserved.                |
| Released under BSD License. See details in http://www.mrpt.org/License |
+------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/core/bits_mem.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/aligned_serialization.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <iostream>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::config;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CPointsMapXYZI", mrpt::maps::CPointsMapXYZI)

CPointsMapXYZI::TMapDefinition::TMapDefinition() = default;

void CPointsMapXYZI::TMapDefinition::loadFromConfigFile_map_specific(
	const CConfigFileBase& source, const std::string& sectionNamePrefix)
{
	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CPointsMapXYZI::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CPointsMapXYZI::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CPointsMapXYZI::TMapDefinition& def =
		*dynamic_cast<const CPointsMapXYZI::TMapDefinition*>(&_def);
	auto* obj = new CPointsMapXYZI();
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CPointsMapXYZI, CPointsMap, mrpt::maps)

#if MRPT_HAS_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#endif

void CPointsMapXYZI::reserve(size_t newLength)
{
	m_x.reserve(newLength);
	m_y.reserve(newLength);
	m_z.reserve(newLength);
	m_intensity.reserve(newLength);
}

// Resizes all point buffers so they can hold the given number of points: newly
// created points are set to default values,
//  and old contents are not changed.
void CPointsMapXYZI::resize(size_t newLength)
{
	m_x.resize(newLength, 0);
	m_y.resize(newLength, 0);
	m_z.resize(newLength, 0);
	m_intensity.resize(newLength, 1);
	mark_as_modified();
}

// Resizes all point buffers so they can hold the given number of points,
// *erasing* all previous contents
//  and leaving all points to default values.
void CPointsMapXYZI::setSize(size_t newLength)
{
	m_x.assign(newLength, 0);
	m_y.assign(newLength, 0);
	m_z.assign(newLength, 0);
	m_intensity.assign(newLength, 0);
	mark_as_modified();
}

void CPointsMapXYZI::impl_copyFrom(const CPointsMap& obj)
{
	// This also does a ::resize(N) of all data fields.
	CPointsMap::base_copyFrom(obj);

	const auto* pXYZI = dynamic_cast<const CPointsMapXYZI*>(&obj);
	if (pXYZI) m_intensity = pXYZI->m_intensity;
}

uint8_t CPointsMapXYZI::serializeGetVersion() const { return 0; }
void CPointsMapXYZI::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t n = m_x.size();

	// First, write the number of points:
	out << n;

	if (n > 0)
	{
		out.WriteBufferFixEndianness(&m_x[0], n);
		out.WriteBufferFixEndianness(&m_y[0], n);
		out.WriteBufferFixEndianness(&m_z[0], n);
		out.WriteBufferFixEndianness(&m_intensity[0], n);
	}
	insertionOptions.writeToStream(out);
	likelihoodOptions.writeToStream(out);
}

void CPointsMapXYZI::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			mark_as_modified();

			// Read the number of points:
			uint32_t n;
			in >> n;
			this->resize(n);
			if (n > 0)
			{
				in.ReadBufferFixEndianness(&m_x[0], n);
				in.ReadBufferFixEndianness(&m_y[0], n);
				in.ReadBufferFixEndianness(&m_z[0], n);
				in.ReadBufferFixEndianness(&m_intensity[0], n);
			}
			insertionOptions.readFromStream(in);
			likelihoodOptions.readFromStream(in);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPointsMapXYZI::internal_clear()
{
	vector_strong_clear(m_x);
	vector_strong_clear(m_y);
	vector_strong_clear(m_z);
	vector_strong_clear(m_intensity);
	mark_as_modified();
}

void CPointsMapXYZI::setPointRGB(
	size_t index, float x, float y, float z, float R, float G, float B)
{
	if (index >= m_x.size()) THROW_EXCEPTION("Index out of bounds");
	m_x[index] = x;
	m_y[index] = y;
	m_z[index] = z;
	m_intensity[index] = R;
	mark_as_modified();
}

void CPointsMapXYZI::setPointIntensity(size_t index, float I)
{
	if (index >= m_x.size()) THROW_EXCEPTION("Index out of bounds");
	this->m_intensity[index] = I;
	// mark_as_modified();  // No need to rebuild KD-trees, etc...
}

void CPointsMapXYZI::insertPointFast(float x, float y, float z)
{
	m_x.push_back(x);
	m_y.push_back(y);
	m_z.push_back(z);
	m_intensity.push_back(0);
	// mark_as_modified(); -> Fast
}

void CPointsMapXYZI::insertPointRGB(
	float x, float y, float z, float R_intensity, float G_ignored,
	float B_ignored)
{
	m_x.push_back(x);
	m_y.push_back(y);
	m_z.push_back(z);
	m_intensity.push_back(R_intensity);
	mark_as_modified();
}

void CPointsMapXYZI::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	ASSERT_(outObj);

	if (!genericMapParams.enableSaveAs3DObject) return;

	auto obj = mrpt::opengl::CPointCloudColoured::Create();

	obj->loadFromPointsMap(this);
	obj->setColor(1, 1, 1, 1.0);
	obj->setPointSize(this->renderOptions.point_size);

	outObj->insert(obj);
}

void CPointsMapXYZI::getPointRGB(
	size_t index, float& x, float& y, float& z, float& R, float& G,
	float& B) const
{
	if (index >= m_x.size()) THROW_EXCEPTION("Index out of bounds");

	x = m_x[index];
	y = m_y[index];
	z = m_z[index];
	R = G = B = m_intensity[index];
}

float CPointsMapXYZI::getPointIntensity(size_t index) const
{
	if (index >= m_x.size()) THROW_EXCEPTION("Index out of bounds");
	return m_intensity[index];
}

bool CPointsMapXYZI::saveXYZI_to_text_file(const std::string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;
	for (unsigned int i = 0; i < m_x.size(); i++)
		os::fprintf(f, "%f %f %f %f\n", m_x[i], m_y[i], m_z[i], m_intensity[i]);
	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
addFrom_classSpecific
---------------------------------------------------------------*/
void CPointsMapXYZI::addFrom_classSpecific(
	const CPointsMap& anotherMap, const size_t nPreviousPoints)
{
	const size_t nOther = anotherMap.size();

	// Specific data for this class:
	const auto* anotheMap_col =
		dynamic_cast<const CPointsMapXYZI*>(&anotherMap);

	if (anotheMap_col)
	{
		for (size_t i = 0, j = nPreviousPoints; i < nOther; i++, j++)
			m_intensity[j] = anotheMap_col->m_intensity[i];
	}
}

/** Save the point cloud as a PCL PCD file, in either ASCII or binary format
 * \return false on any error */
bool CPointsMapXYZI::savePCDFile(
	const std::string& filename, bool save_as_binary) const
{
#if MRPT_HAS_PCL
	pcl::PointCloud<pcl::PointXYZI> cloud;

	const size_t nThis = this->size();

	// Fill in the cloud data
	cloud.width = nThis;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	const float f = 255.f;

	for (size_t i = 0; i < nThis; ++i)
	{
		cloud.points[i].x = m_x[i];
		cloud.points[i].y = m_y[i];
		cloud.points[i].z = m_z[i];
		cloud.points[i].intensity = this->m_intensity[i];
	}

	return 0 == pcl::io::savePCDFile(filename, cloud, save_as_binary);

#else
	MRPT_UNUSED_PARAM(filename);
	MRPT_UNUSED_PARAM(save_as_binary);
	THROW_EXCEPTION("Operation not available: MRPT was built without PCL")
#endif
}

namespace mrpt::maps::detail
{
using mrpt::maps::CPointsMapXYZI;

template <>
struct pointmap_traits<CPointsMapXYZI>
{
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called only once before inserting
	 * points - this is the place to reserve memory in lric for extra working
	 * variables. */
	inline static void internal_loadFromRangeScan2D_init(
		CPointsMapXYZI& me,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		// lric.fVars: not needed
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data */
	inline static void internal_loadFromRangeScan2D_prepareOneRange(
		CPointsMapXYZI& me, const float gx, const float gy, const float gz,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(gx);
		MRPT_UNUSED_PARAM(gy);
		MRPT_UNUSED_PARAM(gz);
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called after each
	 * "{x,y,z}.push_back(...);" */
	inline static void internal_loadFromRangeScan2D_postPushBack(
		CPointsMapXYZI& me,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		float pI = 1.0f;
		me.m_intensity.push_back(pI);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called only once before inserting
	 * points - this is the place to reserve memory in lric for extra working
	 * variables. */
	inline static void internal_loadFromRangeScan3D_init(
		CPointsMapXYZI& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		// Not used.
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data */
	inline static void internal_loadFromRangeScan3D_prepareOneRange(
		CPointsMapXYZI& me, const float gx, const float gy, const float gz,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(gx);
		MRPT_UNUSED_PARAM(gy);
		MRPT_UNUSED_PARAM(gz);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called after each
	 * "{x,y,z}.push_back(...);" */
	inline static void internal_loadFromRangeScan3D_postPushBack(
		CPointsMapXYZI& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		const float pI = 1.0f;
		me.m_intensity.push_back(pI);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data, at the
	 * end */
	inline static void internal_loadFromRangeScan3D_postOneRange(
		CPointsMapXYZI& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
	}
};
}  // namespace mrpt::maps::detail
/** See CPointsMap::loadFromRangeScan() */
void CPointsMapXYZI::loadFromRangeScan(
	const CObservation2DRangeScan& rangeScan, const CPose3D* robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<
		CPointsMapXYZI>::templ_loadFromRangeScan(*this, rangeScan, robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void CPointsMapXYZI::loadFromRangeScan(
	const CObservation3DRangeScan& rangeScan, const CPose3D* robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<
		CPointsMapXYZI>::templ_loadFromRangeScan(*this, rangeScan, robotPose);
}

// ====PLY files import & export virtual methods
void CPointsMapXYZI::PLY_import_set_vertex_count(const size_t N)
{
	this->setSize(N);
}

void CPointsMapXYZI::PLY_import_set_vertex(
	const size_t idx, const mrpt::math::TPoint3Df& pt, const TColorf* pt_color)
{
	if (pt_color)
		this->setPointRGB(
			idx, pt.x, pt.y, pt.z, pt_color->R, pt_color->G, pt_color->B);
	else
		this->setPoint(idx, pt.x, pt.y, pt.z);
}

void CPointsMapXYZI::PLY_export_get_vertex(
	const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
	TColorf& pt_color) const
{
	pt_has_color = true;

	pt.x = m_x[idx];
	pt.y = m_y[idx];
	pt.z = m_z[idx];
	pt_color.R = pt_color.G = pt_color.B = m_intensity[idx];
}

bool CPointsMapXYZI::loadFromKittiVelodyneFile(const std::string& filename)
{
	try
	{
		mrpt::io::CFileGZInputStream f(filename);
		this->clear();
		this->reserve(10000);

		for (;;)
		{
			constexpr std::size_t nToRead = sizeof(float) * 4;
			float xyzi[4];
			std::size_t nRead = f.Read(&xyzi, nToRead);
			if (nRead == 0)
				break;  // EOF
			else if (nRead == nToRead)
			{
				m_x.push_back(xyzi[0]);
				m_y.push_back(xyzi[1]);
				m_z.push_back(xyzi[2]);
				m_intensity.push_back(xyzi[3]);
			}
			else
				throw std::runtime_error(
					"Unexpected EOF at the middle of a XYZI record "
					"(truncated or corrupted file?)");
		}
		this->mark_as_modified();
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << "[loadFromKittiVelodyneFile] " << e.what() << std::endl;
		return false;
	}
}
