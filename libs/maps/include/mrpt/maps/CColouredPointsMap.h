/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt
{
namespace maps
{
/** A map of 2D/3D points with individual colours (RGB).
 *  For different color schemes, see CColouredPointsMap::colorScheme
 *  Colors are defined in the range [0,1].
 * \sa mrpt::maps::CPointsMap, mrpt::maps::CMetricMap,
 * mrpt::serialization::CSerializable
 * \ingroup mrpt_maps_grp
 */
class CColouredPointsMap : public CPointsMap
{
	DEFINE_SERIALIZABLE(CColouredPointsMap, mrpt::maps)

   public:
	CColouredPointsMap() = default;

	CColouredPointsMap(const CPointsMap& o) { CPointsMap::operator=(o); }
	CColouredPointsMap(const CColouredPointsMap& o) : CPointsMap()
	{
		impl_copyFrom(o);
	}
	CColouredPointsMap& operator=(const CPointsMap& o)
	{
		impl_copyFrom(o);
		return *this;
	}
	CColouredPointsMap& operator=(const CColouredPointsMap& o)
	{
		impl_copyFrom(o);
		return *this;
	}

	// --------------------------------------------
	/** @name Pure virtual interfaces to be implemented by any class derived
	   from CPointsMap
		@{ */

	void reserve(size_t newLength) override;  // See base class docs
	void resize(size_t newLength) override;  // See base class docs
	void setSize(size_t newLength) override;  // See base class docs

	/** The virtual method for \a insertPoint() *without* calling
	 * mark_as_modified()   */
	void insertPointFast(float x, float y, float z = 0) override;

	/** Get all the data fields for one point as a vector: [X Y Z R G B]
	 *  Unlike getPointAllFields(), this method does not check for index out of
	 * bounds
	 * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
	 */
	void getPointAllFieldsFast(
		const size_t index, std::vector<float>& point_data) const override
	{
		point_data.resize(6);
		point_data[0] = m_x[index];
		point_data[1] = m_y[index];
		point_data[2] = m_z[index];
		point_data[3] = m_color_R[index];
		point_data[4] = m_color_G[index];
		point_data[5] = m_color_B[index];
	}

	/** Set all the data fields for one point as a vector: [X Y Z R G B]
	 *  Unlike setPointAllFields(), this method does not check for index out of
	 * bounds
	 * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
	 */
	void setPointAllFieldsFast(
		const size_t index, const std::vector<float>& point_data) override
	{
		ASSERTDEB_(point_data.size() == 6);
		m_x[index] = point_data[0];
		m_y[index] = point_data[1];
		m_z[index] = point_data[2];
		m_color_R[index] = point_data[3];
		m_color_G[index] = point_data[4];
		m_color_B[index] = point_data[5];
	}

	/** See CPointsMap::loadFromRangeScan() */
	void loadFromRangeScan(
		const mrpt::obs::CObservation2DRangeScan& rangeScan,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	/** See CPointsMap::loadFromRangeScan() */
	void loadFromRangeScan(
		const mrpt::obs::CObservation3DRangeScan& rangeScan,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;

   protected:
	void impl_copyFrom(const CPointsMap& obj) override;
	void addFrom_classSpecific(
		const CPointsMap& anotherMap, const size_t nPreviousPoints) override;

	// Friend methods:
	template <class Derived>
	friend struct detail::loadFromRangeImpl;
	template <class Derived>
	friend struct detail::pointmap_traits;

   public:
	/** @} */
	// --------------------------------------------

	/** Save to a text file. In each line contains X Y Z (meters) R G B (range
	 * [0,1]) for each point in the map.
	 *     Returns false if any error occured, true elsewere.
	 */
	bool save3D_and_colour_to_text_file(const std::string& file) const;

	/** Changes a given point from map. First index is 0.
	 * \exception Throws std::exception on index out of bound.
	 */
	void setPointRGB(
		size_t index, float x, float y, float z, float R, float G,
		float B) override;

	/** Adds a new point given its coordinates and color (colors range is [0,1])
	 */
	void insertPointRGB(
		float x, float y, float z, float R, float G, float B) override;

	/** Changes just the color of a given point from the map. First index is 0.
	 * \exception Throws std::exception on index out of bound.
	 */
	void setPointColor(size_t index, float R, float G, float B);

	/** Like \c setPointColor but without checking for out-of-index erors */
	inline void setPointColor_fast(size_t index, float R, float G, float B)
	{
		m_color_R[index] = R;
		m_color_G[index] = G;
		m_color_B[index] = B;
	}

	/** Retrieves a point and its color (colors range is [0,1])
	 */
	void getPointRGB(
		size_t index, float& x, float& y, float& z, float& R, float& G,
		float& B) const override;

	/** Retrieves a point color (colors range is [0,1]) */
	void getPointColor(size_t index, float& R, float& G, float& B) const;

	/** Like \c getPointColor but without checking for out-of-index erors */
	inline void getPointColor_fast(
		size_t index, float& R, float& G, float& B) const
	{
		R = m_color_R[index];
		G = m_color_G[index];
		B = m_color_B[index];
	}

	/** Returns true if the point map has a color field for each point */
	bool hasColorPoints() const override { return true; }
	/** Override of the default 3D scene builder to account for the individual
	 * points' color.
	 */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** Colour a set of points from a CObservationImage and the global pose of
	 * the robot */
	bool colourFromObservation(
		const mrpt::obs::CObservationImage& obs,
		const mrpt::poses::CPose3D& robotPose);

	/** The choices for coloring schemes:
	 *		- cmFromHeightRelativeToSensor: The Z coordinate wrt the sensor will
	 *be
	 *used to obtain the color using the limits z_min,z_max.
	 * 	- cmFromIntensityImage: When inserting 3D range scans, take the
	 *color
	 *from the intensity image channel, if available.
	 * \sa TColourOptions
	 */
	enum TColouringMethod
	{
		cmFromHeightRelativeToSensor = 0,
		cmFromHeightRelativeToSensorJet = 0,
		cmFromHeightRelativeToSensorGray = 1,
		cmFromIntensityImage = 2
		// Remember: if new values are added, also update TEnumType below!
	};

	/** The definition of parameters for generating colors from laser scans */
	struct TColourOptions : public mrpt::config::CLoadableOptions
	{
		/** Initilization of default parameters */
		TColourOptions();
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		TColouringMethod scheme{cmFromHeightRelativeToSensor};
		float z_min{-10}, z_max{10};
		float d_max{5};
	};

	/** The options employed when inserting laser scans in the map. */
	TColourOptions colorScheme;

	/** Reset the minimum-observed-distance buffer for all the points to a
	 * predefined value */
	void resetPointsMinDist(float defValue = 2000.0f);

	/** @name PCL library support
		@{ */

#if defined(PCL_LINEAR_VERSION)
	/** Save the point cloud as a PCL PCD file, in either ASCII or binary format
	 * \return false on any error */
	inline bool savePCDFile(
		const std::string& filename, bool save_as_binary) const
	{
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
			cloud.points[i].x = m_x[i];
			cloud.points[i].y = m_y[i];
			cloud.points[i].z = m_z[i];

			aux_val.rgb[0] = static_cast<uint8_t>(this->m_color_B[i] * f);
			aux_val.rgb[1] = static_cast<uint8_t>(this->m_color_G[i] * f);
			aux_val.rgb[2] = static_cast<uint8_t>(this->m_color_R[i] * f);

			cloud.points[i].rgb = aux_val.f;
		}

		return 0 == pcl::io::savePCDFile(filename, cloud, save_as_binary);
	}
#endif

	/** Loads a PCL point cloud (WITH RGB information) into this MRPT class (for
	 * clouds without RGB data, see CPointsMap::setFromPCLPointCloud() ).
	 *  Usage example:
	 *  \code
	 *    pcl::PointCloud<pcl::PointXYZRGB> cloud;
	 *    mrpt::maps::CColouredPointsMap       pc;
	 *
	 *    pc.setFromPCLPointCloudRGB(cloud);
	 *  \endcode
	 * \sa CPointsMap::setFromPCLPointCloud()
	 */
	template <class POINTCLOUD>
	void setFromPCLPointCloudRGB(const POINTCLOUD& cloud)
	{
		const size_t N = cloud.points.size();
		clear();
		reserve(N);
		const float f = 1.0f / 255.0f;
		for (size_t i = 0; i < N; ++i)
			this->insertPoint(
				cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,
				cloud.points[i].r * f, cloud.points[i].g * f,
				cloud.points[i].b * f);
	}

	/** Like CPointsMap::getPCLPointCloud() but for PointCloud<PointXYZRGB> */
	template <class POINTCLOUD>
	void getPCLPointCloudXYZRGB(POINTCLOUD& cloud) const
	{
		const size_t nThis = this->size();
		this->getPCLPointCloud(cloud);  // 1st: xyz data
		// 2nd: RGB data
		for (size_t i = 0; i < nThis; ++i)
		{
			float R, G, B;
			this->getPointColor_fast(i, R, G, B);
			cloud.points[i].r = static_cast<uint8_t>(R * 255);
			cloud.points[i].g = static_cast<uint8_t>(G * 255);
			cloud.points[i].b = static_cast<uint8_t>(B * 255);
		}
	}
	/** @} */

   protected:
	/** The color data */
	mrpt::aligned_std_vector<float> m_color_R, m_color_G, m_color_B;

	/** Minimum distance from where the points have been seen */
	// std::vector<float>	m_min_dist;

	/** Clear the map, erasing all the points */
	void internal_clear() override;

	/** @name Redefinition of PLY Import virtual methods from CPointsMap
		@{ */
	/** In a base class, will be called after PLY_import_set_vertex_count() once
	 * for each loaded point.
	 *  \param pt_color Will be nullptr if the loaded file does not provide
	 * color info.
	 */
	void PLY_import_set_vertex(
		const size_t idx, const mrpt::math::TPoint3Df& pt,
		const mrpt::img::TColorf* pt_color = nullptr) override;

	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_vertex */
	void PLY_import_set_vertex_count(const size_t N) override;
	/** @} */

	/** @name Redefinition of PLY Export virtual methods from CPointsMap
		@{ */
	void PLY_export_get_vertex(
		const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
		mrpt::img::TColorf& pt_color) const override;
	/** @} */

	MAP_DEFINITION_START(CColouredPointsMap)
	mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
	mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
	mrpt::maps::CColouredPointsMap::TColourOptions colourOpts;
	MAP_DEFINITION_END(CColouredPointsMap)

};  // End of class def.

}  // namespace maps

#include <mrpt/opengl/pointcloud_adapters.h>
namespace opengl
{
/** Specialization
 * mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap> \ingroup
 * mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::maps::CColouredPointsMap>
{
   private:
	mrpt::maps::CColouredPointsMap& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = true;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = true;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = false;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const mrpt::maps::CColouredPointsMap& obj)
		: m_obj(*const_cast<mrpt::maps::CColouredPointsMap*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.resize(N); }
	/** Does nothing as of now */
	inline void setDimensions(size_t height, size_t width) {}
	/** Get XYZ coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		m_obj.getPointFast(idx, x, y, z);
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.setPointFast(idx, x, y, z);
	}

	/** Get XYZ_RGBf coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBf(
		const size_t idx, T& x, T& y, T& z, float& r, float& g, float& b) const
	{
		m_obj.getPointRGB(idx, x, y, z, r, g, b);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointXYZ_RGBf(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const float r, const float g, const float b)
	{
		m_obj.setPointRGB(idx, x, y, z, r, g, b);
	}

	/** Get XYZ_RGBu8 coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBu8(
		const size_t idx, T& x, T& y, T& z, uint8_t& r, uint8_t& g,
		uint8_t& b) const
	{
		float Rf, Gf, Bf;
		m_obj.getPointRGB(idx, x, y, z, Rf, Gf, Bf);
		r = Rf * 255;
		g = Gf * 255;
		b = Bf * 255;
	}
	/** Set XYZ_RGBu8 coordinates of i'th point */
	inline void setPointXYZ_RGBu8(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointRGB(idx, x, y, z, r / 255.f, g / 255.f, b / 255.f);
	}

	/** Get RGBf color of i'th point */
	inline void getPointRGBf(
		const size_t idx, float& r, float& g, float& b) const
	{
		m_obj.getPointColor_fast(idx, r, g, b);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointRGBf(
		const size_t idx, const float r, const float g, const float b)
	{
		m_obj.setPointColor_fast(idx, r, g, b);
	}

	/** Get RGBu8 color of i'th point */
	inline void getPointRGBu8(
		const size_t idx, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		float R, G, B;
		m_obj.getPointColor_fast(idx, R, G, B);
		r = mrpt::f2u8(R);
		g = mrpt::f2u8(G);
		b = mrpt::f2u8(B);
	}
	/** Set RGBu8 coordinates of i'th point */
	inline void setPointRGBu8(
		const size_t idx, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointColor_fast(idx, r / 255.f, g / 255.f, b / 255.f);
	}

	/** Set XYZ coordinates of i'th point */
	inline void setInvalidPoint(const size_t idx)
	{
		THROW_EXCEPTION("mrpt::maps::CColouredPointsMap needs to be dense");
	}

};  // end of PointCloudAdapter<mrpt::maps::CColouredPointsMap>
}  // namespace opengl
}  // namespace mrpt

MRPT_ENUM_TYPE_BEGIN(mrpt::maps::CColouredPointsMap::TColouringMethod)
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CColouredPointsMap::TColouringMethod,
	cmFromHeightRelativeToSensor);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CColouredPointsMap::TColouringMethod,
	cmFromHeightRelativeToSensorJet);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CColouredPointsMap::TColouringMethod,
	cmFromHeightRelativeToSensorGray);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CColouredPointsMap::TColouringMethod, cmFromIntensityImage);
MRPT_ENUM_TYPE_END()
