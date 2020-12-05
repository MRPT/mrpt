/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/PLY_import_export.h>
#include <mrpt/opengl/pointcloud_adapters.h>

namespace mrpt::opengl
{
/** A cloud of points, all with the same color or each depending on its value
 * along a particular coordinate axis.
 *  This class is just an OpenGL representation of a point cloud. For operating
 * with maps of points, see mrpt::maps::CPointsMap and derived classes.
 *
 *  To load from a points-map, CPointCloud::loadFromPointsMap().
 *
 *   This class uses smart optimizations while rendering to efficiently draw
 * clouds of millions of points,
 *   as described in this page:
 * https://www.mrpt.org/Efficiently_rendering_point_clouds_of_millions_of_points
 *
 *  \sa opengl::CPlanarLaserScan, opengl::COpenGLScene,
 * opengl::CPointCloudColoured, mrpt::maps::CPointsMap
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CPointCloud </td> <td> \image html
 * preview_CPointCloud.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CPointCloud : public CRenderizableShaderPoints,
					public COctreePointRenderer<CPointCloud>,
					public mrpt::opengl::PLY_Importer,
					public mrpt::opengl::PLY_Exporter
{
	DEFINE_SERIALIZABLE(CPointCloud, mrpt::opengl)
	DEFINE_SCHEMA_SERIALIZABLE()
   protected:
	enum Axis
	{
		colNone = 0,
		colZ,
		colY,
		colX
	};

	Axis m_colorFromDepth = CPointCloud::colNone;

	/** Actually, an alias for the base class shader container of points. Kept
	 * to have an easy to use name. */
	std::vector<mrpt::math::TPoint3Df>& m_points =
		CRenderizableShaderPoints::m_vertex_buffer_data;

	/** Default: false */
	bool m_pointSmooth = false;

	mutable size_t m_last_rendered_count{0}, m_last_rendered_count_ongoing{0};

	/** Do needed internal work if all points are new (octree rebuilt,...) */
	void markAllPointsAsNew();

   protected:
	/** @name PLY Import virtual methods to implement in base classes
		@{ */
	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_vertex */
	void PLY_import_set_vertex_count(const size_t N) override;

	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_face */
	void PLY_import_set_face_count([[maybe_unused]] const size_t N) override {}

	/** In a base class, will be called after PLY_import_set_vertex_count() once
	 * for each loaded point.
	 *  \param pt_color Will be nullptr if the loaded file does not provide
	 * color info.
	 */
	void PLY_import_set_vertex(
		const size_t idx, const mrpt::math::TPoint3Df& pt,
		const mrpt::img::TColorf* pt_color = nullptr) override;
	/** @} */

	/** @name PLY Export virtual methods to implement in base classes
		@{ */
	size_t PLY_export_get_vertex_count() const override;
	size_t PLY_export_get_face_count() const override { return 0; }
	void PLY_export_get_vertex(
		const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
		mrpt::img::TColorf& pt_color) const override;
	/** @} */

   public:
	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override
	{
		this->octree_getBoundingBox(bb_min, bb_max);
	}

	/** @name Read/Write of the list of points to render
		@{ */

	inline size_t size() const { return m_points.size(); }
	/** Set the number of points (with contents undefined) */
	inline void resize(size_t N)
	{
		m_points.resize(N);
		m_minmax_valid = false;
		markAllPointsAsNew();
	}

	/** Like STL std::vector's reserve */
	inline void reserve(size_t N) { m_points.reserve(N); }

	/** Set the list of (X,Y,Z) point coordinates, all at once, from three
	 * vectors with their coordinates */
	template <typename T>
	void setAllPoints(
		const std::vector<T>& x, const std::vector<T>& y,
		const std::vector<T>& z)
	{
		const auto N = x.size();
		m_points.resize(N);
		for (size_t i = 0; i < N; i++)
			m_points[i] = {static_cast<float>(x[i]), static_cast<float>(y[i]),
						   static_cast<float>(z[i])};
		m_minmax_valid = false;
		markAllPointsAsNew();
	}

	/// \overload Prefer setAllPointsFast() instead
	void setAllPoints(const std::vector<mrpt::math::TPoint3D>& pts);

	/** Set the list of (X,Y,Z) point coordinates, DESTROYING the contents of
	 * the input vectors (via swap) */
	void setAllPointsFast(std::vector<mrpt::math::TPoint3Df>& pts)
	{
		this->clear();
		m_points.swap(pts);
		m_minmax_valid = false;
		markAllPointsAsNew();
		CRenderizable::notifyChange();
	}

	/** Get a const reference to the internal array of points */
	inline const std::vector<mrpt::math::TPoint3Df>& getArrayPoints() const
	{
		return m_points;
	}

	/** Empty the list of points. */
	void clear();

	/** Adds a new point to the cloud */
	void insertPoint(float x, float y, float z);

	/** Read access to each individual point (checks for "i" in the valid range
	 * only in Debug). */
	inline const mrpt::math::TPoint3Df& operator[](size_t i) const
	{
#ifdef _DEBUG
		ASSERT_LT_(i, size());
#endif
		return m_points[i];
	}

	inline const mrpt::math::TPoint3Df& getPoint3Df(size_t i) const
	{
		return m_points[i];
	}

	/** Write an individual point (checks for "i" in the valid range only in
	 * Debug). */
	void setPoint(size_t i, const float x, const float y, const float z);

	/** Write an individual point (without checking validity of the index). */
	inline void setPoint_fast(
		size_t i, const float x, const float y, const float z)
	{
		m_points[i] = {x, y, z};
		m_minmax_valid = false;
		markAllPointsAsNew();
	}

	/** Load the points from any other point map class supported by the adapter
	 * mrpt::opengl::PointCloudAdapter. */
	template <class POINTSMAP>
	void loadFromPointsMap(const POINTSMAP* themap);
	// Must be implemented at the end of the header.

	/** Load the points from a list of mrpt::math::TPoint3D
	 */
	template <class LISTOFPOINTS>
	void loadFromPointsList(LISTOFPOINTS& pointsList)
	{
		MRPT_START
		const size_t N = pointsList.size();
		m_points.resize(N);
		size_t idx;
		typename LISTOFPOINTS::const_iterator it;
		for (idx = 0, it = pointsList.begin(); idx < N; ++idx, ++it)
			m_points[idx] = {static_cast<float>(it->x),
							 static_cast<float>(it->y),
							 static_cast<float>(it->z)};
		markAllPointsAsNew();
		CRenderizable::notifyChange();
		MRPT_END
	}

	/** Get the number of elements actually rendered in the last render event.
	 */
	size_t getActuallyRendered() const { return m_last_rendered_count; }
	/** @} */

	/** @name Modify the appearance of the rendered points
		@{ */
	inline void enableColorFromX(bool v = true)
	{
		m_colorFromDepth = v ? CPointCloud::colX : CPointCloud::colNone;
		CRenderizable::notifyChange();
	}
	inline void enableColorFromY(bool v = true)
	{
		m_colorFromDepth = v ? CPointCloud::colY : CPointCloud::colNone;
		CRenderizable::notifyChange();
	}
	inline void enableColorFromZ(bool v = true)
	{
		m_colorFromDepth = v ? CPointCloud::colZ : CPointCloud::colNone;
		CRenderizable::notifyChange();
	}

	inline void enablePointSmooth(bool enable = true)
	{
		m_pointSmooth = enable;
		CRenderizable::notifyChange();
	}
	inline void disablePointSmooth() { m_pointSmooth = false; }
	inline bool isPointSmoothEnabled() const { return m_pointSmooth; }
	/** Sets the colors used as extremes when colorFromDepth is enabled. */
	void setGradientColors(
		const mrpt::img::TColorf& colorMin, const mrpt::img::TColorf& colorMax);

	/** @} */

	void onUpdateBuffers_Points() override;

	/** Render a subset of points (required by octree renderer) */
	void render_subset(
		const bool all, const std::vector<size_t>& idxs,
		const float render_area_sqpixels) const;

	/** Constructor */
	CPointCloud();

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CPointCloud() override = default;

   private:
	/** Buffer for min/max coords when m_colorFromDepth is true. */
	mutable float m_min{0}, m_max{0}, m_max_m_min{0}, m_max_m_min_inv{0};
	/** Color linear function slope */
	mutable mrpt::img::TColorf m_col_slop, m_col_slop_inv;
	mutable bool m_minmax_valid{false};

	/** The colors used to interpolate when m_colorFromDepth is true. */
	mrpt::img::TColorf m_colorFromDepth_min = {0, 0, 0},
					   m_colorFromDepth_max = {0, 0, 1};

	inline void internal_render_one_point(size_t i) const;
};

/** Specialization mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>
 * \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::opengl::CPointCloud>
{
   private:
	mrpt::opengl::CPointCloud& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = false;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = false;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = false;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const mrpt::opengl::CPointCloud& obj)
		: m_obj(*const_cast<mrpt::opengl::CPointCloud*>(&obj))
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
		const auto& pt = m_obj[idx];
		x = pt.x;
		y = pt.y;
		z = pt.z;
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.setPoint_fast(idx, x, y, z);
	}

	/** Set XYZ coordinates of i'th point */
	inline void setInvalidPoint(const size_t idx)
	{
		m_obj.setPoint_fast(idx, 0, 0, 0);
	}

};  // end of PointCloudAdapter<mrpt::opengl::CPointCloud>

// After declaring the adapter we can here implement this method:
template <class POINTSMAP>
void CPointCloud::loadFromPointsMap(const POINTSMAP* themap)
{
	CRenderizable::notifyChange();
	ASSERT_(themap != nullptr);
	mrpt::opengl::PointCloudAdapter<CPointCloud> pc_dst(*this);
	const mrpt::opengl::PointCloudAdapter<POINTSMAP> pc_src(*themap);
	const size_t N = pc_src.size();
	pc_dst.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		float x, y, z;
		pc_src.getPointXYZ(i, x, y, z);
		pc_dst.setPointXYZ(i, x, y, z);
	}
}
}  // namespace mrpt::opengl
