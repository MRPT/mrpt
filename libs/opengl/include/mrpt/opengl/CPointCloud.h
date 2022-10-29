/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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
 * This class is just an OpenGL representation of a point cloud. For operating
 * with maps of points, see mrpt::maps::CPointsMap and derived classes.
 *
 * To load from a points-map, CPointCloud::loadFromPointsMap().
 *
 * This class uses smart optimizations while rendering to efficiently draw
 * clouds of millions of points, using octrees.
 *
 * ![mrpt::opengl::CPointCloud](preview_CPointCloud.png)
 *
 *  \sa opengl::CPlanarLaserScan, opengl::COpenGLScene,
 * opengl::CPointCloudColoured, mrpt::maps::CPointsMap \ingroup mrpt_opengl_grp
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
	mrpt::math::TBoundingBox getBoundingBox() const override
	{
		if (empty()) return {};
		if (auto bb = this->octree_getBoundingBox(); bb) return *bb;
		else
			return {};
	}

	/** @name Read/Write of the list of points to render
		@{ */

	size_t size() const
	{
		std::shared_lock<std::shared_mutex> wfReadLock(
			CRenderizableShaderPoints::m_pointsMtx);
		return m_points.size();
	}
	/// Like size(), but without locking the data mutex (internal usage)
	size_t size_unprotected() const { return m_points.size(); }

	/** Set the number of points (with contents undefined) */
	void resize(size_t N)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);
		m_points.resize(N);
		m_minmax_valid = false;
		wfWriteLock.unlock();
		markAllPointsAsNew();
	}

	/** Like STL std::vector's reserve */
	void reserve(size_t N)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);
		m_points.reserve(N);
	}

	/** Set the list of (X,Y,Z) point coordinates, all at once, from three
	 * vectors with their coordinates */
	template <typename T>
	void setAllPoints(
		const std::vector<T>& x, const std::vector<T>& y,
		const std::vector<T>& z)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);

		const auto N = x.size();
		m_points.resize(N);
		for (size_t i = 0; i < N; i++)
			m_points[i] = {
				static_cast<float>(x[i]), static_cast<float>(y[i]),
				static_cast<float>(z[i])};
		m_minmax_valid = false;
		wfWriteLock.unlock();
		markAllPointsAsNew();
	}

	/// \overload Prefer setAllPointsFast() instead
	void setAllPoints(const std::vector<mrpt::math::TPoint3D>& pts);

	/** Set the list of (X,Y,Z) point coordinates, DESTROYING the contents
	 * of the input vectors (via swap) */
	void setAllPointsFast(std::vector<mrpt::math::TPoint3Df>& pts)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);

		this->clear();
		m_points.swap(pts);
		m_minmax_valid = false;
		wfWriteLock.unlock();
		markAllPointsAsNew();
		CRenderizable::notifyChange();
	}

	/** Get a const reference to the internal array of points */
	const std::vector<mrpt::math::TPoint3Df>& getArrayPoints() const
	{
		std::shared_lock<std::shared_mutex> wfReadLock(
			CRenderizableShaderPoints::m_pointsMtx);

		return m_points;
	}

	/** Empty the list of points. */
	void clear();

	bool empty() const
	{
		std::shared_lock<std::shared_mutex> wfReadLock(
			CRenderizableShaderPoints::m_pointsMtx);
		return m_points.empty();
	}

	/** Adds a new point to the cloud */
	void insertPoint(float x, float y, float z);

	void insertPoint(const mrpt::math::TPoint3Df& p)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);
		insertPoint(p.x, p.y, p.z);
	}
	void insertPoint(const mrpt::math::TPoint3D& p)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);
		insertPoint(p.x, p.y, p.z);
	}

	/** Read access to each individual point (checks for "i" in the valid
	 * range only in Debug). */
	const mrpt::math::TPoint3Df& operator[](size_t i) const
	{
#ifdef _DEBUG
		ASSERT_LT_(i, size());
#endif
		return m_points[i];
	}

	/// NOTE: This method is intentionally not protected by the shared_mutex,
	/// since it's called in the inner loops of the octree, which acquires the
	/// lock once.
	const mrpt::math::TPoint3Df& getPoint3Df(size_t i) const
	{
		return m_points[i];
	}

	/** Write an individual point (checks for "i" in the valid range only in
	 * Debug). */
	void setPoint(size_t i, const float x, const float y, const float z);

	/** Write an individual point (without checking validity of the index).
	 */
	void setPoint_fast(size_t i, const float x, const float y, const float z)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);
		m_points[i] = {x, y, z};
		m_minmax_valid = false;
		wfWriteLock.unlock();
		markAllPointsAsNew();
	}

	/** Load the points from any other point map class supported by the
	 * adapter mrpt::opengl::PointCloudAdapter. */
	template <class POINTSMAP>
	void loadFromPointsMap(const POINTSMAP* themap);
	// Must be implemented at the end of the header.

	/** Load the points from a list of mrpt::math::TPoint3D
	 */
	template <class LISTOFPOINTS>
	void loadFromPointsList(LISTOFPOINTS& pointsList)
	{
		std::unique_lock<std::shared_mutex> wfWriteLock(
			CRenderizableShaderPoints::m_pointsMtx);

		MRPT_START
		const size_t N = pointsList.size();
		m_points.resize(N);
		size_t idx;
		typename LISTOFPOINTS::const_iterator it;
		for (idx = 0, it = pointsList.begin(); idx < N; ++idx, ++it)
			m_points[idx] = {
				static_cast<float>(it->x), static_cast<float>(it->y),
				static_cast<float>(it->z)};
		wfWriteLock.unlock();
		markAllPointsAsNew();
		CRenderizable::notifyChange();
		MRPT_END
	}

	/** Get the number of elements actually rendered in the last render
	 * event.
	 */
	size_t getActuallyRendered() const { return m_last_rendered_count; }
	/** @} */

	/** @name Modify the appearance of the rendered points
		@{ */
	void enableColorFromX(bool v = true)
	{
		m_colorFromDepth = v ? CPointCloud::colX : CPointCloud::colNone;
		CRenderizable::notifyChange();
	}
	void enableColorFromY(bool v = true)
	{
		m_colorFromDepth = v ? CPointCloud::colY : CPointCloud::colNone;
		CRenderizable::notifyChange();
	}
	void enableColorFromZ(bool v = true)
	{
		m_colorFromDepth = v ? CPointCloud::colZ : CPointCloud::colNone;
		CRenderizable::notifyChange();
	}

	void enablePointSmooth(bool enable = true)
	{
		m_pointSmooth = enable;
		CRenderizable::notifyChange();
	}
	void disablePointSmooth() { m_pointSmooth = false; }
	bool isPointSmoothEnabled() const { return m_pointSmooth; }
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

	/** Private, virtual destructor: only can be deleted from smart pointers
	 */
	~CPointCloud() override = default;

	void toYAMLMap(mrpt::containers::yaml& propertiesMap) const override;

   private:
	/** Buffer for min/max coords when m_colorFromDepth is true. */
	mutable float m_min{0}, m_max{0}, m_max_m_min{0}, m_max_m_min_inv{0};
	/** Color linear function slope */
	mutable mrpt::img::TColorf m_col_slop, m_col_slop_inv;
	mutable bool m_minmax_valid{false};

	/** The colors used to interpolate when m_colorFromDepth is true. */
	mrpt::img::TColorf m_colorFromDepth_min = {0, 0, 0},
					   m_colorFromDepth_max = {0, 0, 1};

	void internal_render_one_point(size_t i) const;
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
	PointCloudAdapter(const mrpt::opengl::CPointCloud& obj)
		: m_obj(*const_cast<mrpt::opengl::CPointCloud*>(&obj))
	{
	}
	/** Get number of points */
	size_t size() const { return m_obj.size(); }
	/** Set number of points (to uninitialized values) */
	void resize(const size_t N) { m_obj.resize(N); }
	/** Does nothing as of now */
	void setDimensions(size_t height, size_t width) {}
	/** Get XYZ coordinates of i'th point */
	template <typename T>
	void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		const auto& pt = m_obj[idx];
		x = pt.x;
		y = pt.y;
		z = pt.z;
	}
	/** Set XYZ coordinates of i'th point */
	void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.setPoint_fast(idx, x, y, z);
	}

	/** Set XYZ coordinates of i'th point */
	void setInvalidPoint(const size_t idx)
	{
		m_obj.setPoint_fast(idx, 0, 0, 0);
	}

};	// end of PointCloudAdapter<mrpt::opengl::CPointCloud>

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
