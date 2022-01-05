/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/color_maps.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/PLY_import_export.h>
#include <mrpt/opengl/pointcloud_adapters.h>

namespace mrpt::opengl
{
/** A cloud of points, each one with an individual colour (R,G,B). The alpha
 * component is shared by all the points and is stored in the base member
 * m_color_A.
 *
 * To load from a points-map, CPointCloudColoured::loadFromPointsMap().
 *
 * This class uses smart optimizations while rendering to efficiently draw
 * clouds of millions of points, using octrees.
 *
 * ![mrpt::opengl::CPointCloudColoured](preview_CPointCloudColoured.png)
 *
 * \sa opengl::COpenGLScene, opengl::CPointCloud
 * \ingroup mrpt_opengl_grp
 */
class CPointCloudColoured : public CRenderizableShaderPoints,
							public COctreePointRenderer<CPointCloudColoured>,
							public mrpt::opengl::PLY_Importer,
							public mrpt::opengl::PLY_Exporter
{
	DEFINE_SERIALIZABLE(CPointCloudColoured, mrpt::opengl)

   private:
	/** Actually, an alias for the base class shader container of points. Kept
	 * to have an easy to use name. */
	std::vector<mrpt::math::TPoint3Df>& m_points =
		CRenderizableShaderPoints::m_vertex_buffer_data;

	std::vector<mrpt::img::TColor>& m_point_colors =
		CRenderizableShaderPoints::m_color_buffer_data;

	mutable size_t m_last_rendered_count{0}, m_last_rendered_count_ongoing{0};

   public:
	void onUpdateBuffers_Points() override;

	CPointCloudColoured() = default;
	virtual ~CPointCloudColoured() override = default;

	void markAllPointsAsNew();

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

	/** Inserts a new point into the point cloud. */
	void push_back(
		float x, float y, float z, float R, float G, float B, float A = 1);

	/** inserts a new point */
	void insertPoint(const mrpt::math::TPointXYZfRGBAu8& p);

	/** Set the number of points, with undefined contents */
	void resize(size_t N)
	{
		m_points.resize(N);
		m_point_colors.resize(N);
		markAllPointsAsNew();
		CRenderizable::notifyChange();
	}

	/** Like STL std::vector's reserve */
	void reserve(size_t N)
	{
		m_points.reserve(N);
		m_point_colors.reserve(N);
	}

	const mrpt::math::TPoint3Df& getPoint3Df(size_t i) const
	{
		return m_points[i];
	}

	/** Write an individual point (checks for "i" in the valid range only in
	 * Debug). */
	void setPoint(size_t i, const mrpt::math::TPointXYZfRGBAu8& p);

	/** Like \a setPoint() but does not check for index out of bounds */
	void setPoint_fast(const size_t i, const mrpt::math::TPointXYZfRGBAu8& p)
	{
		m_points[i] = p.pt;
		m_point_colors[i] = mrpt::img::TColor(p.r, p.g, p.b, p.a);
		markAllPointsAsNew();
	}

	/** Like \a setPoint() but does not check for index out of bounds */
	void setPoint_fast(
		const size_t i, const float x, const float y, const float z)
	{
		m_points[i] = {x, y, z};
		markAllPointsAsNew();
	}

	/** Like \c setPointColor but without checking for out-of-index erors */
	void setPointColor_fast(
		size_t index, float R, float G, float B, float A = 1)
	{
		m_point_colors[index].R = f2u8(R);
		m_point_colors[index].G = f2u8(G);
		m_point_colors[index].B = f2u8(B);
		m_point_colors[index].A = f2u8(A);
	}
	void setPointColor_u8_fast(
		size_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 0xff)
	{
		m_point_colors[index].R = r;
		m_point_colors[index].G = g;
		m_point_colors[index].B = b;
		m_point_colors[index].A = a;
	}
	/** Like \c getPointColor but without checking for out-of-index erors */
	void getPointColor_fast(size_t index, float& R, float& G, float& B) const
	{
		R = u8tof(m_point_colors[index].R);
		G = u8tof(m_point_colors[index].G);
		B = u8tof(m_point_colors[index].B);
	}
	void getPointColor_fast(
		size_t index, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		r = m_point_colors[index].R;
		g = m_point_colors[index].B;
		b = m_point_colors[index].B;
	}
	mrpt::img::TColor getPointColor(size_t index) const
	{
		return m_point_colors[index];
	}

	/** Return the number of points */
	size_t size() const { return m_points.size(); }

	bool empty() const { return m_points.empty(); }

	/** Erase all the points */
	void clear()
	{
		m_points.clear();
		m_point_colors.clear();
		markAllPointsAsNew();
		CRenderizable::notifyChange();
	}

	/** Load the points from any other point map class supported by the adapter
	 * mrpt::opengl::PointCloudAdapter. */
	template <class POINTSMAP>
	void loadFromPointsMap(const POINTSMAP* themap);
	// Must be implemented at the end of the header.

	/** Get the number of elements actually rendered in the last render event.
	 */
	size_t getActuallyRendered() const { return m_last_rendered_count; }
	/** @} */

	/** @name Modify the appearance of the rendered points
		@{ */

	/** Regenerates the color of each point according the one coordinate
	 * (coord_index:0,1,2 for X,Y,Z) and the given color map. */
	void recolorizeByCoordinate(
		const float coord_min, const float coord_max, const int coord_index = 2,
		const mrpt::img::TColormap color_map = mrpt::img::cmJET);

	/** @} */

	/** Render a subset of points (required by octree renderer) */
	void render_subset(
		const bool all, const std::vector<size_t>& idxs,
		const float render_area_sqpixels) const;

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
};

/** Specialization
 * mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>  \ingroup
 * mrpt_adapters_grp*/
template <>
class PointCloudAdapter<mrpt::opengl::CPointCloudColoured>
{
   private:
	mrpt::opengl::CPointCloudColoured& m_obj;

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
	PointCloudAdapter(const mrpt::opengl::CPointCloudColoured& obj)
		: m_obj(*const_cast<mrpt::opengl::CPointCloudColoured*>(&obj))
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
		const auto& p = m_obj.getPoint3Df(idx);
		x = p.x;
		y = p.y;
		z = p.z;
	}
	/** Set XYZ coordinates of i'th point */
	void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.setPoint_fast(idx, x, y, z);
	}

	void setInvalidPoint(const size_t idx)
	{
		m_obj.setPoint_fast(idx, 0, 0, 0);
	}

	/** Get XYZ_RGBf coordinates of i'th point */
	template <typename T>
	void getPointXYZ_RGBAf(
		const size_t idx, T& x, T& y, T& z, float& Rf, float& Gf, float& Bf,
		float& Af) const
	{
		const auto& pt = m_obj.getPoint3Df(idx);
		const auto& col = m_obj.getPointColor(idx);
		x = pt.x;
		y = pt.y;
		z = pt.z;
		Rf = u8tof(col.R);
		Gf = u8tof(col.G);
		Bf = u8tof(col.B);
		Af = u8tof(col.A);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	void setPointXYZ_RGBAf(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const float Rf, const float Gf, const float Bf, const float Af)
	{
		m_obj.setPoint(
			idx,
			mrpt::math::TPointXYZfRGBAu8(
				x, y, z, f2u8(Rf), f2u8(Gf), f2u8(Bf), f2u8(Af)));
	}

	/** Get XYZ_RGBu8 coordinates of i'th point */
	template <typename T>
	void getPointXYZ_RGBu8(
		const size_t idx, T& x, T& y, T& z, uint8_t& r, uint8_t& g,
		uint8_t& b) const
	{
		const auto& pt = m_obj.getPoint3Df(idx);
		const auto& col = m_obj.getPointColor(idx);
		x = pt.x;
		y = pt.y;
		z = pt.z;
		r = col.R;
		g = col.G;
		b = col.B;
	}
	/** Set XYZ_RGBu8 coordinates of i'th point */
	void setPointXYZ_RGBu8(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const uint8_t r, const uint8_t g, const uint8_t b,
		const uint8_t a = 0xff)
	{
		m_obj.setPoint_fast(
			idx, mrpt::math::TPointXYZfRGBAu8(x, y, z, r, g, b, a));
	}

	/** Get RGBf color of i'th point */
	void getPointRGBf(const size_t idx, float& r, float& g, float& b) const
	{
		m_obj.getPointColor_fast(idx, r, g, b);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	void setPointRGBf(
		const size_t idx, const float r, const float g, const float b)
	{
		m_obj.setPointColor_fast(idx, r, g, b);
	}

	/** Get RGBu8 color of i'th point */
	void getPointRGBu8(
		const size_t idx, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		m_obj.getPointColor_fast(idx, r, g, b);
	}
	/** Set RGBu8 coordinates of i'th point */
	void setPointRGBu8(
		const size_t idx, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointColor_u8_fast(idx, r, g, b);
	}

};	// end of PointCloudAdapter<mrpt::opengl::CPointCloudColoured>

// After declaring the adapter we can here implement this method:
template <class POINTSMAP>
void CPointCloudColoured::loadFromPointsMap(const POINTSMAP* themap)
{
	CRenderizable::notifyChange();
	mrpt::opengl::PointCloudAdapter<CPointCloudColoured> pc_dst(*this);
	const mrpt::opengl::PointCloudAdapter<POINTSMAP> pc_src(*themap);
	const size_t N = pc_src.size();
	pc_dst.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		if constexpr (mrpt::opengl::PointCloudAdapter<POINTSMAP>::HAS_RGB)
		{
			float x, y, z, r, g, b, a;
			pc_src.getPointXYZ_RGBAf(i, x, y, z, r, g, b, a);
			pc_dst.setPointXYZ_RGBAf(i, x, y, z, r, g, b, a);
		}
		else
		{
			float x, y, z;
			pc_src.getPointXYZ(i, x, y, z);
			pc_dst.setPointXYZ_RGBAf(i, x, y, z, 0, 0, 0, 1);
		}
	}
}
}  // namespace mrpt::opengl
