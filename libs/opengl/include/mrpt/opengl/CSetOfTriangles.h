/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>

namespace mrpt::opengl
{
/** A set of colored triangles, able to draw any solid, arbitrarily complex
 * object without textures. For textures, see CSetOfTexturedTriangles
 *
 * \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class CSetOfTriangles : public CRenderizableShaderTriangles
{
	DEFINE_SERIALIZABLE(CSetOfTriangles, mrpt::opengl)
   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void onUpdateBuffers_Triangles() override;
	/** @} */

	using const_iterator = std::vector<TTriangle>::const_iterator;
	using const_reverse_iterator =
		std::vector<TTriangle>::const_reverse_iterator;

	/** Explicitly updates the internal polygon cache, with all triangles as
	 * polygons. \sa getPolygons() */
	void updatePolygons() const;

	/** Clear this object, removing all triangles. */
	void clearTriangles();

	/** Get triangle count */
	size_t getTrianglesCount() const;

	/** Gets the i-th triangle */
	void getTriangle(size_t idx, TTriangle& t) const
	{
		std::shared_lock<std::shared_mutex> trisReadLock(
			CRenderizableShaderTriangles::m_trianglesMtx.data);

		ASSERT_LT_(idx, shaderTrianglesBuffer().size());
		t = shaderTrianglesBuffer().at(idx);
	}
	/** Inserts a triangle into the set */
	void insertTriangle(const TTriangle& t)
	{
		std::unique_lock<std::shared_mutex> trisLck(
			CRenderizableShaderTriangles::m_trianglesMtx.data);
		auto& tris = CRenderizableShaderTriangles::m_triangles;

		tris.push_back(t);
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}

	/** Inserts a set of triangles, bounded by iterators, into this set.
	 * \sa insertTriangle
	 */
	template <class InputIterator>
	void insertTriangles(const InputIterator& begin, const InputIterator& end)
	{
		std::unique_lock<std::shared_mutex> trisLck(
			CRenderizableShaderTriangles::m_trianglesMtx.data);
		auto& tris = CRenderizableShaderTriangles::m_triangles;

		tris.insert(tris.end(), begin, end);
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}

	/** Inserts an existing CSetOfTriangles into this one */
	void insertTriangles(const CSetOfTriangles::Ptr& p);

	/**
	 * Reserves memory for certain number of triangles, avoiding multiple
	 * memory allocation calls.
	 */
	inline void reserve(size_t t)
	{
		std::unique_lock<std::shared_mutex> trisLck(
			CRenderizableShaderTriangles::m_trianglesMtx.data);
		auto& tris = CRenderizableShaderTriangles::m_triangles;

		tris.reserve(t);
		CRenderizable::notifyChange();
	}

	/** Overwrite all triangles colors with the one provided */
	CRenderizable& setColor_u8(const mrpt::img::TColor& c) override;
	/** Overwrite all triangles colors with the one provided */
	CRenderizable& setColorR_u8(const uint8_t r) override;
	/** Overwrite all triangles colors with the one provided */
	CRenderizable& setColorG_u8(const uint8_t g) override;
	/** Overwrite all triangles colors with the one provided */
	CRenderizable& setColorB_u8(const uint8_t b) override;
	/** Overwrite all triangles colors with the one provided */
	CRenderizable& setColorA_u8(const uint8_t a) override;

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/**
	 * Gets the polygon cache.
	 * \sa insertTriangles
	 */
	void getPolygons(std::vector<mrpt::math::TPolygon3D>& polys) const;

	/**
	 * Inserts a set of triangles, given in a container of either TTriangle's
	 * or TPolygon3D
	 * \sa insertTriangle
	 */
	template <class CONTAINER>
	inline void insertTriangles(const CONTAINER& c)
	{
		this->insertTriangles(c.begin(), c.end());
		CRenderizable::notifyChange();
	}

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

	CSetOfTriangles() = default;
	virtual ~CSetOfTriangles() override = default;

   protected:
	/**
	 * Mutable variable used to check whether polygons need to be recalculated.
	 */
	mutable bool polygonsUpToDate{false};

	/** Polygon cache, used for ray-tracing only */
	mutable std::vector<mrpt::math::TPolygonWithPlane> m_polygons;
};
/** Inserts a set of triangles into the list; note that this method allows to
 * pass another CSetOfTriangles as argument. Allows call chaining.
 * \sa mrpt::opengl::CSetOfTriangles::insertTriangle
 */
template <class T>
inline CSetOfTriangles::Ptr& operator<<(CSetOfTriangles::Ptr& s, const T& t)
{
	s->insertTriangles(t.begin(), t.end());
	return s;
}
/** Inserts a triangle into the list. Allows call chaining.
 * \sa mrpt::opengl::CSetOfTriangles::insertTriangle
 */
template <>
inline CSetOfTriangles::Ptr& operator<<(
	CSetOfTriangles::Ptr& s, const mrpt::opengl::TTriangle& t)
{
	s->insertTriangle(t);
	return s;
}
}  // namespace mrpt::opengl
