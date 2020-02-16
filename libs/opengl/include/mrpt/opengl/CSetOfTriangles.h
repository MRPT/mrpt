/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>

namespace mrpt::opengl
{
/** A set of colored triangles, able to draw any solid, arbitrarily complex
 * object without textures. For textures, see CSetOfTexturedTriangles
 *
 * \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class CSetOfTriangles : public CRenderizable
{
	DEFINE_SERIALIZABLE(CSetOfTriangles, mrpt::opengl)
   public:
	using const_iterator = std::vector<TTriangle>::const_iterator;
	using const_reverse_iterator =
		std::vector<TTriangle>::const_reverse_iterator;

   protected:
	/** List of triangles  \sa TTriangle */
	std::vector<TTriangle> m_triangles;
	mutable unsigned int m_trianglesBuffer = 0, m_vao = 0;

	// Computed in renderUpdateBuffers()
	// Note: a normal per vertex, not per triangle.
	mutable std::vector<mrpt::math::TVector3Df> m_trianglesNormals;
	mutable unsigned int m_normalsBuffer;

	bool m_enableTransparency;  //!< Transparency enabling.

	/**
	 * Mutable variable used to check whether polygons need to be recalculated.
	 */
	mutable bool polygonsUpToDate{false};

	/** Polygon cache, used for ray-tracing only */
	mutable std::vector<mrpt::math::TPolygonWithPlane> m_polygons;

   public:
	/** Explicitly updates the internal polygon cache, with all triangles as
	 * polygons. \sa getPolygons() */
	void updatePolygons() const;

	/** Clear this object, removing all triangles. */
	void clearTriangles()
	{
		m_triangles.clear();
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}

	/** Get triangle count */
	size_t getTrianglesCount() const { return m_triangles.size(); }

	/** Gets the i-th triangle */
	void getTriangle(size_t idx, TTriangle& t) const
	{
		ASSERT_BELOW_(idx, m_triangles.size());
		t = m_triangles[idx];
	}
	/** Inserts a triangle into the set */
	void insertTriangle(const TTriangle& t)
	{
		m_triangles.push_back(t);
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}

	/** Inserts a set of triangles, bounded by iterators, into this set.
	 * \sa insertTriangle
	 */
	template <class InputIterator>
	void insertTriangles(const InputIterator& begin, const InputIterator& end)
	{
		m_triangles.insert(m_triangles.end(), begin, end);
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
		m_triangles.reserve(t);
		CRenderizable::notifyChange();
	}

	/** Enables or disables transparency. */
	inline void enableTransparency(bool v)
	{
		m_enableTransparency = v;
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

	shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TRIANGLES};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
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

	/**
	 * Gets the beginning iterator to this object.
	 */
	inline const_iterator begin() const { return m_triangles.begin(); }
	/**
	 * Gets the ending iterator to this object.
	 */
	inline const_iterator end() const { return m_triangles.end(); }
	/**
	 * Gets the reverse beginning iterator to this object, which points to the
	 * last triangle.
	 */
	inline const_reverse_iterator rbegin() const
	{
		return m_triangles.rbegin();
	}
	/**
	 * Gets the reverse ending iterator to this object, which points to the
	 * beginning of the actual set.
	 */
	inline const_reverse_iterator rend() const { return m_triangles.rend(); }
	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor
	 */
	CSetOfTriangles(bool enableTransparency = false)
		: m_triangles(), m_enableTransparency(enableTransparency)

	{
	}

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CSetOfTriangles() override = default;
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
