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

namespace mrpt::opengl
{
/** A set of colored triangles.
 *  This class can be used to draw any solid, arbitrarily complex object
 * (without textures).
 *  \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class CSetOfTriangles : public CRenderizable
{
	DEFINE_SERIALIZABLE(CSetOfTriangles, mrpt::opengl)
   public:
	/**
	 * Triangle definition. Each vertex has three spatial coordinates and four
	 * color values.
	 */
	struct TTriangle
	{
		inline TTriangle()
		{
			for (size_t i = 0; i < 3; i++)
			{
				r[i] = g[i] = b[i] = a[i] = 1.0f;
			}
		}
		inline TTriangle(const mrpt::math::TPolygon3D& p)
		{
			ASSERT_(p.size() == 3);
			for (size_t i = 0; i < 3; i++)
			{
				x[i] = p[i].x;
				y[i] = p[i].y;
				z[i] = p[i].z;
				r[i] = g[i] = b[i] = a[i] = 1.0f;
			}
		}
		float x[3], y[3], z[3];
		float r[3], g[3], b[3], a[3];
	};
	using const_iterator = std::vector<TTriangle>::const_iterator;
	using const_reverse_iterator =
		std::vector<TTriangle>::const_reverse_iterator;

   protected:
	/**
	 * List of triangles.
	 * \sa TTriangle
	 */
	std::vector<TTriangle> m_triangles;
	/**
	 * Transparency enabling.
	 */
	bool m_enableTransparency;
	/**
	 * Mutable variable used to check whether polygons need to be recalculated.
	 */
	mutable bool polygonsUpToDate{false};
	/**
	 * Polygon cache.
	 */
	mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPolygons;

   public:
	/**
	 * Polygon cache updating.
	 */
	void updatePolygons() const;
	/**
	 * Clear this object.
	 */
	inline void clearTriangles()
	{
		m_triangles.clear();
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}
	/**
	 * Get triangle count.
	 */
	inline size_t getTrianglesCount() const { return m_triangles.size(); }
	/**
	 * Gets the triangle in a given position.
	 */
	inline void getTriangle(size_t idx, TTriangle& t) const
	{
		ASSERT_(idx < m_triangles.size());
		t = m_triangles[idx];
	}
	/**
	 * Inserts a triangle into the set.
	 */
	inline void insertTriangle(const TTriangle& t)
	{
		m_triangles.push_back(t);
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}
	/**
	 * Inserts a set of triangles, bounded by iterators, into this set.
	 * \sa insertTriangle
	 */
	template <class InputIterator>
	inline void insertTriangles(
		const InputIterator& begin, const InputIterator& end)
	{
		m_triangles.insert(m_triangles.end(), begin, end);
		polygonsUpToDate = false;
		CRenderizable::notifyChange();
	}
	/**
	 * Inserts an existing CSetOfTriangles into this one.
	 */
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

	CRenderizable& setColor_u8(const mrpt::img::TColor& c) override;
	CRenderizable& setColorR_u8(const uint8_t r) override;
	CRenderizable& setColorG_u8(const uint8_t g) override;
	CRenderizable& setColorB_u8(const uint8_t b) override;
	CRenderizable& setColorA_u8(const uint8_t a) override;

	void render() const override;
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
	CSetOfTriangles::Ptr& s, const CSetOfTriangles::TTriangle& t)
{
	s->insertTriangle(t);
	return s;
}
}  // namespace mrpt::opengl
