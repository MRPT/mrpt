/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TSegment3D.h>
#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
/** A set of independent lines (or segments), one line with its own start and
 * end positions (X,Y,Z).
 * Optionally, the vertices can be also shown as dots.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CSetOfLines </td> <td> \image html
 * preview_CSetOfLines.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CSetOfLines : public CRenderizable
{
	DEFINE_SERIALIZABLE(CSetOfLines, mrpt::opengl)
   protected:
	std::vector<mrpt::math::TSegment3D> mSegments;
	float mLineWidth{1.0};
	bool m_antiAliasing{true};
	/** 0: means hidden */
	float m_verticesPointSize{.0f};

   public:
	/**
	 * Clear the list of segments
	 */
	inline void clear()
	{
		mSegments.clear();
		CRenderizable::notifyChange();
	}
	/**
	 * Sets the width with which lines will be drawn.
	 */
	inline void setLineWidth(float w)
	{
		mLineWidth = w;
		CRenderizable::notifyChange();
	}
	/**
	 * Gets the width with which lines are drawn.
	 */
	float getLineWidth() const { return mLineWidth; }
	float getVerticesPointSize() const;
	/** Enable showing vertices as dots if size_points>0 */
	void setVerticesPointSize(const float size_points);
	/**
	 * Appends a line to the set.
	 */
	inline void appendLine(const mrpt::math::TSegment3D& sgm)
	{
		mSegments.push_back(sgm);
		CRenderizable::notifyChange();
	}
	/**
	 * Appends a line to the set, given the coordinates of its bounds.
	 */
	inline void appendLine(
		float x0, float y0, float z0, float x1, float y1, float z1)
	{
		appendLine(mrpt::math::TSegment3D(
			mrpt::math::TPoint3D(x0, y0, z0),
			mrpt::math::TPoint3D(x1, y1, z1)));
		CRenderizable::notifyChange();
	}

	/** Appends a line whose starting point is the end point of the last line
	 * (similar to OpenGL's GL_LINE_STRIP)
	 *  \exception std::exception If there is no previous segment */
	inline void appendLineStrip(float x, float y, float z)
	{
		ASSERT_(!this->empty());
		this->appendLine(this->rbegin()->point2, mrpt::math::TPoint3D(x, y, z));
	}
	//! \overload
	template <class U>
	inline void appendLineStrip(const U& point)
	{
		ASSERT_(!this->empty());
		this->appendLine(this->rbegin()->point2, point);
	}

	/**
	 * Appends any iterable collection of lines to the set. Note that this
	 * includes another CSetOfLines.
	 * \sa appendLine
	 */
	template <class T>
	inline void appendLines(const T& sgms)
	{
		mSegments.insert(mSegments.end(), sgms.begin(), sgms.end());
		CRenderizable::notifyChange();
	}
	/**
	 * Appends certain amount of lines, located between two iterators, into the
	 * set.
	 * \sa appendLine
	 */
	template <class T_it>
	inline void appendLines(const T_it& begin, const T_it& end)
	{
		mSegments.reserve(mSegments.size() + (end - begin));
		mSegments.insert(mSegments.end(), begin, end);
		CRenderizable::notifyChange();
	}
	/**
	 * Resizes the set.
	 * \sa reserve
	 */
	void resize(size_t nLines)
	{
		mSegments.resize(nLines);
		CRenderizable::notifyChange();
	}
	/**
	 * Reserves an amount of lines to the set. This method should be used when
	 * some known amount of lines is going to be inserted, so that only a memory
	 * allocation is needed.
	 * \sa resize
	 */
	void reserve(size_t r)
	{
		mSegments.reserve(r);
		CRenderizable::notifyChange();
	}
	/**
	 * Inserts a line, given its bounds. Works with any pair of objects with
	 * access to x, y and z members.
	 */
	template <class T, class U>
	inline void appendLine(T p0, U p1)
	{
		appendLine(p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);
		CRenderizable::notifyChange();
	}
	/** Returns the total count of lines in this set. */
	inline size_t getLineCount() const { return mSegments.size(); }
	/** Returns the total count of lines in this set. */
	inline size_t size() const { return mSegments.size(); }
	/** Returns true if there are no line segments. */
	inline bool empty() const { return mSegments.empty(); }
	/**
	 * Sets a specific line in the set, given its index.
	 * \sa appendLine
	 */
	void setLineByIndex(size_t index, const mrpt::math::TSegment3D& segm);
	/**
	 * Sets a specific line in the set, given its index.
	 * \sa appendLine
	 */
	inline void setLineByIndex(
		size_t index, double x0, double y0, double z0, double x1, double y1,
		double z1)
	{
		setLineByIndex(
			index, mrpt::math::TSegment3D(
					   mrpt::math::TPoint3D(x0, y0, z0),
					   mrpt::math::TPoint3D(x1, y1, z1)));
		CRenderizable::notifyChange();
	}
	/**
	 * Gets a specific line in the set, given its index.
	 * \sa getLineByIndex
	 */
	void getLineByIndex(
		size_t index, double& x0, double& y0, double& z0, double& x1,
		double& y1, double& z1) const;

	void render() const override;
	void renderUpdateBuffers() const override;

	// Iterator management
	using iterator = std::vector<mrpt::math::TSegment3D>::iterator;
	using reverse_iterator =
		std::vector<mrpt::math::TSegment3D>::reverse_iterator;
	using const_iterator = std::vector<mrpt::math::TSegment3D>::const_iterator;
	using const_reverse_iterator =
		std::vector<mrpt::math::TSegment3D>::const_reverse_iterator;
	/**
	 * Beginning const iterator.
	 * \sa end,rbegin,rend
	 */
	inline const_iterator begin() const { return mSegments.begin(); }
	inline iterator begin()
	{
		CRenderizable::notifyChange();
		return mSegments.begin();
	}
	/**
	 * Ending const iterator.
	 * \sa begin,rend,rbegin
	 */
	inline const_iterator end() const { return mSegments.end(); }
	inline iterator end()
	{
		CRenderizable::notifyChange();
		return mSegments.end();
	}
	/**
	 * Beginning const reverse iterator (actually, accesses the end of the
	 * set).
	 * \sa rend,begin,end
	 */
	inline const_reverse_iterator rbegin() const { return mSegments.rbegin(); }
	/**
	 * Ending const reverse iterator (actually, refers to the starting point of
	 * the set).
	 * \sa rbegin,end,begin
	 */
	inline const_reverse_iterator rend() const { return mSegments.rend(); }
	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	void enableAntiAliasing(bool enable = true)
	{
		m_antiAliasing = enable;
		CRenderizable::notifyChange();
	}
	bool isAntiAliasingEnabled() const { return m_antiAliasing; }
	/** Constructor */
	CSetOfLines();
	/** Constructor with a initial set of lines. */
	CSetOfLines(
		const std::vector<mrpt::math::TSegment3D>& sgms,
		bool antiAliasing = true);
	/** Private, virtual destructor: only can be deleted from smart pointers. */
	~CSetOfLines() override = default;
};
/** Inserts a set of segments into the list. Allows call chaining.
 * \sa mrpt::opengl::CSetOfLines::appendLines
 */
template <class T>
inline CSetOfLines::Ptr& operator<<(CSetOfLines::Ptr& l, const T& s)
{
	l->appendLines(s.begin(), s.end());
	return l;
}
/** Inserts a segment into the list. Allows call chaining.
 * \sa mrpt::opengl::CSetOfLines::appendLine(const TSegment &)
 */
template <>
inline CSetOfLines::Ptr& operator<<(
	CSetOfLines::Ptr& l, const mrpt::math::TSegment3D& s)
{
	l->appendLine(s);
	return l;
}
}  // namespace mrpt::opengl
