/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/math/TSegment3D.h>
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A set of independent lines (or segments), one line with its own start and
 * end positions (X,Y,Z). Optionally, the vertices can be also shown as dots.
 *
 * ![mrpt::viz::CSetOfLines](preview_CSetOfLines.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CSetOfLines :
    virtual public CVisualObject,
    public VisualObjectParams_Points,
    public VisualObjectParams_Lines
{
  DEFINE_SERIALIZABLE(CSetOfLines, mrpt::viz)
 protected:
  std::vector<mrpt::math::TSegment3D> m_Segments;

 public:
  /** Clear the list of segments */
  void clear()
  {
    m_Segments.clear();
    CVisualObject::notifyChange();
  }

  [[nodiscard]] float getVerticesPointSize() const
  {
    return VisualObjectParams_Points::getPointSize();
  }

  /** Enable showing vertices as dots if size_points>0 */
  void setVerticesPointSize(const float size_points)
  {
    VisualObjectParams_Points::setPointSize(size_points);
  }

  /** Appends a line to the set. */
  void appendLine(const mrpt::math::TSegment3D& sgm)
  {
    m_Segments.push_back(sgm);
    CVisualObject::notifyChange();
  }

  /** Appends a line to the set, given the coordinates of its bounds. */
  void appendLine(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    appendLine(
        mrpt::math::TSegment3D(mrpt::math::TPoint3D(x0, y0, z0), mrpt::math::TPoint3D(x1, y1, z1)));
    CVisualObject::notifyChange();
  }

  /** Appends a line whose starting point is the end point of the last line
   * (similar to OpenGL's GL_LINE_STRIP)
   *  \exception std::exception If there is no previous segment */
  void appendLineStrip(float x, float y, float z)
  {
    ASSERT_(!this->empty());
    this->appendLine(this->rbegin()->point2, mrpt::math::TPoint3D(x, y, z));
  }

  //! \overload
  template <class U>
  void appendLineStrip(const U& point)
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
  void appendLines(const T& sgms)
  {
    m_Segments.insert(m_Segments.end(), sgms.begin(), sgms.end());
    CVisualObject::notifyChange();
  }
  /**
   * Appends certain amount of lines, located between two iterators, into the
   * set.
   * \sa appendLine
   */
  template <class T_it>
  void appendLines(const T_it& begin, const T_it& end)
  {
    m_Segments.reserve(m_Segments.size() + (end - begin));
    m_Segments.insert(m_Segments.end(), begin, end);
    CVisualObject::notifyChange();
  }
  /**
   * Resizes the set.
   * \sa reserve
   */
  void resize(size_t nLines)
  {
    m_Segments.resize(nLines);
    CVisualObject::notifyChange();
  }
  /**
   * Reserves an amount of lines to the set. This method should be used when
   * some known amount of lines is going to be inserted, so that only a memory
   * allocation is needed.
   * \sa resize
   */
  void reserve(size_t r)
  {
    m_Segments.reserve(r);
    CVisualObject::notifyChange();
  }
  /**
   * Inserts a line, given its bounds. Works with any pair of objects with
   * access to x, y and z members.
   */
  template <class T, class U>
  void appendLine(T p0, U p1)
  {
    appendLine(p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);
    CVisualObject::notifyChange();
  }

  /** Returns the total count of lines in this set. */
  [[nodiscard]] size_t size() const { return m_Segments.size(); }

  /** Returns true if there are no line segments. */
  [[nodiscard]] bool empty() const { return m_Segments.empty(); }

  /** Sets a specific line in the set, given its index. \sa appendLine */
  void setLineByIndex(size_t index, const mrpt::math::TSegment3D& segm);

  /** Sets a specific line in the set, given its index. \sa appendLine */
  void setLineByIndex(
      size_t index, double x0, double y0, double z0, double x1, double y1, double z1)
  {
    setLineByIndex(
        index,
        mrpt::math::TSegment3D(mrpt::math::TPoint3D(x0, y0, z0), mrpt::math::TPoint3D(x1, y1, z1)));
    CVisualObject::notifyChange();
  }

  /** Gets a specific line in the set, given its index. \sa getLineByIndex */
  void getLineByIndex(
      size_t index, double& x0, double& y0, double& z0, double& x1, double& y1, double& z1) const;

  // Iterator management
  using iterator = std::vector<mrpt::math::TSegment3D>::iterator;
  using reverse_iterator = std::vector<mrpt::math::TSegment3D>::reverse_iterator;
  using const_iterator = std::vector<mrpt::math::TSegment3D>::const_iterator;
  using const_reverse_iterator = std::vector<mrpt::math::TSegment3D>::const_reverse_iterator;
  /**
   * Beginning const iterator.
   * \sa end,rbegin,rend
   */
  [[nodiscard]] const_iterator begin() const { return m_Segments.begin(); }
  [[nodiscard]] iterator begin()
  {
    CVisualObject::notifyChange();
    return m_Segments.begin();
  }
  /**
   * Ending const iterator.
   * \sa begin,rend,rbegin
   */
  [[nodiscard]] const_iterator end() const { return m_Segments.end(); }
  [[nodiscard]] iterator end()
  {
    CVisualObject::notifyChange();
    return m_Segments.end();
  }
  /**
   * Beginning const reverse iterator (actually, accesses the end of the
   * set).
   * \sa rend,begin,end
   */
  [[nodiscard]] const_reverse_iterator rbegin() const { return m_Segments.rbegin(); }
  /**
   * Ending const reverse iterator (actually, refers to the starting point of
   * the set).
   * \sa rbegin,end,begin
   */
  [[nodiscard]] const_reverse_iterator rend() const { return m_Segments.rend(); }

  /** Evaluates the bounding box of this object (including possible children)
   * in the coordinate frame of the object parent. */
  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Constructor */
  CSetOfLines();
  /** Constructor with a initial set of lines. */
  CSetOfLines(const std::vector<mrpt::math::TSegment3D>& sgms, bool antiAliasing = true);
  /** Private, virtual destructor: only can be deleted from smart pointers. */
  ~CSetOfLines() override = default;
};
/** Inserts a set of segments into the list. Allows call chaining.
 * \sa mrpt::viz::CSetOfLines::appendLines
 */
template <class T>
CSetOfLines::Ptr& operator<<(CSetOfLines::Ptr& l, const T& s)
{
  l->appendLines(s.begin(), s.end());
  return l;
}
}  // namespace mrpt::viz
