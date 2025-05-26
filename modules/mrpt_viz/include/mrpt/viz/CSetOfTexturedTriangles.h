/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A set of textured triangles.
 *  This class can be used to draw any solid, arbitrarily complex object with
 * textures.
 *  \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CSetOfTexturedTriangles :
    virtual public CVisualObject,
    public VisualObjectParams_TexturedTriangles
{
  DEFINE_SERIALIZABLE(CSetOfTexturedTriangles, mrpt::viz)

 public:
  using TVertex = mrpt::viz::TTriangle::Vertex;
  using TTriangle = mrpt::viz::TTriangle;

  CSetOfTexturedTriangles() = default;
  virtual ~CSetOfTexturedTriangles() override = default;

  /** Evaluates the bounding box of this object (including possible children)
   * in the coordinate frame of the object parent. */
  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  void clearTriangles()
  {
    {
      std::unique_lock<std::shared_mutex> writeLock(m_trianglesMtx.data);
      m_triangles.clear();
    }
    CVisualObject::notifyChange();
  }
  size_t getTrianglesCount() const
  {
    std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);
    return m_triangles.size();
  }
  TTriangle getTriangle(size_t idx) const
  {
    std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);
    ASSERT_LT_(idx, m_triangles.size());
    return m_triangles[idx];
  }
  void getTriangle(size_t idx, TTriangle& t) const
  {
    std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);
    ASSERT_LT_(idx, m_triangles.size());
    t = m_triangles[idx];
    CVisualObject::notifyChange();
  }
  void insertTriangle(const TTriangle& t)
  {
    {
      std::unique_lock<std::shared_mutex> writeLock(m_trianglesMtx.data);
      m_triangles.push_back(t);
    }
    CVisualObject::notifyChange();
  }

  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
};

}  // namespace mrpt::viz
