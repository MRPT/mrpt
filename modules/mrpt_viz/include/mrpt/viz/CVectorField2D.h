/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A 2D vector field representation, consisting of points and arrows drawn on a
 * plane (invisible grid).
 *
 * ![mrpt::viz::CVectorField2D](preview_CVectorField2D.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */

class CVectorField2D :
    virtual public CVisualObject,
    public VisualObjectParams_Lines,
    public VisualObjectParams_Points,
    public VisualObjectParams_Triangles
{
  DEFINE_SERIALIZABLE(CVectorField2D, mrpt::viz)
 protected:
  /** X component of the vector field */
  mrpt::math::CMatrixF xcomp;
  /** Y component of the vector field */
  mrpt::math::CMatrixF ycomp;

  /** Grid bounds */
  float xMin{-1.0f}, xMax{1.0f}, yMin{-1.0f}, yMax{1.0f};

  mrpt::img::TColor m_point_color;
  mrpt::img::TColor m_field_color;

 public:
  /**
   * Clear the matrices
   */
  void clear()
  {
    xcomp.resize(0, 0);
    ycomp.resize(0, 0);
    CVisualObject::notifyChange();
  }

  /**
   * Set the point color in the range [0,1]
   */
  void setPointColor(const float R, const float G, const float B, const float A = 1)
  {
    m_point_color = mrpt::img::TColor(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
    CVisualObject::notifyChange();
  }

  /**
   * Get the point color in the range [0,1]
   */
  mrpt::img::TColorf getPointColor() const { return mrpt::img::TColorf(m_point_color); }

  /**
   * Set the arrow color in the range [0,1]
   */
  void setVectorFieldColor(const float R, const float G, const float B, const float A = 1)
  {
    m_field_color = mrpt::img::TColor(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
    CVisualObject::notifyChange();
  }

  /**
   * Get the arrow color in the range [0,1]
   */
  mrpt::img::TColorf getVectorFieldColor() const { return mrpt::img::TColorf(m_field_color); }

  /**
   * Set the coordinates of the grid on where the vector field will be drawn
   * by setting its center and the cell size.
   * The number of cells is marked by the content of xcomp and ycomp.
   * \sa xcomp, ycomp
   */
  void setGridCenterAndCellSize(
      const float center_x, const float center_y, const float cellsize_x, const float cellsize_y)
  {
    xMin = center_x - 0.5f * cellsize_x * (xcomp.cols() - 1);
    xMax = center_x + 0.5f * cellsize_x * (xcomp.cols() - 1);
    yMin = center_y - 0.5f * cellsize_y * (xcomp.rows() - 1);
    yMax = center_y + 0.5f * cellsize_y * (xcomp.rows() - 1);
    CVisualObject::notifyChange();
  }

  /**
   * Set the coordinates of the grid on where the vector field will be drawn
   * using x-y max and min values.
   */
  void setGridLimits(const float xmin, const float xmax, const float ymin, const float ymax)
  {
    xMin = xmin;
    xMax = xmax;
    yMin = ymin;
    yMax = ymax;
    CVisualObject::notifyChange();
  }

  /**
   * Get the coordinates of the grid on where the vector field is drawn using
   * the max and min values.
   */
  void getGridLimits(float& xmin, float& xmax, float& ymin, float& ymax) const
  {
    xmin = xMin;
    xmax = xMax;
    ymin = yMin;
    ymax = yMax;
  }

  /**
   * Get the vector field. Matrix_x stores the "x" component and Matrix_y
   * stores the "y" component.
   */
  void getVectorField(mrpt::math::CMatrixFloat& Matrix_x, mrpt::math::CMatrixFloat& Matrix_y) const
  {
    Matrix_x = xcomp;
    Matrix_y = ycomp;
  }

  /** Get the "x" component of the vector field, as a matrix where each entry
   * represents a point in the 2D grid. */
  const mrpt::math::CMatrixFloat& getVectorField_x() const { return xcomp; }
  /** \overload */
  mrpt::math::CMatrixFloat& getVectorField_x() { return xcomp; }
  /** Get the "y" component of the vector field, as a matrix where each entry
   * represents a point in the 2D grid. */
  const mrpt::math::CMatrixFloat& getVectorField_y() const { return ycomp; }
  /** \overload */
  mrpt::math::CMatrixFloat& getVectorField_y() { return ycomp; }
  /**
   * Set the vector field. Matrix_x contains the "x" component and Matrix_y
   * contains the "y" component.
   */
  void setVectorField(mrpt::math::CMatrixFloat& Matrix_x, mrpt::math::CMatrixFloat& Matrix_y)
  {
    ASSERT_((Matrix_x.rows() == Matrix_y.rows()) && (Matrix_x.cols() == Matrix_y.cols()));
    xcomp = Matrix_x;
    ycomp = Matrix_y;
    CVisualObject::notifyChange();
  }

  /**
   * Adjust the vector field in the scene (vectors magnitude) according to
   * the grid size.
   */
  void adjustVectorFieldToGrid();

  /** Resizes the set.
   */
  void resize(size_t rows, size_t cols)
  {
    xcomp.resize(rows, cols);
    ycomp.resize(rows, cols);
    CVisualObject::notifyChange();
  }

  /** Returns the total count of rows used to represent the vector field. */
  size_t cols() const { return xcomp.cols(); }
  /** Returns the total count of columns used to represent the vector field.
   */
  size_t rows() const { return xcomp.rows(); }

  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Constructor */
  CVectorField2D();
  /** Constructor with a initial set of lines. */
  CVectorField2D(
      mrpt::math::CMatrixFloat Matrix_x,
      mrpt::math::CMatrixFloat Matrix_y,
      float xmin = -1,
      float xmax = 1,
      float ymin = -1,
      float ymax = 1);
  /** Private, virtual destructor: only can be deleted from smart pointers. */
  ~CVectorField2D() override = default;
};

}  // namespace mrpt::viz
