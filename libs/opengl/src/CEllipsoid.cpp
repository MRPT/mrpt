/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CEllipsoid, CRenderizableDisplayList, mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CEllipsoid::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	const size_t dim = m_cov.cols();

	if (m_eigVal(0, 0) != 0.0 && m_eigVal(1, 1) != 0.0 &&
		(dim == 2 || m_eigVal(2, 2) != 0.0) && m_quantiles != 0.0)
	{
		glEnable(GL_BLEND);
		CHECK_OPENGL_ERROR();
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		CHECK_OPENGL_ERROR();
		glLineWidth(m_lineWidth);
		CHECK_OPENGL_ERROR();

		if (dim == 2)
		{
			glDisable(GL_LIGHTING);  // Disable lights when drawing lines

			// ---------------------
			//     2D ellipse
			// ---------------------

			/* Equivalent MATLAB code:
			 *
			 * q=1;
			 * [vec val]=eig(C);
			 * M=(q*val*vec)';
			 * R=M*[x;y];
			 * xx=R(1,:);yy=R(2,:);
			 * plot(xx,yy), axis equal;
			 */

			double ang;
			unsigned int i;

			// Compute the new vectors for the ellipsoid:
			auto M = CMatrixDouble(m_eigVal.asEigen() * m_eigVec.transpose());
			M *= double(m_quantiles);

			glBegin(GL_LINE_LOOP);

			// Compute the points of the 2D ellipse:
			for (i = 0, ang = 0; i < m_2D_segments;
				 i++, ang += (M_2PI / m_2D_segments))
			{
				double ccos = cos(ang);
				double ssin = sin(ang);

				const float x = ccos * M(0, 0) + ssin * M(1, 0);
				const float y = ccos * M(0, 1) + ssin * M(1, 1);

				glVertex2f(x, y);
			}  // end for points on ellipse

			glEnd();

			// 2D: Save bounding box:
			const double max_radius =
				m_quantiles * std::max(m_eigVal(0, 0), m_eigVal(1, 1));
			m_bb_min = mrpt::math::TPoint3D(-max_radius, -max_radius, 0);
			m_bb_max = mrpt::math::TPoint3D(max_radius, max_radius, 0);
			// Convert to coordinates of my parent:
			m_pose.composePoint(m_bb_min, m_bb_min);
			m_pose.composePoint(m_bb_max, m_bb_max);

			glEnable(GL_LIGHTING);
		}
		else
		{
			// ---------------------
			//    3D ellipsoid
			// ---------------------
			GLfloat mat[16];

			//  A homogeneous transformation matrix, in this order:
			//
			//     0  4  8  12
			//     1  5  9  13
			//     2  6  10 14
			//     3  7  11 15
			//
			mat[3] = mat[7] = mat[11] = 0;
			mat[15] = 1;
			mat[12] = mat[13] = mat[14] = 0;

			mat[0] = m_eigVec(0, 0);
			mat[1] = m_eigVec(1, 0);
			mat[2] = m_eigVec(2, 0);  // New X-axis
			mat[4] = m_eigVec(0, 1);
			mat[5] = m_eigVec(1, 1);
			mat[6] = m_eigVec(2, 1);  // New X-axis
			mat[8] = m_eigVec(0, 2);
			mat[9] = m_eigVec(1, 2);
			mat[10] = m_eigVec(2, 2);  // New X-axis

			GLUquadricObj* obj = gluNewQuadric();
			CHECK_OPENGL_ERROR();

			if (!m_drawSolid3D)
				glDisable(GL_LIGHTING);  // Disable lights when drawing lines

			gluQuadricDrawStyle(obj, m_drawSolid3D ? GLU_FILL : GLU_LINE);

			glPushMatrix();
			glMultMatrixf(mat);
			glScalef(
				m_eigVal(0, 0) * m_quantiles, m_eigVal(1, 1) * m_quantiles,
				m_eigVal(2, 2) * m_quantiles);

			gluSphere(obj, 1, m_3D_segments, m_3D_segments);
			CHECK_OPENGL_ERROR();

			glPopMatrix();

			gluDeleteQuadric(obj);
			CHECK_OPENGL_ERROR();

			// 3D: Save bounding box:
			const double max_radius =
				m_quantiles *
				std::max(
					m_eigVal(0, 0), std::max(m_eigVal(1, 1), m_eigVal(2, 2)));
			m_bb_min = mrpt::math::TPoint3D(-max_radius, -max_radius, 0);
			m_bb_max = mrpt::math::TPoint3D(max_radius, max_radius, 0);
			// Convert to coordinates of my parent:
			m_pose.composePoint(m_bb_min, m_bb_min);
			m_pose.composePoint(m_bb_max, m_bb_max);
		}

		glDisable(GL_BLEND);

		glEnable(GL_LIGHTING);
	}
	MRPT_END_WITH_CLEAN_UP(cout << "Covariance matrix leading to error is:"
								<< endl
								<< m_cov << endl;);
#endif
}

uint8_t CEllipsoid::serializeGetVersion() const { return 1; }
void CEllipsoid::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_cov << m_drawSolid3D << m_quantiles << (uint32_t)m_2D_segments
		<< (uint32_t)m_3D_segments << m_lineWidth;
}

void CEllipsoid::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			uint32_t i;
			readFromStreamRender(in);
			if (version == 0)
			{
				CMatrixF c;
				in >> c;
				m_cov = c.cast_double();
			}
			else
			{
				in >> m_cov;
			}

			in >> m_drawSolid3D >> m_quantiles;
			in >> i;
			m_2D_segments = i;
			in >> i;
			m_3D_segments = i;
			in >> m_lineWidth;

			// Update cov. matrix cache:
			setCovMatrix(m_cov);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizableDisplayList::notifyChange();
}

bool quickSolveEqn(double a, double b_2, double c, double& t)
{
	double delta = square(b_2) - a * c;
	if (delta == 0)
		return (t = -b_2 / a) >= 0;
	else if (delta > 0)
	{
		delta = sqrt(delta);
		if ((t = (-b_2 - delta) / a) >= 0)
			return true;
		else
			return (t = (-b_2 + delta) / a) >= 0;
	}
	else
		return false;
}

bool CEllipsoid::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	if (m_cov.rows() != 3) return false;
	TLine3D lin, lin2;
	createFromPoseX((o - this->m_pose).asTPose(), lin);
	lin.unitarize();  // By adding this line, distance from any point of the
	// line to its base is exactly equal to the "t".
	for (size_t i = 0; i < 3; i++)
	{
		lin2.pBase[i] = 0;
		lin2.director[i] = 0;
		for (size_t j = 0; j < 3; j++)
		{
			double vji = m_eigVec(j, i);
			lin2.pBase[i] += vji * lin.pBase[j];
			lin2.director[i] += vji * lin.director[j];
		}
	}
	double a = 0, b_2 = 0, c = -square(m_quantiles);
	for (size_t i = 0; i < 3; i++)
	{
		double ev = m_eigVal(i, i);
		a += square(lin2.director[i] / ev);
		b_2 += lin2.director[i] * lin2.pBase[i] / square(ev);
		c += square(lin2.pBase[i] / ev);
	}
	return quickSolveEqn(a, b_2, c, dist);
}

void CEllipsoid::setCovMatrix(
	const mrpt::math::CMatrixDouble& m, int resizeToSize)
{
	MRPT_START

	ASSERT_(m.cols() == m.rows());
	ASSERT_(
		m.rows() == 2 || m.rows() == 3 ||
		(resizeToSize > 0 && (resizeToSize == 2 || resizeToSize == 3)));

	m_cov = m;
	if (resizeToSize > 0 && resizeToSize < (int)m.rows())
		m_cov.setSize(resizeToSize, resizeToSize);

	if (m_cov == m_prevComputedCov) return;  // Done.

	m_prevComputedCov = m_cov;

	CRenderizableDisplayList::notifyChange();

	// Handle the special case of an ellipsoid of volume = 0
	const double d = m_cov.det();
	if (d == 0 || d != d)  // Note: "d!=d" is a great test for invalid numbers,
	// don't remove!
	{
		// All zeros:
		m_eigVec.setZero(3, 3);
		m_eigVal.setZero(3, 3);
	}
	else
	{
		// Not null matrix: compute the eigen-vectors & values:
		std::vector<double> eigvals;
		if (m_cov.eig_symmetric(m_eigVec, eigvals))
		{
			// Do the scale at render to avoid recomputing the m_eigVal for
			// different m_quantiles
			m_eigVal.setDiagonal(eigvals);
			m_eigVal.array() = m_eigVal.array().sqrt().matrix();
		}
		else
		{
			m_eigVec.setZero(3, 3);
			m_eigVal.setZero(3, 3);
		}
	}

	MRPT_END
}

void CEllipsoid::setCovMatrix(
	const mrpt::math::CMatrixFloat& m, int resizeToSize)
{
	CRenderizableDisplayList::notifyChange();
	setCovMatrix(CMatrixDouble(m), resizeToSize);
}

/** Evaluates the bounding box of this object (including possible children) in
 * the coordinate frame of the object parent. */
void CEllipsoid::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = m_bb_min;
	bb_max = m_bb_max;
}
