/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/opengl.h>  // Precompiled header


#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_matrices.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CEllipsoid, CRenderizableDisplayList, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CEllipsoid::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

    const size_t dim = m_cov.getColCount();

	if(m_eigVal(0,0) != 0.0 && m_eigVal(1,1) != 0.0 && (dim==2 || m_eigVal(2,2) != 0.0) && m_quantiles!=0.0)
	{
		glEnable(GL_BLEND);
		checkOpenGLError();
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		checkOpenGLError();
	    	glLineWidth(m_lineWidth);
		checkOpenGLError();

		if (dim==2)
		{
			// ---------------------
			//     2D ellipse
			// ---------------------
			float			x1=0,y1=0,x2=0,y2=0;
			double			ang;
			unsigned int	i;

			// Compute the new vectors for the ellipsoid:
			CMatrixDouble 	M;
			M.noalias() = double(m_quantiles) * m_eigVal * m_eigVec.adjoint();

			glBegin( GL_LINES );

			// Compute the points of the 2D ellipse:
			for (i=0,ang=0;i<m_2D_segments;i++,ang+= (M_2PI/(m_2D_segments-1)))
			{
				double ccos = cos(ang);
				double ssin = sin(ang);

				x2 = ccos * M.get_unsafe(0,0) + ssin * M.get_unsafe(1,0);
				y2 = ccos * M.get_unsafe(0,1) + ssin * M.get_unsafe(1,1);

				if (i>0)
				{
					glVertex2f( x1,y1 );
					glVertex2f( x2,y2 );
				}

				x1 = x2;
				y1 = y2;
			} // end for points on ellipse

			glEnd();
		}
		else
		{
			// ---------------------
			//    3D ellipsoid
			// ---------------------
			GLfloat		mat[16];

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

			mat[0] = m_eigVec(0,0); mat[1] = m_eigVec(1,0); mat[2] = m_eigVec(2,0);	// New X-axis
			mat[4] = m_eigVec(0,1); mat[5] = m_eigVec(1,1); mat[6] = m_eigVec(2,1);	// New X-axis
			mat[8] = m_eigVec(0,2); mat[9] = m_eigVec(1,2); mat[10] = m_eigVec(2,2);	// New X-axis

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
			glEnable(GL_COLOR_MATERIAL);
			glShadeModel(GL_SMOOTH);

			GLUquadricObj	*obj = gluNewQuadric();
			checkOpenGLError();

			gluQuadricDrawStyle( obj, m_drawSolid3D ? GLU_FILL : GLU_LINE);

			glPushMatrix();
			glMultMatrixf( mat );
        		glScalef(m_eigVal(0,0)*m_quantiles,m_eigVal(1,1)*m_quantiles,m_eigVal(2,2)*m_quantiles);

			gluSphere( obj, 1,m_3D_segments,m_3D_segments);
			checkOpenGLError();

			glPopMatrix();

			gluDeleteQuadric(obj);
			checkOpenGLError();

			glDisable(GL_LIGHTING);
			glDisable(GL_LIGHT0);


		}


	    	glLineWidth(1.0f);
		glDisable(GL_BLEND);
	}
	MRPT_END_WITH_CLEAN_UP( \
		cout << "Covariance matrix leading to error is:" << endl << m_cov << endl; \
	);
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CEllipsoid::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_cov << m_drawSolid3D << m_quantiles << (uint32_t)m_2D_segments << (uint32_t)m_3D_segments << m_lineWidth;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CEllipsoid::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t	i;
			readFromStreamRender(in);
			if (version==0)
			{
				CMatrix c;
				in >> c; m_cov = c.cast<double>();
			}
			else
			{
				in >> m_cov;
			}

			in >> m_drawSolid3D >> m_quantiles;
			in >> i; m_2D_segments = i;
			in >> i; m_3D_segments = i;
			in >> m_lineWidth;

			// Update cov. matrix cache:
			m_prevComputedCov = m_cov;
			m_cov.eigenVectors(m_eigVec,m_eigVal);
			m_eigVal.Sqrt();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

bool quickSolveEqn(double a,double b_2,double c,double &t)	{
	double delta=square(b_2)-a*c;
	if (delta==0) return (t=-b_2/a)>=0;
	else if (delta>0)	{
		delta=sqrt(delta);
		if ((t=(-b_2-delta)/a)>=0) return true;
		else return (t=(-b_2+delta)/a)>=0;
	}	else return false;
}

bool CEllipsoid::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	if (m_cov.getRowCount()!=3) return false;
	TLine3D lin,lin2;
	createFromPoseX(o-this->m_pose,lin);
	lin.unitarize();	//By adding this line, distance from any point of the line to its base is exactly equal to the "t".
	for (size_t i=0;i<3;i++)	{
		lin2.pBase[i]=0;
		lin2.director[i]=0;
		for (size_t j=0;j<3;j++)	{
			double vji=m_eigVec(j,i);
			lin2.pBase[i]+=vji*lin.pBase[j];
			lin2.director[i]+=vji*lin.director[j];
		}
	}
	double a=0,b_2=0,c=-square(m_quantiles);
	for (size_t i=0;i<3;i++)	{
		double ev=m_eigVal(i,i);
		a+=square(lin2.director[i]/ev);
		b_2+=lin2.director[i]*lin2.pBase[i]/square(ev);
		c+=square(lin2.pBase[i]/ev);
	}
	return quickSolveEqn(a,b_2,c,dist);
}

void CEllipsoid::setCovMatrix( const mrpt::math::CMatrixDouble &m, int resizeToSize)
{
	MRPT_START

	ASSERT_( m.getColCount() == m.getRowCount() );
	ASSERT_( size(m,1)==2 || size(m,1)==3 || (resizeToSize>0 && (resizeToSize==2 || resizeToSize==3)));

	m_cov = m;
	if (resizeToSize>0 && resizeToSize<(int)size(m,1))
		m_cov.setSize(resizeToSize,resizeToSize);

	if (m_cov==m_prevComputedCov)
		return; // Done.

	CRenderizableDisplayList::notifyChange();

	// Handle the special case of an ellipsoid of volume = 0
	if (m_cov.det()==0)
	{
		// All zeros:
		m_prevComputedCov = m_cov;
		m_eigVec.zeros(3,3);
		m_eigVal.zeros(3,3);
	}
	else
	{
		// Not null matrix: compute the eigen-vectors & values:
		m_prevComputedCov = m_cov;
		m_cov.eigenVectors(m_eigVec,m_eigVal);
		m_eigVal.Sqrt();
		// Do the scale at render to avoid recomputing the m_eigVal for different m_quantiles
	}


	MRPT_END
}

void CEllipsoid::setCovMatrix( const mrpt::math::CMatrixFloat &m, int resizeToSize)
{
	CRenderizableDisplayList::notifyChange();
	setCovMatrix( CMatrixDouble(m), resizeToSize);
}
