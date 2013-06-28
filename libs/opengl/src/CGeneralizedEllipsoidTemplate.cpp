/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/opengl.h>  // Precompiled header


#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

/*---------------------------------------------------------------
						Render: 2D implementation
  ---------------------------------------------------------------*/
namespace mrpt {
namespace opengl {
namespace detail {
template <>
void renderGeneralizedEllipsoidTemplate<2>(
	const std::vector<mrpt::math::CArray<float,2> > & pts,
	const float    lineWidth,
	const uint32_t slices,
	const uint32_t stacks)
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_BLEND);  gl_utils::checkOpenGLError();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);   gl_utils::checkOpenGLError();
	glLineWidth(lineWidth); gl_utils::checkOpenGLError();

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines

	glBegin( GL_LINE_LOOP );
	const size_t N = pts.size();
	for (size_t i=0;i<N;i++)
		glVertex2f( pts[i][0], pts[i][1] );

	glEnd();

	glEnable(GL_LIGHTING);
	glDisable(GL_BLEND);
#endif
}

/*---------------------------------------------------------------
						Render: 3D implementation
  ---------------------------------------------------------------*/
template <>
void renderGeneralizedEllipsoidTemplate<3>(
	const std::vector<mrpt::math::CArray<float,3> > & pts,
	const float    lineWidth,
	const uint32_t slices,
	const uint32_t stacks
	)
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_BLEND);  gl_utils::checkOpenGLError();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);   gl_utils::checkOpenGLError();
	glLineWidth(lineWidth); gl_utils::checkOpenGLError();
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines

	// Points in the ellipsoid:
	//  * "#slices" slices, with "#stacks" points each, but for the two ends
	//  * 1 point at each end slice
	// #total points = stacks*(slices-2) + 2
	ASSERT_EQUAL_((slices-2)*stacks+2, pts.size())

	const size_t idx_1st_slice = 1;

	// 1st slice: triangle fan (if it were solid)
	// ----------------------------
	glBegin( GL_LINES );
	for (size_t i=0;i<stacks;i++)
	{
		glVertex3fv(&pts[0][0]);
		glVertex3fv(&pts[idx_1st_slice+i][0]);
	}
	glEnd();

	// Middle slices: triangle strip (if it were solid)
	// ----------------------------
	for (size_t s=0;s<slices-3;s++)
	{
		size_t idx_this_slice = idx_1st_slice + stacks*s;
		size_t idx_next_slice = idx_this_slice + stacks;

		for (size_t i=0;i<stacks;i++)
		{
			const size_t ii = (i==(stacks-1) ? 0 : i+1);  // next i with wrapping

			glBegin( GL_LINE_STRIP );
			glVertex3fv(&pts[idx_this_slice+i][0]);
			glVertex3fv(&pts[idx_next_slice+ii][0]);
			glVertex3fv(&pts[idx_next_slice+i][0]);
			glVertex3fv(&pts[idx_this_slice+i][0]);
			glVertex3fv(&pts[idx_this_slice+ii][0]);
			glVertex3fv(&pts[idx_next_slice+ii][0]);
			glEnd();
		}
	}

	// Last slice: triangle fan (if it were solid)
	// ----------------------------
	const size_t idx_last_pt    = pts.size()-1;
	const size_t idx_last_slice = idx_1st_slice + (slices-3)*stacks;
	glBegin( GL_LINES );
	for (size_t i=0;i<stacks;i++)
	{
		glVertex3fv(&pts[idx_last_pt][0]);
		glVertex3fv(&pts[idx_last_slice+i][0]);
	}
	glEnd();

	//glBegin( GL_POINTS );
	//const size_t N = pts.size();
	//for (size_t i=0;i<N;i++)
	//	glVertex3f( pts[i][0], pts[i][1], pts[i][2] );
	//glEnd();

	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
#endif
}


/*---------------------------------------------------------------
			generalizedEllipsoidPoints: 2D
  ---------------------------------------------------------------*/
template <>
void OPENGL_IMPEXP generalizedEllipsoidPoints<2>(
	const mrpt::math::CMatrixFixedNumeric<double,2,2> & U,
	const mrpt::math::CMatrixFixedNumeric<double,2,1>   & mean,
	std::vector<mrpt::math::CArray<float,2> >   &out_params_pts,
	const uint32_t numSegments,
	const uint32_t numSegments_unused)
{
	out_params_pts.clear();
	out_params_pts.reserve(numSegments);
	const double Aa = 2*M_PI/numSegments;
	for (double ang=0;ang<2*M_PI;ang+=Aa)
	{
		const double ccos = cos(ang);
		const double ssin = sin(ang);

		out_params_pts.resize(out_params_pts.size()+1);

		mrpt::math::CArray<float,2> &pt = out_params_pts.back();

		pt[0] = mean[0] + ccos * U.get_unsafe(0,0) + ssin * U.get_unsafe(0,1);
		pt[1] = mean[1] + ccos * U.get_unsafe(1,0) + ssin * U.get_unsafe(1,1);
	}
}



inline
void aux_add3DpointWithEigenVectors(
	const double x,
	const double y,
	const double z,
	std::vector<mrpt::math::CArray<float,3> >   & pts,
	const mrpt::math::CMatrixFixedNumeric<double,3,3> & M,
	const mrpt::math::CMatrixFixedNumeric<double,3,1>   & mean)
{
	pts.resize(pts.size()+1);
	mrpt::math::CArray<float,3> &pt= pts.back();
	pt[0] = mean[0] + x * M.get_unsafe(0,0) + y * M.get_unsafe(0,1) + z * M.get_unsafe(0,2);
	pt[1] = mean[1] + x * M.get_unsafe(1,0) + y * M.get_unsafe(1,1) + z * M.get_unsafe(1,2);
	pt[2] = mean[2] + x * M.get_unsafe(2,0) + y * M.get_unsafe(2,1) + z * M.get_unsafe(2,2);
}


/*---------------------------------------------------------------
			generalizedEllipsoidPoints: 3D
  ---------------------------------------------------------------*/
template <>
void OPENGL_IMPEXP generalizedEllipsoidPoints<3>(
	const mrpt::math::CMatrixFixedNumeric<double,3,3> & U,
	const mrpt::math::CMatrixFixedNumeric<double,3,1>   & mean,
	std::vector<mrpt::math::CArray<float,3> >   & pts,
	const uint32_t slices,
	const uint32_t stacks)
{
	MRPT_START
	ASSERT_ABOVEEQ_(slices,3)
	ASSERT_ABOVEEQ_(stacks,3)

	// sin/cos cache --------
	// Slices: [0,pi]
	std::vector<double> slice_cos(slices),slice_sin(slices);
    for (uint32_t i = 0; i < slices; i++)
	{
		double angle = M_PI * i / double(slices-1);
		slice_sin[i] = sin(angle);
		slice_cos[i] = cos(angle);
	}
	// Stacks: [0,2*pi]
	std::vector<double> stack_sin(stacks),stack_cos(stacks);
    for (uint32_t i = 0; i < stacks; i++)
	{
		double angle = 2*M_PI * i / double(stacks);
		stack_sin[i] = sin(angle);
		stack_cos[i] = cos(angle);
	}

	// Points in the ellipsoid:
	//  * "#slices" slices, with "#stacks" points each, but for the two ends
	//  * 1 point at each end slice
	// #total points = stacks*(slices-2) + 2
	pts.clear();
	pts.reserve((slices-2)*stacks+2);

	for (uint32_t i=0;i<slices;i++)
	{
		if (i==0)
			aux_add3DpointWithEigenVectors(1,0,0,  pts,U,mean);
		else if (i==(slices-1))
			aux_add3DpointWithEigenVectors(-1,0,0,  pts,U,mean);
		else
		{
			const double x = slice_cos[i];
			const double R = slice_sin[i];

			for (uint32_t j=0;j<stacks;j++)
			{
				const double y = R*stack_cos[j];
				const double z = R*stack_sin[j];
				aux_add3DpointWithEigenVectors(x,y,z,  pts,U,mean);
			}
		}
	}

	MRPT_END
}


}}} // end namespaces
