/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


#include <mrpt/opengl/COctoMapVoxels.h>
#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( COctoMapVoxels, CRenderizableDisplayList, mrpt::opengl )

/** Ctor */
COctoMapVoxels::COctoMapVoxels() :
	m_show_voxels (true), 
	m_show_grids  (true),
	m_grid_width  (1.0f),
	m_grid_color  (0xE0,0xE0,0xE0, 0x90)
{
}

/** Clears everything */
void COctoMapVoxels::clear()	
{
	m_voxels.clear();
	m_grid_cubes.clear();
	
	CRenderizableDisplayList::notifyChange();
}

void COctoMapVoxels::setBoundingBox(const mrpt::math::TPoint3D &bb_min, const mrpt::math::TPoint3D &bb_max)
{
	m_bb_min = bb_min;
	m_bb_max = bb_max;
}


#if MRPT_HAS_OPENGL_GLUT

const GLubyte grid_indices[36] = {
	0,1,2, 2,3,0,   
	0,3,4, 4,5,0,
	0,5,6, 6,1,0,
	1,6,7, 7,2,1,
	7,4,3, 3,2,7,
	4,7,6, 6,5,4};

#endif

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   COctoMapVoxels::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT

	// Draw grids ====================================
	if (m_show_grids)
	{
		glLineWidth(m_grid_width);
		checkOpenGLError();

		if ( m_grid_color.A != 255 )
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		glColor4ub(m_grid_color.R,m_grid_color.G,m_grid_color.B,m_grid_color.A);

		glEnableClientState(GL_VERTEX_ARRAY);

		const size_t nGrids = m_grid_cubes.size();
		for (size_t i=0;i<nGrids;i++)
		{
			const TGridCube &c = m_grid_cubes[i];

			const GLfloat vertices[8*3] = {
				c.max.x,c.max.y,c.max.z,
				c.max.x,c.min.y,c.max.z,
				c.max.x,c.min.y,c.min.z,
				c.max.x,c.max.y,c.min.z,
				c.min.x,c.max.y,c.min.z,
				c.min.x,c.max.y,c.max.z,
				c.min.x,c.min.y,c.max.z,
				c.min.x,c.min.y,c.min.z
			};
			glVertexPointer(3, GL_FLOAT, 0, vertices);
			glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, grid_indices);
		}

		glDisableClientState(GL_VERTEX_ARRAY);
		checkOpenGLError();

		if ( m_grid_color.A != 255 )
			glDisable(GL_BLEND);
	}

	// Draw cubes ====================================
	if (m_show_voxels)
	{

	}


#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  COctoMapVoxels::writeToStream(CStream &out,int *version) const
{
	if (version) *version=0;
	else	
	{
		writeToStreamRender(out);
		//THROW_EXCEPTION("TODO")
		MRPT_TODO("Write")
		//out<<mSegments<<mLineWidth;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  COctoMapVoxels::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			//THROW_EXCEPTION("TODO")
			MRPT_TODO("read")
		}	
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void COctoMapVoxels::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = m_bb_min;
	bb_max = m_bb_max;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
