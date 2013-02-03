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
	m_enable_lighting(false),
	m_enable_cube_transparency(true),
	m_showVoxelsAsPoints(false),
	m_showVoxelsAsPointsSize(3.0f),
	m_show_grids  (false),
	m_grid_width  (1.0f),
	m_grid_color  (0xE0,0xE0,0xE0, 0x90),
	m_visual_mode (COctoMapVoxels::COLOR_FROM_OCCUPANCY)
{
}

/** Clears everything */
void COctoMapVoxels::clear()
{
	m_voxel_sets.clear();
	m_grid_cubes.clear();

	CRenderizableDisplayList::notifyChange();
}

void COctoMapVoxels::setBoundingBox(const mrpt::math::TPoint3D &bb_min, const mrpt::math::TPoint3D &bb_max)
{
	m_bb_min = bb_min;
	m_bb_max = bb_max;
}


#if MRPT_HAS_OPENGL_GLUT

// See: http://www.songho.ca/opengl/gl_vertexarray.html

// cube ///////////////////////////////////////////////////////////////////////
//    v6----- v5
//   /|      /|
//  v1------v0|
//  | |     | |
//  | |v7---|-|v4
//  |/      |/
//  v2------v3

const GLubyte grid_line_indices[] = {
	0,1, 1,2, 2,3, 3,0,
	4,5, 5,6, 6,7, 7,4,
	0,5, 1,6, 2,7, 3,4
	};

const GLubyte cube_indices[36]  = {
	0,1,2, 2,3,0,
	0,3,4, 4,5,0,
	0,5,6, 6,1,0,
	1,6,7, 7,2,1,
	7,4,3, 3,2,7,
	4,7,6, 6,5,4};

MRPT_TODO("Check normal directions")
// normal array
const GLfloat normals_cube[3*6*4]  = {
	0, 0, 1,   0, 0, 1,   0, 0, 1,   0, 0, 1,   // v0,v1,v2,v3 (front)
	1, 0, 0,   1, 0, 0,   1, 0, 0,   1, 0, 0,   // v0,v3,v4,v5 (right)
	0, 1, 0,   0, 1, 0,   0, 1, 0,   0, 1, 0,   // v0,v5,v6,v1 (top)
	-1, 0, 0,  -1, 0, 0,  -1, 0, 0,  -1, 0, 0,   // v1,v6,v7,v2 (left)
	0,-1, 0,   0,-1, 0,   0,-1, 0,   0,-1, 0,   // v7,v4,v3,v2 (bottom)
	0, 0,-1,   0, 0,-1,   0, 0,-1,   0, 0,-1 }; // v4,v7,v6,v5 (back)

#endif

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   COctoMapVoxels::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT

	glEnableClientState(GL_VERTEX_ARRAY);

	// Draw grids ====================================
	if (m_show_grids)
	{
		glLineWidth(m_grid_width);
		checkOpenGLError();

		// Antialiasing:
        glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
        glEnable(GL_LINE_SMOOTH);
		if ( m_grid_color.A != 255 )
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		glColor4ub(m_grid_color.R,m_grid_color.G,m_grid_color.B,m_grid_color.A);

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
			glDrawElements(GL_LINES, sizeof(grid_line_indices)/sizeof(grid_line_indices[0]), GL_UNSIGNED_BYTE, grid_line_indices);
		}

		// End of antialiasing:
        glPopAttrib();
	}

	// Draw cubes ====================================
    glEnableClientState(GL_NORMAL_ARRAY);

	if (m_enable_lighting)
	{
		 // track material ambient and diffuse from surface color, call it before glEnable(GL_COLOR_MATERIAL)
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);

		// set up light colors (ambient, diffuse, specular)
		GLfloat lightKa[] = {.2f, .2f, .2f, 1.0f};  // ambient light
		GLfloat lightKd[] = {.7f, .7f, .7f, 1.0f};  // diffuse light
		GLfloat lightKs[] = {1, 1, 1, 1};           // specular light
		glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

		// position the light
		float lightPos[4] = {10, 10, 10, 1}; // positional light
		glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

		glEnable(GL_LIGHT0);                        // MUST enable each light source after configuration

		glEnable(GL_LIGHTING);
	}

	glNormalPointer(GL_FLOAT, 0, normals_cube);

	if (m_enable_cube_transparency)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	if (m_showVoxelsAsPoints)
	{
		glPointSize(m_showVoxelsAsPointsSize);
	    glBegin( GL_POINTS );
	}


	for (size_t i=0;i<m_voxel_sets.size();i++)
	{
		if (!m_voxel_sets[i].visible) continue;

		const std::vector<TVoxel> & voxels = m_voxel_sets[i].voxels;
		const size_t N = voxels.size();
		for (size_t j=0;j<N;j++)
		{
			glColor4ub(voxels[j].color.R,voxels[j].color.G,voxels[j].color.B,voxels[j].color.A);

			const mrpt::math::TPoint3D &c = voxels[j].coords;
			const double                L = voxels[j].side_length * 0.5;

			if (!m_showVoxelsAsPoints)
			{
				// Render as cubes:
				const GLfloat vertices[8*3] = {
					c.x+L,c.y+L,c.z+L,
					c.x+L,c.y-L,c.z+L,
					c.x+L,c.y-L,c.z-L,
					c.x+L,c.y+L,c.z-L,
					c.x-L,c.y+L,c.z-L,
					c.x-L,c.y+L,c.z+L,
					c.x-L,c.y-L,c.z+L,
					c.x-L,c.y-L,c.z-L
				};
				glVertexPointer(3, GL_FLOAT, 0, vertices);
				glDrawElements(GL_TRIANGLES, sizeof(cube_indices)/sizeof(cube_indices[0]), GL_UNSIGNED_BYTE, cube_indices);
			}
			else
			{
				// Render as simple points:
				glVertex3f(c.x,c.y,c.z);
			}
		}
	}


	if (m_showVoxelsAsPoints)
	{
		glEnd(); // of  GL_POINTS
	}

	if (m_enable_cube_transparency)
		glDisable(GL_BLEND);

	if (m_enable_lighting)
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
	}

    glDisableClientState(GL_NORMAL_ARRAY);

	glDisableClientState(GL_VERTEX_ARRAY);
	checkOpenGLError();

#endif
}

DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TInfoPerVoxelSet)
DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TGridCube)
DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TVoxel)

namespace mrpt{
	namespace utils
	{
		CStream & operator<<(CStream&out, const COctoMapVoxels::TInfoPerVoxelSet &a) {
			out << a.visible << a.voxels;
			return out;
		}
		CStream & operator>>(CStream&in, COctoMapVoxels::TInfoPerVoxelSet &a) {
			in >> a.visible >> a.voxels;
			return in;
		}

		CStream & operator<<(CStream&out, const COctoMapVoxels::TGridCube &a) {
			out << a.min << a.max;
			return out;
		}
		CStream & operator>>(CStream&in, COctoMapVoxels::TGridCube &a) {
			in >> a.min >> a.max;
			return in;
		}

		CStream & operator<<(CStream&out, const COctoMapVoxels::TVoxel &a) {
			out << a.coords << a.side_length << a.color;
			return out;
		}
		CStream & operator>>(CStream&in, COctoMapVoxels::TVoxel &a) {
			in >> a.coords >> a.side_length >> a.color;
			return in;
		}

}
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  COctoMapVoxels::writeToStream(CStream &out,int *version) const
{
	if (version) *version=2;
	else
	{
		writeToStreamRender(out);

		out << m_voxel_sets
			<< m_grid_cubes
			<< m_bb_min << m_bb_max
			<< m_enable_lighting << m_showVoxelsAsPoints <<	m_showVoxelsAsPointsSize
			<< m_show_grids << m_grid_width << m_grid_color
			<< m_enable_cube_transparency // added in v1
			<< uint32_t(m_visual_mode); // added in v2
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
	case 1:
	case 2:
		{
			readFromStreamRender(in);

			in  >> m_voxel_sets
				>> m_grid_cubes
				>> m_bb_min >> m_bb_max
				>> m_enable_lighting >> m_showVoxelsAsPoints >> m_showVoxelsAsPointsSize
				>> m_show_grids >> m_grid_width >> m_grid_color;

			if (version>=1)
				in >> m_enable_cube_transparency;
			else m_enable_cube_transparency = false;

			if (version>=2)
			{
				uint32_t i;
				in >> i;
				m_visual_mode = static_cast<COctoMapVoxels::visualization_mode_t>(i);
			}
			else m_visual_mode = COctoMapVoxels::COLOR_FROM_OCCUPANCY;
		}
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

	CRenderizableDisplayList::notifyChange();
}

void COctoMapVoxels::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = m_bb_min;
	bb_max = m_bb_max;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
