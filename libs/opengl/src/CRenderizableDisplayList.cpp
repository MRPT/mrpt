/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#include <mrpt/opengl/CRenderizableDisplayList.h>

#include "opengl_internals.h"


using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CRenderizableDisplayList, CRenderizable, mrpt::opengl )

// Default constructor:
CRenderizableDisplayList::CRenderizableDisplayList() :
	m_dl(INVALID_DISPLAY_LIST_ID),
	m_dl_recreate(true)
{
}

// Destructor:
CRenderizableDisplayList::~CRenderizableDisplayList()
{
	// If we had an associated display list:
	if (m_dl!=INVALID_DISPLAY_LIST_ID)
	{
		// Delete the graphical memory:
#if MRPT_HAS_OPENGL_GLUT
		glDeleteLists(m_dl, 1);
#endif
	}
}

// This is the virtual rendering method CRenderizable expects from us.
// We call our derived class to save the list, then just call that list:
void   CRenderizableDisplayList::render() const
{
#if MRPT_HAS_OPENGL_GLUT
	if (should_skip_display_list_cache())
	{
		// The object is in a state where caching a display list is not preferred, so render directly:
		render_dl();
	}
	else
	{
		if (m_dl==INVALID_DISPLAY_LIST_ID)
			m_dl = glGenLists(1);  // Assign list ID upon first usage.

		if (m_dl_recreate)
		{
			m_dl_recreate = false;
			glNewList(m_dl,GL_COMPILE);

			// Call derived class:
			render_dl();

			glEndList();
		}

		// Call the list:
		glCallList(m_dl);
	}
#endif 
}


CRenderizable& CRenderizableDisplayList::setColor( double R, double G, double B, double A)
{
	m_color_R = R;
	m_color_G = G;
	m_color_B = B;
	m_color_A = A;
	notifyChange();
	return *this;
}

CRenderizable& CRenderizableDisplayList::setColor( const mrpt::utils::TColorf &c)
{
	m_color_R = c.R;
	m_color_G = c.G;
	m_color_B = c.B;
	m_color_A = c.A;
	return *this;
}
