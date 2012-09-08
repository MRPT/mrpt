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

#include <mrpt/opengl/CRenderizableDisplayList.h>

#include "opengl_internals.h"


using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CRenderizableDisplayList, CRenderizable, mrpt::opengl )


// This is needed since it seems we must delete display lists from the same thread we create them....
struct TAuxDLData
{
	std::vector<unsigned int>      dls_to_delete;
	mrpt::synch::CCriticalSection  dls_to_delete_cs;

	static TAuxDLData& getSingleton()
	{
		static TAuxDLData instance;
		return instance;
	}
};

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
		// Delete the graphical memory (actually, enque the request...)
		TAuxDLData & obj = TAuxDLData::getSingleton();
		obj.dls_to_delete_cs.enter();
			obj.dls_to_delete.push_back(m_dl);
		obj.dls_to_delete_cs.leave();
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
		// We must delete pending dl's in the same thread we create them, so, let's do it here, for example:
		TAuxDLData & obj = TAuxDLData::getSingleton();
		if (!obj.dls_to_delete.empty())
		{
			obj.dls_to_delete_cs.enter();
#if MRPT_HAS_OPENGL_GLUT
			for (size_t i=0;i<obj.dls_to_delete.size();i++)
				glDeleteLists(obj.dls_to_delete[i], 1);
#endif
			obj.dls_to_delete.clear();
			obj.dls_to_delete_cs.leave();
		}

		if (m_dl==INVALID_DISPLAY_LIST_ID)
		{
			m_dl = glGenLists(1);  // Assign list ID upon first usage.
			if (glGetError()!= GL_NO_ERROR)
				std::cerr << "glGenLists: Error" << std::endl;
		}

		if (m_dl_recreate)
		{
			m_dl_recreate = false;
			glNewList(m_dl,GL_COMPILE);
			if (glGetError()!= GL_NO_ERROR)
				std::cerr << "glNewList: Error" << std::endl;

			// Call derived class:
			render_dl();

			glEndList();
			if (glGetError()!= GL_NO_ERROR)
				std::cerr << "glEndList: Error" << std::endl;
		}

		// Call the list:
		glCallList(m_dl);
		glGetError(); // Clear any error flags that may remain: this is because it seems FBO rendering lead to "errors" but it actually renders OK...
	}
#endif
}

