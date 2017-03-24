/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/synch/CCriticalSection.h>
#include <cstdlib> // atexit()

#include "opengl_internals.h"


using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CRenderizableDisplayList, CRenderizable, mrpt::opengl )

// TAuxDLData: needed since it seems we must delete display lists from the same thread we create them....

// Phoenix Singleton pattern
void deleteSingleton();
struct TAuxDLData
{
	std::vector<unsigned int>      dls_to_delete;
	mrpt::synch::CCriticalSectionRecursive  dls_to_delete_cs;

	static TAuxDLData& getSingleton()
	{
	    if (!m_pInstance)
        {
            m_pInstance = new TAuxDLData;
            std::atexit( deleteSingleton );
        }
		return *m_pInstance;
	}

    static TAuxDLData *m_pInstance;
};
void deleteSingleton()
{
    if (TAuxDLData::m_pInstance)
    {
        delete TAuxDLData::m_pInstance;
        TAuxDLData::m_pInstance = NULL;
    }
}
TAuxDLData* TAuxDLData::m_pInstance = NULL;

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

