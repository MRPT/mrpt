/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

// Include the lib3ds library:
#include <lib3ds/file.h>
#include <lib3ds/background.h>
#include <lib3ds/camera.h>
#include <lib3ds/mesh.h>
#include <lib3ds/node.h>
#include <lib3ds/material.h>
#include <lib3ds/matrix.h>
#include <lib3ds/vector.h>
#include <lib3ds/light.h>

#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CTexturedPlane.h>

#include <mrpt/compress/zip.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/vector_loadsave.h>

#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>

#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CAssimpModel, CRenderizableDisplayList, mrpt::opengl )


void render_node(Lib3dsNode *node,Lib3dsFile	*file);
void light_update(Lib3dsLight *l,Lib3dsFile	*file);

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CAssimpModel::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	if (! m_3dsfile->file) return;	// No scene

	Lib3dsFile	*file = (Lib3dsFile*) m_3dsfile->file;
	if (!file) return;

	glEnable(GL_POLYGON_SMOOTH);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);


	// Add an ambient light:
	if (m_enable_extra_lighting)
	{
		int li = GL_LIGHT7;
		const GLfloat a[] = {0.8f, 0.8f, 0.8f, 1.0f};
		GLfloat c[] = {0.5f, 0.5f, 0.5f, 0.5f};

		glLightfv(li, GL_AMBIENT, a);
		glLightfv(li, GL_DIFFUSE, c);
		glLightfv(li, GL_SPECULAR, c);
		glEnable(li);
	}


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, file->ambient);

	/* Lights.  Set them from light nodes if possible.  If not, use the
	* light objects directly.
	*/
	{
		const GLfloat a[] = {0.1f, 0.1f, 0.1f, 1.0f};
		GLfloat c[] = {1.0f, 1.0f, 1.0f, 1.0f};
		GLfloat p[] = {0.0f, 0.0f, 0.0f, 1.0f};

		int li=GL_LIGHT0;
		for (Lib3dsLight *l=file->lights; l; l=l->next)
		{
		  glEnable(li);

		  light_update(l,file);

		  c[0] = l->color[0];
		  c[1] = l->color[1];
		  c[2] = l->color[2];
		  //c[3] = l->multiplier;
		  glLightfv(li, GL_AMBIENT, a);
		  glLightfv(li, GL_DIFFUSE, c);
		  glLightfv(li, GL_SPECULAR, c);

		  //float att = (1.0/m_scale_x)-1;
		  //glLightfv(li, GL_LINEAR_ATTENUATION, &att );

		  float att = 1.0/m_scale_x;
		  glLightfv(li, GL_CONSTANT_ATTENUATION, &att );

		  p[0] = l->position[0];
		  p[1] = l->position[1];
		  p[2] = l->position[2];
		  glLightfv(li, GL_POSITION, p);

		  if (l->spot_light)
		  {
			p[0] = (l->spot[0] - l->position[0]);
			p[1] = (l->spot[1] - l->position[1]);
			p[2] = (l->spot[2] - l->position[2]);
			glLightfv(li, GL_SPOT_DIRECTION, p);
		  }
		  ++li;
		}
	}


	for (Lib3dsNode *p=file->nodes; p!=0; p=p->next)
	  render_node(p,file);

	glDisable(GL_CULL_FACE);
	MRPT_END
#endif
}


// texture size: by now minimum standard
#define	TEX_XSIZE	1024
#define	TEX_YSIZE	1024

struct _player_texture
{
  int valid; // was the loading attempt successful ?
#ifdef	USE_SDL
  SDL_Surface *bitmap;
#else
  void *bitmap;
#endif

#if MRPT_HAS_OPENGL_GLUT
  GLuint tex_id; //OpenGL texture ID
#else
  unsigned int tex_id; //OpenGL texture ID
#endif

  float scale_x, scale_y; // scale the texcoords, as OpenGL thinks in TEX_XSIZE and TEX_YSIZE
};

typedef struct _player_texture Player_texture;

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CAssimpModel::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		writeToStreamRender(out);

		CMemoryChunk chunk;
		if (m_3dsfile && m_3dsfile->file)
		{
			const string	tmpFil = mrpt::system::getTempFileName();
			lib3ds_file_save( (Lib3dsFile*) m_3dsfile->file, tmpFil.c_str() );
			chunk.loadBufferFromFile( tmpFil  );
			mrpt::system::deleteFile( tmpFil );
		}

		// Write the "3dsfile":
		out << chunk;

		out << m_enable_extra_lighting; // Added in version 1

	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CAssimpModel::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			readFromStreamRender(in);

			// Read the memory block, save to a temp. "3dsfile" and load...
			clear();

			CMemoryChunk chunk;
			in >> chunk;

			if (chunk.getTotalBytesCount())
			{
				const string	tmpFil = mrpt::system::getTempFileName();
				if (!chunk.saveBufferToFile( tmpFil ) )
					THROW_EXCEPTION("Error saving temporary 3ds file");

				try
				{
					loadFrom3DSFile( tmpFil );
				}
				catch (...)
				{
					THROW_EXCEPTION("Error loading temporary 3ds file");
				}
				mrpt::system::deleteFile( tmpFil );
			}


			if (version>=1)
			{
				if (version==1)
				{
					double dummy_scale;
					in >> dummy_scale >> dummy_scale >> dummy_scale;
				}
				in >> m_enable_extra_lighting;
			}
			else
			{
				m_enable_extra_lighting = false;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

/*---------------------------------------------------------------
					initializeAllTextures
  ---------------------------------------------------------------*/
void  CAssimpModel::initializeAllTextures()
{
#if MRPT_HAS_OPENGL_GLUT

#endif
}

CAssimpModel::CAssimpModel() :
	m_bbox_min(0,0,0),
	m_bbox_max(0,0,0),
	m_enable_extra_lighting(false)
{
	m_3dsfile.set( new TImpl3DS() );
}

CAssimpModel::~CAssimpModel()
{
	clear();
}

/*---------------------------------------------------------------
							clear
  ---------------------------------------------------------------*/
void   CAssimpModel::clear()
{
	CRenderizableDisplayList::notifyChange();
	m_3dsfile.set( new TImpl3DS() );
}

void CAssimpModel::loadFrom3DSFile( const std::string &filepath )
{
	clear();
	CRenderizableDisplayList::notifyChange();

	Lib3dsFile *file=0;

	// Normal file, or .gz file?
	if (mrpt::system::extractFileExtension(filepath)=="gz")
	{
		// Load compressed file:
		vector_byte		out_data;
		if (!mrpt::compress::zip::decompress_gz_file(filepath, out_data))
			THROW_EXCEPTION_CUSTOM_MSG1("Error loading compressed file: %s",filepath.c_str())

		// Save to tmp file & load:
		string	tmpFil = mrpt::system::getTempFileName();

		mrpt::system::vectorToBinaryFile(out_data,tmpFil);
		out_data.clear();

		file=lib3ds_file_load(tmpFil.c_str());

		mrpt::system::deleteFile( tmpFil );
	}
	else
	{
		// Uncompressed file:
		file=lib3ds_file_load(filepath.c_str());
	}

	if (!file)
	{
		THROW_EXCEPTION_CUSTOM_MSG1("Error loading 3DS file: %s", filepath.c_str() );
	}


  /* No nodes?  Fabricate nodes to display all the meshes. */
  if( !file->nodes )
  {
    for(Lib3dsMesh *mesh = file->meshes; mesh != NULL; mesh = mesh->next)
    {
      Lib3dsNode *node = lib3ds_node_new_object();
      strcpy(node->name, mesh->name);
      node->parent_id = LIB3DS_NO_PARENT;
      lib3ds_file_insert_node(file, node);
    }
  }

  lib3ds_file_eval(file, 1.0f);		// Eval in time


	Lib3dsVector bmin, bmax;
	float	sx, sy, sz, size;	/* bounding box dimensions */
	float	cx, cy, cz;		/* bounding box center */

#if 1 //def lib3ds_file_bounding_box_of_nodes
	lib3ds_file_bounding_box_of_nodes(file, LIB3DS_TRUE, LIB3DS_FALSE, LIB3DS_FALSE, bmin, bmax);
#else
	bmin[0] = -2;  bmax[0] = -2;
	bmin[1] = -2;  bmax[1] = -2;
	bmin[2] = -2;  bmax[2] = -2;
#endif

	for (int k=0;k<3;k++) {
		m_bbox_min[k] = bmin[k];
		m_bbox_max[k] = bmax[k];
	}

	sx = bmax[0] - bmin[0];
	sy = bmax[1] - bmin[1];
	sz = bmax[2] - bmin[2];
	size = max(sx, sy);
	size = max(size, sz);
	cx = (bmin[0] + bmax[0])/2;
	cy = (bmin[1] + bmax[1])/2;
	cz = (bmin[2] + bmax[2])/2;

  /* No lights in the file?  Add some. */

  if (file->lights == NULL)
  {
    Lib3dsLight *light;

    light = lib3ds_light_new("light0");
    light->spot_light = 0;
    light->see_cone = 0;
    light->color[0] = light->color[1] = light->color[2] = .6;
    light->position[0] = cx + size * .75;
    light->position[1] = cy - size * 1.;
    light->position[2] = cz + size * 1.5;
      // Out of bounds? //light->position[3] = 0.;
    light->outer_range = 100;
    light->inner_range = 10;
    light->multiplier = 1;
    lib3ds_file_insert_light(file, light);

    light = lib3ds_light_new("light1");
    light->spot_light = 0;
    light->see_cone = 0;
    light->color[0] = light->color[1] = light->color[2] = .3;
    light->position[0] = cx - size;
    light->position[1] = cy - size;
    light->position[2] = cz + size * .75;
  // Out of bounds?    light->position[3] = 0.;
    light->outer_range = 100;
    light->inner_range = 10;
    light->multiplier = 1;
    lib3ds_file_insert_light(file, light);

    light = lib3ds_light_new("light2");
    light->spot_light = 0;
    light->see_cone = 0;
    light->color[0] = light->color[1] = light->color[2] = .3;
    light->position[0] = cx;
    light->position[1] = cy + size;
    light->position[2] = cz + size;
      // Out of bounds? light->position[3] = 0.;
    light->outer_range = 100;
    light->inner_range = 10;
    light->multiplier = 1;
    lib3ds_file_insert_light(file, light);
  }

  lib3ds_file_eval(file,0.);

  m_3dsfile->file = file;
}

void CAssimpModel::evaluateAnimation( double time_anim )
{
	if (m_3dsfile->file)
	{
		CRenderizableDisplayList::notifyChange();
		lib3ds_file_eval( (Lib3dsFile*) m_3dsfile->file, time_anim );
	}
}

void CAssimpModel::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = m_bbox_min;
	bb_max = m_bbox_max;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}


CAssimpModel::TImpl3DS::TImpl3DS() : file(NULL)
{
}

CAssimpModel::TImpl3DS::~TImpl3DS()
{
	if (file)
	{
		lib3ds_file_free( (Lib3dsFile*) file);
		file= NULL;
	}
}

bool CAssimpModel::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	//TODO
	return false;
}
