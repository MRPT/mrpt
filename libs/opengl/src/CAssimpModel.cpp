/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// This file contains portions of code from Assimp's example: "Sample_SimpleOpenGL.c"

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CAssimpModel.h>

#if MRPT_HAS_ASSIMP
#	if defined(MRPT_ASSIMP_VERSION_MAJOR) && MRPT_ASSIMP_VERSION_MAJOR<3
#		include <assimp.h>
#		include <aiScene.h>
#		include <aiPostProcess.h>
#	else
#		include <assimp/cimport.h>
#		include <assimp/DefaultLogger.hpp>
#		include <assimp/LogStream.hpp>
#		include <assimp/scene.h>
#		include <assimp/postprocess.h>
#	endif
#endif

#include "opengl_internals.h"
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CAssimpModel, CRenderizableDisplayList, mrpt::opengl )

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	void recursive_render (const aiScene *sc, const aiNode* nd,const std::vector<unsigned int> &textureIds,const std::map<std::string,CAssimpModel::TInfoPerTexture> &textureIdMap);
	void apply_material(const aiMaterial *mtl,const std::vector<unsigned int> &textureIds, const std::map<std::string,CAssimpModel::TInfoPerTexture> &textureIdMap);
	void set_float4(float f[4], float a, float b, float c, float d);
	void color4_to_float4(const aiColor4D *c, float f[4]);
	void get_bounding_box (const aiScene *sc,aiVector3D* min, aiVector3D* max);
	void get_bounding_box_for_node (const aiScene *sc,const aiNode* nd, aiVector3D* min, aiVector3D* max, aiMatrix4x4* trafo);
	void load_textures(const aiScene *scene, std::vector<unsigned int> &textureIds, std::map<std::string,CAssimpModel::TInfoPerTexture> &textureIdMap,const std::string &modelPath);
#endif //MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CAssimpModel::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	MRPT_START

	if (!m_assimp_scene->scene) return;	// No scene

	aiScene *scene = (aiScene *) m_assimp_scene->scene;

	glEnable(GL_TEXTURE_2D);
	glDisable(GL_CULL_FACE);
	//glFrontFace(GL_CW);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_NORMALIZE);

	if (!m_textures_loaded)
	{
		load_textures(scene,m_textureIds,m_textureIdMap,m_modelPath);
		m_textures_loaded = true;
	}

	recursive_render(scene, scene->mRootNode,m_textureIds,m_textureIdMap);

	glDisable(GL_NORMALIZE);
	glDisable(GL_TEXTURE_2D);

	MRPT_END
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CAssimpModel::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);

		const bool empty = m_assimp_scene->scene!=NULL;
		out << empty;

		if (!empty)
		{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
			//aiScene *scene = (aiScene *) m_assimp_scene->scene;
			THROW_EXCEPTION("MRPT can't serialize Assimp objects yet!")
#else
	THROW_EXCEPTION("MRPT compiled without OpenGL and/or Assimp")
#endif
		}

}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CAssimpModel::readFromStream(mrpt::utils::CStream &in,int version)
{
	THROW_EXCEPTION("MRPT can't serialize Assimp objects yet!")

	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);

			clear();
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

CAssimpModel::CAssimpModel() :
	m_bbox_min(0,0,0),
	m_bbox_max(0,0,0),
	m_textures_loaded(false)
{
	m_assimp_scene.set( new TImplAssimp() );
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
	m_assimp_scene.set( new TImplAssimp() );
	m_modelPath.clear();
	m_textures_loaded = false;

#if MRPT_HAS_OPENGL_GLUT
	if (!m_textureIds.empty())
	{
		glDeleteTextures(m_textureIds.size(), &m_textureIds[0]);
		m_textureIds.clear();
	}
	m_textureIdMap.clear();
#endif
}

void CAssimpModel::loadScene( const std::string &filepath )
{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	clear();
	CRenderizableDisplayList::notifyChange();

	// we are taking one of the postprocessing presets to avoid
	// spelling out 20+ single postprocessing flags here.
	m_assimp_scene->scene = (void*) aiImportFile(filepath.c_str(), aiProcessPreset_TargetRealtime_MaxQuality );
	m_modelPath = filepath;


	if (m_assimp_scene->scene)
	{
		aiVector3D scene_min, scene_max;
		aiScene *scene = (aiScene *) m_assimp_scene->scene;
		get_bounding_box(scene,&scene_min,&scene_max);
		m_bbox_min.x = scene_min.x; m_bbox_min.y = scene_min.y; m_bbox_min.z = scene_min.z;
		m_bbox_max.x = scene_max.x; m_bbox_max.y = scene_max.y; m_bbox_max.z = scene_max.z;
	}

#else
	THROW_EXCEPTION("MRPT compiled without OpenGL and/or Assimp")
#endif
}

void CAssimpModel::evaluateAnimation( double time_anim )
{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	//if (m_assimp_scene->file)
	//{
	//	CRenderizableDisplayList::notifyChange();
	//	lib3ds_file_eval( (Lib3dsFile*) m_assimp_scene->file, time_anim );
	//}
#endif
}

void CAssimpModel::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = m_bbox_min;
	bb_max = m_bbox_max;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}


CAssimpModel::TImplAssimp::TImplAssimp() : scene(NULL)
{
}

CAssimpModel::TImplAssimp::~TImplAssimp()
{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	if (scene)
	{
		// cleanup - calling 'aiReleaseImport' is important, as the library
		// keeps internal resources until the scene is freed again. Not
		// doing so can cause severe resource leaking.
		aiReleaseImport( (aiScene*) scene);
		scene=NULL;
	}
#endif
}

bool CAssimpModel::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	//TODO
	return false;
}



#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP

// ----------------------------------------------------------------------------
void get_bounding_box_for_node (const aiScene* scene, const aiNode* nd, aiVector3D* min, aiVector3D* max, aiMatrix4x4* trafo)
{
	aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo,&nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n) {
		const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t) {

			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp,trafo);

			min->x = std::min(min->x,tmp.x);
			min->y = std::min(min->y,tmp.y);
			min->z = std::min(min->z,tmp.z);

			max->x = std::max(max->x,tmp.x);
			max->y = std::max(max->y,tmp.y);
			max->z = std::max(max->z,tmp.z);
		}
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		get_bounding_box_for_node(scene,nd->mChildren[n],min,max,trafo);
	}
	*trafo = prev;
}

// ----------------------------------------------------------------------------
void get_bounding_box (const aiScene* scene, aiVector3D* min, aiVector3D* max)
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	min->x = min->y = min->z =  1e10f;
	max->x = max->y = max->z = -1e10f;
	get_bounding_box_for_node(scene,scene->mRootNode,min,max,&trafo);
}

// ----------------------------------------------------------------------------
void color4_to_float4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

// ----------------------------------------------------------------------------
void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

// ----------------------------------------------------------------------------
void apply_material(const aiMaterial *mtl,const std::vector<unsigned int> &textureIds, const std::map<std::string,CAssimpModel::TInfoPerTexture> &textureIdMap)
{
	float c[4];

	GLenum fill_mode;
	int ret1, ret2;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	float shininess, strength;
	int two_sided;
	int wireframe;
	unsigned int max;


	int texIndex = 0;
	aiString texPath;	//contains filename of texture

	if(AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath))
	{
		//bind texture
		std::map<std::string,CAssimpModel::TInfoPerTexture>::const_iterator it=textureIdMap.find(texPath.data);
		if (it==textureIdMap.end())
		{
			std::cerr << "[CAssimpModel] Error: using un-loaded texture '"<< texPath.data <<"'\n";
		}
		else
		{
			unsigned int texId = textureIds[it->second.id_idx];
			glBindTexture(GL_TEXTURE_2D, texId);
		}
	}

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
		color4_to_float4(&diffuse, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
		color4_to_float4(&specular, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

	set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
		color4_to_float4(&ambient, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
		color4_to_float4(&emission, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if(ret1 == AI_SUCCESS) {
    	max = 1;
    	ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if(ret2 == AI_SUCCESS)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
        else
        	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
	else {
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
		set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	}

	max = 1;
	if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
		fill_mode = wireframe ? GL_LINE : GL_FILL;
	else
		fill_mode = GL_FILL;
	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		glDisable(GL_CULL_FACE);
	else
		glEnable(GL_CULL_FACE);
}

// Can't send color down as a pointer to aiColor4D because AI colors are ABGR.
void Color4f(const aiColor4D *color)
{
	glColor4f(color->r, color->g, color->b, color->a);
}

// ----------------------------------------------------------------------------
void recursive_render (const aiScene *sc, const aiNode* nd,const std::vector<unsigned int> &textureIds, const std::map<std::string,CAssimpModel::TInfoPerTexture> &textureIdMap)
{
	unsigned int i;
	unsigned int n=0, t;
	aiMatrix4x4 m = nd->mTransformation;

	// update transform
	m.Transpose();
	glPushMatrix();
	glMultMatrixf((float*)&m);

	// draw all meshes assigned to this node
	for (; n < nd->mNumMeshes; ++n)
	{
		const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

		apply_material(sc->mMaterials[mesh->mMaterialIndex],textureIds,textureIdMap);


		if(mesh->mNormals == NULL)
			glDisable(GL_LIGHTING);
		else
			glEnable(GL_LIGHTING);

		if(mesh->mColors[0] != NULL)
			glEnable(GL_COLOR_MATERIAL);
		else
			glDisable(GL_COLOR_MATERIAL);

		for (t = 0; t < mesh->mNumFaces; ++t)
		{
			const struct aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;

			switch(face->mNumIndices)
			{
				case 1: face_mode = GL_POINTS; break;
				case 2: face_mode = GL_LINES; break;
				case 3: face_mode = GL_TRIANGLES; break;
				default: face_mode = GL_POLYGON; break;
			}

			glBegin(face_mode);

			for(i = 0; i < face->mNumIndices; i++)		// go through all vertices in face
			{
				int vertexIndex = face->mIndices[i];	// get group index for current index
				if(mesh->mColors[0] != NULL)
					Color4f(&mesh->mColors[0][vertexIndex]);
				if(mesh->mNormals != NULL)

				if(mesh->HasTextureCoords(0))		//HasTextureCoords(texture_coordinates_set)
				{
					glTexCoord2f(mesh->mTextureCoords[0][vertexIndex].x, 1 - mesh->mTextureCoords[0][vertexIndex].y); //mTextureCoords[channel][vertex]
				}

				glNormal3fv(&mesh->mNormals[vertexIndex].x);
				glVertex3fv(&mesh->mVertices[vertexIndex].x);
			}
			glEnd();
		}
	}

	// draw all children
	for (n = 0; n < nd->mNumChildren; ++n)
		recursive_render(sc, nd->mChildren[n],textureIds,textureIdMap);

	glPopMatrix();
}

// http://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

void load_textures(
	const aiScene *scene,
	std::vector<unsigned int> &textureIds,
	std::map<std::string,CAssimpModel::TInfoPerTexture> &textureIdMap,
	const std::string &modelPath)
{
	if (scene->HasTextures()) THROW_EXCEPTION("Support for meshes with embedded textures is not implemented")

	textureIdMap.clear();

	/* getTexture Filenames and no. of Textures */
	for (unsigned int m=0; m<scene->mNumMaterials; m++)
	{
		int texIndex = 0;
		for (;;)
		{
			aiString path;	// filename
			aiReturn texFound = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
			if (texFound == AI_SUCCESS)
			{
				CAssimpModel::TInfoPerTexture &ipt = textureIdMap[path.data];
				ipt.id_idx = std::string::npos; //fill map with textures, pointers still NULL yet
				texIndex++;
			}
			else break;
		}
	}

	int numTextures = textureIdMap.size();

	/* create and fill array with GL texture ids */
	textureIds.resize(numTextures);
	glGenTextures(numTextures, &textureIds[0]); /* Texture name generation */

	/* get iterator */
	std::map<std::string,CAssimpModel::TInfoPerTexture>::iterator itr = textureIdMap.begin();

	std::string basepath = mrpt::system::filePathSeparatorsToNative( mrpt::system::extractFileDirectory(modelPath) );
	for (int i=0; i<numTextures; i++)
	{

		//save IL image ID
		std::string filename = itr->first;  // get filename
		CAssimpModel::TInfoPerTexture &ipt = itr->second;
		ipt.id_idx =  i;	  // save texture id for filename in map
		++itr; // next texture

		const std::string fileloc = mrpt::system::filePathSeparatorsToNative( basepath + filename );

		ipt.img_rgb = mrpt::utils::CImage::Create();
		ipt.img_alpha = mrpt::utils::CImage::Create();
		mrpt::utils::CImage *img_rgb = ipt.img_rgb.pointer();
		mrpt::utils::CImage *img_a = ipt.img_alpha.pointer();

		// Load images:
		// TGA is handled specially since it's not supported by OpenCV:
		bool load_ok;
		if ( mrpt::system::lowerCase(mrpt::system::extractFileExtension(fileloc))==string("tga"))
		{
			load_ok = CImage::loadTGA(fileloc,*img_rgb,*img_a);
		}
		else
		{
			load_ok = img_rgb->loadFromFile(fileloc);
		}

		if (load_ok)
		{
			//success = ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE); /* Convert every colour component into unsigned byte. If your image contains alpha channel you can replace IL_RGB with IL_RGBA */
			glBindTexture(GL_TEXTURE_2D, textureIds[i]); /* Binding of texture name */
			//redefine standard texture values
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); /* We will use linear
			interpolation for magnification filter */
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); /* We will use linear
			interpolation for minifying filter */
			//glTexImage2D(
			//	GL_TEXTURE_2D,
			//	0,
			//	8 /*ilGetInteger(IL_IMAGE_BPP)*/,
			//	/* ilGetInteger(IL_IMAGE_WIDTH)*/,
			//	/*ilGetInteger(IL_IMAGE_HEIGHT)*/,
			//	0,
			//	/*ilGetInteger(IL_IMAGE_FORMAT)*/,
			//	GL_UNSIGNED_BYTE,
			//	ilGetData()); /* Texture specification */

			const int width  = img_rgb->getWidth();
			const int height = img_rgb->getHeight();

			// Prepare image data types:
			const GLenum img_type = GL_UNSIGNED_BYTE;
			const int nBytesPerPixel = img_rgb->isColor() ? 3 : 1;
			const bool is_RGB_order = (!::strcmp(img_rgb->getChannelsOrder(),"RGB"));  // Reverse RGB <-> BGR order?
			const GLenum img_format = nBytesPerPixel==3 ? (is_RGB_order ? GL_RGB : GL_BGR): GL_LUMINANCE;

			// Send image data to OpenGL:
			glPixelStorei(GL_UNPACK_ALIGNMENT,4);
			glPixelStorei(GL_UNPACK_ROW_LENGTH,img_rgb->getRowStride()/nBytesPerPixel );
			glTexImage2D(GL_TEXTURE_2D, 0 /*level*/, 3 /* RGB components */, width, height,0 /*border*/, img_format, img_type, img_rgb->get_unsafe(0,0) );
			glPixelStorei(GL_UNPACK_ROW_LENGTH,0);  // Reset
		}
		else
		{
			/* Error occured */
			const std::string sError = mrpt::format("[CAssimpModel] Couldn't load texture image: '%s'", fileloc.c_str());
			cout << sError << endl;
#ifdef _MSC_VER
			OutputDebugStringA(&sError[0]);
#endif
		}
	}
}


#endif // MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
