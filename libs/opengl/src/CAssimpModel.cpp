/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// This file contains portions of code from Assimp's example: "Sample_SimpleOpenGL.c"

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CAssimpModel.h>

#if MRPT_HAS_ASSIMP
#	include <assimp/Importer.hpp>
#	include <assimp/cimport.h>
#	include <assimp/DefaultLogger.hpp>
#	include <assimp/LogStream.hpp>
#	include <assimp/scene.h>
#	include <assimp/postprocess.h>
#endif

MRPT_TODO("textues: SimpleTexturedOpenGL")

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CAssimpModel, CRenderizableDisplayList, mrpt::opengl )

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	void recursive_render (const aiScene *sc, const aiNode* nd);
	void apply_material(const aiMaterial *mtl);
	void set_float4(float f[4], float a, float b, float c, float d);
	void color4_to_float4(const aiColor4D *c, float f[4]);
	void get_bounding_box (const aiScene *sc,aiVector3D* min, aiVector3D* max);
	void get_bounding_box_for_node (const aiScene *sc,const aiNode* nd, aiVector3D* min, aiVector3D* max, aiMatrix4x4* trafo);
	void load_textures(const aiScene *scene);

	MRPT_TODO("to member!")
	std::string m_modelPath;
	// images / texture
	MRPT_TODO("Move to members!")
	#include <map>
	std::map<std::string, GLuint*> textureIdMap;	// map image filenames to textureIds
	GLuint*		textureIds;							// pointer to texture Array

#endif //MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP


bool m_textures_loaded = false;

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
	glShadeModel(GL_SMOOTH);		 // Enables Smooth Shading
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0f);				// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);		// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);			// The Type Of Depth Test To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculation


	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);    // Uses default lighting parameters
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_NORMALIZE);

	GLfloat LightAmbient[]= { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat LightPosition[]= { 0.0f, 0.0f, 15.0f, 1.0f };

	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT1);

	//glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	load_textures(scene); MRPT_TODO("only once!")

	recursive_render(scene, scene->mRootNode);

	//glDisable(GL_CULL_FACE);
	glDisable(GL_NORMALIZE);

	MRPT_END
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CAssimpModel::writeToStream(CStream &out,int *version) const
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
			aiScene *scene = (aiScene *) m_assimp_scene->scene;
#else
	THROW_EXCEPTION("MRPT compiled without OpenGL and/or Assimp")
#endif
		}
		

		MRPT_TODO("Serialize")
		THROW_EXCEPTION("TODO")
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
		{
			readFromStreamRender(in);

			clear();

			MRPT_TODO("Deserialize")
			THROW_EXCEPTION("TODO")

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

CAssimpModel::CAssimpModel() :
	m_bbox_min(0,0,0),
	m_bbox_max(0,0,0)
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
void apply_material(const aiMaterial *mtl)
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
		unsigned int texId = *textureIdMap[texPath.data];
		glBindTexture(GL_TEXTURE_2D, texId);
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
void recursive_render (const aiScene *sc, const aiNode* nd)
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

		apply_material(sc->mMaterials[mesh->mMaterialIndex]);


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
		recursive_render(sc, nd->mChildren[n]);

	glPopMatrix();
}

#include <mrpt/system/filesystem.h>

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

void load_textures(const aiScene *scene)
{
	if (scene->HasTextures()) THROW_EXCEPTION("Support for meshes with embedded textures is not implemented")

	/* getTexture Filenames and no. of Textures */
	for (unsigned int m=0; m<scene->mNumMaterials; m++)
	{
		int texIndex = 0;
		aiReturn texFound = AI_SUCCESS;
		aiString path;	// filename
		while (texFound == AI_SUCCESS)
		{
			texFound = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);

			// Remove double "\\" parts:
			std::string sPath=std::string(path.data);
			replaceAll(sPath,"//","/");
			replaceAll(sPath,"\\\\","\\");

			textureIdMap[sPath] = NULL; //fill map with textures, pointers still NULL yet
			texIndex++;
		}
	}

	int numTextures = textureIdMap.size();

	/* create and fill array with GL texture ids */
	textureIds = new GLuint[numTextures];
	glGenTextures(numTextures, textureIds); /* Texture name generation */

	/* define texture path */
	//std::string texturepath = "../../../test/models/Obj/";

	/* get iterator */
	std::map<std::string, GLuint*>::iterator itr = textureIdMap.begin();

	std::string basepath = mrpt::system::filePathSeparatorsToNative( mrpt::system::extractFileDirectory(m_modelPath) );
	for (int i=0; i<numTextures; i++)
	{

		//save IL image ID
		std::string filename = (*itr).first;  // get filename
		(*itr).second =  &textureIds[i];	  // save texture id for filename in map
		itr++;								  // next texture

		std::string fileloc = basepath + filename;	/* Loading of image */
		MRPT_TODO("to member, don't leak!")
		mrpt::utils::CImage *img = new mrpt::utils::CImage();

		if (img->loadFromFile(fileloc)) /* If no error occured: */
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

			const int width  = img->getWidth();
			const int height = img->getHeight();

			// Prepare image data types:
			const GLenum img_type = GL_UNSIGNED_BYTE;
			const int nBytesPerPixel = img->isColor() ? 3 : 1;
			const bool is_RGB_order = (!::strcmp(img->getChannelsOrder(),"RGB"));  // Reverse RGB <-> BGR order?
			const GLenum img_format = nBytesPerPixel==3 ? (is_RGB_order ? GL_RGB : GL_BGR): GL_LUMINANCE;

			// Send image data to OpenGL:
			glPixelStorei(GL_UNPACK_ALIGNMENT,4);
			glPixelStorei(GL_UNPACK_ROW_LENGTH,img->getRowStride()/nBytesPerPixel );
			glTexImage2D(GL_TEXTURE_2D, 0 /*level*/, 3 /* RGB components */, width, height,0 /*border*/, img_format, img_type, img->get_unsafe(0,0) );
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
