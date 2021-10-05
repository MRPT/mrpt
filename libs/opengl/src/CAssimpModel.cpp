/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// This file contains portions of code from Assimp's example:
// "Sample_SimpleOpenGL.c"

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CAssimpModel.h>

#if MRPT_HAS_ASSIMP
#if defined(MRPT_ASSIMP_VERSION_MAJOR) && MRPT_ASSIMP_VERSION_MAJOR < 3
#include <aiPostProcess.h>
#include <aiScene.h>
#include <assimp.h>
#else
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/types.h>

#include <assimp/DefaultLogger.hpp>
#include <assimp/Importer.hpp>
#include <assimp/LogStream.hpp>
#endif
#endif

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <mutex>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using mrpt::img::CImage;

IMPLEMENTS_SERIALIZABLE(CAssimpModel, CRenderizable, mrpt::opengl)

namespace mrpt::opengl::internal
{
class TexturesCache
{
   public:
	static TexturesCache& Instance()
	{
		static TexturesCache i;
		return i;
	}

	struct CachedTexturesInfo
	{
		bool load_ok = false;
		bool load_attempted = false;
		mrpt::img::CImage img_rgb;
		std::optional<mrpt::img::CImage> img_alpha;
	};

	CachedTexturesInfo& get(
		const CAssimpModel::filepath_t& texturePath, bool verboseLoad)
	{
		using namespace std::string_literals;

		auto lck = mrpt::lockHelper(gTextureCacheMtx);
		auto& entry = gTextureCache[texturePath];
		if (entry.load_attempted) return entry;

		// Load images:
		// TGA is handled specially since it's not supported by OpenCV:
		if (mrpt::system::lowerCase(
				mrpt::system::extractFileExtension(texturePath)) == "tga"s)
		{
			entry.img_alpha.emplace();

			entry.load_ok = mrpt::img::CImage::loadTGA(
				texturePath, entry.img_rgb, *entry.img_alpha);
		}
		else
		{
			entry.load_ok = entry.img_rgb.loadFromFile(texturePath);
		}

		if (entry.load_ok)
		{
			if (verboseLoad)
				std::cout << "[CAssimpModel] Loaded texture: " << texturePath
						  << "\n";
		}
		else
		{
			/* Error occured */
			const std::string sError = mrpt::format(
				"[CAssimpModel] Couldn't load texture image: '%s'",
				texturePath.c_str());
			std::cerr << sError << std::endl;
		}

		entry.load_attempted = true;
		return entry;
	}

   private:
	TexturesCache() = default;
	~TexturesCache() = default;

	std::map<CAssimpModel::filepath_t, CachedTexturesInfo> gTextureCache;
	std::mutex gTextureCacheMtx;
};

struct RenderElements
{
	std::vector<mrpt::math::TPoint3Df>* lines_vbd = nullptr;
	std::vector<mrpt::img::TColor>* lines_cbd = nullptr;
	std::vector<mrpt::math::TPoint3Df>* pts_vbd = nullptr;
	std::vector<mrpt::img::TColor>* pts_cbd = nullptr;
	std::vector<mrpt::opengl::TTriangle>* tris = nullptr;

	// textures:
	const std::map<std::string, CAssimpModel::TInfoPerTexture>* ipt = nullptr;
	const std::vector<CSetOfTexturedTriangles::Ptr>* textObjs = nullptr;
};
}  // namespace mrpt::opengl::internal

struct CAssimpModel::Impl
{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	Impl() = default;
	~Impl() = default;

	Impl(const Impl& o) { *this = o; }
	Impl& operator=(const Impl&)
	{
		THROW_EXCEPTION(
			"Copying CAssimpModel objects via operator= not allowed.");
		return *this;
	}

	Assimp::Importer importer;
	const aiScene* scene = nullptr;	 // Memory owned by "importer"
#endif
};

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP

// Just return the diffuse color:
static mrpt::img::TColor apply_material(const aiMaterial* mtl);
static void get_bounding_box(
	const aiScene* sc, aiVector3D* min, aiVector3D* max);
static void get_bounding_box_for_node(
	const aiScene* sc, const aiNode* nd, aiVector3D* min, aiVector3D* max,
	aiMatrix4x4* trafo);
#endif	// MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP

void CAssimpModel::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::POINTS:
			CRenderizableShaderPoints::render(rc);
			break;
		case DefaultShaderID::TRIANGLES:
			CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CAssimpModel::renderUpdateBuffers() const
{
	// onUpdateBuffers_all: already called upon loading of the model from file.

	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

// special case for assimp: update all buffers within one run over the scene
// structure.
void CAssimpModel::onUpdateBuffers_all()
{
	auto& lines_vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& lines_cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	lines_vbd.clear();
	lines_cbd.clear();

	auto& pts_vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& pts_cbd = CRenderizableShaderPoints::m_color_buffer_data;
	pts_vbd.clear();
	pts_cbd.clear();

	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	if (!m_assimp_scene->scene) return;	 // No scene

	mrpt::opengl::internal::RenderElements re;
	re.lines_vbd = &lines_vbd;
	re.lines_cbd = &lines_cbd;
	re.pts_vbd = &pts_vbd;
	re.pts_cbd = &pts_cbd;
	re.tris = &tris;
	re.ipt = &m_textureIdMap;
	re.textObjs = &m_texturedObjects;

	ASSERT_(m_assimp_scene->scene);

	process_textures(m_assimp_scene->scene);

	const auto transf = mrpt::poses::CPose3D();
	recursive_render(
		m_assimp_scene->scene, m_assimp_scene->scene->mRootNode, transf, re);
#endif
}

// These ones: already done in onUpdateBuffers_all()
void CAssimpModel::onUpdateBuffers_Wireframe() {}
void CAssimpModel::onUpdateBuffers_Points() {}
void CAssimpModel::onUpdateBuffers_Triangles() {}

void CAssimpModel::enqueForRenderRecursive(
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq) const
{
	// Enque rendering all textured meshes:
	mrpt::opengl::CListOpenGLObjects lst;
	for (const auto& o : m_texturedObjects)
		lst.emplace_back(
			std::dynamic_pointer_cast<mrpt::opengl::CRenderizable>(o));

	mrpt::opengl::enqueForRendering(lst, state, rq);
}

uint8_t CAssimpModel::serializeGetVersion() const { return 1; }
void CAssimpModel::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	const bool empty = m_assimp_scene->scene != nullptr;
	out << empty;
	if (!empty)
	{
		// aiScene *scene = (aiScene *) m_assimp_scene->scene;
		THROW_EXCEPTION("MRPT can't serialize Assimp objects yet!");
	}
#else
	THROW_EXCEPTION("MRPT compiled without OpenGL and/or Assimp");
#endif
}

void CAssimpModel::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	THROW_EXCEPTION("MRPT can't serialize Assimp objects yet!");

	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);

			clear();
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

CAssimpModel::CAssimpModel()
	: m_assimp_scene(mrpt::make_impl<CAssimpModel::Impl>())
{
}

CAssimpModel::~CAssimpModel() { clear(); }

void CAssimpModel::clear()
{
	CRenderizable::notifyChange();

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	m_assimp_scene->importer.FreeScene();
#endif
	m_modelPath.clear();
	m_textureIdMap.clear();
	m_texturedObjects.clear();
}

void CAssimpModel::loadScene(const std::string& filepath, int flags)
{
#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
	clear();
	CRenderizable::notifyChange();

	// Assimp flags:
	const std::vector<std::pair<uint32_t, unsigned int>> flagMap = {
		{LoadFlags::RealTimeFast, aiProcessPreset_TargetRealtime_Fast},
		{LoadFlags::RealTimeQuality, aiProcessPreset_TargetRealtime_Quality},
		{LoadFlags::RealTimeMaxQuality,
		 aiProcessPreset_TargetRealtime_MaxQuality},
		{LoadFlags::FlipUVs, aiProcess_FlipUVs}};

	unsigned int pFlags = 0;
	for (const auto& p : flagMap)
		if (flags & p.first) pFlags |= p.second;

	// Own flags:
	m_verboseLoad = !!(flags & LoadFlags::Verbose);

	m_assimp_scene->scene =
		m_assimp_scene->importer.ReadFile(filepath.c_str(), pFlags);

	if (!m_assimp_scene->scene)
	{
		THROW_EXCEPTION_FMT(
			"Error importing '%s': %s", filepath.c_str(),
			m_assimp_scene->importer.GetErrorString());
	}
	m_modelPath = filepath;

	// Evaluate overall bbox:
	{
		aiVector3D scene_min, scene_max;
		get_bounding_box(m_assimp_scene->scene, &scene_min, &scene_max);
		m_bbox_min.x = scene_min.x;
		m_bbox_min.y = scene_min.y;
		m_bbox_min.z = scene_min.z;
		m_bbox_max.x = scene_max.x;
		m_bbox_max.y = scene_max.y;
		m_bbox_max.z = scene_max.z;
	}

	// Process all elements at once:
	// This populates the structures that will be attached to opengl
	// buffers
	const_cast<CAssimpModel&>(*this).onUpdateBuffers_all();

#else
	THROW_EXCEPTION("MRPT compiled without OpenGL and/or Assimp");
#endif
}

auto CAssimpModel::getBoundingBox() const -> mrpt::math::TBoundingBox
{
	return mrpt::math::TBoundingBox(m_bbox_min, m_bbox_max).compose(m_pose);
}

bool CAssimpModel::traceRay(
	[[maybe_unused]] const mrpt::poses::CPose3D& o,
	[[maybe_unused]] double& dist) const
{
	// TODO
	return false;
}

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP

static void get_bounding_box_for_node(
	const aiScene* scene, const aiNode* nd, aiVector3D* min, aiVector3D* max,
	aiMatrix4x4* trafo)
{
	aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo, &nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n)
	{
		const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t)
		{
			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp, trafo);

			min->x = std::min(min->x, tmp.x);
			min->y = std::min(min->y, tmp.y);
			min->z = std::min(min->z, tmp.z);

			max->x = std::max(max->x, tmp.x);
			max->y = std::max(max->y, tmp.y);
			max->z = std::max(max->z, tmp.z);
		}
	}

	for (n = 0; n < nd->mNumChildren; ++n)
	{
		get_bounding_box_for_node(scene, nd->mChildren[n], min, max, trafo);
	}
	*trafo = prev;
}

static void get_bounding_box(
	const aiScene* scene, aiVector3D* min, aiVector3D* max)
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	min->x = min->y = min->z = 1e10f;
	max->x = max->y = max->z = -1e10f;
	get_bounding_box_for_node(scene, scene->mRootNode, min, max, &trafo);
}

static mrpt::img::TColor color4_to_TColor(const aiColor4D& c)
{
	return mrpt::img::TColorf(c.r, c.g, c.b, c.a).asTColor();
}

static mrpt::img::TColor apply_material(const aiMaterial* mtl)
{
	aiColor4D diffuse;
	if (AI_SUCCESS ==
		aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
	{ return color4_to_TColor(diffuse); }
	else
	{
		// Default color:
		return {0xa0, 0xa0, 0xa0, 0xff};
	}
}

static mrpt::math::CMatrixDouble44 aiMatrix_to_mrpt(const aiMatrix4x4& m)
{
	mrpt::math::CMatrixDouble44 M;
	M(0, 0) = m.a1;
	M(0, 1) = m.a2;
	M(0, 2) = m.a3;
	M(0, 3) = m.a4;

	M(1, 0) = m.b1;
	M(1, 1) = m.b2;
	M(1, 2) = m.b3;
	M(1, 3) = m.b4;

	M(2, 0) = m.c1;
	M(2, 1) = m.c2;
	M(2, 2) = m.c3;
	M(2, 3) = m.c4;

	M(3, 0) = m.d1;
	M(3, 1) = m.d2;
	M(3, 2) = m.d3;
	M(3, 3) = m.d4;
	return M;
}

static mrpt::math::TPoint3Df to_mrpt(const aiVector3D& v)
{
	return {v.x, v.y, v.z};
}

void CAssimpModel::recursive_render(
	const aiScene* sc, const aiNode* nd, const mrpt::poses::CPose3D& transf,
	mrpt::opengl::internal::RenderElements& re)
{
	const aiMatrix4x4& m = nd->mTransformation;

	// update transform
	const auto nodeTransf = mrpt::poses::CPose3D(aiMatrix_to_mrpt(m));
	const mrpt::poses::CPose3D curTf = transf + nodeTransf;

	// draw all meshes assigned to this node
	for (unsigned int n = 0; n < nd->mNumMeshes; ++n)
	{
		const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

		mrpt::img::TColor color =
			apply_material(sc->mMaterials[mesh->mMaterialIndex]);

		for (unsigned int t = 0; t < mesh->mNumFaces; ++t)
		{
			const struct aiFace* face = &mesh->mFaces[t];

			switch (face->mNumIndices)
			{
				case 1:
					// GL_POINTS ================
					for (unsigned int i = 0; i < face->mNumIndices; i++)
					{
						// get group index for current index
						int vertexIndex = face->mIndices[i];
						if (mesh->mColors[0] != nullptr)
							color =
								color4_to_TColor(mesh->mColors[0][vertexIndex]);

						re.pts_vbd->emplace_back(curTf.composePoint(
							to_mrpt(mesh->mVertices[vertexIndex])));
						re.pts_cbd->emplace_back(color);
					}
					break;

				case 2:
					// GL_LINES ================
					for (unsigned int i = 0; i < face->mNumIndices; i++)
					{
						// get group index for current index
						int vertexIndex = face->mIndices[i];
						if (mesh->mColors[0] != nullptr)
							color =
								color4_to_TColor(mesh->mColors[0][vertexIndex]);

						re.lines_vbd->emplace_back(curTf.composePoint(
							to_mrpt(mesh->mVertices[vertexIndex])));
						re.lines_cbd->emplace_back(color);
					}
					break;

				case 3:
				{
					// GL_TRIANGLES ================
					const unsigned int nTri = face->mNumIndices / 3;
					ASSERT_EQUAL_(face->mNumIndices % 3, 0);

					size_t textureIdIndex = std::string::npos;
					if (mesh->HasTextureCoords(0))
					{
						ASSERT_LT_(mesh->mMaterialIndex, sc->mNumMaterials);
						const int texIndex = 0;
						aiString path;	// filename
						if (AI_SUCCESS ==
							sc->mMaterials[mesh->mMaterialIndex]->GetTexture(
								aiTextureType_DIFFUSE, texIndex, &path))
						{
							auto itIpt = re.ipt->find(path.data);
							ASSERTMSG_(
								itIpt != re.ipt->end(),
								mrpt::format(
									"Inconsistent texture data structure for "
									"texture with path: '%s'",
									path.data));

							textureIdIndex = itIpt->second.id_idx;
						}
					}

					for (unsigned int iTri = 0; iTri < nTri; iTri++)
					{
						mrpt::opengl::TTriangle tri;
						for (unsigned int v = 0; v < 3; v++)
						{
							unsigned int i = iTri * 3 + v;
							// get group index for current index
							int vertexIndex = face->mIndices[i];
							if (mesh->mColors[0] != nullptr)
								color = color4_to_TColor(
									mesh->mColors[0][vertexIndex]);

							tri.r(v) = color.R;
							tri.g(v) = color.G;
							tri.b(v) = color.B;
							tri.a(v) = color.A;

							// texture_coordinates_set=0
							if (mesh->HasTextureCoords(0))
							{
								tri.vertices[v].uv.x =
									mesh->mTextureCoords[0][vertexIndex].x;
								tri.vertices[v].uv.y =
									mesh->mTextureCoords[0][vertexIndex].y;
							}

							if (mesh->mNormals)
								tri.vertices[v].normal = curTf.rotateVector(
									to_mrpt(mesh->mNormals[vertexIndex]));

							auto pt = curTf.composePoint(
								to_mrpt(mesh->mVertices[vertexIndex]));
							tri.x(v) = pt.x;
							tri.y(v) = pt.y;
							tri.z(v) = pt.z;
						}

						if (textureIdIndex == std::string::npos)
						{
							// Append to default non-textured mesh:
							re.tris->emplace_back(std::move(tri));
						}
						else
						{
							// Append to its corresponding textured object:
							re.textObjs->at(textureIdIndex)
								->insertTriangle(tri);
						}
					}
				}
				break;
				default:
					// GL_POLYGON ================
					THROW_EXCEPTION("ASSIMP polygons not implemented yet.");
					break;
			}
		}
	}

	// draw all children
	for (unsigned int n = 0; n < nd->mNumChildren; ++n)
		recursive_render(sc, nd->mChildren[n], curTf, re);
}

void CAssimpModel::process_textures(const aiScene* scene)
{
	using namespace std::string_literals;

	if (scene->HasTextures())
		THROW_EXCEPTION(
			"Support for meshes with *embedded* textures is not implemented. "
			"Please, use external texture files or contribute a PR to mrpt "
			"with this feature.");

	m_textureIdMap.clear();
	m_texturedObjects.clear();

	/* getTexture Filenames and no. of Textures */
	for (unsigned int m = 0; m < scene->mNumMaterials; m++)
	{
		for (int texIndex = 0;; texIndex++)
		{
			bool anyFound = false;
			const std::vector<aiTextureType> texTypes = {
				aiTextureType_DIFFUSE, aiTextureType_AMBIENT,
				aiTextureType_SPECULAR};

			for (const auto texType : texTypes)
			{
				aiString path;	// filename
				aiReturn texFound =
					scene->mMaterials[m]->GetTexture(texType, texIndex, &path);
				if (texFound != AI_SUCCESS) break;

				CAssimpModel::TInfoPerTexture& ipt = m_textureIdMap[path.data];
				ipt.id_idx = std::string::npos;	 // pending
				anyFound = true;
			}
			if (!anyFound) break;
		}
	}

	const auto basepath = mrpt::system::filePathSeparatorsToNative(
		mrpt::system::extractFileDirectory(m_modelPath));

	for (auto& kv : m_textureIdMap)
	{
		// save image ID
		std::string filename = kv.first;  // get filename
		CAssimpModel::TInfoPerTexture& ipt = kv.second;
		// save texture id for filename in map:
		ipt.id_idx = m_texturedObjects.size();

		// Create "children" textured objects:
		auto& texturedObj =
			m_texturedObjects.emplace_back(CSetOfTexturedTriangles::Create());

		const std::string fileloc =
			mrpt::system::filePathSeparatorsToNative(basepath + filename);

		// Query textureCache:
		auto& cache = internal::TexturesCache::Instance();
		auto& tc = cache.get(fileloc, m_verboseLoad);

		if (tc.load_ok)
		{
			if (tc.img_alpha.has_value())
				texturedObj->assignImage(tc.img_rgb, *tc.img_alpha);
			else
				texturedObj->assignImage(tc.img_rgb);
		}
	}
}

#endif	// MRPT_HAS_OPENGL_GLUT && MRPT_HAS_ASSIMP
