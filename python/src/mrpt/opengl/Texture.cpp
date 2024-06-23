#include <array>
#include <iterator>
#include <memory>
#include <mrpt/containers/NonCopiableData.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/opengl/Texture.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <shared_mutex>
#include <sstream> // __str__
#include <string>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_opengl_Texture(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::texture_name_unit_t file:mrpt/opengl/Texture.h line:23
		pybind11::class_<mrpt::opengl::texture_name_unit_t, std::shared_ptr<mrpt::opengl::texture_name_unit_t>> cl(M("mrpt::opengl"), "texture_name_unit_t", "Texture \"name\" and \"unit\". \n Texture ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::texture_name_unit_t(); } ) );
		cl.def( pybind11::init( [](unsigned int const & a0){ return new mrpt::opengl::texture_name_unit_t(a0); } ), "doc" , pybind11::arg("Name"));
		cl.def( pybind11::init<unsigned int, int>(), pybind11::arg("Name"), pybind11::arg("Unit") );

		cl.def( pybind11::init( [](mrpt::opengl::texture_name_unit_t const &o){ return new mrpt::opengl::texture_name_unit_t(o); } ) );
		cl.def_readwrite("name", &mrpt::opengl::texture_name_unit_t::name);
		cl.def_readwrite("unit", &mrpt::opengl::texture_name_unit_t::unit);
		cl.def("assign", (struct mrpt::opengl::texture_name_unit_t & (mrpt::opengl::texture_name_unit_t::*)(const struct mrpt::opengl::texture_name_unit_t &)) &mrpt::opengl::texture_name_unit_t::operator=, "C++: mrpt::opengl::texture_name_unit_t::operator=(const struct mrpt::opengl::texture_name_unit_t &) --> struct mrpt::opengl::texture_name_unit_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::Texture file:mrpt/opengl/Texture.h line:40
		pybind11::class_<mrpt::opengl::Texture, std::shared_ptr<mrpt::opengl::Texture>> cl(M("mrpt::opengl"), "Texture", "Resource management for OpenGL 2D or Cube textures.\n\n The texture is generated when images are assigned via\n assignImage2D() or assignCubeImages().\n\n \n CRenderizableShaderTexturedTriangles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::Texture(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::Texture const &o){ return new mrpt::opengl::Texture(o); } ) );
		cl.def("assignImage2D", [](mrpt::opengl::Texture &o, const class mrpt::img::CImage & a0, const struct mrpt::opengl::Texture::Options & a1) -> void { return o.assignImage2D(a0, a1); }, "", pybind11::arg("rgb"), pybind11::arg("o"));
		cl.def("assignImage2D", (void (mrpt::opengl::Texture::*)(const class mrpt::img::CImage &, const struct mrpt::opengl::Texture::Options &, int)) &mrpt::opengl::Texture::assignImage2D, "This is how an 2D texture image is loaded into this object, and a\n texture ID is generated underneath. Valid image formats are 8bit per\n channel RGB or RGBA.\n\nC++: mrpt::opengl::Texture::assignImage2D(const class mrpt::img::CImage &, const struct mrpt::opengl::Texture::Options &, int) --> void", pybind11::arg("rgb"), pybind11::arg("o"), pybind11::arg("textureUnit"));
		cl.def("assignImage2D", [](mrpt::opengl::Texture &o, const class mrpt::img::CImage & a0, const class mrpt::img::CImage & a1, const struct mrpt::opengl::Texture::Options & a2) -> void { return o.assignImage2D(a0, a1, a2); }, "", pybind11::arg("rgb"), pybind11::arg("alpha"), pybind11::arg("o"));
		cl.def("assignImage2D", (void (mrpt::opengl::Texture::*)(const class mrpt::img::CImage &, const class mrpt::img::CImage &, const struct mrpt::opengl::Texture::Options &, int)) &mrpt::opengl::Texture::assignImage2D, "C++: mrpt::opengl::Texture::assignImage2D(const class mrpt::img::CImage &, const class mrpt::img::CImage &, const struct mrpt::opengl::Texture::Options &, int) --> void", pybind11::arg("rgb"), pybind11::arg("alpha"), pybind11::arg("o"), pybind11::arg("textureUnit"));
		cl.def("assignCubeImages", [](mrpt::opengl::Texture &o, const struct std::array<class mrpt::img::CImage, 6> & a0) -> void { return o.assignCubeImages(a0); }, "", pybind11::arg("imgs"));
		cl.def("assignCubeImages", (void (mrpt::opengl::Texture::*)(const struct std::array<class mrpt::img::CImage, 6> &, int)) &mrpt::opengl::Texture::assignCubeImages, "This is how an Cube texture is loaded into this object, and a\n texture ID is generated underneath. Valid image formats are 8bit per\n channel RGB or RGBA.\n\n Indices of faces in the array follow the numeric ordering of\n mrpt::opengl::CUBE_TEXTURE_FACE values.\n\nC++: mrpt::opengl::Texture::assignCubeImages(const struct std::array<class mrpt::img::CImage, 6> &, int) --> void", pybind11::arg("imgs"), pybind11::arg("textureUnit"));
		cl.def("initialized", (bool (mrpt::opengl::Texture::*)() const) &mrpt::opengl::Texture::initialized, "Returns true if an image has been already assigned and an OpenGL\n texture ID was already generated. \n\nC++: mrpt::opengl::Texture::initialized() const --> bool");
		cl.def("bindAsTexture2D", (void (mrpt::opengl::Texture::*)()) &mrpt::opengl::Texture::bindAsTexture2D, "Binds the texture to GL_TEXTURE_2D \n\nC++: mrpt::opengl::Texture::bindAsTexture2D() --> void");
		cl.def("bindAsCubeTexture", (void (mrpt::opengl::Texture::*)()) &mrpt::opengl::Texture::bindAsCubeTexture, "Binds the texture to GL_TEXTURE_CUBE_MAP \n\nC++: mrpt::opengl::Texture::bindAsCubeTexture() --> void");
		cl.def("unloadTexture", (void (mrpt::opengl::Texture::*)()) &mrpt::opengl::Texture::unloadTexture, "C++: mrpt::opengl::Texture::unloadTexture() --> void");
		cl.def("textureUnit", (int (mrpt::opengl::Texture::*)() const) &mrpt::opengl::Texture::textureUnit, "Texture unit = the \"i\" in GL_TEXTUREi \n\nC++: mrpt::opengl::Texture::textureUnit() const --> int");
		cl.def("textureNameID", (unsigned int (mrpt::opengl::Texture::*)() const) &mrpt::opengl::Texture::textureNameID, "C++: mrpt::opengl::Texture::textureNameID() const --> unsigned int");
		cl.def("assign", (class mrpt::opengl::Texture & (mrpt::opengl::Texture::*)(const class mrpt::opengl::Texture &)) &mrpt::opengl::Texture::operator=, "C++: mrpt::opengl::Texture::operator=(const class mrpt::opengl::Texture &) --> class mrpt::opengl::Texture &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::Texture::Options file:mrpt/opengl/Texture.h line:47
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::Texture::Options, std::shared_ptr<mrpt::opengl::Texture::Options>> cl(enclosing_class, "Options", "Options while creating a texture from an image.");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::Texture::Options(); } ) );
			cl.def_readwrite("generateMipMaps", &mrpt::opengl::Texture::Options::generateMipMaps);
			cl.def_readwrite("magnifyLinearFilter", &mrpt::opengl::Texture::Options::magnifyLinearFilter);
			cl.def_readwrite("enableTransparency", &mrpt::opengl::Texture::Options::enableTransparency);
		}

	}
	// mrpt::opengl::getNewTextureNumber() file:mrpt/opengl/Texture.h line:118
	M("mrpt::opengl").def("getNewTextureNumber", (unsigned int (*)()) &mrpt::opengl::getNewTextureNumber, "C++: mrpt::opengl::getNewTextureNumber() --> unsigned int");

	// mrpt::opengl::releaseTextureName(const unsigned int &) file:mrpt/opengl/Texture.h line:119
	M("mrpt::opengl").def("releaseTextureName", (void (*)(const unsigned int &)) &mrpt::opengl::releaseTextureName, "C++: mrpt::opengl::releaseTextureName(const unsigned int &) --> void", pybind11::arg("t"));

	{ // mrpt::opengl::CRenderizableShaderTexturedTriangles file:mrpt/opengl/CRenderizableShaderTexturedTriangles.h line:28
		pybind11::class_<mrpt::opengl::CRenderizableShaderTexturedTriangles, std::shared_ptr<mrpt::opengl::CRenderizableShaderTexturedTriangles>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CRenderizableShaderTexturedTriangles", "Renderizable generic renderer for objects using the triangles-with-a-texture\n shader.\n\n  \n CTexturedPlane, opengl::CSetOfTexturedTriangles\n \n\n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::GetRuntimeClass, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CRenderizableShaderTexturedTriangles::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::renderUpdateBuffers, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_TexturedTriangles", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)()) &mrpt::opengl::CRenderizableShaderTexturedTriangles::onUpdateBuffers_TexturedTriangles, "Must be implemented in derived classes to update the geometric entities\n to be drawn in \"m_*_buffer\" fields. \n\nC++: mrpt::opengl::CRenderizableShaderTexturedTriangles::onUpdateBuffers_TexturedTriangles() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)()) &mrpt::opengl::CRenderizableShaderTexturedTriangles::freeOpenGLResources, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::freeOpenGLResources() --> void");
		cl.def("assignImage", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(const class mrpt::img::CImage &, const class mrpt::img::CImage &)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::assignImage, "Assigns a texture and a transparency image, and enables transparency (If\n the images are not 2^N x 2^M, they will be internally filled to its\n dimensions to be powers of two)\n \n\n Images are copied, the original ones can be deleted.\n\nC++: mrpt::opengl::CRenderizableShaderTexturedTriangles::assignImage(const class mrpt::img::CImage &, const class mrpt::img::CImage &) --> void", pybind11::arg("img"), pybind11::arg("imgAlpha"));
		cl.def("assignImage", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(const class mrpt::img::CImage &)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::assignImage, "Assigns a texture image, and disable transparency.\n \n\n Images are copied, the original ones can be deleted. \n\nC++: mrpt::opengl::CRenderizableShaderTexturedTriangles::assignImage(const class mrpt::img::CImage &) --> void", pybind11::arg("img"));
		cl.def("isLightEnabled", (bool (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::isLightEnabled, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::isLightEnabled() const --> bool");
		cl.def("enableLight", [](mrpt::opengl::CRenderizableShaderTexturedTriangles &o) -> void { return o.enableLight(); }, "");
		cl.def("enableLight", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(bool)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::enableLight, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::enableLight(bool) --> void", pybind11::arg("enable"));
		cl.def("cullFaces", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(const enum mrpt::opengl::TCullFace &)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::cullFaces, "Control whether to render the FRONT, BACK, or BOTH (default) set of\n faces. Refer to docs for glCullFace() \n\nC++: mrpt::opengl::CRenderizableShaderTexturedTriangles::cullFaces(const enum mrpt::opengl::TCullFace &) --> void", pybind11::arg("cf"));
		cl.def("cullFaces", (enum mrpt::opengl::TCullFace (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::cullFaces, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::cullFaces() const --> enum mrpt::opengl::TCullFace");
		cl.def("initializeTextures", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::initializeTextures, "VERY IMPORTANT: If you use a multi-thread application, you MUST call\n this from the same thread that will later destruct the object in order to\n the OpenGL texture memory to be correctly deleted.\n  Calling this method more than once has no effects. If you use one\n thread, this method will be automatically called when rendering, so there\n is no need to explicitly call it.\n\nC++: mrpt::opengl::CRenderizableShaderTexturedTriangles::initializeTextures() const --> void");
		cl.def("getTextureImage", (const class mrpt::img::CImage & (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::getTextureImage, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::getTextureImage() const --> const class mrpt::img::CImage &", pybind11::return_value_policy::automatic);
		cl.def("getTextureAlphaImage", (const class mrpt::img::CImage & (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::getTextureAlphaImage, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::getTextureAlphaImage() const --> const class mrpt::img::CImage &", pybind11::return_value_policy::automatic);
		cl.def("textureImageHasBeenAssigned", (bool (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::textureImageHasBeenAssigned, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::textureImageHasBeenAssigned() const --> bool");
		cl.def("enableTextureLinearInterpolation", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(bool)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::enableTextureLinearInterpolation, "Enable linear interpolation of textures (default=false, use nearest\n pixel) \n\nC++: mrpt::opengl::CRenderizableShaderTexturedTriangles::enableTextureLinearInterpolation(bool) --> void", pybind11::arg("enable"));
		cl.def("textureLinearInterpolation", (bool (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::textureLinearInterpolation, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::textureLinearInterpolation() const --> bool");
		cl.def("enableTextureMipMap", (void (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(bool)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::enableTextureMipMap, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::enableTextureMipMap(bool) --> void", pybind11::arg("enable"));
		cl.def("textureMipMap", (bool (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTexturedTriangles::textureMipMap, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::textureMipMap() const --> bool");
		cl.def("assign", (class mrpt::opengl::CRenderizableShaderTexturedTriangles & (mrpt::opengl::CRenderizableShaderTexturedTriangles::*)(const class mrpt::opengl::CRenderizableShaderTexturedTriangles &)) &mrpt::opengl::CRenderizableShaderTexturedTriangles::operator=, "C++: mrpt::opengl::CRenderizableShaderTexturedTriangles::operator=(const class mrpt::opengl::CRenderizableShaderTexturedTriangles &) --> class mrpt::opengl::CRenderizableShaderTexturedTriangles &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CRenderizableShaderTriangles file:mrpt/opengl/CRenderizableShaderTriangles.h line:27
		pybind11::class_<mrpt::opengl::CRenderizableShaderTriangles, std::shared_ptr<mrpt::opengl::CRenderizableShaderTriangles>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CRenderizableShaderTriangles", "Renderizable generic renderer for objects using the triangles shader.\n\n  \n opengl::Scene\n\n \n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CRenderizableShaderTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTriangles::GetRuntimeClass, "C++: mrpt::opengl::CRenderizableShaderTriangles::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CRenderizableShaderTriangles::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CRenderizableShaderTriangles::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CRenderizableShaderTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTriangles::renderUpdateBuffers, "C++: mrpt::opengl::CRenderizableShaderTriangles::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CRenderizableShaderTriangles::*)()) &mrpt::opengl::CRenderizableShaderTriangles::onUpdateBuffers_Triangles, "Must be implemented in derived classes to update the geometric entities\n to be drawn in \"m_*_buffer\" fields. \n\nC++: mrpt::opengl::CRenderizableShaderTriangles::onUpdateBuffers_Triangles() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CRenderizableShaderTriangles::*)()) &mrpt::opengl::CRenderizableShaderTriangles::freeOpenGLResources, "C++: mrpt::opengl::CRenderizableShaderTriangles::freeOpenGLResources() --> void");
		cl.def("isLightEnabled", (bool (mrpt::opengl::CRenderizableShaderTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTriangles::isLightEnabled, "C++: mrpt::opengl::CRenderizableShaderTriangles::isLightEnabled() const --> bool");
		cl.def("enableLight", [](mrpt::opengl::CRenderizableShaderTriangles &o) -> void { return o.enableLight(); }, "");
		cl.def("enableLight", (void (mrpt::opengl::CRenderizableShaderTriangles::*)(bool)) &mrpt::opengl::CRenderizableShaderTriangles::enableLight, "C++: mrpt::opengl::CRenderizableShaderTriangles::enableLight(bool) --> void", pybind11::arg("enable"));
		cl.def("cullFaces", (void (mrpt::opengl::CRenderizableShaderTriangles::*)(const enum mrpt::opengl::TCullFace &)) &mrpt::opengl::CRenderizableShaderTriangles::cullFaces, "Control whether to render the FRONT, BACK, or BOTH (default) set of\n faces. Refer to docs for glCullFace().\n Example: If set to `cullFaces(TCullFace::BACK);`, back faces will not be\n drawn (\"culled\")\n\nC++: mrpt::opengl::CRenderizableShaderTriangles::cullFaces(const enum mrpt::opengl::TCullFace &) --> void", pybind11::arg("cf"));
		cl.def("cullFaces", (enum mrpt::opengl::TCullFace (mrpt::opengl::CRenderizableShaderTriangles::*)() const) &mrpt::opengl::CRenderizableShaderTriangles::cullFaces, "C++: mrpt::opengl::CRenderizableShaderTriangles::cullFaces() const --> enum mrpt::opengl::TCullFace");
		cl.def("assign", (class mrpt::opengl::CRenderizableShaderTriangles & (mrpt::opengl::CRenderizableShaderTriangles::*)(const class mrpt::opengl::CRenderizableShaderTriangles &)) &mrpt::opengl::CRenderizableShaderTriangles::operator=, "C++: mrpt::opengl::CRenderizableShaderTriangles::operator=(const class mrpt::opengl::CRenderizableShaderTriangles &) --> class mrpt::opengl::CRenderizableShaderTriangles &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
