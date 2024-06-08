#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <variant>
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

void bind_mrpt_opengl_DefaultShaders(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::DefaultShaderID file:mrpt/opengl/DefaultShaders.h line:23
		pybind11::class_<mrpt::opengl::DefaultShaderID, std::shared_ptr<mrpt::opengl::DefaultShaderID>> cl(M("mrpt::opengl"), "DefaultShaderID", "");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::DefaultShaderID(); } ) );
	}
	{ // mrpt::opengl::TRenderMatrices file:mrpt/opengl/TRenderMatrices.h line:29
		pybind11::class_<mrpt::opengl::TRenderMatrices, std::shared_ptr<mrpt::opengl::TRenderMatrices>> cl(M("mrpt::opengl"), "TRenderMatrices", "Rendering state related to the projection and model-view matrices.\n Used to store matrices that will be sent to shaders.\n\n The homogeneous coordinates of a rendered point comes from the product\n (from right to left) of MODEL, VIEW and PROJECTION matrices:\n\n  p = p_matrix * v_matrix * m_matrix * [x y z 1.0]'\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::TRenderMatrices(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::TRenderMatrices const &o){ return new mrpt::opengl::TRenderMatrices(o); } ) );
		cl.def_readwrite("p_matrix", &mrpt::opengl::TRenderMatrices::p_matrix);
		cl.def_readwrite("m_matrix", &mrpt::opengl::TRenderMatrices::m_matrix);
		cl.def_readwrite("v_matrix", &mrpt::opengl::TRenderMatrices::v_matrix);
		cl.def_readwrite("v_matrix_no_translation", &mrpt::opengl::TRenderMatrices::v_matrix_no_translation);
		cl.def_readwrite("pmv_matrix", &mrpt::opengl::TRenderMatrices::pmv_matrix);
		cl.def_readwrite("mv_matrix", &mrpt::opengl::TRenderMatrices::mv_matrix);
		cl.def_readwrite("light_pv", &mrpt::opengl::TRenderMatrices::light_pv);
		cl.def_readwrite("light_p", &mrpt::opengl::TRenderMatrices::light_p);
		cl.def_readwrite("light_v", &mrpt::opengl::TRenderMatrices::light_v);
		cl.def_readwrite("light_pmv", &mrpt::opengl::TRenderMatrices::light_pmv);
		cl.def_readwrite("is1stShadowMapPass", &mrpt::opengl::TRenderMatrices::is1stShadowMapPass);
		cl.def_readwrite("pinhole_model", &mrpt::opengl::TRenderMatrices::pinhole_model);
		cl.def_readwrite("FOV", &mrpt::opengl::TRenderMatrices::FOV);
		cl.def_readwrite("azimuth", &mrpt::opengl::TRenderMatrices::azimuth);
		cl.def_readwrite("elev", &mrpt::opengl::TRenderMatrices::elev);
		cl.def_readwrite("eyeDistance", &mrpt::opengl::TRenderMatrices::eyeDistance);
		cl.def_readwrite("viewport_width", &mrpt::opengl::TRenderMatrices::viewport_width);
		cl.def_readwrite("viewport_height", &mrpt::opengl::TRenderMatrices::viewport_height);
		cl.def_readwrite("initialized", &mrpt::opengl::TRenderMatrices::initialized);
		cl.def_readwrite("is_projective", &mrpt::opengl::TRenderMatrices::is_projective);
		cl.def_readwrite("eye", &mrpt::opengl::TRenderMatrices::eye);
		cl.def_readwrite("pointing", &mrpt::opengl::TRenderMatrices::pointing);
		cl.def_readwrite("up", &mrpt::opengl::TRenderMatrices::up);
		cl.def("matricesSetIdentity", (void (mrpt::opengl::TRenderMatrices::*)()) &mrpt::opengl::TRenderMatrices::matricesSetIdentity, "C++: mrpt::opengl::TRenderMatrices::matricesSetIdentity() --> void");
		cl.def("computeProjectionMatrix", (void (mrpt::opengl::TRenderMatrices::*)(float, float)) &mrpt::opengl::TRenderMatrices::computeProjectionMatrix, "Uses is_projective , vw,vh, etc. and computes p_matrix from either:\n  - pinhole_model if set, or\n  - FOV, otherwise.\n Replacement for obsolete: gluPerspective() and glOrtho() \n\nC++: mrpt::opengl::TRenderMatrices::computeProjectionMatrix(float, float) --> void", pybind11::arg("zmin"), pybind11::arg("zmax"));
		cl.def("computeLightProjectionMatrix", (void (mrpt::opengl::TRenderMatrices::*)(float, float, const struct mrpt::opengl::TLightParameters &)) &mrpt::opengl::TRenderMatrices::computeLightProjectionMatrix, "Updates light_pv \n\nC++: mrpt::opengl::TRenderMatrices::computeLightProjectionMatrix(float, float, const struct mrpt::opengl::TLightParameters &) --> void", pybind11::arg("zmin"), pybind11::arg("zmax"), pybind11::arg("lp"));
		cl.def("computeOrthoProjectionMatrix", (void (mrpt::opengl::TRenderMatrices::*)(float, float, float, float, float, float)) &mrpt::opengl::TRenderMatrices::computeOrthoProjectionMatrix, "Especial case for custom parameters of Orthographic projection.\n  Equivalent to `p_matrix = ortho(...);`.\n\n Replacement for obsolete: glOrtho()\n\nC++: mrpt::opengl::TRenderMatrices::computeOrthoProjectionMatrix(float, float, float, float, float, float) --> void", pybind11::arg("left"), pybind11::arg("right"), pybind11::arg("bottom"), pybind11::arg("top"), pybind11::arg("znear"), pybind11::arg("zfar"));
		cl.def_static("OrthoProjectionMatrix", (class mrpt::math::CMatrixFixed<float, 4, 4> (*)(float, float, float, float, float, float)) &mrpt::opengl::TRenderMatrices::OrthoProjectionMatrix, "Computes and returns an orthographic projection matrix.\n  Equivalent to obsolete glOrtho() or glm::ortho().\n\nC++: mrpt::opengl::TRenderMatrices::OrthoProjectionMatrix(float, float, float, float, float, float) --> class mrpt::math::CMatrixFixed<float, 4, 4>", pybind11::arg("left"), pybind11::arg("right"), pybind11::arg("bottom"), pybind11::arg("top"), pybind11::arg("znear"), pybind11::arg("zfar"));
		cl.def("computeNoProjectionMatrix", (void (mrpt::opengl::TRenderMatrices::*)(float, float)) &mrpt::opengl::TRenderMatrices::computeNoProjectionMatrix, "Especial case: no projection, opengl coordinates are pixels from (0,0)\n bottom-left corner.\n\nC++: mrpt::opengl::TRenderMatrices::computeNoProjectionMatrix(float, float) --> void", pybind11::arg("znear"), pybind11::arg("zfar"));
		cl.def("computeViewMatrix", (void (mrpt::opengl::TRenderMatrices::*)()) &mrpt::opengl::TRenderMatrices::computeViewMatrix, "Updates v_matrix (and v_matrix_no_translation) using the current\n  camera position and pointing-to coordinates.\n  Replacement for deprecated OpenGL gluLookAt(). \n\nC++: mrpt::opengl::TRenderMatrices::computeViewMatrix() --> void");
		cl.def_static("LookAt", [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, const struct mrpt::math::TPoint3D_<double> & a2) -> mrpt::math::CMatrixFixed<float, 4, 4> { return mrpt::opengl::TRenderMatrices::LookAt(a0, a1, a2); }, "", pybind11::arg("lookFrom"), pybind11::arg("lookAt"), pybind11::arg("up"));
		cl.def_static("LookAt", (class mrpt::math::CMatrixFixed<float, 4, 4> (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, class mrpt::math::CMatrixFixed<float, 4, 4> *)) &mrpt::opengl::TRenderMatrices::LookAt, "Computes the view matrix from a \"forward\" and an \"up\" vector.\n  Equivalent to obsolete gluLookAt() or glm::lookAt().\n\nC++: mrpt::opengl::TRenderMatrices::LookAt(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, class mrpt::math::CMatrixFixed<float, 4, 4> *) --> class mrpt::math::CMatrixFixed<float, 4, 4>", pybind11::arg("lookFrom"), pybind11::arg("lookAt"), pybind11::arg("up"), pybind11::arg("viewWithoutTranslation"));
		cl.def("projectPoint", (void (mrpt::opengl::TRenderMatrices::*)(float, float, float, float &, float &, float &) const) &mrpt::opengl::TRenderMatrices::projectPoint, "Computes the normalized coordinates (range=[0,1]) on the current\n rendering viewport of a\n point with local coordinates (wrt to the current model matrix) of\n (x,y,z).\n  The output proj_z_depth is the real distance from the eye to the point.\n\nC++: mrpt::opengl::TRenderMatrices::projectPoint(float, float, float, float &, float &, float &) const --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("proj_u"), pybind11::arg("proj_v"), pybind11::arg("proj_z_depth"));
		cl.def("projectPointPixels", (void (mrpt::opengl::TRenderMatrices::*)(float, float, float, float &, float &, float &) const) &mrpt::opengl::TRenderMatrices::projectPointPixels, "Projects a point from global world coordinates into (u,v) pixel\n coordinates. \n\nC++: mrpt::opengl::TRenderMatrices::projectPointPixels(float, float, float, float &, float &, float &) const --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("proj_u_px"), pybind11::arg("proj_v_px"), pybind11::arg("proj_depth"));
		cl.def("getLastClipZNear", (float (mrpt::opengl::TRenderMatrices::*)() const) &mrpt::opengl::TRenderMatrices::getLastClipZNear, "C++: mrpt::opengl::TRenderMatrices::getLastClipZNear() const --> float");
		cl.def("getLastClipZFar", (float (mrpt::opengl::TRenderMatrices::*)() const) &mrpt::opengl::TRenderMatrices::getLastClipZFar, "C++: mrpt::opengl::TRenderMatrices::getLastClipZFar() const --> float");
		cl.def("getLastLightClipZNear", (float (mrpt::opengl::TRenderMatrices::*)() const) &mrpt::opengl::TRenderMatrices::getLastLightClipZNear, "C++: mrpt::opengl::TRenderMatrices::getLastLightClipZNear() const --> float");
		cl.def("getLastLightClipZFar", (float (mrpt::opengl::TRenderMatrices::*)() const) &mrpt::opengl::TRenderMatrices::getLastLightClipZFar, "C++: mrpt::opengl::TRenderMatrices::getLastLightClipZFar() const --> float");
		cl.def("saveToYaml", (void (mrpt::opengl::TRenderMatrices::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::TRenderMatrices::saveToYaml, "C++: mrpt::opengl::TRenderMatrices::saveToYaml(class mrpt::containers::yaml &) const --> void", pybind11::arg("c"));
		cl.def("assign", (struct mrpt::opengl::TRenderMatrices & (mrpt::opengl::TRenderMatrices::*)(const struct mrpt::opengl::TRenderMatrices &)) &mrpt::opengl::TRenderMatrices::operator=, "C++: mrpt::opengl::TRenderMatrices::operator=(const struct mrpt::opengl::TRenderMatrices &) --> struct mrpt::opengl::TRenderMatrices &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::RenderQueueElement file:mrpt/opengl/RenderQueue.h line:25
		pybind11::class_<mrpt::opengl::RenderQueueElement, std::shared_ptr<mrpt::opengl::RenderQueueElement>> cl(M("mrpt::opengl"), "RenderQueueElement", "Each element in the queue to be rendered for each keyframe\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::RenderQueueElement(); } ) );
		cl.def( pybind11::init<const class mrpt::opengl::CRenderizable *, const struct mrpt::opengl::TRenderMatrices &>(), pybind11::arg("obj"), pybind11::arg("state") );

		cl.def_readwrite("renderState", &mrpt::opengl::RenderQueueElement::renderState);
	}
	{ // mrpt::opengl::RenderQueueStats file:mrpt/opengl/RenderQueue.h line:50
		pybind11::class_<mrpt::opengl::RenderQueueStats, std::shared_ptr<mrpt::opengl::RenderQueueStats>> cl(M("mrpt::opengl"), "RenderQueueStats", "Stats for the rendering queue\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::RenderQueueStats(); } ) );
		cl.def_readwrite("numObjTotal", &mrpt::opengl::RenderQueueStats::numObjTotal);
		cl.def_readwrite("numObjRendered", &mrpt::opengl::RenderQueueStats::numObjRendered);
	}
	// mrpt::opengl::depthAndVisibleInView(const class mrpt::opengl::CRenderizable *, const struct mrpt::opengl::TRenderMatrices &, const bool) file:mrpt/opengl/RenderQueue.h line:64
	M("mrpt::opengl").def("depthAndVisibleInView", (class std::tuple<double, bool, bool> (*)(const class mrpt::opengl::CRenderizable *, const struct mrpt::opengl::TRenderMatrices &, const bool)) &mrpt::opengl::depthAndVisibleInView, "Computes the eye-view depth of an object, and whether any part of its\n bounding box is visible by the camera in the current state.\n Return:\n  - double: Depth of representative point.\n  - bool: visible (at least in part)\n  - bool: the whole bbox is visible (only checked for CSetOfObjects)\n \n\n\nC++: mrpt::opengl::depthAndVisibleInView(const class mrpt::opengl::CRenderizable *, const struct mrpt::opengl::TRenderMatrices &, const bool) --> class std::tuple<double, bool, bool>", pybind11::arg("obj"), pybind11::arg("objState"), pybind11::arg("skipCullChecks"));

	{ // mrpt::opengl::TLightParameters file:mrpt/opengl/TLightParameters.h line:23
		pybind11::class_<mrpt::opengl::TLightParameters, std::shared_ptr<mrpt::opengl::TLightParameters>> cl(M("mrpt::opengl"), "TLightParameters", "Unidirectional lighting model parameters for triangle shaders.\n Refer to standard OpenGL literature and tutorials for the meaning of each\n field, and to the shader GLSL code itself.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::TLightParameters(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::TLightParameters const &o){ return new mrpt::opengl::TLightParameters(o); } ) );
		cl.def_readwrite("color", &mrpt::opengl::TLightParameters::color);
		cl.def_readwrite("diffuse", &mrpt::opengl::TLightParameters::diffuse);
		cl.def_readwrite("ambient", &mrpt::opengl::TLightParameters::ambient);
		cl.def_readwrite("specular", &mrpt::opengl::TLightParameters::specular);
		cl.def_readwrite("direction", &mrpt::opengl::TLightParameters::direction);
		cl.def_readwrite("shadow_bias", &mrpt::opengl::TLightParameters::shadow_bias);
		cl.def_readwrite("shadow_bias_cam2frag", &mrpt::opengl::TLightParameters::shadow_bias_cam2frag);
		cl.def_readwrite("shadow_bias_normal", &mrpt::opengl::TLightParameters::shadow_bias_normal);
		cl.def_readwrite("eyeDistance2lightShadowExtension", &mrpt::opengl::TLightParameters::eyeDistance2lightShadowExtension);
		cl.def_readwrite("minimum_shadow_map_extension_ratio", &mrpt::opengl::TLightParameters::minimum_shadow_map_extension_ratio);
		cl.def("writeToStream", (void (mrpt::opengl::TLightParameters::*)(class mrpt::serialization::CArchive &) const) &mrpt::opengl::TLightParameters::writeToStream, "C++: mrpt::opengl::TLightParameters::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def("readFromStream", (void (mrpt::opengl::TLightParameters::*)(class mrpt::serialization::CArchive &)) &mrpt::opengl::TLightParameters::readFromStream, "C++: mrpt::opengl::TLightParameters::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
		cl.def("assign", (struct mrpt::opengl::TLightParameters & (mrpt::opengl::TLightParameters::*)(const struct mrpt::opengl::TLightParameters &)) &mrpt::opengl::TLightParameters::operator=, "C++: mrpt::opengl::TLightParameters::operator=(const struct mrpt::opengl::TLightParameters &) --> struct mrpt::opengl::TLightParameters &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::opengl::TOpenGLFontStyle file:mrpt/opengl/opengl_fonts.h line:20
	pybind11::enum_<mrpt::opengl::TOpenGLFontStyle>(M("mrpt::opengl"), "TOpenGLFontStyle", pybind11::arithmetic(), "Different style for vectorized font rendering \n T2DTextData ")
		.value("FILL", mrpt::opengl::FILL)
		.value("OUTLINE", mrpt::opengl::OUTLINE)
		.value("NICE", mrpt::opengl::NICE)
		.export_values();

;

	{ // mrpt::opengl::TFontParams file:mrpt/opengl/opengl_fonts.h line:37
		pybind11::class_<mrpt::opengl::TFontParams, std::shared_ptr<mrpt::opengl::TFontParams>> cl(M("mrpt::opengl"), "TFontParams", "A description of a bitmapped or vectorized text font.\n  (Vectorized fonts are recommended for new code).\n\n \n mrpt::opengl::gl_utils::glSetFont(),\n mrpt::opengl::gl_utils::glDrawText()");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::TFontParams(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::TFontParams const &o){ return new mrpt::opengl::TFontParams(o); } ) );
		cl.def_readwrite("vfont_name", &mrpt::opengl::TFontParams::vfont_name);
		cl.def_readwrite("vfont_scale", &mrpt::opengl::TFontParams::vfont_scale);
		cl.def_readwrite("color", &mrpt::opengl::TFontParams::color);
		cl.def_readwrite("draw_shadow", &mrpt::opengl::TFontParams::draw_shadow);
		cl.def_readwrite("shadow_color", &mrpt::opengl::TFontParams::shadow_color);
		cl.def_readwrite("vfont_style", &mrpt::opengl::TFontParams::vfont_style);
		cl.def_readwrite("vfont_spacing", &mrpt::opengl::TFontParams::vfont_spacing);
		cl.def_readwrite("vfont_kerning", &mrpt::opengl::TFontParams::vfont_kerning);
		cl.def("assign", (struct mrpt::opengl::TFontParams & (mrpt::opengl::TFontParams::*)(const struct mrpt::opengl::TFontParams &)) &mrpt::opengl::TFontParams::operator=, "C++: mrpt::opengl::TFontParams::operator=(const struct mrpt::opengl::TFontParams &) --> struct mrpt::opengl::TFontParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::T2DTextData file:mrpt/opengl/opengl_fonts.h line:68
		pybind11::class_<mrpt::opengl::T2DTextData, std::shared_ptr<mrpt::opengl::T2DTextData>, mrpt::opengl::TFontParams> cl(M("mrpt::opengl"), "T2DTextData", "An auxiliary struct for holding a list of text messages in some mrpt::opengl\n & mrpt::gui classes\n  The font can be either a bitmapped or a vectorized font.\n  \n\n mrpt::opengl::CTextMessageCapable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::T2DTextData(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::T2DTextData const &o){ return new mrpt::opengl::T2DTextData(o); } ) );
		cl.def_readwrite("text", &mrpt::opengl::T2DTextData::text);
		cl.def_readwrite("x", &mrpt::opengl::T2DTextData::x);
		cl.def_readwrite("y", &mrpt::opengl::T2DTextData::y);
		cl.def("assign", (struct mrpt::opengl::T2DTextData & (mrpt::opengl::T2DTextData::*)(const struct mrpt::opengl::T2DTextData &)) &mrpt::opengl::T2DTextData::operator=, "C++: mrpt::opengl::T2DTextData::operator=(const struct mrpt::opengl::T2DTextData &) --> struct mrpt::opengl::T2DTextData &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
