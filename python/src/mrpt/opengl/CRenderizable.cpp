#include <any>
#include <functional>
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
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
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

void bind_mrpt_opengl_CRenderizable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::opengl::TCullFace file:mrpt/opengl/CRenderizable.h line:43
	pybind11::enum_<mrpt::opengl::TCullFace>(M("mrpt::opengl"), "TCullFace", "Enum for cull face modes in triangle-based shaders.\n  \n\n CRenderizableShaderTriangles, CRenderizableShaderTexturedTriangles\n  \n\n\n ")
		.value("NONE", mrpt::opengl::TCullFace::NONE)
		.value("BACK", mrpt::opengl::TCullFace::BACK)
		.value("FRONT", mrpt::opengl::TCullFace::FRONT);

;

	{ // mrpt::opengl::CRenderizable file:mrpt/opengl/CRenderizable.h line:71
		pybind11::class_<mrpt::opengl::CRenderizable, std::shared_ptr<mrpt::opengl::CRenderizable>, mrpt::serialization::CSerializable> cl(M("mrpt::opengl"), "CRenderizable", "The base class of 3D objects that can be directly rendered through OpenGL.\n  In this class there are a set of common properties to all 3D objects,\nmainly:\n - Its SE(3) pose (x,y,z,yaw,pitch,roll), relative to the parent object,\n or the global frame of reference for root objects (inserted into a\nmrpt::opengl::Scene).\n - A name: A name that can be optionally asigned to objects for\neasing its reference.\n - A RGBA color: This field will be used in simple elements (points,\nlines, text,...) but is ignored in more complex objects that carry their own\ncolor information (triangle sets,...)\n - Shininess: See materialShininess(float)\n\n See the main class opengl::Scene\n\n  \n opengl::Scene, mrpt::opengl\n \n\n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::GetRuntimeClass, "C++: mrpt::opengl::CRenderizable::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CRenderizable::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CRenderizable::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("setName", (void (mrpt::opengl::CRenderizable::*)(const std::string &)) &mrpt::opengl::CRenderizable::setName, "Changes the name of the object \n\nC++: mrpt::opengl::CRenderizable::setName(const std::string &) --> void", pybind11::arg("n"));
		cl.def("getName", (std::string (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getName, "Returns the name of the object \n\nC++: mrpt::opengl::CRenderizable::getName() const --> std::string");
		cl.def("isVisible", (bool (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::isVisible, "Is the object visible? \n setVisibility \n\nC++: mrpt::opengl::CRenderizable::isVisible() const --> bool");
		cl.def("setVisibility", [](mrpt::opengl::CRenderizable &o) -> void { return o.setVisibility(); }, "");
		cl.def("setVisibility", (void (mrpt::opengl::CRenderizable::*)(bool)) &mrpt::opengl::CRenderizable::setVisibility, "Set object visibility (default=true) \n isVisible \n\nC++: mrpt::opengl::CRenderizable::setVisibility(bool) --> void", pybind11::arg("visible"));
		cl.def("castShadows", (bool (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::castShadows, "Does the object cast shadows? (default=true) \n\nC++: mrpt::opengl::CRenderizable::castShadows() const --> bool");
		cl.def("castShadows", [](mrpt::opengl::CRenderizable &o) -> void { return o.castShadows(); }, "");
		cl.def("castShadows", (void (mrpt::opengl::CRenderizable::*)(bool)) &mrpt::opengl::CRenderizable::castShadows, "Enable/disable casting shadows by this object (default=true) \n\nC++: mrpt::opengl::CRenderizable::castShadows(bool) --> void", pybind11::arg("doCast"));
		cl.def("enableShowName", [](mrpt::opengl::CRenderizable &o) -> void { return o.enableShowName(); }, "");
		cl.def("enableShowName", (void (mrpt::opengl::CRenderizable::*)(bool)) &mrpt::opengl::CRenderizable::enableShowName, "Enables or disables showing the name of the object as a label when\n rendering \n\nC++: mrpt::opengl::CRenderizable::enableShowName(bool) --> void", pybind11::arg("showName"));
		cl.def("isShowNameEnabled", (bool (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::isShowNameEnabled, "enableShowName \n\nC++: mrpt::opengl::CRenderizable::isShowNameEnabled() const --> bool");
		cl.def("setPose", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const class mrpt::poses::CPose3D &)) &mrpt::opengl::CRenderizable::setPose, "Defines the SE(3) (pose=translation+rotation) of the object with respect\n to its parent \n\nC++: mrpt::opengl::CRenderizable::setPose(const class mrpt::poses::CPose3D &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("setPose", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const class mrpt::poses::CPose2D &)) &mrpt::opengl::CRenderizable::setPose, "C++: mrpt::opengl::CRenderizable::setPose(const class mrpt::poses::CPose2D &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("setPose", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const struct mrpt::math::TPose3D &)) &mrpt::opengl::CRenderizable::setPose, "C++: mrpt::opengl::CRenderizable::setPose(const struct mrpt::math::TPose3D &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("setPose", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const struct mrpt::math::TPose2D &)) &mrpt::opengl::CRenderizable::setPose, "C++: mrpt::opengl::CRenderizable::setPose(const struct mrpt::math::TPose2D &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("setPose", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const class mrpt::poses::CPoint3D &)) &mrpt::opengl::CRenderizable::setPose, "C++: mrpt::opengl::CRenderizable::setPose(const class mrpt::poses::CPoint3D &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("setPose", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const class mrpt::poses::CPoint2D &)) &mrpt::opengl::CRenderizable::setPose, "C++: mrpt::opengl::CRenderizable::setPose(const class mrpt::poses::CPoint2D &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("getPose", (struct mrpt::math::TPose3D (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getPose, "Returns the 3D pose of the object as TPose3D \n\nC++: mrpt::opengl::CRenderizable::getPose() const --> struct mrpt::math::TPose3D");
		cl.def("getCPose", (class mrpt::poses::CPose3D (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getCPose, "Returns a const ref to the 3D pose of the object as mrpt::poses::CPose3D\n (which explicitly contains the 3x3 rotation matrix) \n\nC++: mrpt::opengl::CRenderizable::getCPose() const --> class mrpt::poses::CPose3D");
		cl.def("setLocation", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(double, double, double)) &mrpt::opengl::CRenderizable::setLocation, "Changes the location of the object, keeping untouched the orientation\n \n\n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setLocation(double, double, double) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setLocation", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::CRenderizable::setLocation, "Changes the location of the object, keeping untouched the orientation\n \n\n a ref to this  \n\nC++: mrpt::opengl::CRenderizable::setLocation(const struct mrpt::math::TPoint3D_<double> &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("getColor", (struct mrpt::img::TColorf (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getColor, "Get color components as floats in the range [0,1] \n\nC++: mrpt::opengl::CRenderizable::getColor() const --> struct mrpt::img::TColorf");
		cl.def("getColor_u8", (struct mrpt::img::TColor (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getColor_u8, "Get color components as uint8_t in the range [0,255] \n\nC++: mrpt::opengl::CRenderizable::getColor_u8() const --> struct mrpt::img::TColor");
		cl.def("setColorA", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const float)) &mrpt::opengl::CRenderizable::setColorA, "Set alpha (transparency) color component in the range [0,1]\n  \n\n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setColorA(const float) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("a"));
		cl.def("setColorA_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const unsigned char)) &mrpt::opengl::CRenderizable::setColorA_u8, "Set alpha (transparency) color component in the range [0,255]\n  \n\n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setColorA_u8(const unsigned char) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("a"));
		cl.def("materialShininess", (float (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::materialShininess, "Material shininess (for specular lights in shaders that support it),\n  between 0.0f (none) to 1.0f (shiny) \n\nC++: mrpt::opengl::CRenderizable::materialShininess() const --> float");
		cl.def("materialShininess", (void (mrpt::opengl::CRenderizable::*)(float)) &mrpt::opengl::CRenderizable::materialShininess, "Material shininess (for specular lights in shaders that support it),\n  between 0.0f (none) to 1.0f (shiny) \n\nC++: mrpt::opengl::CRenderizable::materialShininess(float) --> void", pybind11::arg("shininess"));
		cl.def("setScale", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(float)) &mrpt::opengl::CRenderizable::setScale, "Scale to apply to the object, in all three axes (default=1)  \n a\n ref to this \n\nC++: mrpt::opengl::CRenderizable::setScale(float) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("s"));
		cl.def("setScale", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(float, float, float)) &mrpt::opengl::CRenderizable::setScale, "Scale to apply to the object in each axis (default=1)  \n a ref to\n this \n\nC++: mrpt::opengl::CRenderizable::setScale(float, float, float) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("sx"), pybind11::arg("sy"), pybind11::arg("sz"));
		cl.def("getScaleX", (float (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getScaleX, "Get the current scaling factor in one axis \n\nC++: mrpt::opengl::CRenderizable::getScaleX() const --> float");
		cl.def("getScaleY", (float (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getScaleY, "Get the current scaling factor in one axis \n\nC++: mrpt::opengl::CRenderizable::getScaleY() const --> float");
		cl.def("getScaleZ", (float (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getScaleZ, "Get the current scaling factor in one axis \n\nC++: mrpt::opengl::CRenderizable::getScaleZ() const --> float");
		cl.def("setColor", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const struct mrpt::img::TColorf &)) &mrpt::opengl::CRenderizable::setColor, "Changes the default object color \n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setColor(const struct mrpt::img::TColorf &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("c"));
		cl.def("setColor", [](mrpt::opengl::CRenderizable &o, float const & a0, float const & a1, float const & a2) -> mrpt::opengl::CRenderizable & { return o.setColor(a0, a1, a2); }, "", pybind11::return_value_policy::automatic, pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setColor", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(float, float, float, float)) &mrpt::opengl::CRenderizable::setColor, "Set the color components of this object (R,G,B,Alpha, in the range 0-1)\n \n\n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setColor(float, float, float, float) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("setColor_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::CRenderizable::setColor_u8, "* Changes the default object color \n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setColor_u8(const struct mrpt::img::TColor &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("c"));
		cl.def("setColor_u8", [](mrpt::opengl::CRenderizable &o, uint8_t const & a0, uint8_t const & a1, uint8_t const & a2) -> mrpt::opengl::CRenderizable & { return o.setColor_u8(a0, a1, a2); }, "", pybind11::return_value_policy::automatic, pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setColor_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(uint8_t, uint8_t, uint8_t, uint8_t)) &mrpt::opengl::CRenderizable::setColor_u8, "Set the color components of this object (R,G,B,Alpha, in the range\n 0-255)  \n\n a ref to this \n\nC++: mrpt::opengl::CRenderizable::setColor_u8(uint8_t, uint8_t, uint8_t, uint8_t) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("cullElegible", (bool (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::cullElegible, "Return false if this object should never be checked for being culled out\n (=not rendered if its bbox are out of the screen limits).\n For example, skyboxes or other special effects.\n\nC++: mrpt::opengl::CRenderizable::cullElegible() const --> bool");
		cl.def("toYAMLMap", (void (mrpt::opengl::CRenderizable::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::CRenderizable::toYAMLMap, "Used from Scene::asYAML().\n \n\n (New in MRPT 2.4.2) \n\nC++: mrpt::opengl::CRenderizable::toYAMLMap(class mrpt::containers::yaml &) const --> void", pybind11::arg("propertiesMap"));
		cl.def("isCompositeObject", (bool (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::isCompositeObject, "Should return true if enqueueForRenderRecursive() is defined since\n  the object has inner children. Examples: CSetOfObjects, CAssimpModel.\n\nC++: mrpt::opengl::CRenderizable::isCompositeObject() const --> bool");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::renderUpdateBuffers, "Called whenever m_outdatedBuffers is true: used to re-generate\n OpenGL vertex buffers, etc. before they are sent for rendering in\n render() \n\nC++: mrpt::opengl::CRenderizable::renderUpdateBuffers() const --> void");
		cl.def("updateBuffers", (void (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::updateBuffers, "Calls renderUpdateBuffers() and clear the flag that is set with\n notifyChange() \n\nC++: mrpt::opengl::CRenderizable::updateBuffers() const --> void");
		cl.def("notifyChange", (void (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::notifyChange, "Call to enable calling renderUpdateBuffers() before the next\n render() rendering iteration. \n\nC++: mrpt::opengl::CRenderizable::notifyChange() const --> void");
		cl.def("notifyBBoxChange", (void (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::notifyBBoxChange, "C++: mrpt::opengl::CRenderizable::notifyBBoxChange() const --> void");
		cl.def("hasToUpdateBuffers", (bool (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::hasToUpdateBuffers, "Returns whether notifyChange() has been invoked since the last call\n to renderUpdateBuffers(), meaning the latter needs to be called again\n before rendering.\n\nC++: mrpt::opengl::CRenderizable::hasToUpdateBuffers() const --> bool");
		cl.def("traceRay", (bool (mrpt::opengl::CRenderizable::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CRenderizable::traceRay, "Simulation of ray-trace, given a pose. Returns true if the ray\n effectively collisions with the object (returning the distance to the\n origin of the ray in \"dist\"), or false in other case. \"dist\" variable\n yields undefined behaviour when false is returned\n\nC++: mrpt::opengl::CRenderizable::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("getBoundingBox", (struct mrpt::math::TBoundingBox_<double> (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getBoundingBox, "Evaluates the bounding box of this object (including possible\n children) in the coordinate frame of my parent object,\n i.e. if this object pose changes, the bbox returned here will change too.\n This is in contrast with the local bbox returned by getBoundingBoxLocal()\n\nC++: mrpt::opengl::CRenderizable::getBoundingBox() const --> struct mrpt::math::TBoundingBox_<double>");
		cl.def("getBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<double> (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getBoundingBoxLocal, "Evaluates the bounding box of this object (including possible\n children) in the coordinate frame of my parent object,\n i.e. if this object pose changes, the bbox returned here will change too.\n This is in contrast with the local bbox returned by getBoundingBoxLocal()\n\nC++: mrpt::opengl::CRenderizable::getBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<double>");
		cl.def("getBoundingBoxLocalf", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getBoundingBoxLocalf, "the bbox. const refs are not returned for multi-thread safety.\n\nC++: mrpt::opengl::CRenderizable::getBoundingBoxLocalf() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("getLocalRepresentativePoint", (struct mrpt::math::TPoint3D_<float> (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::getLocalRepresentativePoint, "Provide a representative point (in object local coordinates), used to\n sort objects by eye-distance while rendering with transparencies\n (Default=[0,0,0]) \n\nC++: mrpt::opengl::CRenderizable::getLocalRepresentativePoint() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("setLocalRepresentativePoint", (void (mrpt::opengl::CRenderizable::*)(const struct mrpt::math::TPoint3D_<float> &)) &mrpt::opengl::CRenderizable::setLocalRepresentativePoint, "See getLocalRepresentativePoint() \n\nC++: mrpt::opengl::CRenderizable::setLocalRepresentativePoint(const struct mrpt::math::TPoint3D_<float> &) --> void", pybind11::arg("p"));
		cl.def("labelObject", (class mrpt::opengl::CText & (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::labelObject, "Returns or constructs (in its first invokation) the associated\n mrpt::opengl::CText object representing the label of the object.\n \n\n enableShowName()\n\nC++: mrpt::opengl::CRenderizable::labelObject() const --> class mrpt::opengl::CText &", pybind11::return_value_policy::automatic);
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CRenderizable::*)()) &mrpt::opengl::CRenderizable::freeOpenGLResources, "Free opengl buffers \n\nC++: mrpt::opengl::CRenderizable::freeOpenGLResources() --> void");
		cl.def("initializeTextures", (void (mrpt::opengl::CRenderizable::*)() const) &mrpt::opengl::CRenderizable::initializeTextures, "Initializes all textures (loads them into opengl memory). \n\nC++: mrpt::opengl::CRenderizable::initializeTextures() const --> void");
		cl.def("assign", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CRenderizable::*)(const class mrpt::opengl::CRenderizable &)) &mrpt::opengl::CRenderizable::operator=, "C++: mrpt::opengl::CRenderizable::operator=(const class mrpt::opengl::CRenderizable &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::CRenderizable::RenderContext file:mrpt/opengl/CRenderizable.h line:339
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CRenderizable::RenderContext, std::shared_ptr<mrpt::opengl::CRenderizable::RenderContext>> cl(enclosing_class, "RenderContext", "Context for calls to render() ");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CRenderizable::RenderContext(); } ) );
			cl.def_readwrite("shader_id", &mrpt::opengl::CRenderizable::RenderContext::shader_id);
			cl.def_readwrite("activeCullFace", &mrpt::opengl::CRenderizable::RenderContext::activeCullFace);
			cl.def_readwrite("activeLights", &mrpt::opengl::CRenderizable::RenderContext::activeLights);
			cl.def_readwrite("activeTextureUnit", &mrpt::opengl::CRenderizable::RenderContext::activeTextureUnit);
		}

	}
}
