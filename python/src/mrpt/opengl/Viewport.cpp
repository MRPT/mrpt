#include <any>
#include <functional>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/Viewport.h>
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

// mrpt::opengl::mrptEventGLPreRender file:mrpt/opengl/Viewport.h line:543
struct PyCallBack_mrpt_opengl_mrptEventGLPreRender : public mrpt::opengl::mrptEventGLPreRender {
	using mrpt::opengl::mrptEventGLPreRender::mrptEventGLPreRender;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::mrptEventGLPreRender *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventGLPreRender::do_nothing();
	}
};

// mrpt::opengl::mrptEventGLPostRender file:mrpt/opengl/Viewport.h line:566
struct PyCallBack_mrpt_opengl_mrptEventGLPostRender : public mrpt::opengl::mrptEventGLPostRender {
	using mrpt::opengl::mrptEventGLPostRender::mrptEventGLPostRender;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::mrptEventGLPostRender *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventGLPostRender::do_nothing();
	}
};

// mrpt::opengl::Scene file:mrpt/opengl/Scene.h line:60
struct PyCallBack_mrpt_opengl_Scene : public mrpt::opengl::Scene {
	using mrpt::opengl::Scene::Scene;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Scene *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return Scene::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Scene *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return Scene::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Scene *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return Scene::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Scene *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Scene::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Scene *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Scene::serializeFrom(a0, a1);
	}
};

void bind_mrpt_opengl_Viewport(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::mrptEventGLPreRender file:mrpt/opengl/Viewport.h line:543
		pybind11::class_<mrpt::opengl::mrptEventGLPreRender, std::shared_ptr<mrpt::opengl::mrptEventGLPreRender>, PyCallBack_mrpt_opengl_mrptEventGLPreRender, mrpt::system::mrptEvent> cl(M("mrpt::opengl"), "mrptEventGLPreRender", "An event sent by an mrpt::opengl::Viewport just after clearing the\n viewport and setting the GL_PROJECTION matrix, and before calling the scene\n OpenGL drawing primitives.\n\n While handling this event you can call OpenGL glDraw(), etc.\n\n IMPORTANTE NOTICE: Event handlers in your observer class will most likely be\n invoked from an internal GUI thread of MRPT, so all your code in the handler\n must be thread safe.");
		cl.def( pybind11::init<const class mrpt::opengl::Viewport *>(), pybind11::arg("obj") );

	}
	{ // mrpt::opengl::mrptEventGLPostRender file:mrpt/opengl/Viewport.h line:566
		pybind11::class_<mrpt::opengl::mrptEventGLPostRender, std::shared_ptr<mrpt::opengl::mrptEventGLPostRender>, PyCallBack_mrpt_opengl_mrptEventGLPostRender, mrpt::system::mrptEvent> cl(M("mrpt::opengl"), "mrptEventGLPostRender", "An event sent by an mrpt::opengl::Viewport after calling the scene\n OpenGL drawing primitives and before doing a glSwapBuffers\n\n  While handling this event you can call OpenGL glBegin(),glEnd(),gl*\n functions or those in mrpt::opengl::gl_utils to draw stuff *on the top* of\n the normal\n   objects contained in the Scene.\n\n  IMPORTANTE NOTICE: Event handlers in your observer class will most likely\n be invoked from an internal GUI thread of MRPT,\n    so all your code in the handler must be thread safe.");
		cl.def( pybind11::init<const class mrpt::opengl::Viewport *>(), pybind11::arg("obj") );

	}
	{ // mrpt::opengl::Scene file:mrpt/opengl/Scene.h line:60
		pybind11::class_<mrpt::opengl::Scene, std::shared_ptr<mrpt::opengl::Scene>, PyCallBack_mrpt_opengl_Scene, mrpt::serialization::CSerializable> cl(M("mrpt::opengl"), "Scene", "This class allows the user to create, load, save, and render 3D scenes using\n OpenGL primitives.\n The class can be understood as a program to be run over OpenGL, containing\n a sequence of viewport definitions,\n rendering primitives, etc.\n\n It can contain from 1 up to any number of Viewports, each one\n associated a set of OpenGL objects and, optionally, a preferred camera\n position. Both orthogonal (2D/3D) and projection\n camera models can be used for each viewport independently, greatly\n increasing the possibilities of rendered scenes.\n\n An object of Scene always contains at least one viewport\n (mrpt::opengl::Viewport), named \"main\". Optionally, any\n number of other viewports may exist. Viewports are referenced by their\n names, case-sensitive strings. Each viewport contains\n a different 3D scene (i.e. they render different objects), though a\n mechanism exist to share the same 3D scene by a number of\n viewports so memory is not wasted replicating the same objects (see\n Viewport::setCloneView ).\n\n The main rendering method, Scene::render(), assumes a viewport has\n been set-up for the entire target window. That\n method will internally make the required calls to opengl for creating the\n additional viewports. Note that only the depth\n buffer is cleared by default for each (non-main) viewport, to allow\n transparencies. This can be disabled by the approppriate\n member in Viewport.\n\n An object Scene can be saved to a [\".3Dscene\"\n file](robotics_file_formats.html) using CFileOutputStream or with the direct\n method Scene::saveToFile() for posterior visualization from the\n standalone application \n\n It can be also displayed in real-time using windows in mrpt::gui or\n serialized over a network socket, etc.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::Scene(); }, [](){ return new PyCallBack_mrpt_opengl_Scene(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_Scene const &o){ return new PyCallBack_mrpt_opengl_Scene(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::Scene const &o){ return new mrpt::opengl::Scene(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::Scene> (*)()) &mrpt::opengl::Scene::Create, "C++: mrpt::opengl::Scene::Create() --> class std::shared_ptr<class mrpt::opengl::Scene>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::Scene::GetRuntimeClassIdStatic, "C++: mrpt::opengl::Scene::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::Scene::*)() const) &mrpt::opengl::Scene::GetRuntimeClass, "C++: mrpt::opengl::Scene::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::Scene::*)() const) &mrpt::opengl::Scene::clone, "C++: mrpt::opengl::Scene::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::Scene::CreateObject, "C++: mrpt::opengl::Scene::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::opengl::Scene & (mrpt::opengl::Scene::*)(const class mrpt::opengl::Scene &)) &mrpt::opengl::Scene::operator=, "C++: mrpt::opengl::Scene::operator=(const class mrpt::opengl::Scene &) --> class mrpt::opengl::Scene &", pybind11::return_value_policy::automatic, pybind11::arg("obj"));
		cl.def("insert", [](mrpt::opengl::Scene &o, const class std::shared_ptr<class mrpt::opengl::CRenderizable> & a0) -> void { return o.insert(a0); }, "", pybind11::arg("newObject"));
		cl.def("insert", (void (mrpt::opengl::Scene::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &, const std::string &)) &mrpt::opengl::Scene::insert, "Insert a new object into the scene, in the given viewport (by default,\n into the \"main\" viewport).\n  The viewport must be created previously, an exception will be raised if\n the given name does not correspond to\n   an existing viewport.\n \n\n createViewport, getViewport\n\nC++: mrpt::opengl::Scene::insert(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &, const std::string &) --> void", pybind11::arg("newObject"), pybind11::arg("viewportName"));
		cl.def("createViewport", (class std::shared_ptr<class mrpt::opengl::Viewport> (mrpt::opengl::Scene::*)(const std::string &)) &mrpt::opengl::Scene::createViewport, "Creates a new viewport, adding it to the scene and returning a pointer\n to the new object. Names (case-sensitive) cannot be duplicated: if the\n name provided coincides with an already existing viewport, a pointer to\n the existing object will be returned. The first, default viewport, is\n named \"main\".\n\nC++: mrpt::opengl::Scene::createViewport(const std::string &) --> class std::shared_ptr<class mrpt::opengl::Viewport>", pybind11::arg("viewportName"));
		cl.def("getViewport", [](mrpt::opengl::Scene const &o) -> std::shared_ptr<class mrpt::opengl::Viewport> { return o.getViewport(); }, "");
		cl.def("getViewport", (class std::shared_ptr<class mrpt::opengl::Viewport> (mrpt::opengl::Scene::*)(const std::string &) const) &mrpt::opengl::Scene::getViewport, "Returns the viewport with the given name, or nullptr if it does not\n exist; note that the default viewport is named \"main\" and initially\n occupies the entire rendering area.\n\nC++: mrpt::opengl::Scene::getViewport(const std::string &) const --> class std::shared_ptr<class mrpt::opengl::Viewport>", pybind11::arg("viewportName"));
		cl.def("render", (void (mrpt::opengl::Scene::*)() const) &mrpt::opengl::Scene::render, "Render this scene \n\nC++: mrpt::opengl::Scene::render() const --> void");
		cl.def("viewportsCount", (size_t (mrpt::opengl::Scene::*)() const) &mrpt::opengl::Scene::viewportsCount, "C++: mrpt::opengl::Scene::viewportsCount() const --> size_t");
		cl.def("clear", [](mrpt::opengl::Scene &o) -> void { return o.clear(); }, "");
		cl.def("clear", (void (mrpt::opengl::Scene::*)(bool)) &mrpt::opengl::Scene::clear, "Clear the list of objects and viewports in the scene, deleting objects'\n memory, and leaving just the default viewport with the default values.\n\nC++: mrpt::opengl::Scene::clear(bool) --> void", pybind11::arg("createMainViewport"));
		cl.def("enableFollowCamera", (void (mrpt::opengl::Scene::*)(bool)) &mrpt::opengl::Scene::enableFollowCamera, "If disabled (default), the SceneViewer application will ignore the\n camera of the \"main\" viewport and keep the viewport selected by the user\n by hand; otherwise, the camera in the \"main\" viewport prevails.\n \n\n followCamera\n\nC++: mrpt::opengl::Scene::enableFollowCamera(bool) --> void", pybind11::arg("enabled"));
		cl.def("followCamera", (bool (mrpt::opengl::Scene::*)() const) &mrpt::opengl::Scene::followCamera, "Return the value of \"followCamera\"\n \n\n enableFollowCamera\n\nC++: mrpt::opengl::Scene::followCamera() const --> bool");
		cl.def("getByName", [](mrpt::opengl::Scene &o, const std::string & a0) -> std::shared_ptr<class mrpt::opengl::CRenderizable> { return o.getByName(a0); }, "", pybind11::arg("str"));
		cl.def("getByName", (class std::shared_ptr<class mrpt::opengl::CRenderizable> (mrpt::opengl::Scene::*)(const std::string &, const std::string &)) &mrpt::opengl::Scene::getByName, "Returns the first object with a given name, or nullptr (an empty smart\n pointer) if not found.\n\nC++: mrpt::opengl::Scene::getByName(const std::string &, const std::string &) --> class std::shared_ptr<class mrpt::opengl::CRenderizable>", pybind11::arg("str"), pybind11::arg("viewportName"));
		cl.def("removeObject", [](mrpt::opengl::Scene &o, const class std::shared_ptr<class mrpt::opengl::CRenderizable> & a0) -> void { return o.removeObject(a0); }, "", pybind11::arg("obj"));
		cl.def("removeObject", (void (mrpt::opengl::Scene::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &, const std::string &)) &mrpt::opengl::Scene::removeObject, "Removes the given object from the scene (it also deletes the object to\n free its memory).\n\nC++: mrpt::opengl::Scene::removeObject(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &, const std::string &) --> void", pybind11::arg("obj"), pybind11::arg("viewportName"));
		cl.def("initializeTextures", (void (mrpt::opengl::Scene::*)()) &mrpt::opengl::Scene::initializeTextures, "Initializes all textures in the scene (See\n opengl::CTexturedPlane::initializeTextures)\n\nC++: mrpt::opengl::Scene::initializeTextures() --> void");
		cl.def("dumpListOfObjects", (void (mrpt::opengl::Scene::*)(class std::vector<std::string > &) const) &mrpt::opengl::Scene::dumpListOfObjects, "Retrieves a list of all objects in text form.\n 	\n\n Prefer asYAML() (since MRPT 2.1.3) \n\nC++: mrpt::opengl::Scene::dumpListOfObjects(class std::vector<std::string > &) const --> void", pybind11::arg("lst"));
		cl.def("asYAML", (class mrpt::containers::yaml (mrpt::opengl::Scene::*)() const) &mrpt::opengl::Scene::asYAML, "Prints all viewports and objects in human-readable YAML form.\n Note that not all objects data is serialized, so this method is not\n suitable for deserialization (for that, use saveToFile(), loadFromFile()\n instead).\n \n\n (New in MRPT 2.1.3) \n\nC++: mrpt::opengl::Scene::asYAML() const --> class mrpt::containers::yaml");
		cl.def("saveToFile", (bool (mrpt::opengl::Scene::*)(const std::string &) const) &mrpt::opengl::Scene::saveToFile, "Saves the scene to a [\".3Dscene\" file](robotics_file_formats.html),\n loadable by: \n \n\n loadFromFile\n \n\n false on any error.\n\nC++: mrpt::opengl::Scene::saveToFile(const std::string &) const --> bool", pybind11::arg("fil"));
		cl.def("loadFromFile", (bool (mrpt::opengl::Scene::*)(const std::string &)) &mrpt::opengl::Scene::loadFromFile, "Loads the scene from a [\".3Dscene\" file](robotics_file_formats.html).\n \n\n saveToFile\n \n\n false on any error.\n\nC++: mrpt::opengl::Scene::loadFromFile(const std::string &) --> bool", pybind11::arg("fil"));
		cl.def("traceRay", (bool (mrpt::opengl::Scene::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::Scene::traceRay, "Traces a ray\n\nC++: mrpt::opengl::Scene::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("getBoundingBox", [](mrpt::opengl::Scene const &o) -> mrpt::math::TBoundingBox_<double> { return o.getBoundingBox(); }, "");
		cl.def("getBoundingBox", (struct mrpt::math::TBoundingBox_<double> (mrpt::opengl::Scene::*)(const std::string &) const) &mrpt::opengl::Scene::getBoundingBox, "Evaluates the bounding box of the scene in the given viewport (default:\n \"main\"). \n\nC++: mrpt::opengl::Scene::getBoundingBox(const std::string &) const --> struct mrpt::math::TBoundingBox_<double>", pybind11::arg("vpn"));
		cl.def("unloadShaders", (void (mrpt::opengl::Scene::*)()) &mrpt::opengl::Scene::unloadShaders, "Ensure all shaders are unloaded in all viewports \n\nC++: mrpt::opengl::Scene::unloadShaders() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::Scene::*)()) &mrpt::opengl::Scene::freeOpenGLResources, "Ensure all OpenGL buffers are destroyed. \n\nC++: mrpt::opengl::Scene::freeOpenGLResources() --> void");
	}
}
