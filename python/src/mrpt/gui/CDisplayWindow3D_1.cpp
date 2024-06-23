#include <functional>
#include <iterator>
#include <memory>
#include <mrpt/containers/yaml.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Viewport.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <type_traits>
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

// mrpt::gui::mrptEvent3DWindowGrabImageFile file:mrpt/gui/CDisplayWindow3D.h line:409
struct PyCallBack_mrpt_gui_mrptEvent3DWindowGrabImageFile : public mrpt::gui::mrptEvent3DWindowGrabImageFile {
	using mrpt::gui::mrptEvent3DWindowGrabImageFile::mrptEvent3DWindowGrabImageFile;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::mrptEvent3DWindowGrabImageFile *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEvent3DWindowGrabImageFile::do_nothing();
	}
};

// mrpt::gui::CGlCanvasBase file:mrpt/gui/CGlCanvasBase.h line:24
struct PyCallBack_mrpt_gui_CGlCanvasBase : public mrpt::gui::CGlCanvasBase {
	using mrpt::gui::CGlCanvasBase::CGlCanvasBase;

	void setCameraParams(const struct mrpt::gui::CGlCanvasBase::CamaraParams & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setCameraParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraParams(a0);
	}
	void setCameraPointing(float a0, float a1, float a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setCameraPointing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraPointing(a0, a1, a2);
	}
	void setZoomDistance(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setZoomDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setZoomDistance(a0);
	}
	void setAzimuthDegrees(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setAzimuthDegrees");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setAzimuthDegrees(a0);
	}
	void setElevationDegrees(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setElevationDegrees");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setElevationDegrees(a0);
	}
	void setCameraProjective(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setCameraProjective");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraProjective(a0);
	}
	void setCameraFOV(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "setCameraFOV");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraFOV(a0);
	}
	void OnUserManuallyMovesCamera(float a0, float a1, float a2, float a3, float a4, float a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "OnUserManuallyMovesCamera");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::OnUserManuallyMovesCamera(a0, a1, a2, a3, a4, a5);
	}
	void swapBuffers() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "swapBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGlCanvasBase::swapBuffers\"");
	}
	void preRender() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "preRender");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGlCanvasBase::preRender\"");
	}
	void postRender() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "postRender");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGlCanvasBase::postRender\"");
	}
	void renderError(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "renderError");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGlCanvasBase::renderError\"");
	}
	double renderCanvas(int a0, int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBase *>(this), "renderCanvas");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CGlCanvasBase::renderCanvas(a0, a1);
	}
};

void bind_mrpt_gui_CDisplayWindow3D_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::gui::mrptEvent3DWindowGrabImageFile file:mrpt/gui/CDisplayWindow3D.h line:409
		pybind11::class_<mrpt::gui::mrptEvent3DWindowGrabImageFile, std::shared_ptr<mrpt::gui::mrptEvent3DWindowGrabImageFile>, PyCallBack_mrpt_gui_mrptEvent3DWindowGrabImageFile, mrpt::system::mrptEvent> cl(M("mrpt::gui"), "mrptEvent3DWindowGrabImageFile", "An event sent by a CDisplayWindow3D window when an image is saved after\n enabling this feature with CDisplayWindow3D::grabImagesStart()\n\n  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread,\n    so all your code in the handler must be thread safe.");
		cl.def( pybind11::init<class mrpt::gui::CDisplayWindow3D *, const std::string &>(), pybind11::arg("obj"), pybind11::arg("_img_file") );

	}
	{ // mrpt::gui::CDisplayWindow3DLocker file:mrpt/gui/CDisplayWindow3D.h line:438
		pybind11::class_<mrpt::gui::CDisplayWindow3DLocker, std::shared_ptr<mrpt::gui::CDisplayWindow3DLocker>> cl(M("mrpt::gui"), "CDisplayWindow3DLocker", "Auxiliary class for safely claiming the 3DScene of a\n mrpt::gui::CDisplayWindow3D.\n The mutex will be hold between ctor and dtor calls of objects of this class,\n safely releasing\n the lock upon exceptions. See example usage code in docs of\n mrpt::gui::CDisplayWindow3D\n\n \n\n \n New in MRPT 1.5.0");
		cl.def( pybind11::init<class mrpt::gui::CDisplayWindow3D &, class std::shared_ptr<class mrpt::opengl::Scene> &>(), pybind11::arg("win"), pybind11::arg("out_scene_ptr") );

		cl.def( pybind11::init<class mrpt::gui::CDisplayWindow3D &>(), pybind11::arg("win") );

	}
	{ // mrpt::gui::CGlCanvasBase file:mrpt/gui/CGlCanvasBase.h line:24
		pybind11::class_<mrpt::gui::CGlCanvasBase, std::shared_ptr<mrpt::gui::CGlCanvasBase>, PyCallBack_mrpt_gui_CGlCanvasBase> cl(M("mrpt::gui"), "CGlCanvasBase", "This base class implements a working with opengl::Camera and a OpenGL\n canvas, and it's used in gui::CWxGLCanvasBase and gui::CQtGlCanvasBase.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_gui_CGlCanvasBase(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_gui_CGlCanvasBase const &>());
		cl.def("setMinimumZoom", (void (mrpt::gui::CGlCanvasBase::*)(float)) &mrpt::gui::CGlCanvasBase::setMinimumZoom, "Sets the minimum of the zoom\n See also setMaximumZoom(float) \n\nC++: mrpt::gui::CGlCanvasBase::setMinimumZoom(float) --> void", pybind11::arg("zoom"));
		cl.def("setMaximumZoom", (void (mrpt::gui::CGlCanvasBase::*)(float)) &mrpt::gui::CGlCanvasBase::setMaximumZoom, "Sets the maximum of the zoom\n See also setMinimumZoom(float) \n\nC++: mrpt::gui::CGlCanvasBase::setMaximumZoom(float) --> void", pybind11::arg("zoom"));
		cl.def("setMousePos", (void (mrpt::gui::CGlCanvasBase::*)(int, int)) &mrpt::gui::CGlCanvasBase::setMousePos, "Saves the click position of the mouse\n See also setMouseClicked(bool) \n\nC++: mrpt::gui::CGlCanvasBase::setMousePos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setMouseClicked", (void (mrpt::gui::CGlCanvasBase::*)(bool)) &mrpt::gui::CGlCanvasBase::setMouseClicked, "Sets the property mouseClicked\n By default, this property is false.\n See also setMousePos(int, int) \n\nC++: mrpt::gui::CGlCanvasBase::setMouseClicked(bool) --> void", pybind11::arg("is"));
		cl.def("updateLastPos", (void (mrpt::gui::CGlCanvasBase::*)(int, int)) &mrpt::gui::CGlCanvasBase::updateLastPos, "Sets the last mouse position \n\nC++: mrpt::gui::CGlCanvasBase::updateLastPos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("resizeViewport", (void (mrpt::gui::CGlCanvasBase::*)(int, int)) &mrpt::gui::CGlCanvasBase::resizeViewport, "Calls the glViewport function\n\nC++: mrpt::gui::CGlCanvasBase::resizeViewport(int, int) --> void", pybind11::arg("w"), pybind11::arg("h"));
		cl.def("updateZoom", (void (mrpt::gui::CGlCanvasBase::*)(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const) &mrpt::gui::CGlCanvasBase::updateZoom, "This function for the mouse event\n It gets a reference to CamaraParams, x, y\n and updates the zoom of the CameraParams.\n See also updateZoom(CamaraParams &, float)\n\nC++: mrpt::gui::CGlCanvasBase::updateZoom(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const --> void", pybind11::arg("params"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("updateZoom", (void (mrpt::gui::CGlCanvasBase::*)(struct mrpt::gui::CGlCanvasBase::CamaraParams &, float) const) &mrpt::gui::CGlCanvasBase::updateZoom, "This function for the wheel event\n It gets a reference to CamaraParams, delta\n and updates the zoom of the CameraParams.\n See also updateZoom(CamaraParams &, int, int)\n\nC++: mrpt::gui::CGlCanvasBase::updateZoom(struct mrpt::gui::CGlCanvasBase::CamaraParams &, float) const --> void", pybind11::arg("params"), pybind11::arg("delta"));
		cl.def("updateRotate", (void (mrpt::gui::CGlCanvasBase::*)(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const) &mrpt::gui::CGlCanvasBase::updateRotate, "This function for the mouse event\n It gets a reference to CamaraParams, x, y\n and updates the elevation and azimuth.\n See also getElevationDegrees(), getAzimuthDegrees()\n\nC++: mrpt::gui::CGlCanvasBase::updateRotate(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const --> void", pybind11::arg("params"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("updateOrbitCamera", (void (mrpt::gui::CGlCanvasBase::*)(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const) &mrpt::gui::CGlCanvasBase::updateOrbitCamera, "This function for the mouse event\n It gets a reference to CamaraParams, x, y\n and updates the elevation and azimuth.\n See also getElevationDegrees(), getAzimuthDegrees()\n\nC++: mrpt::gui::CGlCanvasBase::updateOrbitCamera(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const --> void", pybind11::arg("params"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("updatePan", (void (mrpt::gui::CGlCanvasBase::*)(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const) &mrpt::gui::CGlCanvasBase::updatePan, "This function for the mouse event\n It gets a reference to CamaraParams, x, y\n and updates the pointing of the camera.\n See also getCameraPointingX(), getCameraPointingY(),\n getCameraPointingZ()\n\nC++: mrpt::gui::CGlCanvasBase::updatePan(struct mrpt::gui::CGlCanvasBase::CamaraParams &, int, int) const --> void", pybind11::arg("params"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cameraParams", (struct mrpt::gui::CGlCanvasBase::CamaraParams (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::cameraParams, "Returns a copy of CamaraParams\n See also getRefCameraParams(), setCameraParams(const CamaraParams &)\n\nC++: mrpt::gui::CGlCanvasBase::cameraParams() const --> struct mrpt::gui::CGlCanvasBase::CamaraParams");
		cl.def("getRefCameraParams", (const struct mrpt::gui::CGlCanvasBase::CamaraParams & (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getRefCameraParams, "Returns a reference to CamaraParams\n See also cameraParams(), setCameraParams(const CamaraParams &) \n\nC++: mrpt::gui::CGlCanvasBase::getRefCameraParams() const --> const struct mrpt::gui::CGlCanvasBase::CamaraParams &", pybind11::return_value_policy::automatic);
		cl.def("setCameraParams", (void (mrpt::gui::CGlCanvasBase::*)(const struct mrpt::gui::CGlCanvasBase::CamaraParams &)) &mrpt::gui::CGlCanvasBase::setCameraParams, "Sets the CamaraParams\n See also cameraParams(), getRefCameraParams()\n\nC++: mrpt::gui::CGlCanvasBase::setCameraParams(const struct mrpt::gui::CGlCanvasBase::CamaraParams &) --> void", pybind11::arg("params"));
		cl.def("updateCameraParams", (class mrpt::opengl::CCamera & (mrpt::gui::CGlCanvasBase::*)(class mrpt::opengl::CCamera &) const) &mrpt::gui::CGlCanvasBase::updateCameraParams, "This function gets a reference to mrpt::opengl::CCamera and\n updates the camera parameters(pointing, zoom, azimuth, elevation,\n IsProjective, FOV)\n\nC++: mrpt::gui::CGlCanvasBase::updateCameraParams(class mrpt::opengl::CCamera &) const --> class mrpt::opengl::CCamera &", pybind11::return_value_policy::automatic, pybind11::arg("cam"));
		cl.def("setUseCameraFromScene", (void (mrpt::gui::CGlCanvasBase::*)(bool)) &mrpt::gui::CGlCanvasBase::setUseCameraFromScene, "If set to true (default=false), the cameraPointingX,... parameters are\n ignored and the camera stored in the 3D scene is used instead.\n See also void bool getUseCameraFromScene()\n\nC++: mrpt::gui::CGlCanvasBase::setUseCameraFromScene(bool) --> void", pybind11::arg("is"));
		cl.def("getUseCameraFromScene", (bool (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getUseCameraFromScene, "See also void setUseCameraFromScene(bool)\n\nC++: mrpt::gui::CGlCanvasBase::getUseCameraFromScene() const --> bool");
		cl.def("setCameraPointing", (void (mrpt::gui::CGlCanvasBase::*)(float, float, float)) &mrpt::gui::CGlCanvasBase::setCameraPointing, "Saves the pointing of the camera\n See also getCameraPointingX(), getCameraPointingY(), getCameraPointingZ()\n\nC++: mrpt::gui::CGlCanvasBase::setCameraPointing(float, float, float) --> void", pybind11::arg("pointX"), pybind11::arg("pointY"), pybind11::arg("pointZ"));
		cl.def("getCameraPointingX", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getCameraPointingX, "Returns the x pointing of the camera\n See also setCameraPointing(float, float, float)\n\nC++: mrpt::gui::CGlCanvasBase::getCameraPointingX() const --> float");
		cl.def("getCameraPointingY", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getCameraPointingY, "Returns the y pointing of the camera\n See also setCameraPointing(float, float, float)\n\nC++: mrpt::gui::CGlCanvasBase::getCameraPointingY() const --> float");
		cl.def("getCameraPointingZ", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getCameraPointingZ, "Returns the z pointing of the camera\n See also setCameraPointing(float, float, float)\n\nC++: mrpt::gui::CGlCanvasBase::getCameraPointingZ() const --> float");
		cl.def("setZoomDistance", (void (mrpt::gui::CGlCanvasBase::*)(float)) &mrpt::gui::CGlCanvasBase::setZoomDistance, "Saves camera zooming\n See also getZoomDistance()\n\nC++: mrpt::gui::CGlCanvasBase::setZoomDistance(float) --> void", pybind11::arg("zoom"));
		cl.def("getZoomDistance", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getZoomDistance, "Returns a zoom\n See also setZoomDistance(float)\n\nC++: mrpt::gui::CGlCanvasBase::getZoomDistance() const --> float");
		cl.def("setAzimuthDegrees", (void (mrpt::gui::CGlCanvasBase::*)(float)) &mrpt::gui::CGlCanvasBase::setAzimuthDegrees, "Saves the degrees of the azimuth camera\n See also getAzimuthDegrees()\n\nC++: mrpt::gui::CGlCanvasBase::setAzimuthDegrees(float) --> void", pybind11::arg("ang"));
		cl.def("getAzimuthDegrees", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getAzimuthDegrees, "Returns a azimuth degrees\n See also setAzimuthDegrees(float)\n\nC++: mrpt::gui::CGlCanvasBase::getAzimuthDegrees() const --> float");
		cl.def("setElevationDegrees", (void (mrpt::gui::CGlCanvasBase::*)(float)) &mrpt::gui::CGlCanvasBase::setElevationDegrees, "Saves the degrees of the elevation camera\n See also getElevationDegrees()\n\nC++: mrpt::gui::CGlCanvasBase::setElevationDegrees(float) --> void", pybind11::arg("ang"));
		cl.def("getElevationDegrees", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::getElevationDegrees, "Returns a elevation degrees\n See also setElevationDegrees(float)\n\nC++: mrpt::gui::CGlCanvasBase::getElevationDegrees() const --> float");
		cl.def("setCameraProjective", (void (mrpt::gui::CGlCanvasBase::*)(bool)) &mrpt::gui::CGlCanvasBase::setCameraProjective, "C++: mrpt::gui::CGlCanvasBase::setCameraProjective(bool) --> void", pybind11::arg("is"));
		cl.def("isCameraProjective", (bool (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::isCameraProjective, "C++: mrpt::gui::CGlCanvasBase::isCameraProjective() const --> bool");
		cl.def("setCameraFOV", (void (mrpt::gui::CGlCanvasBase::*)(float)) &mrpt::gui::CGlCanvasBase::setCameraFOV, "C++: mrpt::gui::CGlCanvasBase::setCameraFOV(float) --> void", pybind11::arg("FOV"));
		cl.def("cameraFOV", (float (mrpt::gui::CGlCanvasBase::*)() const) &mrpt::gui::CGlCanvasBase::cameraFOV, "C++: mrpt::gui::CGlCanvasBase::cameraFOV() const --> float");
		cl.def("OnUserManuallyMovesCamera", (void (mrpt::gui::CGlCanvasBase::*)(float, float, float, float, float, float)) &mrpt::gui::CGlCanvasBase::OnUserManuallyMovesCamera, "Overload this method to limit the capabilities of the user to move the\n camera using the mouse.\n  For all these variables:\n  - cameraPointingX\n  - cameraPointingY\n  - cameraPointingZ\n  - cameraZoomDistance\n  - cameraElevationDeg\n  - cameraAzimuthDeg\n\n  A \"new_NAME\" variable will be passed with the temptative new\n value after the user action.\n   The default behavior should be to copy all the new variables\n to the variables listed above\n   but in the middle any find of user-defined filter can be\n implemented.\n\nC++: mrpt::gui::CGlCanvasBase::OnUserManuallyMovesCamera(float, float, float, float, float, float) --> void", pybind11::arg("new_cameraPointingX"), pybind11::arg("new_cameraPointingY"), pybind11::arg("new_cameraPointingZ"), pybind11::arg("new_cameraZoomDistance"), pybind11::arg("new_cameraElevationDeg"), pybind11::arg("new_cameraAzimuthDeg"));
		cl.def("getLastMousePosition", (void (mrpt::gui::CGlCanvasBase::*)(int &, int &) const) &mrpt::gui::CGlCanvasBase::getLastMousePosition, "C++: mrpt::gui::CGlCanvasBase::getLastMousePosition(int &, int &) const --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("getOpenGLSceneRef", (class std::shared_ptr<class mrpt::opengl::Scene> & (mrpt::gui::CGlCanvasBase::*)()) &mrpt::gui::CGlCanvasBase::getOpenGLSceneRef, "At constructor an empty scene is created. The object is freed at GL\n  canvas destructor.\n This function returns a smart pointer to the opengl scene\n  getOpenGLSceneRef		  \n\nC++: mrpt::gui::CGlCanvasBase::getOpenGLSceneRef() --> class std::shared_ptr<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic);
		cl.def("setOpenGLSceneRef", (void (mrpt::gui::CGlCanvasBase::*)(class std::shared_ptr<class mrpt::opengl::Scene>)) &mrpt::gui::CGlCanvasBase::setOpenGLSceneRef, "C++: mrpt::gui::CGlCanvasBase::setOpenGLSceneRef(class std::shared_ptr<class mrpt::opengl::Scene>) --> void", pybind11::arg("scene"));
		cl.def("assign", (class mrpt::gui::CGlCanvasBase & (mrpt::gui::CGlCanvasBase::*)(const class mrpt::gui::CGlCanvasBase &)) &mrpt::gui::CGlCanvasBase::operator=, "C++: mrpt::gui::CGlCanvasBase::operator=(const class mrpt::gui::CGlCanvasBase &) --> class mrpt::gui::CGlCanvasBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::gui::CGlCanvasBase::CamaraParams file:mrpt/gui/CGlCanvasBase.h line:27
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::gui::CGlCanvasBase::CamaraParams, std::shared_ptr<mrpt::gui::CGlCanvasBase::CamaraParams>> cl(enclosing_class, "CamaraParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::gui::CGlCanvasBase::CamaraParams(); } ) );
			cl.def( pybind11::init( [](mrpt::gui::CGlCanvasBase::CamaraParams const &o){ return new mrpt::gui::CGlCanvasBase::CamaraParams(o); } ) );
			cl.def_readwrite("cameraPointingX", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraPointingX);
			cl.def_readwrite("cameraPointingY", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraPointingY);
			cl.def_readwrite("cameraPointingZ", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraPointingZ);
			cl.def_readwrite("cameraZoomDistance", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraZoomDistance);
			cl.def_readwrite("cameraElevationDeg", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraElevationDeg);
			cl.def_readwrite("cameraAzimuthDeg", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraAzimuthDeg);
			cl.def_readwrite("cameraIsProjective", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraIsProjective);
			cl.def_readwrite("cameraFOV", &mrpt::gui::CGlCanvasBase::CamaraParams::cameraFOV);
			cl.def("setElevationDeg", (void (mrpt::gui::CGlCanvasBase::CamaraParams::*)(float)) &mrpt::gui::CGlCanvasBase::CamaraParams::setElevationDeg, "Changes elevation, taking care of not going out of the [-90,90]\n range \n\nC++: mrpt::gui::CGlCanvasBase::CamaraParams::setElevationDeg(float) --> void", pybind11::arg("deg"));
			cl.def_static("FromCamera", (struct mrpt::gui::CGlCanvasBase::CamaraParams (*)(const class mrpt::opengl::CCamera &)) &mrpt::gui::CGlCanvasBase::CamaraParams::FromCamera, "Converts from a CCamera objects \n [New in MRPT 2.1.5] \n\nC++: mrpt::gui::CGlCanvasBase::CamaraParams::FromCamera(const class mrpt::opengl::CCamera &) --> struct mrpt::gui::CGlCanvasBase::CamaraParams", pybind11::arg("c"));
			cl.def("assign", (struct mrpt::gui::CGlCanvasBase::CamaraParams & (mrpt::gui::CGlCanvasBase::CamaraParams::*)(const struct mrpt::gui::CGlCanvasBase::CamaraParams &)) &mrpt::gui::CGlCanvasBase::CamaraParams::operator=, "C++: mrpt::gui::CGlCanvasBase::CamaraParams::operator=(const struct mrpt::gui::CGlCanvasBase::CamaraParams &) --> struct mrpt::gui::CGlCanvasBase::CamaraParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
