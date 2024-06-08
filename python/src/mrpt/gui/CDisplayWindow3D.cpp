#include <functional>
#include <iterator>
#include <memory>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Viewport.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <typeinfo>
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

// mrpt::gui::CDisplayWindow3D file:mrpt/gui/CDisplayWindow3D.h line:111
struct PyCallBack_mrpt_gui_CDisplayWindow3D : public mrpt::gui::CDisplayWindow3D {
	using mrpt::gui::CDisplayWindow3D::CDisplayWindow3D;

	void resize(unsigned int a0, unsigned int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow3D *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow3D::resize(a0, a1);
	}
	void setPos(int a0, int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow3D *>(this), "setPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow3D::setPos(a0, a1);
	}
	void setWindowTitle(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow3D *>(this), "setWindowTitle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow3D::setWindowTitle(a0);
	}
	bool getLastMousePosition(int & a0, int & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow3D *>(this), "getLastMousePosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CDisplayWindow3D::getLastMousePosition(a0, a1);
	}
	void setCursorCross(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow3D *>(this), "setCursorCross");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow3D::setCursorCross(a0);
	}
};

void bind_mrpt_gui_CDisplayWindow3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::gui::CDisplayWindow3D file:mrpt/gui/CDisplayWindow3D.h line:111
		pybind11::class_<mrpt::gui::CDisplayWindow3D, std::shared_ptr<mrpt::gui::CDisplayWindow3D>, PyCallBack_mrpt_gui_CDisplayWindow3D, mrpt::gui::CBaseGUIWindow> cl(M("mrpt::gui"), "CDisplayWindow3D", "A graphical user interface (GUI) for efficiently rendering 3D scenes in\n real-time.\n  This class always contains internally an instance of opengl::Scene,\n which\n   the objects, viewports, etc. to be rendered.\n\n  Images can be grabbed automatically to disk for easy creation of videos.\n  See CDisplayWindow3D::grabImagesStart  (and for creating videos,\n mrpt::vision::CVideoFileWriter).\n\n  A short-cut for displaying 2D images (using the OpenGL rendering hardware)\n is available\n  through  . Internally, these methods call methods\n  in the \"main\" viewport of the window (see \n\n  Since the 3D rendering is performed in a detached thread, especial care\n must be taken\n   when updating the 3D scene to be rendered. The process involves an\n internal critical section\n   and it must always consist of these steps:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n An alternative way of updating the scene is by creating, before locking the\n 3D window, a new object\n  of class Scene, then locking the window only for replacing the smart\n pointer. This may be\n  advantageous is generating the 3D scene takes a long time, since while the\n window\n  is locked it will not be responsive to the user input or window redraw.\n\n It is safer against exceptions to use the auxiliary class\n CDisplayWindow3DLocker.\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n Notice however that a copy of the smart pointer is made, so replacement of\n the entire scene via `operator =` is not possible if using this method.\n Instead, the content of the scene should be assigned using the `operator =`\n of the **dereferenced** object as illustrated with\n the `*ptrScene = *otherScene;` above.\n\n The window can also display a set of 2D text messages overlapped to the 3D\n scene. See CDisplayWindow3D::addTextMessage\n\n  For a list of supported events with the observer/observable pattern, see\n the discussion in mrpt::gui::CBaseGUIWindow.\n  In addition to those events, this class introduces\n mrpt::gui::mrptEvent3DWindowGrabImageFile\n\n [CDisplayWindow3D mouse view navigation\n cheatsheet](tutorial-3d-navigation-cheatsheet.html)\n\n ![mrpt::gui::CDisplayWindow3D screenshot](preview_CDisplayWindow3D.png)\n\n \n \n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::gui::CDisplayWindow3D(); }, [](){ return new PyCallBack_mrpt_gui_CDisplayWindow3D(); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::gui::CDisplayWindow3D(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_gui_CDisplayWindow3D(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, unsigned int const & a1){ return new mrpt::gui::CDisplayWindow3D(a0, a1); }, [](const std::string & a0, unsigned int const & a1){ return new PyCallBack_mrpt_gui_CDisplayWindow3D(a0, a1); } ), "doc");
		cl.def( pybind11::init<const std::string &, unsigned int, unsigned int>(), pybind11::arg("windowCaption"), pybind11::arg("initialWindowWidth"), pybind11::arg("initialWindowHeight") );

		cl.def_static("Create", [](const std::string & a0) -> std::shared_ptr<class mrpt::gui::CDisplayWindow3D> { return mrpt::gui::CDisplayWindow3D::Create(a0); }, "", pybind11::arg("windowCaption"));
		cl.def_static("Create", [](const std::string & a0, unsigned int const & a1) -> std::shared_ptr<class mrpt::gui::CDisplayWindow3D> { return mrpt::gui::CDisplayWindow3D::Create(a0, a1); }, "", pybind11::arg("windowCaption"), pybind11::arg("initialWindowWidth"));
		cl.def_static("Create", (class std::shared_ptr<class mrpt::gui::CDisplayWindow3D> (*)(const std::string &, unsigned int, unsigned int)) &mrpt::gui::CDisplayWindow3D::Create, "Class factory returning a smart pointer, equivalent to\n `std::make_shared<>(...)` \n\nC++: mrpt::gui::CDisplayWindow3D::Create(const std::string &, unsigned int, unsigned int) --> class std::shared_ptr<class mrpt::gui::CDisplayWindow3D>", pybind11::arg("windowCaption"), pybind11::arg("initialWindowWidth"), pybind11::arg("initialWindowHeight"));
		cl.def("get3DSceneAndLock", (class std::shared_ptr<class mrpt::opengl::Scene> & (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::get3DSceneAndLock, "Gets a reference to the smart shared pointer that holds the internal\n scene (carefuly read introduction in gui::CDisplayWindow3D before use!)\n  This also locks the critical section for accessing the scene, thus the\n window will not be repainted until it is unlocked.\n \n\n It is safer to use mrpt::gui::CDisplayWindow3DLocker instead.\n\nC++: mrpt::gui::CDisplayWindow3D::get3DSceneAndLock() --> class std::shared_ptr<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic);
		cl.def("unlockAccess3DScene", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::unlockAccess3DScene, "Unlocks the access to the internal 3D scene. It is safer to use\n mrpt::gui::CDisplayWindow3DLocker instead.\n  Typically user will want to call forceRepaint after updating the scene.\n\nC++: mrpt::gui::CDisplayWindow3D::unlockAccess3DScene() --> void");
		cl.def("forceRepaint", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::forceRepaint, "Repaints the window. forceRepaint, repaint and updateWindow are all\n aliases of the same method \n\nC++: mrpt::gui::CDisplayWindow3D::forceRepaint() --> void");
		cl.def("repaint", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::repaint, "Repaints the window. forceRepaint, repaint and updateWindow are all\n aliases of the same method \n\nC++: mrpt::gui::CDisplayWindow3D::repaint() --> void");
		cl.def("updateWindow", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::updateWindow, "Repaints the window. forceRepaint, repaint and updateWindow are all\n aliases of the same method \n\nC++: mrpt::gui::CDisplayWindow3D::updateWindow() --> void");
		cl.def("getFOV", (float (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::getFOV, "Return the camera field of view (in degrees) (used for gluPerspective)\n\nC++: mrpt::gui::CDisplayWindow3D::getFOV() const --> float");
		cl.def("setMinRange", (void (mrpt::gui::CDisplayWindow3D::*)(float)) &mrpt::gui::CDisplayWindow3D::setMinRange, "Changes the camera min clip range (z) (used for gluPerspective). The\n window is not updated with this method, call \"forceRepaint\" to update the\n 3D view. \n\nC++: mrpt::gui::CDisplayWindow3D::setMinRange(float) --> void", pybind11::arg("new_min"));
		cl.def("setMaxRange", (void (mrpt::gui::CDisplayWindow3D::*)(float)) &mrpt::gui::CDisplayWindow3D::setMaxRange, "Changes the camera max clip range (z) (used for gluPerspective. The\n window is not updated with this method, call \"forceRepaint\" to update the\n 3D view. \n\nC++: mrpt::gui::CDisplayWindow3D::setMaxRange(float) --> void", pybind11::arg("new_max"));
		cl.def("setFOV", (void (mrpt::gui::CDisplayWindow3D::*)(float)) &mrpt::gui::CDisplayWindow3D::setFOV, "Changes the camera field of view (in degrees) (used for gluPerspective).\n The window is not updated with this method, call \"forceRepaint\" to update\n the 3D view. \n\nC++: mrpt::gui::CDisplayWindow3D::setFOV(float) --> void", pybind11::arg("v"));
		cl.def("resize", (void (mrpt::gui::CDisplayWindow3D::*)(unsigned int, unsigned int)) &mrpt::gui::CDisplayWindow3D::resize, "Resizes the window, stretching the image to fit into the display area.\n\nC++: mrpt::gui::CDisplayWindow3D::resize(unsigned int, unsigned int) --> void", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("setPos", (void (mrpt::gui::CDisplayWindow3D::*)(int, int)) &mrpt::gui::CDisplayWindow3D::setPos, "Changes the position of the window on the screen. \n\nC++: mrpt::gui::CDisplayWindow3D::setPos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setWindowTitle", (void (mrpt::gui::CDisplayWindow3D::*)(const std::string &)) &mrpt::gui::CDisplayWindow3D::setWindowTitle, "Changes the window title. \n\nC++: mrpt::gui::CDisplayWindow3D::setWindowTitle(const std::string &) --> void", pybind11::arg("str"));
		cl.def("setCameraElevationDeg", (void (mrpt::gui::CDisplayWindow3D::*)(float)) &mrpt::gui::CDisplayWindow3D::setCameraElevationDeg, "Changes the camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::setCameraElevationDeg(float) --> void", pybind11::arg("deg"));
		cl.def("setCameraAzimuthDeg", (void (mrpt::gui::CDisplayWindow3D::*)(float)) &mrpt::gui::CDisplayWindow3D::setCameraAzimuthDeg, "Changes the camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::setCameraAzimuthDeg(float) --> void", pybind11::arg("deg"));
		cl.def("setCameraPointingToPoint", (void (mrpt::gui::CDisplayWindow3D::*)(float, float, float)) &mrpt::gui::CDisplayWindow3D::setCameraPointingToPoint, "Changes the camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::setCameraPointingToPoint(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setCameraZoom", (void (mrpt::gui::CDisplayWindow3D::*)(float)) &mrpt::gui::CDisplayWindow3D::setCameraZoom, "Changes the camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::setCameraZoom(float) --> void", pybind11::arg("zoom"));
		cl.def("setCameraProjective", (void (mrpt::gui::CDisplayWindow3D::*)(bool)) &mrpt::gui::CDisplayWindow3D::setCameraProjective, "Sets the camera as projective, or orthogonal. \n\nC++: mrpt::gui::CDisplayWindow3D::setCameraProjective(bool) --> void", pybind11::arg("isProjective"));
		cl.def("getCameraElevationDeg", (float (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::getCameraElevationDeg, "Get camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::getCameraElevationDeg() const --> float");
		cl.def("getCameraAzimuthDeg", (float (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::getCameraAzimuthDeg, "Get camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::getCameraAzimuthDeg() const --> float");
		cl.def("getCameraPointingToPoint", (void (mrpt::gui::CDisplayWindow3D::*)(float &, float &, float &) const) &mrpt::gui::CDisplayWindow3D::getCameraPointingToPoint, "Get camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::getCameraPointingToPoint(float &, float &, float &) const --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getCameraZoom", (float (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::getCameraZoom, "Get camera parameters programmatically \n\nC++: mrpt::gui::CDisplayWindow3D::getCameraZoom() const --> float");
		cl.def("isCameraProjective", (bool (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::isCameraProjective, "Sets the camera as projective, or orthogonal \n\nC++: mrpt::gui::CDisplayWindow3D::isCameraProjective() const --> bool");
		cl.def("useCameraFromScene", [](mrpt::gui::CDisplayWindow3D &o) -> void { return o.useCameraFromScene(); }, "");
		cl.def("useCameraFromScene", (void (mrpt::gui::CDisplayWindow3D::*)(bool)) &mrpt::gui::CDisplayWindow3D::useCameraFromScene, "If set to true (default = false), the mouse-based scene navigation will\n be disabled and the camera position will be determined by the opengl\n viewports in the 3D scene \n\nC++: mrpt::gui::CDisplayWindow3D::useCameraFromScene(bool) --> void", pybind11::arg("useIt"));
		cl.def("getLastMousePositionRay", (bool (mrpt::gui::CDisplayWindow3D::*)(struct mrpt::math::TLine3D &) const) &mrpt::gui::CDisplayWindow3D::getLastMousePositionRay, "Gets the 3D ray for the direction line of the pixel where the mouse\n cursor is at. \n\n False if the window is closed. \n\n getLastMousePosition \n\nC++: mrpt::gui::CDisplayWindow3D::getLastMousePositionRay(struct mrpt::math::TLine3D &) const --> bool", pybind11::arg("ray"));
		cl.def("getLastMousePosition", (bool (mrpt::gui::CDisplayWindow3D::*)(int &, int &) const) &mrpt::gui::CDisplayWindow3D::getLastMousePosition, "Gets the last x,y pixel coordinates of the mouse. \n False if the\n window is closed. \n\n getLastMousePositionRay \n\nC++: mrpt::gui::CDisplayWindow3D::getLastMousePosition(int &, int &) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setCursorCross", (void (mrpt::gui::CDisplayWindow3D::*)(bool)) &mrpt::gui::CDisplayWindow3D::setCursorCross, "Set cursor style to default (cursorIsCross=false) or to a cross\n (cursorIsCross=true) \n\n getLastMousePositionRay \n\nC++: mrpt::gui::CDisplayWindow3D::setCursorCross(bool) --> void", pybind11::arg("cursorIsCross"));
		cl.def("grabImagesStart", [](mrpt::gui::CDisplayWindow3D &o) -> void { return o.grabImagesStart(); }, "");
		cl.def("grabImagesStart", (void (mrpt::gui::CDisplayWindow3D::*)(const std::string &)) &mrpt::gui::CDisplayWindow3D::grabImagesStart, "Start to save rendered images to disk.\n  Images will be saved independently as png files, depending on\n  the template path passed to this method. For example, the\n  path_prefix ``./video_`` will generate ``./video_000001.png``, etc.\n\n  If this feature is enabled, the window will emit events of the type\n  mrpt::gui::mrptEvent3DWindowGrabImageFile() which you can subscribe to.\n\n  \n grabImagesStop\n\nC++: mrpt::gui::CDisplayWindow3D::grabImagesStart(const std::string &) --> void", pybind11::arg("grab_imgs_prefix"));
		cl.def("grabImagesStop", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::grabImagesStop, "Stops image grabbing started by grabImagesStart\n \n\n grabImagesStart\n\nC++: mrpt::gui::CDisplayWindow3D::grabImagesStop() --> void");
		cl.def("captureImagesStart", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::captureImagesStart, "Enables the grabbing of CImage objects from screenshots of the window.\n  \n\n getLastWindowImage\n\nC++: mrpt::gui::CDisplayWindow3D::captureImagesStart() --> void");
		cl.def("captureImagesStop", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::captureImagesStop, "Stop image grabbing\n \n\n captureImagesStart\n\nC++: mrpt::gui::CDisplayWindow3D::captureImagesStop() --> void");
		cl.def("getLastWindowImage", (bool (mrpt::gui::CDisplayWindow3D::*)(class mrpt::img::CImage &) const) &mrpt::gui::CDisplayWindow3D::getLastWindowImage, "Retrieve the last captured image from the window.\n  You MUST CALL FIRST captureImagesStart to enable image grabbing.\n \n\n false if there was no time yet for grabbing any image (then, the\n output image is undefined).\n \n\n captureImagesStart, getLastWindowImagePtr\n\nC++: mrpt::gui::CDisplayWindow3D::getLastWindowImage(class mrpt::img::CImage &) const --> bool", pybind11::arg("out_img"));
		cl.def("getLastWindowImagePtr", (class std::shared_ptr<class mrpt::img::CImage> (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::getLastWindowImagePtr, "Retrieve the last captured image from the window, as a smart pointer.\n  This method is more efficient than getLastWindowImage since only a copy\n of the pointer is performed, while\n   getLastWindowImage would copy the entire image.\n\n  You MUST CALL FIRST captureImagesStart to enable image grabbing.\n \n\n If there was no time yet for grabbing any image, an empty smart\n pointer will be returned.\n \n\n captureImagesStart, getLastWindowImage\n\nC++: mrpt::gui::CDisplayWindow3D::getLastWindowImagePtr() const --> class std::shared_ptr<class mrpt::img::CImage>");
		cl.def("grabImageGetNextFile", (std::string (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::grabImageGetNextFile, "Increments by one the image counter and return the next image file name\n (Users normally don't want to call this method).\n \n\n grabImagesStart\n\nC++: mrpt::gui::CDisplayWindow3D::grabImageGetNextFile() --> std::string");
		cl.def("isCapturingImgs", (bool (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::isCapturingImgs, "C++: mrpt::gui::CDisplayWindow3D::isCapturingImgs() const --> bool");
		cl.def("addTextMessage", [](mrpt::gui::CDisplayWindow3D &o, const double & a0, const double & a1, const std::string & a2) -> void { return o.addTextMessage(a0, a1, a2); }, "", pybind11::arg("x_frac"), pybind11::arg("y_frac"), pybind11::arg("text"));
		cl.def("addTextMessage", [](mrpt::gui::CDisplayWindow3D &o, const double & a0, const double & a1, const std::string & a2, size_t const & a3) -> void { return o.addTextMessage(a0, a1, a2, a3); }, "", pybind11::arg("x_frac"), pybind11::arg("y_frac"), pybind11::arg("text"), pybind11::arg("unique_index"));
		cl.def("addTextMessage", (void (mrpt::gui::CDisplayWindow3D::*)(const double, const double, const std::string &, size_t, const struct mrpt::opengl::TFontParams &)) &mrpt::gui::CDisplayWindow3D::addTextMessage, "A shortcut for calling mrpt::opengl::Viewport::addTextMessage()\n in the \"main\" viewport of the 3D scene.\n \n\n clearTextMessages, mrpt::opengl::Viewport::addTextMessage()\n\nC++: mrpt::gui::CDisplayWindow3D::addTextMessage(const double, const double, const std::string &, size_t, const struct mrpt::opengl::TFontParams &) --> void", pybind11::arg("x_frac"), pybind11::arg("y_frac"), pybind11::arg("text"), pybind11::arg("unique_index"), pybind11::arg("fontParams"));
		cl.def("clearTextMessages", (void (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::clearTextMessages, "Clear all text messages created with addTextMessage(). A shortcut for\n calling mrpt::opengl::Viewport::clearTextMessages().\n\n \n addTextMessage\n\nC++: mrpt::gui::CDisplayWindow3D::clearTextMessages() --> void");
		cl.def("updateTextMessage", (bool (mrpt::gui::CDisplayWindow3D::*)(const unsigned long, const std::string &)) &mrpt::gui::CDisplayWindow3D::updateTextMessage, "Just updates the text of a given text message, without touching the\n other parameters. A shortcut for\n calling mrpt::opengl::Viewport::updateTextMessage()\n\n \n false if given ID doesn't exist.\n\nC++: mrpt::gui::CDisplayWindow3D::updateTextMessage(const unsigned long, const std::string &) --> bool", pybind11::arg("unique_index"), pybind11::arg("text"));
		cl.def("getRenderingFPS", (double (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::getRenderingFPS, "Get the average Frames Per Second (FPS) value from the last 250\n rendering events \n\nC++: mrpt::gui::CDisplayWindow3D::getRenderingFPS() const --> double");
		cl.def("getDefaultViewport", (class std::shared_ptr<class mrpt::opengl::Viewport> (mrpt::gui::CDisplayWindow3D::*)()) &mrpt::gui::CDisplayWindow3D::getDefaultViewport, "A short cut for getting the \"main\" viewport of the scene object, it is\n equivalent to:\n  \n\n\n\n\n\n   \n\nC++: mrpt::gui::CDisplayWindow3D::getDefaultViewport() --> class std::shared_ptr<class mrpt::opengl::Viewport>");
		cl.def("setImageView", (void (mrpt::gui::CDisplayWindow3D::*)(const class mrpt::img::CImage &)) &mrpt::gui::CDisplayWindow3D::setImageView, "Set the \"main\" viewport into \"image view\"-mode, where an image is\n efficiently drawn (fitting the viewport area) using an OpenGL textured\n quad.\n  Call this method with the new image to update the displayed image (but\n recall to first lock the parent openglscene's critical section, then do\n the update, then release the lock, and then issue a window repaint).\n  Internally, the texture is drawn using a mrpt::opengl::CTexturedPlane\n  The viewport can be reverted to behave like a normal viewport by\n calling setNormalMode()\n \n\n Viewport\n \n\n This method already locks/unlocks the 3D scene of the window, so\n the user must NOT call get3DSceneAndLock() / unlockAccess3DScene()\n before/after calling it.\n\nC++: mrpt::gui::CDisplayWindow3D::setImageView(const class mrpt::img::CImage &) --> void", pybind11::arg("img"));
		cl.def("sendFunctionToRunOnGUIThread", (void (mrpt::gui::CDisplayWindow3D::*)(const class std::function<void (void)> &)) &mrpt::gui::CDisplayWindow3D::sendFunctionToRunOnGUIThread, "C++: mrpt::gui::CDisplayWindow3D::sendFunctionToRunOnGUIThread(const class std::function<void (void)> &) --> void", pybind11::arg("f"));
		cl.def("is_GL_context_created", (bool (mrpt::gui::CDisplayWindow3D::*)() const) &mrpt::gui::CDisplayWindow3D::is_GL_context_created, "C++: mrpt::gui::CDisplayWindow3D::is_GL_context_created() const --> bool");
		cl.def("wait_for_GL_context", [](mrpt::gui::CDisplayWindow3D const &o) -> bool { return o.wait_for_GL_context(); }, "");
		cl.def("wait_for_GL_context", (bool (mrpt::gui::CDisplayWindow3D::*)(const double) const) &mrpt::gui::CDisplayWindow3D::wait_for_GL_context, "C++: mrpt::gui::CDisplayWindow3D::wait_for_GL_context(const double) const --> bool", pybind11::arg("timeout_seconds"));
	}
}
