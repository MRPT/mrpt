#include <iterator>
#include <memory>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
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

// mrpt::gui::CDisplayWindow file:mrpt/gui/CDisplayWindow.h line:34
struct PyCallBack_mrpt_gui_CDisplayWindow : public mrpt::gui::CDisplayWindow {
	using mrpt::gui::CDisplayWindow::CDisplayWindow;

	bool getLastMousePosition(int & a0, int & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow *>(this), "getLastMousePosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CDisplayWindow::getLastMousePosition(a0, a1);
	}
	void setCursorCross(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow *>(this), "setCursorCross");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow::setCursorCross(a0);
	}
	void resize(unsigned int a0, unsigned int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow::resize(a0, a1);
	}
	void setPos(int a0, int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow *>(this), "setPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow::setPos(a0, a1);
	}
	void setWindowTitle(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindow *>(this), "setWindowTitle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindow::setWindowTitle(a0);
	}
};

void bind_mrpt_gui_CDisplayWindow(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::gui::CDisplayWindow file:mrpt/gui/CDisplayWindow.h line:34
		pybind11::class_<mrpt::gui::CDisplayWindow, std::shared_ptr<mrpt::gui::CDisplayWindow>, PyCallBack_mrpt_gui_CDisplayWindow, mrpt::gui::CBaseGUIWindow> cl(M("mrpt::gui"), "CDisplayWindow", "This class creates a window as a graphical user interface (GUI) for\n displaying images to the user.\n\n  For a list of supported events with the observer/observable pattern, see the\n discussion in mrpt::gui::CBaseGUIWindow.\n\n ![mrpt::gui::CDisplayWindow screenshot](preview_CDisplayWindow.jpg)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::gui::CDisplayWindow(); }, [](){ return new PyCallBack_mrpt_gui_CDisplayWindow(); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::gui::CDisplayWindow(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_gui_CDisplayWindow(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, unsigned int const & a1){ return new mrpt::gui::CDisplayWindow(a0, a1); }, [](const std::string & a0, unsigned int const & a1){ return new PyCallBack_mrpt_gui_CDisplayWindow(a0, a1); } ), "doc");
		cl.def( pybind11::init<const std::string &, unsigned int, unsigned int>(), pybind11::arg("windowCaption"), pybind11::arg("initWidth"), pybind11::arg("initHeight") );

		cl.def_static("Create", [](const std::string & a0) -> std::shared_ptr<class mrpt::gui::CDisplayWindow> { return mrpt::gui::CDisplayWindow::Create(a0); }, "", pybind11::arg("windowCaption"));
		cl.def_static("Create", [](const std::string & a0, unsigned int const & a1) -> std::shared_ptr<class mrpt::gui::CDisplayWindow> { return mrpt::gui::CDisplayWindow::Create(a0, a1); }, "", pybind11::arg("windowCaption"), pybind11::arg("initWidth"));
		cl.def_static("Create", (class std::shared_ptr<class mrpt::gui::CDisplayWindow> (*)(const std::string &, unsigned int, unsigned int)) &mrpt::gui::CDisplayWindow::Create, "Class factory returning a smart pointer, equivalent to\n `std::make_shared<>(...)` \n\nC++: mrpt::gui::CDisplayWindow::Create(const std::string &, unsigned int, unsigned int) --> class std::shared_ptr<class mrpt::gui::CDisplayWindow>", pybind11::arg("windowCaption"), pybind11::arg("initWidth"), pybind11::arg("initHeight"));
		cl.def("getLastMousePosition", (bool (mrpt::gui::CDisplayWindow::*)(int &, int &) const) &mrpt::gui::CDisplayWindow::getLastMousePosition, "Gets the last x,y pixel coordinates of the mouse. \n False if the\n window is closed. \n\nC++: mrpt::gui::CDisplayWindow::getLastMousePosition(int &, int &) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setCursorCross", (void (mrpt::gui::CDisplayWindow::*)(bool)) &mrpt::gui::CDisplayWindow::setCursorCross, "Set cursor style to default (cursorIsCross=false) or to a cross\n (cursorIsCross=true) \n\nC++: mrpt::gui::CDisplayWindow::setCursorCross(bool) --> void", pybind11::arg("cursorIsCross"));
		cl.def("showImageAndPoints", [](mrpt::gui::CDisplayWindow &o, const class mrpt::img::CImage & a0, const class mrpt::math::CVectorDynamic<float> & a1, const class mrpt::math::CVectorDynamic<float> & a2) -> void { return o.showImageAndPoints(a0, a1, a2); }, "", pybind11::arg("img"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("showImageAndPoints", [](mrpt::gui::CDisplayWindow &o, const class mrpt::img::CImage & a0, const class mrpt::math::CVectorDynamic<float> & a1, const class mrpt::math::CVectorDynamic<float> & a2, const struct mrpt::img::TColor & a3) -> void { return o.showImageAndPoints(a0, a1, a2, a3); }, "", pybind11::arg("img"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("color"));
		cl.def("showImageAndPoints", (void (mrpt::gui::CDisplayWindow::*)(const class mrpt::img::CImage &, const class mrpt::math::CVectorDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &, const struct mrpt::img::TColor &, bool)) &mrpt::gui::CDisplayWindow::showImageAndPoints, "Show a given color or grayscale image on the window and print a set of\n points on it.\n  It adapts the size of the window to that of the image.\n\nC++: mrpt::gui::CDisplayWindow::showImageAndPoints(const class mrpt::img::CImage &, const class mrpt::math::CVectorDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &, const struct mrpt::img::TColor &, bool) --> void", pybind11::arg("img"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("color"), pybind11::arg("showNumbers"));
		cl.def("showImage", (void (mrpt::gui::CDisplayWindow::*)(const class mrpt::img::CImage &)) &mrpt::gui::CDisplayWindow::showImage, "Show a given color or grayscale image on the window.\n  It adapts the size of the window to that of the image.\n\nC++: mrpt::gui::CDisplayWindow::showImage(const class mrpt::img::CImage &) --> void", pybind11::arg("img"));
		cl.def("plot", (void (mrpt::gui::CDisplayWindow::*)(const class mrpt::math::CVectorDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &)) &mrpt::gui::CDisplayWindow::plot, "Plots a graph in MATLAB-like style.\n\nC++: mrpt::gui::CDisplayWindow::plot(const class mrpt::math::CVectorDynamic<float> &, const class mrpt::math::CVectorDynamic<float> &) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("plot", (void (mrpt::gui::CDisplayWindow::*)(const class mrpt::math::CVectorDynamic<float> &)) &mrpt::gui::CDisplayWindow::plot, "Plots a graph in MATLAB-like style.\n\nC++: mrpt::gui::CDisplayWindow::plot(const class mrpt::math::CVectorDynamic<float> &) --> void", pybind11::arg("y"));
		cl.def("resize", (void (mrpt::gui::CDisplayWindow::*)(unsigned int, unsigned int)) &mrpt::gui::CDisplayWindow::resize, "Resizes the window, stretching the image to fit into the display area.\n\nC++: mrpt::gui::CDisplayWindow::resize(unsigned int, unsigned int) --> void", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("setPos", (void (mrpt::gui::CDisplayWindow::*)(int, int)) &mrpt::gui::CDisplayWindow::setPos, "Changes the position of the window on the screen.\n\nC++: mrpt::gui::CDisplayWindow::setPos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("enableCursorCoordinatesVisualization", (void (mrpt::gui::CDisplayWindow::*)(bool)) &mrpt::gui::CDisplayWindow::enableCursorCoordinatesVisualization, "Enables or disables the visualization of cursor coordinates on the\n window caption (default = enabled).\n\nC++: mrpt::gui::CDisplayWindow::enableCursorCoordinatesVisualization(bool) --> void", pybind11::arg("enable"));
		cl.def("setWindowTitle", (void (mrpt::gui::CDisplayWindow::*)(const std::string &)) &mrpt::gui::CDisplayWindow::setWindowTitle, "Changes the window title text.\n\nC++: mrpt::gui::CDisplayWindow::setWindowTitle(const std::string &) --> void", pybind11::arg("str"));
	}
}
