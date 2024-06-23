#include <deque>
#include <functional>
#include <iterator>
#include <memory>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/gui/MRPT2NanoguiGLCanvas.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <nanogui/common.h>
#include <nanogui/glcanvas.h>
#include <nanogui/layout.h>
#include <nanogui/screen.h>
#include <nanogui/theme.h>
#include <nanogui/widget.h>
#include <nanogui/window.h>
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

// mrpt::gui::MRPT2NanoguiGLCanvas file:mrpt/gui/MRPT2NanoguiGLCanvas.h line:35
struct PyCallBack_mrpt_gui_MRPT2NanoguiGLCanvas : public mrpt::gui::MRPT2NanoguiGLCanvas {
	using mrpt::gui::MRPT2NanoguiGLCanvas::MRPT2NanoguiGLCanvas;

	void drawGL() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::MRPT2NanoguiGLCanvas *>(this), "drawGL");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return MRPT2NanoguiGLCanvas::drawGL();
	}
	void draw(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::MRPT2NanoguiGLCanvas *>(this), "draw");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return GLCanvas::draw(a0);
	}
	bool focusEvent(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::MRPT2NanoguiGLCanvas *>(this), "focusEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Widget::focusEvent(a0);
	}
	bool keyboardEvent(int a0, int a1, int a2, int a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::MRPT2NanoguiGLCanvas *>(this), "keyboardEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Widget::keyboardEvent(a0, a1, a2, a3);
	}
	bool keyboardCharacterEvent(unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::MRPT2NanoguiGLCanvas *>(this), "keyboardCharacterEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Widget::keyboardCharacterEvent(a0);
	}
	void performLayout(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::MRPT2NanoguiGLCanvas *>(this), "performLayout");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Widget::performLayout(a0);
	}
};

// mrpt::gui::CDisplayWindowGUI file:mrpt/gui/CDisplayWindowGUI.h line:88
struct PyCallBack_mrpt_gui_CDisplayWindowGUI : public mrpt::gui::CDisplayWindowGUI {
	using mrpt::gui::CDisplayWindowGUI::CDisplayWindowGUI;

	void drawContents() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "drawContents");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindowGUI::drawContents();
	}
	void onIdleLoopTasks() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "onIdleLoopTasks");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindowGUI::onIdleLoopTasks();
	}
	bool keyboardEvent(int a0, int a1, int a2, int a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "keyboardEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CDisplayWindowGUI::keyboardEvent(a0, a1, a2, a3);
	}
	bool dropEvent(const class std::vector<std::string > & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "dropEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CDisplayWindowGUI::dropEvent(a0);
	}
	void drawAll() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "drawAll");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Screen::drawAll();
	}
	bool keyboardCharacterEvent(unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "keyboardCharacterEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Screen::keyboardCharacterEvent(a0);
	}
	bool focusEvent(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "focusEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Widget::focusEvent(a0);
	}
	void performLayout(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "performLayout");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Widget::performLayout(a0);
	}
	void draw(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowGUI *>(this), "draw");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Widget::draw(a0);
	}
};

// mrpt::gui::CDisplayWindowPlots file:mrpt/gui/CDisplayWindowPlots.h line:33
struct PyCallBack_mrpt_gui_CDisplayWindowPlots : public mrpt::gui::CDisplayWindowPlots {
	using mrpt::gui::CDisplayWindowPlots::CDisplayWindowPlots;

	bool getLastMousePosition(int & a0, int & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowPlots *>(this), "getLastMousePosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CDisplayWindowPlots::getLastMousePosition(a0, a1);
	}
	void setCursorCross(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowPlots *>(this), "setCursorCross");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindowPlots::setCursorCross(a0);
	}
	void resize(unsigned int a0, unsigned int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowPlots *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindowPlots::resize(a0, a1);
	}
	void setPos(int a0, int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowPlots *>(this), "setPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindowPlots::setPos(a0, a1);
	}
	void setWindowTitle(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CDisplayWindowPlots *>(this), "setWindowTitle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisplayWindowPlots::setWindowTitle(a0);
	}
};

void bind_mrpt_gui_MRPT2NanoguiGLCanvas(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::gui::MRPT2NanoguiGLCanvas file:mrpt/gui/MRPT2NanoguiGLCanvas.h line:35
		pybind11::class_<mrpt::gui::MRPT2NanoguiGLCanvas, std::shared_ptr<mrpt::gui::MRPT2NanoguiGLCanvas>, PyCallBack_mrpt_gui_MRPT2NanoguiGLCanvas, nanogui::GLCanvas> cl(M("mrpt::gui"), "MRPT2NanoguiGLCanvas", "An extension of nanogui::GLCanvas to render MRPT OpenGL scenes.\n\n Directly access `scene` (locking its mutex `scene_mtx` first) to update the\n scene to be rendered.\n\n \n CDisplayWindowGUI\n \n\n\n ");
		cl.def_readwrite("scene", &mrpt::gui::MRPT2NanoguiGLCanvas::scene);
		cl.def("drawGL", (void (mrpt::gui::MRPT2NanoguiGLCanvas::*)()) &mrpt::gui::MRPT2NanoguiGLCanvas::drawGL, "C++: mrpt::gui::MRPT2NanoguiGLCanvas::drawGL() --> void");
		cl.def("camera", (class mrpt::gui::CGlCanvasBase & (mrpt::gui::MRPT2NanoguiGLCanvas::*)()) &mrpt::gui::MRPT2NanoguiGLCanvas::camera, "C++: mrpt::gui::MRPT2NanoguiGLCanvas::camera() --> class mrpt::gui::CGlCanvasBase &", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::gui::CDisplayWindowGUI_Params file:mrpt/gui/CDisplayWindowGUI.h line:33
		pybind11::class_<mrpt::gui::CDisplayWindowGUI_Params, std::shared_ptr<mrpt::gui::CDisplayWindowGUI_Params>> cl(M("mrpt::gui"), "CDisplayWindowGUI_Params", "Additional parameters to change the window behavior and OpenGL context");
		cl.def( pybind11::init( [](){ return new mrpt::gui::CDisplayWindowGUI_Params(); } ) );
		cl.def( pybind11::init( [](mrpt::gui::CDisplayWindowGUI_Params const &o){ return new mrpt::gui::CDisplayWindowGUI_Params(o); } ) );
		cl.def_readwrite("resizable", &mrpt::gui::CDisplayWindowGUI_Params::resizable);
		cl.def_readwrite("fullscreen", &mrpt::gui::CDisplayWindowGUI_Params::fullscreen);
		cl.def_readwrite("colorBits", &mrpt::gui::CDisplayWindowGUI_Params::colorBits);
		cl.def_readwrite("alphaBits", &mrpt::gui::CDisplayWindowGUI_Params::alphaBits);
		cl.def_readwrite("depthBits", &mrpt::gui::CDisplayWindowGUI_Params::depthBits);
		cl.def_readwrite("stencilBits", &mrpt::gui::CDisplayWindowGUI_Params::stencilBits);
		cl.def_readwrite("nSamples", &mrpt::gui::CDisplayWindowGUI_Params::nSamples);
		cl.def_readwrite("glMajor", &mrpt::gui::CDisplayWindowGUI_Params::glMajor);
		cl.def_readwrite("glMinor", &mrpt::gui::CDisplayWindowGUI_Params::glMinor);
		cl.def_readwrite("maximized", &mrpt::gui::CDisplayWindowGUI_Params::maximized);
		cl.def_readwrite("gles_context", &mrpt::gui::CDisplayWindowGUI_Params::gles_context);
	}
	{ // mrpt::gui::CDisplayWindowGUI file:mrpt/gui/CDisplayWindowGUI.h line:88
		pybind11::class_<mrpt::gui::CDisplayWindowGUI, std::shared_ptr<mrpt::gui::CDisplayWindowGUI>, PyCallBack_mrpt_gui_CDisplayWindowGUI, nanogui::Screen> cl(M("mrpt::gui"), "CDisplayWindowGUI", "A window with powerful GUI capabilities, via the nanogui library.\n\n You can add a background mrpt::opengl::Scene object rendered on the\n background of the entire window by setting an object in field\n `background_scene`, locking its mutex `background_scene_mtx`.\n\n Refer to nanogui API docs or [MRPT examples](examples.html) for further usage\n examples. A typical lifecycle of a GUI app with this class might look like:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n ![mrpt::gui::CDisplayWindowGUI screenshot](preview_CDisplayWindowGUI.png)\n\n Create managed subwindows with createManagedSubWindow(), with built-in\n support for minimize and restore.\n See demo video in: https://www.youtube.com/watch?v=QKMzdlZRW50\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::gui::CDisplayWindowGUI(); }, [](){ return new PyCallBack_mrpt_gui_CDisplayWindowGUI(); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::gui::CDisplayWindowGUI(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_gui_CDisplayWindowGUI(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, unsigned int const & a1){ return new mrpt::gui::CDisplayWindowGUI(a0, a1); }, [](const std::string & a0, unsigned int const & a1){ return new PyCallBack_mrpt_gui_CDisplayWindowGUI(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, unsigned int const & a1, unsigned int const & a2){ return new mrpt::gui::CDisplayWindowGUI(a0, a1, a2); }, [](const std::string & a0, unsigned int const & a1, unsigned int const & a2){ return new PyCallBack_mrpt_gui_CDisplayWindowGUI(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<const std::string &, unsigned int, unsigned int, const struct mrpt::gui::CDisplayWindowGUI_Params &>(), pybind11::arg("caption"), pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("p") );

		cl.def_readwrite("background_scene", &mrpt::gui::CDisplayWindowGUI::background_scene);
		cl.def("resize", (void (mrpt::gui::CDisplayWindowGUI::*)(unsigned int, unsigned int)) &mrpt::gui::CDisplayWindowGUI::resize, "Resizes the window \n\nC++: mrpt::gui::CDisplayWindowGUI::resize(unsigned int, unsigned int) --> void", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("setPos", (void (mrpt::gui::CDisplayWindowGUI::*)(int, int)) &mrpt::gui::CDisplayWindowGUI::setPos, "Changes the position of the window on the screen. \n\nC++: mrpt::gui::CDisplayWindowGUI::setPos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setWindowTitle", (void (mrpt::gui::CDisplayWindowGUI::*)(const std::string &)) &mrpt::gui::CDisplayWindowGUI::setWindowTitle, "Changes the window title. \n\nC++: mrpt::gui::CDisplayWindowGUI::setWindowTitle(const std::string &) --> void", pybind11::arg("str"));
		cl.def("setIcon", (void (mrpt::gui::CDisplayWindowGUI::*)(const class mrpt::img::CImage &)) &mrpt::gui::CDisplayWindowGUI::setIcon, "Sets the window icon, which must must either RGB or (preferred) RGBA\n channels. You can read it from a .png or .ico file using\n mrpt::img::CImage::loadFromFile().\n\n The image will be resized as needed. Good sizes include 16x16, 32x32 and\n 48x48.\n\n \n (New in MRPT 2.4.2)\n\nC++: mrpt::gui::CDisplayWindowGUI::setIcon(const class mrpt::img::CImage &) --> void", pybind11::arg("img"));
		cl.def("setIconFromData", (void (mrpt::gui::CDisplayWindowGUI::*)(const char *, unsigned int, unsigned int, const unsigned char)) &mrpt::gui::CDisplayWindowGUI::setIconFromData, "\"GIMP header image formar (RGB)\", with a transparent color.\n \n\n (New in MRPT 2.4.2)\n\nC++: mrpt::gui::CDisplayWindowGUI::setIconFromData(const char *, unsigned int, unsigned int, const unsigned char) --> void", pybind11::arg("imgData"), pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("transparent"));
		cl.def("addLoopCallback", (void (mrpt::gui::CDisplayWindowGUI::*)(const class std::function<void (void)> &)) &mrpt::gui::CDisplayWindowGUI::addLoopCallback, "Every time the window is about to be repainted, an optional callback can\n be called, if provided via this method. This method can be safely called\n multiple times to register multiple callbacks.\n\n \n (New in MRPT 2.3.2)\n\nC++: mrpt::gui::CDisplayWindowGUI::addLoopCallback(const class std::function<void (void)> &) --> void", pybind11::arg("callback"));
		cl.def("addDropFilesCallback", (void (mrpt::gui::CDisplayWindowGUI::*)(const class std::function<bool (const class std::vector<std::string > &)> &)) &mrpt::gui::CDisplayWindowGUI::addDropFilesCallback, "Handles drag-and drop events of files into the window.\n This method can be safely called multiple times to register multiple\n callbacks.\n\n \n (New in MRPT 2.3.2)\n\nC++: mrpt::gui::CDisplayWindowGUI::addDropFilesCallback(const class std::function<bool (const class std::vector<std::string > &)> &) --> void", pybind11::arg("callback"));
		cl.def("addKeyboardCallback", (void (mrpt::gui::CDisplayWindowGUI::*)(const class std::function<bool (int, int, int, int)> &)) &mrpt::gui::CDisplayWindowGUI::addKeyboardCallback, "Handles keyboard events.\n This method can be safely called multiple times to register multiple\n callbacks.\n\n \n (New in MRPT 2.3.2)\n\nC++: mrpt::gui::CDisplayWindowGUI::addKeyboardCallback(const class std::function<bool (int, int, int, int)> &) --> void", pybind11::arg("callback"));
		cl.def("setLoopCallback", (void (mrpt::gui::CDisplayWindowGUI::*)(const class std::function<void (void)> &)) &mrpt::gui::CDisplayWindowGUI::setLoopCallback, "This call replaces *all* existing loop callbacks. See\n addLoopCallback().\n \n\n In MRPT 2.3.2, replaced by addLoopCallback()\n\nC++: mrpt::gui::CDisplayWindowGUI::setLoopCallback(const class std::function<void (void)> &) --> void", pybind11::arg("callback"));
		cl.def("loopCallback", (class std::function<void (void)> (mrpt::gui::CDisplayWindowGUI::*)() const) &mrpt::gui::CDisplayWindowGUI::loopCallback, "In MRPT 2.3.2, replaced by loopCallbacks() \n\nC++: mrpt::gui::CDisplayWindowGUI::loopCallback() const --> class std::function<void (void)>");
		cl.def("setDropFilesCallback", (void (mrpt::gui::CDisplayWindowGUI::*)(const class std::function<bool (const class std::vector<std::string > &)> &)) &mrpt::gui::CDisplayWindowGUI::setDropFilesCallback, "This call replaces *all* existing drop file callbacks. See\n addDropFilesCallback().\n \n\n In MRPT 2.3.2, replaced by addDropFilesCallback()\n\nC++: mrpt::gui::CDisplayWindowGUI::setDropFilesCallback(const class std::function<bool (const class std::vector<std::string > &)> &) --> void", pybind11::arg("callback"));
		cl.def("dropFilesCallback", (class std::function<bool (const class std::vector<std::string > &)> (mrpt::gui::CDisplayWindowGUI::*)() const) &mrpt::gui::CDisplayWindowGUI::dropFilesCallback, "In MRPT 2.3.2, replaced by dropFilesCallbacks() \n\nC++: mrpt::gui::CDisplayWindowGUI::dropFilesCallback() const --> class std::function<bool (const class std::vector<std::string > &)>");
		cl.def("setKeyboardCallback", (void (mrpt::gui::CDisplayWindowGUI::*)(const class std::function<bool (int, int, int, int)> &)) &mrpt::gui::CDisplayWindowGUI::setKeyboardCallback, "This call replaces *all* existing keyboard callbacks. See\n addKeyboardCallback().\n \n\n In MRPT 2.3.2, replaced by addKeyboardCallback()\n\nC++: mrpt::gui::CDisplayWindowGUI::setKeyboardCallback(const class std::function<bool (int, int, int, int)> &) --> void", pybind11::arg("callback"));
		cl.def("keyboardCallback", (class std::function<bool (int, int, int, int)> (mrpt::gui::CDisplayWindowGUI::*)() const) &mrpt::gui::CDisplayWindowGUI::keyboardCallback, "In MRPT 2.3.2, replaced by keyboardCallbacks() \n\nC++: mrpt::gui::CDisplayWindowGUI::keyboardCallback() const --> class std::function<bool (int, int, int, int)>");
		cl.def("createManagedSubWindow", (class nanogui::Window * (mrpt::gui::CDisplayWindowGUI::*)(const std::string &)) &mrpt::gui::CDisplayWindowGUI::createManagedSubWindow, "Creates and return a nanogui::Window, adds to it basic minimize/restore\n tool buttons and add it to the list of handled subwindows so it gets\n listed in the subwindows control UI.\n User should set a layout manager, width, height, etc. in the returned\n window as desired.\n\n The returned object is owned by the nanogui system, you should NOT delete\n it.\n\n The first time this is called, an additional subWindow will be created\n (default position:bottom left corner) to hold minimized windows.\n You can access and modify this windows via getSubWindowsUI().\n\n \n [New in MRPT 2.1.1]\n\nC++: mrpt::gui::CDisplayWindowGUI::createManagedSubWindow(const std::string &) --> class nanogui::Window *", pybind11::return_value_policy::automatic, pybind11::arg("title"));
		cl.def("getSubWindowsUI", (class nanogui::Window * (mrpt::gui::CDisplayWindowGUI::*)()) &mrpt::gui::CDisplayWindowGUI::getSubWindowsUI, "C++: mrpt::gui::CDisplayWindowGUI::getSubWindowsUI() --> class nanogui::Window *", pybind11::return_value_policy::automatic);
		cl.def("getSubwindow", (const class nanogui::Window * (mrpt::gui::CDisplayWindowGUI::*)(size_t) const) &mrpt::gui::CDisplayWindowGUI::getSubwindow, "Direct (read-only) access to managed subwindows by 0-based index\n (creation order).\n\n \n createManagedSubWindow()\n \n\n [New in MRPT 2.3.1]\n\nC++: mrpt::gui::CDisplayWindowGUI::getSubwindow(size_t) const --> const class nanogui::Window *", pybind11::return_value_policy::automatic, pybind11::arg("index"));
		cl.def("getSubwindowCount", (size_t (mrpt::gui::CDisplayWindowGUI::*)() const) &mrpt::gui::CDisplayWindowGUI::getSubwindowCount, "Get the number of managed subwindows. \n createManagedSubWindow()\n \n\n [New in MRPT 2.3.1]\n\nC++: mrpt::gui::CDisplayWindowGUI::getSubwindowCount() const --> size_t");
		cl.def("subwindowMinimize", (void (mrpt::gui::CDisplayWindowGUI::*)(size_t)) &mrpt::gui::CDisplayWindowGUI::subwindowMinimize, "Minimize a subwindow. \n [New in MRPT 2.3.1] \n\nC++: mrpt::gui::CDisplayWindowGUI::subwindowMinimize(size_t) --> void", pybind11::arg("index"));
		cl.def("subwindowRestore", (void (mrpt::gui::CDisplayWindowGUI::*)(size_t)) &mrpt::gui::CDisplayWindowGUI::subwindowRestore, "Restore a minimized subwindow. \n [New in MRPT 2.3.1] \n\nC++: mrpt::gui::CDisplayWindowGUI::subwindowRestore(size_t) --> void", pybind11::arg("index"));
		cl.def("subwindowSetFocused", (void (mrpt::gui::CDisplayWindowGUI::*)(size_t)) &mrpt::gui::CDisplayWindowGUI::subwindowSetFocused, "Forces focus on a subwindow. \n [New in MRPT 2.3.1] \n\nC++: mrpt::gui::CDisplayWindowGUI::subwindowSetFocused(size_t) --> void", pybind11::arg("index"));
		cl.def("camera", (class mrpt::gui::CGlCanvasBase & (mrpt::gui::CDisplayWindowGUI::*)()) &mrpt::gui::CDisplayWindowGUI::camera, "C++: mrpt::gui::CDisplayWindowGUI::camera() --> class mrpt::gui::CGlCanvasBase &", pybind11::return_value_policy::automatic);
		cl.def("nanogui_screen", (class nanogui::Screen * (mrpt::gui::CDisplayWindowGUI::*)()) &mrpt::gui::CDisplayWindowGUI::nanogui_screen, "@{ \n\nC++: mrpt::gui::CDisplayWindowGUI::nanogui_screen() --> class nanogui::Screen *", pybind11::return_value_policy::automatic);
		cl.def("drawContents", (void (mrpt::gui::CDisplayWindowGUI::*)()) &mrpt::gui::CDisplayWindowGUI::drawContents, "@} \n\nC++: mrpt::gui::CDisplayWindowGUI::drawContents() --> void");
		cl.def("onIdleLoopTasks", (void (mrpt::gui::CDisplayWindowGUI::*)()) &mrpt::gui::CDisplayWindowGUI::onIdleLoopTasks, "C++: mrpt::gui::CDisplayWindowGUI::onIdleLoopTasks() --> void");
	}
	{ // mrpt::gui::CDisplayWindowPlots file:mrpt/gui/CDisplayWindowPlots.h line:33
		pybind11::class_<mrpt::gui::CDisplayWindowPlots, std::shared_ptr<mrpt::gui::CDisplayWindowPlots>, PyCallBack_mrpt_gui_CDisplayWindowPlots, mrpt::gui::CBaseGUIWindow> cl(M("mrpt::gui"), "CDisplayWindowPlots", "Create a GUI window and display plots with MATLAB-like interfaces and\n commands.\n\n  For a list of supported events with the observer/observable pattern, see the\n discussion in mrpt::gui::CBaseGUIWindow.\n\n   See CDisplayWindowPlots::plot\n\n ![mrpt::gui::CDisplayWindowPlots screenshot](preview_CDisplayWindowPlots.png)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::gui::CDisplayWindowPlots(); }, [](){ return new PyCallBack_mrpt_gui_CDisplayWindowPlots(); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::gui::CDisplayWindowPlots(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_gui_CDisplayWindowPlots(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, unsigned int const & a1){ return new mrpt::gui::CDisplayWindowPlots(a0, a1); }, [](const std::string & a0, unsigned int const & a1){ return new PyCallBack_mrpt_gui_CDisplayWindowPlots(a0, a1); } ), "doc");
		cl.def( pybind11::init<const std::string &, unsigned int, unsigned int>(), pybind11::arg("windowCaption"), pybind11::arg("initialWidth"), pybind11::arg("initialHeight") );

		cl.def_static("Create", [](const std::string & a0) -> std::shared_ptr<class mrpt::gui::CDisplayWindowPlots> { return mrpt::gui::CDisplayWindowPlots::Create(a0); }, "", pybind11::arg("windowCaption"));
		cl.def_static("Create", [](const std::string & a0, unsigned int const & a1) -> std::shared_ptr<class mrpt::gui::CDisplayWindowPlots> { return mrpt::gui::CDisplayWindowPlots::Create(a0, a1); }, "", pybind11::arg("windowCaption"), pybind11::arg("initialWindowWidth"));
		cl.def_static("Create", (class std::shared_ptr<class mrpt::gui::CDisplayWindowPlots> (*)(const std::string &, unsigned int, unsigned int)) &mrpt::gui::CDisplayWindowPlots::Create, "Class factory returning a smart pointer, equivalent to\n `std::make_shared<>(...)` \n\nC++: mrpt::gui::CDisplayWindowPlots::Create(const std::string &, unsigned int, unsigned int) --> class std::shared_ptr<class mrpt::gui::CDisplayWindowPlots>", pybind11::arg("windowCaption"), pybind11::arg("initialWindowWidth"), pybind11::arg("initialWindowHeight"));
		cl.def("getLastMousePosition", (bool (mrpt::gui::CDisplayWindowPlots::*)(int &, int &) const) &mrpt::gui::CDisplayWindowPlots::getLastMousePosition, "Gets the last x,y pixel coordinates of the mouse. \n False if the\n window is closed. \n\nC++: mrpt::gui::CDisplayWindowPlots::getLastMousePosition(int &, int &) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setCursorCross", (void (mrpt::gui::CDisplayWindowPlots::*)(bool)) &mrpt::gui::CDisplayWindowPlots::setCursorCross, "Set cursor style to default (cursorIsCross=false) or to a cross\n (cursorIsCross=true) \n\nC++: mrpt::gui::CDisplayWindowPlots::setCursorCross(bool) --> void", pybind11::arg("cursorIsCross"));
		cl.def("resize", (void (mrpt::gui::CDisplayWindowPlots::*)(unsigned int, unsigned int)) &mrpt::gui::CDisplayWindowPlots::resize, "Resizes the window, stretching the image to fit into the display area.\n\nC++: mrpt::gui::CDisplayWindowPlots::resize(unsigned int, unsigned int) --> void", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("setPos", (void (mrpt::gui::CDisplayWindowPlots::*)(int, int)) &mrpt::gui::CDisplayWindowPlots::setPos, "Changes the position of the window on the screen.\n\nC++: mrpt::gui::CDisplayWindowPlots::setPos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setWindowTitle", (void (mrpt::gui::CDisplayWindowPlots::*)(const std::string &)) &mrpt::gui::CDisplayWindowPlots::setWindowTitle, "Changes the window title text.\n\nC++: mrpt::gui::CDisplayWindowPlots::setWindowTitle(const std::string &) --> void", pybind11::arg("str"));
		cl.def("enableMousePanZoom", (void (mrpt::gui::CDisplayWindowPlots::*)(bool)) &mrpt::gui::CDisplayWindowPlots::enableMousePanZoom, "Enable/disable the feature of pan/zoom with the mouse (default=enabled)\n\nC++: mrpt::gui::CDisplayWindowPlots::enableMousePanZoom(bool) --> void", pybind11::arg("enabled"));
		cl.def("axis", [](mrpt::gui::CDisplayWindowPlots &o, float const & a0, float const & a1, float const & a2, float const & a3) -> void { return o.axis(a0, a1, a2, a3); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"));
		cl.def("axis", (void (mrpt::gui::CDisplayWindowPlots::*)(float, float, float, float, bool)) &mrpt::gui::CDisplayWindowPlots::axis, "Set the view area according to the passed coordinated. \n\nC++: mrpt::gui::CDisplayWindowPlots::axis(float, float, float, float, bool) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("aspectRatioFix"));
		cl.def("axis_equal", [](mrpt::gui::CDisplayWindowPlots &o) -> void { return o.axis_equal(); }, "");
		cl.def("axis_equal", (void (mrpt::gui::CDisplayWindowPlots::*)(bool)) &mrpt::gui::CDisplayWindowPlots::axis_equal, "Enable/disable the fixed X/Y aspect ratio fix feature\n (default=disabled). \n\nC++: mrpt::gui::CDisplayWindowPlots::axis_equal(bool) --> void", pybind11::arg("enable"));
		cl.def("axis_fit", [](mrpt::gui::CDisplayWindowPlots &o) -> void { return o.axis_fit(); }, "");
		cl.def("axis_fit", (void (mrpt::gui::CDisplayWindowPlots::*)(bool)) &mrpt::gui::CDisplayWindowPlots::axis_fit, "Fix automatically the view area according to existing graphs. \n\nC++: mrpt::gui::CDisplayWindowPlots::axis_fit(bool) --> void", pybind11::arg("aspectRatioFix"));
		cl.def("image", [](mrpt::gui::CDisplayWindowPlots &o, const class mrpt::img::CImage & a0, float const & a1, float const & a2, float const & a3, float const & a4) -> void { return o.image(a0, a1, a2, a3, a4); }, "", pybind11::arg("img"), pybind11::arg("x_left"), pybind11::arg("y_bottom"), pybind11::arg("x_width"), pybind11::arg("y_height"));
		cl.def("image", (void (mrpt::gui::CDisplayWindowPlots::*)(const class mrpt::img::CImage &, float, float, float, float, const std::string &)) &mrpt::gui::CDisplayWindowPlots::image, "Adds a bitmap image layer.\n  Each call to this function creates a new layer, unless the plot name\n coincides with an already existing plot: in this case the new values are\n used to update this existing layer (this also applies to using the\n default plot name).\n\n \n axis, axis_equal, axis_fit, hold_on, hold_off\n\nC++: mrpt::gui::CDisplayWindowPlots::image(const class mrpt::img::CImage &, float, float, float, float, const std::string &) --> void", pybind11::arg("img"), pybind11::arg("x_left"), pybind11::arg("y_bottom"), pybind11::arg("x_width"), pybind11::arg("y_height"), pybind11::arg("plotName"));
		cl.def("clear", (void (mrpt::gui::CDisplayWindowPlots::*)()) &mrpt::gui::CDisplayWindowPlots::clear, "Remove all plot objects in the display.\n \n\n plot\n\nC++: mrpt::gui::CDisplayWindowPlots::clear() --> void");
		cl.def("clf", (void (mrpt::gui::CDisplayWindowPlots::*)()) &mrpt::gui::CDisplayWindowPlots::clf, "Remove all plot objects in the display (clear and clf do exactly the\n same).\n \n\n plot, hold_on, hold_off\n\nC++: mrpt::gui::CDisplayWindowPlots::clf() --> void");
		cl.def("hold_on", (void (mrpt::gui::CDisplayWindowPlots::*)()) &mrpt::gui::CDisplayWindowPlots::hold_on, "Enables keeping all the graphs, instead of overwriting them.\n \n\n hold_off, plot\n\nC++: mrpt::gui::CDisplayWindowPlots::hold_on() --> void");
		cl.def("hold_off", (void (mrpt::gui::CDisplayWindowPlots::*)()) &mrpt::gui::CDisplayWindowPlots::hold_off, "Disables keeping all the graphs (this is the default behavior).\n \n\n hold_on, plot\n\nC++: mrpt::gui::CDisplayWindowPlots::hold_off() --> void");
		cl.def("addPopupMenuEntry", (void (mrpt::gui::CDisplayWindowPlots::*)(const std::string &, int)) &mrpt::gui::CDisplayWindowPlots::addPopupMenuEntry, "Disables keeping all the graphs (this is the default behavior).\n \n\n The text that appears in the new popup menu item.\n \n\n Any positive number (0,1,..). Used to tell which menu was\n selected in the user callback.\n \n\n setMenuCallback\n\nC++: mrpt::gui::CDisplayWindowPlots::addPopupMenuEntry(const std::string &, int) --> void", pybind11::arg("label"), pybind11::arg("menuID"));
	}
}
