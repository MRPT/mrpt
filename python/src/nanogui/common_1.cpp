#include <deque>
#include <functional>
#include <iterator>
#include <memory>
#include <nanogui/common.h>
#include <nanogui/layout.h>
#include <nanogui/screen.h>
#include <nanogui/theme.h>
#include <nanogui/widget.h>
#include <nanogui/window.h>
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

// nanogui::Screen file:nanogui/screen.h line:27
struct PyCallBack_nanogui_Screen : public nanogui::Screen {
	using nanogui::Screen::Screen;

	void drawAll() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "drawAll");
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
	void drawContents() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "drawContents");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Screen::drawContents();
	}
	void onIdleLoopTasks() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "onIdleLoopTasks");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Screen::onIdleLoopTasks();
	}
	bool dropEvent(const class std::vector<std::string > & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "dropEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Screen::dropEvent(a0);
	}
	bool keyboardEvent(int a0, int a1, int a2, int a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "keyboardEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return Screen::keyboardEvent(a0, a1, a2, a3);
	}
	bool keyboardCharacterEvent(unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "keyboardCharacterEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "focusEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "performLayout");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Screen *>(this), "draw");
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

// nanogui::Window file:nanogui/window.h line:24
struct PyCallBack_nanogui_Window : public nanogui::Window {
	using nanogui::Window::Window;

	void draw(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Window *>(this), "draw");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Window::draw(a0);
	}
	void performLayout(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Window *>(this), "performLayout");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Window::performLayout(a0);
	}
	void refreshRelativePlacement() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Window *>(this), "refreshRelativePlacement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Window::refreshRelativePlacement();
	}
	bool focusEvent(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Window *>(this), "focusEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Window *>(this), "keyboardEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::Window *>(this), "keyboardCharacterEvent");
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
};

void bind_nanogui_common_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // nanogui::Color file:nanogui/common.h line:193
		pybind11::class_<nanogui::Color, std::shared_ptr<nanogui::Color>> cl(M("nanogui"), "Color", "Stores an RGBA floating point color value.\n\n This class simply wraps around an ``Eigen::Vector4f``, providing some convenient\n methods and terminology for thinking of it as a color.  The data operates in the\n same way as ``Eigen::Vector4f``, and the following values are identical:\n\n \n +---------+-------------+-----------------------+-------------+\n | Channel | Array Index | Eigen::Vector4f Value | Color Value |\n +=========+=============+=======================+=============+\n | Red     | ``0``       | x()                   | r()         |\n +---------+-------------+-----------------------+-------------+\n | Green   | ``1``       | y()                   | g()         |\n +---------+-------------+-----------------------+-------------+\n | Blue    | ``2``       | z()                   | b()         |\n +---------+-------------+-----------------------+-------------+\n | Alpha   | ``3``       | w()                   | w()         |\n +---------+-------------+-----------------------+-------------+\n\n .. note::\n    The method for the alpha component is **always** ``w()``.\n \n\n You can and should still use the various convenience methods such as ``any()``,\n ``all()``, ``head<index>()``, etc provided by Eigen.");
		cl.def( pybind11::init( [](){ return new nanogui::Color(); } ) );
		cl.def( pybind11::init<float, float>(), pybind11::arg("intensity"), pybind11::arg("alpha") );

		cl.def( pybind11::init<int, int>(), pybind11::arg("intensity"), pybind11::arg("alpha") );

		cl.def( pybind11::init<float, float, float, float>(), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a") );

		cl.def( pybind11::init<int, int, int, int>(), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a") );

		cl.def( pybind11::init( [](nanogui::Color const &o){ return new nanogui::Color(o); } ) );
		cl.def("r", (float & (nanogui::Color::*)()) &nanogui::Color::r, "Return a reference to the red channel\n\nC++: nanogui::Color::r() --> float &", pybind11::return_value_policy::automatic);
		cl.def("g", (float & (nanogui::Color::*)()) &nanogui::Color::g, "Return a reference to the green channel\n\nC++: nanogui::Color::g() --> float &", pybind11::return_value_policy::automatic);
		cl.def("b", (float & (nanogui::Color::*)()) &nanogui::Color::b, "Return a reference to the blue channel\n\nC++: nanogui::Color::b() --> float &", pybind11::return_value_policy::automatic);
		cl.def("contrastingColor", (class nanogui::Color (nanogui::Color::*)() const) &nanogui::Color::contrastingColor, "Computes the luminance as ``l = 0.299r + 0.587g + 0.144b + 0.0a``.  If\n the luminance is less than 0.5, white is returned.  If the luminance is\n greater than or equal to 0.5, black is returned.  Both returns will have\n an alpha component of 1.0.\n\nC++: nanogui::Color::contrastingColor() const --> class nanogui::Color");
		cl.def("assign", (class nanogui::Color & (nanogui::Color::*)(const class nanogui::Color &)) &nanogui::Color::operator=, "C++: nanogui::Color::operator=(const class nanogui::Color &) --> class nanogui::Color &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // nanogui::Screen file:nanogui/screen.h line:27
		pybind11::class_<nanogui::Screen, std::shared_ptr<nanogui::Screen>, PyCallBack_nanogui_Screen> cl(M("nanogui"), "Screen", "Represents a display surface (i.e. a full-screen or windowed GLFW\n window) and forms the root element of a hierarchy of nanogui widgets.");
		cl.def( pybind11::init( [](){ return new nanogui::Screen(); }, [](){ return new PyCallBack_nanogui_Screen(); } ) );
		cl.def( pybind11::init( [](PyCallBack_nanogui_Screen const &o){ return new PyCallBack_nanogui_Screen(o); } ) );
		cl.def( pybind11::init( [](nanogui::Screen const &o){ return new nanogui::Screen(o); } ) );
		cl.def("performLayout", [](nanogui::Screen &o, struct NVGcontext * a0) -> void { return o.performLayout(a0); }, "", pybind11::arg("ctx"));
		cl.def("caption", (const std::string & (nanogui::Screen::*)() const) &nanogui::Screen::caption, "Get the window title bar caption\n\nC++: nanogui::Screen::caption() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setCaption", (void (nanogui::Screen::*)(const std::string &)) &nanogui::Screen::setCaption, "Set the window title bar caption\n\nC++: nanogui::Screen::setCaption(const std::string &) --> void", pybind11::arg("caption"));
		cl.def("background", (const class nanogui::Color & (nanogui::Screen::*)() const) &nanogui::Screen::background, "Return the screen's background color\n\nC++: nanogui::Screen::background() const --> const class nanogui::Color &", pybind11::return_value_policy::automatic);
		cl.def("setBackground", (void (nanogui::Screen::*)(const class nanogui::Color &)) &nanogui::Screen::setBackground, "Set the screen's background color\n\nC++: nanogui::Screen::setBackground(const class nanogui::Color &) --> void", pybind11::arg("background"));
		cl.def("setVisible", (void (nanogui::Screen::*)(bool)) &nanogui::Screen::setVisible, "Set the top-level window visibility (no effect on full-screen windows)\n\nC++: nanogui::Screen::setVisible(bool) --> void", pybind11::arg("visible"));
		cl.def("drawAll", (void (nanogui::Screen::*)()) &nanogui::Screen::drawAll, "Draw the Screen contents\n\nC++: nanogui::Screen::drawAll() --> void");
		cl.def("drawContents", (void (nanogui::Screen::*)()) &nanogui::Screen::drawContents, "Draw the window contents --- put your OpenGL draw calls here\n\nC++: nanogui::Screen::drawContents() --> void");
		cl.def("onIdleLoopTasks", (void (nanogui::Screen::*)()) &nanogui::Screen::onIdleLoopTasks, "Tasks to be run on the main opengl thread,\n not directly related with drawing\n\nC++: nanogui::Screen::onIdleLoopTasks() --> void");
		cl.def("pixelRatio", (float (nanogui::Screen::*)() const) &nanogui::Screen::pixelRatio, "Return the ratio between pixel and device coordinates (e.g. >= 2 on Mac\n Retina displays)\n\nC++: nanogui::Screen::pixelRatio() const --> float");
		cl.def("dropEvent", (bool (nanogui::Screen::*)(const class std::vector<std::string > &)) &nanogui::Screen::dropEvent, "Handle a file drop event\n\nC++: nanogui::Screen::dropEvent(const class std::vector<std::string > &) --> bool", pybind11::arg(""));
		cl.def("keyboardEvent", (bool (nanogui::Screen::*)(int, int, int, int)) &nanogui::Screen::keyboardEvent, "Default keyboard event handler\n\nC++: nanogui::Screen::keyboardEvent(int, int, int, int) --> bool", pybind11::arg("key"), pybind11::arg("scancode"), pybind11::arg("action"), pybind11::arg("modifiers"));
		cl.def("keyboardCharacterEvent", (bool (nanogui::Screen::*)(unsigned int)) &nanogui::Screen::keyboardCharacterEvent, "Text input event handler: codepoint is native endian UTF-32 format\n\nC++: nanogui::Screen::keyboardCharacterEvent(unsigned int) --> bool", pybind11::arg("codepoint"));
		cl.def("resizeCallback", (class std::function<void (class Eigen::Matrix<int, 2, 1, 0>)> (nanogui::Screen::*)() const) &nanogui::Screen::resizeCallback, "Set the resize callback\n\nC++: nanogui::Screen::resizeCallback() const --> class std::function<void (class Eigen::Matrix<int, 2, 1, 0>)>");
		cl.def("setResizeCallback", (void (nanogui::Screen::*)(const class std::function<void (class Eigen::Matrix<int, 2, 1, 0>)> &)) &nanogui::Screen::setResizeCallback, "C++: nanogui::Screen::setResizeCallback(const class std::function<void (class Eigen::Matrix<int, 2, 1, 0>)> &) --> void", pybind11::arg("callback"));
		cl.def("mouseState", (int (nanogui::Screen::*)() const) &nanogui::Screen::mouseState, "Returns the last mouse state (bitwise 1 << button)\n\nC++: nanogui::Screen::mouseState() const --> int");
		cl.def("mouseModifiers", (int (nanogui::Screen::*)() const) &nanogui::Screen::mouseModifiers, "Returns the last mouse modifiers. Bitwise or of:\n Shift=0x0001, Control=0x0002, Alt=0x0004, Super=0x0008\n\nC++: nanogui::Screen::mouseModifiers() const --> int");
		cl.def("glfwWindow", (struct GLFWwindow * (nanogui::Screen::*)()) &nanogui::Screen::glfwWindow, "Return a pointer to the underlying GLFW window data structure\n\nC++: nanogui::Screen::glfwWindow() --> struct GLFWwindow *", pybind11::return_value_policy::automatic);
		cl.def("nvgContext", (struct NVGcontext * (nanogui::Screen::*)()) &nanogui::Screen::nvgContext, "Return a pointer to the underlying nanoVG draw context\n\nC++: nanogui::Screen::nvgContext() --> struct NVGcontext *", pybind11::return_value_policy::automatic);
		cl.def("setShutdownGLFWOnDestruct", (void (nanogui::Screen::*)(bool)) &nanogui::Screen::setShutdownGLFWOnDestruct, "C++: nanogui::Screen::setShutdownGLFWOnDestruct(bool) --> void", pybind11::arg("v"));
		cl.def("shutdownGLFWOnDestruct", (bool (nanogui::Screen::*)()) &nanogui::Screen::shutdownGLFWOnDestruct, "C++: nanogui::Screen::shutdownGLFWOnDestruct() --> bool");
		cl.def("performLayout", (void (nanogui::Screen::*)()) &nanogui::Screen::performLayout, "Compute the layout of all widgets\n\nC++: nanogui::Screen::performLayout() --> void");
		cl.def("initialize", (void (nanogui::Screen::*)(struct GLFWwindow *, bool)) &nanogui::Screen::initialize, "Initialize the \n\nC++: nanogui::Screen::initialize(struct GLFWwindow *, bool) --> void", pybind11::arg("window"), pybind11::arg("shutdownGLFWOnDestruct"));
		cl.def("cursorPosCallbackEvent", (bool (nanogui::Screen::*)(double, double)) &nanogui::Screen::cursorPosCallbackEvent, "C++: nanogui::Screen::cursorPosCallbackEvent(double, double) --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("mouseButtonCallbackEvent", (bool (nanogui::Screen::*)(int, int, int)) &nanogui::Screen::mouseButtonCallbackEvent, "C++: nanogui::Screen::mouseButtonCallbackEvent(int, int, int) --> bool", pybind11::arg("button"), pybind11::arg("action"), pybind11::arg("modifiers"));
		cl.def("keyCallbackEvent", (bool (nanogui::Screen::*)(int, int, int, int)) &nanogui::Screen::keyCallbackEvent, "C++: nanogui::Screen::keyCallbackEvent(int, int, int, int) --> bool", pybind11::arg("key"), pybind11::arg("scancode"), pybind11::arg("action"), pybind11::arg("mods"));
		cl.def("charCallbackEvent", (bool (nanogui::Screen::*)(unsigned int)) &nanogui::Screen::charCallbackEvent, "C++: nanogui::Screen::charCallbackEvent(unsigned int) --> bool", pybind11::arg("codepoint"));
		cl.def("scrollCallbackEvent", (bool (nanogui::Screen::*)(double, double)) &nanogui::Screen::scrollCallbackEvent, "C++: nanogui::Screen::scrollCallbackEvent(double, double) --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("resizeCallbackEvent", (bool (nanogui::Screen::*)(int, int)) &nanogui::Screen::resizeCallbackEvent, "C++: nanogui::Screen::resizeCallbackEvent(int, int) --> bool", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("disposeWindow", (void (nanogui::Screen::*)(class nanogui::Window *)) &nanogui::Screen::disposeWindow, "C++: nanogui::Screen::disposeWindow(class nanogui::Window *) --> void", pybind11::arg("window"));
		cl.def("centerWindow", (void (nanogui::Screen::*)(class nanogui::Window *)) &nanogui::Screen::centerWindow, "C++: nanogui::Screen::centerWindow(class nanogui::Window *) --> void", pybind11::arg("window"));
		cl.def("moveWindowToFront", (void (nanogui::Screen::*)(class nanogui::Window *)) &nanogui::Screen::moveWindowToFront, "C++: nanogui::Screen::moveWindowToFront(class nanogui::Window *) --> void", pybind11::arg("window"));
		cl.def("drawWidgets", (void (nanogui::Screen::*)()) &nanogui::Screen::drawWidgets, "C++: nanogui::Screen::drawWidgets() --> void");
	}
	{ // nanogui::Window file:nanogui/window.h line:24
		pybind11::class_<nanogui::Window, std::shared_ptr<nanogui::Window>, PyCallBack_nanogui_Window> cl(M("nanogui"), "Window", "Top-level window widget.");
		cl.def( pybind11::init( [](PyCallBack_nanogui_Window const &o){ return new PyCallBack_nanogui_Window(o); } ) );
		cl.def( pybind11::init( [](nanogui::Window const &o){ return new nanogui::Window(o); } ) );
		cl.def("title", (const std::string & (nanogui::Window::*)() const) &nanogui::Window::title, "Return the window title\n\nC++: nanogui::Window::title() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setTitle", (void (nanogui::Window::*)(const std::string &)) &nanogui::Window::setTitle, "Set the window title\n\nC++: nanogui::Window::setTitle(const std::string &) --> void", pybind11::arg("title"));
		cl.def("modal", (bool (nanogui::Window::*)() const) &nanogui::Window::modal, "Is this a model dialog?\n\nC++: nanogui::Window::modal() const --> bool");
		cl.def("setModal", (void (nanogui::Window::*)(bool)) &nanogui::Window::setModal, "Set whether or not this is a modal dialog\n\nC++: nanogui::Window::setModal(bool) --> void", pybind11::arg("modal"));
		cl.def("dispose", (void (nanogui::Window::*)()) &nanogui::Window::dispose, "Dispose the window\n\nC++: nanogui::Window::dispose() --> void");
		cl.def("center", (void (nanogui::Window::*)()) &nanogui::Window::center, "Center the window in the current \n\nC++: nanogui::Window::center() --> void");
		cl.def("draw", (void (nanogui::Window::*)(struct NVGcontext *)) &nanogui::Window::draw, "Draw the window\n\nC++: nanogui::Window::draw(struct NVGcontext *) --> void", pybind11::arg("ctx"));
		cl.def("performLayout", (void (nanogui::Window::*)(struct NVGcontext *)) &nanogui::Window::performLayout, "Invoke the associated layout generator to properly place child widgets, if any\n\nC++: nanogui::Window::performLayout(struct NVGcontext *) --> void", pybind11::arg("ctx"));
	}
}
