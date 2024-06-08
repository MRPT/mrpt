#include <deque>
#include <iterator>
#include <memory>
#include <nanogui/common.h>
#include <nanogui/glcanvas.h>
#include <nanogui/layout.h>
#include <nanogui/screen.h>
#include <nanogui/theme.h>
#include <nanogui/widget.h>
#include <nanogui/window.h>
#include <sstream> // __str__
#include <string>

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

// nanogui::GLCanvas file:nanogui/glcanvas.h line:44
struct PyCallBack_nanogui_GLCanvas : public nanogui::GLCanvas {
	using nanogui::GLCanvas::GLCanvas;

	void draw(struct NVGcontext * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::GLCanvas *>(this), "draw");
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
	void drawGL() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::GLCanvas *>(this), "drawGL");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return GLCanvas::drawGL();
	}
	bool focusEvent(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::GLCanvas *>(this), "focusEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::GLCanvas *>(this), "keyboardEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::GLCanvas *>(this), "keyboardCharacterEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const nanogui::GLCanvas *>(this), "performLayout");
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

void bind_nanogui_glcanvas(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // nanogui::GLCanvas file:nanogui/glcanvas.h line:44
		pybind11::class_<nanogui::GLCanvas, std::shared_ptr<nanogui::GLCanvas>, PyCallBack_nanogui_GLCanvas> cl(M("nanogui"), "GLCanvas", "Canvas widget for rendering OpenGL content.  This widget was\n        contributed by Jan Winkler.\n\n Canvas widget that can be used to display arbitrary OpenGL content. This is\n useful to display and manipulate 3D objects as part of an interactive\n application. The implementation uses scissoring to ensure that rendered\n objects don't spill into neighboring widgets.\n\n \n **Usage**\n     Override :func:`nanogui::GLCanvas::drawGL` in subclasses to provide\n     custom drawing code.  See :ref:`nanogui_example_4`.\n\n ");
		cl.def( pybind11::init( [](PyCallBack_nanogui_GLCanvas const &o){ return new PyCallBack_nanogui_GLCanvas(o); } ) );
		cl.def( pybind11::init( [](nanogui::GLCanvas const &o){ return new nanogui::GLCanvas(o); } ) );
		cl.def("backgroundColor", (const class nanogui::Color & (nanogui::GLCanvas::*)() const) &nanogui::GLCanvas::backgroundColor, "Returns the background color.\n\nC++: nanogui::GLCanvas::backgroundColor() const --> const class nanogui::Color &", pybind11::return_value_policy::automatic);
		cl.def("setBackgroundColor", (void (nanogui::GLCanvas::*)(const class nanogui::Color &)) &nanogui::GLCanvas::setBackgroundColor, "Sets the background color.\n\nC++: nanogui::GLCanvas::setBackgroundColor(const class nanogui::Color &) --> void", pybind11::arg("backgroundColor"));
		cl.def("setDrawBorder", (void (nanogui::GLCanvas::*)(const bool)) &nanogui::GLCanvas::setDrawBorder, "Set whether to draw the widget border or not.\n\nC++: nanogui::GLCanvas::setDrawBorder(const bool) --> void", pybind11::arg("bDrawBorder"));
		cl.def("drawBorder", (const bool & (nanogui::GLCanvas::*)() const) &nanogui::GLCanvas::drawBorder, "Return whether the widget border gets drawn or not.\n\nC++: nanogui::GLCanvas::drawBorder() const --> const bool &", pybind11::return_value_policy::automatic);
		cl.def("draw", (void (nanogui::GLCanvas::*)(struct NVGcontext *)) &nanogui::GLCanvas::draw, "Draw the canvas.\n\nC++: nanogui::GLCanvas::draw(struct NVGcontext *) --> void", pybind11::arg("ctx"));
		cl.def("drawGL", (void (nanogui::GLCanvas::*)()) &nanogui::GLCanvas::drawGL, "Draw the GL scene. Override this method to draw the actual GL content.\n\nC++: nanogui::GLCanvas::drawGL() --> void");
	}
}
