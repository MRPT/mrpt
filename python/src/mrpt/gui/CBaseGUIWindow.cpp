#include <iterator>
#include <memory>
#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/keycodes.h>
#include <mrpt/img/TPixelCoord.h>
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

// mrpt::gui::CBaseGUIWindow file:mrpt/gui/CBaseGUIWindow.h line:40
struct PyCallBack_mrpt_gui_CBaseGUIWindow : public mrpt::gui::CBaseGUIWindow {
	using mrpt::gui::CBaseGUIWindow::CBaseGUIWindow;

	void resize(unsigned int a0, unsigned int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CBaseGUIWindow *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CBaseGUIWindow::resize\"");
	}
	void setPos(int a0, int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CBaseGUIWindow *>(this), "setPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CBaseGUIWindow::setPos\"");
	}
	void setWindowTitle(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CBaseGUIWindow *>(this), "setWindowTitle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CBaseGUIWindow::setWindowTitle\"");
	}
	bool getLastMousePosition(int & a0, int & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CBaseGUIWindow *>(this), "getLastMousePosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CBaseGUIWindow::getLastMousePosition\"");
	}
	void setCursorCross(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CBaseGUIWindow *>(this), "setCursorCross");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CBaseGUIWindow::setCursorCross\"");
	}
};

// mrpt::gui::mrptEventWindowChar file:mrpt/gui/CBaseGUIWindow.h line:168
struct PyCallBack_mrpt_gui_mrptEventWindowChar : public mrpt::gui::mrptEventWindowChar {
	using mrpt::gui::mrptEventWindowChar::mrptEventWindowChar;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::mrptEventWindowChar *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventWindowChar::do_nothing();
	}
};

// mrpt::gui::mrptEventWindowResize file:mrpt/gui/CBaseGUIWindow.h line:194
struct PyCallBack_mrpt_gui_mrptEventWindowResize : public mrpt::gui::mrptEventWindowResize {
	using mrpt::gui::mrptEventWindowResize::mrptEventWindowResize;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::mrptEventWindowResize *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventWindowResize::do_nothing();
	}
};

// mrpt::gui::mrptEventMouseDown file:mrpt/gui/CBaseGUIWindow.h line:219
struct PyCallBack_mrpt_gui_mrptEventMouseDown : public mrpt::gui::mrptEventMouseDown {
	using mrpt::gui::mrptEventMouseDown::mrptEventMouseDown;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::mrptEventMouseDown *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventMouseDown::do_nothing();
	}
};

// mrpt::gui::mrptEventMouseMove file:mrpt/gui/CBaseGUIWindow.h line:244
struct PyCallBack_mrpt_gui_mrptEventMouseMove : public mrpt::gui::mrptEventMouseMove {
	using mrpt::gui::mrptEventMouseMove::mrptEventMouseMove;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::mrptEventMouseMove *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventMouseMove::do_nothing();
	}
};

// mrpt::gui::mrptEventWindowClosed file:mrpt/gui/CBaseGUIWindow.h line:276
struct PyCallBack_mrpt_gui_mrptEventWindowClosed : public mrpt::gui::mrptEventWindowClosed {
	using mrpt::gui::mrptEventWindowClosed::mrptEventWindowClosed;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::mrptEventWindowClosed *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventWindowClosed::do_nothing();
	}
};

void bind_mrpt_gui_CBaseGUIWindow(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::gui::CBaseGUIWindow file:mrpt/gui/CBaseGUIWindow.h line:40
		pybind11::class_<mrpt::gui::CBaseGUIWindow, std::shared_ptr<mrpt::gui::CBaseGUIWindow>, PyCallBack_mrpt_gui_CBaseGUIWindow, mrpt::system::CObservable> cl(M("mrpt::gui"), "CBaseGUIWindow", "The base class for GUI window classes based on wxWidgets.\n\n This class can be observed (see mrpt::system::CObserver) for the following\n events (see mrpt::system::mrptEvent):\n - mrpt::gui::mrptEventWindowChar\n - mrpt::gui::mrptEventWindowResize\n - mrpt::gui::mrptEventMouseDown\n - mrpt::gui::mrptEventWindowClosed\n\n See derived classes to check if they emit other additional events.\n\n IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread, so all your code in the handler\n must be thread safe.\n \n\n\n ");
		cl.def( pybind11::init( [](void * a0, int const & a1, int const & a2){ return new PyCallBack_mrpt_gui_CBaseGUIWindow(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<void *, int, int, const std::string &>(), pybind11::arg("winobj_voidptr"), pybind11::arg("CMD_CREATE_WIN"), pybind11::arg("CMD_DESTROY_WIN"), pybind11::arg("initial_caption") );

		cl.def("getWxObject", (void * (mrpt::gui::CBaseGUIWindow::*)()) &mrpt::gui::CBaseGUIWindow::getWxObject, "Read-only access to the wxDialog object. \n\nC++: mrpt::gui::CBaseGUIWindow::getWxObject() --> void *", pybind11::return_value_policy::automatic);
		cl.def("notifyChildWindowDestruction", (void (mrpt::gui::CBaseGUIWindow::*)()) &mrpt::gui::CBaseGUIWindow::notifyChildWindowDestruction, "Called by wx main thread to set m_hwnd to NULL. \n\nC++: mrpt::gui::CBaseGUIWindow::notifyChildWindowDestruction() --> void");
		cl.def("notifySemThreadReady", (void (mrpt::gui::CBaseGUIWindow::*)()) &mrpt::gui::CBaseGUIWindow::notifySemThreadReady, "Called by wx main thread to signal the semaphore that the wx window is\n built and ready. \n\nC++: mrpt::gui::CBaseGUIWindow::notifySemThreadReady() --> void");
		cl.def("isOpen", (bool (mrpt::gui::CBaseGUIWindow::*)()) &mrpt::gui::CBaseGUIWindow::isOpen, "Returns false if the user has already closed the window.\n\nC++: mrpt::gui::CBaseGUIWindow::isOpen() --> bool");
		cl.def("resize", (void (mrpt::gui::CBaseGUIWindow::*)(unsigned int, unsigned int)) &mrpt::gui::CBaseGUIWindow::resize, "Resizes the window, stretching the image to fit into the display area.\n\nC++: mrpt::gui::CBaseGUIWindow::resize(unsigned int, unsigned int) --> void", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("setPos", (void (mrpt::gui::CBaseGUIWindow::*)(int, int)) &mrpt::gui::CBaseGUIWindow::setPos, "Changes the position of the window on the screen.\n\nC++: mrpt::gui::CBaseGUIWindow::setPos(int, int) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setWindowTitle", (void (mrpt::gui::CBaseGUIWindow::*)(const std::string &)) &mrpt::gui::CBaseGUIWindow::setWindowTitle, "Changes the window title text.\n\nC++: mrpt::gui::CBaseGUIWindow::setWindowTitle(const std::string &) --> void", pybind11::arg("str"));
		cl.def("getLastMousePosition", (bool (mrpt::gui::CBaseGUIWindow::*)(int &, int &) const) &mrpt::gui::CBaseGUIWindow::getLastMousePosition, "Gets the last x,y pixel coordinates of the mouse. \n False if the\n window is closed. \n\nC++: mrpt::gui::CBaseGUIWindow::getLastMousePosition(int &, int &) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setCursorCross", (void (mrpt::gui::CBaseGUIWindow::*)(bool)) &mrpt::gui::CBaseGUIWindow::setCursorCross, "Set cursor style to default (cursorIsCross=false) or to a cross\n (cursorIsCross=true) \n\nC++: mrpt::gui::CBaseGUIWindow::setCursorCross(bool) --> void", pybind11::arg("cursorIsCross"));
		cl.def("waitForKey", [](mrpt::gui::CBaseGUIWindow &o) -> int { return o.waitForKey(); }, "");
		cl.def("waitForKey", [](mrpt::gui::CBaseGUIWindow &o, bool const & a0) -> int { return o.waitForKey(a0); }, "", pybind11::arg("ignoreControlKeys"));
		cl.def("waitForKey", (int (mrpt::gui::CBaseGUIWindow::*)(bool, enum mrpt::gui::mrptKeyModifier *)) &mrpt::gui::CBaseGUIWindow::waitForKey, "Waits for any key to be pushed on the image or the console, and returns\n the key code.\n  This method remove key strokes previous to its call, so it will always\n wait. To get\n   the latest pushed key, see\n\n \n If set to false, any push of shift, cmd,\n control, etc... will make this method to return.\n \n\n If set to !=nullptr, the modifiers of the key\n stroke will be saved here.\n \n\n The virtual key code, as defined in mrptKeyCode (a replication\n of wxWidgets key codes).\n\n \n getPushedKey, Key codes in the enum mrptKeyCode\n\nC++: mrpt::gui::CBaseGUIWindow::waitForKey(bool, enum mrpt::gui::mrptKeyModifier *) --> int", pybind11::arg("ignoreControlKeys"), pybind11::arg("out_pushModifier"));
		cl.def("keyHit", (bool (mrpt::gui::CBaseGUIWindow::*)() const) &mrpt::gui::CBaseGUIWindow::keyHit, "Returns true if a key has been pushed, without blocking waiting for a\n new key being pushed.\n \n\n waitForKey, clearKeyHitFlag\n\nC++: mrpt::gui::CBaseGUIWindow::keyHit() const --> bool");
		cl.def("clearKeyHitFlag", (void (mrpt::gui::CBaseGUIWindow::*)()) &mrpt::gui::CBaseGUIWindow::clearKeyHitFlag, "Assure that \"keyHit\" will return false until the next pushed key.\n \n\n keyHit, waitForKey\n\nC++: mrpt::gui::CBaseGUIWindow::clearKeyHitFlag() --> void");
		cl.def("getPushedKey", [](mrpt::gui::CBaseGUIWindow &o) -> int { return o.getPushedKey(); }, "");
		cl.def("getPushedKey", (int (mrpt::gui::CBaseGUIWindow::*)(enum mrpt::gui::mrptKeyModifier *)) &mrpt::gui::CBaseGUIWindow::getPushedKey, "Returns the latest pushed key, or 0 if there is no new key stroke.\n \n\n If set to !=nullptr, the modifiers of the key\n stroke will be saved here.\n \n\n The virtual key code, as defined in <mrpt/gui/keycodes.h> (a\n replication of wxWidgets key codes).\n\n \n keyHit, waitForKey\n\nC++: mrpt::gui::CBaseGUIWindow::getPushedKey(enum mrpt::gui::mrptKeyModifier *) --> int", pybind11::arg("out_pushModifier"));
	}
	{ // mrpt::gui::mrptEventWindowChar file:mrpt/gui/CBaseGUIWindow.h line:168
		pybind11::class_<mrpt::gui::mrptEventWindowChar, std::shared_ptr<mrpt::gui::mrptEventWindowChar>, PyCallBack_mrpt_gui_mrptEventWindowChar, mrpt::system::mrptEvent> cl(M("mrpt::gui"), "mrptEventWindowChar", "An event sent by a window upon a char pressed by the user.\n\n  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread,\n    so all your code in the handler must be thread safe.");
		cl.def( pybind11::init<class mrpt::gui::CBaseGUIWindow *, int, enum mrpt::gui::mrptKeyModifier>(), pybind11::arg("obj"), pybind11::arg("_char_code"), pybind11::arg("_key_mod") );

		cl.def_readwrite("char_code", &mrpt::gui::mrptEventWindowChar::char_code);
		cl.def_readwrite("key_modifiers", &mrpt::gui::mrptEventWindowChar::key_modifiers);
		cl.def("assign", (class mrpt::gui::mrptEventWindowChar & (mrpt::gui::mrptEventWindowChar::*)(const class mrpt::gui::mrptEventWindowChar &)) &mrpt::gui::mrptEventWindowChar::operator=, "C++: mrpt::gui::mrptEventWindowChar::operator=(const class mrpt::gui::mrptEventWindowChar &) --> class mrpt::gui::mrptEventWindowChar &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::gui::mrptEventWindowResize file:mrpt/gui/CBaseGUIWindow.h line:194
		pybind11::class_<mrpt::gui::mrptEventWindowResize, std::shared_ptr<mrpt::gui::mrptEventWindowResize>, PyCallBack_mrpt_gui_mrptEventWindowResize, mrpt::system::mrptEvent> cl(M("mrpt::gui"), "mrptEventWindowResize", "An event sent by a window upon resize.\n\n  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread,\n    so all your code in the handler must be thread safe.");
		cl.def( pybind11::init<class mrpt::gui::CBaseGUIWindow *, size_t, size_t>(), pybind11::arg("obj"), pybind11::arg("_new_width"), pybind11::arg("_new_height") );

		cl.def_readwrite("new_width", &mrpt::gui::mrptEventWindowResize::new_width);
		cl.def_readwrite("new_height", &mrpt::gui::mrptEventWindowResize::new_height);
		cl.def("assign", (class mrpt::gui::mrptEventWindowResize & (mrpt::gui::mrptEventWindowResize::*)(const class mrpt::gui::mrptEventWindowResize &)) &mrpt::gui::mrptEventWindowResize::operator=, "C++: mrpt::gui::mrptEventWindowResize::operator=(const class mrpt::gui::mrptEventWindowResize &) --> class mrpt::gui::mrptEventWindowResize &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::gui::mrptEventMouseDown file:mrpt/gui/CBaseGUIWindow.h line:219
		pybind11::class_<mrpt::gui::mrptEventMouseDown, std::shared_ptr<mrpt::gui::mrptEventMouseDown>, PyCallBack_mrpt_gui_mrptEventMouseDown, mrpt::system::mrptEvent> cl(M("mrpt::gui"), "mrptEventMouseDown", "An event sent by a window upon a mouse click, giving the (x,y) pixel\n coordinates.\n\n  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread,\n    so all your code in the handler must be thread safe.\n\n \n mrptEventMouseMove");
		cl.def( pybind11::init<class mrpt::gui::CBaseGUIWindow *, struct mrpt::img::TPixelCoord, bool, bool>(), pybind11::arg("obj"), pybind11::arg("_coords"), pybind11::arg("_leftButton"), pybind11::arg("_rightButton") );

		cl.def_readwrite("coords", &mrpt::gui::mrptEventMouseDown::coords);
		cl.def_readwrite("leftButton", &mrpt::gui::mrptEventMouseDown::leftButton);
		cl.def_readwrite("rightButton", &mrpt::gui::mrptEventMouseDown::rightButton);
		cl.def("assign", (class mrpt::gui::mrptEventMouseDown & (mrpt::gui::mrptEventMouseDown::*)(const class mrpt::gui::mrptEventMouseDown &)) &mrpt::gui::mrptEventMouseDown::operator=, "C++: mrpt::gui::mrptEventMouseDown::operator=(const class mrpt::gui::mrptEventMouseDown &) --> class mrpt::gui::mrptEventMouseDown &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::gui::mrptEventMouseMove file:mrpt/gui/CBaseGUIWindow.h line:244
		pybind11::class_<mrpt::gui::mrptEventMouseMove, std::shared_ptr<mrpt::gui::mrptEventMouseMove>, PyCallBack_mrpt_gui_mrptEventMouseMove, mrpt::system::mrptEvent> cl(M("mrpt::gui"), "mrptEventMouseMove", "An event sent by a window when the mouse is moved over it.\n  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread,\n    so all your code in the handler must be thread safe.\n \n\n mrptEventMouseDown");
		cl.def( pybind11::init<class mrpt::gui::CBaseGUIWindow *, struct mrpt::img::TPixelCoord, bool, bool>(), pybind11::arg("obj"), pybind11::arg("_coords"), pybind11::arg("_leftButton"), pybind11::arg("_rightButton") );

		cl.def_readwrite("coords", &mrpt::gui::mrptEventMouseMove::coords);
		cl.def_readwrite("leftButton", &mrpt::gui::mrptEventMouseMove::leftButton);
		cl.def_readwrite("rightButton", &mrpt::gui::mrptEventMouseMove::rightButton);
		cl.def("assign", (class mrpt::gui::mrptEventMouseMove & (mrpt::gui::mrptEventMouseMove::*)(const class mrpt::gui::mrptEventMouseMove &)) &mrpt::gui::mrptEventMouseMove::operator=, "C++: mrpt::gui::mrptEventMouseMove::operator=(const class mrpt::gui::mrptEventMouseMove &) --> class mrpt::gui::mrptEventMouseMove &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::gui::mrptEventWindowClosed file:mrpt/gui/CBaseGUIWindow.h line:276
		pybind11::class_<mrpt::gui::mrptEventWindowClosed, std::shared_ptr<mrpt::gui::mrptEventWindowClosed>, PyCallBack_mrpt_gui_mrptEventWindowClosed, mrpt::system::mrptEvent> cl(M("mrpt::gui"), "mrptEventWindowClosed", "An event sent by a window upon when it's about to be closed, either\n manually by the user or programmatically.\n   The event field member  is default by default, but can be\n set to false in the event callback\n   to forbid the window to be closed by the user. If the event corresponds to\n a programatic close, this field is ignored.\n\n  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked\n from the wxWidgets internal MRPT thread,\n    so all your code in the handler must be thread safe.\n\n \n CBaseGUIWindow");
		cl.def( pybind11::init( [](class mrpt::gui::CBaseGUIWindow * a0){ return new mrpt::gui::mrptEventWindowClosed(a0); }, [](class mrpt::gui::CBaseGUIWindow * a0){ return new PyCallBack_mrpt_gui_mrptEventWindowClosed(a0); } ), "doc");
		cl.def( pybind11::init<class mrpt::gui::CBaseGUIWindow *, bool>(), pybind11::arg("obj"), pybind11::arg("_allow_close") );

		cl.def_readwrite("allow_close", &mrpt::gui::mrptEventWindowClosed::allow_close);
		cl.def("assign", (class mrpt::gui::mrptEventWindowClosed & (mrpt::gui::mrptEventWindowClosed::*)(const class mrpt::gui::mrptEventWindowClosed &)) &mrpt::gui::mrptEventWindowClosed::operator=, "C++: mrpt::gui::mrptEventWindowClosed::operator=(const class mrpt::gui::mrptEventWindowClosed &) --> class mrpt::gui::mrptEventWindowClosed &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
