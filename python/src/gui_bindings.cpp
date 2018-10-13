/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::gui;
using namespace mrpt::opengl;

// CDisplayWindow3D
tuple CDisplayWindow3D_waitForKey(
	CDisplayWindow3D& self, bool ignoreControlKeys = true)
{
	list ret_val;
	mrptKeyModifier out_pushModifier = mrptKeyModifier::MRPTKMOD_NONE;
	int key = self.waitForKey(ignoreControlKeys, &out_pushModifier);
	ret_val.append(key);
	ret_val.append(out_pushModifier);
	return tuple(ret_val);
}

tuple CDisplayWindow3D_getPushedKey(CDisplayWindow3D& self)
{
	list ret_val;
	mrptKeyModifier out_pushModifier = mrptKeyModifier::MRPTKMOD_NONE;
	int key = self.getPushedKey(&out_pushModifier);
	ret_val.append(key);
	ret_val.append(out_pushModifier);
	return tuple(ret_val);
}

BOOST_PYTHON_FUNCTION_OVERLOADS(
	CDisplayWindow3D_waitForKey_overloads, CDisplayWindow3D_waitForKey, 1, 2)
// end of CDisplayWindow3D

#define MAKE_ENUM_VALUE(enum_type, enum_name) \
	.value(#enum_name, enum_type::enum_name)

void export_gui()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(gui)

	// mrptKeyCode
	enum_<mrptKeyCode>("mrptKeyCode") MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_BACK) MAKE_ENUM_VALUE(
		mrptKeyCode,
		MRPTK_TAB) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_RETURN) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_ESCAPE)
		MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPACE) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_DELETE) MAKE_ENUM_VALUE(
			mrptKeyCode,
			MRPTK_START) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_LBUTTON) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_RBUTTON)
			MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_MBUTTON) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_CLEAR) MAKE_ENUM_VALUE(
				mrptKeyCode,
				MRPTK_SHIFT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_ALT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_CONTROL)
				MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_MENU) MAKE_ENUM_VALUE(
					mrptKeyCode,
					MRPTK_PAUSE) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_CAPITAL)
					MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_END) MAKE_ENUM_VALUE(
						mrptKeyCode,
						MRPTK_HOME) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_CANCEL)
						MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_LEFT) MAKE_ENUM_VALUE(
							mrptKeyCode,
							MRPTK_UP) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_RIGHT)
							MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_DOWN) MAKE_ENUM_VALUE(
								mrptKeyCode,
								MRPTK_SELECT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_PRINT)
								MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_EXECUTE) MAKE_ENUM_VALUE(
									mrptKeyCode,
									MRPTK_SNAPSHOT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_INSERT)
									MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_HELP) MAKE_ENUM_VALUE(
										mrptKeyCode,
										MRPTK_NUMPAD0) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD1)
										MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD2) MAKE_ENUM_VALUE(
											mrptKeyCode,
											MRPTK_NUMPAD3) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD4)
											MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD5) MAKE_ENUM_VALUE(
												mrptKeyCode,
												MRPTK_NUMPAD6) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD7)
												MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD8) MAKE_ENUM_VALUE(
													mrptKeyCode,
													MRPTK_NUMPAD9) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_MULTIPLY)
													MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_ADD) MAKE_ENUM_VALUE(
														mrptKeyCode,
														MRPTK_SEPARATOR)
														MAKE_ENUM_VALUE(
															mrptKeyCode,
															MRPTK_SUBTRACT)
															MAKE_ENUM_VALUE(
																mrptKeyCode,
																MRPTK_DECIMAL)
																MAKE_ENUM_VALUE(
																	mrptKeyCode, MRPTK_DIVIDE) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F1) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F2) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F3) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F4) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F5) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F6) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F7) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F8) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F9) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F10) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F11) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F12) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F13) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F14) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F15) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F16) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F17) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F18) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F19) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F20) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F21)
																	MAKE_ENUM_VALUE(
																		mrptKeyCode,
																		MRPTK_F22) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F23)
																		MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_F24) MAKE_ENUM_VALUE(
																			mrptKeyCode,
																			MRPTK_NUMLOCK) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SCROLL)
																			MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_PAGEUP) MAKE_ENUM_VALUE(
																				mrptKeyCode,
																				MRPTK_PAGEDOWN)
																				MAKE_ENUM_VALUE(
																					mrptKeyCode,
																					MRPTK_NUMPAD_SPACE)
																					MAKE_ENUM_VALUE(
																						mrptKeyCode,
																						MRPTK_NUMPAD_TAB)
																						MAKE_ENUM_VALUE(
																							mrptKeyCode, MRPTK_NUMPAD_ENTER) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_F1) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_F2) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_F3) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_F4) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_HOME) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_LEFT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_UP) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_RIGHT)
																							MAKE_ENUM_VALUE(
																								mrptKeyCode,
																								MRPTK_NUMPAD_DOWN)
																								MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_PAGEUP) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_PAGEDOWN) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_END) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_BEGIN) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_INSERT) MAKE_ENUM_VALUE(
																									mrptKeyCode, MRPTK_NUMPAD_DELETE) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_EQUAL) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_MULTIPLY) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_ADD) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_SEPARATOR)
																									MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_SUBTRACT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_DECIMAL) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_NUMPAD_DIVIDE) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_WINDOWS_LEFT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_WINDOWS_RIGHT) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_WINDOWS_MENU) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_COMMAND) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL1) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL2) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL3) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL4) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL5) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL6) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL7) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL8) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL9) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL10) MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL11) MAKE_ENUM_VALUE(
																										mrptKeyCode,
																										MRPTK_SPECIAL12)
																										MAKE_ENUM_VALUE(mrptKeyCode, MRPTK_SPECIAL13) MAKE_ENUM_VALUE(
																											mrptKeyCode,
																											MRPTK_SPECIAL14)
																											MAKE_ENUM_VALUE(
																												mrptKeyCode,
																												MRPTK_SPECIAL15)
																												MAKE_ENUM_VALUE(
																													mrptKeyCode,
																													MRPTK_SPECIAL16)
																													MAKE_ENUM_VALUE(
																														mrptKeyCode,
																														MRPTK_SPECIAL17)
																														MAKE_ENUM_VALUE(
																															mrptKeyCode,
																															MRPTK_SPECIAL18)
																															MAKE_ENUM_VALUE(
																																mrptKeyCode,
																																MRPTK_SPECIAL19)
																																MAKE_ENUM_VALUE(
																																	mrptKeyCode,
																																	MRPTK_SPECIAL20);

	// mrptKeyModifier
	enum_<mrptKeyModifier>("mrptKeyModifier")
		MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_NONE)
			MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_ALT) MAKE_ENUM_VALUE(
				mrptKeyModifier, MRPTKMOD_CONTROL)
				MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_ALTGR)
					MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_SHIFT)
						MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_META)
							MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_WIN)
								MAKE_ENUM_VALUE(mrptKeyModifier, MRPTKMOD_CMD);

	// CDisplayWindow3D
	{
		scope s =
			class_<CDisplayWindow3D, boost::noncopyable>(
				"CDisplayWindow3D",
				"A graphical user interface (GUI) for efficiently rendering 3D "
				"scenes in real-time.",
				init<optional<std::string, unsigned int, unsigned int>>(args(
					"windowCaption", "initialWindowWidth",
					"initialWindowHeight")))
				.def(
					"get3DSceneAndLock", &CDisplayWindow3D::get3DSceneAndLock,
					return_value_policy<reference_existing_object>(),
					"Gets a reference to the smart shared pointer that holds "
					"the internal scene (carefuly read introduction in "
					"gui::CDisplayWindow3D before use!) This also locks the "
					"critical section for accesing the scene, thus the window "
					"will not be repainted until it is unlocked.")
				.def(
					"unlockAccess3DScene",
					&CDisplayWindow3D::unlockAccess3DScene,
					"Unlocks the access to the internal 3D scene.")
				.def(
					"forceRepaint", &CDisplayWindow3D::forceRepaint,
					"Repaints the window.")
				.def(
					"repaint", &CDisplayWindow3D::repaint,
					"Repaints the window.")
				.def(
					"updateWindow", &CDisplayWindow3D::updateWindow,
					"Repaints the window.")
				.def(
					"setCameraElevationDeg",
					&CDisplayWindow3D::setCameraElevationDeg, args("deg"),
					"Changes the camera parameters programmatically.")
				.def(
					"setCameraAzimuthDeg",
					&CDisplayWindow3D::setCameraAzimuthDeg, args("deg"),
					"Changes the camera parameters programmatically.")
				.def(
					"setCameraPointingToPoint",
					&CDisplayWindow3D::setCameraPointingToPoint,
					args("x", "y", "z"),
					"Changes the camera parameters programmatically.")
				.def(
					"setCameraZoom", &CDisplayWindow3D::setCameraZoom,
					args("zoom"),
					"Changes the camera parameters programmatically.")
				.def(
					"setCameraProjective",
					&CDisplayWindow3D::setCameraProjective,
					args("isProjective"),
					"Sets the camera as projective, or orthogonal.")
				.def(
					"isOpen", &CDisplayWindow3D::isOpen,
					"Returns false if the user has closed the window.")
				.def(
					"waitForKey", &CDisplayWindow3D_waitForKey,
					CDisplayWindow3D_waitForKey_overloads())
				.def(
					"keyHit", &CDisplayWindow3D::keyHit,
					"Returns true if a key has been pushed, without blocking "
					"waiting for a new key being pushed.")
				.def(
					"clearKeyHitFlag", &CDisplayWindow3D::clearKeyHitFlag,
					"Assure that \"keyHit\" will return false until the next "
					"pushed key.")
				.def(
					"getPushedKey", &CDisplayWindow3D_getPushedKey,
					"Returns the latest pushed key, or 0 if there is no new "
					"key stroke.")
				.def(
					"getRenderingFPS", &CDisplayWindow3D::getRenderingFPS,
					"Get the average Frames Per Second (FPS) value from the "
					"last 250 rendering events.");
	}
}
