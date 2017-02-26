/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_KEYCODES_H
#define  MRPT_KEYCODES_H

#include <mrpt/config.h>

namespace mrpt
{
namespace gui
{
	// These key codes are an exact replication of those of wxWidgets.
	//  They are defined within MRPT for convenience, since users don't have to install
	//  wxWidgets in order to build MRPT applications.
	//  See: http://docs.wxwidgets.org/stable/wx_keycodes.html

/*  Virtual keycodes */
enum mrptKeyCode
{
    MRPTK_BACK    =    8,
    MRPTK_TAB     =    9,
    MRPTK_RETURN  =    13,
    MRPTK_ESCAPE  =    27,
    MRPTK_SPACE   =    32,
    MRPTK_DELETE  =    127,

    MRPTK_START   = 300,
    MRPTK_LBUTTON,
    MRPTK_RBUTTON,
    MRPTK_CANCEL,
    MRPTK_MBUTTON,
    MRPTK_CLEAR,
    MRPTK_SHIFT,
    MRPTK_ALT,
    MRPTK_CONTROL,
    MRPTK_MENU,
    MRPTK_PAUSE,
    MRPTK_CAPITAL,
    MRPTK_END,
    MRPTK_HOME,
    MRPTK_LEFT,
    MRPTK_UP,
    MRPTK_RIGHT,
    MRPTK_DOWN,
    MRPTK_SELECT,
    MRPTK_PRINT,
    MRPTK_EXECUTE,
    MRPTK_SNAPSHOT,
    MRPTK_INSERT,
    MRPTK_HELP,
    MRPTK_NUMPAD0,
    MRPTK_NUMPAD1,
    MRPTK_NUMPAD2,
    MRPTK_NUMPAD3,
    MRPTK_NUMPAD4,
    MRPTK_NUMPAD5,
    MRPTK_NUMPAD6,
    MRPTK_NUMPAD7,
    MRPTK_NUMPAD8,
    MRPTK_NUMPAD9,
    MRPTK_MULTIPLY,
    MRPTK_ADD,
    MRPTK_SEPARATOR,
    MRPTK_SUBTRACT,
    MRPTK_DECIMAL,
    MRPTK_DIVIDE,
    MRPTK_F1,
    MRPTK_F2,
    MRPTK_F3,
    MRPTK_F4,
    MRPTK_F5,
    MRPTK_F6,
    MRPTK_F7,
    MRPTK_F8,
    MRPTK_F9,
    MRPTK_F10,
    MRPTK_F11,
    MRPTK_F12,
    MRPTK_F13,
    MRPTK_F14,
    MRPTK_F15,
    MRPTK_F16,
    MRPTK_F17,
    MRPTK_F18,
    MRPTK_F19,
    MRPTK_F20,
    MRPTK_F21,
    MRPTK_F22,
    MRPTK_F23,
    MRPTK_F24,
    MRPTK_NUMLOCK,
    MRPTK_SCROLL,
    MRPTK_PAGEUP,
    MRPTK_PAGEDOWN,

    MRPTK_NUMPAD_SPACE,
    MRPTK_NUMPAD_TAB,
    MRPTK_NUMPAD_ENTER,
    MRPTK_NUMPAD_F1,
    MRPTK_NUMPAD_F2,
    MRPTK_NUMPAD_F3,
    MRPTK_NUMPAD_F4,
    MRPTK_NUMPAD_HOME,
    MRPTK_NUMPAD_LEFT,
    MRPTK_NUMPAD_UP,
    MRPTK_NUMPAD_RIGHT,
    MRPTK_NUMPAD_DOWN,
    MRPTK_NUMPAD_PAGEUP,
    MRPTK_NUMPAD_PAGEDOWN,

    MRPTK_NUMPAD_END,
    MRPTK_NUMPAD_BEGIN,
    MRPTK_NUMPAD_INSERT,
    MRPTK_NUMPAD_DELETE,
    MRPTK_NUMPAD_EQUAL,
    MRPTK_NUMPAD_MULTIPLY,
    MRPTK_NUMPAD_ADD,
    MRPTK_NUMPAD_SEPARATOR,
    MRPTK_NUMPAD_SUBTRACT,
    MRPTK_NUMPAD_DECIMAL,
    MRPTK_NUMPAD_DIVIDE,

    MRPTK_WINDOWS_LEFT,
    MRPTK_WINDOWS_RIGHT,
    MRPTK_WINDOWS_MENU ,
    MRPTK_COMMAND,

    /* Hardware-specific buttons */
    MRPTK_SPECIAL1 = 193,
    MRPTK_SPECIAL2,
    MRPTK_SPECIAL3,
    MRPTK_SPECIAL4,
    MRPTK_SPECIAL5,
    MRPTK_SPECIAL6,
    MRPTK_SPECIAL7,
    MRPTK_SPECIAL8,
    MRPTK_SPECIAL9,
    MRPTK_SPECIAL10,
    MRPTK_SPECIAL11,
    MRPTK_SPECIAL12,
    MRPTK_SPECIAL13,
    MRPTK_SPECIAL14,
    MRPTK_SPECIAL15,
    MRPTK_SPECIAL16,
    MRPTK_SPECIAL17,
    MRPTK_SPECIAL18,
    MRPTK_SPECIAL19,
    MRPTK_SPECIAL20
};

/* This enum contains bit mask constants used in wxKeyEvent */
enum mrptKeyModifier
{
    MRPTKMOD_NONE      = 0x0000,
    MRPTKMOD_ALT       = 0x1000,
    MRPTKMOD_CONTROL   = 0x2000,
    MRPTKMOD_ALTGR     = MRPTKMOD_ALT | MRPTKMOD_CONTROL,
    MRPTKMOD_SHIFT     = 0x4000,
    MRPTKMOD_META      = 0x8000,
    MRPTKMOD_WIN       = MRPTKMOD_META,
#ifdef MRPT_OS_APPLE
    MRPTKMOD_CMD       = MRPTKMOD_META
#else
    MRPTKMOD_CMD       = MRPTKMOD_CONTROL
#endif
};

} // End of namespace
} // End of namespace
#endif
