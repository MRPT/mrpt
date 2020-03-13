/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Implementation file for CWindowObserver class
#include "graphslam-precomp.h"  // Precompiled headers

#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/opengl/COpenGLViewport.h>

using namespace mrpt::graphslam;

CWindowObserver::CWindowObserver()
{
	m_help_msg =
		"User options:\n"
		" - h/H: Toggle help message\n"
		" - Alt+Enter: Toggle fullscreen\n"
		" - Mouse click: Set camera manually\n"
		" - Ctrl+c: Halt program execution";

	// register the default keystrokes
	m_key_codes_to_pressed["h"] = false;
	m_key_codes_to_pressed["Alt+Enter"] = false;
	m_key_codes_to_pressed["Ctrl+c"] = false;
	m_key_codes_to_pressed["mouse_clicked"] = false;
}

void CWindowObserver::returnEventsStruct(
	std::map<std::string, bool>* codes_to_pressed,
	bool reset_keypresses /*= true */)
{
	*codes_to_pressed = m_key_codes_to_pressed;

	// reset the code flags
	if (reset_keypresses)
	{
		for (auto& map_it : m_key_codes_to_pressed)
		{
			map_it.second = false;
		}
	}
}

void CWindowObserver::registerKeystroke(
	const std::string key_str, const std::string key_desc)
{
	m_help_msg += std::string("\n") + " - " + key_str + "/" +
				  mrpt::system::upperCase(key_str) + ": " + key_desc;

	m_key_codes_to_pressed[key_str] = false;
}

void CWindowObserver::OnEvent(const mrpt::system::mrptEvent& e)
{
	if (e.isOfType<mrpt::system::mrptEventOnDestroy>())
	{
		const auto& ev =
			dynamic_cast<const mrpt::system::mrptEventOnDestroy&>(e);
		MRPT_UNUSED_PARAM(ev);
		std::cout << "Event received: mrptEventOnDestroy" << std::endl;
	}
	else if (e.isOfType<mrpt::gui::mrptEventWindowResize>())
	{
		const auto& ev =
			static_cast<const mrpt::gui::mrptEventWindowResize&>(e);
		std::cout << "Resize event received from: " << ev.source_object
				  << ", new size: " << ev.new_width << " x " << ev.new_height
				  << std::endl;
	}
	else if (e.isOfType<mrpt::gui::mrptEventWindowChar>())
	{
		const auto& ev = dynamic_cast<const mrpt::gui::mrptEventWindowChar&>(e);
		std::cout << "Char event received from: " << ev.source_object
				  << ". Char code: " << ev.char_code
				  << " modif: " << ev.key_modifiers << std::endl;
		;

		switch (ev.char_code)
		{
			case 'h':
			case 'H':
				m_key_codes_to_pressed["h"] = true;
				break;
			case 'c':
			case 'C':
				// case 3: // <C-c>
				if (ev.key_modifiers == 8192)
				{
					std::cout << "Pressed C-c inside CDisplayWindow3D"
							  << std::endl;
					m_key_codes_to_pressed["Ctrl+c"] = true;
				}
				break;

			default:
				// just raise the corresponding flag. Let the class which
				// actually
				// cares translate the character to its corresponding meaning.
				// Pressed letter is stored in lower case form only to make it
				// easier
				// to check afterwards
				m_key_codes_to_pressed[mrpt::system::lowerCase(
					std::string(1, ev.char_code))] = true;
				break;
		}

	}  // end of e.isOftype<mrptEventWindowChar>
	else if (e.isOfType<mrpt::gui::mrptEventWindowClosed>())
	{
		const auto& ev =
			dynamic_cast<const mrpt::gui::mrptEventWindowClosed&>(e);
		std::cout << "Window closed event received from: " << ev.source_object
				  << "\n";
	}
	else if (e.isOfType<mrpt::gui::mrptEventMouseDown>())
	{
		const auto& ev = dynamic_cast<const mrpt::gui::mrptEventMouseDown&>(e);
		m_key_codes_to_pressed["mouse_clicked"] = true;

		std::cout << "Mouse down event received from: " << ev.source_object
				  << "pt: " << ev.coords.x << "," << ev.coords.y << "\n";
	}
	else if (e.isOfType<mrpt::opengl::mrptEventGLPostRender>())
	{
		/*
		 * An event sent by an mrpt::opengl::COpenGLViewport AFTER calling
		 * the
		 * SCENE OPENGL DRAWING PRIMITIVES and before doing a glSwapBuffers.
		 */

		// was: show m_help_msg.c_str()
	}
}
