/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Implementation file for CWindowObserver class
#include "graphslam-precomp.h"  // Precompiled headers

#include <mrpt/opengl/COpenGLViewport.h>
#include <mrpt/graphslam/misc/CWindowObserver.h>

using namespace mrpt::graphslam;

CWindowObserver::CWindowObserver() :
	m_showing_help(false),
	m_hiding_help(false)
{
	help_msg = "User options:\n"
		" - h/H: Toggle help message\n"
		" - Alt+Enter: Toggle fullscreen\n"
		" - Mouse click: Set camera manually\n"
		" - Ctrl+c: Halt program execution";

	// register the default keystrokes
	m_key_codes_to_pressed["h"] = false;
	m_key_codes_to_pressed["Alt+Enter"] = false;
	m_key_codes_to_pressed["Ctrl+c"] = false;
	m_key_codes_to_pressed["mouse_clicked"] = false;

	std::cout << "WindowObserver initialized." << std::endl;
}


void CWindowObserver::returnEventsStruct(
		std::map<std::string, bool>* codes_to_pressed,
		bool reset_keypresses /*= true */) {
	*codes_to_pressed = m_key_codes_to_pressed;

	// reset the code flags
	if (reset_keypresses) {
		for ( std::map<std::string, bool>::iterator map_it = 
				m_key_codes_to_pressed.begin(); map_it != m_key_codes_to_pressed.end();
				++map_it ) {
			map_it->second = false;
		}
}
}

void CWindowObserver::registerKeystroke(const std::string key_str, const std::string key_desc) {
	help_msg += std::string("\n") + " - " + 
		key_str + "/" + mrpt::system::upperCase(key_str) +
		": " + key_desc;

	m_key_codes_to_pressed[key_str] = false;
}

void CWindowObserver::OnEvent(const mrpt::utils::mrptEvent &e) {
	if (e.isOfType<mrpt::utils::mrptEventOnDestroy>()) {
		const mrpt::utils::mrptEventOnDestroy &ev = static_cast<const mrpt::utils::mrptEventOnDestroy &>(e);
		MRPT_UNUSED_PARAM(ev);
		std::cout  << "Event received: mrptEventOnDestroy" << std::endl;
	}
	else if (e.isOfType<mrpt::gui::mrptEventWindowResize>()) {
		const mrpt::gui::mrptEventWindowResize &ev = static_cast<const mrpt::gui::mrptEventWindowResize &>(e);
		std::cout  << "Resize event received from: " << ev.source_object
			<< ", new size: " << ev.new_width << " x " << ev.new_height << std::endl;
	}
	else if (e.isOfType<mrpt::gui::mrptEventWindowChar>()) {
		const mrpt::gui::mrptEventWindowChar &ev = static_cast<const mrpt::gui::mrptEventWindowChar &>(e);
		std::cout  << "Char event received from: " << ev.source_object
			<< ". Char code: " <<  ev.char_code << " modif: " << ev.key_modifiers << std::endl;;

		switch (ev.char_code)
		{
			case 'h':
			case 'H':
				if (!m_showing_help) {
					m_showing_help = true;
					std::cout << "h/H was pressed!" << std::endl;
				}
				else {
					m_showing_help = false;
					m_hiding_help = true;
				}
				m_key_codes_to_pressed["h"] = true;
				break;
			case 3: // <C-c>
				if (ev.key_modifiers == 8192) {
					std::cout << "Pressed C-c inside CDisplayWindow3D" << std::endl;
					m_key_codes_to_pressed["Ctrl+c"] = true;
				}
				break;

			default:
				// just raise the corresponding flag. Let the class which actually
				// cares translate the character to its corresponding meaning.
				// Pressed letter is stored in lower case form only to make it easier
				// to check afterwards
				m_key_codes_to_pressed[mrpt::system::lowerCase(std::string(1, ev.char_code))] = true;
				break;
		}

	} // end of e.isOftype<mrptEventWindowChar>
	else if (e.isOfType<mrpt::gui::mrptEventWindowClosed>()) {
		const mrpt::gui::mrptEventWindowClosed &ev = static_cast<const mrpt::gui::mrptEventWindowClosed &>(e);
		std::cout  << "Window closed event received from: "
			<< ev.source_object<< "\n";
	}
	else if (e.isOfType<mrpt::gui::mrptEventMouseDown>()) {
		const mrpt::gui::mrptEventMouseDown &ev = static_cast<const mrpt::gui::mrptEventMouseDown&>(e);
		m_key_codes_to_pressed["mouse_clicked"] = true;

		std::cout  << "Mouse down event received from: "
			<< ev.source_object<< "pt: " <<ev.coords.x << "," << ev.coords.y << "\n";
	}
	else if (e.isOfType<mrpt::opengl::mrptEventGLPostRender>()) {
		/*
			* An event sent by an mrpt::opengl::COpenGLViewport AFTER calling the
			* SCENE OPENGL DRAWING PRIMITIVES and before doing a glSwapBuffers.
			*/

		//std::cout << "mrpt::opengl::mrpptEventGLPostRender received." << std::endl;

		mrpt::opengl::gl_utils::renderMessageBox(
			0.70f,  0.05f,  // x,y (in screen "ratios")
			0.25f, 0.09f, // width, height (in screen "ratios")
			"Press 'h' for help",
			0.02f  // text size
			);

		// Also showing help?
		if (m_showing_help || m_hiding_help) {
			//std::cout << "In the m_showing_help ... if-clause" << std::endl;
			static const double TRANSP_ANIMATION_TIME_SEC = 0.5;

			const double show_tim = m_tim_show_start.Tac();
			const double hide_tim = m_tim_show_end.Tac();

			const double tranparency = m_hiding_help ?
				1.0-std::min(1.0,hide_tim/TRANSP_ANIMATION_TIME_SEC)
				:
				std::min(1.0,show_tim/TRANSP_ANIMATION_TIME_SEC);

			mrpt::opengl::gl_utils::renderMessageBox(
					0.25f,  0.25f,  // x,y (in screen "ratios")
					0.50f, 0.50f, // width, height (in screen "ratios")
					help_msg.c_str(),
					0.02f,  // text size
					mrpt::utils::TColor(190,190,190, 200*tranparency),   // background
					mrpt::utils::TColor(0,0,0, 200*tranparency),  // border
					mrpt::utils::TColor(200,0,0, 150*tranparency), // text
					6.0f, // border width
					"serif", // text font
					mrpt::opengl::NICE // text style
					);

			if (hide_tim > TRANSP_ANIMATION_TIME_SEC && m_hiding_help) {
				m_hiding_help = false;
			}
		}

	}
	else {
		//std::cout  << "Unregistered mrptEvent received\n";
	}
}


