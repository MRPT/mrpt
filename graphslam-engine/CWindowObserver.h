#ifndef CWINDOWOBSERVER_H
#define CWINDOWOBSERVER_H

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/utils/CObserver.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/opengl/gl_utils.h>

#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::utils;


class CWindowObserver : public mrpt::utils::CObserver
{
public:
	CWindowObserver() :
		m_showing_help(false),
		m_hiding_help(false),
		m_mouse_clicked(false),
		m_request_to_exit(false) {

			std::cout << "WindowObserver initialized." << endl;
		}

	const TParameters<bool>* returnEventsStruct() {
		TParameters<bool>* params = new TParameters<bool>;
		(*params)["mouse_clicked"] = m_mouse_clicked;
		(*params)["request_to_exit"] = m_request_to_exit;
		
		return params;
	}

protected:	
	virtual void OnEvent(const mrptEvent &e) {
		if (e.isOfType<mrptEventOnDestroy>()) {
			const mrptEventOnDestroy &ev = static_cast<const mrptEventOnDestroy &>(e);
			std::cout  << "Event received: mrptEventOnDestroy" << endl;
		}
		else if (e.isOfType<mrptEventWindowResize>()) {
			const mrptEventWindowResize &ev = static_cast<const mrptEventWindowResize &>(e);
			std::cout  << "Resize event received from: " << ev.source_object
				<< ", new size: " << ev.new_width << " x " << ev.new_height << endl;
		}
		else if (e.isOfType<mrptEventWindowChar>()) {
			const mrptEventWindowChar &ev = static_cast<const mrptEventWindowChar &>(e);
			std::cout  << "Char event received from: " << ev.source_object
				<< ". Char code: " <<  ev.char_code << " modif: " << ev.key_modifiers << endl;;

			switch (ev.char_code)
			{
				case 'h':
				case 'H':
					if (!m_showing_help) {
						m_showing_help = true;
						cout << "h was pressed!" << endl;
					}
					else {
						m_showing_help = false;
						m_hiding_help = true;
					}
					break;
				case 3: // <C-c>
					if (ev.key_modifiers == 8192) {
						cout << "Pressed C-c! inside CDisplayWindow3D" << endl;
						m_request_to_exit = true;
					}
					break;
				default:
					break;
			}

		} // end of e.isOftype<mrptEventWindowChar>
		else if (e.isOfType<mrptEventWindowClosed>()) {
			const mrptEventWindowClosed &ev = static_cast<const mrptEventWindowClosed &>(e);
			std::cout  << "Window closed event received from: "
				<< ev.source_object<< "\n";
		}
		else if (e.isOfType<mrptEventMouseDown>()) {
			const mrptEventMouseDown &ev = static_cast<const mrptEventMouseDown&>(e);
			m_mouse_clicked = true;

			std::cout  << "Mouse down event received from: "
				<< ev.source_object<< "pt: " <<ev.coords.x << "," << ev.coords.y << "\n";
		}
		else if (e.isOfType<mrptEventGLPostRender>()) {
			/**
			 * An event sent by an mrpt::opengl::COpenGLViewport AFTER calling the
			 * SCENE OPENGL DRAWING PRIMITIVES and before doing a glSwapBuffers. 
			 */

			//cout << "mrpptEventGLPostRender received." << endl;

			mrpt::opengl::gl_utils::renderMessageBox(
				0.70f,  0.05f,  // x,y (in screen "ratios")
				0.25f, 0.09f, // width, height (in screen "ratios")
				"Press 'h' for help",
				0.02f  // text size
				);
			
			// Also showing help?
			if (m_showing_help || m_hiding_help) {
				cout << "In the m_showing_help ... if-clause" << endl;
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
						"User options:\n"
						" - h: Toogle help message\n"
						" - Alt+Enter: Toogle fullscreen\n"
						" - Mouse click: Set camera manually\n"
						" - Ctrl+c: Halt program execution",
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

private:
	bool m_mouse_clicked, m_request_to_exit;

	bool m_showing_help, m_hiding_help;
	CTicTac  m_tim_show_start, m_tim_show_end;
	string m_help_text; 
};

#endif /* end of include guard: CWINDOWOBSERVER_H */
