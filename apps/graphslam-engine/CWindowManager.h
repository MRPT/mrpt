#ifndef CWINDOWMANAGER_H
#define CWINDOWMANAGER_H

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include "CWindowObserver.h"

namespace mrpt { namespace gui {

/**
 * Class responsible for keeping ponters to CDisplayWindow3D and CObserver
 * instances.
 */
class CWindowManager_t {
  public:
    CWindowManager_t(CDisplayWindow3D* win_in):
    	win(win_in) // pointer to window to manage
		{
			observer = NULL;
  		this->initCWindowManager();
  	}
    CWindowManager_t(CDisplayWindow3D* win_in, CWindowObserver* observer_in):
    	win(win_in), // pointer to window to manage
    	observer(observer_in) // pointer to CWindowObserver instance
		{
  		this->initCWindowManager();
  	}

    virtual ~CWindowManager_t() { }
    void initCWindowManager() {

    	m_offset_y_step = 20.0;
    	m_index_text_step = 1;
    	m_font_name = "sans";
    	m_font_size = 12;
    	m_curr_offset_y = 30;
    	m_curr_text_index = 1;

    	m_viewp_width = 0.2;
    	m_viewp_height = 0.2;
    	m_viewp_x = 0.75;
    	m_viewp_margin = 0.01;
    	m_viewp_y = 0.72;


    }
		/**
		 	* assignTextMessageParameters
		 	*
		 	* Assign the next available offset_y and text_index for the textMessage
		 	* under construction. Then increment the respective current counters
		 	*/
		inline void assignTextMessageParameters(
				double* offset_y, int* text_index) {

			*offset_y = m_curr_offset_y;
			m_curr_offset_y += m_offset_y_step;

			*text_index = m_curr_text_index;
			m_curr_text_index += m_index_text_step;
		}

		/**
			* Wrapper around the CDisplayWindow3D::addTextMessage method, so that
			* the user does not have to specify the font name and size
			*/
		inline void addTextMessage(
		  	const double x, const double y,
		  	const std::string& text,
		  	const mrpt::utils::TColorf& color=mrpt::utils::TColorf(1.0, 1.0, 1.0),
		  	const size_t unique_index=0) {

		  if (win) {
				win->addTextMessage(x,y,
						text,
						color,
						m_font_name, m_font_size,
						mrpt::opengl::NICE,
 						unique_index);
 			}
 		}

		/**
		 * Assign position and size values for the placement of the next viewport
		 * This way the user doesn't have to decide on the layout of the viewports
		 * in the CDisplayWindow3D.
		 */
 		inline void assignViewportParameters(double *x, double *y, double *width, double *height) {
 			*x = m_viewp_x;
			*y = m_viewp_y;

 			*width = m_viewp_width;
 			*height = m_viewp_height;

 			m_viewp_y -= m_viewp_height + m_viewp_margin;
 		}

		CDisplayWindow3D* win;
		CWindowObserver* observer;
  private:

		double m_offset_y_step;
		int m_index_text_step;
		double m_curr_offset_y;
		int m_curr_text_index;

		std::string m_font_name;
		int m_font_size;

		// viewports configuration
		double m_viewp_width;
		double m_viewp_height;
		double m_viewp_x;
		double m_viewp_y; // vertical layouts of the viewports
		double m_viewp_margin;

};

} } // END OF NAMESPACES

#endif /* end of include guard: CWINDOWMANAGER_H */
