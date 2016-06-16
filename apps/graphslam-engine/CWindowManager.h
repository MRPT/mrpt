#ifndef CWINDOWMANAGER_H
#define CWINDOWMANAGER_H

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>

namespace mrpt { namespace gui {

	class CWindowManager_t {
  	public:
    	CWindowManager_t(CDisplayWindow3D* win):
    		m_win(win), // pointer to window to manage
				kOffsetYStep(20.0), // textMessage vertical text position
				kIndexTextStep(1), // textMessage index
				kFontName("sans"),
				kFontSize(12)
  	{
  		this->initCWindowManager();
  	}
    	virtual ~CWindowManager_t() { }
    	void initCWindowManager() {
    		m_curr_offset_y = 30;
    		m_curr_text_index = 1;
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
				m_curr_offset_y += kOffsetYStep;

				*text_index = m_curr_text_index;
				m_curr_text_index += kIndexTextStep;
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

		  	if (m_win) {
					m_win->addTextMessage(x,y,
							text,
							color,
							kFontName, kFontSize,
							mrpt::opengl::NICE,
 							unique_index);
 				}
 			}

  	private:
			CDisplayWindow3D* m_win;

			const double kOffsetYStep;
			const int kIndexTextStep;
			double m_curr_offset_y;
			int m_curr_text_index;

			std::string kFontName;
			int kFontSize;

	};

} } // END OF NAMESPACES

#endif /* end of include guard: CWINDOWMANAGER_H */
