/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/COutputLogger.h>

#include "CWindowObserver.h"

namespace mrpt::graphslam
{
/**\brief Class acts as a container for storing pointers to
 * mrpt::gui::CDisplayWindow3D,
 * mrpt::graphslam::CWindowObserver instances.
 *
 * CWindowManager also provides methods for adding/positioning textMessages
 * and viewports in the CDisplayWindow in a compact and consistent way.
 *
 * \ingroup mrpt_graphslam_grp
 */
class CWindowManager : public mrpt::system::COutputLogger
{
   public:
	/**\brief Default class constructor */
	CWindowManager() { this->initCWindowManager(); }
	/**\brief Class constructor*/
	CWindowManager(mrpt::gui::CDisplayWindow3D* win_in)
	{
		this->initCWindowManager();
		this->setCDisplayWindow3DPtr(win_in);
	}
	/**\brief Class constructor*/
	CWindowManager(
		mrpt::gui::CDisplayWindow3D* win_in,
		mrpt::graphslam::CWindowObserver* observer_in)
	{
		this->initCWindowManager();

		this->setWindowObserverPtr(observer_in);
		this->setCDisplayWindow3DPtr(win_in);
	}
	/**\brief Class destructor. */
	~CWindowManager() override = default;
	/**\brief Store the CDisplayWindow3D pointer in the CWindowManager
	 * instance.
	 *
	 * \sa setWindowObserverPtr
	 */
	void setCDisplayWindow3DPtr(mrpt::gui::CDisplayWindow3D* win_in)
	{
		m_fetched_displaywindow_pointer = true;
		win = win_in;

		MRPT_LOG_DEBUG_STREAM("Fetched the CDisplayWindow3D* successfully");
	}
	/**\brief Store the CWindowObserver pointer in the CWindowManager instance.
	 *
	 * \sa setCDisplayWindow3DPtr
	 */
	void setWindowObserverPtr(mrpt::graphslam::CWindowObserver* obsever_in)
	{
		m_fetched_observer_pointer = true;
		observer = obsever_in;

		MRPT_LOG_DEBUG_STREAM("Fetched the CWindowObserver* successfully");
	}

	/**\brief Assign the next available offset_y and text_index for the
	 * textMessage under construction.
	 *
	 * Used for consistent positioning of textMessages in the
	 * CDisplayWindow3D.
	 *
	 * \sa assignViewPortParameters
	 */
	void assignTextMessageParameters(double* offset_y, int* text_index)
	{
		*offset_y = m_curr_offset_y;
		m_curr_offset_y += m_offset_y_step;

		*text_index = m_curr_text_index;
		m_curr_text_index += m_index_text_step;
	}

	/**\brief Wrapper call around the CWindowDisplay3D */
	bool isOpen()
	{
		if (win)
		{
			return win->isOpen();
		}
		else
		{
			return false;
		}
	}

	/**\brief Wrapper around the CDisplayWindow3D::addTextMessage method, so
	 * that the
	 * user does not have to specify the font name and size.
	 *
	 * \note see initCWindowManager method for the default fontName and
	 * fontSize used.
	 */
	void addTextMessage(
		const double x, const double y, const std::string& text,
		const mrpt::img::TColorf& color = mrpt::img::TColorf(1.0, 1.0, 1.0),
		const size_t unique_index = 0)
	{
		if (!win)
		{
			return;
		}

		win->addTextMessage(
			x, y, text, color, m_font_name, m_font_size, mrpt::opengl::NICE,
			unique_index);
	}

	/**\brief Assign position and size values for the placement of the next
	 * viewport
	 *
	 * Used for consistent positioning of the Viewports in the CDisplayWindow3D
	 *
	 * \sa assignTextMessageParameters
	 */
	void assignViewportParameters(
		double* x, double* y, double* width, double* height)
	{
		*x = m_viewp_x;
		*y = m_viewp_y;

		*width = m_viewp_width;
		*height = m_viewp_height;

		m_viewp_y -= m_viewp_height + m_viewp_margin;
	}

	mrpt::gui::CDisplayWindow3D* win; /**< CDisplayWindow instance */
	mrpt::graphslam::CWindowObserver* observer; /**< CWindowObserver instance */
   private:
	/**\brief Initialization method, to be called from the various Constructors.
	 */
	void initCWindowManager()
	{
		m_fetched_displaywindow_pointer = false;
		m_fetched_observer_pointer = false;
		win = nullptr;
		observer = nullptr;

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

		// loger related directives
		this->setLoggerName("CWindowManager");
	}

	bool m_fetched_observer_pointer;
	bool m_fetched_displaywindow_pointer;

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
	double m_viewp_y; /**< vertical layout of the viewports */
	double m_viewp_margin;
};
}  // namespace mrpt::graphslam
