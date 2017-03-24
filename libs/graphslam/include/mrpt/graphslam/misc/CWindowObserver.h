/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CWINDOWOBSERVER_H
#define CWINDOWOBSERVER_H

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/utils/CObserver.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/system/string_utils.h>

#include <mrpt/graphslam/link_pragmas.h>

#include <iostream>
#include <string>
#include <map>

namespace mrpt { namespace graphslam {

/**\brief Monitor events in the visualization window.
 *
 * \ingroup mrpt_graphslam_grp
 */
class GRAPHSLAM_IMPEXP CWindowObserver : public mrpt::utils::CObserver
{
public:
	CWindowObserver();
	/**\brief Return a map of key code to a boolean indicating whether it was
	 * pressed since the previous time the class was quarried.
	 *
	 * \note By default the codes are reset every time the method is invoked,
	 * unless <em>reset_keypresses<\em> is set to False
	 */
	void returnEventsStruct(
			std::map<std::string, bool>* codes_to_pressed,
			bool reset_keypresses=true);
	/**\brief Make new keystrokes available in the help message box.
	 *
	 * Classes with access to the CWindowObserver can use this method to add
	 * keystrokes according to their needs
	 */
	void registerKeystroke(const std::string key_str, const std::string key_desc);

protected:
	void OnEvent(const mrpt::utils::mrptEvent &e);

private:
	bool m_showing_help, m_hiding_help;

	std::string help_msg;

	/**\brief Map from registered char_code (std::string to support <C-c>) to boolean
	 * true/false indicating whether it has been pressed since previous time
	 * checked
	 */
	std::map<std::string, bool> m_key_codes_to_pressed;
	mrpt::utils::CTicTac  m_tim_show_start, m_tim_show_end;
	std::string m_help_text;
};

} } // END OF NAMESPACES

#endif /* end of include guard: CWINDOWOBSERVER_H */
