/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CObserver.h>

#include <string>
#include <map>

namespace mrpt::graphslam
{
/**\brief Monitor events in the visualization window.
 *
 * \ingroup mrpt_graphslam_grp
 */
class CWindowObserver : public mrpt::system::CObserver
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
		bool reset_keypresses = true);
	/**\brief Make new keystrokes available in the help message box.
	 *
	 * Classes with access to the CWindowObserver can use this method to add
	 * keystrokes according to their needs
	 */
	void registerKeystroke(
		const std::string key_str, const std::string key_desc);

   protected:
	void OnEvent(const mrpt::system::mrptEvent& e) override;

   private:
	bool m_showing_help = false;
	bool m_hiding_help = false;

	std::string m_help_msg;

	/**\brief Map from registered char_code (std::string to support <C-c>) to
	 * boolean
	 * true/false indicating whether it has been pressed since previous time
	 * checked
	 */
	std::map<std::string, bool> m_key_codes_to_pressed;
	mrpt::system::CTicTac m_tim_show_start, m_tim_show_end;
};
}  // namespace mrpt::graphslam
