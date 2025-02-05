/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hwdrivers/CCameraSensor.h>

namespace mrpt::apps
{
/// \ingroup mrpt_apps_grp

/** Used only from MRPT apps: Use with caution since "panel" MUST be a
 * "mrpt::gui::CPanelCameraSelection *"
 */
mrpt::hwdrivers::CCameraSensor::Ptr prepareVideoSourceFromPanel(void* panel);

/** Parse the user options in the wxWidgets "panel" and write the configuration
 * into the given section of the given configuration file.
 * Use with caution since "panel" MUST be a "mrpt::gui::CPanelCameraSelection
 * *"
 * \sa prepareVideoSourceFromUserSelection, prepareVideoSourceFromPanel,
 * readConfigIntoVideoSourcePanel
 */
void writeConfigFromVideoSourcePanel(
    void* panel,
    const std::string& in_cfgfile_section_name,
    mrpt::config::CConfigFileBase* out_cfgfile);

/** Parse the given section of the given configuration file and set accordingly
 * the controls of the wxWidgets "panel".
 * Use with caution since "panel" MUST be a "mrpt::gui::CPanelCameraSelection
 * *"
 * \sa prepareVideoSourceFromUserSelection, prepareVideoSourceFromPanel,
 * writeConfigFromVideoSourcePanel
 */
void readConfigIntoVideoSourcePanel(
    void* panel,
    const std::string& in_cfgfile_section_name,
    const mrpt::config::CConfigFileBase* in_cfgfile);

/** Show to the user a list of possible camera drivers and creates and open the
 * selected camera.
 */
mrpt::hwdrivers::CCameraSensor::Ptr prepareVideoSourceFromUserSelection();

}  // namespace mrpt::apps