/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/nav.h>
#include <mrpt/system/CTicTac.h>

class NavlogViewerApp
{
   public:
	NavlogViewerApp();
	~NavlogViewerApp() = default;

	void loadLogfile(const std::string& filName);

   private:
	// =========== UI controls ==============
	mrpt::gui::CDisplayWindowGUI::Ptr m_win;

	nanogui::Window* m_winMain = nullptr;
	nanogui::TabWidget* m_tabWidget = nullptr;
	nanogui::TextBox* txtLogEntries = nullptr;
	nanogui::TextBox* txtLogDuration = nullptr;
	nanogui::TextBox* edShapeMinDist = nullptr;
	nanogui::Slider* slidLog = nullptr;
	nanogui::Button* m_btnPlay = nullptr;
	nanogui::Button* m_btnStop = nullptr;
	nanogui::TextBox* txtSelectedPTG = nullptr;
	nanogui::Label* m_txtTimeIndex = nullptr;
	nanogui::CheckBox* m_cbUseOdometryCoords = nullptr;
	nanogui::CheckBox* m_cbGlobalFrame = nullptr;
	nanogui::CheckBox* m_cbShowDelays = nullptr;
	nanogui::CheckBox* m_cbDrawShape = nullptr;
	nanogui::CheckBox* m_cbShowAllDebugFields = nullptr;
	nanogui::CheckBox* m_cbShowCursor = nullptr;
	nanogui::CheckBox* m_ClearanceOverPath = nullptr;
	nanogui::ComboBox* m_rbPerPTGPlots = nullptr;

	// ============ app data ===============
	std::string m_openedFileName = "UNNAMED";

	std::vector<mrpt::serialization::CSerializable::Ptr> m_logdata;
	// Retrieved from the first entry in m_logdata when loading
	std::vector<mrpt::nav::CParameterizedTrajectoryGenerator::Ptr>
		m_logdata_ptg_paths;

	struct VizPTG
	{
		nanogui::Window* win = nullptr;
		mrpt::gui::MRPT2NanoguiGLCanvas* glCanvas = nullptr;
	};

	std::map<std::string, VizPTG> m_mywins3D;

	mrpt::system::TTimeStamp m_log_first_tim = INVALID_TIMESTAMP;
	mrpt::system::TTimeStamp m_log_last_tim = INVALID_TIMESTAMP;

	void updateInfoFromLoadedLog();

   private:
	void OnbtnLoadClick();
	void OnslidLogCmdScroll();
	void OnmnuMatlabPlotsSelected();
	void OnmnuSeePTGParamsSelected();
	void OnmnuSaveScoreMatrixSelected();
	void OntimMouseXY();
	void OnmnuMatlabExportPaths();
	void OnmnuSaveCurrentObstacles();
	void OnMainIdleLoop();

	mrpt::system::CTicTac m_autoPlayTimer;
	bool m_autoPlayEnabled = false;
	double m_autoPlayInterval = 100e-3;

	bool m_showCursorXY = false;
};
