/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "navlog-viewer-ui.h"

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/containers/printf_vector.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/gui/about_box.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>	 // intersect()
#include <mrpt/math/utils.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/rtti/CListOfClasses.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>  // replace()
#include <fstream>

// Configure look:
static const mrpt::opengl::TFontParams& getFontParams()
{
	static mrpt::opengl::TFontParams fp;
	static bool init = false;
	if (!init)
	{
		fp.vfont_style = mrpt::opengl::FILL;
		fp.vfont_name = "mono";
		fp.vfont_scale = 10.0f;	 // pixels
		fp.draw_shadow = true;
	}
	return fp;
}

// Font size & line spaces for GUI-overlayed text lines
static const float Ay = getFontParams().vfont_scale + 3;
#define ADD_WIN_TEXTMSG_COL(__MSG, __COL)                                      \
	{                                                                          \
		auto fp = getFontParams();                                             \
		fp.color = __COL;                                                      \
		m_win->background_scene->getViewport()->addTextMessage(                \
			5.0, 5 + (lineY++) * Ay, __MSG, unique_id++, fp);                  \
	}

#define ADD_WIN_TEXTMSG(__MSG)                                                 \
	ADD_WIN_TEXTMSG_COL(__MSG, mrpt::img::TColorf(1, 1, 1))

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::serialization;
using namespace mrpt::config;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::nav;

NavlogViewerApp::NavlogViewerApp()
{
	// Create main window:
	mrpt::gui::CDisplayWindowGUI_Params cp;
	cp.maximized = true;

	// Create GUI:
	m_win = mrpt::gui::CDisplayWindowGUI::Create("navlog-viewer", 900, 700, cp);

	// Add main control window:
	// -----------------------------
	m_winMain = m_win->createManagedSubWindow("Control");
	m_winMain->setPosition(nanogui::Vector2i(10, 10));
	m_winMain->setLayout(new nanogui::GroupLayout(5 /*margin*/, 5 /*spacing*/));
	m_winMain->setFixedWidth(540);

	{
		nanogui::Widget* panel = m_winMain->add<nanogui::Widget>();
		panel->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Fill,
			5 /*margin*/));

		panel->add<nanogui::Button>("Load...", ENTYPO_ICON_UPLOAD)
			->setCallback([this]() { this->OnbtnLoadClick(); });

		m_btnPlay =
			panel->add<nanogui::Button>("Play", ENTYPO_ICON_CONTROLLER_PLAY);
		m_btnStop =
			panel->add<nanogui::Button>("Stop", ENTYPO_ICON_CONTROLLER_STOP);

		panel->add<nanogui::Label>(" Animation delay (s):");
		nanogui::TextBox* edPlayInterval =
			panel->add<nanogui::TextBox>("0.100");
		edPlayInterval->setEditable(true);
		edPlayInterval->setFormat("[0-9.e+\\-]*");

		m_btnPlay->setCallback([edPlayInterval, this]() {
			// Rewind if we are at the end:
			if (std::abs(slidLog->value() - slidLog->range().second) < 1e-6)
				slidLog->setValue(0);

			m_autoPlayEnabled = true;
			m_btnPlay->setEnabled(false);
			m_btnStop->setEnabled(true);
			m_btnPlay->setPushed(true);
			m_btnStop->setPushed(false);
			m_autoPlayInterval = std::stod(edPlayInterval->value());
		});
		m_btnStop->setCallback([this]() {
			m_autoPlayEnabled = false;
			m_btnPlay->setEnabled(true);
			m_btnStop->setEnabled(false);
			m_btnPlay->setPushed(false);
			m_btnStop->setPushed(false);
		});

		m_btnStop->setEnabled(false);

		panel->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_RIGHT)
			->setCallback([this]() { m_win->setVisible(false); });
	}

	// ===== Time slider:
	m_txtTimeIndex = m_winMain->add<nanogui::Label>("Time index:");
	slidLog = m_winMain->add<nanogui::Slider>();
	slidLog->setCallback([this](float /*v*/) { OnslidLogCmdScroll(); });

	m_tabWidget = m_winMain->add<nanogui::TabWidget>();

	// ===== TAB: Information
	{
		nanogui::Widget* layer = m_tabWidget->createTab("Information");
		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 5);
		layer->setLayout(layout);

		layer->add<nanogui::Label>("Log entries:");
		txtLogEntries = layer->add<nanogui::TextBox>("  ");

		layer->add<nanogui::Label>("Log duration:");
		txtLogDuration = layer->add<nanogui::TextBox>("  ");

		layer->add<nanogui::Label>("Selected PTG index:");
		txtSelectedPTG = layer->add<nanogui::TextBox>("       ");

		layer->add<nanogui::Label>(" ");  // spacer
	}

	// ===== TAB: View options
	{
		nanogui::Widget* layer = m_tabWidget->createTab("View options");
		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 5);
		layer->setLayout(layout);

		m_cbUseOdometryCoords =
			layer->add<nanogui::CheckBox>("Use raw odometry");
		m_cbUseOdometryCoords->setCallback(
			[this](bool) { OnslidLogCmdScroll(); });

		m_cbGlobalFrame =
			layer->add<nanogui::CheckBox>("In global coordinates");
		m_cbGlobalFrame->setChecked(true);
		m_cbGlobalFrame->setCallback([this](bool) { OnslidLogCmdScroll(); });

		m_cbShowDelays =
			layer->add<nanogui::CheckBox>("Show delays model-based poses");
		m_cbShowDelays->setChecked(true);
		m_cbShowDelays->setCallback([this](bool) { OnslidLogCmdScroll(); });

		m_cbDrawShape =
			layer->add<nanogui::CheckBox>("Draw robot shape on trajectories");
		m_cbDrawShape->setChecked(true);
		m_cbDrawShape->setCallback([this](bool) { OnslidLogCmdScroll(); });

		m_cbShowAllDebugFields =
			layer->add<nanogui::CheckBox>("Show all debug fields");
		m_cbShowAllDebugFields->setCallback(
			[this](bool) { OnslidLogCmdScroll(); });

		m_cbShowCursor = layer->add<nanogui::CheckBox>("Mouse coordinates");
		m_cbShowCursor->setChecked(m_showCursorXY);
		m_cbShowCursor->setCallback([this](bool v) { m_showCursorXY = v; });

		m_ClearanceOverPath =
			layer->add<nanogui::CheckBox>("Clearance: path/!pointwise");
		m_ClearanceOverPath->setCallback(
			[this](bool) { OnslidLogCmdScroll(); });

		{
			auto* panel = layer->add<nanogui::Widget>();
			panel->setLayout(new nanogui::BoxLayout(
				nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 5));
			panel->add<nanogui::Label>("Min. dist. robot shapes:");
			edShapeMinDist = panel->add<nanogui::TextBox>("1.0");
			edShapeMinDist->setEditable(true);
			edShapeMinDist->setFormat("[0-9.]*");
		}

		layer->add<nanogui::Label>("Show for each PTG:");
		const auto lst = std::vector<std::string>(
			{"TP-Obstacles only", "+ final scores", "+ preliminary scores"});
		m_rbPerPTGPlots = layer->add<nanogui::ComboBox>(lst, lst);
		m_rbPerPTGPlots->setCallback([this](int) { OnslidLogCmdScroll(); });
		m_rbPerPTGPlots->setSelectedIndex(2);
	}

	// ===== TAB: Advanced
	{
		nanogui::Widget* layer = m_tabWidget->createTab("Advanced");
		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 10);
		layer->setLayout(layout);

		auto lambdaAddSmallFontBtn =
			[](nanogui::Widget* parent, const std::string& caption,
			   const std::function<void(void)>& callback, int icon = 0) {
				auto btn = parent->add<nanogui::Button>(caption, icon);
				btn->setCallback(callback);
				btn->setFontSize(16);
			};

		lambdaAddSmallFontBtn(layer, "See PTGs details", [this]() {
			OnmnuSeePTGParamsSelected();
		});
		lambdaAddSmallFontBtn(layer, "Save current obstacle map...", [this]() {
			OnmnuSaveCurrentObstacles();
		});
		lambdaAddSmallFontBtn(layer, "Export map plot to MATLAB...", [this]() {
			OnmnuMatlabPlotsSelected();
		});
		lambdaAddSmallFontBtn(
			layer, "Export paths info to MATLAB...",
			[this]() { OnmnuMatlabExportPaths(); });
		lambdaAddSmallFontBtn(layer, "Save score matrices...", [this]() {
			OnmnuSaveScoreMatrixSelected();
		});
	}

	m_tabWidget->setActiveTab(0);

	// Add a background scene:
	// -----------------------------
	{
		mrpt::opengl::COpenGLScene::Ptr scene =
			mrpt::opengl::COpenGLScene::Create();

		m_win->camera().setAzimuthDegrees(-90.0f);
		m_win->camera().setElevationDegrees(90.0f);
		m_win->camera().setZoomDistance(25.0f);

		// XY ground plane:
		mrpt::opengl::CGridPlaneXY::Ptr gl_grid =
			mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1, 0.75f);
		gl_grid->setColor_u8(mrpt::img::TColor(0xa0a0a0, 0x90));
		scene->insert(gl_grid);

		// XYZ corner at origin:
		scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 2.0));

		auto lck = mrpt::lockHelper(m_win->background_scene_mtx);
		m_win->background_scene = std::move(scene);
	}

	// Setup idle loop code:
	// -----------------------------
	m_win->setLoopCallback([this]() { OnMainIdleLoop(); });
	m_win->setKeyboardCallback(
		[this](int key, int scancode, int action, int modifiers) {
			return OnKeyboardCallback(key, scancode, action, modifiers);
		});

	// Update view and show window:
	// -----------------------------
	m_win->performLayout();

	m_win->drawAll();
	m_win->setVisible(true);

	// Disable some controls, etc..
	updateInfoFromLoadedLog();
}

void NavlogViewerApp::OnbtnLoadClick()
{
	NANOGUI_START_TRY

	const std::string fileName = nanogui::file_dialog(
		{{"reactivenavlog", "Navigation logs"}, {"navlog", "Navigation logs"}},
		false /*load*/);

	if (fileName.empty()) return;

	loadLogfile(fileName);

	NANOGUI_END_TRY(*m_win)
}

void NavlogViewerApp::loadLogfile(const std::string& fileName)
{
	using namespace std::string_literals;

	NANOGUI_START_TRY
	m_openedFileName = fileName;
	m_winMain->setTitle(
		"Control ["s + mrpt::system::extractFileName(fileName) + "]"s);

	mrpt::io::CFileGZInputStream f(fileName);

	m_logdata.clear();
	m_logdata_ptg_paths.clear();

	mrpt::rtti::CListOfClasses validClasses;
	validClasses.insert(CLASS_ID(mrpt::nav::CLogFileRecord));

	m_log_first_tim = INVALID_TIMESTAMP;
	m_log_last_tim = INVALID_TIMESTAMP;

	auto arch = archiveFrom(f);
	for (;;)
	{
		try
		{
			CSerializable::Ptr obj = arch.ReadObject();
			if (!validClasses.contains(obj->GetRuntimeClass()))
			{
				// auto dlg =
				new nanogui::MessageDialog(
					m_win.get(), nanogui::MessageDialog::Type::Warning,
					"Error loading navlog file",
					format(
						"Unexpected class found: %s",
						obj->GetRuntimeClass()->className));
				break;
			}
			m_logdata.push_back(obj);

			// generate time stats:
			if (IS_CLASS(*obj, CLogFileRecord))
			{
				const CLogFileRecord::Ptr logptr =
					std::dynamic_pointer_cast<CLogFileRecord>(obj);
				const auto it = logptr->timestamps.find("tim_start_iteration");
				if (it != logptr->timestamps.end()) m_log_last_tim = it->second;

				if (!logptr->infoPerPTG.empty())
				{
					size_t nPTGs = logptr->infoPerPTG.size();
					if (nPTGs > m_logdata_ptg_paths.size())
					{
						m_logdata_ptg_paths.resize(nPTGs);
						for (size_t i = 0; i < nPTGs; i++)
							if (logptr->infoPerPTG[i].ptg)
								m_logdata_ptg_paths[i] =
									logptr->infoPerPTG[i].ptg;
					}
				}
			}

			if (m_log_first_tim == INVALID_TIMESTAMP &&
				m_log_last_tim != INVALID_TIMESTAMP)
				m_log_first_tim = m_log_last_tim;
		}
		catch (CExceptionEOF&)
		{
			break;
		}
		catch (const std::exception& e)
		{
			// EOF in the middle of an object... It may be usual if the logger
			// is shut down not cleanly.
			// auto dlg =
			new nanogui::MessageDialog(
				m_win.get(), nanogui::MessageDialog::Type::Warning,
				"Loading ended with an exception", mrpt::exception_to_str(e));

			break;
		}
	}

	// Update stats, etc...
	updateInfoFromLoadedLog();

	NANOGUI_END_TRY(*m_win)
}

void NavlogViewerApp::OnMainIdleLoop()
{
	// Idle loop tasks:

	// Auto play:
	if (m_autoPlayEnabled && m_autoPlayTimer.Tac() >= m_autoPlayInterval)
	{
		m_autoPlayTimer.Tic();

		float p = slidLog->value();
		if ((p + 1) <= slidLog->range().second)
		{
			slidLog->setValue(p + 1);
			OnslidLogCmdScroll();
		}
		else
		{
			m_btnStop->callback()();
		}
	}

	if (m_showCursorXY) { OntimMouseXY(); }
}

bool NavlogViewerApp::OnKeyboardCallback(
	int key, [[maybe_unused]] int scancode, int action,
	[[maybe_unused]] int modifiers)
{
	if (action != GLFW_PRESS && action != GLFW_REPEAT) return false;

	switch (key)
	{
		case GLFW_KEY_LEFT:
		{
			float p = slidLog->value();
			if ((p - 1) >= slidLog->range().first)
			{
				slidLog->setValue(p - 1);
				OnslidLogCmdScroll();
			}
		}
		break;
		case GLFW_KEY_RIGHT:
		{
			float p = slidLog->value();
			if ((p + 1) <= slidLog->range().second)
			{
				slidLog->setValue(p + 1);
				OnslidLogCmdScroll();
			}
		}
		break;
	};

	return false;
}

void NavlogViewerApp::updateInfoFromLoadedLog()
{
	const size_t N = m_logdata.size();

	if (N > 0)
	{
		this->txtLogEntries->setValue(std::to_string(N));
		this->slidLog->setRange({0, N - 1});
		this->slidLog->setValue(0);
		OnslidLogCmdScroll();
	}

	std::string sDuration("???");
	if (m_log_first_tim != INVALID_TIMESTAMP &&
		m_log_last_tim != INVALID_TIMESTAMP)
	{
		sDuration = mrpt::system::intervalFormat(
			mrpt::system::timeDifference(m_log_first_tim, m_log_last_tim));
	}
	txtLogDuration->setValue(sDuration);

	// m_win->performLayout();
}

// ---------------------------------------------
// 				DRAW ONE LOG RECORD
// ---------------------------------------------
void NavlogViewerApp::OnslidLogCmdScroll()
{
	NANOGUI_START_TRY

	using namespace std::string_literals;

	const int log_idx = mrpt::round(slidLog->value());
	if (log_idx >= int(m_logdata.size())) return;
	// In the future, we could handle more log classes. For now, only
	// "CLogFileRecord::Ptr":
	auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
	const CLogFileRecord& log = *logptr;

	txtSelectedPTG->setValue(
		std::to_string(log.nSelectedPTG) + "from [0-"s +
		std::to_string(log.nPTGs - 1) + "]"s);

	m_txtTimeIndex->setCaption(
		"Time index: "s + std::to_string(log_idx) + " / "s +
		std::to_string(std::max<size_t>(1U, m_logdata.size()) - 1));

	const bool is_NOP_cmd = log.ptg_index_NOP >= 0;
	const int sel_ptg_idx = !is_NOP_cmd ? log.nSelectedPTG : log.ptg_index_NOP;

	// Draw WS-obstacles
	// --------------------------------
	// Update 3D view:
	{
		auto lck = mrpt::lockHelper(m_win->background_scene_mtx);
		mrpt::opengl::COpenGLScene::Ptr& scene = m_win->background_scene;

		const CVectorFloat shap_x = log.robotShape_x, shap_y = log.robotShape_y;

		// Robot frame of reference:
		mrpt::opengl::CSetOfObjects::Ptr gl_robot_frame;
		{
			mrpt::opengl::CRenderizable::Ptr gl_rbframe_r =
				scene->getByName("robot_frame");  // Get or create if new
			if (!gl_rbframe_r)
			{
				gl_robot_frame = mrpt::opengl::CSetOfObjects::Create();
				gl_robot_frame->setName("robot_frame");
				scene->insert(gl_robot_frame);
			}
			else
			{
				gl_robot_frame =
					std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
						gl_rbframe_r);
			}

			// Global or local coordinates?
			if (m_cbUseOdometryCoords->checked())
			{
				// Use odometry pose increment wrt initial instant,
				// taking the localization-based "good" pose of the initial
				// instant as a reference, such that the obtained
				// coordinates closely match those expected in the "map"
				// frame:
				auto log0ptr =
					mrpt::ptr_cast<CLogFileRecord>::from(m_logdata[0]);
				ASSERT_(log0ptr.get());
				const auto curPose = log0ptr->robotPoseLocalization +
					(log.robotPoseOdometry - log0ptr->robotPoseOdometry);

				gl_robot_frame->setPose(curPose);
			}
			else if (m_cbGlobalFrame->checked())
			{
				gl_robot_frame->setPose(mrpt::poses::CPose3D(
					mrpt::poses::CPose2D(log.robotPoseLocalization)));
				// Move the window focus:
				auto& cam = m_win->camera();
				const float px = cam.getCameraPointingX();
				const float py = cam.getCameraPointingY();
				const float cam_zoom = cam.getZoomDistance();
				if ((mrpt::math::TPoint2D(log.robotPoseLocalization) -
					 mrpt::math::TPoint2D(px, py))
						.norm() > .3 * cam_zoom)
					cam.setCameraPointing(
						log.robotPoseLocalization.x,
						log.robotPoseLocalization.y, 0.0);
			}
			else
			{
				gl_robot_frame->setPose(mrpt::poses::CPose3D());
			}
		}

		// Extrapolated poses from delay models:
		{
			mrpt::opengl::CSetOfObjects::Ptr gl_relposes;
			mrpt::opengl::CRenderizable::Ptr gl_relposes_r =
				gl_robot_frame->getByName("relposes");	// Get or create if new
			if (!gl_relposes_r)
			{
				gl_relposes = mrpt::opengl::CSetOfObjects::Create();
				gl_relposes->setName("relposes");
				gl_robot_frame->insert(gl_relposes);
			}
			else
			{
				gl_relposes =
					std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
						gl_relposes_r);
			}

			gl_relposes->clear();
			if (m_cbShowDelays->checked())
			{
				{
					mrpt::opengl::CSetOfObjects::Ptr gl_relpose_sense =
						mrpt::opengl::stock_objects::CornerXYSimple(0.3f, 1);
					gl_relpose_sense->setName("sense");
					gl_relpose_sense->enableShowName(true);
					gl_relpose_sense->setPose(log.relPoseSense);
					gl_relposes->insert(gl_relpose_sense);
				}
				{
					mrpt::opengl::CSetOfObjects::Ptr gl_relpose_cmdvel =
						mrpt::opengl::stock_objects::CornerXYSimple(0.3f, 1);
					gl_relpose_cmdvel->setName("cmdVel");
					gl_relpose_cmdvel->enableShowName(true);
					gl_relpose_cmdvel->setPose(log.relPoseVelCmd);
					gl_relposes->insert(gl_relpose_cmdvel);
				}
			}
		}

		{
			// Obstacles: original
			mrpt::opengl::CPointCloud::Ptr gl_obs;
			mrpt::opengl::CRenderizable::Ptr gl_obs_r =
				gl_robot_frame->getByName("obs-raw");  // Get or create if new
			if (!gl_obs_r)
			{
				gl_obs = mrpt::opengl::CPointCloud::Create();
				gl_obs->setName("obs-raw");
				gl_obs->setPointSize(3);
				gl_obs->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0x00));
				gl_robot_frame->insert(gl_obs);
			}
			else
			{
				gl_obs =
					dynamic_pointer_cast<mrpt::opengl::CPointCloud>(gl_obs_r);
			}
			gl_obs->loadFromPointsMap(&log.WS_Obstacles_original);
			if (m_cbShowDelays->checked()) gl_obs->setPose(log.relPoseSense);
			else
				gl_obs->setPose(mrpt::poses::CPose3D());
		}
		{
			// Obstacles: after optional filtering
			mrpt::opengl::CPointCloud::Ptr gl_obs;
			mrpt::opengl::CRenderizable::Ptr gl_obs_r =
				gl_robot_frame->getByName("obs");  // Get or create if new
			if (!gl_obs_r)
			{
				gl_obs = mrpt::opengl::CPointCloud::Create();
				gl_obs->setName("obs");
				gl_obs->setPointSize(1.5);
				gl_obs->setLocation(0, 0, 0.001);
				gl_obs->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff));
				gl_robot_frame->insert(gl_obs);
			}
			else
			{
				gl_obs = std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
					gl_obs_r);
			}
			gl_obs->loadFromPointsMap(&log.WS_Obstacles);
			if (m_cbShowDelays->checked()) gl_obs->setPose(log.relPoseSense);
			else
				gl_obs->setPose(mrpt::poses::CPose3D());
		}

		{
			// Selected PTG path:
			mrpt::opengl::CSetOfLines::Ptr gl_path;
			mrpt::opengl::CRenderizable::Ptr gl_path_r =
				gl_robot_frame->getByName("path");	// Get or create if new
			if (!gl_path_r)
			{
				gl_path = mrpt::opengl::CSetOfLines::Create();
				gl_path->setName("path");
				gl_path->setLineWidth(2.0);
				gl_path->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff));
				gl_robot_frame->insert(gl_path);
			}
			else
				gl_path =
					mrpt::ptr_cast<mrpt::opengl::CSetOfLines>::from(gl_path_r);
			gl_path->clear();
			if (sel_ptg_idx < int(m_logdata_ptg_paths.size()) &&
				sel_ptg_idx >= 0)
			{
				mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
					m_logdata_ptg_paths[sel_ptg_idx];
				if (ptg)
				{
					if (!ptg->isInitialized()) ptg->initialize();

					// Set instantaneous dyn state:
					ptg->updateNavDynamicState(
						is_NOP_cmd ? log.ptg_last_navDynState
								   : log.navDynState);

					// Draw path:
					const int selected_k = log.ptg_index_NOP < 0
						? ptg->alpha2index(
							  log.infoPerPTG[sel_ptg_idx].desiredDirection)
						: log.ptg_last_k_NOP;
					float max_dist = ptg->getRefDistance();
					ptg->add_robotShape_to_setOfLines(*gl_path);

					ptg->renderPathAsSimpleLine(
						selected_k, *gl_path, 0.10, max_dist);
					gl_path->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00));

					// PTG origin:
					// enable delays model?
					mrpt::math::TPose2D ptg_origin = (m_cbShowDelays->checked())
						? log.relPoseVelCmd
						: mrpt::math::TPose2D(0, 0, 0);

					// "NOP cmd" case:
					if (log.ptg_index_NOP >= 0)
					{
						ptg_origin =
							ptg_origin - log.rel_cur_pose_wrt_last_vel_cmd_NOP;
					}

					gl_path->setPose(ptg_origin);

					// Overlay a sequence of robot shapes:
					if (m_cbDrawShape->checked())
					{
						double min_shape_dists =
							std::max(0.01, std::stod(edShapeMinDist->value()));

						for (double d = min_shape_dists; d < max_dist;
							 d += min_shape_dists)
						{
							uint32_t step;
							if (!ptg->getPathStepForDist(selected_k, d, step))
								continue;
							mrpt::math::TPose2D p;
							ptg->getPathPose(selected_k, step, p);
							ptg->add_robotShape_to_setOfLines(
								*gl_path, mrpt::poses::CPose2D(p));
						}
					}
					{
						// Robot shape:
						mrpt::opengl::CSetOfLines::Ptr gl_shape;
						mrpt::opengl::CRenderizable::Ptr gl_shape_r =
							gl_robot_frame->getByName(
								"shape");  // Get or create if new
						if (!gl_shape_r)
						{
							gl_shape =
								std::make_shared<mrpt::opengl::CSetOfLines>();
							gl_shape->setName("shape");
							gl_shape->setLineWidth(4.0);
							gl_shape->setColor_u8(
								mrpt::img::TColor(0xff, 0x00, 0x00));
							gl_robot_frame->insert(gl_shape);
						}
						else
						{
							gl_shape = std::dynamic_pointer_cast<
								mrpt::opengl::CSetOfLines>(gl_shape_r);
						}
						gl_shape->clear();
						ptg->add_robotShape_to_setOfLines(*gl_shape);
					}
					{
						mrpt::opengl::CSetOfLines::Ptr gl_shape;
						mrpt::opengl::CRenderizable::Ptr gl_shape_r =
							gl_robot_frame->getByName(
								"velocity");  // Get or create if new
						if (!gl_shape_r)
						{
							gl_shape =
								std::make_shared<mrpt::opengl::CSetOfLines>();
							gl_shape->setName("velocity");
							gl_shape->setLineWidth(4.0);
							gl_shape->setColor_u8(
								mrpt::img::TColor(0x00, 0xff, 0xff));
							gl_robot_frame->insert(gl_shape);
						}
						else
						{
							gl_shape = std::dynamic_pointer_cast<
								mrpt::opengl::CSetOfLines>(gl_shape_r);
						}
						gl_shape->clear();
						const mrpt::math::TTwist2D& velLocal =
							log.cur_vel_local;
						gl_shape->appendLine(
							0, 0, 0, velLocal.vx, velLocal.vy, 0);
					}
				}
			}
		}
		{
			// Target:
			mrpt::opengl::CPointCloud::Ptr gl_trg;
			mrpt::opengl::CRenderizable::Ptr gl_trg_r =
				gl_robot_frame->getByName("target");  // Get or create if new
			if (!gl_trg_r)
			{
				gl_trg = mrpt::opengl::CPointCloud::Create();
				gl_trg->setName("target");
				gl_trg->enableShowName(true);
				gl_trg->setPointSize(9.0);
				gl_trg->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0x00));
				gl_robot_frame->insert(gl_trg);
			}
			else
			{
				gl_trg = std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
					gl_trg_r);
			}
			// Move the map & add a point at (0,0,0) so the name label
			// appears at the target:
			gl_trg->clear();
			if (!log.WS_targets_relative.empty())
			{
				const auto t0 = log.WS_targets_relative[0];
				const float tz = .05f;
				gl_trg->setLocation(t0.x, t0.y, tz);
				for (const auto& t : log.WS_targets_relative)
				{
					gl_trg->insertPoint(t.x - t0.x, t.y - t0.y, tz);
				}
			}
		}
	}

	// Show extra info as text msgs:
	// ---------------------------------
	int lineY = 0, unique_id = 0;
	m_win->background_scene->getViewport()->clearTextMessages();

	// Mouse position at Z=0
	// Updated in timer callback:
	if (m_cbShowCursor->checked())
	{
		lineY++;
		unique_id++;
	}

	if (m_cbShowAllDebugFields->checked())
	{
		for (const auto& e : log.timestamps)
			ADD_WIN_TEXTMSG(mrpt::format(
				"Timestamp %-20s=%s", e.first.c_str(),
				mrpt::system::dateTimeLocalToString(e.second).c_str()));
	}

	{
		const unsigned int nObsOrg = log.WS_Obstacles_original.size(),
						   nObs = log.WS_Obstacles.size();
		const unsigned int nObsFiltered = nObsOrg - nObs;

		if (nObsFiltered)
		{
			ADD_WIN_TEXTMSG(mrpt::format(
				"Obstacle points=%u (%u filtered out)", nObs, nObsFiltered));
		}
		else
		{
			ADD_WIN_TEXTMSG(mrpt::format("Obstacle points=%u", nObs));
		}
	}

	ADD_WIN_TEXTMSG(mrpt::format(
		"cmd_vel=%s",
		log.cmd_vel ? log.cmd_vel->asString().c_str()
					: "NOP (Continue last PTG)"));

	ADD_WIN_TEXTMSG(mrpt::format(
		"cur_vel      =[%.02f m/s, %0.2f m/s, %.02f dps]", log.cur_vel.vx,
		log.cur_vel.vy, mrpt::RAD2DEG(log.cur_vel.omega)));
	ADD_WIN_TEXTMSG(mrpt::format(
		"cur_vel_local=[%.02f m/s, %0.2f m/s, %.02f dps]", log.cur_vel_local.vx,
		log.cur_vel_local.vy, mrpt::RAD2DEG(log.cur_vel_local.omega)));

	ADD_WIN_TEXTMSG(mrpt::format(
		"robot_pose=%s", log.robotPoseLocalization.asString().c_str()));
	{
		for (unsigned int i = 0; i < log.WS_targets_relative.size(); i++)
		{
			ADD_WIN_TEXTMSG(mrpt::format(
				"rel_target[%u]=%s", i,
				log.WS_targets_relative[i].asString().c_str()));
		}
	}

	if (log.cmd_vel_original)
	{
		std::stringstream ss;
		ss << "original cmd_vel: ";
		ss << log.cmd_vel_original->asString();
		ADD_WIN_TEXTMSG(ss.str());
	}

	{
		std::stringstream ss;
		ss << "Performance: ";
		for (size_t i = 0; i < log.infoPerPTG.size(); i++)
			ss << "PTG#" << i
			   << mrpt::format(
					  " TPObs:%ss HoloNav:%ss |",
					  mrpt::system::unitsFormat(
						  log.infoPerPTG[i].timeForTPObsTransformation)
						  .c_str(),
					  mrpt::system::unitsFormat(
						  log.infoPerPTG[i].timeForHolonomicMethod)
						  .c_str());
		ADD_WIN_TEXTMSG(ss.str());
	}

	for (unsigned int nPTG = 0; nPTG < log.infoPerPTG.size(); nPTG++)
	{
		const CLogFileRecord::TInfoPerPTG& pI = log.infoPerPTG[nPTG];

		mrpt::img::TColorf col;
		if (((int)nPTG) == log.nSelectedPTG) col = mrpt::img::TColorf(1, 1, 1);
		else
			col = mrpt::img::TColorf(.8f, .8f, .8f);

		std::string sFactors;
		for (const auto& kv : pI.evalFactors)
		{
			sFactors += kv.first;
			sFactors += ": ";
			sFactors += std::to_string(kv.second);
			sFactors += " ";
		}

		ADD_WIN_TEXTMSG_COL(
			mrpt::format(
				"PTG#%u: SelDir=%+7.01f deg SelSpeed=%.03f Eval=%5.03f. %s",
				nPTG, mrpt::RAD2DEG(pI.desiredDirection), pI.desiredSpeed,
				pI.evaluation, sFactors.c_str()),
			col);
	}

	ADD_WIN_TEXTMSG(mrpt::format(
		"relPoseSense: %s relPoseVelCmd:%s",
		log.relPoseSense.asString().c_str(),
		log.relPoseVelCmd.asString().c_str()));

	if (m_cbShowAllDebugFields->checked())
	{
		for (const auto& e : log.values)
			ADD_WIN_TEXTMSG(format(
				"%-30s=%s ", e.first.c_str(),
				mrpt::system::unitsFormat(e.second, 3, false).c_str()));

		for (const auto& e : log.additional_debug_msgs)
			ADD_WIN_TEXTMSG(
				format("%-30s=%s ", e.first.c_str(), e.second.c_str()));
	}

	// Draw TP-obstacles
	// --------------------------------
	// log.infoPerPTG.size() may be != nPTGs in the last entry is used for
	// "NOP cmdvel"

	bool mustPerformLayout = false;

	for (unsigned int nPTG = 0; nPTG < log.infoPerPTG.size(); nPTG++)
	{
		const bool is_selected_ptg = (int(nPTG) == log.nSelectedPTG);
		VizPTG& viz = m_mywins3D[format("PTG%u", nPTG)];
		if (!viz.win)
		{
			mustPerformLayout = true;

			const static int W = 270;
			const static int H = 270;

			viz.win =
				m_win->createManagedSubWindow(format("%u|TP-Obstacles", nPTG));
			viz.win->setLayout(new nanogui::GroupLayout(0, 0, 0, 0));

			viz.win->setSize({W, H});
			viz.win->setPosition({5 + W * (nPTG % 3), 315 + H * (nPTG / 3)});

			viz.glCanvas = viz.win->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
			viz.glCanvas->setFixedSize({W, H});

			viz.win->buttonPanel()
				->add<nanogui::Button>("", ENTYPO_ICON_PLUS)
				->setCallback([this, viz]() {
					// viz.win->setFixedSize({W, H});
					viz.glCanvas->setFixedSize({W, H});
					m_win->performLayout();
				});
			viz.win->buttonPanel()
				->add<nanogui::Button>("", ENTYPO_ICON_POPUP)
				->setCallback([this, viz]() {
					// viz.win->setFixedSize({2 * W, 2 * H});
					viz.glCanvas->setFixedSize({2 * W, 2 * H});
					m_win->performLayout();
				});

			viz.glCanvas->scene = mrpt::opengl::COpenGLScene::Create();
			auto& scene = viz.glCanvas->scene;

			// win
			viz.glCanvas->scene->getViewport()->addTextMessage(
				4, 4,
				format("[%u]:%s", nPTG, log.infoPerPTG[nPTG].PTG_desc.c_str()),
				0 /*id*/, getFontParams());

			scene->insert(mrpt::opengl::CGridPlaneXY::Create(
				-1.0f, 1.0f, -1.0f, 1.0f, .0f, 1.0f));
			scene->insert(
				mrpt::opengl::stock_objects::CornerXYSimple(0.4f, 2.0f));

			auto& cam = viz.glCanvas->camera();
			cam.setAzimuthDegrees(-90.0f);
			cam.setElevationDegrees(90.0f);
			cam.setZoomDistance(4.6f);
			cam.setCameraProjective(false);

			{
				auto gl_obj = mrpt::opengl::CDisk::Create();
				gl_obj->setDiskRadius(1.01f, 1.0);
				gl_obj->setSlicesCount(30);
				gl_obj->setColor_u8(mrpt::img::TColor(0x30, 0x30, 0x30, 0xff));
				scene->insert(gl_obj);
			}
			{
				auto gl_obj = mrpt::opengl::CSetOfLines::Create();
				gl_obj->setName("tp_obstacles");
				gl_obj->setLineWidth(1.0f);
				gl_obj->setVerticesPointSize(4.0f);
				gl_obj->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff, 0xff));
				scene->insert(gl_obj);
			}
			{
				auto gl_obj = mrpt::opengl::CSetOfLines::Create();
				gl_obj->setName("score_phase1");
				gl_obj->setLineWidth(1.0f);
				gl_obj->setVerticesPointSize(2.0f);
				gl_obj->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0x00, 0xff));
				scene->insert(gl_obj);
			}
			{
				auto gl_obj = mrpt::opengl::CSetOfLines::Create();
				gl_obj->setName("score_phase2");
				gl_obj->setLineWidth(1.0f);
				gl_obj->setVerticesPointSize(2.0f);
				gl_obj->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0xff, 0xff));
				scene->insert(gl_obj);
			}
			{
				auto gl_obj = mrpt::opengl::CSetOfLines::Create();
				gl_obj->setName("tp_selected_dir");
				gl_obj->setLineWidth(3.0f);
				gl_obj->setColor_u8(mrpt::img::TColor(0x00, 0xff, 0x00, 0xd0));
				scene->insert(gl_obj);
			}
			{
				auto gl_obj = mrpt::opengl::CPointCloud::Create();
				gl_obj->setName("tp_target");
				gl_obj->setPointSize(5.0f);
				gl_obj->setColor_u8(mrpt::img::TColor(0x30, 0x30, 0x30, 0xff));
				gl_obj->setLocation(0, 0, 0.02f);
				scene->insert(gl_obj);
			}
			{
				auto gl_obj = mrpt::opengl::CPointCloud::Create();
				gl_obj->setName("tp_robot");
				gl_obj->setPointSize(4.0f);
				gl_obj->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00, 0xa0));
				gl_obj->setLocation(0, 0, 0.02f);
				scene->insert(gl_obj);
			}
#if 0  // should be re-done with CMesh3D?
			{
				auto gl_obj =
					mrpt::opengl::CMesh::Create(true /*transparency*/);
				gl_obj->setName("tp_clearance");
				gl_obj->setScale(1.0f, 1.0f, 5.0f);
				scene->insert(gl_obj);
			}
#endif
		}

		mrpt::opengl::COpenGLScene::Ptr& scene = viz.glCanvas->scene;
		auto view = scene->getViewport();

		// Draw dynamic stuff:
		const CLogFileRecord::TInfoPerPTG& pI = log.infoPerPTG[nPTG];
		vector<float> xs, ys;

		const size_t nAlphas = pI.TP_Obstacles.size();

		view->clearTextMessages();

		auto fp = getFontParams();
		fp.color = is_selected_ptg ? TColorf(1.0f, 1.0f, 0.f)
								   : TColorf(1.0f, 1.0f, 1.0f);
		view->addTextMessage(
			4, 4,
			format("[%u]:%s", nPTG, log.infoPerPTG[nPTG].PTG_desc.c_str()),
			0 /*id*/, fp);

		// Chosen direction:
		{
			const double aDir = pI.desiredDirection;

			auto gl_obj = std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
				scene->getByName("tp_selected_dir"));
			gl_obj->clear();
			gl_obj->appendLine(
				0, 0, 0, pI.desiredSpeed * cos(aDir),
				pI.desiredSpeed * sin(aDir), 0);
		}

		// obstacles:
		xs.clear();
		ys.clear();
		xs.reserve(nAlphas);
		ys.reserve(nAlphas);
		for (size_t i = 0; i < nAlphas; ++i)
		{
			const double a = -M_PI + (i + 0.5) * 2 * M_PI / double(nAlphas);
			const double r = pI.TP_Obstacles[i];
			xs.push_back(r * cos(a));
			ys.push_back(r * sin(a));
		}
		{
			auto gl_obj = std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
				scene->getByName("tp_obstacles"));
			gl_obj->clear();
			if (nAlphas > 2)
			{
				gl_obj->appendLine(xs[0], ys[0], 0, xs[1], ys[1], 0);
				for (size_t i = 2; i < nAlphas; i++)
					gl_obj->appendLineStrip(xs[i], ys[i], 0);
			}
		}

		// Target:
		{
			auto gl_obj = std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
				scene->getByName("tp_target"));
			gl_obj->clear();
			for (const auto& p : pI.TP_Targets)
			{
				gl_obj->insertPoint(p.x, p.y, .0);
			}

			if (!pI.TP_Targets.empty())
			{
				double ang = ::atan2(pI.TP_Targets[0].y, pI.TP_Targets[0].x);
				int tp_target_k = 0;
				if (sel_ptg_idx < int(m_logdata_ptg_paths.size()) &&
					sel_ptg_idx >= 0)
				{
					mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
						m_logdata_ptg_paths[sel_ptg_idx];
					if (ptg) { tp_target_k = ptg->alpha2index(ang); }
				}

				auto fp2 = getFontParams();
				fp2.draw_shadow = false;
				view->addTextMessage(
					4, -12,
					format(
						"TP_Target[0]=(%.02f,%.02f) k=%i ang=%.02f deg",
						pI.TP_Targets[0].x, pI.TP_Targets[0].y, tp_target_k,
						mrpt::RAD2DEG(ang)),
					1 /*id*/, fp2);
			}
		}
		if (m_cbShowAllDebugFields->checked())
		{
			unsigned int ptg_unique_id = 2;
			int ptg_lineY = 1;
			for (const auto& e : pI.evalFactors)
			{
				view->addTextMessage(
					4, 5 + (ptg_lineY++) * (getFontParams().vfont_scale + 3),
					mrpt::format("%20s=%6.03f", e.first.c_str(), e.second),
					ptg_unique_id++, getFontParams());
			}
		}

		// Current robot pt (normally in pure reactive, at (0,0)):
		{
			auto gl_obj = std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
				scene->getByName("tp_robot"));
			gl_obj->clear();
			gl_obj->insertPoint(pI.TP_Robot.x, pI.TP_Robot.y, 0);
		}

		// Clearance-diagram:
#if 0
		{
			auto gl_obj = std::dynamic_pointer_cast<mrpt::opengl::CMesh>(
				scene->getByName("tp_clearance"));
			if (pI.clearance.empty())
				gl_obj->setVisibility(false);
			else
			{
				gl_obj->setVisibility(true);
				pI.clearance.renderAs3DObject(
					*gl_obj, -1.0, 1.0, -1.0, 1.0, 0.15,
					m_ClearanceOverPath->checked() /*interp over path*/);
			}
		}
#endif
		// Scores
		{
			auto gl_obj1 = std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
				scene->getByName("score_phase1"));
			auto gl_obj2 = std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
				scene->getByName("score_phase2"));
			const bool visible1 = m_rbPerPTGPlots->selectedIndex() >= 1;
			const bool visible2 = m_rbPerPTGPlots->selectedIndex() >= 2;
			gl_obj1->clear();
			gl_obj2->clear();
			gl_obj1->setVisibility(visible1);
			gl_obj2->setVisibility(visible2);

			if ((visible1 || visible2) && pI.HLFR && nAlphas > 2)
			{
				const auto& dir_evals = pI.HLFR->dirs_eval;
				const bool has_scores =
					!dir_evals.empty() && dir_evals[0].size() == nAlphas;
				const size_t num_scores = dir_evals.size();

				for (size_t iScore = 0; has_scores && iScore < num_scores;
					 iScore++)
				{
					vector<mrpt::math::TPoint2D> pts;
					pts.reserve(nAlphas);
					for (size_t i = 0; i < nAlphas; ++i)
					{
						const double a =
							-M_PI + (i + 0.5) * 2 * M_PI / double(nAlphas);
						const double r = dir_evals[iScore][i];
						pts.emplace_back(r * cos(a), r * sin(a));
					}

					mrpt::opengl::CSetOfLines::Ptr& gl_obj =
						iScore == (num_scores - 1) ? gl_obj1 : gl_obj2;

					gl_obj->appendLine(
						pts[0].x, pts[0].y, 0, pts[1].x, pts[1].y, 0);
					for (size_t i = 2; i < nAlphas; i++)
						gl_obj->appendLineStrip(pts[i].x, pts[i].y, 0);
				}
			}
		}
	}  // end for each PTG

	if (mustPerformLayout) m_win->performLayout();

	NANOGUI_END_TRY(*m_win)
}

// ------------------------------------------------------------------------
//     Generate a MATLAB script that draws the overall navigation log
// ------------------------------------------------------------------------
void NavlogViewerApp::OnmnuMatlabPlotsSelected()
{
	NANOGUI_START_TRY

	const std::string fileName =
		nanogui::file_dialog({{"m", "MATLAB/Octave script"}}, true /*save*/);

	if (fileName.empty()) return;

	ofstream f(fileName);
	if (!f.is_open()) throw runtime_error("Error writing to file!");

	f << "% Script for drawing navigation log\n"
	  << "% Generated automatically by navlog-viewer - MRPT "
	  << mrpt::system::MRPT_getVersion() << "\n"
	  << "%  From log: " << m_openedFileName << "\n"
	  << "% "
		 "-------------------------------------------------------------"
		 "--------"
		 "----\n\n";

	f << "%%\n"
	  << "function [] = main()\n"
	  << "figure;"
	  << "title('Path for " << mrpt::system::extractFileName(fileName) << "');"
	  << "% Robot shape: (x,y) in each line\n"
	  << "rs = [-0.3 -0.3;0.6 -0.3;0.6 0.3;-0.3 0.3];\n"
	  << "dec_shps = 15;"
	  << "dec=0;";

	const int DECIMATE_POINTS = 10;
	int decim_point_cnt = 0;

	std::vector<float> X, Y;  // Obstacles
	std::vector<float> TX, TY;	// Target over time

	const size_t N = m_logdata.size();
	for (size_t i = 0; i < N; i++)
	{
		const CLogFileRecord::Ptr logsptr =
			std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
		const CLogFileRecord* logptr = logsptr.get();

		const auto robotPose = logptr->robotPoseLocalization;
		CPose2D observationBasePose = CPose2D(robotPose);

		if (m_cbShowDelays->checked())
			observationBasePose =
				observationBasePose + CPose2D(logptr->relPoseSense);

		f << format(
			"dec=dec+1; if (dec>=dec_shps); drawRobotShape(rs,[%f %f "
			"%f]); "
			"dec=0; end\n",
			robotPose.x, robotPose.y, robotPose.phi);

		if (++decim_point_cnt >= DECIMATE_POINTS)
		{
			CSimplePointsMap pts;
			pts.changeCoordinatesReference(
				logptr->WS_Obstacles, CPose3D(observationBasePose));

			const auto& pX = pts.getPointsBufferRef_x();
			const auto& pY = pts.getPointsBufferRef_y();

			X.insert(X.begin(), pX.begin(), pX.end());
			Y.insert(Y.begin(), pY.begin(), pY.end());
		}

		// Target:
		const mrpt::math::TPoint2D trg_glob =
			mrpt::math::TPoint2D(robotPose + logptr->WS_targets_relative[0]);
		if (TX.empty() || std::abs((*TX.rbegin()) - trg_glob.x) > 1e-3 ||
			std::abs((*TY.rbegin()) - trg_glob.y) > 1e-3)
		{
			TX.push_back(trg_glob.x);
			TY.push_back(trg_glob.y);
		}
	}

	f << "\n % Points: \n"
	  << " Ps = [";
	for (size_t k = 0; k < X.size(); k++)
	{
		f << X[k] << " " << Y[k] << "\n";
	}

	f << "];\n"
	  << "plot(Ps(:,1),Ps(:,2),'k.','MarkerSize',3);\n";

	f << "\n % Target point: \n"
	  << " Ts = [";
	for (size_t k = 0; k < TX.size(); k++)
	{
		f << TX[k] << " " << TY[k] << "\n";
	}

	f << "];\n"
	  << "plot(Ts(:,1),Ts(:,2),'rx','MarkerSize',10);\n";

	f << "axis equal;\n"
	  << "\n";

	f << "%% drawRobotShape()\n"
	  << "function [] = drawRobotShape(sh,pose)\n"
	  << "nPts=size(sh,1);\n"
	  << "Pts=[];\n"
	  << "for i=1:(nPts+1),\n"
	  << "    j=mod(i-1,nPts)+1;\n"
	  << "    cc=cos(pose(3)); ss=sin(pose(3)); x=pose(1); y=pose(2);\n"
	  << "    Pts=[Pts;x+cc*sh(j,1)-ss*sh(j,2) y+ss*sh(j,1)+cc*sh(j,2) "
		 "];\n"
	  << "end\n"
	  << "plot(Pts(:,1),Pts(:,2)); hold on;\n";

	NANOGUI_END_TRY(*m_win)
}

void NavlogViewerApp::OnmnuSeePTGParamsSelected()
{
	NANOGUI_START_TRY

	const std::string sSection = "PTG_PARAMS";
	mrpt::config::CConfigFileMemory cfg;

	cfg.write(sSection, "PTG_COUNT", m_logdata_ptg_paths.size());

	CConfigFilePrefixer cfg_pre;
	cfg_pre.bind(cfg);

	for (size_t i = 0; i < m_logdata_ptg_paths.size(); i++)
	{
		mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
			m_logdata_ptg_paths[i];
		if (!ptg) continue;

		const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)i);
		cfg_pre.setPrefixes("", sKeyPrefix);

		ptg->saveToConfigFile(cfg_pre, sSection);
	}

	const std::string sCfgText = cfg.getContent();

	const std::string fileName = nanogui::file_dialog(
		{{"ini", "Save PTG configuration"}}, true /*save*/);

	if (fileName.empty()) return;

	std::ofstream f(fileName);
	f << sCfgText;

	NANOGUI_END_TRY(*m_win)
}

void NavlogViewerApp::OnmnuSaveScoreMatrixSelected()
{
	NANOGUI_START_TRY

	const std::string fileName =
		nanogui::file_dialog({{"txt", "Save score matrices"}}, true /*save*/);
	if (fileName.empty()) return;

	const size_t N = m_logdata.size();
	for (size_t i = 0; i < N; i++)
	{
		const CLogFileRecord::Ptr logsptr =
			std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
		const CLogFileRecord* logptr = logsptr.get();

		for (size_t iPTG = 0; iPTG < logptr->infoPerPTG.size(); iPTG++)
		{
			const CHolonomicLogFileRecord::Ptr& hlog =
				logptr->infoPerPTG[iPTG].HLFR;
			if (!hlog) continue;

			const mrpt::math::CMatrixD* dirs_scores =
				hlog->getDirectionScores();
			if (!dirs_scores || dirs_scores->rows() < 2) continue;

			const std::string sFil = mrpt::system::fileNameChangeExtension(
				fileName,
				mrpt::format(
					"step%06u_ptg%02u.txt", (unsigned int)i,
					(unsigned int)iPTG));

			dirs_scores->saveToTextFile(sFil, mrpt::math::MATRIX_FORMAT_FIXED);
		}
	}

	NANOGUI_END_TRY(*m_win)
}

void NavlogViewerApp::OnmnuSaveCurrentObstacles()
{
	NANOGUI_START_TRY

	const int log_idx = mrpt::round(slidLog->value());
	if (log_idx >= int(m_logdata.size())) return;
	auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
	const CLogFileRecord& log = *logptr;

	const std::string fileName = nanogui::file_dialog(
		{{"txt", "Obstacles ASCII plain text file"}}, true /*save*/);
	if (fileName.empty()) return;

	log.WS_Obstacles_original.save2D_to_text_file(fileName);

	NANOGUI_END_TRY(*m_win)
}

void NavlogViewerApp::OntimMouseXY()
{
	// Mouse position at Z=0
	if (!m_win) return;

	const auto mousexY = m_win->mousePos();

	mrpt::math::TLine3D mouse_ray;
	m_win->background_scene->getViewport("main")->get3DRayForPixelCoord(
		mousexY.x(), mousexY.y(), mouse_ray);

	int lineY = 0, unique_id = 0;

	// Create a 3D plane, e.g. Z=0
	const mrpt::math::TPlane ground_plane(
		TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
	// Intersection of the line with the plane:
	mrpt::math::TObject3D inters;
	mrpt::math::intersect(mouse_ray, ground_plane, inters);
	// Interpret the intersection as a point, if there is an
	// intersection:
	mrpt::math::TPoint3D inters_pt;
	if (inters.getPoint(inters_pt))
	{
		ADD_WIN_TEXTMSG(mrpt::format(
			"Mouse pos: X=%.04f  Y=%.04f", inters_pt.x, inters_pt.y));
	}
}

void NavlogViewerApp::OnmnuMatlabExportPaths()
{
	NANOGUI_START_TRY;

	const size_t N = m_logdata.size();
	ASSERTMSG_(N > 0, "Log is empty! Load a valid log first...");

	const std::string fileName =
		nanogui::file_dialog({{"m", "MATLAB/Octave script"}}, true /*save*/);

	if (fileName.empty()) return;

	ofstream f(fileName);
	if (!f.is_open()) throw runtime_error("Error writing to file!");

	f << "% Script for drawing navigation paths\n"
	  << "% Generated automatically by navlog-viewer - MRPT "
	  << mrpt::system::MRPT_getVersion() << "\n"
	  << "%  From log: " << m_openedFileName << "\n"
	  << "% "
		 "-------------------------------------------------------------"
		 "----"
		 "----"
		 "----\n\n";

	std::map<double, int> selected_PTG_over_time;  // time: tim_start_iteration
	std::map<double, double> iteration_duration;  // time: tim_start_iteration
	struct TRobotPoseVel
	{
		mrpt::math::TPose2D pose, poseOdom;
		mrpt::math::TTwist2D velGlobal, velLocal;
	};
	std::map<double, TRobotPoseVel> global_local_vel;  // time: curPoseAndVel
	std::map<double, double> vals_timoff_obstacles, vals_timoff_curPoseVelAge,
		vals_timoff_sendVelCmd;	 // time: tim_start_iteration

	const int MAX_CMDVEL_COMPONENTS = 15;
	using cmdvel_vector_t = mrpt::math::CVectorDouble;
	// map: time -> tim_send_cmd_vel
	std::map<double, cmdvel_vector_t> cmdvels;

	double tim_start_iteration = .0;

	for (size_t i = 0; i < N; i++)
	{
		const CLogFileRecord::Ptr logsptr =
			std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
		const CLogFileRecord* logptr = logsptr.get();

		{
			const auto it = logptr->timestamps.find("tim_start_iteration");
			if (it != logptr->timestamps.end())
			{
				tim_start_iteration =
					mrpt::system::timestampToDouble(it->second);
			}
			else
			{
				tim_start_iteration++;	// just in case we don't have
										// valid
				// iteration timestamp (??)
			}
		}

		// selected PTG:
		selected_PTG_over_time[tim_start_iteration] = logptr->nSelectedPTG;

		// iter dur:
		{
			auto itExecT = logptr->values.find("executionTime");
			if (itExecT != logptr->values.end())
				iteration_duration[tim_start_iteration] = itExecT->second;
		}

		// timoff_obstacles;
		{
			const auto it = logptr->values.find("timoff_obstacles");
			if (it != logptr->values.end())
				vals_timoff_obstacles[tim_start_iteration] = it->second;
		}
		// timoff_curPoseVelAge;
		{
			const auto it = logptr->values.find("timoff_curPoseVelAge");
			if (it != logptr->values.end())
				vals_timoff_curPoseVelAge[tim_start_iteration] = it->second;
		}
		// timoff_sendVelCmd
		{
			const auto it = logptr->values.find("timoff_sendVelCmd");
			if (it != logptr->values.end())
				vals_timoff_sendVelCmd[tim_start_iteration] = it->second;
		}

		// curPoseAndVel:
		{
			double tim_pose = tim_start_iteration;	// default

			const auto it = logptr->timestamps.find("curPoseAndVel");
			if (it != logptr->timestamps.end())
			{ tim_pose = mrpt::system::timestampToDouble(it->second); }

			auto& p = global_local_vel[tim_pose];
			p.pose = logptr->robotPoseLocalization;
			p.poseOdom = logptr->robotPoseOdometry;
			p.velGlobal = logptr->cur_vel;
			p.velLocal = logptr->cur_vel_local;
		}

		// send cmd vels:
		if (logptr->cmd_vel)
		{
			double tim_send_cmd_vel = tim_start_iteration;	// default
			const auto it = logptr->timestamps.find("tim_send_cmd_vel");
			if (it != logptr->timestamps.end())
			{
				tim_send_cmd_vel = mrpt::system::timestampToDouble(it->second);
			}

			auto& p = cmdvels[tim_send_cmd_vel];
			p.resize(MAX_CMDVEL_COMPONENTS);
			p.setZero();
			for (size_t k = 0; k < logptr->cmd_vel->getVelCmdLength(); k++)
				p[k] = logptr->cmd_vel->getVelCmdElement(k);
		}

	}  // end for each timestep

	double t_ref = 0;
	if (!selected_PTG_over_time.empty())
	{ t_ref = selected_PTG_over_time.begin()->first; }

	f << "clear; close all;\n";

	f << "% robot pose over time. Columns: [time curPoseAndVel, "
		 "x,y,phi_rad,  "
		 "odo_x,odo_y,odo_phi_rad]\n"
		 "robot_pose = [";
	for (const auto& e : global_local_vel)
	{
		f << (e.first - t_ref) << "," << e.second.pose.x << ","
		  << e.second.pose.y << "," << e.second.pose.phi << ", "
		  << e.second.poseOdom.x << "," << e.second.poseOdom.y << ","
		  << e.second.poseOdom.phi << " ; ";
	}
	f << "];\n"
		 "robot_pose(:,4) = unwrap(robot_pose(:,4));\n"
		 "figure(); subplot(2,1,1); \n"
		 "plot(robot_pose(:, 1), robot_pose(:, 2 : 4), '.', "
		 "robot_pose(:, "
		 "1), "
		 "robot_pose(:, 2 : 4), '-'); xlabel('Time'); legend('x', 'y', "
		 "'phi "
		 "(rad)'); title('robot pose'); \n"
		 "xl=xlim; subplot(2,1,2);\n"
		 "robot_pose_diff = diff(robot_pose(:, 2 : 4));\n"
		 "idxs_rob_incr_z = find(robot_pose_diff(:, 1) == 0 & "
		 "robot_pose_diff(:, 2) == 0 & robot_pose_diff(:, 3) == 0);\n"
		 "plot(robot_pose(2:end, 1), diff(robot_pose(:, 2 : 4)), "
		 "'.');\n"
		 "hold on;\n"
		 "plot(robot_pose(idxs_rob_incr_z, 1), "
		 "robot_pose_diff(idxs_rob_incr_z, :), 'k', 'MarkerSize', 11, "
		 "'Marker', 'square');\n"
		 "xlabel('Time'); legend('x', 'y', 'phi (rad)'); title('Robot "
		 "pose "
		 "*increments*');\n"
		 "xlim(xl);\n\n";

	f << "% Selected PTG over time. Columns: [tim_start_iteration, "
		 "0-based "
		 "index selected PTG]\n"
		 "selected_PTG = [";
	for (const auto& e : selected_PTG_over_time)
	{
		f << (e.first - t_ref) << "," << e.second << " ; ";
	}
	f << "];\n"
		 "figure(); plot(selected_PTG(:,1),selected_PTG(:,2), 'x'); "
		 "xlabel('Time'); ylabel('Selected PTG');\n\n";

	if (!iteration_duration.empty())
	{
		f << "% Iteration duration. Columns: [tim_start_iteration, "
			 "iter_duration_seconds]\n"
			 "iteration_duration = [";
		for (const auto& e : iteration_duration)
		{
			f << (e.first - t_ref) << "," << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); "
			 "plot(iteration_duration(:,1),iteration_duration(:,2), "
			 "'x');\n"
			 "hold on;\n"
			 "plot(selected_PTG(1:(end-1),1),diff(selected_PTG(:,1)),'."
			 "')"
			 ";"
			 "xlabel('Time'); legend('Iteration duration', 'Diff "
			 "consecutive "
			 "call time'); title('rnav_iter_call_time_duration');\n\n";
	}

	if (!vals_timoff_obstacles.empty())
	{
		f << "% vals_timoff_obstacles. Columns: [tim_start_iteration, "
			 "value]\n"
			 "timoff_obstacles = [";
		for (const auto& e : vals_timoff_obstacles)
		{
			f << (e.first - t_ref) << " , " << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); "
			 "plot(timoff_obstacles(:,1),timoff_obstacles(:,2), "
			 "'.',timoff_obstacles(:,1),timoff_obstacles(:,2), '-'); "
			 "xlabel('Time'); title('timoff_obstacles');\n\n";
	}
	if (!vals_timoff_curPoseVelAge.empty())
	{
		f << "% vals_timoff_curPoseVelAge. Columns: "
			 "[tim_start_iteration, "
			 "value]\n"
			 "timoff_curPoseVelAge = [";
		for (const auto& e : vals_timoff_curPoseVelAge)
		{
			f << (e.first - t_ref) << " , " << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); "
			 "plot(timoff_curPoseVelAge(:,1),timoff_curPoseVelAge(:,2),"
			 " "
			 "'.',timoff_curPoseVelAge(:,1),timoff_curPoseVelAge(:,2), "
			 "'-'); "
			 "xlabel('Time'); title('timoff_curPoseVelAge');\n\n";
	}
	if (!vals_timoff_sendVelCmd.empty())
	{
		f << "% vals_timoff_sendVelCmd. Columns: [tim_start_iteration, "
			 "value]\n"
			 "timoff_sendVelCmd = [";
		for (const auto& e : vals_timoff_sendVelCmd)
		{
			f << (e.first - t_ref) << " , " << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); plot(timoff_sendVelCmd (:,1),timoff_sendVelCmd "
			 "(:,2), "
			 "'.',timoff_sendVelCmd (:,1),timoff_sendVelCmd (:,2), "
			 "'-'); "
			 "xlabel('Time'); title('timoff_sendVelCmd ');\n\n";
	}

	f << "% robot vel over time. Columns: [time curPoseAndVel, "
		 "vx,vy,omega_rad_sec]\n"
		 "robot_vel_global = [";
	for (const auto& e : global_local_vel)
	{
		f << (e.first - t_ref) << "," << e.second.velGlobal.vx << ","
		  << e.second.velGlobal.vy << "," << e.second.velGlobal.omega << " ; ";
	}
	f << "];\n"
		 "figure(); "
		 "plot(robot_vel_global(:,1),robot_vel_global(:,2:4), "
		 "'.', "
		 "robot_vel_global(:,1),robot_vel_global(:,2:4), '-'); "
		 "xlabel('Time'); "
		 "title('Velocities (global)'); legend('vx','vy','omega');\n\n";

	f << "robot_vel_local = [";
	for (const auto& e : global_local_vel)
	{
		f << (e.first - t_ref) << "," << e.second.velLocal.vx << ","
		  << e.second.velLocal.vy << "," << e.second.velLocal.omega << " ; ";
	}
	f << "];\n"
		 "figure(); plot(robot_vel_local(:,1),robot_vel_local(:,2:4), "
		 "'.', "
		 "robot_vel_local(:,1),robot_vel_local(:,2:4), '-'); "
		 "xlabel('Time'); "
		 "title('Velocities (local)'); legend('vx','vy','omega');\n\n";

	// cmdvels:
	f << "% Movement commands sent to robot. Columns: [time "
		 "curPoseAndVel, "
		 "vx,vy,omega_rad_sec]\n"
		 "cmdvels = [";
	for (const auto& e : cmdvels)
	{
		f << (e.first - t_ref) << " ";
		f << e.second << " ; ";
	}
	f << "];\n"
		 "figure(); plot(cmdvels(:,1),cmdvels(:,2:"
	  << (MAX_CMDVEL_COMPONENTS + 1)
	  << "), '.', cmdvels(:,1),cmdvels(:,2:" << (MAX_CMDVEL_COMPONENTS + 1)
	  << "), '-'); xlabel('Time'); title('Issued motion commands "
		 "(meaning "
		 "CVehicleVelCmd-dependend)');\n\n";

	NANOGUI_END_TRY(*m_win)
}
