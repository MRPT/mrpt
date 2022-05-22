/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifdef __EMSCRIPTEN__  // are we actually in a Wasm JS instance?
#include <emscripten.h>
#include <emscripten/html5.h>

#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
#include <GLFW/glfw3.h>

#include <functional>

std::function<void()> loop;
void main_loop() { loop(); }
#endif

#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>

#include <iostream>

#if MRPT_HAS_NANOGUI

constexpr int SMALL_FONT_SIZE = 15;
constexpr int LARGE_FONT_SIZE = 20;

// We need to define all variables here to avoid emscripten memory errors
// if they are defined as *locals* in the function.
struct AppData
{
	AppData() = default;

	mrpt::gui::CDisplayWindowGUI::Ptr win;
	mrpt::opengl::CAxis::Ptr gl_corner_reference;
	mrpt::obs::CObservation3DRangeScan::Ptr obs;

	mrpt::img::TCamera originalCalib;
	mrpt::poses::CPose3D originalSensorPose;

	mrpt::opengl::CPointCloudColoured::Ptr gl_pts;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensorPoseCorner;

	std::vector<nanogui::TextBox*> edBoxes /* cx,cy,... dist[7]*/
		,
		edBoxesExtrinsics /*x,y,... pitch, roll*/;

	std::vector<nanogui::Slider*> sliders /* cx,cy,... dist[7] */,
		slidersExtrinsics /*x,y,... pitch, roll*/;

	void obs_params_to_gui();
	void gui_params_to_obs();
};

static AppData app;

// The main function: update all graphs
static void recalcAll()
{
	auto lck = mrpt::lockHelper(app.win->background_scene_mtx);

	if (!app.obs || !app.obs->hasRangeImage)
	{
		app.gl_pts->clear();
		return;
	}

	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = true;

	app.obs->unprojectInto(*app.gl_pts, pp);

	const auto bbox = app.gl_pts->getBoundingBox();

	app.gl_pts->recolorizeByCoordinate(
		bbox.min.z, bbox.max.z, 2 /*0,1,2=x,y,z*/);

	app.gl_pts->setPointSize(2.0f);

	app.gl_sensorPoseCorner->setPose(app.obs->sensorPose);
};

void AppData::obs_params_to_gui()
{
	ASSERT_(obs);
	ASSERT_EQUAL_(edBoxes.size(), sliders.size());
	ASSERT_EQUAL_(edBoxes.size(), 12U);

	const auto& p = obs->cameraParams;

	edBoxes[0]->setValue(mrpt::format("%.04f", p.cx()));
	edBoxes[1]->setValue(mrpt::format("%.04f", p.cy()));
	edBoxes[2]->setValue(mrpt::format("%.04f", p.fx()));
	edBoxes[3]->setValue(mrpt::format("%.04f", p.fy()));

	for (int i = 0; i < 8; i++)
		edBoxes[4 + i]->setValue(mrpt::format("%.04e", p.dist[i]));

	// slider ranges:
	const double maxPixelRanges =
		std::max(std::max(p.cx(), p.cy()), std::max(p.fx(), p.fy()));
	const double minPixelRanges =
		std::min(std::min(p.cx(), p.cy()), std::min(p.fx(), p.fy()));
	for (int i = 0; i < 4; i++)
	{
		sliders[i]->setRange({0.5 * minPixelRanges, 2.0 * maxPixelRanges});
	}
	for (int i = 4; i < 12; i++)
	{
		const double absVal =
			std::max(1e-3, std::abs(std::stod(edBoxes[i]->value())));
		sliders[i]->setRange({-2.0 * absVal, 2.0 * absVal});
	}

	// slider values:
	for (size_t i = 0; i < edBoxes.size(); i++)
		sliders[i]->setValue(std::stod(edBoxes[i]->value()));

	// Extrinsics ranges:
	for (int i = 0; i < 3; i++)
		slidersExtrinsics[i]->setRange({-2.0, 2.0});

	slidersExtrinsics[3]->setRange({-180.0, 180.0});
	slidersExtrinsics[4]->setRange({-90.0, 90.0});
	slidersExtrinsics[5]->setRange({-180.0, 180.0});

	// Extrinsics values:
	for (int i = 0; i < 6; i++)
	{
		double val = obs->sensorPose[i];
		if (i > 3) val = mrpt::RAD2DEG(val);

		edBoxesExtrinsics[i]->setValue(mrpt::format("%.04f", val));
		slidersExtrinsics[i]->setValue(val);
	}
}

void AppData::gui_params_to_obs()
{
	ASSERT_(obs);
	ASSERT_EQUAL_(edBoxes.size(), sliders.size());
	ASSERT_EQUAL_(edBoxes.size(), 12U);

	auto& p = obs->cameraParams;
	p.distortion = mrpt::img::DistortionModel::plumb_bob;

	p.cx(std::stod(edBoxes[0]->value()));
	p.cy(std::stod(edBoxes[1]->value()));
	p.fx(std::stod(edBoxes[2]->value()));
	p.fy(std::stod(edBoxes[3]->value()));

	for (int i = 0; i < 8; i++)
		p.dist.at(i) = std::stod(edBoxes[4 + i]->value());

	// Extrinsics:
	app.obs->sensorPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
		std::stod(edBoxesExtrinsics[0]->value()),  // x
		std::stod(edBoxesExtrinsics[1]->value()),  // y
		std::stod(edBoxesExtrinsics[2]->value()),  // z
		mrpt::DEG2RAD(std::stod(edBoxesExtrinsics[3]->value())),  // yaw
		mrpt::DEG2RAD(std::stod(edBoxesExtrinsics[4]->value())),  // pitch
		mrpt::DEG2RAD(std::stod(edBoxesExtrinsics[5]->value()))	 // roll
	);
}

static void AppDepthCamDemo()
{
	nanogui::init();

	// Create main window:
	mrpt::gui::CDisplayWindowGUI_Params cp;
	// cp.fullscreen = true;

	// Init values:
	app.gl_corner_reference = mrpt::opengl::CAxis::Create(
		-5.0f, -5.0f, .0f, 5.0f, 5.0f, 2.0f, 0.2f, 1.0f, true);
	app.gl_corner_reference->setTextScale(0.04);

	app.gl_pts = mrpt::opengl::CPointCloudColoured::Create();
	app.gl_sensorPoseCorner =
		mrpt::opengl::stock_objects::CornerXYZSimple(0.10f);

	// Create GUI:
	app.win = mrpt::gui::CDisplayWindowGUI::Create(
		"Depth camera distortion demo", 1000, 700, cp);

	// Add main window:
	// -----------------------------
	nanogui::Window* w = new nanogui::Window(&(*app.win), "Camera parameters");
	w->setPosition(nanogui::Vector2i(10, 50));
	w->setLayout(new nanogui::GridLayout(
		nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 5, 0));
	w->setFixedWidth(350);

	// Load button:
	w->add<nanogui::Label>("Operations")->setFontSize(LARGE_FONT_SIZE);

	w->add<nanogui::Button>("Load depth image from rawlog...")
		->setCallback([]() {
			try
			{
				const std::string loadFile = nanogui::file_dialog(
					{{"rawlog", "MRPT rawlog dataset files"},
					 {"*", "Any file"}},
					false);
				if (loadFile.empty()) return;

				// Enable MRPT to find externalized rawlogs:
				mrpt::io::setLazyLoadPathBase(
					mrpt::obs::CRawlog::detectImagesDirectory(loadFile));

				mrpt::obs::CRawlog rawlog;
				if (!rawlog.loadFromRawLogFile(loadFile)) return;

				ASSERT_(!rawlog.empty());

				mrpt::obs::CObservation3DRangeScan::Ptr obs;

				// Obs-only format:
				if (rawlog.getType(0) == mrpt::obs::CRawlog::etObservation)
				{
					obs = std::dynamic_pointer_cast<
						mrpt::obs::CObservation3DRangeScan>(
						rawlog.getAsObservation(0));
					ASSERTMSG_(
						obs,
						"First observation in rawlog is not "
						"CObservation3DRangeScan");
				}
				else
				{
					// SF format:
					ASSERT_(
						rawlog.getType(0) ==
						mrpt::obs::CRawlog::etSensoryFrame);

					obs = rawlog.getAsObservations(0)
							  ->getObservationByClass<
								  mrpt::obs::CObservation3DRangeScan>();
					ASSERT_(obs);
				}

				app.obs = obs;
				app.originalCalib = obs->cameraParams;
				app.originalSensorPose = obs->sensorPose;
				app.obs_params_to_gui();
				recalcAll();
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
				auto dlg = new nanogui::MessageDialog(
					app.win->screen(), nanogui::MessageDialog::Type::Warning,
					"Exception", e.what());
			}
		});

	w->add<nanogui::Button>("Revert to original calibration")
		->setCallback([]() {
			if (!app.obs) return;
			app.obs->cameraParams = app.originalCalib;
			app.obs->sensorPose = app.originalSensorPose;
			app.obs_params_to_gui();
			recalcAll();
		});

	// Sliders and value controls:
	w->add<nanogui::Label>("Intrinsics")->setFontSize(LARGE_FONT_SIZE);
	;
	{
		auto pn = w->add<nanogui::Widget>();
		pn->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 5,
			0));

		std::vector<std::string> lb = {
			"cx=",			 "cy=",			  "fx=",		   "fy=",
			"dist[0] (k1)=", "dist[1] (k2)=", "dist[2] (t1)=", "dist[3] (t2)=",
			"dist[4] (k3)=", "dist[5] (k4)=", "dist[6] (k5)=", "dist[7] (k6)=",
		};

		for (size_t i = 0; i < lb.size(); i++)
		{
			pn->add<nanogui::Label>(lb[i])->setFontSize(SMALL_FONT_SIZE);

			nanogui::TextBox* ed = pn->add<nanogui::TextBox>(" ");
			app.edBoxes.push_back(ed);

			ed->setEditable(true);
			ed->setFormat("[-+]?[0-9.e+-]*");
			ed->setCallback([](const std::string&) {
				app.gui_params_to_obs();
				recalcAll();
				return true;
			});
			nanogui::Slider* sl = pn->add<nanogui::Slider>();
			app.sliders.push_back(sl);

			sl->setRange({-1.0f, 1.0f});
			sl->setCallback([&, ed](float val) {
				ed->setValue(mrpt::format("%.03g", val));
				ed->callback()(ed->value());
			});

			ed->setFontSize(SMALL_FONT_SIZE);
		}
	}

	// Extrinsics:
	w->add<nanogui::Label>("Extrinsics")->setFontSize(LARGE_FONT_SIZE);
	{
		auto pn = w->add<nanogui::Widget>();
		pn->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 5,
			0));

		std::vector<std::string> lb = {
			"x=", "y=", "z=", "yaw=", "pitch=", "roll="};

		for (size_t i = 0; i < lb.size(); i++)
		{
			pn->add<nanogui::Label>(lb[i])->setFontSize(SMALL_FONT_SIZE);

			nanogui::TextBox* ed = pn->add<nanogui::TextBox>(" ");
			app.edBoxesExtrinsics.push_back(ed);

			ed->setEditable(true);
			ed->setFormat("[-+]?[0-9.e+-]*");
			ed->setCallback([](const std::string&) {
				app.gui_params_to_obs();
				recalcAll();
				return true;
			});
			ed->setFontSize(SMALL_FONT_SIZE);

			nanogui::Slider* sl = pn->add<nanogui::Slider>();
			app.slidersExtrinsics.push_back(sl);

			sl->setRange({-1.0f, 1.0f});
			sl->setCallback([&, ed](float val) {
				ed->setValue(mrpt::format("%.03g", val));
				ed->callback()(ed->value());
			});
		}
	}

	// Add top menu subwindow:
	// -----------------------------
	{
		nanogui::ref<nanogui::Window> winMenu =
			new nanogui::Window(&(*app.win), "");
		winMenu->setPosition(nanogui::Vector2i(0, 0));
		winMenu->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5));
		nanogui::Theme* modTheme =
			new nanogui::Theme(app.win->screen()->nvgContext());
		modTheme->mWindowHeaderHeight = 1;
		winMenu->setTheme(modTheme);

#ifndef __EMSCRIPTEN__	// are we actually in a Wasm JS instance?
		winMenu->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
			->setCallback([]() { app.win->setVisible(false); });
#endif

		winMenu->add<nanogui::Label>("      ");	 // separator

		winMenu
			->add<nanogui::CheckBox>(
				"Show reference frame",
				[&](bool b) { app.gl_corner_reference->setVisibility(b); })
			->setChecked(true);

		winMenu
			->add<nanogui::CheckBox>(
				"Ortho. view",
				[&](bool b) { app.win->camera().setCameraProjective(!b); })
			->setChecked(false);
	}

	// Add a background scene:
	// -----------------------------
	{
		auto scene = mrpt::opengl::COpenGLScene::Create();
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());

		scene->insert(app.gl_corner_reference);
		scene->insert(app.gl_pts);
		scene->insert(app.gl_sensorPoseCorner);

		auto lck = mrpt::lockHelper(app.win->background_scene_mtx);
		app.win->background_scene = std::move(scene);
	}

	app.win->performLayout();

	app.win->camera().setZoomDistance(7.0f);

	// Update view and process events:
	app.win->drawAll();
	app.win->setVisible(true);

#if !defined(__EMSCRIPTEN__)  // are we in a Wasm JS instance?
	// No: regular procedure:

	nanogui::mainloop();
	nanogui::shutdown();
#else

	// Yes, we are in a web browser running on JS:
	loop = [] {
		// Check if any events have been activated (key pressed, mouse moved
		// etc.) and call corresponding response functions
		glfwPollEvents();

		glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		// Draw nanogui
		app.win->drawContents();
		app.win->drawWidgets();

		glfwSwapBuffers(app.win->glfwWindow());
	};
	emscripten_set_main_loop(main_loop, 0, true);

	glfwDestroyWindow(app.win->nanogui_screen()->glfwWindow());

	// Terminate GLFW, clearing any resources allocated by GLFW.
	glfwTerminate();

#endif
}

#endif	// MRPT_HAS_NANOGUI

int main()
{
	try
	{
#if MRPT_HAS_NANOGUI
		AppDepthCamDemo();
#else
		std::cerr << "This example requires MRPT built with NANOGUI.\n";
#endif
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
