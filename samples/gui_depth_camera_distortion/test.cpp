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
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>

#include <iostream>

// We need to define all variables here to avoid emscripten memory errors
// if they are defined as *locals* in the function.
struct AppData
{
	AppData() = default;

	mrpt::gui::CDisplayWindowGUI::Ptr win;
	mrpt::opengl::CAxis::Ptr gl_corner_reference;
	mrpt::obs::CObservation3DRangeScan::Ptr obs;

	mrpt::opengl::CPointCloudColoured::Ptr gl_pts;

	std::vector<nanogui::TextBox*> edBoxes;	 // cx,cy,... dist[7]

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

	// app.obs->cameraParams
};

void AppData::obs_params_to_gui()
{
	ASSERT_(obs);
	ASSERT_EQUAL_(edBoxes.size(), 12U);

	const auto& p = obs->cameraParams;

	edBoxes[0]->setValue(mrpt::format("%.04f", p.cx()));
	edBoxes[1]->setValue(mrpt::format("%.04f", p.cy()));
	edBoxes[2]->setValue(mrpt::format("%.04f", p.fx()));
	edBoxes[3]->setValue(mrpt::format("%.04f", p.fy()));

	for (int i = 0; i < 8; i++)
		edBoxes[3 + i]->setValue(mrpt::format("%.04g", p.dist[i]));
}

void AppData::gui_params_to_obs()
{
	ASSERT_(obs);
	ASSERT_EQUAL_(edBoxes.size(), 12U);

	auto& p = obs->cameraParams;

	p.cx(std::stod(edBoxes[0]->value()));
	p.cy(std::stod(edBoxes[1]->value()));
	p.fx(std::stod(edBoxes[2]->value()));
	p.fy(std::stod(edBoxes[3]->value()));

	for (int i = 0; i < 8; i++)
		p.dist.at(i) = std::stod(edBoxes[3 + i]);
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

	// Create GUI:
	app.win = mrpt::gui::CDisplayWindowGUI::Create(
		"Depth camera distortion demo", 900, 700, cp);

	// Add INPUT window:
	// -----------------------------
	nanogui::Window* w = new nanogui::Window(&(*app.win), "Input");
	w->setPosition(nanogui::Vector2i(10, 50));
	w->setLayout(new nanogui::GridLayout(
		nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 5, 0));
	w->setFixedWidth(350);

	// Load button:
	w->add<nanogui::Button>("Load depth image from rawlog...")
		->setCallback([w]() {
			try
			{
				const std::string loadFile = nanogui::file_dialog(
					{{"rawlog", "MRPT rawlog dataset files"},
					 {"*", "Any file"}},
					false);
				if (loadFile.empty()) return;

				mrpt::obs::CRawlog rawlog;
				if (!rawlog.loadFromRawLogFile(loadFile)) return;

				ASSERT_(!rawlog.empty());
				ASSERT_(
					rawlog.getType(0) == mrpt::obs::CRawlog::etSensoryFrame);

				const auto obs = rawlog.getAsObservations(0)
									 ->getObservationByClass<
										 mrpt::obs::CObservation3DRangeScan>();
				ASSERT_(obs);
				app.obs = obs;
				app.obs_params_to_gui();
				recalcAll();
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
				auto msg = nanogui::MessageDialog(
					w, nanogui::MessageDialog::Type::Warning, "Exception",
					e.what());
			}
		});

	// Sliders and value controls:
	auto pn = w->add<nanogui::Widget>();
	pn->setLayout(new nanogui::GridLayout(
		nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 5, 0));

	{
		std::vector<std::string> lb = {
			"cx=",			 "cy=",			  "fx=",		   "fy=",
			"dist[0] (k1)=", "dist[1] (k2)=", "dist[2] (t1)=", "dist[3] (t2)=",
			"dist[4] (k3)=", "dist[5] (k4)=", "dist[6] (k5)=", "dist[7] (k6)=",
		};

		for (size_t i = 0; i < lb.size(); i++)
		{
			pn->add<nanogui::Label>(lb[i]);

			nanogui::TextBox* ed = pn->add<nanogui::TextBox>(" ");
			app.edBoxes.push_back(ed);

			ed->setEditable(true);
			ed->setFormat("[-+]?[0-9.e+-]*");
			ed->setCallback([](const std::string&) {
				recalcAll();
				return true;
			});
			nanogui::Slider* sl = pn->add<nanogui::Slider>();
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

int main()
{
	try
	{
		AppDepthCamDemo();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
