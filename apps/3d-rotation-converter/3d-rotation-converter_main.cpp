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
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/Lie/SO.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>

// We need to define all variables here to avoid emscripten memory errors
// if they are defined as *locals* in the function.
struct AppData
{
	mrpt::gui::CDisplayWindowGUI::Ptr win;
	mrpt::opengl::CSetOfObjects::Ptr gl_corner_user;
	mrpt::opengl::CAxis::Ptr gl_corner_reference;

	// Input variables (they are bound to the GUI controls):
	mrpt::math::CQuaternionDouble in_quat;
	mrpt::math::CMatrixFixed<double, 3, 3> in_rot;
	std::array<double, 3> in_ypr = {0, 0, 0};
	mrpt::math::TVector3D in_axisangle_ax{0, 0, 0};
	double in_axisangle_ang = 0;
	mrpt::math::CVectorFixed<double, 3> in_lie_log =
		mrpt::math::CVectorFixed<double, 3>::Zero();
	bool units_radians = true;
};

static AppData app;

static void AppRotationConverter()
{
	nanogui::init();

	// Create main window:
	mrpt::gui::CDisplayWindowGUI_Params cp;
	// cp.fullscreen = true;

	// Init values:
	app.in_rot.setIdentity();

	app.gl_corner_user = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
	app.gl_corner_reference = mrpt::opengl::CAxis::Create(
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 0.2f, 1.0f, true);
	app.gl_corner_reference->setTextScale(0.04);

	// In/out UI control declarations (declared here so we can use them in the
	// lambda below):
	nanogui::TabWidget* tabWidget = nullptr;
	nanogui::TextBox* ed_in_rot[3][3];
	nanogui::TextBox *edOutMatrix = nullptr, *edOutQuat = nullptr,
					 *edOutAxisAngle_Ax = nullptr, *edOutAxisAngle_An = nullptr,
					 *edOutLogSO3 = nullptr;
	nanogui::TextBox* ed_out_rot[3][3];
	nanogui::Slider* sl_in_ypr[3] = {nullptr, nullptr, nullptr};

	// The main function: update all calculations:
	auto lambdaRecalcAll = [&]() {
		mrpt::poses::CPose3D userPose;
		switch (tabWidget->activeTab())
		{
			// YPR
			case 0:
			{
				const double K = app.units_radians ? 1.0 : (M_PI / 180.0);
				userPose.setFromValues(
					0, 0, 0, app.in_ypr[0] * K, app.in_ypr[1] * K,
					app.in_ypr[2] * K);
			}
			break;
			// Rot matrix:
			case 1:
			{
				// Run a SVD to ensure we take the closest SO(3) matrix to the
				// user input:
				Eigen::Matrix3d M = app.in_rot.asEigen();
				Eigen::JacobiSVD<Eigen::Matrix3d> svd(
					M, Eigen::ComputeFullU | Eigen::ComputeFullV);

				auto R = mrpt::math::CMatrixDouble33(
					(svd.matrixU() * svd.matrixV().transpose()).eval());
				userPose.setRotationMatrix(R);
				for (int r = 0; r < 3; r++)
					for (int c = 0; c < 3; c++)
						ed_in_rot[r][c]->setValue(
							mrpt::format("%.05f", R(r, c)));
			}
			break;
			// Quaternion
			case 2:
			{
				userPose = mrpt::poses::CPose3D(app.in_quat, 0, 0, 0);
			}
			break;
			// axis+angle
			case 3:
			{
				const double K = app.units_radians ? 1.0 : (M_PI / 180.0);
				mrpt::math::TVector3D v =
					app.in_axisangle_ax * (app.in_axisangle_ang * K);
				mrpt::math::CVectorFixed<double, 3> vn;
				for (int i = 0; i < 3; i++)
					vn[i] = v[i];

				userPose.setRotationMatrix(mrpt::poses::Lie::SO<3>::exp(vn));
			}
			break;
			// log(SO(3))
			case 4:
			{
				userPose.setRotationMatrix(
					mrpt::poses::Lie::SO<3>::exp(app.in_lie_log));
			}
			break;
		};

		// Set 3D view corner:
		app.gl_corner_user->setPose(userPose);

		// Update text output:
		const mrpt::math::CMatrixDouble33 Rout = userPose.getRotationMatrix();

		// matrix:
		edOutMatrix->setValue(Rout.inMatlabFormat());
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				ed_out_rot[r][c]->setValue(mrpt::format("%.05f", Rout(r, c)));

		// quat:
		mrpt::math::CQuaternionDouble q;
		userPose.getAsQuaternion(q);
		edOutQuat->setValue(q.asString());

		// SO(3) log:
		const auto log_R = mrpt::poses::Lie::SO<3>::log(Rout);
		edOutLogSO3->setValue(
			mrpt::format("[%.05f %.05f %.05f]", log_R[0], log_R[1], log_R[2]));

		// axis-angle:
		mrpt::math::TVector3D axis(log_R[0], log_R[1], log_R[2]);
		if (axis.norm() > 1e-20) axis = axis.unitarize();

		edOutAxisAngle_Ax->setValue(
			mrpt::format("[%.05f %.05f %.05f]", axis[0], axis[1], axis[2]));
		edOutAxisAngle_An->setValue(mrpt::format(
			"%.05f %s", log_R.norm() * (app.units_radians ? 1.0 : 180.0 / M_PI),
			(app.units_radians ? "rad" : "deg")));
	};

	// Create GUI:
	app.win = mrpt::gui::CDisplayWindowGUI::Create(
		"3D rotation converter", 900, 700, cp);

	// Add INPUT window:
	// -----------------------------
	nanogui::Window* winInput =
		new nanogui::Window(&(*app.win), "Rotation input");
	winInput->setPosition(nanogui::Vector2i(10, 50));
	winInput->setLayout(new nanogui::GroupLayout());
	winInput->setFixedWidth(350);

	tabWidget = winInput->add<nanogui::TabWidget>();

	{  // Yaw/pitch/roll  tab:
		nanogui::Widget* layer = tabWidget->createTab("Yaw-pitch-roll");
		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 5,
			0);
		layer->setLayout(layout);

		const char* lb[3] = {"yaw (Z)=", "pitch (Y)=", "roll (x)="};

		for (int i = 0; i < 3; i++)
		{
			layer->add<nanogui::Label>(lb[i]);
			nanogui::TextBox* ed = layer->add<nanogui::TextBox>(
				mrpt::format("%.2f", app.in_quat[i]));
			ed->setEditable(true);
			ed->setFormat("[-+]?[0-9.e+-]*");
			ed->setCallback(
				[i, lambdaRecalcAll, &sl_in_ypr](const std::string& s) {
					app.in_ypr[i] = std::stod(s);
					sl_in_ypr[i]->setValue(
						app.in_ypr[i] / (app.units_radians ? M_PIf : 180.0f));
					lambdaRecalcAll();
					return true;
				});
			nanogui::Slider* sl = layer->add<nanogui::Slider>();
			sl_in_ypr[i] = sl;
			sl->setRange({-1.0f, 1.0f});
			sl->setCallback([&, i, ed](float val) {
				val *= app.units_radians ? M_PIf : 180.0f;
				if (i == 1) val *= 0.5f;  // Pitch
				ed->setValue(mrpt::format("%.03f", val));
				ed->callback()(ed->value());
			});
		}
	}

	{  // rotation matrix tab:
		nanogui::Widget* layer = tabWidget->createTab("SO(3) matrix");

		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 5,
			0);
		layer->setLayout(layout);

		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				nanogui::TextBox* ed = layer->add<nanogui::TextBox>(
					mrpt::format("%.04f", app.in_rot(r, c)));
				ed->setEditable(true);
				ed->setFormat("[-+]?[0-9.e+-]*");
				ed->setCallback([r, c](const std::string& s) {
					app.in_rot(r, c) = std::stod(s);
					return true;
				});
				ed_in_rot[r][c] = ed;
			}
		}
	}

	{  // Quaternion tab:
		nanogui::Widget* layer = tabWidget->createTab("Quaternion");
		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 5,
			0);
		layer->setLayout(layout);

		const char* lb[4] = {"w (real)=", "x=", "y=", "z="};

		for (int i = 0; i < 4; i++)
		{
			layer->add<nanogui::Label>(lb[i]);

			nanogui::TextBox* ed = layer->add<nanogui::TextBox>(
				mrpt::format("%.2f", app.in_quat[i]));
			ed->setEditable(true);
			ed->setFormat("[-+]?[0-9.e+-]*");
			ed->setCallback([i](const std::string& s) {
				app.in_quat[i] = std::stod(s);
				return true;
			});
		}
	}

	{  // Axis-angle tab:
		nanogui::Widget* layer = tabWidget->createTab("Axis-angle");
		layer->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 5,
			0));
		layer->add<nanogui::Label>("Axis:");

		{
			nanogui::Widget* panel = layer->add<nanogui::Widget>();
			panel->setLayout(new nanogui::GridLayout(
				nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill,
				5, 0));

			for (int r = 0; r < 3; r++)
			{
				nanogui::TextBox* ed = panel->add<nanogui::TextBox>(
					mrpt::format("%.04f", app.in_axisangle_ax[r]));
				ed->setEditable(true);
				ed->setFormat("[-+]?[0-9.e+-]*");
				ed->setCallback([r](const std::string& s) {
					app.in_axisangle_ax[r] = std::stod(s);
					return true;
				});
			}
		}
		{
			layer->add<nanogui::Label>("Angle:");
			nanogui::TextBox* ed = layer->add<nanogui::TextBox>(
				mrpt::format("%.04f", app.in_axisangle_ang));
			ed->setEditable(true);
			ed->setFormat("[-+]?[0-9.e+-]*");
			ed->setCallback([](const std::string& s) {
				app.in_axisangle_ang = std::stod(s);
				return true;
			});
		}
	}

	{  // axis with magnitude = log(R) in Lie group
		nanogui::Widget* layer = tabWidget->createTab("log(SO(3))");

		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 5,
			0);
		layer->setLayout(layout);

		layer->add<nanogui::Label>("Axis with magnitude", "sans-bold");
		layer->add<nanogui::Label>("(i.e. vee(log(R)) in SO(3))");
		{
			nanogui::Widget* panel = layer->add<nanogui::Widget>();
			panel->setLayout(new nanogui::GridLayout(
				nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill,
				5, 0));

			for (int r = 0; r < 3; r++)
			{
				nanogui::TextBox* ed = layer->add<nanogui::TextBox>(
					mrpt::format("%.04f", app.in_lie_log[r]));
				ed->setEditable(true);
				ed->setFormat("[-+]?[0-9.e+-]*");
				ed->setCallback([r](const std::string& s) {
					app.in_lie_log[r] = std::stod(s);
					return true;
				});
			}
		}
	}

	tabWidget->setActiveTab(0);

	// Apply button:
	winInput->add<nanogui::Button>("Apply", ENTYPO_ICON_CALCULATOR)
		->setCallback(lambdaRecalcAll);

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
				"Show rotated frame",
				[&](bool b) { app.gl_corner_user->setVisibility(b); })
			->setChecked(true);
		winMenu
			->add<nanogui::CheckBox>(
				"Ortho. view",
				[&](bool b) { app.win->camera().setCameraProjective(!b); })
			->setChecked(false);

		winMenu->add<nanogui::Label>("Units:");
		winMenu
			->add<nanogui::ComboBox>(
				std::vector<std::string>({"radians", "degrees"}))
			->setCallback([&](int index) {
				// On units update:
				app.units_radians = (index == 0);
				lambdaRecalcAll();
			});
	}

	// Add OUTPUT window:
	// -----------------------------
	{
		nanogui::Window* winOutput =
			new nanogui::Window(&(*app.win), "Rotation output");
		winOutput->setPosition(nanogui::Vector2i(10, 320));
		auto layout = new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 1, nanogui::Alignment::Fill, 5,
			0);
		winOutput->setLayout(layout);
		winOutput->setFixedWidth(350);

		winOutput->add<nanogui::Label>("SO(3) rotation matrix", "sans-bold");
		{
			nanogui::Widget* panel = winOutput->add<nanogui::Widget>();
			panel->setLayout(new nanogui::GridLayout(
				nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill,
				5, 0));
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					ed_out_rot[r][c] = panel->add<nanogui::TextBox>("");
		}

		winOutput->add<nanogui::Label>("In MATLAB-like notation:");
		edOutMatrix = winOutput->add<nanogui::TextBox>("");
		edOutMatrix->setEditable(true);

		winOutput->add<nanogui::Label>("Quaternion (r,x,y,z)", "sans-bold");
		edOutQuat = winOutput->add<nanogui::TextBox>("");
		edOutQuat->setEditable(true);

		winOutput->add<nanogui::Label>("Axis-angle (r,x,y,z)", "sans-bold");
		edOutAxisAngle_Ax = winOutput->add<nanogui::TextBox>("");
		edOutAxisAngle_Ax->setEditable(true);
		edOutAxisAngle_An = winOutput->add<nanogui::TextBox>("");
		edOutAxisAngle_An->setEditable(true);

		winOutput->add<nanogui::Label>("Axis with angle (log(R))", "sans-bold");
		edOutLogSO3 = winOutput->add<nanogui::TextBox>("");
		edOutLogSO3->setEditable(true);
	}

	// Add a background scene:
	// -----------------------------
	{
		auto scene = mrpt::opengl::COpenGLScene::Create();
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());

		scene->insert(app.gl_corner_user);
		scene->insert(app.gl_corner_reference);

		auto lck = mrpt::lockHelper(app.win->background_scene_mtx);
		app.win->background_scene = std::move(scene);
	}

	app.win->performLayout();

	app.win->camera().setZoomDistance(5.0f);

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
		AppRotationConverter();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
