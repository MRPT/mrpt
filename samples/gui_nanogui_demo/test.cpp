/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

#include <iostream>

#if MRPT_HAS_NANOGUI
void TestGUI()
{
	nanogui::init();

	{
		// Create main window:
		mrpt::gui::CDisplayWindowGUI_Params cp;
		// cp.fullscreen = true;

		mrpt::gui::CDisplayWindowGUI win(
			"CDisplayWindowGUI demo", 800, 600, cp);

#if 0
// Define a custom icon image:
		win.setIcon(mrpt::img::CImage::LoadFromFile("/path/to/icon.png"));
#endif
#if 0
		// Define a custom icon from a GIMP header C source data block:
		win.setIconFromData(header_data, 32, 32, 0);
#endif

		nanogui::FormHelper* fh = new nanogui::FormHelper(&win);

		// Add subwindow:
		nanogui::ref<nanogui::Window> subWin2 =
			fh->addWindow({300, 400}, "Test");
		subWin2->setLayout(new nanogui::GroupLayout());

		mrpt::gui::MRPT2NanoguiGLCanvas* glControl =
			subWin2->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
		subWin2->setPosition({10, 300});

		{
			auto scene = mrpt::opengl::COpenGLScene::Create();
			scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());

			glControl->camera().setZoomDistance(5.0f);

			auto lck = mrpt::lockHelper(glControl->scene_mtx);
			glControl->scene = std::move(scene);
		}

		// Add subwindow:
		nanogui::ref<nanogui::Window> subWin =
			fh->addWindow({300, 400}, "Test");
		bool show_corner = true;

		fh->addGroup("Visualization");
		fh->addVariable("Show XYZ corner", show_corner)
			->setCallback([&](const bool& c) { subWin2->setVisible(c); });

		fh->addButton("Quit", [&]() { win.setVisible(false); });

		subWin->setPosition({10, 10});

		// add a background scene:
		{
			auto scene = mrpt::opengl::COpenGLScene::Create();
			scene->insert(mrpt::opengl::CGridPlaneXY::Create());

			std::lock_guard<std::mutex> lck(win.background_scene_mtx);
			win.background_scene = std::move(scene);
		}

		win.performLayout();

		win.camera().setZoomDistance(10.0f);

		// Update view and process events:
		win.drawAll();
		win.setVisible(true);
		nanogui::mainloop();
	}

	nanogui::shutdown();
}
#endif

int main()
{
	try
	{
#if MRPT_HAS_NANOGUI
		TestGUI();
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
