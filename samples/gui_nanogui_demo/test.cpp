/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/opengl/stock_objects.h>
#include <iostream>

#if MRPT_HAS_NANOGUI
void TestGUI()
{
	nanogui::init();

	{
		// Create main window:
		mrpt::gui::CDisplayWindowGUI win("CDisplayWindowGUI demo", 500, 400);

		// Add controls:
		auto window = new nanogui::Window(&win, "Main");
		window->setPosition(Eigen::Vector2i(15, 15));
		window->setLayout(new nanogui::GroupLayout());

		nanogui::Widget* tools = new nanogui::Widget(window);
		tools->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0,
			5));

		auto b0 = tools->add<nanogui::Button>("Do it");
		b0->setCallback([]() { std::cout << "button0\n"; });

		mrpt::gui::MRPTGLCanvas* glControl =
			tools->add<mrpt::gui::MRPTGLCanvas>();

		{
			auto scene = mrpt::opengl::COpenGLScene::Create();
			scene->insert(mrpt::opengl::stock_objects::CornerXYZEye());

			glControl->scene_mtx.lock();
			glControl->scene = std::move(scene);
			glControl->scene_mtx.unlock();
		}

		win.performLayout();

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
