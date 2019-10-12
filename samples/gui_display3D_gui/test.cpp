/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindowGUI.h>

void TestDisplayGUI()
{
	mrpt::gui::CDisplayWindowGUI win("Example of GUI Window - MRPT", 640, 480);

	//	COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	CTicTac timer;
	timer.Tic();

	while (win.isOpen())
	{
		const double t = timer.Tac();

		// Move the scene:
		// COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

		// Update window:
		// win.forceRepaint();
		std::this_thread::sleep_for(20ms);
	};
}

int main()
{
	try
	{
		TestDisplayGUI();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
