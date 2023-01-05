/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>	 // pause()

#include <chrono>
#include <iostream>
#include <thread>

using namespace mrpt::gui;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::img;
using namespace std;

// ------------------------------------------------------
//					Test_FFMPEG_CaptureCamera
// ------------------------------------------------------
void Test_FFMPEG_CaptureCamera(const std::string& video_url)
{
	CFFMPEG_InputStream in_video;

	if (!in_video.openURL(
			video_url, false /*grayscale*/, true /* verbose */,
			{{"rtsp_transport", "tcp"}}))
		return;

	CDisplayWindow3D win("Video");

	CTicTac tictac;
	tictac.Tic();
	unsigned int nFrames = 0;

	std::cout << "Close the window to end program.\n";

	CImage img;
	for (;;)
	{
		if (!win.isOpen())
		{
			std::cout << "Window closed. Quitting.\n";
			break;
		}
		if (!in_video.retrieveFrame(img))
		{
			std::cout << "Video stream ended. Quitting.\n";
			break;
		}

		double fps = ++nFrames / tictac.Tac();

		// decimate for easier viewing:
		while (img.getWidth() > 1024)
			img = img.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);

		img.textOut(
			5, 5, mrpt::format("%.02f fps", fps), TColor(0x80, 0x80, 0x80));
		if (nFrames > 100)
		{
			tictac.Tic();
			nFrames = 0;
		}

		if (nFrames == 1)
			cout << "Video FPS: " << in_video.getVideoFPS() << endl;

		{
			auto& scene = win.get3DSceneAndLock();
			scene->getViewport()->setImageView(std::move(img));
			win.unlockAccess3DScene();
			win.repaint();
		}
		std::this_thread::sleep_for(1ms);
	}

	in_video.close();
	mrpt::system::pause();
}

int main(int argc, char** argv)
{
	try
	{
		if (argc != 2)
		{
			cout << "Usage: " << endl;
			cout << " Open a video file: " << argv[0] << " <VIDEOFILE>" << endl;
			cout << " Open an IP camera: " << argv[0]
				 << " rtsp://a.b.c.d/live.sdp" << endl;
			cout << endl;
			return 1;
		}

		Test_FFMPEG_CaptureCamera(argv[1]);

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
