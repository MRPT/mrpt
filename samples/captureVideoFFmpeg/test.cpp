/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/threads.h>

using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::hwdrivers;
using namespace std;


// ------------------------------------------------------
//					Test_FFMPEG_CaptureCamera
// ------------------------------------------------------
void Test_FFMPEG_CaptureCamera(const std::string &video_url)
{
	CFFMPEG_InputStream		in_video;

	if (!in_video.openURL(video_url,false /*grayscale*/, true /* verbose */ ))
		return;

	CDisplayWindow   win("Video");

	CTicTac	tictac;
	tictac.Tic();
	unsigned int nFrames = 0;

	CImage img;
	while (win.isOpen() && in_video.retrieveFrame(img))
	{
		double fps = ++nFrames / tictac.Tac();
		img.textOut(5,5,mrpt::format("%.02f fps",fps),TColor(0x80,0x80,0x80) );
		if (nFrames>100)
		{
			tictac.Tic();
			nFrames=0;
		}

		if (nFrames==1)
			cout << "Video FPS: " << in_video.getVideoFPS() << endl;


		win.showImage(img);
		mrpt::system::sleep(10);

		if (win.keyHit() && win.waitForKey()==27) break;
	}


	in_video.close();
	mrpt::system::pause();
}


int main(int argc, char **argv)
{
	try
	{
		if (argc!=2)
		{
			cout << "Usage: " << endl;
			cout << " Open a video file: " << argv[0] << " <VIDEOFILE>" << endl;
			cout << " Open an IP camera: " << argv[0] << " rtsp://a.b.c.d/live.sdp" << endl;
			cout << endl;
			return 1;
		}

		Test_FFMPEG_CaptureCamera(argv[1]);

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
