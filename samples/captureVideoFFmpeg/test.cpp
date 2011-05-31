/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>
#include <mrpt/hwdrivers.h>

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
		img.textOut(5,5,format("%.02f fps",fps),TColor(0x80,0x80,0x80) );
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
