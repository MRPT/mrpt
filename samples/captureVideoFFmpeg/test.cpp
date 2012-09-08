/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
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
