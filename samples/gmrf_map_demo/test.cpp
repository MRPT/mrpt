/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/gui.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;


void Example_GMRF()
{
	const double X_SIZE     = 10.0;
	const double Y_SIZE     = 10.0;
	const double RESOLUTION = 0.5;

	mrpt::slam::CGasConcentrationGridMap2D  gasmap(
		CRandomFieldGridMap2D::mrGMRF_G /*map type*/,
		0,X_SIZE,
		0,Y_SIZE,
		RESOLUTION /* resolution */
		);

	for (int i=0;i<20;i++)
	{
		const double value = randomGenerator.drawUniform(0.01,0.99);

		const double x = randomGenerator.drawUniform(0.1, 0.95*X_SIZE);
		const double y = randomGenerator.drawUniform(0.1, 0.95*Y_SIZE);

		gasmap.insertIndividualReading(value,  TPoint2D(x,y) );
	}


	// 3D view:
	mrpt::opengl::CSetOfObjectsPtr glObj = mrpt::opengl::CSetOfObjects::Create();
	gasmap.getAs3DObject( glObj );

	mrpt::gui::CDisplayWindow3D win("Map",640,480);

	mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
	scene->insert( glObj );
	win.unlockAccess3DScene();
	win.repaint();

	win.waitForKey();
}

int main(int argc, char **argv)
{
	try
	{
	    Example_GMRF();
		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
