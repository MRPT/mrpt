/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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


/*
   This example was contributed by Adrien Barral - Robopec (France)
*/

#include <mrpt/gui.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // [mrpt-maps]
#include <mrpt/hwdrivers/CLMS100eth.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace std;


int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        cout << "Usage : " << argv[0] << " <IP> <port> " << endl;
        return 0;
    }

    CLMS100Eth laser(string(argv[1]), atoi(argv[2]));
    laser.turnOn();

    bool isOutObs, hardwareError;
    CObservation2DRangeScan outObs;
    laser.doProcessSimple(isOutObs, outObs, hardwareError);

    CDisplayWindow3D win3D("Scan", 200, 200);

    COpenGLScenePtr ptr_scene = win3D.get3DSceneAndLock();

    opengl::CPlanarLaserScanPtr obj = opengl::CPlanarLaserScan::Create();
    obj->clear();
    obj->setColor(0,0,1);
    obj->setName( "scan_LMS100" );
    obj->setScan(outObs);
    ptr_scene->insert( obj );

    win3D.unlockAccess3DScene();
    win3D.forceRepaint();

    while(win3D.isOpen())
    {
        laser.doProcessSimple(isOutObs, outObs, hardwareError);

        ptr_scene = win3D.get3DSceneAndLock();
        opengl::CPlanarLaserScanPtr obj = (opengl::CPlanarLaserScanPtr)(ptr_scene->getByName("scan_LMS100"));
        obj->clear();
        obj->setScan(outObs);
        win3D.unlockAccess3DScene();
        win3D.forceRepaint();
        mrpt::system::sleep(20);
    }
    win3D.waitForKey();
    return 0;
}

