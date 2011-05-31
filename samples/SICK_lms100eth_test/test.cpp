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


/*
   This example was contributed by Adrien Barral - Robopec (France)
*/

#include <mrpt/gui.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
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

