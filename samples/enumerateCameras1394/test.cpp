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

#include <mrpt/hwdrivers.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace std;


//#define DO_CAPTURE		1
#define DO_CAPTURE		0

// ------------------------------------------------------
//				TestEnumerate_1394
// ------------------------------------------------------
void TestEnumerate_1394()
{
	CImageGrabber_dc1394::TCameraInfoList 	lstCams;

	cout << "Enumerating cameras..." << endl;

	CImageGrabber_dc1394::enumerateCameras( lstCams );

	cout << "Found " << lstCams.size() << " cameras." << endl;

	for (CImageGrabber_dc1394::TCameraInfoList::const_iterator it=lstCams.begin(); it!=lstCams.end();it++)
	{
		cout << "======= CAMERA =========" << endl;
		cout << format("   GUID: %"PRIX64"\n   Unit: %i\n", it->guid, it->unit );
		cout << "  Vendor: " << it->vendor << endl;
		cout << "  Model: " << it->model << endl;
		cout << endl;
	}
}


int main(int argc, char **argv)
{
	try
	{
		TestEnumerate_1394();

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
