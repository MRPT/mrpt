/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
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
		cout << "   GUID : "<< it->guid << endl;
		cout << "   Unit : "<<it->unit << endl;
		cout << "  Vendor: " << it->vendor << endl;
		cout << "  Model : " << it->model << endl;
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
