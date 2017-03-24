/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses.h>
#include <mrpt/utils.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// ------------------------------------------------------
//				TestSE3
// ------------------------------------------------------
void TestSE3()
{
	const CPose3D  p0;
	const CPose3D  p1(1,2,3, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	const CPose3D  p2(1,2,3, DEG2RAD(20),DEG2RAD(0),DEG2RAD(0));


	cout << "p0: " << p0 << " => ln SE(3)->se(3) => " << p0.ln() << endl;
	cout << "p1: " << p1 << " => ln SE(3)->se(3) => " << p1.ln() << endl;
	cout << "p2: " << p2 << " => ln SE(3)->se(3) => " << p2.ln() << endl;


#if 0
	const CPose3D  p3(1,2,3, DEG2RAD(20),DEG2RAD(-30),DEG2RAD(15));
	const CPose3D  p4(4,2.5,1.5, DEG2RAD(-25),DEG2RAD(5),DEG2RAD(10));
	cout << "p3+p4: "<< (p3+p4) << " => ln SE(3)->se(3) => "
		<< endl << (p3.ln()+p4.ln())
		<< endl << (p3+p4).ln() << endl
		<< endl << CPose3D::exp( (p3.ln()+p4.ln()) ) << endl;
#endif

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestSE3();
		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}


