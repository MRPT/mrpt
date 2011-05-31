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


