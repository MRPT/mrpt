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

using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


// ------------------------------------------------------
//				TestQuaternions
// ------------------------------------------------------
void TestQuaternions()
{
	CQuaternionDouble  q1,q2,q3;

	//q1 = CQuaternionDouble(1,2,3,4); q1.normalize();
	CPose3D p1(0,0,0, DEG2RAD(10),DEG2RAD(30),DEG2RAD(-20));
	p1.getAsQuaternion(q1);

	CPose3D p2(0,0,0, DEG2RAD(30),DEG2RAD(-20),DEG2RAD(10));
	p2.getAsQuaternion(q2);

	// q3 = q1 x q2
	q3.crossProduct(q1,q2);

	const CPose3D p3 = p1 + p2;

	cout << "q1 = " << q1 << endl;
	cout << "q1 as CPose3D = " << CPose3D(q1,0,0,0) << endl;
	cout << endl;
	cout << "q2 = " << q2 << endl;
	cout << "q2 as CPose3D = " << CPose3D(q2,0,0,0) << endl;
	cout << endl;
	cout << "q3 = q1 * q2 = " << q3 << endl;
	cout << "q3 as CPose3D = " << CPose3D(q3,0,0,0) << endl;

	cout << endl << "Should be equal to p3 = p1 (+) p2 = " << p3 << endl;
}

void TestQuaternionsIterators()
{
	CPose3DQuat    q(1.0,2.0,3.0, CQuaternionDouble() );

	cout << "Dump with iterators: ";
	for (CPose3DQuat::iterator it=q.begin();it!=q.end();++it)
		cout << *it << " "; 
	cout << endl;
}

int main(int argc, char **argv)
{
	try
	{
		TestQuaternions();
		TestQuaternionsIterators();

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
