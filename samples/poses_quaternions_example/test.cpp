/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// ------------------------------------------------------
//				TestQuaternions
// ------------------------------------------------------
void TestQuaternions()
{
	CQuaternionDouble q1, q2, q3;

	// q1 = CQuaternionDouble(1,2,3,4); q1.normalize();
	CPose3D p1(0, 0, 0, 10.0_deg, 30.0_deg, -20.0_deg);
	p1.getAsQuaternion(q1);

	CPose3D p2(0, 0, 0, 30.0_deg, -20.0_deg, 10.0_deg);
	p2.getAsQuaternion(q2);

	// q3 = q1 x q2
	q3.crossProduct(q1, q2);

	const CPose3D p3 = p1 + p2;

	cout << "q1 = " << q1 << endl;
	cout << "q1 as CPose3D = " << CPose3D(q1, 0, 0, 0) << endl;
	cout << endl;
	cout << "q2 = " << q2 << endl;
	cout << "q2 as CPose3D = " << CPose3D(q2, 0, 0, 0) << endl;
	cout << endl;
	cout << "q3 = q1 * q2 = " << q3 << endl;
	cout << "q3 as CPose3D = " << CPose3D(q3, 0, 0, 0) << endl;

	cout << endl << "Should be equal to p3 = p1 (+) p2 = " << p3 << endl;
}

void TestQuaternionsIterators()
{
	CPose3DQuat q(1.0, 2.0, 3.0, CQuaternionDouble());

	cout << "Dump with iterators: ";
	for (CPose3DQuat::iterator it = q.begin(); it != q.end(); ++it)
		cout << *it << " ";
	cout << endl;
}

int main(int argc, char** argv)
{
	try
	{
		TestQuaternions();
		TestQuaternionsIterators();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
