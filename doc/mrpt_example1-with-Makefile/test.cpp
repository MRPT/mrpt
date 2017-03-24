/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

int main()
{
	try
	{
		// The landmark (global) position: 3D (x,y,z)
		CPoint3D L(0,4,2);

		// Robot pose: 2D (x,y,phi)
		CPose2D  R(2,1, DEG2RAD(45.0f) );  

		// Camera pose relative to the robot: 6D (x,y,z,yaw,pitch,roll).
		CPose3D  C(0.5f,0.5f,1.5f ,DEG2RAD(-90.0f),DEG2RAD(0),DEG2RAD(-90.0f)  );  

		// TEST 1. Relative position L' of the landmark wrt the camera
		// --------------------------------------------------------------
		cout << "L: " << L << endl;
		cout << "R: " << R << endl;
		cout << "C: " << C << endl;
		cout << "R+C:" << (R+C) << endl;
		//cout << (R+C).getHomogeneousMatrix();

		CPoint3D L2;
		CTicTac tictac;
		tictac.Tic();
		size_t i,N = 10000;
		for (i=0;i<N;i++)
			L2 = L - (R+C);
		cout << "Computation in: " << 1e6 * tictac.Tac()/((double)N) << " us" << endl;

		cout << "L': " << L2 << endl;

		// TEST 2. Reconstruct the landmark position:
		// --------------------------------------------------------------
		CPoint3D L3 = R + C + L2;
		cout << "R(+)C(+)L' = " << L3 << endl;
		cout << "Should be equal to L = " << L << endl;

		// TEST 3. Distance from the camera to the landmark
		// --------------------------------------------------------------
		cout << "|(R(+)C)-L|= " << (R+C).distanceTo(L) << endl;
		cout << "|L-(R(+)C)|= " << (R+C).distanceTo(L) << endl;

		return 0;
	} catch (exception &e)
	{
		cerr << "EXCEPCTION: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		cerr << "Untyped excepcion!!";
		return -1;
	}
}

