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

#include <mrpt/base.h>

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

