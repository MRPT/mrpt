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

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// ------------------------------------------------------
//				TestSmartPointers
// ------------------------------------------------------
void TestSmartPointers()
{
	// Create a smart pointer to a CPose3D:
	CPose3DPtr	p3D  = CPose3D::Create();
	p3D->setFromValues( 1, 2, 3, DEG2RAD(30),DEG2RAD(-45),DEG2RAD(-30)  );

	// And a smart pointer to a CPose2D:
	CPose2DPtr	p2D  = CPose2DPtr( new CPose2D() );  // This is exactly the same than calling ::Create()
	p2D->x( 4 );
	p2D->phi( DEG2RAD(90) );

	cout << "p2d: " << *p2D << endl;
	cout << "p3d: " << *p3D << endl;

	// We can cast a smart pointer to a base class:
	CObjectPtr pBase = p3D;

	// We can cast a base smart pointer to a pointer to a derived class:
	CPose3DPtr p3Dbis = CPose3DPtr( pBase );
	cout << "p3d bis: " << *p3Dbis << endl;

	// If the cast does not match the actual classes, an exception is raised:
	cout << "Now we'll try a bad typecasting, so an exception will be raised:" << endl;
	CPose3DPtr p3Dbad = CPose3DPtr( p2D );
	cout << "p3d bad: " << *p3Dbad << endl;  // Shouldn't arrive here!
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestSmartPointers();

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

