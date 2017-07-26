/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// ------------------------------------------------------
//				TestSmartPointers
// ------------------------------------------------------
void TestSmartPointers()
{
	// Create a smart pointer to a CPose3D:
	CPose3D::Ptr p3D = mrpt::make_aligned_shared<CPose3D>();
	p3D->setFromValues(1, 2, 3, DEG2RAD(30), DEG2RAD(-45), DEG2RAD(-30));

	// And a smart pointer to a CPose2D:
	CPose2D::Ptr p2D = mrpt::make_aligned_shared<CPose2D>();  // This is exactly the same
	// than calling
	// mrpt::make_aligned_shared<>()
	p2D->x(4);
	p2D->phi(DEG2RAD(90));

	cout << "p2d: " << *p2D << endl;
	cout << "p3d: " << *p3D << endl;

	// We can cast a smart pointer to a base class:
	CObject::Ptr pBase = p3D;

	// We can cast a base smart pointer to a pointer to a derived class:
	CPose3D::Ptr p3Dbis = std::dynamic_pointer_cast<CPose3D>(pBase);
	cout << "p3d bis: " << *p3Dbis << endl;

	// If the cast does not match the actual classes, an exception is raised:
	cout << "Now we'll try a bad typecasting, so an exception will be raised:"
		 << endl;
	CPose3D::Ptr p3Dbad = std::dynamic_pointer_cast<CPose3D>(p2D);
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
	}
	catch (exception& e)
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
