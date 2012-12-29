/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

