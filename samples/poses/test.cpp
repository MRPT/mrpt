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

#include <mrpt/poses.h>
#include <mrpt/utils.h>

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;


// ------------------------------------------------------
//				TestPosePDFOperations
// ------------------------------------------------------
void TestPosePDFOperations()
{
	CPointPDFGaussian		p1,p2,p;

	p1.mean = CPoint3D(0, -0.12, 0);
	p2.mean = CPoint3D(0, -0.1, 0);

	p1.cov.zeros();
	p1.cov(0,0)=0.06f;    p1.cov(0,1)=0.002f;
	p1.cov(1,0)=0.002f;   p1.cov(1,1)=0.02f;
	p1.cov(2,2)=0.0002f;

	p2.cov.zeros();
	p2.cov(0,0)=0.02f;    p2.cov(0,1)=-0.004f;
	p2.cov(1,0)=-0.004f;   p2.cov(1,1)=0.01f;
	p2.cov(2,2)=0.0002f;

	// Integral of Product of gaussians:
	CTicTac		tictac;
	double		v;

	tictac.Tic();
	for (int i=0;i<10000;i++)
		v = p1.productIntegralWith(p2);

	printf("Time for computing: %.04fus\n", tictac.Tac()*1e+6f/10000);

	printf("product p1,p2 -> %f\n", v );
	printf("product p2,p1 -> %f\n", p2.productIntegralNormalizedWith(p1) );

	// Bayesian fusion:
	p.bayesianFusion(p1,p2);
	p.saveToTextFile("BayesFusion.txt");

	cout << "Bayesian fusin of p1 & p2: " << endl;
	cout << " MEAN: " << p.mean << " COV:" << endl << p.cov << endl;

}

// ------------------------------------------------------
//				TestPoseComposition
// ------------------------------------------------------
void TestPoseComposition()
{
	CTicTac		tictac;

// ---------------------------------------------------------------
	CPose3D		A(0,0,0), B(1,1,0,DEG2RAD(45),0,0), C;

	C = A - B;

	cout << "A:\n" << A << endl;
	cout << "B:\n" << B << endl;
	cout << "C:\n" << C << endl;

// ---------------------------------------------------------------
	CPose2D				p(0.5f,0.2f,DEG2RAD(10.0f) );

	CPoses2DSequence		seq;
	CPose2D		a(1,2,(float) DEG2RAD(0) );
	CPose2D		b(2,3, (float) DEG2RAD(45));
	CPose2D		D;

	CPose2D		x(1,0,(float) DEG2RAD(0) );
	CPose2D		y(1,0, (float) DEG2RAD(45));

	cout << "a= " << a << endl;
	cout << "b= " << b << endl;

	// ------------------------------------------
	tictac.Tic();
	D = a+b;
	printf("%f us\t", tictac.Tac()*1e6);
	cout << "a+b= " << D << endl;
	// ------------------------------------------
	tictac.Tic();
	D = b-a;
	printf("%f us\t", tictac.Tac()*1e6);
	cout << "b-a= " << D << endl;
	// ------------------------------------------
	tictac.Tic();
	D = a + (b-a);
	printf("%f us\t", tictac.Tac()*1e6);
	cout << "a + (b-a)= " << D << endl;
	// ------------------------------------------
	seq.appendPose( y );
	cout << "last= " << seq.absolutePoseAfterAll() << endl;
	seq.appendPose( y );
	cout << "last= " << seq.absolutePoseAfterAll() << endl;
	seq.appendPose( x );
	cout << "last= " << seq.absolutePoseAfterAll() << endl;


	seq.getPose(0,D);
	cout << "Pose[0] in seq.= " << D << endl;
	seq.getPose(1,D);
	cout << "Pose[1] in seq.= " << D << endl;
	seq.getPose(2,D);
	cout << "Pose[2] in seq.= " << D << endl;

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestPoseComposition();
		TestPosePDFOperations();

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

