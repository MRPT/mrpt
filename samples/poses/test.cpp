/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/**
 * samples/poses example.
 * Shows the common PDF and 2D/3D pose manipulation operations.
 */

#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


// ------------------------------------------------------
//				TestPosePDFOperations
// ------------------------------------------------------
void TestPosePDFOperations()
{
	CPointPDFGaussian		p1,p2,p;

    /**
     * Construct the CPointPDFGaussian instances
     * initialize and pass a mean to the p1, p2 instances and a covariance
     * matrix. The latter is done by explicitly adding the cov. matrix elemnts
     */
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

	cout << "Bayesian fusing of p1 & p2: " << endl;
	cout << " MEAN: " << p.mean << " COV:" << endl << p.cov << endl;

}

// ------------------------------------------------------
//				TestPoseComposition
// ------------------------------------------------------
void TestPoseComposition()
{
	CTicTac		tictac;

// ---------------------------------------------------------------
    /**
     * CPose3D default constructor method takes the arguments in the 
     * (X, Y, Z, YAW=0, PITCH=0, ROLL=0) format. The angles are optional
     */
	CPose3D		A(0,0,0), B(1,1,0,DEG2RAD(45),0,0), C;

	C = A - B;

	cout << "A:\n" << A << endl;
	cout << "B:\n" << B << endl;
	cout << "C:\n" << C << endl;

// ---------------------------------------------------------------
	CPose2D				p(0.5f,0.2f,DEG2RAD(10.0f) );
    
    // stores a sequence of relative, incremental 2D poses
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
    
    /**
     * Incrementally update the pose of the "robot".
     * Appending the pose is equivalent to a position/rotation change with
     * regards to the body-fixed frame of reference
     * For more information refer to:
     * http://reference.mrpt.org/stable/_c_poses2_d_sequence_8h_source.html
     */
	seq.appendPose( y );
	cout << "last= " << seq.absolutePoseAfterAll() << endl;
	seq.appendPose( y );
	cout << "last= " << seq.absolutePoseAfterAll() << endl;
	seq.appendPose( x );
	cout << "last= " << seq.absolutePoseAfterAll() << endl;


    // play the poses from the beginning using the getPose method
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

