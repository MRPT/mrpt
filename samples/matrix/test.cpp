/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/math.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("matrix/") );


// ------------------------------------------------------
//				TestChol
// ------------------------------------------------------
// JL: It seems there's a bug in MSVC9 with this code, in this file... it works fine if alone. (WTF?)
#if 0
void TestChol()
{
	CMatrixFloat	A,B;
	A.loadFromTextFile( myDataDir+string("in_for_cholesky.txt") );
	A.chol(B);

	cout << "Cholesky decomposition result:" << endl << B;
}
#endif

void TestInitMatrix()
{
	// Initialize a matrix from a C array:
	const double numbers[] = {
		1,2,3,
		4,5,6 };
	CMatrixDouble   M(2,3,numbers);
	cout << "Initialized matrix (I): " << endl << M << endl;

	const double numbers2[] = { 0.5, 4.5, 6.7, 8.9, 15.2 };
	CVectorDouble  v1;
	loadVector(v1, numbers2);
	cout << "Initialized double vector: " << v1 << endl;

	vector_int  v2;
	loadVector(v2, numbers2);
	cout << "Initialized int vector: " << v2 << endl;

/*	// I/O Test
	CMatrixD  B(M);
	CFileOutputStream("mat.bin") << B;
	CMatrixD  A;
	CFileInputStream("mat.bin") >> A;
	cout << "B:" << endl << B;
	cout << "A:" << endl << A;
*/
}

// ------------------------------------------------------
//				TestHCH
// ------------------------------------------------------
void TestHCH()
{
	CMatrixFloat		H,C,RES;

	cout << "reading H.txt...";
	H.loadFromTextFile(myDataDir+string("H.txt"));
	cout << "ok"<<endl;

	cout << "reading C.txt...";
	C.loadFromTextFile(myDataDir+string("C.txt"));
	cout << "ok"<<endl;

	// RES = H * C * ~H
	H.multiply_HCHt(C, RES);
	cout << "Saving RES.txt ...";
	RES.saveToTextFile("RES.txt");
	cout << "ok"<<endl;

	// The same for a column vector:
	H.loadFromTextFile(myDataDir+string("H_col.txt"));
	cout << "H*C*(~H) = " << H.multiply_HCHt_scalar(C) << endl;
	cout << "Should be= 31.434 "<< endl;

	// The same for a row vector:
	H.loadFromTextFile(myDataDir+string("H_row.txt"));
	cout << "Loaded H: "  << endl << H;
	cout << "H*C*(~H) = " << H.multiply_HCHt_scalar(C) << endl;
	cout << "Should be= 31.434"<< endl;

	CMatrixFixedNumeric<double,1,5>  Hfix;
	Hfix.loadFromTextFile(myDataDir+string("H_row.txt"));
	cout << "Again, loaded as a fixed matrix: "  << endl << Hfix;
}

// ------------------------------------------------------
//				TestMatrixs
// ------------------------------------------------------
void TestMatrixTemplate()
{
	CTicTac		tictac;
	CMatrixDouble	M,Z,D,RES;

	// --------------------------------------
	M.loadFromTextFile(myDataDir+string("matrixA.txt"));
	cout << M << "\n";

	M.eigenVectors(Z,D);
//	cout << "Z:\n" << Z << "D:\n" << D;

	int	I,N = 1000;
	tictac.Tic();
	for (I =0;I<N;I++) RES= Z * D * (~Z);
	printf("Operation 'RES= Z * D * (~Z)' done in %.03fus\n",1e6*tictac.Tac()/N);

//	cout << "RES (1):\n" << RES;

	tictac.Tic();
	for (I =0;I<N;I++) Z.multiply_HCHt(D,RES);
	printf("Operation 'Z.multiply_HCHt(D,RES)' done in %.03fus\n",1e6f*tictac.Tac()/N);

//	cout << "RES (2):\n" << RES;

	CMatrixDouble			Q(M);
//	cout << "A:\n" << Q;

}

// ------------------------------------------------------
//				TestEigenvector
// ------------------------------------------------------
void TestEigenvector()
{
	CMatrixFloat		A,eigen;			// Adjacency matrix

	// -------
	CMatrixFloat		Z,D;
	A.loadFromTextFile(myDataDir+string("matrixA.txt"));
	printf("A matrix loaded:\n");
	cout << A << "\n";

	A.eigenVectors(Z,D);
	cout << "Z:\n" << Z << "D:\n" << D;

	D = D.array().sqrt().matrix();
	Z= Z*D;
	cout << "Z:\n" << Z;


	// -------

/*
	CTicTac			tictac;
	int				iters;
	float			estRes;
	double			T;

	FILE	*f=mrpt::system::os::fopen("eigen_times.txt","wt");

	for (int N=100;N<3000;N+=100)
	{
		// Generate random matrix:
		A.setSize(N,N);
		for (int i=0;i<N;i++)
			for (int j=0;j<N;j++)
				A(i,j)= double(rand()) / double(RAND_MAX);

		tictac.Tic();
		eigen = A.largestEigenvector(1e-3f,10, &iters, &estRes );
		T = tictac.Tac();
		printf("Size=%u  Iters=%u Res=%e\tT=%.03fms\n",N,iters,estRes,1000.0f*T);

//		A.saveToTextFile("A.txt");
//		eigen.saveToTextFile("eigen.txt");
		fprintf(f,"%u %e\n",N,T);
	}

	os::fclose(f);
*/

}

// ------------------------------------------------------
//				TestMatrixs
// ------------------------------------------------------
void TestMatrixs()
{
	CMatrixFloat		m,l;
	CTicTac		tictac;
	double		t;

	m.setSize(4,4);
	m(0,0)= 4;
	m(0,1)=-2;
	m(0,2)=-1;
	m(0,3)= 0;
	m(1,0)= -2;
	m(1,1)= 4;
	m(1,2)= 0;
	m(1,3)= -1;
	m(2,0)= -1;
	m(2,1)= 0;
	m(2,2)= 4;
	m(2,3)= -2;
	m(3,0)= 0;
	m(3,1)= -1;
	m(3,2)= -2;
	m(3,3)= 4;

	cout << "Matrix:\n" << m << endl;

	// I/O test through a text file:
	m.saveToTextFile("matrix1.txt");
	tictac.Tic();
	l.loadFromTextFile(myDataDir+string("matrix1.txt"));
	t=tictac.Tac();
	cout << "Read (text file) in " << 1e6*t << "us:\n" << l << endl;

	m.laplacian(l);

	cout << "Laplacian:\n" << l << endl;

	CMatrixFloat		Z,D;
	m.eigenVectors(Z,D);

	cout << "Eigenvectors: M = Z * D * Z':\n Z=\n" << Z << endl;
	cout << "D=\n" << D << endl;

	cout << "Z * D * Z'=\n" << Z * D * (~Z) << endl;

}

void TestCov()
{
	// Initialize a matrix from a C array:
	const double numbers[] = {
		1,2,3,
		10,4,5,
		6,14,10,
		-5,-3,1 };
	CMatrixDouble   Mdyn(4,3,numbers);
	CMatrixFixedNumeric<double,4,3> Mfix(numbers);

	vector<CVectorDouble> samples(4);
	for (size_t i=0;i<4;i++)
	{
		samples[i].resize(3);
		for (size_t j=0;j<3;j++)
			samples[i][j]=Mdyn(i,j);
	}

	cout << "COV (vector of vectors): " << endl << mrpt::math::covVector<vector<CVectorDouble>,Eigen::MatrixXd>(samples)  << endl;
	cout << "COV (mat fix): " << endl << mrpt::math::cov(Mfix)  << endl;
	cout << "COV (mat dyn): " << endl << mrpt::math::cov(Mdyn)  << endl;
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestInitMatrix();
		TestMatrixTemplate();
        TestMatrixs();
		TestHCH();
		//TestChol();
		TestEigenvector();
		TestCov();

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




