/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/CTicTac.h>
#include <Eigen/Dense>
#include <iostream>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>
string myDataDir(MRPT_EXAMPLES_BASE_DIRECTORY + string("math_matrix_example/"));

// ------------------------------------------------------
//				TestChol
// ------------------------------------------------------
void TestChol()
{
	CMatrixFloat A, B;
	A.loadFromTextFile(myDataDir + string("in_for_cholesky.txt"));
	A.chol(B);

	cout << "Cholesky decomposition result:" << endl << B;
}

void TestInitMatrix()
{
	// Initialize a matrix from a C array:
	const double numbers[] = {1, 2, 3, 4, 5, 6};
	CMatrixDouble M(2, 3, numbers);
	cout << "Initialized matrix (I): " << endl << M << endl;

	const double numbers2[] = {0.5, 4.5, 6.7, 8.9, 15.2};
	CVectorDouble v1;
	loadVector(v1, numbers2);
	cout << "Initialized double vector: " << v1 << endl;

	std::vector<int> v2;
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

void TestHCH()
{
	CMatrixFloat H, C, RES;

	cout << "reading H.txt...";
	H.loadFromTextFile(myDataDir + string("H.txt"));
	cout << "ok" << endl;

	cout << "reading C.txt...";
	C.loadFromTextFile(myDataDir + string("C.txt"));
	cout << "ok" << endl;

	// RES = H * C * H'
	mrpt::math::multiply_HCHt(H, C, RES);
	cout << "Saving RES.txt ...";
	RES.saveToTextFile("RES.txt");
	cout << "ok" << endl;

	// The same for a column vector:
	H.loadFromTextFile(myDataDir + string("H_col.txt"));
	cout << "H*C*(H') = " << mrpt::math::multiply_HCHt_scalar(H, C) << endl;
	cout << "Should be= 31.434 " << endl;

	// The same for a row vector:
	H.loadFromTextFile(myDataDir + string("H_row.txt"));
	cout << "Loaded H: " << endl << H;
	cout << "H*C*(H') = " << mrpt::math::multiply_HCHt_scalar(H, C) << endl;
	cout << "Should be= 31.434" << endl;
}

void TestMatrixTemplate()
{
	CTicTac tictac;
	CMatrixDouble M;

	// --------------------------------------
	M.loadFromTextFile(myDataDir + string("matrixA.txt"));
	cout << M << "\n";

	CMatrixDouble eigenVectors;
	std::vector<double> eigenValues;
	M.eig(eigenVectors, eigenValues);
	cout << "eigenVectors:\n"
		 << eigenVectors << "\n Eigenvalues:\n"
		 << eigenValues;

	CMatrixDouble D;
	D.setDiagonal(eigenValues);

	CMatrixDouble RES;
	RES = M.asEigen() * D.asEigen() * M.transpose();
	cout << "RES:\n" << RES;
}

void TestMatrices()
{
	CMatrixFloat m, l;
	CTicTac tictac;
	double t;

	m.setSize(4, 4);
	m(0, 0) = 4;
	m(0, 1) = -2;
	m(0, 2) = -1;
	m(0, 3) = 0;
	m(1, 0) = -2;
	m(1, 1) = 4;
	m(1, 2) = 0;
	m(1, 3) = -1;
	m(2, 0) = -1;
	m(2, 1) = 0;
	m(2, 2) = 4;
	m(2, 3) = -2;
	m(3, 0) = 0;
	m(3, 1) = -1;
	m(3, 2) = -2;
	m(3, 3) = 4;

	cout << "Matrix:\n" << m << endl;

	// I/O test through a text file:
	m.saveToTextFile("matrix1.txt");
	tictac.Tic();
	l.loadFromTextFile(myDataDir + string("matrix1.txt"));
	t = tictac.Tac();
	cout << "Read (text file) in " << 1e6 * t << "us:\n" << l << endl;
	mrpt::math::laplacian(m, l);

	cout << "Laplacian:\n" << l << endl;
}

void TestCov()
{
	// Initialize a matrix from a C array:
	const double numbers[] = {1, 2, 3, 10, 4, 5, 6, 14, 10, -5, -3, 1};
	CMatrixDouble Mdyn(4, 3, numbers);
	CMatrixFixed<double, 4, 3> Mfix(numbers);

	vector<CVectorDouble> samples(4);
	for (size_t i = 0; i < 4; i++)
	{
		samples[i].resize(3);
		for (size_t j = 0; j < 3; j++) samples[i][j] = Mdyn(i, j);
	}

	cout << "COV (vector of vectors): " << endl
		 << mrpt::math::covVector<vector<CVectorDouble>, Eigen::MatrixXd>(
				samples)
		 << endl;
	cout << "COV (mat fix): " << endl << mrpt::math::cov(Mfix) << endl;
	cout << "COV (mat dyn): " << endl << mrpt::math::cov(Mdyn) << endl;
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
		TestMatrices();
		TestHCH();
		TestChol();
		TestCov();

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
