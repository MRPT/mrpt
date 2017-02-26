/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CSparseMatrix.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;


void ExampleCSparse()
{
     // Initialize a 10x10 sparse matrix:
	CSparseMatrix SM(10,10);

	// With 2 dense blocks of 6x6 and 4x4:
	const CMatrixDouble COV1 = mrpt::random::randomGenerator.drawDefinitePositiveMatrix(6, 0.2);
	const CMatrixDouble COV2 = mrpt::random::randomGenerator.drawDefinitePositiveMatrix(4, 0.2);
	SM.insert_submatrix(0,0, COV1);
	SM.insert_submatrix(6,6, COV2);

	// Get as a dense matrix just for displaying to console:
	CMatrixDouble M;
	SM.get_dense(M);
	cout << "M (as dense):\n" << M;


	cout << "Saving to sparse_demo1.txt...\n";
	SM.saveToTextFile_sparse("sparse_demo1.txt");

	// Compress from the triplet to the column-compressed form:
     cout << "Compressing as CCS...\n";
     SM.compressFromTriplet();

	cout << "Saving to sparse_demo2.txt...\n";
	SM.saveToTextFile_sparse("sparse_demo2.txt");

	// Compute the Cholesky decomposition:
	CSparseMatrix::CholeskyDecomp  Chol(SM);

     // And display the L factor:
	const CMatrixDouble L = Chol.get_L();
	cout << "L:\n" << L << endl;
}

int main(int argc, char **argv)
{
	try
	{
	     ExampleCSparse();
		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
