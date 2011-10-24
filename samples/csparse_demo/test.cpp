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
