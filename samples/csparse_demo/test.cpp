/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
