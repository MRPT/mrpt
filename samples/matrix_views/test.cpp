/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace std;

// ------------------------------------------------------
//                  TestMatrixViews
// ------------------------------------------------------
void TestMatrixViews()
{
	CMatrixDouble  C(4,2);
	randomGenerator.drawGaussian1DMatrix(C,0,1);

	cout << "C: " << endl << C << endl;

	CMatrixViewTranspose<CMatrixDouble>  Ct(C);  // Transpose view of C:
	cout << "C^t: " << endl << Ct << endl;
	cout << "C^t size: " << Ct.size() << endl;

	CMatrixDouble  A(2,2);
	randomGenerator.drawGaussian1DMatrix(A,0,1);

	CTimeLogger  tims;


	CMatrixDouble RES;

	// A * C^t -----------------------
	size_t N = 10000;

	tims.enter("mult_AC_trans");
	for (size_t i=0;i<N;i++)
		RES.multiply(A,Ct);
	tims.leave("mult_AC_trans");

	tims.enter("mult_ACt");
	for (size_t i=0;i<N;i++)
		RES.multiply_ABt(A,C);
	tims.leave("mult_ACt");

	cout << "A * C^t" << endl << RES << endl;


	// submatrixview -----------------------
	CMatrixDouble  M(10,10);
	randomGenerator.drawGaussian1DMatrix(M,0,1);

	cout << "M: " << endl << M << endl;

	CSubmatrixView<CMatrixDouble,3,2>  Msub(M,5,6);  // Msub is M([5,6,7],[6,7])
	cout << "Submatrix M([5,6,7],[6,7]): " << endl << Msub << endl;

	Msub = CMatrixFixedNumeric<double,3,2>(); // Set to zeros

	cout << "M: " << endl << M << endl;


	// CArbitrarySubmatrixView -----------------------
	CMatrixDouble  S(100,100);
	randomGenerator.drawGaussian1DMatrix(S,0,1);

	vector_size_t idxs;
	for (size_t i=0;i<8;i++) idxs.push_back(randomGenerator.drawUniform32bit() % size(S,1) );

	CArbitrarySubmatrixView<CMatrixDouble>  Ssub(S,idxs);
	//cout << "Submatrix S(idx,idx): " << endl << Ssub << endl;

	{
		CMatrixDouble  A(5,idxs.size());
		randomGenerator.drawGaussian1DMatrix(A,0,1);

		tims.enter("mult_AsubS");
		for (size_t i=0;i<N;i++)
			RES.multiply(A,Ssub);
		tims.leave("mult_AsubS");

		tims.enter("mult_AS_extracting");
		for (size_t i=0;i<N;i++)
		{
			CMatrixDouble Ssub2;
			S.extractSubmatrixSymmetrical(idxs,Ssub2);
			RES.multiply(A,Ssub2);
		}
		tims.leave("mult_AS_extracting");

	}

}

// ------------------------------------------------------
//                        MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestMatrixViews();
		return 0;
	} catch (exception &e)
	{
		cerr << "EXCEPCTION: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		cerr << "Untyped excepcion!!";
		return -1;
	}
}


