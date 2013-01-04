/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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


#include <Eigen/Dense>
#include <gtest/gtest.h>

using namespace Eigen;
using namespace std;

/** A macro for obtaining the name of the current function:  */
#if defined(__BORLANDC__)
		#define	__CURRENT_FUNCTION_NAME__	__FUNC__
#elif defined(_MSC_VER) && (_MSC_VER>=1300)
		#define	__CURRENT_FUNCTION_NAME__	__FUNCTION__
#elif defined(_MSC_VER) && (_MSC_VER<1300)
		// Visual C++ 6 HAS NOT A __FUNCTION__ equivalent.
#define	__CURRENT_FUNCTION_NAME__	::system::extractFileName(__FILE__).c_str()
#else
		#define	__CURRENT_FUNCTION_NAME__	__PRETTY_FUNCTION__
#endif

#if 0

template <int ColRowOrder>
void do_test_EigenVal4x4_sym_vs_generic_eigen()
{
	typedef Matrix<double,4,4,ColRowOrder> Mat44;

	const double   dat_C1[] = {
		13.737245,10.248641,-5.839599,11.108320,
		10.248641,14.966139,-5.259922,11.662222,
		-5.839599,-5.259922,9.608822,-4.342505,
		11.108320,11.662222,-4.342505,12.121940 };
	const Mat44 C1(dat_C1);  // It doesn't mind the row/col major order since data are symetric

	// Symetric --------------------
	// This solver returns the eigenvectors already sorted.
	Eigen::SelfAdjointEigenSolver<Mat44> eigensolver(C1);
//	MatrixXd eVecs_s = eigensolver.eigenvectors();
//	MatrixXd eVals_s = eigensolver.eigenvalues();

	cout << endl << __CURRENT_FUNCTION_NAME__ << endl
		<< "SelfAdjointEigenSolver:\n"
		<< "eigvecs: " << endl << eigensolver.eigenvectors() << endl
		<< "eigvals: " << endl << eigensolver.eigenvalues() << endl;

	// Generic ---------------------
	Eigen::EigenSolver<Mat44> es(C1, true);
//	MatrixXd eVecs_g = es.eigenvectors().real();
//	MatrixXd eVals_g = es.eigenvalues().real();

	cout << endl
		<< "EigenSolver:\n"
		<< "eigvecs: " << endl << es.eigenvectors() << endl
		<< "eigvals: " << endl << es.eigenvalues() << endl;
}

// Compare the two ways of computing matrix eigenvectors: generic & for symmetric matrices:
TEST(MatricesEigen,EigenVal4x4_sym_vs_generic)
{
	do_test_EigenVal4x4_sym_vs_generic_eigen<Eigen::ColMajor>();
	do_test_EigenVal4x4_sym_vs_generic_eigen<Eigen::RowMajor>();
}

#endif
