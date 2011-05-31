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
