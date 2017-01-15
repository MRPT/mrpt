/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
