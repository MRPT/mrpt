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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CVectorTemplate.h>

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/math/ops_matrices.h>

using namespace std;   // Iterators, STL, ...
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::math::detail;  // **** QUITAR CUANDO SE PASE "SVD" ****
using namespace mrpt::utils;


template <class T>
CMatrixTemplateNumeric<T>::CMatrixTemplateNumeric() :  CMatrixTemplate<T>( 1,1 )
{
}


template <class T>
CMatrixTemplateNumeric<T>::CMatrixTemplateNumeric(size_t row , size_t col ) :  CMatrixTemplate<T>( row, col )
{
}


/*---------------------------------------------------------------
						setSize
 ---------------------------------------------------------------*/
template <class T>
void CMatrixTemplateNumeric<T>::setSize(size_t row, size_t col)
{
	CMatrixTemplate<T>::realloc(row,col,true);
}

/*---------------------------------------------------------------
						resize
 ---------------------------------------------------------------*/
template <class T>
void CMatrixTemplateNumeric<T>::resize(size_t row, size_t col)
{
	CMatrixTemplate<T>::realloc(row,col,true);
}

namespace mrpt
{
	namespace math
	{

		/** SVD-Decomposition.
		 *   Extracted from "Numerical recipes in C++" eBook at http://www.nr.com<br>
		  */
		template <class T>
		void svdcmp(T* a[], int m, int n, T w[], T* v[])
		{
			// Given a matrix a[m][n], this routine computes its singular value
			// decomposition, A = U*W*V'.  The matrix U replaces a on output.
			// The diagonal matrix of singular values W is output as a vector w[n].
			// The matrix V  is output as v[n][n].
			// m must be greater or equal to n;  if it is smaller, then a should be
			// filled up to square with zero rows.

			int		flag, i, its, j, jj, k, l, nm;
			double c, f, h, s, x, y, z;
			double anorm = 0.0, g = 0.0, scale = 0.0;

			if (m < n) THROW_EXCEPTION("svdcmp(): Matrix is not augmented with extra rows of zeros");
			std::vector<double> rv1(n);
			const T     EPS= std::numeric_limits<T>::epsilon();


			// Householder reduction to bidiagonal form.
			l = 0;    // added by T. Wang to avoid warning in g++
			nm = 0;   // added by T. Wang to avoid warning in g++
			for (i = 0; i < n; i++)
			{
				l = i + 1;
				rv1[i] = scale*g;
				g = s = scale = 0.0;
				if (i < m) {
					for (k = i; k < m; k++) scale += fabs(a[k][i]);
						if (scale) {
						for (k = i; k < m; k++) {
							a[k][i] /= static_cast<T>(scale);
							s += a[k][i]*a[k][i];
						}
						f = a[i][i];
						g = -SIGN(static_cast<T>(::sqrt(s)),static_cast<T>(f));
						h = f*g - s;
						a[i][i] = static_cast<T>((f - g));
						if (i != n - 1) {
							for (j = l; j < n; j++) {
								for (s  = 0.0, k = i; k < m; k++) s += a[k][i]*a[k][j];
								f = s/h;
								for ( k = i; k < m; k++) a[k][j] += static_cast<T>((f*a[k][i]));
							}
						}
						for (k = i; k < m; k++) a[k][i] *= static_cast<T>(scale);
						}
					}
					w[i] = static_cast<T>((scale*g));
					g = s= scale = 0.0;
					if (i < m && i != n - 1) {
						for (k = l; k < n; k++)  scale += fabs(a[i][k]);
						if (scale) {
						for (k = l; k < n; k++) {
							a[i][k] /= static_cast<T>(scale);
							s += a[i][k]*a[i][k];
						}
						f = a[i][l];
						g = -SIGN(static_cast<T>(::sqrt(s)), static_cast<T>(f));
						h = f*g - s;
						a[i][l] = static_cast<T>((f - g));
						for (k = l; k < n; k++)  rv1[k] = a[i][k]/h;
						if (i != m - 1) {
							for (j = l; j < m; j++) {
								for (s = 0.0, k = l; k < n; k++) s += a[j][k]*a[i][k];
								for (k = l; k < n; k++) a[j][k] += static_cast<T>((s*rv1[k]));
							}
						}
						for (k = l; k < n; k++) a[i][k] *= static_cast<T>(scale);
						}
					}
					anorm = std::max(static_cast<T>(anorm), static_cast<T>(fabs(w[i]) + fabs(rv1[i])));
				}
					/* Accumulation of right-hand transformations.  */
				for (i = n - 1; 0 <= i; i--) {
					if (i < n - 1) {
						if (g) {
						for (j = l; j < n; j++)  v[j][i] = static_cast<T>(((a[i][j]/a[i][l])/g));
							/* Double division to avoid possible underflow: */
						for (j = l; j < n; j++) {
							for (s = 0.0, k = l; k < n; k++) s += a[i][k]*v[k][j];
							for (k = l; k < n; k++)  v[k][j] += static_cast<T>((s*v[k][i]));
						}
						}
						for (j = l; j < n; j++) v[i][j] = v[j][i] = 0.0;
			}
					v[i][i] = 1.0;
					g = rv1[i];
					l = i;
				}
					/* Accumulation of left-hand transformations.   */
				for (i = n - 1; 0 <= i; i--) {
					l = i + 1;
					g = w[i];
					if (i < n - 1) for (j = l; j < n; j++) a[i][j] = 0.0;
					if (g) {
						g = 1.0/g;
						if (i != n - 1) {
						for (j = l; j < n; j++) {
							for (s = 0.0, k = l; k < m; k++) s += a[k][i]*a[k][j];
							f = (s/a[i][i])*g;
							for (k = i; k < m; k++) a[k][j] += static_cast<T>((f*a[k][i]));
						}
						}
						for (j = i; j < m; j++)  a[j][i] *= static_cast<T>(g);
					}
					else {
						for (j = i; j < m; j++) a[j][i] = 0.0;
					}
					a[i][i] += 1.0;   // ++a[i][i]
				}
				/* Diagonalization of the bidiagonal form.  */
				for (k = n - 1; 0 <= k; k--) {        /* Loop over singular values. */
					for (its = 0; its < 30; its++) {    /* Loop over allowed iterations.*/
						flag = 1;
						for (l = k; 0 <= l; l--) {     // Test for splitting:
						nm = l - 1;                 // Note that rv1[0] is always zero

						//if (fabs(rv1[l]) + anorm == anorm) {
						if (fabs(rv1[l]) <= EPS*anorm)
						{
							flag = 0;
							break;
						}
						if ( fabs(w[nm]) + anorm == anorm) break;
						}
						if (flag) {
						c = 0.0;                       /* Cancellation of rv1[l], if l>0:*/
						s = 1.0;
						for (i = l; i <= k; i++) {
						f = s*rv1[i];
						if (fabs(f) + anorm != anorm) {
							g = w[i];
							h = pythag(static_cast<T>(f), static_cast<T>(g));
							w[i] = static_cast<T>(h);
							h = static_cast<T>((1.0/h));
							c = static_cast<T>((g*h));
							s = (-f*h);
							for (j = 0; j < m; j++) {
								y = a[j][nm];
								z = a[j][i];
								a[j][nm] = static_cast<T>((y*c + z*s));
								a[j][i]  = static_cast<T>((z*c - y*s));
							}
						}
						}
					}
					z = w[k];
					if (l == k) {       /* Convergence.  */
						if (z < 0.0) {        /* Singular value is made non-negative. */
						w[k] = static_cast<T>(-z);
						for (j = 0; j < n; j++) v[j][k] = (-v[j][k]);
						}
						break;
					}
					if (its == 29)
					THROW_EXCEPTION("svdcmp(): Not convergence in 30 SVDCMP iterations!");

					x = w[l];               /* Shift from bottom 2-by-2 minor. */
					nm = k - 1;
					y = w[nm];
					g = rv1[nm];
					h = rv1[k];
					f = ((y - z)*(y + z) + (g - h)*(g + h))/(2.0*h*y);
					g = pythag(static_cast<T>(f), static_cast<T>(1.0));
					f = ((x - z)*(x + z) + h*((y/(f + SIGN(static_cast<T>(g), static_cast<T>(f)))) - h))/x;
						/* Next QR transformation:    */
					c = s = 1.0;
					for (j = l; j <= nm; j++) {
						i = j + 1;
						g = rv1[i];
						y = w[i];
						h = s*g;
						g = c*g;
						z = pythag(static_cast<T>(f), static_cast<T>(h));
						rv1[j] = z;
						c = f/z;
						s = h/z;
						f = x*c + g*s;
						g = g*c - x*s;
						h = y*s;
						y = y*c;
						for (jj = 0; jj < n;  jj++) {
						x = v[jj][j];
						z = v[jj][i];
						v[jj][j] = static_cast<T>(x*c + z*s);
						v[jj][i] = static_cast<T>(z*c - x*s);
						}
						z = pythag(static_cast<T>(f), static_cast<T>(h));
						w[j] = static_cast<T>(z);        /* Rotation can be arbitrary if z = 0.*/
						if (z) {
						z = 1.0/z;
						c = f*z;
						s = h*z;
						}
						f = (c*g) + (s*y);
						x = (c*y) - (s*g);
						for (jj = 0; jj < m; jj++) {
						y = a[jj][j];
						z = a[jj][i];
						a[jj][j] = static_cast<T>(y*c + z*s);
						a[jj][i] = static_cast<T>(z*c - y*s);
						}
					}
					rv1[l] = 0.0;
					rv1[k] = f;
					w[k] = static_cast<T>(x);
				}
			}
		}

	} // end namespace math
} // end namespace mrpt


template <class T>
CMatrixTemplateNumeric<T>  CMatrixTemplateNumeric<T>::largestEigenvector(
	T			resolution,
	size_t			maxIterations ,
	int			*out_Iterations,
	float		*out_estimatedResolution ) const
{
	// Apply the iterative Power Method:
	// -------------------------------------
	size_t						i, iter=0, n = CMatrixTemplate<T>::m_Rows;
	CMatrixTemplateNumeric<T>	x,xx;		// The iterative vector
	T							norm,dif;

	// Initially, set to ones for example.
	x.ones(n,1);

	// Iterative loop:
	do
	{
		xx = (*this) * x;

		// Normalize:
		norm = 0;
		for (i=0;i<n;i++)
			norm+= mrpt::utils::square(xx(i,0));
		xx *= static_cast<T>((1.0/::sqrt(norm)));

		// Compute diference between iterations:
		dif = 0;
		for (i=0;i<n;i++)
			dif+=static_cast<T>(( mrpt::utils::square(fabs(xx(i,0)-x(i,0)))));
		dif=static_cast<T>(::sqrt(dif));

		// Set as current estimation:
		x = xx;

		// Iteration counter:
		iter++;

	} while (iter<maxIterations && dif>resolution);

	if (out_Iterations) *out_Iterations=static_cast<int>(iter);
	if (out_estimatedResolution) *out_estimatedResolution=dif;

	// OK:
	return x;
}


template <class T>
void CMatrixTemplateNumeric<T>::laplacian( CMatrixTemplateNumeric<T> &ret ) const
{
	if ( CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols ) THROW_EXCEPTION( "laplacian: Defined for square matrixes only!");

	size_t							i,j,size;

	size = CMatrixTemplate<T>::m_Rows;

	// Compute the "degree" of each node:
	// -------------------------------------
	std::vector<T> deg(size);

	for (i=0;i<size;i++)
	{
		deg[i] = 0;
		for (j=0;j<size;j++)
				deg[i] += CMatrixTemplate<T>::m_Val[j][i];
	}

	// Compute lapplacian
	//   LAPLACIAN = D - W
	// -----------------------------------
	ret.realloc(size,size);

	for(i=0;i<size;i++)
	{
		ret(i,i) = deg[i] - CMatrixTemplate<T>::m_Val[i][i];

		for(j=i+1;j<size;j++)
		{
			ret(i,j) =
			ret(j,i) = -CMatrixTemplate<T>::m_Val[i][j];
		}
	}
}

/** Computes the SVD (Singular Value Decomposition) of the matrix.
  *  If "this" matrix is named A with dimensions M x N, this method computes: <br>
  *			A = U * W * V' <br>
  * <br>
  *  , where U is a M x N column orthogonal matrix, W is a diagonal matrix
  *  containing the singular values, and V is a NxN matrix. <br>
  * This method returns the U matrix, the N elements in the diagonal of W as a vector,
  *  and the matrix V, NOT TRANSPOSED.
  */
template <class T>
void  CMatrixTemplateNumeric<T>::svd(CMatrixTemplateNumeric<T> &U, std::vector<T> &W,CMatrixTemplateNumeric<T> &V) const
{
	// Algorithm from "Numerical recipes in C"
	// void  CMatrixD::svdcmp(double* a[], int m, int n, double w[], double* v[])
	//
	//  a <-- this,
	//    execute svdcmp
	//  U <-- a
	//  W <-- w
	//  V <-- v
	// -----------------------------------------------

	size_t		i,j;
	T			**a, **v;
	T			*w;
	size_t		m = CMatrixTemplate<T>::getRowCount(),n=CMatrixTemplate<T>::getColCount();

	// Copy the matrix content to "a":
	// --------------------------------------
	typedef T* matrix_type_ptr;

	w = new T[n];
	a = new matrix_type_ptr[m];
	v = new matrix_type_ptr[n];
	for (i=0;i<m;i++)	a[i] = new T[n];
	for (i=0;i<n;i++)	v[i] = new T[n];


	for (i=0;i<m;i++)
		for (j=0;j<n;j++)
			a[i][j] = (*this)(i,j);

	// Algorithm
	// --------------------------------------
	svdcmp(a,m,n,w,v);

	// Copy results to matrices classes
	// --------------------------------------
	U.setSize( m,n );
	W.resize( n );
	V.setSize(n,n);

	for (i=0;i<m;i++)
		for (j=0;j<n;j++)
			U(i,j)= a[i][j];

	for (i=0;i<n;i++)
		W[i] = w[i];

	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			V(i,j)=v[i][j];

	// Free
	// --------------------------------------
	for (i=0;i<m;i++)	delete[] a[i];
	for (i=0;i<n;i++)	delete[] v[i];
	delete[]	a;
	delete[]	v;
	delete[]	w;
}

/** combined power and assignment operator
*/
template <class T>
CMatrixTemplateNumeric<T>& CMatrixTemplateNumeric<T>:: operator ^= (const unsigned int& pow)
{
	CMatrixTemplateNumeric<T> temp(*this);

	for (size_t i=2; i <= pow; i++)
		*this = *this * temp;

	return *this;
}

/** Scalar power of all elements to a given power, this is diferent of ^ operator.
	*/
template <class T>
void CMatrixTemplateNumeric<T>::scalarPow(T s)
{
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			CMatrixTemplate<T>::m_Val[i][j] = pow(CMatrixTemplate<T>::m_Val[i][j],s);
}

/** Set all elements to zero
*/
template <class T>
void CMatrixTemplateNumeric<T>::zeros(const size_t row, const size_t col)
{
   setSize(row,col);
   zeros();
}

/** Set all elements to zero
*/
template <class T>
void CMatrixTemplateNumeric<T>::zeros()
{
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			CMatrixTemplate<T>::m_Val[i][j] = 0;
}

/** Set all elements to one
*/
template <class T>
void CMatrixTemplateNumeric<T>::ones(const size_t row, const size_t col)
{
   setSize(row,col);
   ones();
}

/** Set all elements to one
*/
template <class T>
void CMatrixTemplateNumeric<T>::ones()
{
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			CMatrixTemplate<T>::m_Val[i][j] = 1;
}

/** Build an unit matrix.
*/
template <class T>
void CMatrixTemplateNumeric<T>::unit (const size_t row)
{
   setSize(row,row);
   unit();
}

/** Build an unit matrix.
*/
template <class T>
void CMatrixTemplateNumeric<T>::unit()
{
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			CMatrixTemplate<T>::m_Val[i][j] = (i==j) ? 1 : 0;
}

/** Solve the matrix as linear equations system.
*/
template <class T>
CMatrixTemplateNumeric<T> CMatrixTemplateNumeric<T>::solve (const CMatrixTemplateNumeric<T>& v) const
{
	if (!(CMatrixTemplate<T>::m_Rows == CMatrixTemplate<T>::m_Cols && CMatrixTemplate<T>::m_Cols == v.m_Rows))
		THROW_EXCEPTION( "solve:Inconsistent matrices!");

	CMatrixTemplateNumeric<T>	temp(CMatrixTemplate<T>::m_Rows,CMatrixTemplate<T>::m_Cols+v.m_Cols);
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
	{
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			temp.m_Val[i][j] = CMatrixTemplate<T>::m_Val[i][j];
		for (size_t k=0; k < v.m_Cols; k++)
			temp.m_Val[i][CMatrixTemplate<T>::m_Cols+k] = v.m_Val[i][k];
	}
	for (size_t k=0; k < CMatrixTemplate<T>::m_Rows; k++)
	{
		int indx = temp.pivot(k);
		if (indx == -1)
		{
			std::cout << "[solve] Matrix that leaded to error is:" << std::endl << (*this) << std::endl;
			THROW_EXCEPTION( "solve: Singular matrix!");
		}

		T a1 = temp.m_Val[k][k];
		for (size_t j=k; j < temp.m_Cols; j++)
			temp.m_Val[k][j] /= a1;

		for (size_t i=k+1; i < CMatrixTemplate<T>::m_Rows; i++)
		{
			a1 = temp.m_Val[i][k];
			for (size_t j=k; j < temp.m_Cols; j++)
				temp.m_Val[i][j] -= a1 * temp.m_Val[k][j];
		}
	}

	CMatrixTemplateNumeric<T>	s(v.m_Rows,v.m_Cols);
	for (size_t k=0; k < v.m_Cols; k++)
		for (int m=int(CMatrixTemplate<T>::m_Rows)-1; m >= 0; m--)
		{
			s.m_Val[m][k] = temp.m_Val[m][CMatrixTemplate<T>::m_Cols+k];
			for (size_t j=m+1; j < CMatrixTemplate<T>::m_Cols; j++)
				s.m_Val[m][k] -= temp.m_Val[m][j] * s.m_Val[j][k];
		}
	return s;
}

/** Computes the adjunt of matrix.
*/
template <class T>
CMatrixTemplateNumeric<T> CMatrixTemplateNumeric<T>:: adj() const
{
	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		THROW_EXCEPTION( "adj: Adjoin of a non-square matrix.");

	CMatrixTemplateNumeric<T>	temp(CMatrixTemplate<T>::m_Rows,CMatrixTemplate<T>::m_Cols);

	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			temp.m_Val[j][i] = cofact(i,j);
	return temp;
}


/** Computes the cofact.
*/
template <class T>
T CMatrixTemplateNumeric<T>::cofact (size_t row, size_t col) const
{
	size_t i,i1,j,j1;

	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		THROW_EXCEPTION( "cofact: Cofactor of a non-square matrix!");

	if (row > CMatrixTemplate<T>::m_Rows || col > CMatrixTemplate<T>::m_Cols)
		THROW_EXCEPTION( "cofact: Index out of range!");

	CMatrixTemplateNumeric<T> temp (CMatrixTemplate<T>::m_Rows-1,CMatrixTemplate<T>::m_Cols-1);

	for (i=i1=0; i < CMatrixTemplate<T>::m_Rows; i++)
	{
		if (i == row)
			continue;
		for (j=j1=0; j < CMatrixTemplate<T>::m_Cols; j++)
		{
			if (j == col)
				continue;
			temp.m_Val[i1][j1] = CMatrixTemplate<T>::m_Val[i][j];
			j1++;
		}
		i1++;
	}
	T	cof = temp.det();
	if ((row+col)%2 == 1)
		cof = -cof;

	return cof;
}

/** Computes the cond.
*/
template <class T>
T CMatrixTemplateNumeric<T>::cond()
{
	return norm() * inv().norm();
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isDiagonal() const
{
	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		return false;
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			if (i != j && CMatrixTemplate<T>::m_Val[i][j] != T(0))
				return false;
	return true;
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isScalar() const
{
	if (!isDiagonal())
		return false;
	T	v = CMatrixTemplate<T>::m_Val[0][0];
	for (size_t i=1; i < CMatrixTemplate<T>::m_Rows; i++)
		if (CMatrixTemplate<T>::m_Val[i][i] != v)
			return false;
	return true;
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isUnit() const
{
	return (isScalar() && CMatrixTemplate<T>::m_Val[0][0] == T(1));
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isNull() const
{
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
			if (CMatrixTemplate<T>::m_Val[i][j] != T(0))
				return false;
	return true;
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isSymmetric() const
{
	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		return false;
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
		if (CMatrixTemplate<T>::m_Val[i][j] != CMatrixTemplate<T>::m_Val[j][i])
			return false;
	return true;
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isSkewSymmetric() const
{
	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		return false;
	for (size_t i=0; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < CMatrixTemplate<T>::m_Cols; j++)
		if (CMatrixTemplate<T>::m_Val[i][j] != -CMatrixTemplate<T>::m_Val[j][i])
			return false;
	return true;
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isUpperTriangular() const
{
	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		return false;
	for (size_t i=1; i < CMatrixTemplate<T>::m_Rows; i++)
		for (size_t j=0; j < i-1; j++)
		if (CMatrixTemplate<T>::m_Val[i][j] != T(0))
			return false;
	return true;
}

/** Checks for matrix type
  */
template <class T>
bool CMatrixTemplateNumeric<T>::isLowerTriangular() const
{
	if (CMatrixTemplate<T>::m_Rows != CMatrixTemplate<T>::m_Cols)
		return false;

	for (size_t j=1; j < CMatrixTemplate<T>::m_Cols; j++)
		for (size_t i=0; i < j-1; i++)
		if (CMatrixTemplate<T>::m_Val[i][j] != T(0))
			return false;

	return true;
}


/** Round towards minus infinity modifying the matrix
  * (by AJOGD @ JAN-2007)
  */
template <class T>
void CMatrixTemplateNumeric<T>::matrix_floor()
{
	for (size_t i=0;i<CMatrixTemplate<T>::m_Rows;i++)
		for (size_t j=0;j<CMatrixTemplate<T>::m_Cols;j++)
			CMatrixTemplate<T>::m_Val[i][j] = floor(CMatrixTemplate<T>::m_Val[i][j]);
}

/** Round towards minus infinity
  * (by AJOGD @ JAN-2007)
  */
template <class T>
void CMatrixTemplateNumeric<T>::matrix_floor(CMatrixTemplateNumeric<T> &out)
{
	out.setSize(CMatrixTemplate<T>::m_Rows,CMatrixTemplate<T>::m_Cols);
	for (size_t i=0;i<CMatrixTemplate<T>::m_Rows;i++)
		for (size_t j=0;j<CMatrixTemplate<T>::m_Cols;j++)
			out(i,j) = floor(CMatrixTemplate<T>::m_Val[i][j]);
}

/** Round towards plus infinity
  * (by AJOGD @ JAN-2007)
  */
template <class T>
void CMatrixTemplateNumeric<T>::matrix_ceil()
{
	for (size_t i=0;i<CMatrixTemplate<T>::m_Rows;i++)
		for (size_t j=0;j<CMatrixTemplate<T>::m_Cols;j++)
			CMatrixTemplate<T>::m_Val[i][j] = ceil(CMatrixTemplate<T>::m_Val[i][j]);
}

/** Finds the maximum value in the matrix, and returns its position.
  * (by AJOGD @ JAN-2007)
  */
template <class T>
void CMatrixTemplateNumeric<T>::find_index_max_value(size_t &umax, size_t &vmax, T &max_val) const
{
	max_val = CMatrixTemplate<T>::get_unsafe(0,0);
	umax = vmax = 0;
	for (size_t i=0;i<CMatrixTemplate<T>::getRowCount();i++)
	{
		for(size_t j=0;j<CMatrixTemplate<T>::getColCount();j++)
		{
			if (max_val<CMatrixTemplate<T>::get_unsafe(i,j))
			{
				max_val=CMatrixTemplate<T>::get_unsafe(i,j);
				umax=j;
				vmax=i;
			}
		}
	}
}

/** Finds the maximum value in the diagonal of the matrix.
  */
template <class T>
T CMatrixTemplateNumeric<T>::maximumDiagonal() const
{
	if (!CMatrixTemplate<T>::getRowCount() || !CMatrixTemplate<T>::getColCount()) return static_cast<T>(0);
	ASSERT_( CMatrixTemplate<T>::getRowCount() == CMatrixTemplate<T>::getColCount() );
	T max_val = (*this)(0,0);
	for (size_t i=0;i<CMatrixTemplate<T>::getRowCount();i++)
		if (max_val<CMatrixTemplate<T>::get_unsafe(i,i))
				max_val=CMatrixTemplate<T>::get_unsafe(i,i);

	return max_val;
}

/** Finds the minimum value in the matrix, and returns its position.
  * (by AJOGD @ JAN-2007)
  */
template <class T>
void CMatrixTemplateNumeric<T>::find_index_min_value(size_t  &umin, size_t  &vmin, T &min_val) const
{
	ASSERT_(CMatrixTemplate<T>::getRowCount()>0 && CMatrixTemplate<T>::getColCount()>0);
	min_val = (*this)(0,0);
	for (size_t i=0;i<CMatrixTemplate<T>::getRowCount();i++)
	{
		for(size_t j=0;j<CMatrixTemplate<T>::getColCount();j++)
		{
			if (min_val>(*this)(i,j))
			{
				min_val=(*this)(i,j);
				umin=j;
				vmin=i;
			}
		}
	}
}

/** Force symmetry in the matrix
  * (by AJOGD @ JAN-2007)
  */
template <class T>
void CMatrixTemplateNumeric<T>::force_symmetry()
{
	if (CMatrixTemplate<T>::m_Rows!=CMatrixTemplate<T>::m_Cols)
		THROW_EXCEPTION("Error in force_symmetry. The matrix is not square");

	for (size_t i=0;i<CMatrixTemplate<T>::m_Rows-1;i++)
		for(size_t j=i+1;j<CMatrixTemplate<T>::m_Cols;j++)
			CMatrixTemplate<T>::set_unsafe(j,i,  CMatrixTemplate<T>::get_unsafe(i,j) );
}


/** Computes a row with the mean values of each column in the matrix.
  * \sa meanAndStdAll
  */
template <class T>
void CMatrixTemplateNumeric<T>::mean( std::vector<T> &outMeanVector ) const
{
	MRPT_START;

	size_t	nCols = CMatrixTemplate<T>::getColCount();
	size_t	nRows = CMatrixTemplate<T>::getRowCount();
	ASSERT_(nCols!=0);
	ASSERT_(nRows!=0);

	// Compute the mean row:
	outMeanVector.resize(nCols);

	for (size_t c=0;c<nCols;c++)
	{
		T	accum = 0;
		for (size_t r=0;r<nRows;r++) accum += (*this)(r,c);
		outMeanVector[c] = accum / nRows;
	}

	MRPT_END;
}

/** Computes a row with the mean values of each column in the matrix and the associated vector with the standard deviation of each column.
  * \sa mean,meanAndStdAll
  */
template <class T>
void CMatrixTemplateNumeric<T>::meanAndStd(
	std::vector<T> &outMeanVector,
	std::vector<T> &outStdVector ) const
{
	MRPT_START;

	size_t	nCols = CMatrixTemplate<T>::getColCount();
	size_t	nRows = CMatrixTemplate<T>::getRowCount();
	ASSERT_(nCols>=1);
	ASSERT_(nRows>=1);

	// Compute the mean row:
	outMeanVector.resize(nCols);

	for (size_t c=0;c<nCols;c++)
	{
		T	accum = 0;
		for (size_t r=0;r<nRows;r++)
			accum += (*this)(r,c);
		outMeanVector[c] = accum / nRows;
	}

	// Compute the STD:
	outStdVector.resize(nCols);
	for (size_t c=0;c<nCols;c++)
	{
		T	accum = 0, thisMean = outMeanVector[c];
		for (size_t r=0;r<nRows;r++)
			accum += mrpt::utils::square( (*this)(r,c) - thisMean );
		if (nRows>1)
				outStdVector[c] = ::sqrt(accum / (nRows-1));
		else	outStdVector[c] = ::sqrt(accum / nRows);
	}

	MRPT_END;
}

/** Computes the mean and standard deviation of all the elements in the matrix as a whole.
  * \sa mean,meanAndStd
  */
template <class T>
void CMatrixTemplateNumeric<T>::meanAndStdAll(
	T &outMean,
	T &outStd )  const
{
	MRPT_START;

	size_t	nCols = CMatrixTemplate<T>::getColCount();
	size_t	nRows = CMatrixTemplate<T>::getRowCount();
	size_t	c;
	ASSERT_(nCols!=0);
	ASSERT_(nRows!=0);

	// Compute the mean:
	outMean=0;

	for (c=0;c<nCols;c++)
		for (size_t r=0;r<nRows;r++)
			outMean += (*this)(r,c);
	outMean /= nRows*nCols;

	// Compute the STD:
	outStd=0;
	for (c=0;c<nCols;c++)
		for (size_t r=0;r<nRows;r++)
			outStd += mrpt::utils::square( (*this)(r,c) - outMean );
	outStd = ::sqrt(outStd  / (nRows*nCols) );

	MRPT_END;
}

template <class T>
void CMatrixTemplateNumeric<T>::asCol(CMatrixTemplateNumeric<T>	&aux) const
{
	aux.setSize(CMatrixTemplate<T>::m_Cols*CMatrixTemplate<T>::m_Rows,1);

	for (size_t i=0;i<CMatrixTemplate<T>::m_Rows;i++)
		for (size_t j=0;j<CMatrixTemplate<T>::m_Cols;j++)
			aux(j+i*CMatrixTemplate<T>::m_Cols,0)=CMatrixTemplate<T>::m_Val[i][j];
}


template <class T>
void CMatrixTemplateNumeric<T>::asRow(CMatrixTemplateNumeric<T>	&aux) const
{
	aux.setSize(1,CMatrixTemplate<T>::m_Cols*CMatrixTemplate<T>::m_Rows);

	for (size_t i=0;i<CMatrixTemplate<T>::m_Rows;i++)
		for (size_t j=0;j<CMatrixTemplate<T>::m_Rows;j++)
			aux(0,j+i*CMatrixTemplate<T>::m_Cols)=CMatrixTemplate<T>::m_Val[i][j];
}


/** Finds elements whose values are a given number of times above (or below) the mean, in 1D Mahalanobis distance.
  *  This returns two lists with the "row" and "column" indexes (i,j) of those elements m[i][j] such as:
  *    m[i][j] > mean(matrix) + stdTimes·std(matrix)
  *  The elements below the threshold
  *    mean(matrix) - stdTimes·std(matrix)
  *  can also be obtained setting "below" to "true".
  */
template <class T>
void CMatrixTemplateNumeric<T>::findElementsPassingMahalanobisThreshold(
	double					stdTimes,
	std::vector<size_t>		&rowIndexes,
	std::vector<size_t>		&colIndexes,
	bool					below ) const
{
	MRPT_START;

	size_t	nCols = CMatrixTemplate<T>::getColCount();
	size_t	nRows = CMatrixTemplate<T>::getRowCount();

	rowIndexes.clear();
	colIndexes.clear();

	// Find mean and std:
	T	mean,std;
	meanAndStdAll(mean,std);

	// Compute threshold:
	double	thres = mean + stdTimes * std * (below ? (-1):1);

	if (below)
	{
		for (size_t c=0;c<nCols;c++)
			for (size_t r=0;r<nRows;r++)
				if ( (*this)(r,c) < thres )
				{
					rowIndexes.push_back(r);
					colIndexes.push_back(c);
				}
	}
	else
	{
		for (size_t c=0;c<nCols;c++)
			for (size_t r=0;r<nRows;r++)
				if ( (*this)(r,c) > thres )
				{
					rowIndexes.push_back(r);
					colIndexes.push_back(c);
				}
	}

	MRPT_END;
}

/** Returns the sum of a given part of the matrix.
  *  The default value (std::numeric_limits<size_t>::max()) for the last column/row means to sum up to the last column/row.
  * \sa sumAll
  */
template <class T>
T CMatrixTemplateNumeric<T>::sum(
	size_t firstRow ,
	size_t firstCol ,
	size_t lastRow  ,
	size_t lastCol  ) const
{
	MRPT_START;
	size_t	col_1 = CMatrixTemplate<T>::getColCount();
	size_t	row_1 = CMatrixTemplate<T>::getRowCount();

	if (col_1==0 || row_1==0) return 0;

	col_1--; row_1--; // Limits are inclusive.

	if (lastCol!=std::numeric_limits<size_t>::max())
	{
		ASSERT_(lastCol>= firstCol);
		col_1=lastCol;
	}
	if (lastRow!=std::numeric_limits<size_t>::max())
	{
		ASSERT_(lastRow>= firstRow );
		row_1=lastRow;
	}

	ASSERT_( row_1 < CMatrixTemplate<T>::getRowCount() );
	ASSERT_( col_1 < CMatrixTemplate<T>::getColCount() );

	T accum=0;

	for (size_t c=firstCol;c<=col_1;c++)
		for (size_t r=firstRow;r<=row_1;r++)
			accum += CMatrixTemplate<T>::m_Val[r][c];

	return accum;
	MRPT_END;
}

/** Computes:  R = H * C * H^t , where H is this matrix.
  *
  */
template <class T>
void CMatrixTemplateNumeric<T>::multiplyByMatrixAndByTransposeNonSymmetric(
	const CMatrixTemplateNumeric<T>		&C,
	CMatrixTemplateNumeric<T>			&R,
	bool								accumOnOutput,
	bool								substractInsteadOfSum
	) const
{
	MRPT_START;
	ASSERT_( (C.m_Rows == C.m_Cols) && (C.m_Rows == this->CMatrixTemplate<T>::m_Cols) );
	ASSERT_( &C != this );
	ASSERT_( &R != this );
	if (&C == &R) THROW_EXCEPTION("Matrices C and R must be different matrices.");
	size_t						N = CMatrixTemplate<T>::m_Rows;
	size_t						M = CMatrixTemplate<T>::m_Cols;
	size_t						i,j,k,l;
	T							sumAccumInner;

	if (accumOnOutput)
			R.setSize(N,N);
	else 	R.zeros(N,N);

	if (substractInsteadOfSum)  // Duplicating code is faster than branchin' within inner loops!
	{
		for (i=0;i<N;i++)
		{
			for (l=0;l<M;l++)
			{
				sumAccumInner = 0;
				for (k=0;k<M;k++)
					sumAccumInner += CMatrixTemplate<T>::m_Val[i][k] * C.m_Val[k][l];
				for (j=0;j<N;j++)
					R.m_Val[i][j] -= sumAccumInner * CMatrixTemplate<T>::m_Val[j][l];
			}
		}
	}
	else
	{
		for (i=0;i<N;i++)
		{
			for (l=0;l<M;l++)
			{
				sumAccumInner = 0;
				for (k=0;k<M;k++)
					sumAccumInner += CMatrixTemplate<T>::m_Val[i][k] * C.m_Val[k][l];
				for (j=0;j<N;j++)
					R.m_Val[i][j] += sumAccumInner * CMatrixTemplate<T>::m_Val[j][l];
			}
		}
	}

	MRPT_END;
}

// Template instantiations:
template class CMatrixTemplateNumeric<double>;
template class CMatrixTemplateNumeric<float>;
#ifdef HAVE_LONG_DOUBLE
template class CMatrixTemplateNumeric<long double>;
#endif

