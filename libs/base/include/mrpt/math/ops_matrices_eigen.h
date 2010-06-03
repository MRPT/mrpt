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
#ifndef  mrpt_ops_matrices_eigen_H
#define  mrpt_ops_matrices_eigen_H

#include <mrpt/math/math_frwds.h>  // Fordward declarations
#include <mrpt/math/matrices_metaprogramming.h>  // TMatrixProductType, ...


/** \file ops_matrices_eigen.h
  * This file implements eigenvalues/eigenvectors for any kind of matrix-like object.
  */

namespace mrpt
{
	namespace math
	{
	namespace detail
	{
		/** @name Matrix eigenvectors
		   @{ */

		template <class MAT,class ARR>  void  tred2(MAT &a, const size_t nn, ARR &d, ARR &e);
		template <class MAT,class ARR>  void  tqli(ARR &d, ARR &e, const size_t nn, MAT &z);

		/** eigenVectorsMatrix
		  * \param eVecs if set to NULL, only compute eigenvalues
		  */
		template <class MATRIX1,class MATRIX2,class VECTOR1> void
		eigenVectorsMatrix(const MATRIX1 &M,MATRIX2 *eVecs,VECTOR1 &eVals )
		{
			ASSERTMSG_(M.isSquare(), "eigenvalues can be computed for square matrices only.")
			bool doEVecs = (eVecs!=NULL);
			const size_t N = M.getColCount();

			// TODO: (JL) Rewrite this whole thing, please!
			typedef typename MATRIX1::value_type T;
			MAT_TYPE_SAMESIZE_OF(MATRIX1)  a; // Auxiliary matrix a, of the same size than "M"
			a.setSize(M.getRowCount(),M.getColCount());

			// d, e: Auxiliary vectors of the same length than the matrix dims:
			ARRAY_TYPE_SAMESIZE_ROWS_OF(MATRIX1) d,e;
			d.resize(M.getRowCount());
			e.resize(M.getRowCount());

			MRPT_START

			// Check for symmetry
#ifdef _DEBUG
			for (size_t i=0;i<N;i++)
				for (size_t j=i;j<N;j++)
					if (M.get_unsafe(i,j)!=M.get_unsafe(j,i))
					{
						THROW_EXCEPTION(format("eigenVectors: The matrix is not symmetric! m(%lu,%lu)=%.16e != m(%lu,%lu)=%.16e\n",
							static_cast<unsigned long>(i),static_cast<unsigned long>(j), static_cast<double> ( M.get_unsafe(i,j) ),
							static_cast<unsigned long>(j),static_cast<unsigned long>(i), static_cast<double> ( M.get_unsafe(j,i) )) )
					}
#endif

			// Copy the matrix content to "a":
			// --------------------------------------
			a = M;

			// Algorithm
			// --------------------------------------
			tred2( a, N, d, e);
			tqli(d,e,N,a);

			// "d" -> eigenvalues
			// "a" -> eigenvectors as columns:

			// SORT: Build a list of the N index in
			//   ascending order of eigenvalue:
			// --------------------------------------
			std::vector<unsigned int>	indxs(N+1);
			std::vector<bool>	already(N+1, false);

			for (size_t i=0;i<N;i++)
			{
				size_t	minIndx = std::numeric_limits<size_t>::max();
				for (size_t j=0;j<N;j++)
					if (!already[j])
					{
						if (minIndx==std::numeric_limits<size_t>::max())		minIndx = j;
						else
							if (d[j]<d[minIndx])	minIndx = j;
					}

				// The i'th sorted element:
				indxs[i] = static_cast<unsigned int> ( minIndx );
				already[minIndx] = true;
			}

			for (size_t i=0;i<N;i++)
				ASSERT_(already[i]);

			// Prepare output:
			eVals.resize(N);
			if (doEVecs) eVecs->setSize(N,N);

			// Copy results to matrices classes
			for (size_t i=0;i<N;i++)
			{
				eVals[i]=d[indxs[i]]; // Copy eigenvalue
				if (doEVecs)
					for (size_t j=0;j<N;j++)
						eVecs->get_unsafe(i,j) = a.get_unsafe(i,indxs[j]);
			}

			MRPT_END_WITH_CLEAN_UP( std::cout << "[eigenVectors] The matrix leading to exception is:" << std::endl << M << std::endl; )
		}


		// Special case for 2x2 matrices:
		template <class MATRIX1,class MATRIX2,class VECTOR1> void
		eigenVectorsMatrix_special_2x2(const MATRIX1 &M,MATRIX2 *eVecs,VECTOR1 &eVals )
		{
			typedef typename MATRIX1::value_type T;
			const T t = M.get_unsafe(0,0) + M.get_unsafe(1,1);
			const T de = M.get_unsafe(0,0)*M.get_unsafe(1,1)-M.get_unsafe(1,0)*M.get_unsafe(0,1);
			eVals.resize(2);
			// Eigenvalues, in ascending order:
			const T disc = ::sqrt(0.25*t*t-de);
			eVals[0] =0.5*t - disc;
			eVals[1] =0.5*t + disc;
			if (!eVecs) return;
			eVecs->setSize(2,2);
			const T c = M.get_unsafe(1,0);
			const T b = M.get_unsafe(0,1);
			if (c!=0)
			{
				eVecs->get_unsafe(0,0)=eVals[0]-M.get_unsafe(1,1);
				const T ev1n = 1.0/hypot( eVecs->get_unsafe(0,0), c );
				eVecs->get_unsafe(0,0)*=ev1n;
				eVecs->get_unsafe(1,0)=c*ev1n;

				eVecs->get_unsafe(0,1)=eVals[1]-M.get_unsafe(1,1);
				const T ev2n = 1.0/hypot( eVecs->get_unsafe(0,1), c );
				eVecs->get_unsafe(0,1)*=ev2n;
				eVecs->get_unsafe(1,1)=c*ev2n;
			}
			else if (b!=0)
			{
				eVecs->get_unsafe(1,0)=eVals[0]-M.get_unsafe(0,0);
				const T ev1n = 1.0/hypot( eVecs->get_unsafe(1,0), b );
				eVecs->get_unsafe(1,0)*=ev1n;
				eVecs->get_unsafe(0,0)=b*ev1n;

				eVecs->get_unsafe(1,1)=eVals[1]-M.get_unsafe(0,0);
				const T ev2n = 1.0/hypot( eVecs->get_unsafe(1,1), b );
				eVecs->get_unsafe(1,1)*=ev2n;
				eVecs->get_unsafe(0,1)=b*ev2n;
				// Normalize eigenvectors:
			} else
			{
				eVecs->get_unsafe(0,0)=1;
				eVecs->get_unsafe(1,0)=0;
				eVecs->get_unsafe(0,1)=0;
				eVecs->get_unsafe(1,1)=1;
			}
		}


		// Auxiliary funcs:
		template <class T> inline T  pythag(const T a, const T b) {
			T at, bt, ct;
			return static_cast<T>(( ((at = fabs(a)) > (bt = fabs(b)) ?
					(ct = bt/at, at*(::sqrt(1.0+ct*ct))) :
					(bt ? (ct = at/bt, bt*(::sqrt(1.0+ct*ct))): 0)) ) );
		}

		template <class T> inline T  SIGN(T a,T b) { return static_cast<T>(( ((b) >= 0 ? fabs(a) : -fabs(a)) )); }

		/**
			Householder reduction of a real, symmetric matrix a[1..n][1..n]. On output, a is replaced
			by the orthogonal matrix Q reflecting the transformation. d[1..n] returns the diagonal elements
			of the tridiagonal matrix, and e[1..n] the off-diagonal elements, with e[1]=0. Several
			statements, as noted in comments, can be omitted if only eigenvalues are to be found, in which
			case a contains no useful information on output. Otherwise they are to be included.
		*/
		template <class MAT,class ARR>
		void  tred2(MAT &a, const size_t n, ARR &d, ARR &e)
		{
			typename MAT::value_type scale,hh,h,g,f;

			for (size_t i=n-1;i!=0;i--)
			{
				//size_t l=i-1;
				h=scale=0.0;
				if (i > 1)
				{
					for (size_t k=0;k<i;k++)
						scale += fabs(a.get_unsafe(i,k));
					if (scale == 0)				// Skip transformation.
						e[i]=a.get_unsafe(i,i-1);
					else
					{
						for (size_t k=0;k<i;k++)
						{
							a.get_unsafe(i,k) /= scale;		// Use scaled a's for transformation.
							h += a.get_unsafe(i,k)*a.get_unsafe(i,k);	// Form sigma in h.
						}
						f=a.get_unsafe(i,i-1);
						g=(f >= 0 ? -::sqrt(h) : (::sqrt(h)));
						e[i]=scale*g;
						h -= f*g;						// Now h is equation (11.2.4).
						a.get_unsafe(i,i-1)=f-g;					// Store u in the ith row of a.
						f=0;
						for (size_t j=0;j<i;j++)
						{
							a.get_unsafe(j,i)=a.get_unsafe(i,j)/h;			// Store u=H in ith column of a.
							g=0.0;						// Form an element of A . u in g.
							for (size_t k=0;k<=j;k++)
								g += a.get_unsafe(j,k)*a.get_unsafe(i,k);
							for (size_t k=j+1;k<i;k++)
								g += a.get_unsafe(k,j)*a.get_unsafe(i,k);
							e[j]=g/h;					// Form element of p in temporarily unused element of e.
							f += e[j]*a.get_unsafe(i,j);
						}
						hh=f/(h+h);						// Form K, equation (11.2.11).
						for (size_t j=0;j<i;j++)
						{								// Form q and store in e overwriting p.
							f=a.get_unsafe(i,j);
							e[j]=g=e[j]-hh*f;
							for (size_t k=0;k<=j;k++)			// Reduce a, equation (11.2.13).
								a.get_unsafe(j,k) -= (f*e[k]+g*a.get_unsafe(i,k));
						}
					}
				} else
					e[i]=a.get_unsafe(i,i-1);
				d[i]=h;
			}

			/* Next statement can be omitted if eigenvectors not wanted */
			d[0]=0;
			e[0]=0;

			/* Contents of this loop can be omitted if eigenvectors not
			wanted except for statement d[i]=a[i][i]; */
			for (size_t i=0;i<n;i++)	// Begin accumulation of transformationmatrices
			{
				if (d[i]) // This block skipped when i=1.
					for (size_t j=0;j<i;j++)
					{
						g=0.0;
						for (size_t k=0;k<i;k++) // Use u and u=H stored in a to form P.Q.
							g += a.get_unsafe(i,k)*a.get_unsafe(k,j);
						for (size_t k=0;k<i;k++)
							a.get_unsafe(k,j) -= g*a.get_unsafe(k,i);
					}
				d[i]=a.get_unsafe(i,i);			// This statement remains.
				a.get_unsafe(i,i)=1;				// Reset row and column of a to identity
				for (size_t j=0;j<i;j++)
					a.get_unsafe(j,i)=a.get_unsafe(i,j)= 0; // matrix for next iteration.
			}
		}

		/** QL algorithm with implicit shifts, to determine the eigenvalues and eigenvectors of a real, symmetric,
			tridiagonal matrix, or of a real, symmetric matrix previously reduced by tred2 x11.2. On
			input, d[1..n] contains the diagonal elements of the tridiagonal matrix. On output, it returns
			the eigenvalues. The vector e[1..n] inputs the subdiagonal elements of the tridiagonal matrix,
			with e[1] arbitrary. On output e is destroyed. When finding only the eigenvalues, several lines
			may be omitted, as noted in the comments. If the eigenvectors of a tridiagonal matrix are desired,
			the matrix z[1..n][1..n] is input as the identity matrix. If the eigenvectors of a matrix
			that has been reduced by tred2 are required, then z is input as the matrix output by tred2.
			In either case, the kth column of z returns the normalized eigenvector corresponding to d[k].
		*/
		template <class MAT,class ARR>
		void  tqli(ARR &d, ARR &e, const size_t n, MAT &z)
		{
			typedef typename MAT::value_type T;

			T			s,r,p,g,f,dd,c,b;
			const T		EPS= std::numeric_limits<T>::epsilon();

			for (size_t i=1;i<n;i++)
				e[i-1]=e[i];				// Convenient to renumber the elements of e.

			e[n-1]=0.0;
			for (size_t l=0;l<n;l++)
			{
				size_t iter=0;
				size_t m;
				do
				{
					for (m=l;m<(n-1);m++)	// Look for a single small subdiagonal element to split the matrix.
					{
						dd=static_cast<T>(( fabs(d[m])+fabs(d[m+1]) ) );
						// ----------------------------------------------------------------------
						// Fix added 14/DEC/2006: Avoid a convergence problem due to compiler
						//  optimization. See http://www.nr.com/forum/showthread.php?p=1803
						// ----------------------------------------------------------------------
						// Fixed again 25/JAN/2008: Using EPS is a better approach!
						// ----------------------------------------------------------------------
						//temp = fabs(e[m])+dd;
						//if (fabs(e[m])+dd == dd) break;
						if (fabs(e[m]) <= EPS*dd)  break;
					}
					if (m != l)
					{
						if (iter++ == 60) THROW_EXCEPTION("tqli: Too many iterations in tqli!")

						g=static_cast<T>(( (d[l+1]-d[l])/(2.0*e[l])));			// Form shift.
						r=pythag(g,static_cast<T>(1.0));
						g=d[m]-d[l]+e[l]/(g+SIGN(r,g));		// This is dm - ks.
						s = c = 1;
						p = 0;
						int i;
						for (i=int(m)-1;i>=int(l);i--)				// A plane rotation as in the original QL, followed by Givens rotations to restore tridiagonal	form.
						{
							f=s*e[i];
							b=c*e[i];
							e[i+1]=(r=pythag(f,g));
							if (r == 0.0)						// Recover from underflow.
							{
								d[i+1] -= p;
								e[m]=0.0;
								break;
							}
							s=f/r;
							c=g/r;
							g=d[i+1]-p;
							r=(d[i]-g)*s+2.0f*c*b;
							d[i+1]=g+(p=s*r);
							g=c*r-b;
							/* Next loop can be omitted if eigenvectors not wanted*/
							for (size_t k=0;k<n;k++)  // Form eigenvectors.
							{
								f=z.get_unsafe(k,i+1);
								z.get_unsafe(k,i+1)=s*z.get_unsafe(k,i)+c*f;
								z.get_unsafe(k,i)=c*z.get_unsafe(k,i)-s*f;
							}
						}

						if (r == 0.0 && i >= int(l)) continue;
						d[l] -= p;
						e[l]=g;
						e[m]=0.0;
					}
				} while (m != l);
			}
		}



		/** @} */  // END OF MATRIX EIGENVECTORS

	} // end namespaces
	}
}

#endif
