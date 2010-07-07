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

#include <mrpt/math/CSparseMatrix.h>

using std::string;
using std::cout;
using std::endl;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;


/** Copy constructor */
CSparseMatrix::CSparseMatrix(const CSparseMatrix & other)
{
  sparse_matrix.i = (int*)malloc(sizeof(int)*other.sparse_matrix.nzmax);
  sparse_matrix.p = (int*)malloc(sizeof(int)*(other.sparse_matrix.n+1));
  sparse_matrix.x = (double*)malloc(sizeof(double)*other.sparse_matrix.nzmax);
  copy(&other.sparse_matrix);
}

/** Copy constructor from an existing "cs" CSparse data structure */
CSparseMatrix::CSparseMatrix(const cs  * const sm)
{
  ASSERT_(CS_CSC(sm)==true);
  sparse_matrix.i = (int*)malloc(sizeof(int)*sm->nzmax);
  sparse_matrix.p = (int*)malloc(sizeof(int)*(sm->n+1));
  sparse_matrix.x = (double*)malloc(sizeof(double)*sm->nzmax);
  copy(sm);
}

/** Copy the data from an existing "cs" CSparse data structure */
void  CSparseMatrix::copy(const cs  * const sm)
{
  // TODO: Will this work for both triplet / compressed??
  sparse_matrix.m = sm->m;
  sparse_matrix.n = sm->n;
  sparse_matrix.nz = sm->nz;
  sparse_matrix.nzmax = sm->nzmax;

  ::memcpy(sparse_matrix.i,sm->i,sizeof(int)*sm->nzmax);
  ::memcpy(sparse_matrix.p,sm->p,sizeof(int)*(sm->n+1));
  ::memcpy(sparse_matrix.x,sm->x,sizeof(double)*sm->nzmax);
}

/** Fast copy the data from an existing "cs" CSparse data structure, copying the pointers and leaving NULLs in the source structure. */
void  CSparseMatrix::copy_fast(cs  * const sm)
{
  // Free previous contents, if any.
  internal_free_mem(); 

  // Fast copy / Move:
  sparse_matrix.m = sm->m;
  sparse_matrix.n = sm->n;
  sparse_matrix.nz = sm->nz;
  sparse_matrix.nzmax = sm->nzmax;

  sparse_matrix.i = sm->i;
  sparse_matrix.p = sm->p;
  sparse_matrix.x = sm->x;

  // Mark source as empty:
  sm->i=NULL;
  sm->p=NULL;
  sm->x=NULL;
}


// Dtor
CSparseMatrix::~CSparseMatrix()
{
	internal_free_mem();
}

/** Erase all previous contents and leave the matrix as a "triplet" 1x1 matrix without any data. */
void CSparseMatrix::clear()
{
	// Free old data:
	internal_free_mem();

	// Init as 1x1 triplet:
	sparse_matrix.nzmax = 1;
	sparse_matrix.m = 1; //nRows;
	sparse_matrix.n = 1; // nCols;
	sparse_matrix.i = (int*)malloc(sizeof(int)*sparse_matrix.nzmax);
	sparse_matrix.p = (int*)malloc(sizeof(int)*(sparse_matrix.n+1));
	sparse_matrix.x = (double*)malloc(sizeof(double)*sparse_matrix.nzmax);
	sparse_matrix.nz = 0;
}

/** free buffers (deallocate the memory of the i,p,x buffers) */
void CSparseMatrix::internal_free_mem()
{
  cs_free(sparse_matrix.i);
  cs_free(sparse_matrix.p);
  cs_free(sparse_matrix.x);
}

/** Initialization from a triplet "cs", which is first compressed */
void CSparseMatrix::construct_from_triplet(const cs & triplet)
{
	cs * sm = cs_compress(&triplet);
	copy_fast(sm);
	cs_spfree(sm); // This will release just the "cs" structure itself, not the internal buffers, now set to NULL.
}

/** Default constructor: empty */
CSparseMatrix::CSparseMatrix(const size_t nRows, const size_t nCols)
{
	sparse_matrix.nzmax = 1;
	sparse_matrix.m = nRows;
	sparse_matrix.n = nCols;
	sparse_matrix.i = (int*)malloc(sizeof(int)*sparse_matrix.nzmax);
	sparse_matrix.p = (int*)malloc(sizeof(int)*(sparse_matrix.n+1));
	sparse_matrix.x = (double*)malloc(sizeof(double)*sparse_matrix.nzmax);
	sparse_matrix.nz = 0;
}


/** Insert an element into a "cs", return false on error. */
void CSparseMatrix::insert_entry(const size_t row, const size_t col, const double val )
{
	if (!isTriplet())
		THROW_EXCEPTION("insert_entry() is only available for sparse matrix in 'triplet' format.")
	if (!cs_entry(&sparse_matrix,row,col,val))
		THROW_EXCEPTION("Error inserting element in sparse matrix (out of mem?)")
}


/** Copy operator from another existing object */
void CSparseMatrix::operator = (const CSparseMatrix & other)
{
	if (&other==this) return;

	cs_free(sparse_matrix.i);
	cs_free(sparse_matrix.p);
	cs_free(sparse_matrix.x);

	sparse_matrix.i = (int*)malloc(sizeof(int)*other.sparse_matrix.nzmax);
	sparse_matrix.p = (int*)malloc(sizeof(int)*(other.sparse_matrix.n+1));
	sparse_matrix.x = (double*)malloc(sizeof(double)*other.sparse_matrix.nzmax);
	copy(&other.sparse_matrix);
}


CSparseMatrix CSparseMatrix::operator + (const CSparseMatrix & other) const
{
	cs * sm = cs_add(&(this->sparse_matrix), &(other.sparse_matrix),1,1);
	ASSERT_(sm)
	CSparseMatrix SM(sm);
	cs_spfree(sm);
	return SM;
}

CSparseMatrix CSparseMatrix::operator * (const CSparseMatrix & other) const
{
	cs * sm = cs_multiply(&(this->sparse_matrix), &(other.sparse_matrix));
	ASSERT_(sm)
	CSparseMatrix SM(sm);
	cs_spfree(sm);
	return SM;
}

std::vector<double> CSparseMatrix::operator * (const std::vector<double> & other) const
{
	ASSERT_(other.size() == getColCount());
	std::vector<double> res(getRowCount());
	const double * y = &(other[0]);
	double * x = &(res[0]);
	cs_gaxpy(&sparse_matrix,y,x);
	return res;
}

void CSparseMatrix::operator += (const CSparseMatrix & other)
{
	cs * sm = cs_add(&(this->sparse_matrix), &(other.sparse_matrix),1,1);
	ASSERT_(sm)
	copy(sm);
	cs_spfree(sm);
}

void CSparseMatrix::operator *= (const CSparseMatrix & other)
{
	cs * sm = cs_multiply(&(this->sparse_matrix), &(other.sparse_matrix));
	ASSERT_(sm)
	copy(sm);
	cs_spfree(sm);
}

CSparseMatrix CSparseMatrix::transpose() const
{
	cs * sm = cs_transpose(&sparse_matrix,1);
	ASSERT_(sm)
	CSparseMatrix SM(sm);
	cs_spfree(sm);
	return SM;
}


void CSparseMatrix::get_dense(CMatrixDouble &d_M) const
{
	d_M.zeros(sparse_matrix.m,sparse_matrix.n);
	if (isTriplet())
	{	// It's in triplet form.
		for (int idx=0;idx<sparse_matrix.nzmax; ++idx)
			d_M(sparse_matrix.i[idx],sparse_matrix.p[idx]) = sparse_matrix.x[idx];
	}
	else
	{	// Column compressed format:
		ASSERT_(sparse_matrix.x)  // JL: Could it be NULL and be OK??? 

        //printf ("%d-by-%d, nzmax: %d nnz: %d, 1-norm: %g\n", m, n, nzmax, Ap [n], cs_norm (A)) ;
		for (int j = 0 ; j < sparse_matrix.n ; j++)
        {
            //printf ("    col %d : locations %d to %d\n", j, Ap [j], Ap [j+1]-1);
			const int p0 = sparse_matrix.p [j];
			const int p1 = sparse_matrix.p [j+1];
            for (int p = p0 ; p < p1 ; p++)
				d_M(sparse_matrix.i[p],j) = sparse_matrix.x [p];
        }
	}
}

void CSparseMatrix::compressFromTriplet()
{
	if (!isTriplet())
		THROW_EXCEPTION("compressFromTriplet(): Matrix is already in column-compressed format.")
	
	cs * sm = cs_compress(&this->sparse_matrix);
	copy_fast(sm);
	cs_spfree(sm); // This will release just the "cs" structure itself, not the internal buffers, now set to NULL.
}


/** save as a dense matrix to a text file \return False on any error.
*/
bool CSparseMatrix::saveToTextFile_dense(const std::string &filName)
{
	CMatrixDouble    dense;
	this->get_dense(dense);
	try
	{
		dense.saveToTextFile(filName);
		return true;
	}
	catch(...) { return false; } 
}
