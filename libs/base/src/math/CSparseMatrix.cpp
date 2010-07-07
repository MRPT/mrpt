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
  sparse_matrix.m = sm->m;
  sparse_matrix.n = sm->n;
  sparse_matrix.nz = sm->nz;
  sparse_matrix.nzmax = sm->nzmax;

  ::memcpy(sparse_matrix.i,sm->i,sizeof(int)*sm->nzmax);
  ::memcpy(sparse_matrix.p,sm->p,sizeof(int)*(sm->n+1));
  ::memcpy(sparse_matrix.x,sm->x,sizeof(double)*sm->nzmax);
}

// Dtor
CSparseMatrix::~CSparseMatrix()
{
  cs_free(sparse_matrix.i);
  cs_free(sparse_matrix.p);
  cs_free(sparse_matrix.x);
}

/** Initialization from a triplet "cs", which is first compressed */
void CSparseMatrix::construct_from_triplet(const cs & triplet)
{
	cs * sm = cs_compress(&triplet);
	sparse_matrix.i = (int*)malloc(sizeof(int)*sm->nzmax);
	sparse_matrix.p = (int*)malloc(sizeof(int)*(sm->n+1));
	sparse_matrix.x = (double*)malloc(sizeof(double)*sm->nzmax);
	copy(sm);
	cs_spfree(sm);
}

/** Insert an element into a "cs", return false on error. */
bool CSparseMatrix::internal_add_entry(cs &MAT, int i, int j, double val )
{
	return 0!=cs_entry(&MAT,i,j,val);
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
	CSparseMatrix SM(sm);
	cs_spfree(sm);
	return SM;
}

CSparseMatrix CSparseMatrix::operator * (const CSparseMatrix & other) const
{
	cs * sm = cs_multiply(&(this->sparse_matrix), &(other.sparse_matrix));
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
	copy(sm);
	cs_spfree(sm);
}

void CSparseMatrix::operator *= (const CSparseMatrix & other)
{
	cs * sm = cs_multiply(&(this->sparse_matrix), &(other.sparse_matrix));
	copy(sm);
	cs_spfree(sm);
}

CSparseMatrix CSparseMatrix::transpose() const
{
	cs * sm = cs_transpose(&sparse_matrix,1);
	CSparseMatrix SM(sm);
	cs_spfree(sm);
	return SM;
}


void CSparseMatrix::get_dense(CMatrixDouble &d_M) const
{
	d_M.zeros(sparse_matrix.m,sparse_matrix.n);
	int col_idx = -1;

	for (int r=0;r<sparse_matrix.nzmax; ++r)
	{
		if (sparse_matrix.p[col_idx+1]==r)
			++col_idx;
		d_M(sparse_matrix.i[r],col_idx) = sparse_matrix.x[r];
	}
}
