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
#ifndef  KDTreeMatrixAdaptor_H
#define  KDTreeMatrixAdaptor_H

#include <mrpt/utils/utils_defs.h>

// FLANN library:
#include <mrpt/otherlibs/flann/flann.hpp>
#include <mrpt/otherlibs/flann/algorithms/kdtree_single_index_mrpt.h> // An index class with an MRPT adaptor instead of a Matrix<> for the dataset

namespace mrpt
{
	namespace math
	{
		/** A simple FLANN KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
		 * \sa KDTreeCapable
		 */
		template <class MatrixType>
		struct KDTreeMatrixAdaptor
		{
			typedef KDTreeMatrixAdaptor<MatrixType> self_t;
			typedef typename MatrixType::Scalar           num_t;

			typedef mrpt_flann::KDTreeSingleIndexAdaptor<
				mrpt_flann::L2_Simple_Adaptor<num_t,self_t>,  // Metric
				KDTreeMatrixAdaptor<MatrixType>  // Adaptor (myself)
				>  base_t;

			base_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

			/// Constructor: takes a const ref to the matrix object with the data points
			KDTreeMatrixAdaptor(const int dimensionality, const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat)
			{
				const size_t dims = mat.cols();
				index = new base_t( dims, *this /* adaptor */, mrpt_flann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
				index->buildIndex();
			}

			void query(const num_t *query_point, int num_closest, int *out_indices, num_t *out_distances_sq, const int nChecks = 10)
			{
				mrpt_flann::KNNResultSet<typename MatrixType::Scalar> resultSet(num_closest);
				resultSet.init(out_indices, out_distances_sq);
				index->findNeighbors(resultSet, query_point, mrpt_flann::SearchParams(nChecks));
			}

			~KDTreeMatrixAdaptor() { delete index; }

			const MatrixType &m_data_matrix;


			const self_t & derived() const { return *this; }
				  self_t & derived()       { return *this; }

			// Must return the number of data points
			inline size_t kdtree_get_point_count() const { return m_data_matrix.rows(); }

			// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
			inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const
			{
				num_t s=0;
				for (size_t i=0;i<size;i++) {
					const num_t d= p1[i]-m_data_matrix.coeff(idx_p2,i);
					s+=d*d;
				}
				return s;
			}

			// Returns the dim'th component of the idx'th point in the class:
			inline num_t kdtree_get_pt(const size_t idx, int dim) const { return m_data_matrix.coeff(idx,dim); }

			// Optional bounding-box computation: return false to default to a standard bbox computation loop.
			//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
			//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
			template <class BBOX>
			bool kdtree_get_bbox(BBOX &bb) const { return false; }

		}; // end of KDTreeMatrixAdaptor

	} // End of namespace
} // End of namespace
#endif
