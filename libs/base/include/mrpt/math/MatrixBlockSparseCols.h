/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _mrpt_math_MatrixBlockSparseCols_H
#define _mrpt_math_MatrixBlockSparseCols_H

#include <mrpt/utils/map_as_vector.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>  // For mrpt::math::CMatrixDouble

namespace mrpt
{
	namespace math
	{

		/** A templated column-indexed efficient storage of block-sparse Jacobian or Hessian matrices, together with other arbitrary information.
		  *  Columns are stored in a non-associative container, but the contents of each column are kept within an std::map<> indexed by row.
		  *  All submatrix blocks have the same size, which allows dense storage of them in fixed-size matrices, avoiding costly memory allocations.
		  *
		  * \tparam NROWS Rows in each elementary matrix.
		  * \tparam NCOLS Cols in each elementary matrix.
		  * \tparam INFO  Type of the extra data fields within each block
		  * \tparam HAS_REMAP Is true, an inverse mapping between column indices and "user IDs" is kept.
		  * \tparam INDEX_REMAP_MAP_IMPL Ignore if HAS_REMAP=false. Defaults to "mrpt::utils::map_as_vector<size_t,size_t>" for amortized O(1). Can be set to "std::map<size_t,size_t>" in very sparse systems to save memory at the cost of a O(log N) access time when using the remap indices.
		  *
		  * \ingroup mrpt_base_grp
		  */
		template <
			typename Scalar,
			int NROWS,
			int NCOLS,
			typename INFO,
			bool HAS_REMAP,
			typename INDEX_REMAP_MAP_IMPL = mrpt::utils::map_as_vector<size_t,size_t>
		>
		struct MatrixBlockSparseCols
		{
			typedef Eigen::Matrix<Scalar,NROWS,NCOLS> matrix_t;
			typedef INFO symbolic_t;

			struct TEntry
			{
				matrix_t   num;  //!< Numeric matrix
				symbolic_t sym;  //!< Extra symbolic info
			};

			typedef typename mrpt::aligned_containers<size_t,TEntry>::map_t col_t; //!< Each compressed sparse column

		private:
			/** -> cols[i]: i'th column.
			  * -> Each column is a map [row] -> TEntry
			  */
			std::deque<col_t>            cols;
			/** "remapped index" is the index of some global variable, interpreted by the external user of this class. */
			//map<size_t,size_t> col_inverse_remapped_indices;
			mrpt::utils::map_as_vector<size_t,size_t> col_inverse_remapped_indices;
			std::vector<size_t>     col_remapped_indices;

		public:
			inline MatrixBlockSparseCols() : cols(0) { }

			inline col_t &       getCol(const size_t idx)       { return cols[idx]; }
			inline const col_t & getCol(const size_t idx) const { return cols[idx]; }

			inline const mrpt::utils::map_as_vector<size_t,size_t> & getColInverseRemappedIndices() const { if (!HAS_REMAP) assert(false); return col_inverse_remapped_indices; }
			inline const std::vector<size_t>     & getColRemappedIndices() const        { if (!HAS_REMAP) assert(false); return col_remapped_indices; }

			/** Append one column, returning a ref to the new col_t data */
			inline col_t & appendCol(const size_t remapIndex) {
				const size_t idx = cols.size();
				cols.push_back( col_t() );

				if (HAS_REMAP)
				{
					col_remapped_indices.resize(idx+1);
					col_remapped_indices[idx] = remapIndex;

					col_inverse_remapped_indices[remapIndex] = idx;
				}

				return *cols.rbegin();
			}

			/** Change the number of columns (keep old contents) */
			inline void setColCount(const size_t nCols) { cols.resize(nCols); }

			/** Get current number of cols. \sa findCurrentNumberOfRows */
			inline size_t getColCount() const { return cols.size(); }

			/** Clear all the entries in each column (do not change the number of columns, though!) \sa getColCount */
			inline void clearColEntries() {
				for (size_t i=0;i<cols.size();i++) cols[i].clear();
			}

			/** Clear all the entries in each column (do not change the number of columns, though!) \sa getColCount */
			inline void clearAll() {
				cols.clear();
				if (HAS_REMAP)
				{
					col_remapped_indices.clear();
					col_inverse_remapped_indices.clear();
				}
			}

			/** Builds a dense representation of the matrix and saves to a text file. */
			void saveToTextFileAsDense(
				const std::string &filename,
				const bool force_symmetry=false,
				const bool is_col_compressed = true ) const
			{
				mrpt::math::CMatrixDouble D;
				getAsDense(D,force_symmetry,is_col_compressed);
				return D.saveToTextFile(filename);
			}

			/** Builds a dense representation of the matrix and saves to a text file.
			  * \param is_col_compressed true: interpret this object as compressed by cols; false: compressed by rows
			  */
			void getAsDense(
				mrpt::math::CMatrixDouble &D,
				const bool force_symmetry=false,
				const bool is_col_compressed = true ) const
			{
				const size_t nCols = cols.size();
				const size_t nRows = findCurrentNumberOfRows();

				if (is_col_compressed)
					 D.setSize(nRows*NROWS,nCols*NCOLS);
				else D.setSize(nCols*NROWS,nRows*NCOLS);

				for (size_t j=0;j<nCols;j++)
				{
					for (typename col_t::const_iterator itRow=cols[j].begin();itRow!=cols[j].end();++itRow)
					{
						const size_t row = itRow->first;
						const size_t row_idx = is_col_compressed ? row*NROWS : j*NROWS;
						const size_t col_idx = is_col_compressed ? j*NCOLS : row*NCOLS;
						D.block(row_idx,col_idx, NROWS,NCOLS) = itRow->second.num;
						if (force_symmetry && row_idx!=col_idx)
							D.block(col_idx,row_idx, NCOLS,NROWS) = itRow->second.num.transpose();
					}
				}
			}

			/** Goes over all the columns and keep the largest column length. \sa getColCount() */
			size_t findCurrentNumberOfRows() const
			{
				size_t nRows = 0;
				const size_t nCols = cols.size();
				for (size_t j=0;j<nCols;j++)
					for (typename col_t::const_iterator itRow=cols[j].begin();itRow!=cols[j].end();++itRow)
						mrpt::utils::keep_max(nRows, itRow->first);
				return nRows+1; // nRows was the max. row index, now it's the row count.
			}

			/** Builds a binary matrix with 1s where an elementary matrix is stored, 0s elsewhere. */
			template <class MATRIX>
			void getBinaryBlocksRepresentation( MATRIX &out ) const
			{
				const size_t nCols = cols.size();
				const size_t nRows = findCurrentNumberOfRows();
				out.zeros(nRows,nCols);
				for (size_t j=0;j<nCols;j++)
					for (typename col_t::const_iterator itRow=cols[j].begin();itRow!=cols[j].end();++itRow)
					{
						const size_t row = itRow->first;
						out(row,j) = 1;
					}
			}

			/** Clear the current contents of this objects and replicates the sparse structure and numerical values of \a o */
			void copyNumericalValuesFrom( const MatrixBlockSparseCols<Scalar,NROWS,NCOLS,INFO,HAS_REMAP> & o )
			{
				const size_t nC = o.cols.size();
				if (cols.size()!=nC)
				{
					// Just create an empty structure with the numerical matrices:
					cols.resize(nC);
					for (size_t i=0;i<nC;i++)
					{
						cols[i].clear();
						for (typename col_t::const_iterator it=o.cols[i].begin();it!=o.cols[i].end();++it)
							cols[i][it->first].num = it->second.num;
					}
				}
				else
				{
					// It might be that we're overwriting an existing data structure:
					for (size_t i=0;i<nC;i++)
					{
						//ASSERTMSG_(cols[i].size()>=o.cols[i].size(), "copyNumericalValuesFrom() invoked on dissimilar structures")
						typename col_t::iterator       it_dst = cols[i].begin();
						typename col_t::const_iterator it_src = o.cols[i].begin();
						while (it_src!=o.cols[i].end())
						{
							if (it_dst->first < it_src->first)
							{
								it_dst->second.num.setZero();
								it_dst++;
							}
							else if (it_dst->first > it_src->first)
							{
								cols[i][it_src->first].num = it_src->second.num;
								it_src++;
							}
							else
							{
								it_dst->second.num = it_src->second.num;
								++it_dst;
								++it_src;
							}
						}
					}
				}
			} // end copyNumericalValuesFrom()

		}; // end of MatrixBlockSparseCols

	} // end NS
}// end NS

#endif //_mrpt_math_MatrixBlockSparseCols_H
