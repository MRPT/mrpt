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
#ifndef  mrpt_ransac_H
#define  mrpt_ransac_H

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/math/CMatrixD.h>
#include <set>

namespace mrpt
{
	namespace math
	{
		/** @addtogroup ransac_grp RANSAC and other model fitting algorithms
		  * \ingroup mrpt_base_grp
		  * @{ */


		/** A generic RANSAC implementation with models as matrices.
		  *  See \a RANSAC_Template::execute for more info on usage.
		  *  \sa mrpt::math::ModelSearch, a more versatile RANSAC implementation where models can be anything else, not only matrices.
		  */
		template <typename NUMTYPE = double>
		class BASE_IMPEXP RANSAC_Template : public mrpt::utils::CDebugOutputCapable
		{
		public:

			/** The type of the function passed to mrpt::math::ransac - See the documentation for that method for more info. */
			typedef void (*TRansacFitFunctor)(
				const CMatrixTemplateNumeric<NUMTYPE>  &allData,
				const mrpt::vector_size_t &useIndices,
				std::vector< CMatrixTemplateNumeric<NUMTYPE> > &fitModels );

			/** The type of the function passed to mrpt::math::ransac  - See the documentation for that method for more info. */
			typedef void (*TRansacDistanceFunctor)(
				const CMatrixTemplateNumeric<NUMTYPE>  &allData,
				const std::vector< CMatrixTemplateNumeric<NUMTYPE> > & testModels,
				const 	NUMTYPE   distanceThreshold,
				unsigned int & out_bestModelIndex,
				mrpt::vector_size_t & out_inlierIndices );

			/** The type of the function passed to mrpt::math::ransac  - See the documentation for that method for more info. */
			typedef bool (*TRansacDegenerateFunctor)(
				const CMatrixTemplateNumeric<NUMTYPE>  &allData,
				const mrpt::vector_size_t &useIndices );

			/** An implementation of the RANSAC algorithm for robust fitting of models to data.
			  *
			  *  \param data A DxN matrix with all the observed data. D is the dimensionality of data points and N the number of points.
			  *  \param
			  *
			  *  This implementation is highly inspired on Peter Kovesi's MATLAB scripts (http://www.csse.uwa.edu.au/~pk).
			  * \return false if no good solution can be found, true on success.
			  */
			static bool execute(
				const CMatrixTemplateNumeric<NUMTYPE>	  &data,
				TRansacFitFunctor			fit_func,
				TRansacDistanceFunctor  	dist_func,
				TRansacDegenerateFunctor 	degen_func,
				const double   				distanceThreshold,
				const unsigned int			minimumSizeSamplesToFit,
				mrpt::vector_size_t			&out_best_inliers,
				CMatrixTemplateNumeric<NUMTYPE> &out_best_model,
				bool						verbose = false,
				const double                prob_good_sample = 0.999,
				const size_t				maxIter = 2000
				);

		}; // end class

		typedef RANSAC_Template<double> RANSAC;   //!< The default instance of RANSAC, for double type

		/** @} */

	} // End of namespace
} // End of namespace

#endif
