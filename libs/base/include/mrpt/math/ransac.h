/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_ransac_H
#define  mrpt_ransac_H

#include <mrpt/utils/COutputLogger.h>
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
		class BASE_IMPEXP RANSAC_Template : public mrpt::utils::COutputLogger
		{
		public:
			RANSAC_Template() :
				mrpt::utils::COutputLogger("RANSAC_Template")
			{
			}

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
			  * \note [MRPT 1.5.0] `verbose` parameter has been removed, supersedded by COutputLogger settings.
			  */
			bool execute(
				const CMatrixTemplateNumeric<NUMTYPE>	  &data,
				TRansacFitFunctor			fit_func,
				TRansacDistanceFunctor  	dist_func,
				TRansacDegenerateFunctor 	degen_func,
				const double   				distanceThreshold,
				const unsigned int			minimumSizeSamplesToFit,
				mrpt::vector_size_t			&out_best_inliers,
				CMatrixTemplateNumeric<NUMTYPE> &out_best_model,
				const double                prob_good_sample = 0.999,
				const size_t				maxIter = 2000
				) const;

		}; // end class

		typedef RANSAC_Template<double> RANSAC;   //!< The default instance of RANSAC, for double type

		/** @} */

	} // End of namespace
} // End of namespace

#endif
