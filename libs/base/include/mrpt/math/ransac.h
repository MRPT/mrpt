/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
