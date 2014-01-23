/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt { namespace srba {
namespace options
{
	/** \defgroup mrpt_srba_options_noise Types for RBA_OPTIONS::obs_noise_matrix_t 
		* \ingroup mrpt_srba_options */

		/** Usage: A possible type for RBA_OPTIONS::obs_noise_matrix_t.
		  * Meaning: The sensor noise matrix is the same for all observations and equal to \sigma * I(identity).  
		  * \ingroup mrpt_srba_options_noise */
		struct observation_noise_identity
		{
			/** Observation noise parameters to be filled by the user in srba.parameters.obs_noise */
			struct parameters_t
			{
				/** One sigma of the Gaussian noise assumed for every component of observations (Default value: 1) */
				double std_noise_observations;

				parameters_t() : std_noise_observations (1.) 
				{ }
			};

			/** Internal struct for data that must be stored for each observation  */
			struct noise_data_per_obs_t
			{
				// None: all obs. have the same value="std_noise_observations" 
			};

			/** Must execute H+= J1^t * \Lambda * J2 */
			template <class MATRIX_H,class MATRIX_J1,class MATRIX_J2>
			inline static void accum_JtJ(MATRIX_H & H, const MATRIX_J1 & J1, const MATRIX_J2 &J2, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				H.noalias() += J1.transpose() * J2;  // The constant scale factor 1/sigma will be applied in the end (below)
			}
			/** Do scaling, if applicable, to H after end of all calls to accum_JtJ()  */
			template <class MATRIX_H>
			inline static void scale_H(MATRIX_H & H, const parameters_t & obs_noise_params) 
			{
				ASSERTDEB_(obs_noise_params.std_noise_observations>0)
				H *= 1.0/obs_noise_params.std_noise_observations;
			}

			/** Must execute grad+= J^t * \Lambda * r */
			template <class VECTOR_GRAD,class MATRIX_J,class VECTOR_R>
			inline static void accum_Jtr(VECTOR_GRAD & g, const MATRIX_J & J, const VECTOR_R &r, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				g.noalias() += J.transpose() * r;  // The constant scale factor 1/sigma will be applied in the end (below)
			}
			/** Do scaling, if applicable, to GRAD after end of all calls to accum_Jtr()  */
			template <class VECTOR_GRAD>
			inline static void scale_Jtr(VECTOR_GRAD & g, const parameters_t & obs_noise_params) 
			{
				ASSERTDEB_(obs_noise_params.std_noise_observations>0)
				g *= 1.0/obs_noise_params.std_noise_observations;
			}

		};  // end of "observation_noise_identity"

		/** Usage: A possible type for RBA_OPTIONS::obs_noise_matrix_t.
		  * Meaning: The sensor noise matrix is an arbitrary matrix and the same for all observations. 
		  * \ingroup mrpt_srba_options_noise */
		template <class OBS_TYPE>
		struct observation_noise_constant_matrix
		{
			static const size_t OBS_DIMS = OBS_TYPE::OBS_DIMS;  //!< The dimension of one observation

			typedef Eigen::Matrix<double,OBS_DIMS,OBS_DIMS>  obs_noise_matrix_t; //!< Type for symetric, positive-definite noise matrices.

			/** Observation noise parameters to be filled by the user in srba.parameters.obs_noise */
			struct parameters_t
			{
				/** The constant information matrix (inverse of covariance) for all the observations (\Lambda in common SLAM notation) */
				obs_noise_matrix_t  lambda;

				parameters_t() : lambda( obs_noise_matrix_t::Identity() ) 
				{ }
			};

			/** Internal struct for data that must be stored for each observation  */
			struct noise_data_per_obs_t
			{
				// None: all obs. have the same value
			};

			/** Must execute H+= J1^t * \Lambda * J2 */
			template <class MATRIX_H,class MATRIX_J1,class MATRIX_J2>
			inline static void accum_JtJ(MATRIX_H & H, const MATRIX_J1 & J1, const MATRIX_J2 &J2, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				H.noalias() += J1.transpose() * obs_noise_params.lambda * J2;
			}
			/** Do scaling, if applicable, to H after end of all calls to accum_JtJ()  */
			template <class MATRIX_H>
			inline static void scale_H(MATRIX_H & H, const parameters_t & obs_noise_params) 
			{  // Nothing else to do.
			}

			/** Must execute grad+= J^t * \Lambda * r */
			template <class VECTOR_GRAD,class MATRIX_J,class VECTOR_R>
			inline static void accum_Jtr(VECTOR_GRAD & g, const MATRIX_J & J, const VECTOR_R &r, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				g.noalias() += J.transpose() * obs_noise_params.lambda * r;
			}
			/** Do scaling, if applicable, to GRAD after end of all calls to accum_Jtr()  */
			template <class VECTOR_GRAD>
			inline static void scale_Jtr(VECTOR_GRAD & g, const parameters_t & obs_noise_params) 
			{  // Nothing else to do.
			}

		};  // end of "observation_noise_constant_matrix"

} } } // End of namespaces
