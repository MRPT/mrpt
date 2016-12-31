/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/base/link_pragmas.h>
#include <vector>
#include <map>
#if EIGEN_VERSION_AT_LEAST(3,1,0) // Require Eigen 3.1+
#	include <Eigen/SparseCore>
#	include <Eigen/SparseCholesky>
#endif

namespace mrpt
{
namespace math
{
	/** Sparse solver for GMRF (Gaussian Markov Random Fields) graphical models.
	 *  Usage:
	 *   - The MRF must be first initialized to build the "prior" factors:
	 *      - init_priors(): Each cell is connected to neighbors, with a
	 *
	 * \ingroup mrpt_base_grp
	 * \note [New in MRPT 1.5.0]
	 */
	class BASE_IMPEXP GaussianMarkovRandomField : 
		public mrpt::utils::COutputLogger
	{
	public:
		GaussianMarkovRandomField();

		class BASE_IMPEXP InitPriorInformer
		{
		public:
			virtual size_t getNodeCount() const = 0; //!< Must return the overall number of nodes in the MRF
			virtual size_t getPriorFactorsCount() const = 0; //!< Must return the number of prior factors in the MRF

		};

		/** Initialize the GMRF prior factors. */
		void init_priors(const InitPriorInformer &params);

	private:

#if EIGEN_VERSION_AT_LEAST(3,1,0)
		std::vector<Eigen::Triplet<double> >  m_H_prior; // the prior part of H
#endif
		Eigen::VectorXd m_g;       //!< Gradient vector
		size_t m_nPriorFactors;    //!< L
		size_t m_nObsFactors;      //!< M
		size_t m_nFactors;         //!< L+M
		std::multimap<size_t, size_t> m_cell_interconnections;		//Store the interconnections (relations) of each cell with its neighbourds

		struct BASE_IMPEXP TObservationGMRF
		{
			double obsValue;       //!< Observation value
			double Lambda;         //!< "Information" of the observation (=inverse of the variance)
			bool   time_invariant; //!< whether the observation will lose weight (lambda) as time goes on (default false)

			TObservationGMRF() : obsValue(.0), Lambda(.0), time_invariant(false) {}
		};
		std::vector<std::vector<TObservationGMRF> > m_activeObs; //!< Vector with the active observations and their respective Information

	private:

	}; // End of class def.

} // End of namespace
} // End of namespace

