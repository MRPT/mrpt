/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPROBABILITYPARTICLE_H
#define CPROBABILITYPARTICLE_H

#include <mrpt/utils/copy_ptr.h>

namespace mrpt
{
namespace bayes
{
	/** A template class for holding a the data and the weight of a particle.
	*    Particles are composed of two parts:
	 *		- A state vector descritor, which in this case can be any user defined CSerializable class
	 *		- A (logarithmic) weight value.
	 *
	 *  This structure is used within CParticleFilterData, see that class for more information.
	 * \ingroup mrpt_base_grp
	 */
	template <class T>
	struct CProbabilityParticle
	{
	public:
		mrpt::utils::copy_ptr<T> d;   //!< The data associated with this particle. The use of copy_ptr<> allows relying on compiler-generated copy ctor, etc.
		double log_w; //!< The (logarithmic) weight value for this particle.

		/** Default constructor */
		CProbabilityParticle() : d(), log_w(.0) {}
	};

	} // end namespace
} // end namespace
#endif
