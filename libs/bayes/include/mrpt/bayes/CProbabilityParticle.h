/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/deepcopy_ptr.h>  // copy_ptr<>

namespace mrpt::bayes
{
/** use for CProbabilityParticle
 * \ingroup mrpt_bayes_grp
 */
enum class particle_storage_mode
{
	VALUE,
	POINTER
};

/** A template class for holding a the data and the weight of a particle.
 * Particles comprise two parts: a "data payload field", and a logarithmic
 * importance sampling weight.
 *
 * \tparam T The type of the payload. Must be default constructable.
 * \tparam STORAGE If `POINTER`, a (wrapper to a) pointer is used to store
 * the payload; if `VALUE`, the payload is stored directly as a value.
 *
 * \sa CParticleFilterData
 * \ingroup mrpt_bayes_grp
 */
template <class T, particle_storage_mode STORAGE>
struct CProbabilityParticle;

struct CProbabilityParticleBase
{
	CProbabilityParticleBase(double logw = 0) : log_w(logw) {}
	/** The (logarithmic) weight value for this particle. */
	double log_w{.0};
};

template <class T>
struct CProbabilityParticle<T, particle_storage_mode::POINTER>
	: public CProbabilityParticleBase
{
	/** The data associated with this particle. The use of copy_ptr<> allows
	 * relying on compiler-generated copy ctor, etc. */
	mrpt::containers::copy_ptr<T> d{};
};

template <class T>
struct CProbabilityParticle<T, particle_storage_mode::VALUE>
	: public CProbabilityParticleBase
{
	CProbabilityParticle() = default;
	CProbabilityParticle(const T& data, const double logw)
		: CProbabilityParticleBase(logw), d(data)
	{
	}
	/** The data associated with this particle */
	T d{};
};

}  // namespace mrpt::bayes
