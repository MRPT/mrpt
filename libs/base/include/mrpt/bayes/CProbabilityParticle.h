/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPROBABILITYPARTICLE_H
#define CPROBABILITYPARTICLE_H

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
		/** The data associated with this particle.
		 */
		T *d;

		/** The (logarithmic) weight value for this particle.
		 */
		double log_w;

		/** Default constructor:
		 */
		CProbabilityParticle() : d(NULL), log_w(0)
		{
		}

		/** Copy constructor:
		 */
		CProbabilityParticle(const CProbabilityParticle &o) : d(NULL), log_w(o.log_w)
		{
			if (o.d)
			{
				// Copy
				d = new T(*o.d);
			}
		}

		/** Copy operator
		  */
 		CProbabilityParticle<T> & operator =(const CProbabilityParticle &o)
		{
			if (this == &o) return *this;
			log_w = o.log_w;
			if (o.d)
			{
				// Copy semantic:
				if (d)
					*d = *o.d;			// Copy using the object "operator =".
				else d = new T(*o.d);	// Create a new object from the copy constructor
			}
			else
			{
				if (d)
				{
					delete d;
					d = NULL;
				}
			}
			return *this;
		}
	};

	} // end namespace
} // end namespace
#endif
