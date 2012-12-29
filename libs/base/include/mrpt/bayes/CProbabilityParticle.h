/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef CPROBABILITYPARTICLE_H
#define CPROBABILITYPARTICLE_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>

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
