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
