/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_MATH_WRAP2PI_H
#define  MRPT_MATH_WRAP2PI_H

#define _USE_MATH_DEFINES // (For VS to define M_PI, etc. in cmath)
#include <cmath>
#include <cstddef>  // size_t

namespace mrpt
{
	namespace math
	{
		/** \addtogroup container_ops_grp
		  * @{ */

		/** Modifies the given angle to translate it into the [0,2pi[ range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline void wrapTo2PiInPlace(T &a)
		{
			bool was_neg = a<0;
			a = fmod(a, static_cast<T>(2.0*M_PI) );
			if (was_neg) a+=static_cast<T>(2.0*M_PI);
		}

		/** Modifies the given angle to translate it into the [0,2pi[ range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline T wrapTo2Pi(T a)
		{
			wrapTo2PiInPlace(a);
			return a;
		}

		/** Modifies the given angle to translate it into the ]-pi,pi] range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
		  */
		template <class T>
		inline T wrapToPi(T a)
		{
			return wrapTo2Pi( a + static_cast<T>(M_PI) )-static_cast<T>(M_PI);
		}

		/** Modifies the given angle to translate it into the ]-pi,pi] range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi,wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline void wrapToPiInPlace(T &a)
		{
			a = wrapToPi(a);
		}

		/** Modify a sequence of angle values such as no consecutive values have a jump larger than PI in absolute value.
		  * \sa wrapToPi
		  */
		template <class VECTOR>
		void unwrap2PiSequence(VECTOR &x)
		{
			const size_t N=x.size();
			for (size_t i=0;i<N;i++)
			{
				mrpt::math::wrapToPiInPlace(x[i]); // assure it's in the -pi,pi range.
				if (!i) continue;
				double Ap = x[i]-x[i-1];
				if (Ap>M_PI)  x[i]-=2.*M_PI;
				if (Ap<-M_PI) x[i]+=2.*M_PI;
			}
		}

		/** Computes the shortest angular increment (or distance) between two planar orientations, 
		  * such that it is constrained to [-pi,pi] and is correct for any combination of angles (e.g. near +-pi)
		  * Examples: angDistance(0,pi) -> +pi; angDistance(pi,0) -> -pi; 
		  *           angDistance(-3.1,3.1) -> -0.08; angDistance(3.1,-3.1) -> +0.08; 
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi
		  */
		template <class T>
		inline T angDistance(T from, T to)
		{
			wrapToPiInPlace(from); 
			wrapToPiInPlace(to);
			T d = to-from;
			if (d>M_PI)  d-=2.*M_PI; 
			else if (d<-M_PI) d+=2.*M_PI;
			return d;
		}

		/** @} */

	} // End of MATH namespace
} // End of namespace

#endif
