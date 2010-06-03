/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef  mrpt_math_vector_ops_H
#define  mrpt_math_vector_ops_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CVectorTemplate.h>

// Many of the functions originally in this file are now in ops_containers.h
#include <mrpt/math/ops_containers.h>


namespace mrpt
{
	namespace utils { class CFileStream; }

	namespace math
	{

		/** \name Generic vector element-wise operations
		  * @{
		  */

		/** a*=b (element-wise multiplication) */
		template <class VECTOR1,class VECTOR2>
		inline RET_CONT1_ASSERT_MRPTVECTORS(VECTOR1,VECTOR2)
		operator *=(VECTOR1&a, const VECTOR2 &b)
		{
			ASSERT_(a.size()==b.size());
			typename VECTOR1::iterator ita = a.begin();
			typename VECTOR2::const_iterator itb = b.begin();
			const typename VECTOR1::iterator last=a.end();
			while (ita!=last) { *(ita++)*=*(itb++); }
			return a;
		}

		/** return a*b  (element-wise multiplication for vectors) */
		template <class VECTOR1,class VECTOR2>
		inline RET_CONT1_ASSERT_MRPTVECTORS(VECTOR1,VECTOR2)
		operator *(const VECTOR1 &a, const VECTOR2 &b)
		{
			VECTOR1 ret = a;
			ret*=b;
			return ret;
		}

		/** a/=b (element-wise division) */
		template <class VECTOR1,class VECTOR2>
		inline RET_CONT1_ASSERT_MRPTVECTORS(VECTOR1,VECTOR2)
		operator /=(VECTOR1&a, const VECTOR2 &b)
		{
			ASSERT_(a.size()==b.size());
			typename VECTOR1::iterator ita = a.begin();
			typename VECTOR2::const_iterator itb = b.begin();
			const typename VECTOR1::iterator last=a.end();
			while (ita!=last) { *(ita++)/=*(itb++); }
			return a;
		}

		/** return a/b  (element-wise division for vectors) */
		template <class VECTOR1,class VECTOR2>
		inline RET_CONT1_ASSERT_MRPTVECTORS(VECTOR1,VECTOR2)
		operator /(const VECTOR1 &a, const VECTOR2 &b)
		{
			VECTOR1 ret = a;
			ret/=b;
			return ret;
		}



		/** @} */

		/** A template function for printing out the contents of a std::vector variable.
			*/
		template <class T>
		std::ostream& operator << (std::ostream& out, const std::vector<T> &d)
		{
			out << "[" << std::fixed << std::setprecision(4);
			copy(d.begin(),d.end(), std::ostream_iterator<T>(out," "));
			out << "]";
			return out;
		}

		/** A template function for printing out the contents of a std::vector variable.
			*/
		template <class T>
		std::ostream& operator << (std::ostream& out, std::vector<T> *d)
		{
			out << "[";
			copy(d->begin(),d->end(), std::ostream_iterator<T>(out," "));
			out << "]";
			return out;
		}


	} // End of math namespace

} // End of mrpt namespace


#endif
