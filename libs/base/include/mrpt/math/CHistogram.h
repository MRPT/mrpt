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
#ifndef  CHISTOGRAM_H
#define  CHISTOGRAM_H

#include <cmath>
#include <mrpt/utils/utils_defs.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace math
{
	/** This class provides an easy way of computing histograms for unidimensional real valued variables.
	 *   Call "getHistogram" or "getHistogramNormalized" to retrieve the full list of bin positions & hit counts.
	 *
	 *  Example:
	\code
	CHistogram		hist(0,100,10);
	hist.add(86);
	hist.add(7);
	hist.add(45);

	std::cout << hist.getBinCount(0) << std::endl;		// Result: "1"
	std::cout << hist.getBinRatio(0) << std::endl;		// Result: "0.33"
	\endcode
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CHistogram
	{
	private:
		double	m_min,m_max;		//!< The histogram limits
		double  m_binSizeInv;		//!< ((max-min)/nBins)^-1
		std::vector<size_t>	m_bins;	//!< The bins counter
		size_t m_count;				//!< The total elements count

	public:
		/** Constructor
		 * \exception std::exception On nBins<=0 or max<=min
		 */
		CHistogram(const double min, const double max, const size_t nBins);

		/** Constructor with a fixed bin width.
		 * \exception std::exception On max<=min or width<=0
		 */
		static inline CHistogram createWithFixedWidth(double min,double max,double binWidth)	{
			ASSERT_(max>min);
			ASSERT_(binWidth>0);
			return CHistogram(min,max,static_cast<size_t>(ceil((max-min)/binWidth)));
		}

		/** Clear the histogram:
		 */
		void	clear();

		/**	Add an element to the histogram. If element is out of [min,max] it is ignored. */
		void	add(const double x);

		/**	Add all the elements from a MRPT container to the histogram. If an element is out of [min,max] it is ignored. */
		template <typename Derived>
		inline void add(const Eigen::MatrixBase<Derived> &x)
		{
			const size_t N = x.size();
			for (size_t i=0;i<N;i++)
				this->add(static_cast<const double>(x(i)));
		}

		//! \overload
		template <typename T>
		inline void add(const std::vector<T> &x)
		{
			const size_t N = x.size();
			for (size_t i=0;i<N;i++)
				this->add(static_cast<const double>(x[i]));
		}

		/** Retuns the elements count into the selected bin index, where first one is 0.
		 * \exception std::exception On invalid index
		 */
		int		getBinCount(const size_t index) const;

		/** Retuns the ratio in [0,1] range for the selected bin index, where first one is 0.
		 *  It returns 0 if no elements have been added.
		 * \exception std::exception On invalid index.
		 */
		double	getBinRatio(const size_t index) const;

		/** Returns the list of bin centers & hit counts
		  * \sa getHistogramNormalized
		  */
		void getHistogram( vector_double &x, vector_double &hits ) const;

		/** Returns the list of bin centers & hit counts, normalized such as the integral of the histogram, interpreted as a density PDF, amounts to 1.
		  * \sa getHistogram
		  */
		void getHistogramNormalized( vector_double &x, vector_double &hits ) const;


	}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
