/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
