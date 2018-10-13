/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <mrpt/math/eigen_frwds.h>

namespace mrpt::math
{
/** This class provides an easy way of computing histograms for unidimensional
real valued variables.
 *   Call "getHistogram" or "getHistogramNormalized" to retrieve the full list
of bin positions & hit counts.
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
 * \ingroup mrpt_math_grp
 */
class CHistogram
{
   private:
	/** The histogram limits */
	double m_min, m_max;
	/** ((max-min)/nBins)^-1 */
	double m_binSizeInv;
	/** The bins counter */
	std::vector<size_t> m_bins;
	/** The total elements count */
	size_t m_count;

   public:
	/** Constructor
	 * \exception std::exception On nBins<=0 or max<=min
	 */
	CHistogram(const double min, const double max, const size_t nBins);

	/** Constructor with a fixed bin width.
	 * \exception std::exception On max<=min or width<=0
	 */
	inline CHistogram createWithFixedWidth(
		double min, double max, double binWidth);

	/** Clear the histogram:
	 */
	void clear();

	/**	Add an element to the histogram. If element is out of [min,max] it is
	 * ignored. */
	void add(const double x);

	/**	Add all the elements from a MRPT container to the histogram. If an
	 * element is out of [min,max] it is ignored. */
	template <typename Derived>
	inline void add(const Eigen::MatrixBase<Derived>& x)
	{
		const size_t N = x.size();
		for (size_t i = 0; i < N; i++)
			this->add(static_cast<const double>(x(i)));
	}

	//! \overload
	template <typename T>
	inline void add(const std::vector<T>& x)
	{
		const size_t N = x.size();
		for (size_t i = 0; i < N; i++)
			this->add(static_cast<const double>(x[i]));
	}

	/** Retuns the elements count into the selected bin index, where first one
	 * is 0.
	 * \exception std::exception On invalid index
	 */
	size_t getBinCount(const size_t index) const;

	/** Retuns the ratio in [0,1] range for the selected bin index, where first
	 * one is 0.
	 *  It returns 0 if no elements have been added.
	 * \exception std::exception On invalid index.
	 */
	double getBinRatio(const size_t index) const;

	/** Returns the list of bin centers & hit counts
	 * \sa getHistogramNormalized
	 */
	void getHistogram(std::vector<double>& x, std::vector<double>& hits) const;

	/** Returns the list of bin centers & hit counts, normalized such as the
	 * integral of the histogram, interpreted as a density PDF, amounts to 1.
	 * \sa getHistogram
	 */
	void getHistogramNormalized(
		std::vector<double>& x, std::vector<double>& hits) const;

};  // End of class def.

}  // namespace mrpt::math
