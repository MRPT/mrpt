/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef TSLIDINGWINDOW_H
#define TSLIDINGWINDOW_H

#include <mrpt/utils/CLoadableOptions.h>

#include <algorithm> // std::transform
#include <cmath> // sqrt
#include <functional> // std::bind2nd
#include <numeric> // std::accumulate
#include <string>
#include <vector>

namespace mrpt { namespace graphslam {

/**\brief Class to monitor the evolution of a statistical quantity. Keeps
 * track of the last N incoming measurements of the quantity at hand and upon
 * request returns statistical properties of these measurements (mean,
 * median, etc.)
 *
 * Class also contains methods for evaluating incoming measurements (weather
 * to accept or reject those)
 *
 * \ingroup mrpt_graphslam_grp
 */
struct TSlidingWindow: public mrpt::utils::CLoadableOptions {
	public:
		TSlidingWindow(std::string name="window");
		~TSlidingWindow();
		/**\brief Return the current median value.  */
		double getMedian();
		/**\brief Return the current mean value.  */
		double getMean();
		/**\brief Return the Standard deviation of the current measurement vector*/
		double getStdDev();
		/**\brief Determine whether the incoming measurement is inside the
		 * [-3sigma, +3sigma] boundaries from the current mean value. 
		 *
		 * \return True if it's inside the uncertainty boundaries 
		 */
		bool evaluateMeasurementInGaussian(double measurement);
		/**\brief Determine whether the incoming measurement is over the current
		 * mean value.
		 *
		 * \return True if it's above the mean
		 */
		bool evaluateMeasurementAbove(double value);
		/**\brief Determine whether the incoming measurement is *less or equal* to
		 * the current mean value.
		 *
		 * \return True if it's <= to the mean
		 */
		bool evaluateMeasurementBelow(double value);
		/**\brief Update the sliding window by appending a new measurement */
		void addNewMeasurement(double measurement);
		/** Resize the window.
		 *
		 * \note Method affects the underlying vector only if the new_size
		 * specified has already been reached
		 */
		void resizeWindow(size_t new_size);
		void loadFromConfigFile(
				const mrpt::utils::CConfigFileBase &source,
				const std::string &section);
		// TODO - make it use the boundaries
		void 	dumpToTextStream(mrpt::utils::CStream &out) const;

	private:
		size_t m_win_size;
		std::vector<double> m_measurements_vec;

		/**\brief Name of the TSlidingWindow Instance at hand */
		std::string m_name;

		double m_mean_cached; /**< Cached mean value */
		double m_median_cached; /**< Cached median value */
		double m_std_dev_cached; /**< Cached version of the stanard deviation */
		bool m_mean_updated; /**< Is the mean up-to-date? */
		bool m_median_updated; /**< Is the median up-to-date? */
		bool m_std_dev_updated; /**< Is the standard deviation up-to-date? */

		/**\brief flag is raised the first time that
		 * TSlidingWindow::addNewMeasurement is
		 * called
		 */
		bool m_is_initialized;
};

} } // end of namespaces
#endif /* end of include guard: TSLIDINGWINDOW_H */
