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

#include <string>
#include <vector>

/**\brief Class to monitor the evolution of a statistical quantity. Keeps
 * track of the last N incoming measurements of the quantity at hand and upon
 * request returns statistical properties of these measurements (mean,
 * median, etc.)
 *
 * Class also contains methods for evaluating incoming measurements (weather
 * to accept or reject those)
 */
struct TSlidingWindow: public mrpt::utils::CLoadableOptions {
	public:
		TSlidingWindow(std::string name="window");
		~TSlidingWindow();
		/**\brief Return the current median value.  */
		double getMedian();
		/**\brief Return the current mean value.  */
		double getMean();
		/**\brief Determine whether the incoming measurement is inside the
		 * [-3sigma, +3sigma] boundaries from the current mean value. 
		 *
		 * \return True if it's inside the uncertainty boundaries 
		 */
		// TODO - make it use the boundaries
		bool evaluateMeasurementInGaussian(double measurement);
		bool evaluateMeasurementAbove(double value);
		/**\brief Update the sliding window by appending a new measurement */
		void addNewMeasurement(double goodness_val);
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
		std::vector<double> m_goodness_vec;

		/**\brief Name of the TSlidingWindow Instance at hand */
		std::string m_name;

		double m_mean_cached; /**< Cached mean value */
		double m_median_cached; /**< Cached median value */
		bool m_mean_updated; /**< Is the mean up-to-date? */
		bool m_median_updated; /**< Is the median up-to-date? */

		/**\brief flag is raised the first time that
		 * TSlidingWindow::addNewMeasurement is
		 * called
		 */
		bool m_is_initialized;

};

#endif /* end of include guard: TSLIDINGWINDOW_H */
