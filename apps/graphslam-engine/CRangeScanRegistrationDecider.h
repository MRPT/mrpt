/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CRANGESCANREGISTRATIONDECIDER_H
#define CRANGESCANREGISTRATIONDECIDER_H

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/system/os.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Class for keeping together all the RangeScanner-related functions.
 *
 * Deciders that make use of either 2DRangeScans (laser generated
 * observations) or 3DRangeScans (RGBD-cameras) generated can inherit from
 * this class in case they want to use the underlying methods
 *
 * \ingroup mrpt_graphslam_grp
 */
template< class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CRangeScanRegistrationDecider {
	typedef typename GRAPH_t::constraint_t constraint_t;
	typedef CRangeScanRegistrationDecider<GRAPH_t> self_t;

	protected:
	// Protected methods
	// ////////////////////////////////////////////////////////////

	/**\brief Allign the 2D range scans provided and fill the potential edge that
	 * can transform the one into the other.
	 *
	 * User can optionally ask that additional information be returned in a
	 * TReturnInfo struct
	 */
	void getICPEdge(
			const mrpt::obs::CObservation2DRangeScan& from,
			const mrpt::obs::CObservation2DRangeScan& to,
			constraint_t* rel_edge,
			const mrpt::poses::CPose2D* initial_pose=NULL,
			mrpt::slam::CICP::TReturnInfo* icp_info=NULL);
	/**\brief Allign the 3D range scans provided and find the potential edge that
	 * can transform the one into the other.
	 *
	 * Fills the 2D part (rel_edge) of the 3D constraint between the scans, since
	 * we are interested in computing the 2D alignment. User can optionally ask
	 * that additional information be returned in a TReturnInfo struct
	 */
	void getICPEdge(
			const mrpt::obs::CObservation3DRangeScan& from,
			const mrpt::obs::CObservation3DRangeScan& to,
			constraint_t* rel_edge,
			const mrpt::poses::CPose2D* initial_pose=NULL,
			mrpt::slam::CICP::TReturnInfo* icp_info=NULL);
	/**\brief Reduce the size of the given CPointsMap by keeping one out of
	 * "keep_point_every" points.
	 *
	 * \note If low_lim is set then the PointsMap will contain at least low_lim
	 * measurements, regardless of keep_point_every value. Set low_lim to 0 if no
	 * lower limit is to be specified
	 */
	void decimatePointsMap(mrpt::maps::CPointsMap* m,
			size_t keep_point_every=4,
			size_t low_lim=0);
	/**\brief Wrapper around the CObservation3DRangeScan::convertTo2DScan method
	 */
	void convert3DTo2DRangeScan(
			/*from = */ mrpt::obs::CObservation3DRangeScanPtr& scan3D_in,
			/*to   = */ mrpt::obs::CObservation2DRangeScanPtr* scan2D_out=NULL);

	/**\brief Class to monitor the ICP edges and adapt the ICP goodness value used to
	 * accept or reject an edge. Class can compute both the current mean and
	 * median so that the decider may choose which one to use as a criterion
	 */
	struct TSlidingWindow: public mrpt::utils::CLoadableOptions {
		public:
			TSlidingWindow(double win_size=10);
			~TSlidingWindow();
			/**\brief Return the current median goodness.
			 */
			inline double getMedian();
			/**\brief Return the current mean goodness.
			 */
			inline double getMean();
			/**\brief Determine whether the goodness value provided is above the current
			 * mean or median.
			 *
			 * Depending on \b m_evaluate_using_mean it either uses the mean or
			 * median as the evaluation threshold
			 *
			 * \sa setEvaluationCriterion
			 * \return True if it above the threshold
			 */
			inline bool evaluateICPgoodness(double goodness);
			/**\brief Update the sliding window by appending a new ICP goodness
			 */
			inline void addNewMeasurement(double goodness_val);
			/** Resize the window.
			 *
			 * \note Method affects the underlying vector only if the new_size
			 * specified has already been reached
			 */
			void resizeWindow(size_t new_size);
			void loadFromConfigFile(
					const mrpt::utils::CConfigFileBase &source,
					const std::string &section);
			void 	dumpToTextStream(mrpt::utils::CStream &out) const;
			/**\brief Set the criterion to be used when evaluating an ICP goodness
			 * value.
			 *
			 * \sa evaluateICPgoodness
			 */
			void setEvaluationCriterion(std::string criterion);

		private:
			size_t m_win_size;
			std::vector<double> m_goodness_vec;

			double m_mean_cached; /**< Cached mean value */
			double m_median_cached; /**< Cached median value */
			bool m_mean_updated; /**< Is the mean up-to-date? */
			bool m_median_updated; /**< Is the median up-to-date? */

			/**\brief flag is raised the first time that
			 * TSlidingWindow::addNewMeasurement is
			 * called
			 */
			bool m_is_initialized;

			/** Option for deciding whether to use mean or median in the
			 * evaluateICPgoodness method
			 */
			bool m_evaluate_using_mean;


	};

	struct TParams: public mrpt::utils::CLoadableOptions {
		public:
			TParams();
			~TParams();

			void loadFromConfigFile(
					const mrpt::utils::CConfigFileBase &source,
					const std::string &section);
			void 	dumpToTextStream(mrpt::utils::CStream &out) const;

			mrpt::slam::CICP icp;

			// parameters for conversion 3D=>2D Range Scan
			std::string conversion_sensor_label;
			double conversion_angle_sup;
			double conversion_angle_inf;
			double conversion_oversampling_ratio;

			bool has_read_config;
	};
	TParams params;

};

} } } // end of namespaces

#include "CRangeScanRegistrationDecider_impl.h"
#endif /* end of include guard: CRANGESCANREGISTRATIONDECIDER_H */

