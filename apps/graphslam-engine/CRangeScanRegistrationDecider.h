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

#include <iostream>
#include <vector>

namespace mrpt { namespace graphslam { namespace deciders {

	/**
	 * Class for keeping together all the RangeScanner-related functions.
	 * Deciders that make use of either 2DRangeScans (laser generated
	 * observations) or 3DRangeScans (RGBD-cameras) generated can inherit from
	 * this class in case they want to use ICP alignment functions
	 */
	template< class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
		class CRangeScanRegistrationDecider_t {
			typedef typename GRAPH_t::constraint_t constraint_t;
			typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
			typedef mrpt::math::CMatrixFixedNumeric<double,
							constraint_t::state_length,
							constraint_t::state_length> InfMat;
			typedef CRangeScanRegistrationDecider_t<GRAPH_t> self_t;

  		public:
			protected:
			// Protected methods
			// ////////////////////////////////////////////////////////////

			/**
			 * allign the 2D range scans provided and fill the potential edge that
			 * can transform the one into the other. User can optionally ask that
			 * additional information be returned in a TReturnInfo struct
			 */
			void getICPEdge(
					const CObservation2DRangeScan& from,
					const CObservation2DRangeScan& to,
					constraint_t* rel_edge,
					mrpt::poses::CPose2D* initial_pose=NULL,
					mrpt::slam::CICP::TReturnInfo* icp_info=NULL);
			/**
			 * allign the 3D range scans provided and find the potential edge that
			 * can transform the one into the other. Fills the 2D part (rel_edge) of
			 * the 3D constraint between the scans, since we are interested in
			 * computing the 2D alignment. User can optionally ask that additional
			 * information be returned in a TReturnInfo struct
			 */
			void getICPEdge(
					CObservation3DRangeScan& from,
					CObservation3DRangeScan& to,
					constraint_t* rel_edge,
					mrpt::poses::CPose2D* initial_pose=NULL,
					mrpt::slam::CICP::TReturnInfo* icp_info=NULL);
			/**
			 * Reduce the size of the given CPointsMap by keeping one out of
			 * "keep_point_every" points. If low_lim is set then the PointsMap will
			 * contain at least low_lim measurements, regardless of keep_point_every
			 * value. Set low_lim to 0 if no lower limit is to be specified
			 */
			void decimatePointsMap(mrpt::maps::CPointsMap* m,
					size_t keep_point_every=4,
					size_t low_lim=0);
			/**
			 * Wrapper around the CObservation3DRangeScan::convertTo2DScan method
			 */
			void convert3DTo2DRangeScan(
					/*from = */ CObservation3DRangeScanPtr& scan3D_in,
					/*to   = */ CObservation2DRangeScanPtr* scan2D_out=NULL);

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

