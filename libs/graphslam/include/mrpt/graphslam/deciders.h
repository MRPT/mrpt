/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_DECIDERS_H
#define GRAPH_SLAM_DECIDERS_H

#include <mrpt/graphslam/types.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/graphslam/link_pragmas.h>

namespace mrpt { namespace graphslam { namespace deciders {

	/** Generic GraphSLAM update decider. See mrpt::graphslam::GraphSlamEngine<> */
	struct GRAPHSLAM_IMPEXP GS_GenericDecider
	{
		/** Returns true to save the current frame; return false to overwrite it with more recent data and continue updating its pose */
		template <class GRAPHSLAMENGINE>
		bool shouldCreateNewFrame(const GRAPHSLAMENGINE &gse, const mrpt::slam::CObservationPtr &obs)
		{
			const typename GRAPHSLAMENGINE::graph_t & graph = gse.getGraph();
			if (graph.nodes.empty()) 
				return true; // If the map is empty -> yes, create the first keyframe!

			// Current pose:
			typename GRAPHSLAMENGINE::pose_t cur_pose;
			gse.getCurrentPose(cur_pose);

			// Get KD-tree to search for neaby keyframes close to the current pose:
			const double maxRadiusSqr = mrpt::utils::square(1.01* params.new_kf_min_distance_xy);
			std::vector<std::pair<size_t,double> > nearbyKFs;

			const typename GRAPHSLAMENGINE::KeyFramesKDTree &kdtree = gse.getKeyFrameKDTree();
			if (GRAPHSLAMENGINE::rotation_dimensions==3)
			     kdtree.kdTreeRadiusSearch3D (cur_pose[0], cur_pose[1], cur_pose[2], maxRadiusSqr,nearbyKFs);
			else kdtree.kdTreeRadiusSearch2D (cur_pose[0], cur_pose[1], maxRadiusSqr,nearbyKFs);
			
			// No nearby KF means: 
			// 1) Empty list of neigbors, OR
			// 2) All neigbors have a distant orientation.
			if (nearbyKFs.size()==1)
				return true; // Create new KF if the current KF is alone in a given radius 

			MRPT_TODO("check angles")
			return false;
		} // end shouldCreateNewFrame()

		struct GRAPHSLAM_IMPEXP TParams : public mrpt::utils::CLoadableOptions
		{
			// ...
			double new_kf_min_distance_xy;  //!< Minimum distance (X,Y) between KeyFrames (in meters)
			double new_kf_min_angle;        //!< Minimum angle between KeyFrames (in radians)

			TParams(); 
			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase	&source,const std::string &section);
			virtual void dumpToTextStream(mrpt::utils::CStream &out) const;
		};
		
		GS_GenericDecider();  //!< Default ctor

		TParams params;
	}; 

}}} // end namespaces


#endif
