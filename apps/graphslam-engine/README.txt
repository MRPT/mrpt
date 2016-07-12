############################################################
# Application: graphslam-engine
#
# Part of "The Mobile Robot Programming Toolkit (MRPT)"
############################################################

# Sat Jun 25 17:11:25 EEST 2016, Nikos Koukis

# Description
########################

README File summarises usage instructions to the graphslam-engine MRPT
application.

Aim of the app is to perform 2D graphSLAM: Robot *localizes* itself
in the envornment while, at the same time *build a map* of that environment.
- App currently executes SLAM using MRPT rawlog files (both MRPT rawlog formats are
supported) as input which should contain (some of) the following observation types:
  - CObservationOdometry
  - CObservation2DRangeScan
  - CObservation3DRangeScan (experimental)

- The majority of the graphSLAM parameters in each case should be specified in
	an external .ini file which would be given as a command-line argument in the
	application launch. The following parameters can also be provided via the
	command line:
	+ ground-truth: Ground truth can be plotted in the visualization window if a
	  valid ground-truth file is provided and visualize_ground_truth is set to
	  true in the .ini file

	+ Node/Edge registration decider classes to be used. If not specified the
	  default CFixedIntervalsNRD, CICPGoodnessERD are used.

Sample calls to the graphslam-engine application are given below:

- show full list of node/edge registration deciders that can be used:
  graphslam-engine --list-regs
- run specifying both node and edge registration deciders
  graphslam-engine -i
  MRPT/share/mrpt/config_files/graphslam-engine/odometry_2DRangeScans.ini 
  -r MRPT/share/mrpt/config_files/graphslam-engine-demos/action_observations_map/range_030_bearing_015.rawlog
  -g MRPT/share/mrpt/config_files/graphslam-engine-demos/action_observations_map/range_030_bearing_015.rawlog.GT.txt
  --node-reg CFixedIntervalsNRD --edge-reg CICPGoodnessERD


The following commands should be working without any modification to the .ini
file - independently of the current working directory (modify paths to MRPT,
graphslam-engine accordingly):
- graphslam-engine -i
	MRPT/share/config_files/graphslam-engine/odometry_2DRangeScans.ini
	-r MRPT/datasets/graphslam-engine-demos/observation_only_map2/range030_bearing_015.rawlog
	-g MRPT/datasets/graphslam-engine-demos/observation_only_map2/range030_bearing_015.rawlog.GT.txt
	--node-reg CFixedIntervalsNRD --edge-reg CICPGoodnessERD
- graphslam-engine -i
  MRPT/share/config_files/graphslam-engine/odometry_2DRangeScans.ini
  -r MRPT/datasets/graphslam-engine-demos/action_observations_map/range030_bearing_015.rawlog
  -g MRPT/datasets/graphslam-engine-demos/action_observations_map/range030_bearing_015.rawlog.GT.txt
  --node-reg CFixedIntervalsNRD --edge-reg CICPGoodnessERD
- graphslam-engine -i
  MRPT/share/config_files/graphslam-engine/laser_odometry.ini -r
  MRPT/datasets/graphslam-engine-demos/action_observations_map/range030_bearing_015.rawlog
  -g MRPT/datasets/graphslam-engine-demos/action_observations_map/range030_bearing_015.rawlog.GT.txt
  --node-reg CICPGoodnessNRD --edge-reg CICPGoodnessERD


# TODO
# Update with the following
# Available Optimizers
