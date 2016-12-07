/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */
#ifndef SENSORSOURCEOFMEASUREMENTS_H
#define SENSORSOURCEOFMEASUREMENTS_H

namespace mrpt { namespace graphslam { namespace detail {

/** \brief Sensors that may be used for registering a measurement (i.e. a
 * graph constraint/edge)
 *
 * \note Can be used for labelling the constraints of a graph.
 *
 * \sa mrpt::graphslam::TGraphSlamHypothesis
 * \ingroup mrpt_graphslam_grp
 */
enum SensorSourceOfMeasurements {
	SSOM_Laser=0,
	SSOM_Odometry,
	SSOM_Kinect,
	SSOM_Camera,
	SSOM_NumOfSensorSources
};

} } } // end of namespaces

#endif /* end of include guard: SENSORSOURCEOFMEASUREMENTS_H */
