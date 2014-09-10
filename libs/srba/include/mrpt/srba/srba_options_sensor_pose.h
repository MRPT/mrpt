/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt { namespace srba {
namespace options
{
	/** \defgroup mrpt_srba_options_pose_on_robot Types for RBA_OPTIONS::sensor_pose_on_robot_t 
		* \ingroup mrpt_srba_options */

		namespace internal {
			/** Typedefs for determining whether the result of combining a KF pose (+) a sensor
			 * pose leads to a SE(2) or SE(3) pose */
			template <class SENSOR_POSE_CLASS, size_t KF_POSE_DIMS> struct resulting_pose_t;
		}

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t.
		  * Meaning: The robot pose and the sensor pose coincide, i.e. the sensor pose on the
		  * robot is the identitity transformation.
		  * \ingroup mrpt_srba_options_pose_on_robot */
		struct sensor_pose_on_robot_none
		{
			/** In this case there are no needed parameters */
			struct parameters_t
			{
			};

			/** Gets the pose of the sensor given the robot pose (robot->sensor) */
			template <class POSE>
			static inline void robot2sensor(const POSE & robot, POSE & sensor, const parameters_t &p) {
				MRPT_UNUSED_PARAM(p);
				sensor = robot;
			}

			/** Converts a pose relative to the robot coordinate frame (P) into a pose relative to
			 * the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class KF_POSE>
			static inline void pose_robot2sensor(const KF_POSE & pose_wrt_robot,
				KF_POSE & pose_wrt_sensor, const parameters_t &p) {
				MRPT_UNUSED_PARAM(p);
				pose_wrt_sensor = pose_wrt_robot;
			}

			/** Converts a point relative to the robot coordinate frame (P) into a point relative
			 * to the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class LM_TYPE, class POINT>
			static inline void point_robot2sensor(const POINT & pt_wrt_robot,
				POINT & pt_wrt_sensor, const parameters_t &p) {
				MRPT_UNUSED_PARAM(p);
				pt_wrt_sensor=pt_wrt_robot;
			}

			/** Take into account the possible displacement of the sensor wrt the keyframe when
			 * evaluating the Jacobian dh_dx */
			template <class MATRIX>
			static inline void jacob_dh_dx_rotate( MATRIX & dh_dx, const parameters_t &p) {
				MRPT_UNUSED_PARAM(dh_dx);
				MRPT_UNUSED_PARAM(p);
				/* nothing to do, since there's no displacement */
			}
			template <class LANDMARK_T>
			static inline void sensor2robot_point(typename landmark_traits<LANDMARK_T>::array_landmark_t & pt,
				const parameters_t &p) {
				MRPT_UNUSED_PARAM(pt); MRPT_UNUSED_PARAM(p);
				/* nothing to do, since there's no displacement */
			}
		};

		namespace internal {
			/** Typedefs for determining whether the result of combining a KF pose (+) a sensor
			 * pose leads to a SE(2) or SE(3) pose */
			template <> struct resulting_pose_t<sensor_pose_on_robot_none,3> {
				typedef mrpt::poses::CPose2D pose_t; };
			template <> struct resulting_pose_t<sensor_pose_on_robot_none,6> {
				typedef mrpt::poses::CPose3D pose_t; };
		}

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t.
		  * Meaning: The sensor is located at an arbitrary SE(3) pose wrt the robot reference frame.
		  * \ingroup mrpt_srba_options_pose_on_robot */
		struct sensor_pose_on_robot_se3
		{
			/** In this case there are no needed parameters */
			struct parameters_t
			{
				mrpt::poses::CPose3D  relative_pose;
			};

			/** Gets the pose of the sensor given the robot pose (robot->sensor) */
			template <class KF_POSE>
			static inline void robot2sensor(const KF_POSE & robot, mrpt::poses::CPose3D & sensor,
				const parameters_t &p) {
				sensor.composeFrom(robot, p.relative_pose);
			}

			/** Converts a pose relative to the robot coordinate frame (P) into a pose relative to
			 * the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class KF_POSE>
			static inline void pose_robot2sensor(const KF_POSE & pose_wrt_robot,
				mrpt::poses::CPose3D & pose_wrt_sensor, const parameters_t &p) {
				pose_wrt_sensor.inverseComposeFrom(pose_wrt_robot, p.relative_pose);
			}

			/** Converts a point relative to the robot coordinate frame (P) into a point relative
			 * to the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class LM_TYPE,class POINT>
			static inline void point_robot2sensor(const POINT & pt_wrt_robot,
				POINT & pt_wrt_sensor, const parameters_t &p) {
				LM_TYPE::inverseComposePosePoint(pt_wrt_robot,pt_wrt_sensor,p.relative_pose);
			}

			/** Take into account the possible displacement of the sensor wrt the keyframe when
			 * evaluating the Jacobian dh_dx */
			template <class MATRIX>
			static inline void jacob_dh_dx_rotate( MATRIX & dh_dx, const parameters_t &p) {
				// dh(R2S+x)_dx = dh(x')_dx' * d(x')_dx
				dh_dx = dh_dx * p.relative_pose.getRotationMatrix().transpose();
			}

			template <class LANDMARK_T>
			static inline void sensor2robot_point(typename landmark_traits<LANDMARK_T>::array_landmark_t & pt, const parameters_t &p) {
				landmark_traits<LANDMARK_T>::composePosePoint(pt, p.relative_pose);
			}
		};
		namespace internal {
			/** Typedefs for determining whether the result of combining a KF pose (+) a sensor pose leads to a SE(2) or SE(3) pose */
			template <> struct resulting_pose_t<sensor_pose_on_robot_se3,3> { typedef mrpt::poses::CPose3D pose_t; };
			template <> struct resulting_pose_t<sensor_pose_on_robot_se3,6> { typedef mrpt::poses::CPose3D pose_t; };
		}

} } } // End of namespaces
