/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/TStereoCamera.h>

namespace mrpt { namespace srba {
namespace observations {

	/** \defgroup mrpt_srba_observations Observation types
		* \ingroup mrpt_srba_grp */

	/** \addtogroup mrpt_srba_observations
		* @{ */

	/** Observation = one monocular camera feature (the coordinates of one pixel) */
	struct MonocularCamera
	{
		static const size_t  OBS_DIMS = 2; //!< Each observation is one pixel (px,py) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::utils::TPixelCoordf  px;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = px.x;
				obs[1] = px.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			mrpt::utils::TCamera  camera_calib;
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = one stereo camera feature (the coordinates of two pixels) */
	struct StereoCamera
	{
		static const size_t  OBS_DIMS = 4; //!< Each observation is a pair of pixels (px_l,py_l,px_r,py_r) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::utils::TPixelCoordf  left_px, right_px;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = left_px.x;  obs[1] = left_px.y;
				obs[2] = right_px.x; obs[3] = right_px.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			mrpt::utils::TStereoCamera camera_calib;
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = XYZ coordinates of landmarks relative to the sensor */
	struct Cartesian_3D
	{
		static const size_t  OBS_DIMS = 3; //!< Each observation is a triplet of coordinates (x,y,z) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::math::TPoint3D  pt;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = pt.x; obs[1] = pt.y; obs[2] = pt.z; 
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = XY coordinates of landmarks relative to the sensor */
	struct Cartesian_2D
	{
		static const size_t  OBS_DIMS = 2; //!< Each observation is a pair of coordinates (x,y) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::math::TPoint2D  pt;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = pt.x; obs[1] = pt.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = Range+Bearing (yaw & pitch) of landmarks relative to the sensor */
	struct RangeBearing_3D
	{
		static const size_t  OBS_DIMS = 3; //!< Each observation is a triplet of coordinates (range,yaw,pitch) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			double range; //!< Distance (in meters)
			double yaw;   //!< Angle around +Z (in radians)
			double pitch; //!< Angle around +Y (in radians)

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = range; obs[1] = yaw; obs[2] = pitch; 
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = Range+Bearing (yaw) of landmarks relative to the sensor, for planar environments only. */
	struct RangeBearing_2D
	{
		static const size_t  OBS_DIMS = 2; //!< Each observation is a pair of coordinates (range,yaw) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			double range; //!< Distance (in meters)
			double yaw;   //!< Angle around +Z (in radians)

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = range; obs[1] = yaw;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = Relative SE(2) poses (x,y,yaw) */
	struct RelativePoses_2D
	{
		static const size_t  OBS_DIMS = 3; //!< Each observation is a triplet (x,y,yaw) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			double x,y;   //!< Displacement (in meters)
			double yaw;   //!< Angle around +Z (in radians)

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0]=x; obs[1]=y; obs[2]=yaw;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	// -------------------------------------------------------------------------------------------------


	/** @} */

}
} } // end NS
