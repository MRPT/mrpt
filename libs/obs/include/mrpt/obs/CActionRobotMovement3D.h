/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CActionRobotMovement3D_H
#define CActionRobotMovement3D_H

#include <mrpt/obs/CAction.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CActionRobotMovement3D, CAction, OBS_IMPEXP )

	/** Represents a probabilistic 3D (6D) movement.
	*   Currently this can be determined from visual odometry for full 6D, or from wheel encoders for 2D movements only.
	* Here implemented the  motion model from the next article: A. L. Ballardini, A. Furlan, A. Galbiati, M. Matteucci, F. Sacchi, D. G. Sorrenti An effective 6DoF motion model for 3D-6DoF Monte Carlo Localization 4th Workshop on Planning, Perception and Navigation for Intelligent Vehicles, IROS, 2012
	* \ingroup mrpt_obs_grp
	* \sa CAction
	*/
	class OBS_IMPEXP CActionRobotMovement3D : public CAction
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CActionRobotMovement3D )

	public:
		/** A list of posible ways for estimating the content of a CActionRobotMovement3D object.
			*/
		enum TEstimationMethod
		{
			emOdometry = 0,
			emVisualOdometry
		};

		CActionRobotMovement3D();

		/** The 3D pose change probabilistic estimation. It can be converted to/from these alternative classes:
		  * - mrpt::poses::CPose3DQuatPDFGaussian 
		  */
		mrpt::poses::CPose3DPDFGaussian poseChange;

		/** This is the raw odometry reading, and only is used when "estimationMethod" is "TEstimationMethod::emOdometry" */
			mrpt::poses::CPose3D					rawOdometryIncrementReading;
		/** This fields indicates the way this estimation was obtained.
		  */
		TEstimationMethod		estimationMethod;

			enum TDrawSampleMotionModel
			{
				mmGaussian = 0,
				mm6DOF
			};


			/** The parameter to be passed to "computeFromOdometry". */
			struct OBS_IMPEXP TMotionModelOptions
			{
				TMotionModelOptions(); //!< Default values loader.

				TDrawSampleMotionModel	modelSelection; //!< The model to be used.


				struct OBS_IMPEXP  TOptions_6DOFModel
				{

				/** Options for the 6DOFModel model  which generates a CPosePDFParticles object an then create from that  CPosePDFGaussian object  in poseChange */
					uint32_t		nParticlesCount;
					float			a1,a2,a3,a4,a5,a6,a7,a8,a9,a10;
					/** An additional noise added to the 6DOF model (std. dev. in meters and radians). */
					float			additional_std_XYZ, additional_std_angle;
				}mm6DOFModel;

			} motionModelConfiguration;

			/** Computes the PDF of the pose increment from an odometry reading and according to the given motion model (speed and encoder ticks information is not modified).
			  * According to the parameters in the passed struct, it will be called one the private sampling functions (see "see also" next).
			  * \sa computeFromOdometry_model6DOF
			  */
			void  computeFromOdometry(
				const mrpt::poses::CPose3D &odometryIncrement,
				const TMotionModelOptions	&options);
			/** Computes the PDF of the pose increment from an odometry reading, using the motion model for 6 DOF.
			 *  The source: A. L. Ballardini, A. Furlan, A. Galbiati, M. Matteucci, F. Sacchi, D. G. Sorrenti An effective 6DoF motion model for 3D-6DoF Monte Carlo Localization 4th Workshop on Planning, Perception and Navigation for Intelligent Vehicles, IROS, 2012
			 * \sa computeFromOdometry
			 */
		void  computeFromOdometry_model6DOF(
				const mrpt::poses::CPose3D		&odometryIncrement,
				const TMotionModelOptions	&o
				) ;


		/** Each "true" entry means that the corresponding "velocities" element contains valid data - There are 6 entries.
		  */
		vector_bool				hasVelocities;

		/** The velocity of the robot in each of 6D: v_x,v_y,v_z,v_yaw,v_pitch,v_roll (linear in meters/sec and angular in rad/sec).
		  */
		mrpt::math::CVectorFloat	velocities;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CActionRobotMovement3D, CAction, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
