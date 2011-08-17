/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CActionRobotMovement2D_H
#define CActionRobotMovement2D_H

#include <mrpt/slam/CAction.h>
#include <mrpt/poses/CPose2D.h>
//#include <mrpt/poses/CPosePDFGaussian.h>
//#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDF.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::math;
		using namespace mrpt::poses;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CActionRobotMovement2D, CAction, OBS_IMPEXP )

		/** Represents a probabilistic 2D movement of the robot mobile base
		 *
		 *  See the tutorial on <a href="http://www.mrpt.org/Probabilistic_Motion_Models" >probabilistic motion models</a>.
		 *
		 * \sa CAction
	 	 * \ingroup mrpt_obs_grp
		 */
		class OBS_IMPEXP  CActionRobotMovement2D : public CAction
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CActionRobotMovement2D )

		public:
			/** A list of posible ways for estimating the content of a CActionRobotMovement2D object.
				*/
			enum TEstimationMethod
			{
				emOdometry = 0,
				emScan2DMatching
			};

			/** Constructor
			  */
			CActionRobotMovement2D();

			/** Copy constructor
			  */
			CActionRobotMovement2D(const CActionRobotMovement2D &o);

			/** Copy operator
			  */
			CActionRobotMovement2D & operator =(const CActionRobotMovement2D &o);

			/** Destructor
			  */
			~CActionRobotMovement2D();

			/** The 2D pose change probabilistic estimation.
			  */
			CPosePDFPtr				poseChange;

			/** This is the raw odometry reading, and only is used when "estimationMethod" is "TEstimationMethod::emOdometry"
			  */
			CPose2D					rawOdometryIncrementReading;

			/** This fields indicates the way this estimation was obtained.
			  */
			TEstimationMethod		estimationMethod;

			/** If "true" means that "encoderLeftTicks" and "encoderRightTicks" contain valid values.
			  */
			bool					hasEncodersInfo;

			/** For odometry only: the ticks count for each wheel FROM the last reading (positive means FORWARD, for both wheels);
			  * \sa hasEncodersInfo
			  */
			int32_t					encoderLeftTicks,encoderRightTicks;

			/** If "true" means that "velocityLin" and "velocityAng" contain valid values.
			  */
			bool					hasVelocities;

			/** The velocity of the robot, linear in meters/sec and angular in rad/sec.
			  */
			float					velocityLin, velocityAng;

			enum TDrawSampleMotionModel
			{
				mmGaussian = 0,
				mmThrun
			};
			/** The parameter to be passed to "computeFromOdometry".
			  */
			struct OBS_IMPEXP TMotionModelOptions
			{
				/** Default values loader.
				  */
				TMotionModelOptions();

				/** The model to be used.
				  */
				TDrawSampleMotionModel	modelSelection;

				/** Options for the gaussian model, which generates a CPosePDFGaussian object in poseChange
				  */
				struct OBS_IMPEXP TOptions_GaussianModel
				{
					float		a1,a2,a3,a4,minStdXY,minStdPHI;
				} gausianModel;

				/** Options for the Thrun's model, which generates a CPosePDFParticles object in poseChange
				  */
				struct OBS_IMPEXP  TOptions_ThrunModel
				{
					/** The default number of particles to generate in a internal representation (anyway you can draw as many samples as you want through CActionRobotMovement2D::drawSingleSample)
					  */
					uint32_t		nParticlesCount;

					float			alfa1_rot_rot;
					float			alfa2_rot_trans;
					float			alfa3_trans_trans;
					float			alfa4_trans_rot;

					/** An additional noise added to the thrun model (std. dev. in meters and radians).
					  */
					float			additional_std_XY, additional_std_phi;
				} thrunModel;

			} motionModelConfiguration;

			/** Computes the PDF of the pose increment from an odometry reading and according to the given motion model (speed and encoder ticks information is not modified).
			  * According to the parameters in the passed struct, it will be called one the private sampling functions (see "see also" next).
			  * \sa computeFromOdometry_modelGaussian, computeFromOdometry_modelThrun
			  */
			void  computeFromOdometry(
				const CPose2D				&odometryIncrement,
				const TMotionModelOptions	&options);

			/** If "hasEncodersInfo"=true, this method updates the pose estimation according to the ticks from both encoders and the passed parameters, which is passed internally to the method "computeFromOdometry" with the last used PDF options (or the defualt ones if not explicitly called by the user).
			  *
			  * \param K_left The meters / tick ratio for the left encoder.
			  * \param K_right The meters / tick ratio for the right encoder.
			  * \param D The distance between both wheels, in meters.
			  */
			void  computeFromEncoders(
				double	K_left,
				double	K_right,
				double	D );

			/** Using this method instead of "poseChange->drawSingleSample()" may be more efficient in most situations.
			  * \sa CPosePDF::drawSingleSample
			  */
			void  drawSingleSample( CPose2D &outSample ) const;

			/** Call this before calling a high number of times "fastDrawSingleSample", which is much faster than "drawSingleSample"
			  */
			void  prepareFastDrawSingleSamples() const;

			/** Faster version than "drawSingleSample", but requires a previous call to "prepareFastDrawSingleSamples"
			  */
			void  fastDrawSingleSample( CPose2D &outSample ) const;

		protected:
			/** Computes the PDF of the pose increment from an odometry reading, using a Gaussian approximation as the motion model.
			 * \sa computeFromOdometry
			 */
			void  computeFromOdometry_modelGaussian(
				const CPose2D				&odometryIncrement,
				const TMotionModelOptions	&o
				);

			/** Computes the PDF of the pose increment from an odometry reading, using the motion model from Thrun's book.
			 * This model is discussed in "Probabilistic Robotics", Thrun, Burgard, and Fox, 2006, pp.136.
			 * \sa computeFromOdometry
			 */
			void  computeFromOdometry_modelThrun(
				const CPose2D				&odometryIncrement,
				const TMotionModelOptions	&o
				);

			/** The sample generator for the model "computeFromOdometry_modelGaussian", internally called when the user invokes "drawSingleSample".
			  */
			void  drawSingleSample_modelGaussian( CPose2D &outSample ) const;

			/** The sample generator for the model "computeFromOdometry_modelThrun", internally called when the user invokes "drawSingleSample".
			  */
			void  drawSingleSample_modelThrun( CPose2D &outSample ) const;

			/** Internal use
			  */
			void  prepareFastDrawSingleSample_modelGaussian() const;

			/** Internal use
			  */
			void  prepareFastDrawSingleSample_modelThrun() const;

			/** Internal use
			  */
			void  fastDrawSingleSample_modelGaussian( CPose2D &outSample ) const;

			/** Internal use
			  */
			void  fastDrawSingleSample_modelThrun( CPose2D &outSample ) const;

			/** Auxiliary matrix
			  */
			mutable CMatrixDouble33	m_fastDrawGauss_Z;
			mutable CPose2D			m_fastDrawGauss_M;


		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
