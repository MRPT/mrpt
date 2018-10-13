/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CAction.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt::obs
{
/** Represents a probabilistic 2D movement of the robot mobile base
 *
 *  See docs:
 * https://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
 *
 * \note [New in MRPT 1.5.0] Velocity is now encoded as mrpt::math::TTwist2D as
 * a more general version of the old (linVel, angVel).
 * \sa CAction
 * \ingroup mrpt_obs_grp
 */
class CActionRobotMovement2D : public CAction
{
	DEFINE_SERIALIZABLE(CActionRobotMovement2D)

   public:
	/** A list of posible ways for estimating the content of a
	 * CActionRobotMovement2D object.
	 */
	enum TEstimationMethod
	{
		emOdometry = 0,
		emScan2DMatching
	};

	CActionRobotMovement2D();

	/** The 2D pose change probabilistic estimation. */
	mrpt::containers::deepcopy_poly_ptr<mrpt::poses::CPosePDF::Ptr> poseChange;
	/** This is the raw odometry reading, and only is used when
	 * "estimationMethod" is "TEstimationMethod::emOdometry" */
	mrpt::poses::CPose2D rawOdometryIncrementReading;
	/** This fields indicates the way in which this estimation was obtained. */
	TEstimationMethod estimationMethod{emOdometry};

	/** If "true" means that "encoderLeftTicks" and "encoderRightTicks" contain
	 * valid values. */
	bool hasEncodersInfo{false};
	/** For odometry only: the ticks count for each wheel FROM the last reading
	 * (positive means FORWARD, for both wheels);
	 * \sa hasEncodersInfo
	 */
	int32_t encoderLeftTicks{0}, encoderRightTicks{0};

	/** If "true" means that "velocityLin" and "velocityAng" contain valid
	 * values. */
	bool hasVelocities{false};
	/** If "hasVelocities"=true, the robot velocity in local (robot frame, +X
	 * forward) coordinates. */
	mrpt::math::TTwist2D velocityLocal{.0, .0, .0};

	double velocityLin() const { return velocityLocal.vx; }
	double velocityAng() const { return velocityLocal.omega; }
	enum TDrawSampleMotionModel
	{
		mmGaussian = 0,
		mmThrun
	};
	/** The parameter to be passed to "computeFromOdometry". */
	struct TMotionModelOptions
	{
		/** Default values loader. */
		TMotionModelOptions();

		/** The model to be used. */
		TDrawSampleMotionModel modelSelection{
			CActionRobotMovement2D::mmGaussian};

		/** Options for the gaussian model, which generates a CPosePDFGaussian
		 * object in poseChange
		 * See docs in :
		 * http://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
		 */
		struct TOptions_GaussianModel
		{
			float a1, a2, a3, a4, minStdXY, minStdPHI;
		} gaussianModel;

		/** Options for the Thrun's model, which generates a CPosePDFParticles
		 * object in poseChange
		 * See docs in :
		 * http://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
		 */
		struct TOptions_ThrunModel
		{
			/** The default number of particles to generate in a internal
			 * representation (anyway you can draw as many samples as you want
			 * through CActionRobotMovement2D::drawSingleSample)  */
			uint32_t nParticlesCount;
			float alfa1_rot_rot;
			float alfa2_rot_trans;
			float alfa3_trans_trans;
			float alfa4_trans_rot;

			/** An additional noise added to the thrun model (std. dev. in
			 * meters and radians). */
			float additional_std_XY, additional_std_phi;
		} thrunModel;

	} motionModelConfiguration;

	/** Computes the PDF of the pose increment from an odometry reading and
	 * according to the given motion model (speed and encoder ticks information
	 * is not modified).
	 * According to the parameters in the passed struct, it will be called one
	 * the private sampling functions (see "see also" next).
	 * \sa computeFromOdometry_modelGaussian, computeFromOdometry_modelThrun
	 */
	void computeFromOdometry(
		const mrpt::poses::CPose2D& odometryIncrement,
		const TMotionModelOptions& options);

	/** If "hasEncodersInfo"=true, this method updates the pose estimation
	 * according to the ticks from both encoders and the passed parameters,
	 * which is passed internally to the method "computeFromOdometry" with the
	 * last used PDF options (or the defualt ones if not explicitly called by
	 * the user).
	 *
	 * \param K_left The meters / tick ratio for the left encoder.
	 * \param K_right The meters / tick ratio for the right encoder.
	 * \param D The distance between both wheels, in meters.
	 */
	void computeFromEncoders(double K_left, double K_right, double D);

	/** Using this method instead of "poseChange->drawSingleSample()" may be
	 * more efficient in most situations.
	 * \sa CPosePDF::drawSingleSample
	 */
	void drawSingleSample(mrpt::poses::CPose2D& outSample) const;

	/** Call this before calling a high number of times "fastDrawSingleSample",
	 * which is much faster than "drawSingleSample"
	 */
	void prepareFastDrawSingleSamples() const;

	/** Faster version than "drawSingleSample", but requires a previous call to
	 * "prepareFastDrawSingleSamples"
	 */
	void fastDrawSingleSample(mrpt::poses::CPose2D& outSample) const;

   protected:
	/** Computes the PDF of the pose increment from an odometry reading, using a
	 * Gaussian approximation as the motion model.
	 * \sa computeFromOdometry
	 */
	void computeFromOdometry_modelGaussian(
		const mrpt::poses::CPose2D& odometryIncrement,
		const TMotionModelOptions& o);

	/** Computes the PDF of the pose increment from an odometry reading, using
	 * the motion model from Thrun's book.
	 * This model is discussed in "Probabilistic Robotics", Thrun, Burgard, and
	 * Fox, 2006, pp.136.
	 * \sa computeFromOdometry
	 */
	void computeFromOdometry_modelThrun(
		const mrpt::poses::CPose2D& odometryIncrement,
		const TMotionModelOptions& o);

	/** The sample generator for the model "computeFromOdometry_modelGaussian",
	 * internally called when the user invokes "drawSingleSample".
	 */
	void drawSingleSample_modelGaussian(mrpt::poses::CPose2D& outSample) const;

	/** The sample generator for the model "computeFromOdometry_modelThrun",
	 * internally called when the user invokes "drawSingleSample".
	 */
	void drawSingleSample_modelThrun(mrpt::poses::CPose2D& outSample) const;

	/** Internal use
	 */
	void prepareFastDrawSingleSample_modelGaussian() const;

	/** Internal use
	 */
	void prepareFastDrawSingleSample_modelThrun() const;

	/** Internal use
	 */
	void fastDrawSingleSample_modelGaussian(
		mrpt::poses::CPose2D& outSample) const;

	/** Internal use
	 */
	void fastDrawSingleSample_modelThrun(mrpt::poses::CPose2D& outSample) const;

	/** Auxiliary matrix
	 */
	mutable mrpt::math::CMatrixDouble33 m_fastDrawGauss_Z;
	mutable mrpt::poses::CPose2D m_fastDrawGauss_M;

};  // End of class def.

}  // namespace mrpt::obs
