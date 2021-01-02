/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/typemeta/TEnumType.h>

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
	DEFINE_SERIALIZABLE(CActionRobotMovement2D, mrpt::obs)

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
		TMotionModelOptions() = default;

		/** The model to be used. */
		TDrawSampleMotionModel modelSelection{mmGaussian};

		/** Options for the gaussian model, which generates a CPosePDFGaussian
		 * object in poseChange using a closed-form linear Gaussian model.
		 * See docs in:
		 * https://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
		 */
		struct TOptions_GaussianModel
		{
			TOptions_GaussianModel() = default;
			TOptions_GaussianModel(
				double a1_, double a2_, double a3_, double a4_,
				double minStdXY_, double minStdPHI_)
				: a1(a1_),
				  a2(a2_),
				  a3(a3_),
				  a4(a4_),
				  minStdXY(minStdXY_),
				  minStdPHI(minStdPHI_)
			{
			}

			/** Ratio of uncertainty: [meter/meter] */
			double a1{0.01};
			/** Ratio of uncertainty: [meter/degree] */
			double a2{mrpt::RAD2DEG(0.001)};
			/** Ratio of uncertainty: [degree/meter] */
			double a3{mrpt::DEG2RAD(1.0)};
			/** Ratio of uncertainty: [degree/degree] */
			double a4{0.05};
			/** Additional uncertainty: [meters] */
			double minStdXY{0.01};
			/** Additional uncertainty: [degrees] */
			double minStdPHI{mrpt::DEG2RAD(0.2)};
		};

		TOptions_GaussianModel gaussianModel;

		/** Options for the Thrun's model, which generates a CPosePDFParticles
		 * object in poseChange using a MonteCarlo simulation.
		 * See docs in:
		 * https://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
		 */
		struct TOptions_ThrunModel
		{
			/** The default number of particles to generate in a internal
			 * representation (anyway you can draw as many samples as you want
			 * through CActionRobotMovement2D::drawSingleSample)  */
			uint32_t nParticlesCount{300};
			float alfa1_rot_rot{0.05f};
			float alfa2_rot_trans{mrpt::DEG2RAD(4.0f)};
			float alfa3_trans_trans{0.01f};
			float alfa4_trans_rot{mrpt::RAD2DEG(0.0001f)};

			/** An additional noise added to the thrun model (std. dev. in
			 * meters and radians). */
			float additional_std_XY{0.001f};
			float additional_std_phi{mrpt::DEG2RAD(0.05f)};
		};

		TOptions_ThrunModel thrunModel;
	};

	TMotionModelOptions motionModelConfiguration;

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

	virtual void getDescriptionAsText(std::ostream& o) const override;

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

MRPT_ENUM_TYPE_BEGIN(mrpt::obs::CActionRobotMovement2D::TEstimationMethod)
MRPT_FILL_ENUM_MEMBER(mrpt::obs::CActionRobotMovement2D, emOdometry);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::CActionRobotMovement2D, emScan2DMatching);
MRPT_ENUM_TYPE_END()
