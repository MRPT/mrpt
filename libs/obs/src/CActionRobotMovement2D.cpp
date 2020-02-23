/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/point_poses2vectors.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CActionRobotMovement2D, CAction, mrpt::obs)

// Note: dont move this to the .h, to avoid having to include the definition of
// the full pose type
CActionRobotMovement2D::CActionRobotMovement2D()
	: poseChange(mrpt::poses::CPosePDFGaussian::Create())
{
	// Re-build the PDF to have a consistent object state:
	computeFromOdometry(rawOdometryIncrementReading, motionModelConfiguration);
}

uint8_t CActionRobotMovement2D::serializeGetVersion() const { return 7; }
void CActionRobotMovement2D::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(estimationMethod);
	// Added in version 2:
	// If the estimation method is emOdometry, save the rawOdo + config data
	// instead of
	//  the PDF itself:
	if (estimationMethod == emOdometry)
	{
		// The odometry data:
		out << rawOdometryIncrementReading;
		out.WriteAs<uint32_t>(motionModelConfiguration.modelSelection);
		const auto& gm = motionModelConfiguration.gaussianModel;
		out.WriteAs<float>(gm.a1);
		out.WriteAs<float>(gm.a2);
		out.WriteAs<float>(gm.a3);
		out.WriteAs<float>(gm.a4);
		out.WriteAs<float>(gm.minStdXY);
		out.WriteAs<float>(gm.minStdPHI);

		out << motionModelConfiguration.thrunModel.nParticlesCount
			<< motionModelConfiguration.thrunModel.alfa1_rot_rot
			<< motionModelConfiguration.thrunModel.alfa2_rot_trans
			<< motionModelConfiguration.thrunModel.alfa3_trans_trans
			<< motionModelConfiguration.thrunModel.alfa4_trans_rot
			<< motionModelConfiguration.thrunModel.additional_std_XY
			<< motionModelConfiguration.thrunModel.additional_std_phi;
	}
	else
	{
		// The PDF:
		out << (*poseChange);
	}

	// Added in version 1:
	out << hasVelocities;
	if (hasVelocities) out << velocityLocal;  // v7

	out << hasEncodersInfo;
	if (hasEncodersInfo)
		out << encoderLeftTicks << encoderRightTicks;  // added if() in v7

	// Added in version 6
	out << timestamp;
}

void CActionRobotMovement2D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 4:
		case 5:
		case 6:
		case 7:
		{
			int32_t i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			// If the estimation method is emOdometry, save the rawOdo + config
			// data instead of
			//  the PDF itself:
			if (estimationMethod == emOdometry)
			{
				// The odometry data:
				in >> rawOdometryIncrementReading;

				in >> i;
				motionModelConfiguration.modelSelection =
					static_cast<TDrawSampleMotionModel>(i);

				auto& gm = motionModelConfiguration.gaussianModel;
				gm.a1 = in.ReadAs<float>();
				gm.a2 = in.ReadAs<float>();
				gm.a3 = in.ReadAs<float>();
				gm.a4 = in.ReadAs<float>();
				gm.minStdXY = in.ReadAs<float>();
				gm.minStdPHI = in.ReadAs<float>();

				in >> i;
				motionModelConfiguration.thrunModel.nParticlesCount = i;
				in >> motionModelConfiguration.thrunModel.alfa1_rot_rot >>
					motionModelConfiguration.thrunModel.alfa2_rot_trans >>
					motionModelConfiguration.thrunModel.alfa3_trans_trans >>
					motionModelConfiguration.thrunModel.alfa4_trans_rot;

				if (version >= 5)
				{
					in >>
						motionModelConfiguration.thrunModel.additional_std_XY >>
						motionModelConfiguration.thrunModel.additional_std_phi;
				}
				else
				{
					motionModelConfiguration.thrunModel.additional_std_XY =
						motionModelConfiguration.thrunModel.additional_std_phi =
							0;
				}

				// Re-build the PDF:
				computeFromOdometry(
					rawOdometryIncrementReading, motionModelConfiguration);
			}
			else
			{
				// Read the PDF directly from the stream:
				CPosePDF::Ptr pc;
				in >> pc;
				poseChange = pc;
			}

			in >> hasVelocities;
			if (version >= 7)
			{
				if (hasVelocities) in >> velocityLocal;
			}
			else
			{
				double velocityLin, velocityAng;
				in >> velocityLin >> velocityAng;
				velocityLocal.vx = velocityLin;
				velocityLocal.vy = .0f;
				velocityLocal.omega = velocityAng;
			}
			in >> hasEncodersInfo;
			if (version < 7 || hasEncodersInfo)
			{
				in >> i;
				encoderLeftTicks = i;
				in >> i;
				encoderRightTicks = i;
			}
			else
			{
				encoderLeftTicks = 0;
				encoderRightTicks = 0;
			}

			if (version >= 6)
				in >> timestamp;
			else
				timestamp = INVALID_TIMESTAMP;
		}
		break;

		case 3:
		{
			int32_t i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			// If the estimation method is emOdometry, save the rawOdo + config
			// data instead of
			//  the PDF itself:
			if (estimationMethod == emOdometry)
			{
				// The odometry data:
				in >> rawOdometryIncrementReading;

				in >> i;
				motionModelConfiguration.modelSelection =
					static_cast<TDrawSampleMotionModel>(i);

				double dum1, dum2, dum3;

				in >> dum1 >> dum2 >> dum3 >>
					motionModelConfiguration.gaussianModel.minStdXY >>
					motionModelConfiguration.gaussianModel.minStdPHI;

				// Leave the default values for a1,a2,a3,a4:
				in >> i;
				motionModelConfiguration.thrunModel.nParticlesCount = i;
				in >> motionModelConfiguration.thrunModel.alfa1_rot_rot;
				in >> motionModelConfiguration.thrunModel.alfa2_rot_trans >>
					motionModelConfiguration.thrunModel.alfa3_trans_trans >>
					motionModelConfiguration.thrunModel.alfa4_trans_rot;

				// Re-build the PDF:
				computeFromOdometry(
					rawOdometryIncrementReading, motionModelConfiguration);
			}
			else
			{
				// Read the PDF directly from the stream:
				CPosePDF::Ptr pc;
				in >> pc;
				poseChange = pc;
			}

			in >> hasVelocities;
			{
				double velocityLin, velocityAng;
				in >> velocityLin >> velocityAng;
				velocityLocal.vx = velocityLin;
				velocityLocal.vy = .0f;
				velocityLocal.omega = velocityAng;
			}
			in >> hasEncodersInfo;

			in >> i;
			encoderLeftTicks = i;
			in >> i;
			encoderRightTicks = i;
		}
		break;

		case 2:
		{
			int32_t i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			// If the estimation method is emOdometry, save the rawOdo + config
			// data instead of
			//  the PDF itself:
			if (estimationMethod == emOdometry)
			{
				// The odometry data:
				in >> rawOdometryIncrementReading;

				//				TMotionModelOptions_OLD_VERSION_2	dummy;
				//				in.ReadBuffer( &dummy,
				// sizeof(TMotionModelOptions_OLD_VERSION_2) );
				uint8_t dummy[44];
				in.ReadBuffer(&dummy, sizeof(dummy));

				motionModelConfiguration = TMotionModelOptions();

				// Re-build the PDF:
				computeFromOdometry(
					rawOdometryIncrementReading, motionModelConfiguration);
			}
			else
			{
				// Read the PDF directly from the stream:
				CPosePDF::Ptr pc;
				in >> pc;
				poseChange = pc;
			}

			in >> hasVelocities;
			{
				double velocityLin, velocityAng;
				in >> velocityLin >> velocityAng;
				velocityLocal.vx = velocityLin;
				velocityLocal.vy = .0f;
				velocityLocal.omega = velocityAng;
			}
			in >> hasEncodersInfo;
			in >> i;
			encoderLeftTicks = i;
			in >> i;
			encoderRightTicks = i;
		}
		break;
		case 0:
		case 1:
		{
			int32_t i;
			{
				CPosePDF::Ptr pc;
				in >> pc;
				poseChange = pc;
			}
			in >> i;

			// copy:
			estimationMethod = static_cast<TEstimationMethod>(i);

			// Simulate the "rawOdometryIncrementReading" as the mean value:
			if (estimationMethod == emOdometry)
				poseChange->getMean(rawOdometryIncrementReading);
			else
				rawOdometryIncrementReading = CPose2D(0, 0, 0);

			if (version >= 1)
			{
				in >> hasVelocities;
				{
					double velocityLin, velocityAng;
					in >> velocityLin >> velocityAng;
					velocityLocal.vx = velocityLin;
					velocityLocal.vy = .0f;
					velocityLocal.omega = velocityAng;
				}
				in >> hasEncodersInfo;
				in >> i;
				encoderLeftTicks = i;
				in >> i;
				encoderRightTicks = i;
			}
			else
			{
				// Default values:
				hasVelocities = hasEncodersInfo = false;
				encoderLeftTicks = encoderRightTicks = 0;
				velocityLocal = mrpt::math::TTwist2D(.0, .0, .0);
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
					computeFromEncoders
 ---------------------------------------------------------------*/
void CActionRobotMovement2D::computeFromEncoders(
	double K_left, double K_right, double D)
{
	if (hasEncodersInfo)
	{
		const double As =
			0.5 * (K_right * encoderRightTicks + K_left * encoderLeftTicks);
		const double Aphi =
			(K_right * encoderRightTicks - K_left * encoderLeftTicks) / D;

		double x, y;
		if (Aphi != 0)
		{
			const double R = As / Aphi;
			x = R * sin(Aphi);
			y = R * (1 - cos(Aphi));
		}
		else
		{
			x = As;
			y = 0;
		}

		// Build the whole PDF with the current parameters:
		computeFromOdometry(CPose2D(x, y, Aphi), motionModelConfiguration);
	}
}

/*---------------------------------------------------------------
					computeFromOdometry
 ---------------------------------------------------------------*/
void CActionRobotMovement2D::computeFromOdometry(
	const CPose2D& odometryIncrement, const TMotionModelOptions& options)
{
	// Set the members data:
	estimationMethod = emOdometry;
	rawOdometryIncrementReading = odometryIncrement;
	motionModelConfiguration = options;

	if (options.modelSelection == mmGaussian)
		computeFromOdometry_modelGaussian(odometryIncrement, options);
	else
		computeFromOdometry_modelThrun(odometryIncrement, options);
}

/*---------------------------------------------------------------
				computeFromOdometry_modelGaussian
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::computeFromOdometry_modelGaussian(
	const CPose2D& odometryIncrement, const TMotionModelOptions& o)
{
	// The Gaussian PDF:
	// ---------------------------
	poseChange = std::make_shared<CPosePDFGaussian>();
	auto* aux = dynamic_cast<CPosePDFGaussian*>(poseChange.get());
	ASSERT_(aux != nullptr);

	// See https://www.mrpt.org/Probabilistic_Motion_Models
	// -----------------------------------

	// Build the odometry noise matrix:
	double Al = odometryIncrement.norm();
	auto ODO_INCR = CMatrixDouble31(odometryIncrement);
	CMatrixDouble33 C_ODO;
	C_ODO(0, 0) = square(
		o.gaussianModel.minStdXY + o.gaussianModel.a1 * Al +
		o.gaussianModel.a2 * fabs(odometryIncrement.phi()));
	C_ODO(1, 1) = square(
		o.gaussianModel.minStdXY + o.gaussianModel.a1 * Al +
		o.gaussianModel.a2 * fabs(odometryIncrement.phi()));
	C_ODO(2, 2) = square(
		o.gaussianModel.minStdPHI + o.gaussianModel.a3 * Al +
		o.gaussianModel.a4 * fabs(odometryIncrement.phi()));

	// Build the transformation matrix:
	CMatrixDouble33 H;

	double cos_Aphi_2 = cos(odometryIncrement.phi() / 2);
	double sin_Aphi_2 = sin(odometryIncrement.phi() / 2);

	H(0, 0) = H(1, 1) = cos_Aphi_2;
	H(0, 1) = -(H(1, 0) = sin_Aphi_2);
	H(2, 2) = 1;

	// Build the Jacobian matrix:
	CMatrixDouble33 J = H;
	J(0, 2) = -sin_Aphi_2 * ODO_INCR(0, 0) - cos_Aphi_2 * ODO_INCR(1, 0);
	J(1, 2) = cos_Aphi_2 * ODO_INCR(0, 0) - sin_Aphi_2 * ODO_INCR(1, 0);

	// The mean:
	aux->mean = odometryIncrement;

	// The covariance:
	aux->cov = mrpt::math::multiply_HCHt(J, C_ODO);
}

/*---------------------------------------------------------------
				computeFromOdometry_modelThrun
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::computeFromOdometry_modelThrun(
	const CPose2D& odometryIncrement, const TMotionModelOptions& o)
{
	// The Gaussian PDF:
	// ---------------------------
	const mrpt::math::TPose2D nullPose(0, 0, 0);

	poseChange = CPosePDFParticles::Create();
	auto* aux = dynamic_cast<CPosePDFParticles*>(poseChange.get());
	ASSERT_(aux != nullptr);

	// Set the number of particles:
	aux->resetDeterministic(nullPose, o.thrunModel.nParticlesCount);

	// ---------------------------------------------------------------------------------------------
	// The following is an implementation from Thrun et al.'s book
	// (Probabilistic Robotics),
	//  page 136. Here "odometryIncrement" actually represents the incremental
	//  odometry difference.
	// ---------------------------------------------------------------------------------------------

	// The increments in odometry:
	double Arot1 = (odometryIncrement.y() != 0 || odometryIncrement.x() != 0)
					   ? atan2(odometryIncrement.y(), odometryIncrement.x())
					   : 0;
	double Atrans = odometryIncrement.norm();
	double Arot2 = math::wrapToPi(odometryIncrement.phi() - Arot1);

	// Draw samples:
	for (size_t i = 0; i < o.thrunModel.nParticlesCount; i++)
	{
		double Arot1_draw =
			Arot1 - (o.thrunModel.alfa1_rot_rot * fabs(Arot1) +
					 o.thrunModel.alfa2_rot_trans * Atrans) *
						getRandomGenerator().drawGaussian1D_normalized();
		double Atrans_draw =
			Atrans -
			(o.thrunModel.alfa3_trans_trans * Atrans +
			 o.thrunModel.alfa4_trans_rot * (fabs(Arot1) + fabs(Arot2))) *
				getRandomGenerator().drawGaussian1D_normalized();
		double Arot2_draw =
			Arot2 - (o.thrunModel.alfa1_rot_rot * fabs(Arot2) +
					 o.thrunModel.alfa2_rot_trans * Atrans) *
						getRandomGenerator().drawGaussian1D_normalized();

		// Output:
		aux->m_particles[i].d.x =
			Atrans_draw * cos(Arot1_draw) +
			motionModelConfiguration.thrunModel.additional_std_XY *
				getRandomGenerator().drawGaussian1D_normalized();
		aux->m_particles[i].d.y =
			Atrans_draw * sin(Arot1_draw) +
			motionModelConfiguration.thrunModel.additional_std_XY *
				getRandomGenerator().drawGaussian1D_normalized();
		aux->m_particles[i].d.phi =
			Arot1_draw + Arot2_draw +
			motionModelConfiguration.thrunModel.additional_std_phi *
				getRandomGenerator().drawGaussian1D_normalized();
		aux->m_particles[i].d.normalizePhi();
	}
}

/*---------------------------------------------------------------
				drawSingleSample
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::drawSingleSample(CPose2D& outSample) const
{
	// Only in the case of "emOdometry" we have the rawOdometryMeasurement and
	//  the parameters to draw new samples:
	if (estimationMethod == emOdometry)
	{
		if (motionModelConfiguration.modelSelection == mmGaussian)
			drawSingleSample_modelGaussian(outSample);
		else
			drawSingleSample_modelThrun(outSample);
	}
	else
	{
		// If is not odometry, just employ the stored distribution:
		poseChange->drawSingleSample(outSample);
	}
}

/*---------------------------------------------------------------
				drawSingleSample_modelGaussian
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::drawSingleSample_modelGaussian(
	CPose2D& outSample) const
{
	// In the Gaussian case it is more efficient just to
	// draw a sample from the already computed PDF:
	poseChange->drawSingleSample(outSample);
}

/*---------------------------------------------------------------
				drawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::drawSingleSample_modelThrun(
	CPose2D& outSample) const
{
	// ---------------------------------------------------------------------------------------------
	// The following is an implementation from Thrun et al.'s book
	// (Probabilistic Robotics),
	//  page 136. Here "odometryIncrement" actually represents the incremental
	//  odometry difference.
	// ---------------------------------------------------------------------------------------------

	// The increments in odometry:
	double Arot1 = (rawOdometryIncrementReading.y() != 0 ||
					rawOdometryIncrementReading.x() != 0)
					   ? atan2(
							 rawOdometryIncrementReading.y(),
							 rawOdometryIncrementReading.x())
					   : 0;
	double Atrans = rawOdometryIncrementReading.norm();
	double Arot2 = math::wrapToPi(rawOdometryIncrementReading.phi() - Arot1);

	double Arot1_draw =
		Arot1 -
		(motionModelConfiguration.thrunModel.alfa1_rot_rot * fabs(Arot1) +
		 motionModelConfiguration.thrunModel.alfa2_rot_trans * Atrans) *
			getRandomGenerator().drawGaussian1D_normalized();
	double Atrans_draw =
		Atrans -
		(motionModelConfiguration.thrunModel.alfa3_trans_trans * Atrans +
		 motionModelConfiguration.thrunModel.alfa4_trans_rot *
			 (fabs(Arot1) + fabs(Arot2))) *
			getRandomGenerator().drawGaussian1D_normalized();
	double Arot2_draw =
		Arot2 -
		(motionModelConfiguration.thrunModel.alfa1_rot_rot * fabs(Arot2) +
		 motionModelConfiguration.thrunModel.alfa2_rot_trans * Atrans) *
			getRandomGenerator().drawGaussian1D_normalized();

	// Output:
	outSample.x(
		Atrans_draw * cos(Arot1_draw) +
		motionModelConfiguration.thrunModel.additional_std_XY *
			getRandomGenerator().drawGaussian1D_normalized());
	outSample.y(
		Atrans_draw * sin(Arot1_draw) +
		motionModelConfiguration.thrunModel.additional_std_XY *
			getRandomGenerator().drawGaussian1D_normalized());
	outSample.phi(
		Arot1_draw + Arot2_draw +
		motionModelConfiguration.thrunModel.additional_std_phi *
			getRandomGenerator().drawGaussian1D_normalized());
	outSample.normalizePhi();
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSamples
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::prepareFastDrawSingleSamples() const
{
	// Only in the case of "emOdometry" we have the rawOdometryMeasurement and
	//  the parameters to draw new samples:
	if (estimationMethod == emOdometry)
	{
		if (motionModelConfiguration.modelSelection == mmGaussian)
			prepareFastDrawSingleSample_modelGaussian();
		else
			prepareFastDrawSingleSample_modelThrun();
	}
}

/*---------------------------------------------------------------
				fastDrawSingleSample
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::fastDrawSingleSample(CPose2D& outSample) const
{
	// Only in the case of "emOdometry" we have the rawOdometryMeasurement and
	//  the parameters to draw new samples:
	if (estimationMethod == emOdometry)
	{
		if (motionModelConfiguration.modelSelection == mmGaussian)
			fastDrawSingleSample_modelGaussian(outSample);
		else
			fastDrawSingleSample_modelThrun(outSample);
	}
	else
	{
		// If is not odometry, just employ the stored distribution:
		poseChange->drawSingleSample(outSample);
	}
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelGaussian
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::prepareFastDrawSingleSample_modelGaussian() const
{
	MRPT_START

	ASSERT_(IS_CLASS(*poseChange, CPosePDFGaussian));

	const auto* gPdf = dynamic_cast<const CPosePDFGaussian*>(poseChange.get());
	ASSERT_(gPdf != nullptr);
	const CMatrixDouble33& cov = gPdf->cov;

	m_fastDrawGauss_M = gPdf->mean;

	/** Computes the eigenvalues/eigenvector decomposition of this matrix,
	 *    so that: M = Z · D · Z<sup>T</sup>, where columns in Z are the
	 *	  eigenvectors and the diagonal matrix D contains the eigenvalues
	 *    as diagonal elements, sorted in <i>ascending</i> order.
	 */
	std::vector<double> eigvals;
	cov.eig_symmetric(m_fastDrawGauss_Z, eigvals);
	CMatrixDouble33 D;
	D.setDiagonal(eigvals);

	// Scale eigenvectors with eigenvalues:
	D.asEigen() = D.array().sqrt().matrix();
	m_fastDrawGauss_Z = m_fastDrawGauss_Z * D;

	MRPT_END
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::prepareFastDrawSingleSample_modelThrun() const {}
/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::fastDrawSingleSample_modelGaussian(
	CPose2D& outSample) const
{
	CVectorDouble rndVector;
	rndVector.assign(3, 0.0f);
	for (size_t i = 0; i < 3; i++)
	{
		double rnd = getRandomGenerator().drawGaussian1D_normalized();
		for (size_t d = 0; d < 3; d++)
			rndVector[d] += (m_fastDrawGauss_Z(d, i) * rnd);
	}

	outSample = CPose2D(
		m_fastDrawGauss_M.x() + rndVector[0],
		m_fastDrawGauss_M.y() + rndVector[1],
		m_fastDrawGauss_M.phi() + rndVector[2]);
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void CActionRobotMovement2D::fastDrawSingleSample_modelThrun(
	CPose2D& outSample) const
{
	drawSingleSample_modelThrun(outSample);
}

void CActionRobotMovement2D::getDescriptionAsText(std::ostream& o) const
{
	CAction::getDescriptionAsText(o);

	CPose2D Ap;
	CMatrixDouble33 mat;
	poseChange->getCovarianceAndMean(mat, Ap);

	o << "Robot Movement (as a gaussian pose change):\n";
	o << " Mean = " << Ap << "\n";

	o << format(" Covariance:     DET=%e\n", mat.det());

	o << format("      %e %e %e\n", mat(0, 0), mat(0, 1), mat(0, 2));
	o << format("      %e %e %e\n", mat(1, 0), mat(1, 1), mat(1, 2));
	o << format("      %e %e %e\n", mat(2, 0), mat(2, 1), mat(2, 2));

	o << "\n";

	o << "Actual odometry increment reading: " << rawOdometryIncrementReading
	  << "\n";

	o << format(
		"Actual PDF class is: '%s'\n",
		poseChange->GetRuntimeClass()->className);

	if (poseChange->GetRuntimeClass() == CLASS_ID(CPosePDFParticles))
	{
		CPosePDFParticles::Ptr aux =
			std::dynamic_pointer_cast<CPosePDFParticles>(poseChange.get_ptr());
		o << format(
			" (Particle count = %u)\n", (unsigned)aux->m_particles.size());
	}
	o << "\n";

	o << "Estimation method: "
	  << mrpt::typemeta::TEnumType<TEstimationMethod>::value2name(
			 estimationMethod)
	  << "\n";

	// Additional data:
	if (hasEncodersInfo)
		o << format(
			"Encoder info: deltaL=%i deltaR=%i\n", encoderLeftTicks,
			encoderRightTicks);
	else
		o << "Encoder info: Not available!\n";

	if (hasVelocities)
		o << "Velocity info: v=" << velocityLocal.asString() << "\n";
	else
		o << "Velocity info: Not available!\n";
}
