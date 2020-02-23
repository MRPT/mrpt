/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::random;

IMPLEMENTS_SERIALIZABLE(CActionRobotMovement3D, CAction, mrpt::obs)

uint8_t CActionRobotMovement3D::serializeGetVersion() const { return 1; }
void CActionRobotMovement3D::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(estimationMethod);
	out << poseChange;
	out << hasVelocities << velocities;
	out << timestamp;
}

void CActionRobotMovement3D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			uint32_t i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			in >> poseChange;
			in >> hasVelocities >> velocities;

			if (version >= 1)
				in >> timestamp;
			else
				timestamp = INVALID_TIMESTAMP;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CActionRobotMovement3D::computeFromOdometry(
	const CPose3D& odometryIncrement, const TMotionModelOptions& options)
{
	// Set the members data:
	estimationMethod = emOdometry;
	rawOdometryIncrementReading = odometryIncrement;
	motionModelConfiguration = options;

	if (options.modelSelection == mm6DOF)
		computeFromOdometry_model6DOF(odometryIncrement, options);
}

/*---------------------------------------------------------------
				computeFromOdometry_model6DOF
	---------------------------------------------------------------*/
void CActionRobotMovement3D::computeFromOdometry_model6DOF(
	const CPose3D& odometryIncrement, const TMotionModelOptions& o)
{
	// The Gaussian PDF:
	// ---------------------------
	CPose3DPDFParticles* aux;
	const mrpt::math::TPose3D nullPose(0, 0, 0, 0, 0, 0);

	mrpt::poses::CPose3DPDF::Ptr poseChangeTemp =
		std::make_shared<CPose3DPDFParticles>();
	aux = dynamic_cast<CPose3DPDFParticles*>(poseChangeTemp.get());

	// Set the number of particles:
	aux->resetDeterministic(nullPose, o.mm6DOFModel.nParticlesCount);

	// The motion model: A. L. Ballardini, A. Furlan, A. Galbiati, M. Matteucci,
	// F. Sacchi, D. G. Sorrenti An effective 6DoF motion model for 3D-6DoF
	// Monte Carlo Localization 4th Workshop on Planning, Perception and
	// Navigation for Intelligent Vehicles, IROS, 2012

	/*
		The brief description:
		Find XYZ deltas: dX=x(t)-x(t-1); dY=y(t)-y(t-1); dZ=z(t)-z(t-1)
		Find angular deltas (normalised): dRoll=roll(t)-roll(t-1);
	   dPitch=pitch(t)-pitch(t-1); dYaw=yaw(t)-yaw(t-1)

		Here t - is time step

		 Calculate the odometry for 6DOF:


				yaw1 = atan2 (dY,dX)
				pitch1 = atan2 (dZ , sqrt(dX^2 + dY^2 + dZ^2) )
				trans = sqrt(dX^2 + dY^2 + dZ^2)
				roll = dRoll
				yaw2= dYaw
				pitch2= dPitch


		Model the error:


				yaw1_draw = yaw1 + sample(a1 * yaw1 + a2 * trans )
				pitch1_draw = pitch1 + sample(a3 * dZ )
				trans_draw = trans + sample( a4 * trans + a5 * yaw2 + a6 * (roll
	   + pitch2 ) )
				roll_draw = roll + sample(a7 * roll)
				pitch2_draw = pitch2 + sample(a8 * pitch2 )
				yaw2_draw = yaw2 + sample(a9 * yaw2 + a10 * trans )

		 Here sample() - sampling from zero mean Gaussian distribution

		Calculate the spherical coordinates:
		// Original:
				x = trans_draw * sin( pitch1_draw ) * cos ( yaw1_draw )
				y = trans_draw * sin( pitch1_draw ) * sin ( yaw1_draw )
				z = trans_draw * cos( pitch1_draw )

		// Corrected (?) by JLBC (Jun 2019)
				x = trans_draw * cos( pitch1_draw ) * cos ( yaw1_draw )
				y = trans_draw * cos( pitch1_draw ) * sin ( yaw1_draw )
				z = -trans_draw * sin( pitch1_draw )

		 roll = roll_draw
		pitch = pitch1_draw + pitch2_draw
			yaw = yaw1_draw + yaw2_draw

		normalize_angle(roll, pitch, yaw )
	*/

	// The increments in odometry:
	double Ayaw1 = (odometryIncrement.y() != 0 || odometryIncrement.x() != 0)
					   ? atan2(odometryIncrement.y(), odometryIncrement.x())
					   : 0;

	double Atrans = odometryIncrement.norm();

	double Apitch1 =
		(odometryIncrement.y() != 0 || odometryIncrement.x() != 0 ||
		 odometryIncrement.z() != 0)
			? atan2(odometryIncrement.z(), odometryIncrement.norm())
			: 0;

	double Aroll = odometryIncrement.roll();
	double Apitch2 = odometryIncrement.pitch();
	double Ayaw2 = odometryIncrement.yaw();

	const auto stdxyz = motionModelConfiguration.mm6DOFModel.additional_std_XYZ;
	auto& rnd = mrpt::random::getRandomGenerator();

	// Draw samples:
	for (size_t i = 0; i < o.mm6DOFModel.nParticlesCount; i++)
	{
		auto& ith_part = aux->m_particles[i].d;
		double Ayaw1_draw =
			Ayaw1 + (o.mm6DOFModel.a1 * Ayaw1 + o.mm6DOFModel.a2 * Atrans) *
						rnd.drawGaussian1D_normalized();
		double Apitch1_draw =
			Apitch1 + (o.mm6DOFModel.a3 * odometryIncrement.z()) *
						  rnd.drawGaussian1D_normalized();
		double Atrans_draw =
			Atrans + (o.mm6DOFModel.a4 * Atrans + o.mm6DOFModel.a5 * Ayaw2 +
					  o.mm6DOFModel.a6 * (Aroll + Apitch2)) *
						 rnd.drawGaussian1D_normalized();

		double Aroll_draw = Aroll + (o.mm6DOFModel.a7 * Aroll) *
										rnd.drawGaussian1D_normalized();
		double Apitch2_draw = Apitch2 + (o.mm6DOFModel.a8 * Apitch2) *
											rnd.drawGaussian1D_normalized();
		double Ayaw2_draw =
			Ayaw2 + (o.mm6DOFModel.a9 * Ayaw2 + o.mm6DOFModel.a10 * Atrans) *
						rnd.drawGaussian1D_normalized();

		// Output:
		ith_part.x = Atrans_draw * cos(Apitch1_draw) * cos(Ayaw1_draw) +
					 stdxyz * rnd.drawGaussian1D_normalized();
		ith_part.y = Atrans_draw * cos(Apitch1_draw) * sin(Ayaw1_draw) +
					 stdxyz * rnd.drawGaussian1D_normalized();
		ith_part.z = -Atrans_draw * sin(Apitch1_draw) +
					 stdxyz * rnd.drawGaussian1D_normalized();

		ith_part.yaw =
			Ayaw1_draw + Ayaw2_draw +
			motionModelConfiguration.mm6DOFModel.additional_std_angle *
				rnd.drawGaussian1D_normalized();
		ith_part.pitch =
			Apitch1_draw + Apitch2_draw +
			motionModelConfiguration.mm6DOFModel.additional_std_angle *
				rnd.drawGaussian1D_normalized();
		ith_part.roll =
			Aroll_draw +
			motionModelConfiguration.mm6DOFModel.additional_std_angle *
				rnd.drawGaussian1D_normalized();
	}

	poseChange.copyFrom(*poseChangeTemp);
}

void CActionRobotMovement3D::getDescriptionAsText(std::ostream& o) const
{
	CAction::getDescriptionAsText(o);

	o << "Robot Movement (as a gaussian pose change):\n";
	o << poseChange << "\n";
}
