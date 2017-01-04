/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/random.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CActionRobotMovement3D, CAction, mrpt::obs)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CActionRobotMovement3D::CActionRobotMovement3D() :
	poseChange(),
	rawOdometryIncrementReading(),
	estimationMethod( emOdometry ),
	motionModelConfiguration(),
	hasVelocities(6,false),
	velocities(6)
{
	velocities.assign(.0);
}


/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CActionRobotMovement3D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t		i  = static_cast<uint32_t>( estimationMethod );

		out << i;

		// The PDF:
		out << poseChange;

		out << hasVelocities << velocities;

		out << timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CActionRobotMovement3D::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t	i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			in >> poseChange;
			in >> hasVelocities >> velocities;

			if (version>=1)
					in >> timestamp;
			else	timestamp = INVALID_TIMESTAMP;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}


void  CActionRobotMovement3D::computeFromOdometry(
	const CPose3D				&odometryIncrement,
	const TMotionModelOptions	&options)
{
	// Set the members data:
	estimationMethod			= emOdometry;
	rawOdometryIncrementReading	= odometryIncrement;
	motionModelConfiguration	= options;

	if ( options.modelSelection == mm6DOF )
				computeFromOdometry_model6DOF( odometryIncrement, options );
}


/*---------------------------------------------------------------
						TMotionModelOptions
 ---------------------------------------------------------------*/
CActionRobotMovement3D::TMotionModelOptions::TMotionModelOptions() :
modelSelection(mm6DOF),
	mm6DOFModel()
{

	mm6DOFModel.nParticlesCount		= 300;
	mm6DOFModel.a1		= 0.05f;
	mm6DOFModel.a2		= 0.05f;
	mm6DOFModel.a3		= 0.05f;
	mm6DOFModel.a4		= 0.05f;
	mm6DOFModel.a5		= 0.05f;
	mm6DOFModel.a6		= 0.05f;
	mm6DOFModel.a7		= 0.05f;
	mm6DOFModel.a8		= 0.05f;
	mm6DOFModel.a9		= 0.05f;
	mm6DOFModel.a10		= 0.05f;
	mm6DOFModel.additional_std_XYZ	= 0.001f;
	mm6DOFModel.additional_std_angle	= DEG2RAD(0.05f);

}

/*---------------------------------------------------------------
				computeFromOdometry_model6DOF
	---------------------------------------------------------------*/
void  CActionRobotMovement3D::computeFromOdometry_model6DOF(
	const CPose3D				&odometryIncrement,
	const TMotionModelOptions	&o
	)
{
	// The Gaussian PDF:
	// ---------------------------
	CPose3DPDFParticles		*aux;
	static CPose3D			nullPose(0,0,0,0,0,0);

	mrpt::poses::CPose3DPDFPtr poseChangeTemp = CPose3DPDFParticles::Create();
	aux = static_cast<CPose3DPDFParticles*>( poseChangeTemp.pointer() );

	// Set the number of particles:
	aux->resetDeterministic(nullPose, o.mm6DOFModel.nParticlesCount );

	//The motion model: A. L. Ballardini, A. Furlan, A. Galbiati, M. Matteucci, F. Sacchi, D. G. Sorrenti An effective 6DoF motion model for 3D-6DoF Monte Carlo Localization 4th Workshop on Planning, Perception and Navigation for Intelligent Vehicles, IROS, 2012

	/*
		The brief description:
		Find XYZ deltas: dX=x(t)-x(t-1); dY=y(t)-y(t-1); dZ=z(t)-z(t-1)
		Find angular deltas (normalised): dRoll=roll(t)-roll(t-1); dPitch=pitch(t)-pitch(t-1); dYaw=yaw(t)-yaw(t-1)

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
				trans_draw = trans + sample( a4 * trans + a5 * yaw2 + a6 * (roll + pitch2 ) )
				roll_draw = roll + sample(a7 * roll)
				pitch2_draw = pitch2 + sample(a8 * pitch2 )
				yaw2_draw = yaw2 + sample(a9 * yaw2 + a10 * trans )

		 Here sample() - sampling from zero mean Gaussian distribution

		Calculate the spherical coordinates:
				x = trans_draw * sin( pitch1_draw ) * cos ( yaw1_draw )
				y = trans_draw * sin( pitch1_draw ) * sin ( yaw1_draw )
				z = trans_draw * cos( pitch1_draw )
		 roll = roll_draw
		pitch = pitch1_draw + pitch2_draw
			yaw = yaw1_draw + yaw2_draw

		normalize_angle(roll, pitch, yaw )
	*/


	// The increments in odometry:
	float	Ayaw1	= ( odometryIncrement.y()!=0 || odometryIncrement.x()!=0) ?
						atan2( odometryIncrement.y(), odometryIncrement.x() ) : 0;

	float	 Atrans	= odometryIncrement.norm();

	float	Apitch1	= ( odometryIncrement.y()!=0 || odometryIncrement.x()!=0 || odometryIncrement.z()!=0) ?
						atan2( odometryIncrement.z(),	odometryIncrement.norm()) : 0;

	float Aroll =	odometryIncrement.roll();
	float Apitch2 = odometryIncrement.pitch();
	float Ayaw2 = odometryIncrement.yaw();



	// Draw samples:
	for (size_t i=0;i<o.mm6DOFModel.nParticlesCount;i++)
	{
		float	Ayaw1_draw	= Ayaw1	+ (o.mm6DOFModel.a1*Ayaw1 +o.mm6DOFModel.a2*Atrans ) * randomGenerator.drawGaussian1D_normalized();
		float	Apitch1_draw= Apitch1 + (o.mm6DOFModel.a3*odometryIncrement.z())* randomGenerator.drawGaussian1D_normalized();
		float	Atrans_draw = Atrans + (o.mm6DOFModel.a4*Atrans+ o.mm6DOFModel.a5*Ayaw2+ o.mm6DOFModel.a6*(Aroll+Apitch2)) * randomGenerator.drawGaussian1D_normalized();

		float Aroll_draw=Aroll+(o.mm6DOFModel.a7*Aroll) * randomGenerator.drawGaussian1D_normalized();
		float Apitch2_draw=Apitch2+(o.mm6DOFModel.a8*Apitch2)* randomGenerator.drawGaussian1D_normalized();
		float Ayaw2_draw=Ayaw2 +(o.mm6DOFModel.a9*Ayaw2+o.mm6DOFModel.a10*Atrans)* randomGenerator.drawGaussian1D_normalized();

		// Output:
		aux->m_particles[i].d->x( Atrans_draw * sin( Apitch1_draw )*cos(Ayaw1_draw) + motionModelConfiguration.mm6DOFModel.additional_std_XYZ * randomGenerator.drawGaussian1D_normalized() );
		aux->m_particles[i].d->y( Atrans_draw * sin( Apitch1_draw )*sin(Ayaw1_draw) + motionModelConfiguration.mm6DOFModel.additional_std_XYZ * randomGenerator.drawGaussian1D_normalized() );
		aux->m_particles[i].d->z( Atrans_draw * cos( Apitch1_draw ) + motionModelConfiguration.mm6DOFModel.additional_std_XYZ * randomGenerator.drawGaussian1D_normalized() );


		double new_yaw= Ayaw1_draw + Ayaw2_draw + motionModelConfiguration.mm6DOFModel.additional_std_angle * randomGenerator.drawGaussian1D_normalized();
		double new_pitch=	Apitch1_draw + Apitch2_draw + motionModelConfiguration.mm6DOFModel.additional_std_angle * randomGenerator.drawGaussian1D_normalized() ;
		double new_roll = Aroll_draw + motionModelConfiguration.mm6DOFModel.additional_std_angle * randomGenerator.drawGaussian1D_normalized() ;

		aux->m_particles[i].d->setYawPitchRoll(new_yaw,new_pitch,new_roll);
		aux->m_particles[i].d->normalizeAngles();

	}

		poseChange.copyFrom(*poseChangeTemp);
}
