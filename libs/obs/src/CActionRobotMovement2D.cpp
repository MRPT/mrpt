/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/random.h>
#include <mrpt/math/point_poses2vectors.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CActionRobotMovement2D, CAction, mrpt::obs)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CActionRobotMovement2D::CActionRobotMovement2D() :
	poseChange(CPosePDFGaussian::Create()),
	rawOdometryIncrementReading(),
	estimationMethod(emOdometry),
	hasEncodersInfo(false),
	encoderLeftTicks(0),
	encoderRightTicks(0),
	hasVelocities(false),
	velocityLocal(.0,.0,.0),
	motionModelConfiguration(),
	m_fastDrawGauss_Z(),
	m_fastDrawGauss_M()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CActionRobotMovement2D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 7;
	else
	{
		uint32_t	i  = static_cast<uint32_t>(estimationMethod);

		out << i;

		// Added in version 2:
		// If the estimation method is emOdometry, save the rawOdo + config data instead of
		//  the PDF itself:
		if ( estimationMethod == emOdometry )
		{
			// The odometry data:
            out << rawOdometryIncrementReading;

			i = static_cast<uint32_t>(motionModelConfiguration.modelSelection);
			out << i;
			out << motionModelConfiguration.gaussianModel.a1
                << motionModelConfiguration.gaussianModel.a2
                << motionModelConfiguration.gaussianModel.a3
                << motionModelConfiguration.gaussianModel.a4
                << motionModelConfiguration.gaussianModel.minStdXY
                << motionModelConfiguration.gaussianModel.minStdPHI;

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
		if (hasVelocities)
			out << velocityLocal; // v7

		out << hasEncodersInfo;
		if (hasEncodersInfo)
			out << encoderLeftTicks << encoderRightTicks;  // added if() in v7

		// Added in version 6
		out << timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CActionRobotMovement2D::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 4:
	case 5:
	case 6:
	case 7:
		{
			int32_t	i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			// If the estimation method is emOdometry, save the rawOdo + config data instead of
			//  the PDF itself:
			if ( estimationMethod == emOdometry )
			{
				// The odometry data:
				in >> rawOdometryIncrementReading;

				in >> i;
				motionModelConfiguration.modelSelection = static_cast<TDrawSampleMotionModel>(i);

				in >> motionModelConfiguration.gaussianModel.a1
					>> motionModelConfiguration.gaussianModel.a2
					>> motionModelConfiguration.gaussianModel.a3
					>> motionModelConfiguration.gaussianModel.a4
					>> motionModelConfiguration.gaussianModel.minStdXY
					>> motionModelConfiguration.gaussianModel.minStdPHI;

				in  >> i; motionModelConfiguration.thrunModel.nParticlesCount=i;
				in  >> motionModelConfiguration.thrunModel.alfa1_rot_rot
					>> motionModelConfiguration.thrunModel.alfa2_rot_trans
					>> motionModelConfiguration.thrunModel.alfa3_trans_trans
					>> motionModelConfiguration.thrunModel.alfa4_trans_rot;

				if (version>=5)
				{
					in  >> motionModelConfiguration.thrunModel.additional_std_XY
						>> motionModelConfiguration.thrunModel.additional_std_phi;
				}
				else
				{
					motionModelConfiguration.thrunModel.additional_std_XY =
					motionModelConfiguration.thrunModel.additional_std_phi = 0;
				}

				// Re-build the PDF:
				computeFromOdometry( rawOdometryIncrementReading,motionModelConfiguration );
			}
			else
			{
				// Read the PDF directly from the stream:
				CPosePDFPtr pc;
				in >> pc;
				poseChange = pc;
			}

			in >> hasVelocities;
			if (version >= 7) {
				if (hasVelocities)
					in >> velocityLocal;
			}
			else {
				float velocityLin, velocityAng;
				in >> velocityLin >> velocityAng;
				velocityLocal.vx = velocityLin;
				velocityLocal.vy = .0f;
				velocityLocal.omega = velocityAng;
			}
			in >> hasEncodersInfo;
			if (version < 7 || hasEncodersInfo) {
				in >> i; encoderLeftTicks = i;
				in >> i; encoderRightTicks = i;
			}
			else {
				encoderLeftTicks = 0;
				encoderRightTicks = 0;
			}

			if (version>=6)
				in >> timestamp;
			else	timestamp = INVALID_TIMESTAMP;

		} break;

	case 3:
		{
			int32_t	i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			// If the estimation method is emOdometry, save the rawOdo + config data instead of
			//  the PDF itself:
			if ( estimationMethod == emOdometry )
			{
				// The odometry data:
				in >> rawOdometryIncrementReading;

				in >> i;
				motionModelConfiguration.modelSelection = static_cast<TDrawSampleMotionModel>(i);

				float   dum1,dum2,dum3;

				in >> dum1 >> dum2 >> dum3 >> motionModelConfiguration.gaussianModel.minStdXY >> motionModelConfiguration.gaussianModel.minStdPHI;

				// Leave the default values for a1,a2,a3,a4:
				in  >> i; motionModelConfiguration.thrunModel.nParticlesCount=i;
				in  >> motionModelConfiguration.thrunModel.alfa1_rot_rot;
				in  >> motionModelConfiguration.thrunModel.alfa2_rot_trans >> motionModelConfiguration.thrunModel.alfa3_trans_trans >> motionModelConfiguration.thrunModel.alfa4_trans_rot;

				// Re-build the PDF:
				computeFromOdometry( rawOdometryIncrementReading,motionModelConfiguration );
			}
			else
			{
				// Read the PDF directly from the stream:
				CPosePDFPtr pc;
				in >> pc;
				poseChange = pc;
			}

			in >> hasVelocities;
			{
				float velocityLin, velocityAng;
				in >> velocityLin >> velocityAng;
				velocityLocal.vx = velocityLin;
				velocityLocal.vy = .0f;
				velocityLocal.omega = velocityAng;
			}
			in >> hasEncodersInfo;

			in >> i; encoderLeftTicks=i;
			in >> i; encoderRightTicks=i;

		} break;

	case 2:
		{
			int32_t		i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			// If the estimation method is emOdometry, save the rawOdo + config data instead of
			//  the PDF itself:
			if ( estimationMethod == emOdometry )
			{
				// The odometry data:
				in >> rawOdometryIncrementReading;

//				TMotionModelOptions_OLD_VERSION_2	dummy;
//				in.ReadBuffer( &dummy, sizeof(TMotionModelOptions_OLD_VERSION_2) );
				uint8_t		dummy[44];
				in.ReadBuffer( &dummy, sizeof(dummy) );

				motionModelConfiguration = TMotionModelOptions();

				// Re-build the PDF:
				computeFromOdometry( rawOdometryIncrementReading,motionModelConfiguration );
			}
			else
			{
				// Read the PDF directly from the stream:
				CPosePDFPtr pc;
				in >> pc;
				poseChange = pc;
			}

			in >> hasVelocities;
			{
				float velocityLin, velocityAng;
				in >> velocityLin >> velocityAng;
				velocityLocal.vx = velocityLin;
				velocityLocal.vy = .0f;
				velocityLocal.omega = velocityAng;
			}
			in >> hasEncodersInfo;
			in >> i; encoderLeftTicks=i;
			in >> i; encoderRightTicks=i;

		} break;
	case 0:
	case 1:
		{
			int32_t	i;
			{
				CPosePDFPtr pc;
				in >> pc;
				poseChange = pc;
			}
			in >> i;

			// copy:
			estimationMethod = static_cast<TEstimationMethod>(i);

			// Simulate the "rawOdometryIncrementReading" as the mean value:
			if (estimationMethod==emOdometry)
					poseChange->getMean( rawOdometryIncrementReading );
			else	rawOdometryIncrementReading = CPose2D(0,0,0);

			if (version>=1)
			{
				in >> hasVelocities;
				{
					float velocityLin, velocityAng;
					in >> velocityLin >> velocityAng;
					velocityLocal.vx = velocityLin;
					velocityLocal.vy = .0f;
					velocityLocal.omega = velocityAng;
				}
				in >> hasEncodersInfo;
				in >> i; encoderLeftTicks = i;
				in >> i; encoderRightTicks = i;
			}
			else
			{
				// Default values:
				hasVelocities  = hasEncodersInfo = false;
				encoderLeftTicks= encoderRightTicks= 0;
				velocityLocal = mrpt::math::TTwist2D(.0, .0, .0);
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					computeFromEncoders
 ---------------------------------------------------------------*/
void  CActionRobotMovement2D::computeFromEncoders(
	double	K_left,
	double	K_right,
	double	D )
{
	if (hasEncodersInfo)
	{
		const double	As   = 0.5* ( K_right*encoderRightTicks + K_left*encoderLeftTicks );
		const double	Aphi = ( K_right*encoderRightTicks - K_left*encoderLeftTicks ) / D;

		double  x,y;
		if (Aphi!=0)
		{
			const double R = As/Aphi;
			x=R*sin(Aphi);
			y=R*(1-cos(Aphi));
		}
		else
		{
			x=As;
			y=0;
		}

		// Build the whole PDF with the current parameters:
		computeFromOdometry( CPose2D(x,y,Aphi), motionModelConfiguration );
	}
}

/*---------------------------------------------------------------
					computeFromOdometry
 ---------------------------------------------------------------*/
void  CActionRobotMovement2D::computeFromOdometry(
	const CPose2D				&odometryIncrement,
	const TMotionModelOptions	&options)
{
	// Set the members data:
	estimationMethod			= emOdometry;
	rawOdometryIncrementReading	= odometryIncrement;
	motionModelConfiguration	= options;

	if ( options.modelSelection == mmGaussian )
			computeFromOdometry_modelGaussian( odometryIncrement, options );
	else	computeFromOdometry_modelThrun( odometryIncrement, options );
}


/*---------------------------------------------------------------
						TMotionModelOptions
 ---------------------------------------------------------------*/
CActionRobotMovement2D::TMotionModelOptions::TMotionModelOptions() :
	modelSelection( CActionRobotMovement2D::mmGaussian ),
	gaussianModel(),
	thrunModel()
{
	gaussianModel.a1		    = 0.01f;
	gaussianModel.a2		    = RAD2DEG( 0.001f );
	gaussianModel.a3		    = DEG2RAD( 1.0f );
	gaussianModel.a4		    = 0.05f;

	gaussianModel.minStdXY	= 0.01f;
	gaussianModel.minStdPHI	= DEG2RAD(0.2f);

	thrunModel.nParticlesCount		= 300;
	thrunModel.alfa1_rot_rot		= 0.05f;
	thrunModel.alfa2_rot_trans	= DEG2RAD( 4.0f );
	thrunModel.alfa3_trans_trans	= 0.01f;
	thrunModel.alfa4_trans_rot	= RAD2DEG( 0.0001f );
	thrunModel.additional_std_XY	= 0.001f;
	thrunModel.additional_std_phi	= DEG2RAD(0.05f);
}

/*---------------------------------------------------------------
				computeFromOdometry_modelGaussian
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::computeFromOdometry_modelGaussian(
	const CPose2D				&odometryIncrement,
	const TMotionModelOptions	&o
	)
{
	// The Gaussian PDF:
	// ---------------------------
	poseChange = CPosePDFGaussian::Create();
	CPosePDFGaussian		*aux;
	aux = static_cast<CPosePDFGaussian*>( poseChange.pointer() );

	// See http://www.mrpt.org/Probabilistic_Motion_Models
	// -----------------------------------

	// Build the odometry noise matrix:
	double	Al = odometryIncrement.norm();
	CMatrixDouble31 ODO_INCR = CMatrixDouble31(odometryIncrement);
	CMatrixDouble33 C_ODO;
	C_ODO(0,0) =square( o.gaussianModel.minStdXY + o.gaussianModel.a1*Al + o.gaussianModel.a2*fabs( odometryIncrement.phi() ) );
	C_ODO(1,1) =square( o.gaussianModel.minStdXY + o.gaussianModel.a1*Al + o.gaussianModel.a2*fabs( odometryIncrement.phi() ) );
	C_ODO(2,2) =square( o.gaussianModel.minStdPHI + o.gaussianModel.a3*Al + o.gaussianModel.a4*fabs( odometryIncrement.phi() ) );

    // Build the transformation matrix:
    CMatrixDouble33 H;

    double cos_Aphi_2 = cos( odometryIncrement.phi()/2 );
    double sin_Aphi_2 = sin( odometryIncrement.phi()/2 );

    H(0,0) =  H(1,1) = cos_Aphi_2 ;
    H(0,1) = - (H(1,0) = sin_Aphi_2);
    H(2,2) = 1;

    // Build the Jacobian matrix:
    CMatrixDouble33  J = H;
    J(0,2) = -sin_Aphi_2 * ODO_INCR(0,0) - cos_Aphi_2 * ODO_INCR(1,0);
    J(1,2) =  cos_Aphi_2 * ODO_INCR(0,0) - sin_Aphi_2 * ODO_INCR(1,0);

    // The mean:
    aux->mean = odometryIncrement;

	// The covariance:
	J.multiply_HCHt( C_ODO, aux->cov );

}

/*---------------------------------------------------------------
				computeFromOdometry_modelThrun
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::computeFromOdometry_modelThrun(
	const CPose2D				&odometryIncrement,
	const TMotionModelOptions	&o
	)
{
	// The Gaussian PDF:
	// ---------------------------
	CPosePDFParticles		*aux;
	static CPose2D			nullPose(0,0,0);

	poseChange = CPosePDFParticles::Create();
	aux = static_cast<CPosePDFParticles*>( poseChange.pointer() );

	// Set the number of particles:
	aux->resetDeterministic(nullPose, o.thrunModel.nParticlesCount );

	// ---------------------------------------------------------------------------------------------
	// The following is an implementation from Thrun et al.'s book  (Probabilistic Robotics),
	//  page 136. Here "odometryIncrement" actually represents the incremental odometry difference.
	// ---------------------------------------------------------------------------------------------

	// The increments in odometry:
	float	Arot1	= ( odometryIncrement.y()!=0 || odometryIncrement.x()!=0) ?
						atan2( odometryIncrement.y(), odometryIncrement.x() ) : 0;
	float   Atrans	= odometryIncrement.norm();
	float	Arot2	= math::wrapToPi( odometryIncrement.phi() - Arot1 );

	// Draw samples:
	for (size_t i=0;i<o.thrunModel.nParticlesCount;i++)
	{
		float	Arot1_draw	= Arot1  - (o.thrunModel.alfa1_rot_rot*fabs(Arot1)+o.thrunModel.alfa2_rot_trans*Atrans) * randomGenerator.drawGaussian1D_normalized();
		float	Atrans_draw = Atrans - (o.thrunModel.alfa3_trans_trans*Atrans+o.thrunModel.alfa4_trans_rot*(fabs(Arot1)+fabs(Arot2))) * randomGenerator.drawGaussian1D_normalized();
		float	Arot2_draw	= Arot2  - (o.thrunModel.alfa1_rot_rot*fabs(Arot2)+o.thrunModel.alfa2_rot_trans*Atrans) * randomGenerator.drawGaussian1D_normalized();

		// Output:
		aux->m_particles[i].d->x( Atrans_draw * cos( Arot1_draw ) + motionModelConfiguration.thrunModel.additional_std_XY * randomGenerator.drawGaussian1D_normalized() );
		aux->m_particles[i].d->y( Atrans_draw * sin( Arot1_draw ) + motionModelConfiguration.thrunModel.additional_std_XY * randomGenerator.drawGaussian1D_normalized() );
		aux->m_particles[i].d->phi( Arot1_draw + Arot2_draw + motionModelConfiguration.thrunModel.additional_std_phi * randomGenerator.drawGaussian1D_normalized() );
		aux->m_particles[i].d->normalizePhi();
	}
}

/*---------------------------------------------------------------
				drawSingleSample
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::drawSingleSample( CPose2D &outSample ) const
{
	// Only in the case of "emOdometry" we have the rawOdometryMeasurement and
	//  the parameters to draw new samples:
	if (estimationMethod== emOdometry)
	{
		if ( motionModelConfiguration.modelSelection == mmGaussian )
				drawSingleSample_modelGaussian( outSample );
		else	drawSingleSample_modelThrun( outSample );
	}
	else
	{
		// If is not odometry, just employ the stored distribution:
		poseChange->drawSingleSample( outSample );
	}

}

/*---------------------------------------------------------------
				drawSingleSample_modelGaussian
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::drawSingleSample_modelGaussian(
	CPose2D &outSample ) const
{
	// In the Gaussian case it is more efficient just to
	// draw a sample from the already computed PDF:
	poseChange->drawSingleSample( outSample );
}

/*---------------------------------------------------------------
				drawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::drawSingleSample_modelThrun(
	CPose2D &outSample ) const
{
	// ---------------------------------------------------------------------------------------------
	// The following is an implementation from Thrun et al.'s book  (Probabilistic Robotics),
	//  page 136. Here "odometryIncrement" actually represents the incremental odometry difference.
	// ---------------------------------------------------------------------------------------------

	// The increments in odometry:
	float	Arot1	= ( rawOdometryIncrementReading.y()!=0 || rawOdometryIncrementReading.x()!=0) ?
						atan2( rawOdometryIncrementReading.y(), rawOdometryIncrementReading.x() ) : 0;
	float   Atrans	= rawOdometryIncrementReading.norm();
	float	Arot2	= math::wrapToPi( rawOdometryIncrementReading.phi() - Arot1 );

	float	Arot1_draw	= Arot1  - (motionModelConfiguration.thrunModel.alfa1_rot_rot*fabs(Arot1)+motionModelConfiguration.thrunModel.alfa2_rot_trans*Atrans) * randomGenerator.drawGaussian1D_normalized();
	float	Atrans_draw = Atrans - (motionModelConfiguration.thrunModel.alfa3_trans_trans*Atrans+motionModelConfiguration.thrunModel.alfa4_trans_rot*(fabs(Arot1)+fabs(Arot2))) * randomGenerator.drawGaussian1D_normalized();
	float	Arot2_draw	= Arot2  - (motionModelConfiguration.thrunModel.alfa1_rot_rot*fabs(Arot2)+motionModelConfiguration.thrunModel.alfa2_rot_trans*Atrans) * randomGenerator.drawGaussian1D_normalized();

	// Output:
	outSample.x(  Atrans_draw * cos( Arot1_draw ) + motionModelConfiguration.thrunModel.additional_std_XY * randomGenerator.drawGaussian1D_normalized() );
	outSample.y(  Atrans_draw * sin( Arot1_draw ) + motionModelConfiguration.thrunModel.additional_std_XY * randomGenerator.drawGaussian1D_normalized() );
	outSample.phi( Arot1_draw + Arot2_draw  + motionModelConfiguration.thrunModel.additional_std_phi * randomGenerator.drawGaussian1D_normalized() );
	outSample.normalizePhi();
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSamples
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::prepareFastDrawSingleSamples()const
{
	// Only in the case of "emOdometry" we have the rawOdometryMeasurement and
	//  the parameters to draw new samples:
	if (estimationMethod== emOdometry)
	{
		if ( motionModelConfiguration.modelSelection == mmGaussian )
				prepareFastDrawSingleSample_modelGaussian( );
		else	prepareFastDrawSingleSample_modelThrun( );
	}
}

/*---------------------------------------------------------------
				fastDrawSingleSample
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::fastDrawSingleSample( CPose2D &outSample )const
{
	// Only in the case of "emOdometry" we have the rawOdometryMeasurement and
	//  the parameters to draw new samples:
	if (estimationMethod== emOdometry)
	{
		if ( motionModelConfiguration.modelSelection == mmGaussian )
				fastDrawSingleSample_modelGaussian( outSample );
		else	fastDrawSingleSample_modelThrun( outSample );
	}
	else
	{
		// If is not odometry, just employ the stored distribution:
		poseChange->drawSingleSample( outSample );
	}
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelGaussian
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::prepareFastDrawSingleSample_modelGaussian() const
{
	MRPT_START

	ASSERT_(IS_CLASS(poseChange,CPosePDFGaussian));

	CMatrixDouble33		D;

	const CPosePDFGaussian*		gPdf = static_cast<const CPosePDFGaussian*> (poseChange.pointer());
	const CMatrixDouble33		&cov = gPdf->cov;

	m_fastDrawGauss_M = gPdf->mean;

	/** Computes the eigenvalues/eigenvector decomposition of this matrix,
	*    so that: M = Z · D · Z<sup>T</sup>, where columns in Z are the
	*	  eigenvectors and the diagonal matrix D contains the eigenvalues
	*    as diagonal elements, sorted in <i>ascending</i> order.
	*/
	cov.eigenVectors( m_fastDrawGauss_Z, D );

	// Scale eigenvectors with eigenvalues:
	D = D.array().sqrt().matrix();
	m_fastDrawGauss_Z = m_fastDrawGauss_Z * D;

	MRPT_END
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::prepareFastDrawSingleSample_modelThrun() const
{
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::fastDrawSingleSample_modelGaussian( CPose2D &outSample ) const
{
	CVectorFloat	rndVector(3,0);
	for (size_t i=0;i<3;i++)
	{
		float	rnd = randomGenerator.drawGaussian1D_normalized();
		for (size_t d=0;d<3;d++)
			rndVector[d]+= ( m_fastDrawGauss_Z.get_unsafe(d,i)*rnd );
	}

	outSample = CPose2D(
		m_fastDrawGauss_M.x() + rndVector[0],
		m_fastDrawGauss_M.y() + rndVector[1],
		m_fastDrawGauss_M.phi() + rndVector[2] );
}

/*---------------------------------------------------------------
				prepareFastDrawSingleSample_modelThrun
  ---------------------------------------------------------------*/
void  CActionRobotMovement2D::fastDrawSingleSample_modelThrun( CPose2D &outSample ) const
{
	drawSingleSample_modelThrun( outSample );
}
