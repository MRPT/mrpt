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

#include <mrpt/slam.h>  // Precompiled header

#include <mrpt/slam/CICP.h>
#include <mrpt/scanmatching.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/utils/CTicTac.h>

#include <mrpt/math/utils.h>
#include <mrpt/math/geometry.h>

using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;



/*---------------------------------------------------------------
The method for aligning a pair of 2D points map.
*   The meaning of some parameters are implementation dependant,
*    so look for derived classes for instructions.
*  The target is to find a PDF for the pose displacement between
*   maps, <b>thus the pose of m2 relative to m1</b>. This pose
*   is returned as a PDF rather than a single value.
*
* \param m1			[IN] The first map
* \param m2			[IN] The second map. The pose of this map respect to m1 is to be estimated.
* \param grossEst		[IN] An initial gross estimation for the displacement. If a given algorithm doesn't need it, set to <code>CPose2D(0,0,0)</code> for example.
* \param pdf			[IN/OUT] A pointer to a CPosePDF pointer, initially set to NULL for this method to create the object. For greater efficiency, this object can be left undeleted and passed again to the method. When <b>not used anymore remember to delete it</b> using <code>delete pdf;</code>
* \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
* \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
*
* \sa CPointsMapAlignmentAlgorithm
  ---------------------------------------------------------------*/
CPosePDFPtr CICP::AlignPDF(
    const CMetricMap		*m1,
    const CMetricMap		*mm2,
    const CPosePDFGaussian	&initialEstimationPDF,
    float					*runningTime,
    void					*info )
{
	MRPT_START

	CTicTac	tictac;
	TReturnInfo		outInfo;
	CPosePDFPtr		resultPDF;

	if (runningTime)  tictac.Tic();

	switch( options.ICP_algorithm )
	{
	case icpClassic:
		resultPDF = ICP_Method_Classic( m1, mm2, initialEstimationPDF, outInfo );
		break;
	case icpLevenbergMarquardt:
		resultPDF = ICP_Method_LM( m1, mm2, initialEstimationPDF, outInfo );
		break;
	case icpIKF:
		resultPDF = ICP_Method_IKF( m1, mm2, initialEstimationPDF, outInfo );
		break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("Invalid value for ICP_algorithm: %i", static_cast<int>(options.ICP_algorithm));
	} // end switch

	if (runningTime)  *runningTime = tictac.Tac();

	// Copy the output info if requested:
	if (info)
	{
		if ( static_cast<TReturnInfo*>(info)->cbSize != sizeof(TReturnInfo) )
			THROW_EXCEPTION("'TReturnInfo' size is wrong, filling it would lead to memory corruption")
		// It's ok:
		*static_cast<TReturnInfo*>(info) = outInfo;
	}

	return resultPDF;

	MRPT_END
}

/*---------------------------------------------------------------
					TConfigParams
  ---------------------------------------------------------------*/
CICP::TConfigParams::TConfigParams() :
	ICP_algorithm				( icpClassic ),

	onlyClosestCorrespondences	( true ),
	onlyUniqueRobust			( false ),
	maxIterations				( 40 ),
	minAbsStep_trans		( 1e-6f ),
	minAbsStep_rot			( 1e-6f ),
	thresholdDist				( 0.75f ),
	thresholdAng				( DEG2RAD(0.15f) ),
	ALFA						( 0.50f ),
	smallestThresholdDist		( 0.10f ),
	covariance_varPoints		( square(0.02f) ),
	doRANSAC					( false ),

	ransac_minSetSize			( 3 ),
	ransac_maxSetSize			( 20 ),
	ransac_nSimulations			( 100 ),
	ransac_mahalanobisDistanceThreshold ( 3.0f ),
	normalizationStd			( 0.02f ),
	ransac_fuseByCorrsMatch		( true ),
	ransac_fuseMaxDiffXY		( 0.01f ),
	ransac_fuseMaxDiffPhi		( DEG2RAD(0.1f) ),

	kernel_rho					( 0.07f ),
	use_kernel					( true ),
	Axy_aprox_derivatives		( 0.05f ),

	LM_initial_lambda			( 1e-4f ),

	skip_cov_calculation		(false),

	corresponding_points_decimation ( 5 )
{
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CICP::TConfigParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR( maxIterations, int,			iniFile, section);
	MRPT_LOAD_CONFIG_VAR( minAbsStep_trans, float,			iniFile, section);
	MRPT_LOAD_CONFIG_VAR( minAbsStep_rot, float,			iniFile, section);

	MRPT_LOAD_CONFIG_VAR_CAST( ICP_algorithm, int, TICPAlgorithm, iniFile, section);

	MRPT_LOAD_CONFIG_VAR( thresholdDist, float,			iniFile, section);
	thresholdAng = DEG2RAD( iniFile.read_float(section.c_str(),"thresholdAng_DEG",RAD2DEG(thresholdAng)) );

	MRPT_LOAD_CONFIG_VAR( ALFA, float,						iniFile, section);
	MRPT_LOAD_CONFIG_VAR( smallestThresholdDist, float,		iniFile, section);
	MRPT_LOAD_CONFIG_VAR( onlyClosestCorrespondences, bool,	iniFile, section);
	MRPT_LOAD_CONFIG_VAR( onlyUniqueRobust, bool, 			iniFile, section);
	MRPT_LOAD_CONFIG_VAR( doRANSAC, bool, 					iniFile, section);
	MRPT_LOAD_CONFIG_VAR( covariance_varPoints,float, 		iniFile, section);

	MRPT_LOAD_CONFIG_VAR( ransac_minSetSize, int, 			iniFile, section);
	MRPT_LOAD_CONFIG_VAR( ransac_maxSetSize, int,			iniFile, section);
	MRPT_LOAD_CONFIG_VAR( ransac_mahalanobisDistanceThreshold, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR( ransac_nSimulations, int, 		iniFile, section);
	MRPT_LOAD_CONFIG_VAR( normalizationStd, float, 			iniFile, section);
	MRPT_LOAD_CONFIG_VAR( ransac_fuseByCorrsMatch, bool, 	iniFile, section);
	MRPT_LOAD_CONFIG_VAR( ransac_fuseMaxDiffXY, float,  	iniFile, section);
	ransac_fuseMaxDiffPhi			= DEG2RAD( iniFile.read_float(section.c_str(),"ransac_fuseMaxDiffPhi_DEG",RAD2DEG(ransac_fuseMaxDiffPhi)) );

	MRPT_LOAD_CONFIG_VAR( kernel_rho, float,				iniFile, section);
	MRPT_LOAD_CONFIG_VAR( use_kernel, bool, 				iniFile, section);
	MRPT_LOAD_CONFIG_VAR( Axy_aprox_derivatives, float,		iniFile, section);
	MRPT_LOAD_CONFIG_VAR( LM_initial_lambda, float,			iniFile, section);

	MRPT_LOAD_CONFIG_VAR( skip_cov_calculation, bool, 				iniFile, section);

	MRPT_LOAD_CONFIG_VAR( corresponding_points_decimation, int, 				iniFile, section);

}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CICP::TConfigParams::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CICP::TConfigParams] ------------ \n\n");

	out.printf("ICP_algorithm                           = %s\n",
		ICP_algorithm==icpClassic ?  "icpClassic" :
		ICP_algorithm==icpLevenbergMarquardt ? "icpLevenbergMarquardt" :
		ICP_algorithm==icpIKF ? "icpIKF" : "(INVALID VALUE!)" );

	out.printf("maxIterations                           = %i\n",maxIterations);
	out.printf("minAbsStep_trans                        = %f\n",minAbsStep_trans);
	out.printf("minAbsStep_rot                          = %f\n",minAbsStep_rot);

	out.printf("thresholdDist                           = %f\n",thresholdDist);
	out.printf("thresholdAng                            = %f deg\n",RAD2DEG(thresholdAng));
	out.printf("ALFA                                    = %f\n",ALFA);
	out.printf("smallestThresholdDist                   = %f\n",smallestThresholdDist);
	out.printf("onlyClosestCorrespondences              = %c\n",onlyClosestCorrespondences ? 'Y':'N');
	out.printf("onlyUniqueRobust                        = %c\n",onlyUniqueRobust ? 'Y':'N');
	out.printf("covariance_varPoints                    = %f\n",covariance_varPoints);
	out.printf("doRANSAC                                = %c\n",doRANSAC ? 'Y':'N');
	out.printf("ransac_minSetSize                       = %i\n",ransac_minSetSize);
	out.printf("ransac_maxSetSize                       = %i\n",ransac_maxSetSize);
	out.printf("ransac_mahalanobisDistanceThreshold     = %f\n",ransac_mahalanobisDistanceThreshold);
	out.printf("ransac_nSimulations                     = %i\n",ransac_nSimulations);
	out.printf("ransac_fuseByCorrsMatch                 = %c\n",ransac_fuseByCorrsMatch ? 'Y':'N');
	out.printf("ransac_fuseMaxDiffXY                    = %f\n",ransac_fuseMaxDiffXY);
	out.printf("ransac_fuseMaxDiffPhi                   = %f deg\n",RAD2DEG( ransac_fuseMaxDiffPhi ));
	out.printf("normalizationStd                        = %f\n",normalizationStd);
	out.printf("kernel_rho                              = %f\n",kernel_rho);
	out.printf("use_kernel                              = %c\n",use_kernel  ? 'Y':'N');
	out.printf("Axy_aprox_derivatives                   = %f\n",Axy_aprox_derivatives );
	out.printf("LM_initial_lambda                       = %f\n",LM_initial_lambda);
	out.printf("skip_cov_calculation                    = %c\n",skip_cov_calculation ? 'Y':'N');
	out.printf("corresponding_points_decimation         = %u\n",(unsigned int)corresponding_points_decimation);
	out.printf("\n");
}

/*---------------------------------------------------------------
					kernel
  ---------------------------------------------------------------*/
float CICP::kernel(const float &x2, const float &rho2)
{
	return options.use_kernel ?  ( x2 / (x2+rho2) ) : x2;
}

/*----------------------------------------------------------------------------

					ICP_Method_Classic

  ----------------------------------------------------------------------------*/
CPosePDFPtr CICP::ICP_Method_Classic(
		const CMetricMap		*m1,
		const CMetricMap		*mm2,
		const CPosePDFGaussian	&initialEstimationPDF,
		TReturnInfo				&outInfo )
{
	MRPT_START

	// The result can be either a Gaussian or a SOG:
	CPosePDFPtr								resultPDF;
	CPosePDFGaussianPtr						gaussPdf;
	CPosePDFSOGPtr							SOG;

	size_t  nCorrespondences=0;
	float   umbral_dist,umbral_ang;
	bool    keepApproaching;
	CPose2D	grossEst = initialEstimationPDF.mean;
	mrpt::utils::TMatchingPairList correspondences,old_correspondences;
	float   correspondencesRatio;
	CPose2D lastMeanPose;

	const bool onlyUniqueRobust = options.onlyUniqueRobust;
	const bool onlyKeepTheClosest = options.onlyClosestCorrespondences;

	// Assure the class of the maps:
	const CMetricMap		*m2 = mm2;

	// Asserts:
	// -----------------
	ASSERT_( options.ALFA>=0 && options.ALFA<1 );

	// The algorithm output auxiliar info:
	// -------------------------------------------------
	outInfo.cbSize			= sizeof(TReturnInfo);
	outInfo.nIterations		= 0;
	outInfo.goodness		= 1;
	outInfo.quality			= 0;

	// The gaussian PDF to estimate:
	// ------------------------------------------------------
	gaussPdf = CPosePDFGaussian::Create();

	// First gross approximation:
	gaussPdf->mean = grossEst;

	// Initial thresholds:
	umbral_dist		= options.thresholdDist;
	umbral_ang		= options.thresholdAng;


	// Asure maps are not empty!
	// ------------------------------------------------------
	if ( !m2->isEmpty() )
	{
		size_t offset_other_map_points = 0;

		// ------------------------------------------------------
		//					The ICP loop
		// ------------------------------------------------------
		do
		{
			// ------------------------------------------------------
			//		Find the matching (for a points map)
			// ------------------------------------------------------
			m1->computeMatchingWith2D(
				m2,						// The other map
				gaussPdf->mean,			// The other map pose
				umbral_dist,			// Distance threshold
				umbral_ang,				// Angular threshold
				gaussPdf->mean,			// Pivot point for angular measurements
				correspondences,		// Output
				correspondencesRatio,	// Ratio
				NULL,					// MSE
				onlyKeepTheClosest,
				onlyUniqueRobust,
				options.corresponding_points_decimation,
				offset_other_map_points );

			nCorrespondences = correspondences.size();


			// ***DEBUG***
//				correspondences.dumpToFile("debug_correspondences.txt");

			if ( !nCorrespondences )
			{
				// Nothing we can do !!
				keepApproaching = false;
			}
			else
			{
				// Compute the estimated pose.
				//  (Method from paper of J.Gonzalez, Martinez y Morales)
				// ----------------------------------------------------------------------
				scanmatching::leastSquareErrorRigidTransformation( correspondences,	gaussPdf->mean );

				// If matching has not changed, decrease the thresholds:
				// --------------------------------------------------------
				keepApproaching = true;
				if	(!(fabs(lastMeanPose.x()-gaussPdf->mean.x())>options.minAbsStep_trans ||
					fabs(lastMeanPose.y()-gaussPdf->mean.y())>options.minAbsStep_trans ||
					fabs(math::wrapToPi(lastMeanPose.phi()-gaussPdf->mean.phi()))>options.minAbsStep_rot))
				{
					umbral_dist		*= options.ALFA;
					umbral_ang		*= options.ALFA;
					if (umbral_dist < options.smallestThresholdDist )
						keepApproaching = false;

					if (++offset_other_map_points>=options.corresponding_points_decimation)
						offset_other_map_points=0;

				}

				lastMeanPose = gaussPdf->mean;

			}	// end of "else, there are correspondences"

			// Next iteration:
			outInfo.nIterations++;

			if (outInfo.nIterations >= options.maxIterations && umbral_dist>options.smallestThresholdDist)
			{
				umbral_dist		*= options.ALFA;
			}

		} while	( (keepApproaching && outInfo.nIterations<options.maxIterations) ||
					(outInfo.nIterations >= options.maxIterations && umbral_dist>options.smallestThresholdDist) );


		// -------------------------------------------------
		//   Obtain the covariance matrix of the estimation
		// -------------------------------------------------
		if (!options.skip_cov_calculation && nCorrespondences)
		{
#if 0
			// ----------------------------------------------
			// METHOD 1: MSE linear estimation
			// ----------------------------------------------
			scanmatching::leastSquareErrorRigidTransformation(
				correspondences,
				gaussPdf->mean,
				&gaussPdf->cov );
			// Scale covariance:
			gaussPdf->cov *= options.covariance_varPoints;
#else
			// ----------------------------------------------
			// METHOD 2: Method from Oxford MRG's "OXSMV2"
			//
			//  It is the equivalent covariance resulting
			//   from a Levenberg-Maquardt optimization stage.
			// ----------------------------------------------
			Eigen::Matrix<double,3,Eigen::Dynamic> D(3,nCorrespondences);

			const TPose2D transf( gaussPdf->mean );

			double  ccos = cos(transf.phi);
			double  csin = sin(transf.phi);

			double  w1,w2,w3;
			double  q1,q2,q3;
			double  A,B;
			double  Axy = 0.01;

			// Fill out D:
			double  rho2 = square( options.kernel_rho );
			mrpt::utils::TMatchingPairList::iterator	it;
			size_t  i;
			for (i=0, it=correspondences.begin();i<nCorrespondences;i++, it++)
			{
				float   other_x_trans = transf.x + ccos * it->other_x - csin * it->other_y;
				float   other_y_trans = transf.y + csin * it->other_x + ccos * it->other_y;

				// Jacobian: dR2_dx
				// --------------------------------------
				w1= other_x_trans-Axy;
				q1= kernel( square(it->this_x - w1)+ square( it->this_y - other_y_trans ),  rho2 );

				w2= other_x_trans;
				q2= kernel( square(it->this_x - w2)+ square( it->this_y - other_y_trans ),  rho2 );

				w3= other_x_trans+Axy;
				q3= kernel( square(it->this_x - w3)+ square( it->this_y - other_y_trans ),  rho2 );

				//interpolate
				A=(  (q3-q2)/((w3-w2)*(w3-w1))  ) - (  (q1-q2)/((w1-w2)*(w3-w1))  );
				B=(   (q1-q2)+(A*((w2*w2)-(w1*w1)))   )/(w1-w2);

				D(0,i) = (2*A*other_x_trans)+B;

				// Jacobian: dR2_dy
				// --------------------------------------
				w1= other_y_trans-Axy;
				q1= kernel( square(it->this_x - other_x_trans)+ square( it->this_y - w1 ),  rho2 );

				w2= other_y_trans;
				q2= kernel( square(it->this_x - other_x_trans)+ square( it->this_y - w2 ),  rho2 );

				w3= other_y_trans+Axy;
				q3= kernel( square(it->this_x - other_x_trans)+ square( it->this_y - w3 ),  rho2 );

				//interpolate
				A=(  (q3-q2)/((w3-w2)*(w3-w1))  ) - (  (q1-q2)/((w1-w2)*(w3-w1))  );
				B=(   (q1-q2)+(A*((w2*w2)-(w1*w1)))   )/(w1-w2);

				D(1,i) = (2*A*other_y_trans)+B;

				// Jacobian: dR_dphi
				// --------------------------------------
				D(2,i) = D(0,i) * ( -csin * it->other_x - ccos * it->other_y )  +
					     D(1,i) * (  ccos * it->other_x - csin * it->other_y );

			} // end for each corresp.

			// COV = ( D*D^T + lamba*I )^-1
			CMatrixDouble33  DDt = D*D.transpose();

			for (i=0;i<3;i++)
				DDt( i,i ) += 6000.0;  // Lambda...

			DDt.inv(gaussPdf->cov);
#endif
		}

		//
		outInfo.goodness = correspondencesRatio;

		// Compute a crude estimate of the quality of scan matching at this local minimum:
		// -----------------------------------------------------------------------------------
		float  	Axy = umbral_dist*0.9f;

		float	E0,EX1,EX2,EY1,EY2;		// sqr. errors at each point:

		TPose2D  	P0( gaussPdf->mean );
		TPose2D  	PX1(P0);	PX1.x -= Axy;
		TPose2D  	PX2(P0);	PX2.x += Axy;
		TPose2D  	PY1(P0);	PY1.y -= Axy;
		TPose2D  	PY2(P0);	PY2.y += Axy;

		float		dist_thresh = umbral_dist;

		m1->computeMatchingWith2D(
				m2,						// The other map
				P0,			// The other map pose
				dist_thresh,			// Distance threshold
				umbral_ang,				// Angular threshold
				gaussPdf->mean,			// Pivot point for angular measurements
				correspondences,		// Output
				E0, //correspondencesRatio,	// Ratio
				NULL, //&E0,					// MSE
				onlyKeepTheClosest,
				onlyUniqueRobust,
				options.corresponding_points_decimation );

		m1->computeMatchingWith2D(
				m2,						// The other map
				PX1,			// The other map pose
				dist_thresh,			// Distance threshold
				umbral_ang,				// Angular threshold
				gaussPdf->mean,			// Pivot point for angular measurements
				correspondences,		// Output
				EX1,	// Ratio
				NULL, //&EX1,					// MSE
				true, 					// onlyKeepTheClosest
				false,				//onlyUniqueRobust
				options.corresponding_points_decimation );

		m1->computeMatchingWith2D(
				m2,						// The other map
				PX2,			// The other map pose
				dist_thresh,			// Distance threshold
				umbral_ang,				// Angular threshold
				gaussPdf->mean,			// Pivot point for angular measurements
				correspondences,		// Output
				EX2,	// Ratio
				NULL, //&EX2,					// MSE
				true, 					// onlyKeepTheClosest
				false,				//onlyUniqueRobust
				options.corresponding_points_decimation );

		m1->computeMatchingWith2D(
				m2,						// The other map
				PY1,			// The other map pose
				dist_thresh,			// Distance threshold
				umbral_ang,				// Angular threshold
				gaussPdf->mean,			// Pivot point for angular measurements
				correspondences,		// Output
				EY1,	// Ratio
				NULL, //&EY1,					// MSE
				true, 					// onlyKeepTheClosest
				false,				//onlyUniqueRobust
				options.corresponding_points_decimation );
		m1->computeMatchingWith2D(
				m2,						// The other map
				PY2,			// The other map pose
				dist_thresh,			// Distance threshold
				umbral_ang,				// Angular threshold
				gaussPdf->mean,			// Pivot point for angular measurements
				correspondences,		// Output
				EY2,	// Ratio
				NULL, //&EY2,					// MSE
				true, 					// onlyKeepTheClosest
				false,				//onlyUniqueRobust
				options.corresponding_points_decimation );

		outInfo.quality= -max(EX1-E0, max(EX2-E0, max(EY1-E0 , EY2-E0 ) ) )/(E0+1e-1);

	} // end of "if m2 is not empty"

	// We'll return a CPosePDFGaussian or a CPosePDFSOG if RANSAC is enabled:
	// -----------------------------------------------------------------------

	// RANSAC?
	if (options.doRANSAC)
	{
		// Discard the gaussian:
		//delete gaussPdf; gaussPdf=NULL;

		SOG = CPosePDFSOG::Create();
		scanmatching::robustRigidTransformation(
			correspondences,
			*SOG,
			options.normalizationStd,
			options.ransac_minSetSize,
			options.ransac_maxSetSize,
			options.ransac_mahalanobisDistanceThreshold,
			options.ransac_nSimulations,
			NULL,
			options.ransac_fuseByCorrsMatch,
			options.ransac_fuseMaxDiffXY,
			options.ransac_fuseMaxDiffPhi,
			false // ransac_useMahalanobisAsTest
			) ;

		// And return the SOG:
		resultPDF = SOG;
	}
	else
	{
		// Return the gaussian distribution:
		resultPDF = gaussPdf;
	}


	return resultPDF;

	MRPT_END
}




/*----------------------------------------------------------------------------

						ICP_Method_LM

  ----------------------------------------------------------------------------*/
CPosePDFPtr CICP::ICP_Method_LM(
		const CMetricMap		*mm1,
		const CMetricMap		*m2,
		const CPosePDFGaussian	&initialEstimationPDF,
		TReturnInfo				&outInfo )
{
	MRPT_START

	// The result can be either a Gaussian or a SOG:
	size_t									nCorrespondences=0;
	double									umbral_dist,umbral_ang;
	bool									keepIteratingICP;
	CPose2D									grossEst = initialEstimationPDF.mean;
	mrpt::utils::TMatchingPairList			correspondences,old_correspondences;
	float									correspondencesRatio;
	CPose2D									lastMeanPose;
	vector_float							other_xs_trans,other_ys_trans; // temporary container of "other" map (map2) transformed by "q"
	CMatrixFloat  							dJ_dq;  // The jacobian
	CPose2D   								q;	// The updated 2D transformation estimate
	CPose2D									q_new;

	const bool onlyUniqueRobust = options.onlyUniqueRobust;
	const bool onlyKeepTheClosest = options.onlyClosestCorrespondences;

	// Assure the class of the maps:
	ASSERT_(mm1->GetRuntimeClass()->derivedFrom(CLASS_ID(CPointsMap)));
	const CPointsMap	*m1 = static_cast<const CPointsMap*>(mm1);

	// Asserts:
	// -----------------
	ASSERT_( options.ALFA>0 && options.ALFA<1 );

	// The algorithm output auxiliar info:
	// -------------------------------------------------
	outInfo.cbSize			= sizeof(TReturnInfo);
	outInfo.nIterations		= 0;
	outInfo.goodness		= 1.0f;

	// The gaussian PDF to estimate:
	// ------------------------------------------------------
	// First gross approximation:
	q = grossEst;

	// Initial thresholds:
	umbral_dist		= options.thresholdDist;
	umbral_ang		= options.thresholdAng;

	// For LM inverse
	CMatrixFixedNumeric<float,3,3>	C;
	CMatrixFixedNumeric<float,3,3>	C_inv;		// This will keep the cov. matrix at the end

	// Asure maps are not empty!
	// ------------------------------------------------------
	if ( !m2->isEmpty() )
	{
		size_t offset_other_map_points = 0;
		// ------------------------------------------------------
		//					The ICP loop
		// ------------------------------------------------------
		do
		{
			// ------------------------------------------------------
			//		Find the matching (for a points map)
			// ------------------------------------------------------
			m1->computeMatchingWith2D(
				m2,						// The other map
				q,						// The other map pose
				umbral_dist,			// Distance threshold
				umbral_ang,				// Angular threshold
				q,						// Pivot point for angular measurements
				correspondences,		// Output
				correspondencesRatio,	// Ratio
				NULL,					// MSE
				onlyKeepTheClosest,
				onlyUniqueRobust,
				options.corresponding_points_decimation,
				offset_other_map_points );


			nCorrespondences = correspondences.size();


			if ( !nCorrespondences )
			{
				// Nothing we can do !!
				keepIteratingICP = false;
			}
			else
			{
				// Compute the estimated pose through iterative least-squares: Levenberg-Marquardt
				// ----------------------------------------------------------------------
				dJ_dq.setSize(3,nCorrespondences);  // The jacobian of the error function wrt the transformation q

				double  lambda = options.LM_initial_lambda;		// The LM parameter

				double  ccos = cos(q.phi());
				double	csin = sin(q.phi());

				double  w1,w2,w3;
				double  q1,q2,q3;
				double  A,B;
				const double  Axy = options.Axy_aprox_derivatives;		// For approximating the derivatives

				// Compute at once the square errors for each point with the current "q" and the transformed points:
				vector_float 	sq_errors;
				correspondences.squareErrorVector( q, sq_errors, other_xs_trans,other_ys_trans);
				double OSE_initial = math::sum( sq_errors );

				// Compute "dJ_dq"
				// ------------------------------------
				double  rho2 = square( options.kernel_rho );
				mrpt::utils::TMatchingPairList::iterator	it;
				vector_float::const_iterator other_x_trans,other_y_trans;
				size_t  i;

				for (i=0,
					it=correspondences.begin(),
					other_x_trans = other_xs_trans.begin(),
					other_y_trans = other_ys_trans.begin();
					i<nCorrespondences;
					i++, it++,other_x_trans++,other_y_trans++ )
				{
					// Jacobian: dJ_dx
					// --------------------------------------
//#define ICP_DISTANCES_TO_LINE

#ifndef ICP_DISTANCES_TO_LINE
					w1= *other_x_trans-Axy;
					q1 = m1->squareDistanceToClosestCorrespondence( w1, *other_y_trans );
					q1= kernel( q1, rho2 );

					w2= *other_x_trans;
					q2 = m1->squareDistanceToClosestCorrespondence( w2, *other_y_trans );
					q2= kernel( q2, rho2 );

					w3= *other_x_trans+Axy;
					q3 = m1->squareDistanceToClosestCorrespondence( w3, *other_y_trans );
					q3= kernel( q3, rho2 );
#else
					// The distance to the line that interpolates the TWO closest points:
					float  x1,y1, x2,y2, d1,d2;
					m1->kdTreeTwoClosestPoint2D(
						*other_x_trans, *other_y_trans, 	// The query
						x1, y1,   // Closest point #1
						x2, y2,   // Closest point #2
						d1,d2);

					w1= *other_x_trans-Axy;
					q1 = math::closestSquareDistanceFromPointToLine( w1, *other_y_trans,  x1,y1, x2,y2 );
					q1= kernel( q1, rho2 );

					w2= *other_x_trans;
					q2 = math::closestSquareDistanceFromPointToLine( w2, *other_y_trans,  x1,y1, x2,y2 );
					q2= kernel( q2, rho2 );

					w3= *other_x_trans+Axy;
					q3 = math::closestSquareDistanceFromPointToLine( w3, *other_y_trans,  x1,y1, x2,y2 );
					q3= kernel( q3, rho2 );
#endif
					//interpolate
					A=(  (q3-q2)/((w3-w2)*(w3-w1))  ) - (  (q1-q2)/((w1-w2)*(w3-w1))  );
					B=(   (q1-q2)+(A*((w2*w2)-(w1*w1)))   )/(w1-w2);

					dJ_dq.get_unsafe(0,i) = (2*A* *other_x_trans)+B;

					// Jacobian: dJ_dy
					// --------------------------------------
					w1= *other_y_trans-Axy;
#ifdef ICP_DISTANCES_TO_LINE
					q1 = math::closestSquareDistanceFromPointToLine( *other_x_trans, w1,  x1,y1, x2,y2 );
					q1= kernel( q1, rho2 );
#else
					q1= kernel( square(it->this_x - *other_x_trans)+ square( it->this_y - w1 ),  rho2 );
#endif

					w2= *other_y_trans;
					// q2 is alreay computed from above!
					//q2 = m1->squareDistanceToClosestCorrespondence( *other_x_trans, w2 );
					//q2= kernel( square(it->this_x - *other_x_trans)+ square( it->this_y - w2 ),  rho2 );

					w3= *other_y_trans+Axy;
#ifdef ICP_DISTANCES_TO_LINE
					q3 = math::closestSquareDistanceFromPointToLine( *other_x_trans, w3,  x1,y1, x2,y2 );
					q3= kernel( q3, rho2 );
#else
					q3= kernel( square(it->this_x - *other_x_trans)+ square( it->this_y - w3 ),  rho2 );
#endif

					//interpolate
					A=(  (q3-q2)/((w3-w2)*(w3-w1))  ) - (  (q1-q2)/((w1-w2)*(w3-w1))  );
					B=(   (q1-q2)+(A*((w2*w2)-(w1*w1)))   )/(w1-w2);

					dJ_dq.get_unsafe(1,i) = (2*A* *other_y_trans)+B;

					// Jacobian: dR_dphi
					// --------------------------------------
					dJ_dq.get_unsafe(2,i) = dJ_dq.get_unsafe(0,i) * ( -csin * it->other_x - ccos * it->other_y )  +
								 dJ_dq.get_unsafe(1,i) * (  ccos * it->other_x - csin * it->other_y );

				} // end for each corresp.

				// Now we have the Jacobian in dJ_dq.

				// Compute the Hessian matrix H = dJ_dq * dJ_dq^T
				CMatrixFloat  H_(3,3);
				H_.multiply_AAt(dJ_dq);

				CMatrixFixedNumeric<float,3,3>  H = CMatrixFixedNumeric<float,3,3>(H_);

				bool  keepIteratingLM = true;

				// ---------------------------------------------------
				// Iterate the inner LM loop until convergence:
				// ---------------------------------------------------
				q_new = q;

				vector_float new_sq_errors, new_other_xs_trans, new_other_ys_trans;
				size_t   		nLMiters = 0;
				const size_t 	maxLMiters = 100;

				while ( keepIteratingLM &&  ++nLMiters<maxLMiters)
				{
					// The LM heuristic is:
					//  x_{k+1} = x_k  - ( H + \lambda diag(H) )^-1 * grad(J)
					//  grad(J) = dJ_dq * e (vector of errors)
					C = H;
					for (i=0;i<3;i++)
						C(i,i) *= (1+lambda);	// Levenberg-Maquardt heuristic
					//	C(i,i) += lambda;		// Levenberg  heuristic

					C_inv = C.inv();

					// LM_delta = C_inv * dJ_dq * sq_errors
					vector_float dJsq, LM_delta;
					dJ_dq.multiply_Ab( sq_errors, dJsq );
					C_inv.multiply_Ab(dJsq,LM_delta);

					q_new.x( q.x() - LM_delta[0] );
					q_new.y( q.y() - LM_delta[1] );
					q_new.phi( q.phi() - LM_delta[2] );

					// Compute the new square errors:
					// ---------------------------------------
					correspondences.squareErrorVector(
						q_new,
						new_sq_errors,
						new_other_xs_trans,
						new_other_ys_trans);

					float OSE_new = math::sum( new_sq_errors );

					bool improved = OSE_new < OSE_initial;

#if 0  // Debuggin'
					cout << "_____________" << endl;
					cout << "q -> q_new   : " << q << " -> " << q_new << endl;
					printf("err: %f  -> %f    lambda: %e\n", OSE_initial ,OSE_new, lambda );
					cout << "\\/J = "; utils::operator <<(cout,dJsq); cout << endl;
					mrpt::system::pause();
#endif

					keepIteratingLM =
						fabs(LM_delta[0])>options.minAbsStep_trans ||
						fabs(LM_delta[1])>options.minAbsStep_trans ||
						fabs(LM_delta[2])>options.minAbsStep_rot;

					if(improved)
					{
						//If resids have gone down, keep change and make lambda smaller by factor of 10
						lambda/=10;
						q=q_new;
						OSE_initial = OSE_new;
					}
					else
					{
						// Discard movement and try with larger lambda:
						lambda*=10;
					}

				} // end iterative LM

#if 0  // Debuggin'
				cout << "ICP loop: " << lastMeanPose  << " -> " << q << " LM iters: " << nLMiters  << " threshold: " << umbral_dist << endl;
#endif
				// --------------------------------------------------------
				// now the conditions for the outer ICP loop
				// --------------------------------------------------------
				keepIteratingICP = true;
				if	(fabs(lastMeanPose.x()-q.x())<options.minAbsStep_trans &&
					fabs(lastMeanPose.y()-q.y())<options.minAbsStep_trans &&
					fabs( math::wrapToPi( lastMeanPose.phi()-q.phi()) )<options.minAbsStep_rot)
				{
					umbral_dist		*= options.ALFA;
					umbral_ang		*= options.ALFA;
					if (umbral_dist < options.smallestThresholdDist )
						keepIteratingICP = false;

					if (++offset_other_map_points>=options.corresponding_points_decimation)
						offset_other_map_points=0;
				}
				lastMeanPose = q;
			}	// end of "else, there are correspondences"

			// Next iteration:
			outInfo.nIterations++;

			if (outInfo.nIterations >= options.maxIterations && umbral_dist>options.smallestThresholdDist)
			{
				umbral_dist		*= options.ALFA;
			}

		} while	( (keepIteratingICP && outInfo.nIterations<options.maxIterations) ||
					(outInfo.nIterations >= options.maxIterations && umbral_dist>options.smallestThresholdDist) );

		outInfo.goodness = correspondencesRatio;



		{
			// Compute a crude estimate of the quality of scan matching at this local minimum:
			// -----------------------------------------------------------------------------------
			float  	Axy = 0.03;

			float	E0,EX1,EX2,EY1,EY2;		// sqr. errors at each point:

			TPose2D  	P0( q );
			TPose2D  	PX1(P0);	PX1.x -= Axy;
			TPose2D  	PX2(P0);	PX2.x += Axy;
			TPose2D  	PY1(P0);	PY1.y -= Axy;
			TPose2D  	PY2(P0);	PY2.y += Axy;

			m1->computeMatchingWith2D(
					m2,						// The other map
					P0,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					q,			// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					&E0,					// MSE
					onlyKeepTheClosest,
					onlyUniqueRobust,
					options.corresponding_points_decimation );

			m1->computeMatchingWith2D(
					m2,						// The other map
					PX1,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					q,			// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					&EX1,					// MSE
					true, 					// onlyKeepTheClosest
					false, 				//onlyUniqueRobust
					options.corresponding_points_decimation );
			m1->computeMatchingWith2D(
					m2,						// The other map
					PX2,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					q,			// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					&EX2,					// MSE
					true, 					// onlyKeepTheClosest
					false, 				//onlyUniqueRobust
					options.corresponding_points_decimation );

			m1->computeMatchingWith2D(
					m2,						// The other map
					PY1,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					q,			// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					&EY1,					// MSE
					true, 					// onlyKeepTheClosest
					false, 				//onlyUniqueRobust
					options.corresponding_points_decimation );
			m1->computeMatchingWith2D(
					m2,						// The other map
					PY2,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					q,			// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					&EY2,					// MSE
					true, 					// onlyKeepTheClosest
					false, 				//onlyUniqueRobust
					options.corresponding_points_decimation );

			outInfo.quality= min(EX1-E0, min(EX2-E0, min(EY1-E0 , EY2-E0 ) ) )/(E0+1e-10);
		}


	} // end of "if m2 is not empty"

	return CPosePDFGaussianPtr( new CPosePDFGaussian(q, C_inv.cast<double>() ) );
	MRPT_END
}

/*---------------------------------------------------------------
					ICP_Method_IKF
  ---------------------------------------------------------------*/
CPosePDFPtr CICP::ICP_Method_IKF(
		const CMetricMap		*m1,
		const CMetricMap		*mm2,
		const CPosePDFGaussian	&initialEstimationPDF,
		TReturnInfo				&outInfo )
{
	MRPT_START
	THROW_EXCEPTION("Not implemented yet");
	return CPosePDFGaussian::Create();

	MRPT_END
}


/*---------------------------------------------------------------
The method for aligning a pair of 2D points map.
*   The meaning of some parameters are implementation dependant,
*    so look for derived classes for instructions.
*  The target is to find a PDF for the pose displacement between
*   maps, <b>thus the pose of m2 relative to m1</b>. This pose
*   is returned as a PDF rather than a single value.
*
* \param m1			[IN] The first map
* \param m2			[IN] The second map. The pose of this map respect to m1 is to be estimated.
* \param grossEst		[IN] An initial gross estimation for the displacement. If a given algorithm doesn't need it, set to <code>CPose2D(0,0,0)</code> for example.
* \param pdf			[IN/OUT] A pointer to a CPosePDF pointer, initially set to NULL for this method to create the object. For greater efficiency, this object can be left undeleted and passed again to the method. When <b>not used anymore remember to delete it</b> using <code>delete pdf;</code>
* \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
* \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
*
* \sa CPointsMapAlignmentAlgorithm
  ---------------------------------------------------------------*/
CPose3DPDFPtr CICP::Align3DPDF(
    const CMetricMap		*m1,
    const CMetricMap		*mm2,
    const CPose3DPDFGaussian	&initialEstimationPDF,
    float					*runningTime,
    void					*info )
{
	MRPT_START

	static CTicTac	tictac;
	TReturnInfo		outInfo;
	CPose3DPDFPtr		resultPDF;

	if (runningTime)  tictac.Tic();

	switch( options.ICP_algorithm )
	{
	case icpClassic:
		resultPDF = ICP3D_Method_Classic( m1, mm2, initialEstimationPDF, outInfo );
		break;
	case icpLevenbergMarquardt:
		THROW_EXCEPTION("Only icpClassic is implemented for ICP-3D")
		break;
	case icpIKF:
		THROW_EXCEPTION("Only icpClassic is implemented for ICP-3D")
		break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("Invalid value for ICP_algorithm: %i", static_cast<int>(options.ICP_algorithm));
	} // end switch

	if (runningTime)  *runningTime = tictac.Tac();

	// Copy the output info if requested:
	if (info)
	{
		if ( static_cast<TReturnInfo*>(info)->cbSize != sizeof(TReturnInfo) )
			THROW_EXCEPTION("'TReturnInfo' size is wrong, filling it would lead to memory corruption")
		// It's ok:
		*static_cast<TReturnInfo*>(info) = outInfo;
	}

	return resultPDF;

	MRPT_END
}



CPose3DPDFPtr CICP::ICP3D_Method_Classic(
		const CMetricMap		*m1,
		const CMetricMap		*mm2,
		const CPose3DPDFGaussian &initialEstimationPDF,
		TReturnInfo				&outInfo )
{
	MRPT_START

	// The result can be either a Gaussian or a SOG:
	CPose3DPDFPtr								resultPDF;
	CPose3DPDFGaussianPtr						gaussPdf;

	size_t									nCorrespondences=0;
	float									umbral_dist,umbral_ang;
	bool									keepApproaching;
	CPose3D									grossEst = initialEstimationPDF.mean;
	mrpt::utils::TMatchingPairList			correspondences,old_correspondences;
	float									correspondencesRatio;
	CPose3D									lastMeanPose;

	bool									onlyUniqueRobust = options.onlyUniqueRobust;
	bool									onlyKeepTheClosest = options.onlyClosestCorrespondences;

	// Assure the class of the maps:
	ASSERT_(mm2->GetRuntimeClass()->derivedFrom(CLASS_ID(CPointsMap)));
	const CPointsMap		*m2 = (CPointsMap*)mm2;

	// Asserts:
	// -----------------
	ASSERT_( options.ALFA>0 && options.ALFA<1 );

	// The algorithm output auxiliar info:
	// -------------------------------------------------
	outInfo.cbSize			= sizeof(TReturnInfo);
	outInfo.nIterations		= 0;
	outInfo.goodness		= 1;
	outInfo.quality			= 0;

	// The gaussian PDF to estimate:
	// ------------------------------------------------------
	gaussPdf = CPose3DPDFGaussian::Create();

	// First gross approximation:
	gaussPdf->mean = grossEst;

	// Initial thresholds:
	umbral_dist		= options.thresholdDist;
	umbral_ang		= options.thresholdAng;

	// Asure maps are not empty!
	// ------------------------------------------------------
	if ( !m2->isEmpty() )
	{
		size_t offset_other_map_points = 0;

		// ------------------------------------------------------
		//					The ICP loop
		// ------------------------------------------------------
		do
		{
			CPoint3D  pivotPoint = CPoint3D( gaussPdf->mean );

			// ------------------------------------------------------
			//		Find the matching (for a points map)
			// ------------------------------------------------------
			m1->computeMatchingWith3D(
					m2,						// The other map
					gaussPdf->mean,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					pivotPoint,				// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					NULL,					// MSE
					onlyKeepTheClosest,
					onlyUniqueRobust,
					options.corresponding_points_decimation,
					offset_other_map_points );

			nCorrespondences = correspondences.size();

			if ( !nCorrespondences )
			{
				// Nothing we can do !!
				keepApproaching = false;
			}
			else
			{
				// Compute the estimated pose, using Horn's method.
				// ----------------------------------------------------------------------
				double transf_scale;
				scanmatching::leastSquareErrorRigidTransformation6D( correspondences, gaussPdf->mean, transf_scale, false );

				//cout << gaussPdf->mean << " scale: " << transf_scale << " corrs: " << correspondences.size() << endl;

				// If matching has not changed, decrease the thresholds:
				// --------------------------------------------------------
				keepApproaching = true;
				if	(!(fabs(lastMeanPose.x()-gaussPdf->mean.x())>options.minAbsStep_trans ||
					fabs(lastMeanPose.y()-gaussPdf->mean.y())>options.minAbsStep_trans ||
					fabs(lastMeanPose.z()-gaussPdf->mean.z())>options.minAbsStep_trans ||
					fabs(math::wrapToPi(lastMeanPose.yaw()-gaussPdf->mean.yaw()))>options.minAbsStep_rot ||
					fabs(math::wrapToPi(lastMeanPose.pitch()-gaussPdf->mean.pitch()))>options.minAbsStep_rot ||
					fabs(math::wrapToPi(lastMeanPose.roll()-gaussPdf->mean.roll()))>options.minAbsStep_rot ))
				{
					umbral_dist		*= options.ALFA;
					umbral_ang		*= options.ALFA;
					if (umbral_dist < options.smallestThresholdDist )
						keepApproaching = false;

					if (++offset_other_map_points>=options.corresponding_points_decimation)
						offset_other_map_points=0;
				}

				lastMeanPose = gaussPdf->mean;

			}	// end of "else, there are correspondences"

			// Next iteration:
			outInfo.nIterations++;

			if (outInfo.nIterations >= options.maxIterations && umbral_dist>options.smallestThresholdDist)
			{
				umbral_dist		*= options.ALFA;
			}

		} while	( (keepApproaching && outInfo.nIterations<options.maxIterations) ||
					(outInfo.nIterations >= options.maxIterations && umbral_dist>options.smallestThresholdDist) );


		// -------------------------------------------------
		//   Obtain the covariance matrix of the estimation
		// -------------------------------------------------
		if (!options.skip_cov_calculation && nCorrespondences)
		{
			// ...
		}

		//
		outInfo.goodness = correspondencesRatio;


	} // end of "if m2 is not empty"

	// Return the gaussian distribution:
	resultPDF = gaussPdf;

	return resultPDF;

	MRPT_END
}



