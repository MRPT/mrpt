/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/random.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/round.h> // round()
#include <mrpt/math/geometry.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/math/data_utils.h> // averageLogLikelihood()
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::system;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CBeaconMap,beaconMap", mrpt::maps::CBeaconMap)

CBeaconMap::TMapDefinition::TMapDefinition()
{
}

void CBeaconMap::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	//const std::string sSectCreation = sectionNamePrefix+string("_creationOpts");
	//MRPT_LOAD_CONFIG_VAR(resolution, float,   source,sSectCreation);

	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
	likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix+string("_likelihoodOpts") );
}

void CBeaconMap::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	//LOADABLEOPTS_DUMP_VAR(resolution     , float);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CBeaconMap::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CBeaconMap::TMapDefinition &def = *dynamic_cast<const CBeaconMap::TMapDefinition*>(&_def);
	CBeaconMap *obj = new CBeaconMap();
	obj->insertionOptions  = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========


IMPLEMENTS_SERIALIZABLE(CBeaconMap, CMetricMap,mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CBeaconMap::CBeaconMap() :
	m_beacons(),
	likelihoodOptions(),
	insertionOptions()
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CBeaconMap::internal_clear()
{
	m_beacons.clear();
}

/*---------------------------------------------------------------
						getLandmarksCount
  ---------------------------------------------------------------*/
size_t  CBeaconMap::size() const
{
	return m_beacons.size();
}

/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CBeaconMap::resize(const size_t N)
{
	m_beacons.resize(N);
}

/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CBeaconMap::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << genericMapParams; // v1

		// First, write the number of landmarks:
		const uint32_t n = m_beacons.size();
		out << n;
		// Write all landmarks:
		for (const_iterator	it=begin();it!=end();++it)
			out << (*it);

	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CBeaconMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			if (version>=1)
				in >> genericMapParams; // v1

			uint32_t	n,i;

			// Delete previous content of map:
			clear();

			// Load from stream:
			// Read all landmarks:
			in >> n;
			m_beacons.resize(n);
			for (i=0;i<n;i++) in >> m_beacons[i];

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}


/*---------------------------------------------------------------
					computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CBeaconMap::internal_computeObservationLikelihood(
				const CObservation	*obs,
				const CPose3D		&robotPose3D )
{
	MRPT_START

	/* ===============================================================================================================
		Refer to the papers:
		- IROS 2008, "Efficient Probabilistic Range-Only SLAM",
			http://www.mrpt.org/Paper:RO-SLAM_with_SOG

		- ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
			http://www.mrpt.org/Paper:A_Pure_Probabilistic_Approach_to_Range-Only_SLAM_(ICRA_2008)
	   =============================================================================================================== */

	if ( CLASS_ID(CObservationBeaconRanges)==obs->GetRuntimeClass() )
	{
		/********************************************************************

						OBSERVATION TYPE: CObservationBeaconRanges

				Lik. between "this" and "auxMap";

			********************************************************************/
		double															ret = 0;
		const CObservationBeaconRanges 									*o = static_cast<const CObservationBeaconRanges*>(obs);
		deque<CObservationBeaconRanges::TMeasurement>::const_iterator	it_obs;
		const CBeacon													*beac;
		CPoint3D														sensor3D;

		for (it_obs = o->sensedData.begin();it_obs!=o->sensedData.end();++it_obs)
		{
			// Look for the beacon in this map:
			beac=getBeaconByID( it_obs->beaconID );

			if (beac!=NULL &&
				it_obs->sensedDistance > 0 &&
				!isNaN(it_obs->sensedDistance))
			{
				float sensedRange = it_obs->sensedDistance;

				// FOUND: Compute the likelihood function:
				// -----------------------------------------------------
				// Compute the 3D position of the sensor:
				sensor3D = robotPose3D + it_obs->sensorLocationOnRobot;

				// Depending on the PDF type of the beacon in the map:
				switch(beac->m_typePDF)
				{
				// ------------------------------
				// PDF is MonteCarlo
				// ------------------------------
				case CBeacon::pdfMonteCarlo:
					{
						CPointPDFParticles::CParticleList::const_iterator it;
						CVectorDouble				logWeights(beac->m_locationMC.m_particles.size());
						CVectorDouble				logLiks(beac->m_locationMC.m_particles.size());
						CVectorDouble::iterator 	itLW,itLL;

						for (it=beac->m_locationMC.m_particles.begin(),itLW=logWeights.begin(),itLL=logLiks.begin();it!=beac->m_locationMC.m_particles.end();++it,++itLW,++itLL)
						{
							float	expectedRange = sensor3D.distance3DTo( it->d->x,it->d->y,it->d->z );
							//expectedRange += float(0.1*(1-exp(-0.16*expectedRange)));

							*itLW  = it->log_w; // Linear weight of this likelihood component
							*itLL = -0.5*square((sensedRange-expectedRange)/likelihoodOptions.rangeStd);
							//ret+= exp( -0.5*square((sensedRange-expectedRange)/likelihoodOptions.rangeStd) );
						} // end for it

						if (logWeights.size())
							ret+= math::averageLogLikelihood(logWeights,logLiks);	// A numerically-stable method to average the likelihoods
					}
					break;
				// ------------------------------
				// PDF is Gaussian
				// ------------------------------
				case CBeacon::pdfGauss:
					{
						// Compute the Jacobian H and varZ
						CMatrixFixedNumeric<double,1,3>	 H;
						float		varZ, varR = square( likelihoodOptions.rangeStd );
						float		Ax = beac->m_locationGauss.mean.x() - sensor3D.x();
						float		Ay = beac->m_locationGauss.mean.y() - sensor3D.y();
						float		Az = beac->m_locationGauss.mean.z() - sensor3D.z();
						H(0,0) = Ax;
						H(0,1) = Ay;
						H(0,2) = Az;
						float	expectedRange = sensor3D.distanceTo( beac->m_locationGauss.mean );
						H *= 1.0/expectedRange; //sqrt(Ax*Ax+Ay*Ay+Az*Az);

						varZ = H.multiply_HCHt_scalar(beac->m_locationGauss.cov);

						varZ += varR;

						// Compute the mean expected range (add bias!):
						//expectedRange += float(0.1*(1-exp(-0.16*expectedRange)));

						// Compute the likelihood:
						//   lik \propto exp( -0.5* ( ^z - z  )^2 / varZ );
						//   log_lik = -0.5* ( ^z - z  )^2 / varZ
						ret += -0.5 * square( sensedRange - expectedRange ) / varZ;
					}
					break;
				// ------------------------------
				// PDF is SOG
				// ------------------------------
				case CBeacon::pdfSOG:
					{
						CMatrixDouble13				H;
						CVectorDouble				logWeights(beac->m_locationSOG.size());
						CVectorDouble				logLiks(beac->m_locationSOG.size());
						CVectorDouble::iterator 	itLW,itLL;
						CPointPDFSOG::const_iterator it;
						// For each Gaussian mode:
						for (it=beac->m_locationSOG.begin(),itLW=logWeights.begin(),itLL=logLiks.begin();it!=beac->m_locationSOG.end();++it,++itLW,++itLL)
						{
							// Compute the Jacobian H and varZ
							double varZ, varR = square( likelihoodOptions.rangeStd );
							double Ax = it->val.mean.x() - sensor3D.x();
							double Ay = it->val.mean.y() - sensor3D.y();
							double Az = it->val.mean.z() - sensor3D.z();
							H(0,0) = Ax;
							H(0,1) = Ay;
							H(0,2) = Az;
							double expectedRange = sensor3D.distanceTo( it->val.mean );
							H *= 1.0/expectedRange; //sqrt(Ax*Ax+Ay*Ay+Az*Az);

							varZ = H.multiply_HCHt_scalar(it->val.cov);
							varZ += varR;

							// Compute the mean expected range (add bias!):
							//expectedRange += float(0.1*(1-exp(-0.16*expectedRange)));

							// Compute the likelihood:
							*itLW  = it->log_w; // log-weight of this likelihood component
							*itLL = -0.5 * square( sensedRange - expectedRange ) / varZ;
							} // end for each mode

						// Accumulate to the overall (log) likelihood value:
						if (logWeights.size())
							ret += math::averageLogLikelihood(logWeights,logLiks);  // log( linear_lik / sumW );
					}
					break;

				default:
					THROW_EXCEPTION("Invalid beac->m_typePDF!!!");
				};
			}
			else
			{
				// If not found, a uniform distribution:
				if ( o->maxSensorDistance != o->minSensorDistance )
					ret+= log(1.0/ (o->maxSensorDistance - o->minSensorDistance));
			}
		} // for each sensed beacon "it"

		//printf("ret: %e\n",ret);
		MRPT_CHECK_NORMAL_NUMBER(ret);
		return ret;

	} // end of likelihood of CObservationBeaconRanges
	else
	{
		/********************************************************************
					OBSERVATION TYPE: Unknown
		********************************************************************/
		return 0;
	}
	MRPT_END
}

/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CBeaconMap::internal_insertObservation( const mrpt::obs::CObservation *obs, const CPose3D *robotPose)
{
	MRPT_START

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( CLASS_ID(CObservationBeaconRanges)==obs->GetRuntimeClass() )
	{
		/********************************************************************
						OBSERVATION TYPE: CObservationBeaconRanges
		 ********************************************************************/

		/* ===============================================================================================================
		    Refer to the papers:
			- IROS 2008, "Efficient Probabilistic Range-Only SLAM",
			    http://www.mrpt.org/Paper:RO-SLAM_with_SOG

			- ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
			    http://www.mrpt.org/Paper:A_Pure_Probabilistic_Approach_to_Range-Only_SLAM_(ICRA_2008)
		   =============================================================================================================== */

		// Here we fuse OR create the beacon position PDF:
		// --------------------------------------------------------
		const CObservationBeaconRanges	*o = static_cast<const CObservationBeaconRanges*>(obs);

		for (deque<CObservationBeaconRanges::TMeasurement>::const_iterator it=o->sensedData.begin();it!=o->sensedData.end();++it)
		{
			CPoint3D		sensorPnt( robotPose3D + it->sensorLocationOnRobot );
			float			sensedRange = it->sensedDistance;
			unsigned int	sensedID = it->beaconID;

			CBeacon			*beac = getBeaconByID(sensedID);

			if (sensedRange>0) // Only sensible range values!
			{
				if (!beac)
				{
					// ======================================
					//                INSERT
					// ======================================
					CBeacon			newBeac;
					newBeac.m_ID = sensedID;

					if ( insertionOptions.insertAsMonteCarlo )
					{
						// Insert as a new set of samples:
						// ------------------------------------------------

						newBeac.m_typePDF = CBeacon::pdfMonteCarlo;

						size_t		numParts = round(insertionOptions.MC_numSamplesPerMeter * sensedRange);
						ASSERT_(insertionOptions.minElevation_deg<=insertionOptions.maxElevation_deg)
						double 		minA = DEG2RAD(insertionOptions.minElevation_deg);
						double 		maxA = DEG2RAD(insertionOptions.maxElevation_deg);
						newBeac.m_locationMC.setSize(numParts);
						for ( CPointPDFParticles::CParticleList::iterator itP=newBeac.m_locationMC.m_particles.begin();itP!=newBeac.m_locationMC.m_particles.end();++itP)
						{
							double th = randomGenerator.drawUniform(-M_PI,M_PI);
							double el = randomGenerator.drawUniform(minA,maxA);
							double R  = randomGenerator.drawGaussian1D(sensedRange , likelihoodOptions.rangeStd );
							itP->d->x = sensorPnt.x() + R*cos(th)*cos(el);
							itP->d->y = sensorPnt.y() + R*sin(th)*cos(el);
							itP->d->z = sensorPnt.z() + R*sin(el);
						} // end for itP
					}
					else
					{
						// Insert as a Sum of Gaussians:
						// ------------------------------------------------
                        newBeac.m_typePDF = CBeacon::pdfSOG;
						CBeacon::generateRingSOG(
							sensedRange,            // Sensed range
							newBeac.m_locationSOG,  // Output SOG
							this,                   // My CBeaconMap, for options.
							sensorPnt				// Sensor point
							);
					}

					// and insert it:
					m_beacons.push_back( newBeac );

				} // end insert
				else
				{
					// ======================================
					//					FUSE
					// ======================================
					switch(beac->m_typePDF)
					{
					// ------------------------------
					// FUSE: PDF is MonteCarlo
					// ------------------------------
					case CBeacon::pdfMonteCarlo:
						{
							double		maxW = -1e308, sumW=0;
							// Update weights:
							// --------------------
							for (CPointPDFParticles::CParticleList::iterator it=beac->m_locationMC.m_particles.begin();it!=beac->m_locationMC.m_particles.end();++it)
							{
								float	expectedRange = sensorPnt.distance3DTo( it->d->x,it->d->y,it->d->z );
								// Add bias:
								//expectedRange += float(0.1*(1-exp(-0.16*expectedRange)));
								it->log_w += -0.5*square((sensedRange-expectedRange)/likelihoodOptions.rangeStd);
								maxW=max(it->log_w,maxW);
								sumW+=exp(it->log_w);
							} // end for it

							// Perform resampling (SIR filter) or not (simply accumulate weights)??
							// ------------------------------------------------------------------------
							if (insertionOptions.MC_performResampling)
							{
								// Yes, perform an auxiliary PF SIR here:
								// ---------------------------------------------
								if (beac->m_locationMC.ESS() < 0.5)
								{
									// We must resample:
									// Make a list with the log weights:
									vector<double> log_ws;
									vector<size_t> indxs;
									beac->m_locationMC.getWeights( log_ws );

									// And compute the resampled indexes:
									CParticleFilterCapable::computeResampling(
										CParticleFilter::prSystematic,
										log_ws,
										indxs );

									// Replace with the new samples:
									beac->m_locationMC.performSubstitution( indxs );

									// Determine if this is a 2D beacon map:
									bool	is2D = (insertionOptions.minElevation_deg==insertionOptions.maxElevation_deg);
									float	noiseStd = insertionOptions.MC_afterResamplingNoise;

									// AND, add a small noise:
									CPointPDFParticles::CParticleList::iterator		itSample;
									for (itSample=beac->m_locationMC.m_particles.begin();itSample!=beac->m_locationMC.m_particles.end();++itSample)
									{
										itSample->d->x += randomGenerator.drawGaussian1D( 0,noiseStd );
										itSample->d->y += randomGenerator.drawGaussian1D( 0,noiseStd );
										if (!is2D)
											itSample->d->z += randomGenerator.drawGaussian1D( 0,noiseStd );
									}

								}
							} // end "do resample"
							else
							{
								// Do not resample:
								// ---------------------------------------------

								// Remove very very very unlikely particles:
								// -------------------------------------------
								for (CPointPDFParticles::CParticleList::iterator it=beac->m_locationMC.m_particles.begin();it!=beac->m_locationMC.m_particles.end();  )
								{
									if ( it->log_w < (maxW-insertionOptions.MC_thresholdNegligible) )
									{
										it->d.reset();
										it = beac->m_locationMC.m_particles.erase( it );
									}
									else ++it;
								}
							} // end "do not resample"

							// Normalize weights:
							//  log_w = log( exp(log_w)/sumW ) ->
							//  log_w -= log(sumW);
							// -----------------------------------------
							sumW=log(sumW);
							for (CPointPDFParticles::CParticleList::iterator it=beac->m_locationMC.m_particles.begin();it!=beac->m_locationMC.m_particles.end();++it)
								it->log_w -= sumW;

							// Is the moment to turn into a Gaussian??
							// -------------------------------------------
							CPoint3D MEAN;
							CMatrixDouble33	COV;
							beac->m_locationMC.getCovarianceAndMean(COV,MEAN);

							double D1 = sqrt(COV(0,0));
							double D2 = sqrt(COV(1,1));
							double D3 = sqrt(COV(2,2));

							double mxVar = max3( D1, D2, D3 );

							if (mxVar < insertionOptions.MC_maxStdToGauss )
							{
								// Collapse into Gaussian:
								beac->m_locationMC.clear();		// Erase prev. samples

								// Assure a non-null covariance!
								CMatrixDouble	COV2 = CMatrixDouble(COV);
								COV2.setSize(2,2);
								if (COV2.det()==0)
								{
									COV.setIdentity();
									COV*= square( 0.01f );
									if (insertionOptions.minElevation_deg == insertionOptions.maxElevation_deg )
										COV(2,2) = 0;	// We are in a 2D map:
								}

								beac->m_typePDF = CBeacon::pdfGauss; // Pass to gaussian.
								beac->m_locationGauss.mean = MEAN;
								beac->m_locationGauss.cov  = COV;
							}
						}
						break;

					// ------------------------------
					// FUSE: PDF is Gaussian:
					// ------------------------------
					case CBeacon::pdfGauss:
						{
							// Compute the mean expected range:
							float	expectedRange = sensorPnt.distanceTo( beac->m_locationGauss.mean );
							float	varR = square( likelihoodOptions.rangeStd );
							//bool	useEKF_or_KF = true;

							//if (useEKF_or_KF)
							{
								// EKF method:
								// ---------------------
								// Add bias:
								//expectedRange += float(0.1*(1-exp(-0.16*expectedRange)));

								// An EKF for updating the Gaussian:
								float	y = sensedRange - expectedRange;

								// Compute the Jacobian H and varZ
								CMatrixDouble13		H;
								double varZ;
								double Ax = (beac->m_locationGauss.mean.x() - sensorPnt.x());
								double Ay = (beac->m_locationGauss.mean.y() - sensorPnt.y());
								double Az = (beac->m_locationGauss.mean.z() - sensorPnt.z());
								H(0,0) = Ax; H(0,1) = Ay; H(0,2) = Az;
								H *= 1.0/expectedRange; //sqrt(Ax*Ax+Ay*Ay+Az*Az);
								varZ =  H.multiply_HCHt_scalar(beac->m_locationGauss.cov);
								varZ += varR;

								CMatrixDouble31		K;
								K.multiply_ABt( beac->m_locationGauss.cov, H );
								K *= 1.0/varZ;

								// Update stage of the EKF:
								beac->m_locationGauss.mean.x_incr( K(0,0) * y );
								beac->m_locationGauss.mean.y_incr(K(1,0) * y );
								beac->m_locationGauss.mean.z_incr( K(2,0) * y );

								beac->m_locationGauss.cov = (Eigen::Matrix<double,3,3>::Identity() - K*H) * beac->m_locationGauss.cov;
								//beac->m_locationGauss.cov.force_symmetry();
							}
						}
						break;
					// ------------------------------
					// FUSE: PDF is SOG
					// ------------------------------
					case CBeacon::pdfSOG:
						{
							// Compute the mean expected range for this mode:
							float	varR = square( likelihoodOptions.rangeStd );

							// For each Gaussian mode:
							//  1) Update its weight (using the likelihood of the observation linearized at the mean)
							//  2) Update its mean/cov (as in the simple EKF)
							CPointPDFSOG::iterator it;
							double max_w = -1e9;
							for (it=beac->m_locationSOG.begin();it!=beac->m_locationSOG.end();++it)
							{
								double 	expectedRange = sensorPnt.distanceTo( it->val.mean );

								// An EKF for updating the Gaussian:
								double y = sensedRange - expectedRange;

								// Compute the Jacobian H and varZ
								CMatrixDouble13		H;
								double varZ;
								double Ax = ( it->val.mean.x() - sensorPnt.x());
								double Ay = ( it->val.mean.y() - sensorPnt.y());
								double Az = ( it->val.mean.z() - sensorPnt.z());
								H(0,0) = Ax; H(0,1) = Ay; H(0,2) = Az;
								H *= 1.0/expectedRange; //sqrt(Ax*Ax+Ay*Ay+Az*Az);
								varZ =  H.multiply_HCHt_scalar( it->val.cov );
								varZ += varR;
								CMatrixDouble31		K;
								K.multiply( it->val.cov, H.transpose());
								K *= 1.0/varZ;

								// Update stage of the EKF:
								it->val.mean.x_incr( K(0,0) * y );
								it->val.mean.y_incr( K(1,0) * y );
								it->val.mean.z_incr( K(2,0) * y );

								it->val.cov = (Eigen::Matrix<double,3,3>::Identity() - K*H) * it->val.cov;
								//it->val.cov.force_symmetry();

								// Update the weight of this mode:
								// ----------------------------------
								it->log_w += -0.5 * square( y ) / varZ;

								max_w = max(max_w,it->log_w);	// keep the maximum mode weight
							} // end for each mode

							// Remove modes with negligible weights:
							// -----------------------------------------------------------
							for (it=beac->m_locationSOG.begin();it!=beac->m_locationSOG.end(); )
							{
								if (max_w - it->log_w > insertionOptions.SOG_thresholdNegligible )
								{
									// Remove the mode:
									it = beac->m_locationSOG.erase( it );
								}
								else ++it;
							}

							//printf("ESS: %f\n",beac->m_locationSOG.ESS());

							// Normalize the weights:
							beac->m_locationSOG.normalizeWeights();

							// Should we pass this beacon to a single Gaussian mode?
							// -----------------------------------------------------------
							CPoint3D  curMean;
							CMatrixDouble33	curCov;
							beac->m_locationSOG.getCovarianceAndMean(curCov,curMean);

							double D1 = sqrt(curCov(0,0));
							double D2 = sqrt(curCov(1,1));
							double D3 = sqrt(curCov(2,2));
							float maxDiag = max3(D1,D2,D3);

							if (maxDiag<0.10f)
							{
								// Yes, transform:
								beac->m_locationGauss.mean = curMean;
								beac->m_locationGauss.cov = curCov;
								beac->m_typePDF = CBeacon::pdfGauss;
								// Done!
							}
						}
						break;
					default:
						THROW_EXCEPTION("Invalid beac->m_typePDF!!!");
					};

				} // end fuse
			} // end if range makes sense
		} // end for each observation

		// DONE!!
		// Observation was successfully inserted into the map
		return true;
	}
	else
	{
		return false;
	}

	MRPT_END
}

/*---------------------------------------------------------------
				determineMatching2D
  ---------------------------------------------------------------*/
void CBeaconMap::determineMatching2D(
	const mrpt::maps::CMetricMap      * otherMap,
	const CPose2D         & otherMapPose,
	TMatchingPairList     & correspondences,
	const TMatchingParams & params,
	TMatchingExtraResults & extraResults ) const
{
	MRPT_UNUSED_PARAM(params);
	MRPT_START
	extraResults = TMatchingExtraResults();

	CBeaconMap	auxMap;
	CPose3D		otherMapPose3D(otherMapPose);

	// Check the other map class:
	ASSERT_( otherMap->GetRuntimeClass() == CLASS_ID(CBeaconMap) );
	const CBeaconMap	*otherMap2 = static_cast<const CBeaconMap*>( otherMap );
	vector<bool>	otherCorrespondences;

	// Coordinates change:
	auxMap.changeCoordinatesReference( otherMapPose3D, otherMap2 );

	// Use the 3D matching method:
	computeMatchingWith3DLandmarks( otherMap2,
									correspondences,
									extraResults.correspondencesRatio,
									otherCorrespondences );

	MRPT_END

}

/*---------------------------------------------------------------
				changeCoordinatesReference
  ---------------------------------------------------------------*/
void  CBeaconMap::changeCoordinatesReference( const CPose3D &newOrg )
{
	// Change the reference of each individual beacon:
	for (iterator lm=m_beacons.begin();lm!=m_beacons.end();++lm)
		lm->changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
				changeCoordinatesReference
  ---------------------------------------------------------------*/
void  CBeaconMap::changeCoordinatesReference( const CPose3D &newOrg, const mrpt::maps::CBeaconMap *otherMap )
{
	// In this object we cannot apply any special speed-up: Just copy and change coordinates:
	(*this) = *otherMap;
	changeCoordinatesReference(newOrg);
}


/*---------------------------------------------------------------
						computeMatchingWith3DLandmarks
  ---------------------------------------------------------------*/
void  CBeaconMap::computeMatchingWith3DLandmarks(
    const mrpt::maps::CBeaconMap						*anotherMap,
    TMatchingPairList						&correspondences,
    float									&correspondencesRatio,
    vector<bool>						&otherCorrespondences) const
{
	MRPT_START

	TSequenceBeacons::const_iterator		thisIt,otherIt;
	size_t									nThis,nOther;
	unsigned int							j,k;
	TMatchingPair							match;
	CPointPDFGaussian						pointPDF_k, pointPDF_j;
	vector<bool>						thisLandmarkAssigned;

	// Get the number of landmarks:
	nThis = m_beacons.size();
	nOther = anotherMap->m_beacons.size();

	// Initially no LM has a correspondence:
	thisLandmarkAssigned.resize( nThis, false );

	// Initially, set all landmarks without correspondences:
	correspondences.clear();
	otherCorrespondences.clear();
	otherCorrespondences.resize( nOther, false );
	correspondencesRatio = 0;

	for (k=0,otherIt=anotherMap->m_beacons.begin();otherIt!=anotherMap->m_beacons.end();++otherIt,++k)
	{
		for (j=0,thisIt=m_beacons.begin();thisIt!=m_beacons.end();++thisIt,++j)
		{
			// Is it a correspondence?
			if ( (otherIt)->m_ID == (thisIt)->m_ID )
			{
				// If a previous correspondence for this LM was found, discard this one!
				if ( !thisLandmarkAssigned[ j ] )
				{
					thisLandmarkAssigned[ j ] = true;

					// OK: A correspondence found!!
					otherCorrespondences[ k ] = true;

					match.this_idx  = j;

					CPoint3D	mean_j = m_beacons[j].getMeanVal();

					match.this_x	= mean_j.x();
					match.this_y	= mean_j.y();
					match.this_z	= mean_j.z();

					CPoint3D	mean_k = anotherMap->m_beacons[k].getMeanVal();
					match.other_idx = k;
					match.other_x	= mean_k.x();
					match.other_y	= mean_k.y();
					match.other_z	= mean_k.z();

					correspondences.push_back( match );
				}
			}

		} // end of "otherIt" is SIFT

	} // end of other it., k

	// Compute the corrs ratio:
	correspondencesRatio = 2.0f * correspondences.size() / static_cast<float>( nThis + nOther);

	MRPT_END
}

/*---------------------------------------------------------------
						saveToMATLABScript3D
  ---------------------------------------------------------------*/
bool  CBeaconMap::saveToMATLABScript3D(
	const string & file,
	const char   *style,
	float  confInterval ) const
{
	MRPT_UNUSED_PARAM(style);
	MRPT_UNUSED_PARAM(confInterval);

	FILE	*f= os::fopen(file.c_str(),"wt");
	if (!f)
		return false;

	// Header:
	os::fprintf(f,"%%-------------------------------------------------------\n");
	os::fprintf(f,"%% File automatically generated using the MRPT method:\n");
	os::fprintf(f,"%%   'CBeaconMap::saveToMATLABScript3D'\n");
	os::fprintf(f,"%%\n");
	os::fprintf(f,"%%                        ~ MRPT ~\n");
	os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @ 2006\n");
	os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
	os::fprintf(f,"%%-------------------------------------------------------\n\n");

	// Main code:
	os::fprintf(f,"hold on;\n\n");
	utils::CStringList	strs;
	string			s;

	for (const_iterator	it=m_beacons.begin();it!=m_beacons.end();++it)
	{
		it->getAsMatlabDrawCommands( strs );
		strs.getText(s);
		os::fprintf(f,"%s",s.c_str());
	}

	os::fprintf(f,"axis equal;grid on;");

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
					TLikelihoodOptions
  ---------------------------------------------------------------*/
CBeaconMap::TLikelihoodOptions::TLikelihoodOptions() :
	rangeStd			(0.08f )
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CBeaconMap::TLikelihoodOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CBeaconMap::TLikelihoodOptions] ------------ \n\n");

	out.printf("rangeStd                                = %f\n",rangeStd);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CBeaconMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&iniFile,
	const string &section)
{
	rangeStd					= iniFile.read_float(section.c_str(),"rangeStd",rangeStd);
}

/*---------------------------------------------------------------
					TInsertionOptions
  ---------------------------------------------------------------*/
CBeaconMap::TInsertionOptions::TInsertionOptions() :
	insertAsMonteCarlo		( true ),
	maxElevation_deg		( 0),
	minElevation_deg		( 0 ),
	MC_numSamplesPerMeter	( 1000 ),
	MC_maxStdToGauss		( 0.4f ),
	MC_thresholdNegligible	( 5 ),
	MC_performResampling	( false ),
	MC_afterResamplingNoise ( 0.01f ),
	SOG_thresholdNegligible ( 20.0f ),
	SOG_maxDistBetweenGaussians ( 1.0f ),
	SOG_separationConstant ( 3.0f )
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CBeaconMap::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CBeaconMap::TInsertionOptions] ------------ \n\n");

	out.printf("insertAsMonteCarlo                      = %c\n",insertAsMonteCarlo ? 'Y':'N');
	out.printf("minElevation_deg                        = %.03f\n",minElevation_deg);
	out.printf("maxElevation_deg                        = %.03f\n",maxElevation_deg);
	out.printf("MC_numSamplesPerMeter                   = %d\n",MC_numSamplesPerMeter);
	out.printf("MC_maxStdToGauss                        = %.03f\n",MC_maxStdToGauss);
	out.printf("MC_thresholdNegligible                  = %.03f\n",MC_thresholdNegligible);
	out.printf("MC_performResampling                    = %c\n",MC_performResampling ? 'Y':'N');
	out.printf("MC_afterResamplingNoise                 = %.03f\n",MC_afterResamplingNoise);
	out.printf("SOG_thresholdNegligible                 = %.03f\n",SOG_thresholdNegligible);
	out.printf("SOG_maxDistBetweenGaussians             = %.03f\n",SOG_maxDistBetweenGaussians);
	out.printf("SOG_separationConstant                  = %.03f\n",SOG_separationConstant);


	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CBeaconMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&iniFile,
	const string &section)
{
	MRPT_LOAD_CONFIG_VAR(insertAsMonteCarlo,bool,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(maxElevation_deg,float,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(minElevation_deg,float,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_numSamplesPerMeter,int,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_maxStdToGauss,float,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_thresholdNegligible,float,			iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_performResampling,bool,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_afterResamplingNoise,float,			iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(SOG_thresholdNegligible,float,			iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(SOG_maxDistBetweenGaussians,float,		iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(SOG_separationConstant,float,			iniFile,section.c_str());

}

/*---------------------------------------------------------------
					 isEmpty
  ---------------------------------------------------------------*/
bool  CBeaconMap::isEmpty() const
{
	return size()==0;
}

/*---------------------------------------------------------------
					 simulateBeaconReadings
  ---------------------------------------------------------------*/
void  CBeaconMap::simulateBeaconReadings(
    const CPose3D							&in_robotPose,
    const CPoint3D						&in_sensorLocationOnRobot,
    CObservationBeaconRanges        &out_Observations ) const
{
	TSequenceBeacons::const_iterator					    it;
	mrpt::obs::CObservationBeaconRanges::TMeasurement	newMeas;
	CPoint3D										point3D,beacon3D;
	CPointPDFGaussian								beaconPDF;

	// Compute the 3D position of the sensor:
	point3D = in_robotPose + in_sensorLocationOnRobot;

	// Clear output data:
	out_Observations.sensedData.clear();

	// For each BEACON landmark in the map:
	for (it=m_beacons.begin();it!=m_beacons.end();++it)
	{
	    it->getMean(beacon3D);

        float	range = point3D.distanceTo( beacon3D );

		if (range < out_Observations.maxSensorDistance && range > out_Observations.minSensorDistance)
		{
			// Add noise:
			range += randomGenerator.drawGaussian1D(0, out_Observations.stdError );

			// Fill out:
			newMeas.beaconID				= it->m_ID;
			newMeas.sensorLocationOnRobot	= in_sensorLocationOnRobot;
			newMeas.sensedDistance			= range;

			// Insert:
			out_Observations.sensedData.push_back( newMeas );
		}
	} // end for it
	// Done!
}

/*---------------------------------------------------------------
					 saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CBeaconMap::saveMetricMapRepresentationToFile( const string	&filNamePrefix ) const
{
	MRPT_START

	// Matlab:
	string		fil1( filNamePrefix + string("_3D.m") );
	saveToMATLABScript3D( fil1 );

	// 3D Scene:
	opengl::COpenGLScene				scene;
	opengl::CSetOfObjectsPtr obj3D = opengl::CSetOfObjects::Create();

	getAs3DObject( obj3D );
	opengl::CGridPlaneXYPtr	objGround = opengl::CGridPlaneXY::Create(-100,100,-100,100,0,1);

	scene.insert(obj3D);
	scene.insert(objGround);

	string		fil2( filNamePrefix + string("_3D.3Dscene") );
	CFileOutputStream	f(fil2.c_str());
	f << scene;

	// Textual representation:
	string		fil3( filNamePrefix + string("_covs.txt") );
	saveToTextFile( fil3 );

	// Total number of particles / modes:
	string		fil4( filNamePrefix + string("_population.txt") );
	{
			FILE *f = os::fopen(fil4.c_str(),"wt");
			if (f)
			{
				size_t  nParts = 0, nGaussians = 0;

				for (TSequenceBeacons::const_iterator	it=m_beacons.begin();it!=m_beacons.end();++it)
				{
					switch (it->m_typePDF)
					{
					case CBeacon::pdfMonteCarlo:
						nParts += it->m_locationMC.size();
						break;
					case CBeacon::pdfSOG:
						nGaussians += it->m_locationSOG.size();
						break;
					case CBeacon::pdfGauss:
						nGaussians ++;
						break;
					};
				}

				fprintf(f,"%u %u",static_cast<unsigned>(nParts), static_cast<unsigned>(nGaussians) );
				os::fclose(f);
			}
	}


	MRPT_END
}

/*---------------------------------------------------------------
 						getAs3DObject
  ---------------------------------------------------------------*/
void  CBeaconMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START

	if (!genericMapParams.enableSaveAs3DObject) return;

	// ------------------------------------------------
	//  Add the XYZ corner for the current area:
	// ------------------------------------------------
	outObj->insert( opengl::stock_objects::CornerXYZ() );

	// Save 3D ellipsoids or whatever representation:
	for (const_iterator	it=m_beacons.begin();it!=m_beacons.end();++it)
		it->getAs3DObject( outObj );

	MRPT_END
}

/*---------------------------------------------------------------
   Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  maxDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
 ----------------------------------------------------------------*/
float CBeaconMap::compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap2, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const
{
	MRPT_START

	// Compare to a similar map only:
	const CBeaconMap	*otherMap = NULL;

	if ( otherMap2->GetRuntimeClass() == CLASS_ID(CBeaconMap) )
		otherMap = static_cast<const CBeaconMap*>( otherMap2);

	if (!otherMap) return 0;

	TMatchingPairList		matchList;
	vector<bool>		otherCorrespondences;
	float					out_corrsRatio;

	CBeaconMap			modMap;

	modMap.changeCoordinatesReference( 	otherMapPose, otherMap );

	computeMatchingWith3DLandmarks(
		&modMap,
		matchList,
		out_corrsRatio,
		otherCorrespondences );

	return out_corrsRatio;

	MRPT_END
}


/*---------------------------------------------------------------
					getBeaconByID
 ---------------------------------------------------------------*/
const CBeacon * CBeaconMap::getBeaconByID( CBeacon::TBeaconID  id ) const
{
	for (const_iterator	it=m_beacons.begin();it!=m_beacons.end();++it)
		if (it->m_ID==id)
			return &(*it);
	return NULL;
}

/*---------------------------------------------------------------
					getBeaconByID
 ---------------------------------------------------------------*/
CBeacon * CBeaconMap::getBeaconByID( CBeacon::TBeaconID  id )
{
	for (iterator	it=m_beacons.begin();it!=m_beacons.end();++it)
		if (it->m_ID==id)
			return &(*it);
	return NULL;
}

/*---------------------------------------------------------------
					saveToTextFile
- VX VY VZ: Variances of each dimension (C11, C22, C33)
- DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
- C12, C13, C23: Cross covariances
 ---------------------------------------------------------------*/
void CBeaconMap::saveToTextFile(const string &fil) const
{
	MRPT_START
	FILE	*f = os::fopen(fil.c_str(),"wt");
	ASSERT_(f!=NULL);

	CPoint3D		p;
	CMatrixDouble33 C;

	for (const_iterator	it=m_beacons.begin();it!=m_beacons.end();++it)
	{
		it->getCovarianceAndMean(C,p);

		float	D3 = C.det();
		float	D2 = C(0,0)*C(1,1) - square( C(0,1) );
		os::fprintf(f,"%i %f %f %f %e %e %e %e %e %e %e %e\n",
			static_cast<int>(it->m_ID),
			p.x(),p.y(),p.z(),
			C(0,0),C(1,1),C(2,2),
			D2,D3,
			C(0,1),C(1,2),C(1,2)
			);
	}

	os::fclose(f);
	MRPT_END
}
