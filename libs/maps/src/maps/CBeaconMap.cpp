/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/math/data_utils.h>  // averageLogLikelihood()
#include <mrpt/math/geometry.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::system;
using namespace mrpt::tfest;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"mrpt::maps::CBeaconMap,beaconMap", mrpt::maps::CBeaconMap)

CBeaconMap::TMapDefinition::TMapDefinition() = default;
void CBeaconMap::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	// const std::string sSectCreation =
	// sectionNamePrefix+string("_creationOpts");
	// MRPT_LOAD_CONFIG_VAR(resolution, float,   source,sSectCreation);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CBeaconMap::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	// LOADABLEOPTS_DUMP_VAR(resolution     , float);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CBeaconMap::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CBeaconMap::TMapDefinition& def =
		*dynamic_cast<const CBeaconMap::TMapDefinition*>(&_def);
	auto* obj = new CBeaconMap();
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CBeaconMap, CMetricMap, mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CBeaconMap::CBeaconMap() : m_beacons(), likelihoodOptions(), insertionOptions()
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CBeaconMap::internal_clear() { m_beacons.clear(); }
/*---------------------------------------------------------------
						getLandmarksCount
  ---------------------------------------------------------------*/
size_t CBeaconMap::size() const { return m_beacons.size(); }
/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CBeaconMap::resize(const size_t N) { m_beacons.resize(N); }
uint8_t CBeaconMap::serializeGetVersion() const { return 1; }
void CBeaconMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << genericMapParams;  // v1

	// First, write the number of landmarks:
	const uint32_t n = m_beacons.size();
	out << n;
	// Write all landmarks:
	for (const auto& it : *this) out << it;
}

void CBeaconMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			if (version >= 1) in >> genericMapParams;  // v1

			uint32_t n, i;

			// Delete previous content of map:
			clear();

			// Load from stream:
			// Read all landmarks:
			in >> n;
			m_beacons.resize(n);
			for (i = 0; i < n; i++) in >> m_beacons[i];
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
					computeObservationLikelihood
  ---------------------------------------------------------------*/
double CBeaconMap::internal_computeObservationLikelihood(
	const CObservation& obs, const CPose3D& robotPose3D)
{
	MRPT_START

	/* ===============================================================================================================
		Refer to the papers:
		- IROS 2008, "Efficient Probabilistic Range-Only SLAM",
			http://www.mrpt.org/paper-ro-slam-with-sog/

		- ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
			http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/
	   ===============================================================================================================
	   */

	if (CLASS_ID(CObservationBeaconRanges) == obs.GetRuntimeClass())
	{
		/********************************************************************

						OBSERVATION TYPE: CObservationBeaconRanges

				Lik. between "this" and "auxMap";

			********************************************************************/
		double ret = 0;
		const auto& o = dynamic_cast<const CObservationBeaconRanges&>(obs);
		const CBeacon* beac;
		CPoint3D sensor3D;

		for (auto& it_obs : o.sensedData)
		{
			// Look for the beacon in this map:
			beac = getBeaconByID(it_obs.beaconID);

			if (beac != nullptr && it_obs.sensedDistance > 0 &&
				!std::isnan(it_obs.sensedDistance))
			{
				float sensedRange = it_obs.sensedDistance;

				// FOUND: Compute the likelihood function:
				// -----------------------------------------------------
				// Compute the 3D position of the sensor:
				sensor3D = robotPose3D + it_obs.sensorLocationOnRobot;

				// Depending on the PDF type of the beacon in the map:
				switch (beac->m_typePDF)
				{
					// ------------------------------
					// PDF is MonteCarlo
					// ------------------------------
					case CBeacon::pdfMonteCarlo:
					{
						CPointPDFParticles::CParticleList::const_iterator it;
						CVectorDouble logWeights(
							beac->m_locationMC.m_particles.size());
						CVectorDouble logLiks(
							beac->m_locationMC.m_particles.size());
						CVectorDouble::iterator itLW, itLL;

						for (it = beac->m_locationMC.m_particles.begin(),
							itLW = logWeights.begin(), itLL = logLiks.begin();
							 it != beac->m_locationMC.m_particles.end();
							 ++it, ++itLW, ++itLL)
						{
							float expectedRange = sensor3D.distance3DTo(
								it->d->x, it->d->y, it->d->z);
							// expectedRange +=
							// float(0.1*(1-exp(-0.16*expectedRange)));

							*itLW = it->log_w;  // Linear weight of this
							// likelihood component
							*itLL = -0.5 * square(
											   (sensedRange - expectedRange) /
											   likelihoodOptions.rangeStd);
							// ret+= exp(
							// -0.5*square((sensedRange-expectedRange)/likelihoodOptions.rangeStd)
							// );
						}  // end for it

						if (logWeights.size())
							ret += math::averageLogLikelihood(
								logWeights, logLiks);  // A numerically-stable
						// method to average the
						// likelihoods
					}
					break;
					// ------------------------------
					// PDF is Gaussian
					// ------------------------------
					case CBeacon::pdfGauss:
					{
						// Compute the Jacobian H and varZ
						CMatrixFixed<double, 1, 3> H;
						float varZ, varR = square(likelihoodOptions.rangeStd);
						float Ax =
							beac->m_locationGauss.mean.x() - sensor3D.x();
						float Ay =
							beac->m_locationGauss.mean.y() - sensor3D.y();
						float Az =
							beac->m_locationGauss.mean.z() - sensor3D.z();
						H(0, 0) = Ax;
						H(0, 1) = Ay;
						H(0, 2) = Az;
						float expectedRange =
							sensor3D.distanceTo(beac->m_locationGauss.mean);
						H.asEigen() *=
							1.0 / expectedRange;  // sqrt(Ax*Ax+Ay*Ay+Az*Az);

						varZ = mrpt::math::multiply_HCHt_scalar(
							H, beac->m_locationGauss.cov);

						varZ += varR;

						// Compute the mean expected range (add bias!):
						// expectedRange +=
						// float(0.1*(1-exp(-0.16*expectedRange)));

						// Compute the likelihood:
						//   lik \propto exp( -0.5* ( ^z - z  )^2 / varZ );
						//   log_lik = -0.5* ( ^z - z  )^2 / varZ
						ret +=
							-0.5 * square(sensedRange - expectedRange) / varZ;
					}
					break;
					// ------------------------------
					// PDF is SOG
					// ------------------------------
					case CBeacon::pdfSOG:
					{
						CMatrixDouble13 H;
						CVectorDouble logWeights(beac->m_locationSOG.size());
						CVectorDouble logLiks(beac->m_locationSOG.size());
						CVectorDouble::iterator itLW, itLL;
						CPointPDFSOG::const_iterator it;
						// For each Gaussian mode:
						for (it = beac->m_locationSOG.begin(),
							itLW = logWeights.begin(), itLL = logLiks.begin();
							 it != beac->m_locationSOG.end();
							 ++it, ++itLW, ++itLL)
						{
							// Compute the Jacobian H and varZ
							double varZ,
								varR = square(likelihoodOptions.rangeStd);
							double Ax = it->val.mean.x() - sensor3D.x();
							double Ay = it->val.mean.y() - sensor3D.y();
							double Az = it->val.mean.z() - sensor3D.z();
							H(0, 0) = Ax;
							H(0, 1) = Ay;
							H(0, 2) = Az;
							double expectedRange =
								sensor3D.distanceTo(it->val.mean);
							H.asEigen() *=
								1.0 /
								expectedRange;  // sqrt(Ax*Ax+Ay*Ay+Az*Az);

							varZ = mrpt::math::multiply_HCHt_scalar(
								H, it->val.cov);
							varZ += varR;

							// Compute the mean expected range (add bias!):
							// expectedRange +=
							// float(0.1*(1-exp(-0.16*expectedRange)));

							// Compute the likelihood:
							*itLW = it->log_w;  // log-weight of this likelihood
							// component
							*itLL = -0.5 * square(sensedRange - expectedRange) /
									varZ;
						}  // end for each mode

						// Accumulate to the overall (log) likelihood value:
						if (logWeights.size())
							ret += math::averageLogLikelihood(
								logWeights,
								logLiks);  // log( linear_lik / sumW );
					}
					break;

					default:
						THROW_EXCEPTION("Invalid beac->m_typePDF!!!");
				};
			}
			else
			{
				// If not found, a uniform distribution:
				if (o.maxSensorDistance != o.minSensorDistance)
					ret +=
						log(1.0 / (o.maxSensorDistance - o.minSensorDistance));
			}
		}  // for each sensed beacon "it"

		// printf("ret: %e\n",ret);
		MRPT_CHECK_NORMAL_NUMBER(ret);
		return ret;

	}  // end of likelihood of CObservationBeaconRanges
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
bool CBeaconMap::internal_insertObservation(
	const mrpt::obs::CObservation& obs, const CPose3D* robotPose)
{
	MRPT_START

	CPose2D robotPose2D;
	CPose3D robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if (CLASS_ID(CObservationBeaconRanges) == obs.GetRuntimeClass())
	{
		/********************************************************************
						OBSERVATION TYPE: CObservationBeaconRanges
		 ********************************************************************/

		/* ===============================================================================================================
		   Refer to the papers:
		   - IROS 2008, "Efficient Probabilistic Range-Only SLAM",
			   https://www.mrpt.org/paper-ro-slam-with-sog/

		   - ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
			   https://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/
		  ===============================================================================================================
		  */

		// Here we fuse OR create the beacon position PDF:
		// --------------------------------------------------------
		const auto& o = static_cast<const CObservationBeaconRanges&>(obs);

		for (const auto& it : o.sensedData)
		{
			CPoint3D sensorPnt(robotPose3D + it.sensorLocationOnRobot);
			float sensedRange = it.sensedDistance;
			unsigned int sensedID = it.beaconID;

			CBeacon* beac = getBeaconByID(sensedID);

			if (sensedRange > 0)  // Only sensible range values!
			{
				if (!beac)
				{
					// ======================================
					//                INSERT
					// ======================================
					CBeacon newBeac;
					newBeac.m_ID = sensedID;

					if (insertionOptions.insertAsMonteCarlo)
					{
						// Insert as a new set of samples:
						// ------------------------------------------------

						newBeac.m_typePDF = CBeacon::pdfMonteCarlo;

						size_t numParts = round(
							insertionOptions.MC_numSamplesPerMeter *
							sensedRange);
						ASSERT_(
							insertionOptions.minElevation_deg <=
							insertionOptions.maxElevation_deg);
						double minA =
							DEG2RAD(insertionOptions.minElevation_deg);
						double maxA =
							DEG2RAD(insertionOptions.maxElevation_deg);
						newBeac.m_locationMC.setSize(numParts);
						for (auto& m_particle :
							 newBeac.m_locationMC.m_particles)
						{
							double th =
								getRandomGenerator().drawUniform(-M_PI, M_PI);
							double el =
								getRandomGenerator().drawUniform(minA, maxA);
							double R = getRandomGenerator().drawGaussian1D(
								sensedRange, likelihoodOptions.rangeStd);
							m_particle.d->x =
								sensorPnt.x() + R * cos(th) * cos(el);
							m_particle.d->y =
								sensorPnt.y() + R * sin(th) * cos(el);
							m_particle.d->z = sensorPnt.z() + R * sin(el);
						}  // end for itP
					}
					else
					{
						// Insert as a Sum of Gaussians:
						// ------------------------------------------------
						newBeac.m_typePDF = CBeacon::pdfSOG;
						CBeacon::generateRingSOG(
							sensedRange,  // Sensed range
							newBeac.m_locationSOG,  // Output SOG
							this,  // My CBeaconMap, for options.
							sensorPnt  // Sensor point
						);
					}

					// and insert it:
					m_beacons.push_back(newBeac);

				}  // end insert
				else
				{
					// ======================================
					//					FUSE
					// ======================================
					switch (beac->m_typePDF)
					{
						// ------------------------------
						// FUSE: PDF is MonteCarlo
						// ------------------------------
						case CBeacon::pdfMonteCarlo:
						{
							double maxW = -1e308, sumW = 0;
							// Update weights:
							// --------------------
							for (auto& p : beac->m_locationMC.m_particles)
							{
								float expectedRange = sensorPnt.distance3DTo(
									p.d->x, p.d->y, p.d->z);
								p.log_w +=
									-0.5 * square(
											   (sensedRange - expectedRange) /
											   likelihoodOptions.rangeStd);
								maxW = max(p.log_w, maxW);
								sumW += exp(p.log_w);
							}

							// Perform resampling (SIR filter) or not (simply
							// accumulate weights)??
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
									beac->m_locationMC.getWeights(log_ws);

									// And compute the resampled indexes:
									CParticleFilterCapable::computeResampling(
										CParticleFilter::prSystematic, log_ws,
										indxs);

									// Replace with the new samples:
									beac->m_locationMC.performSubstitution(
										indxs);

									// Determine if this is a 2D beacon map:
									bool is2D =
										(insertionOptions.minElevation_deg ==
										 insertionOptions.maxElevation_deg);
									float noiseStd =
										insertionOptions
											.MC_afterResamplingNoise;

									// AND, add a small noise:
									CPointPDFParticles::CParticleList::iterator
										itSample;
									for (itSample = beac->m_locationMC
														.m_particles.begin();
										 itSample !=
										 beac->m_locationMC.m_particles.end();
										 ++itSample)
									{
										itSample->d->x +=
											getRandomGenerator().drawGaussian1D(
												0, noiseStd);
										itSample->d->y +=
											getRandomGenerator().drawGaussian1D(
												0, noiseStd);
										if (!is2D)
											itSample->d->z +=
												getRandomGenerator()
													.drawGaussian1D(
														0, noiseStd);
									}
								}
							}  // end "do resample"
							else
							{
								// Do not resample:
								// ---------------------------------------------

								// Remove very very very unlikely particles:
								// -------------------------------------------
								for (auto it_p =
										 beac->m_locationMC.m_particles.begin();
									 it_p !=
									 beac->m_locationMC.m_particles.end();)
								{
									if (it_p->log_w <
										(maxW - insertionOptions
													.MC_thresholdNegligible))
									{
										it_p->d.reset();
										it_p = beac->m_locationMC.m_particles
												   .erase(it_p);
									}
									else
										++it_p;
								}
							}  // end "do not resample"

							// Normalize weights:
							//  log_w = log( exp(log_w)/sumW ) ->
							//  log_w -= log(sumW);
							// -----------------------------------------
							sumW = log(sumW);
							for (auto& p : beac->m_locationMC.m_particles)
								p.log_w -= sumW;

							// Is the moment to turn into a Gaussian??
							// -------------------------------------------
							auto [COV, MEAN] =
								beac->m_locationMC.getCovarianceAndMean();

							double D1 = sqrt(COV(0, 0));
							double D2 = sqrt(COV(1, 1));
							double D3 = sqrt(COV(2, 2));

							double mxVar = max3(D1, D2, D3);

							if (mxVar < insertionOptions.MC_maxStdToGauss)
							{
								// Collapse into Gaussian:
								beac->m_locationMC
									.clear();  // Erase prev. samples

								// Assure a non-null covariance!
								CMatrixDouble COV2 = CMatrixDouble(COV);
								COV2.setSize(2, 2);
								if (COV2.det() == 0)
								{
									COV.setIdentity();
									COV *= square(0.01f);
									if (insertionOptions.minElevation_deg ==
										insertionOptions.maxElevation_deg)
										COV(2, 2) = 0;  // We are in a 2D map:
								}

								beac->m_typePDF =
									CBeacon::pdfGauss;  // Pass to gaussian.
								beac->m_locationGauss.mean = MEAN;
								beac->m_locationGauss.cov = COV;
							}
						}
						break;

						// ------------------------------
						// FUSE: PDF is Gaussian:
						// ------------------------------
						case CBeacon::pdfGauss:
						{
							// Compute the mean expected range:
							float expectedRange = sensorPnt.distanceTo(
								beac->m_locationGauss.mean);
							float varR = square(likelihoodOptions.rangeStd);
							// bool	useEKF_or_KF = true;

							// if (useEKF_or_KF)
							{
								// EKF method:
								// ---------------------
								// Add bias:
								// expectedRange +=
								// float(0.1*(1-exp(-0.16*expectedRange)));

								// An EKF for updating the Gaussian:
								float y = sensedRange - expectedRange;

								// Compute the Jacobian H and varZ
								CMatrixDouble13 H;
								double varZ;
								double Ax =
									(beac->m_locationGauss.mean.x() -
									 sensorPnt.x());
								double Ay =
									(beac->m_locationGauss.mean.y() -
									 sensorPnt.y());
								double Az =
									(beac->m_locationGauss.mean.z() -
									 sensorPnt.z());
								H(0, 0) = Ax;
								H(0, 1) = Ay;
								H(0, 2) = Az;
								H.asEigen() *=
									1.0 /
									expectedRange;  // sqrt(Ax*Ax+Ay*Ay+Az*Az);
								varZ = mrpt::math::multiply_HCHt_scalar(
									H, beac->m_locationGauss.cov);
								varZ += varR;

								Eigen::Vector3d K =
									beac->m_locationGauss.cov.asEigen() *
									H.transpose();
								K *= 1.0 / varZ;

								// Update stage of the EKF:
								beac->m_locationGauss.mean.x_incr(K(0, 0) * y);
								beac->m_locationGauss.mean.y_incr(K(1, 0) * y);
								beac->m_locationGauss.mean.z_incr(K(2, 0) * y);

								beac->m_locationGauss.cov =
									((Eigen::Matrix<double, 3, 3>::Identity() -
									  K * H.asEigen()) *
									 beac->m_locationGauss.cov.asEigen())
										.eval();
							}
						}
						break;
						// ------------------------------
						// FUSE: PDF is SOG
						// ------------------------------
						case CBeacon::pdfSOG:
						{
							// Compute the mean expected range for this mode:
							float varR = square(likelihoodOptions.rangeStd);

							// For each Gaussian mode:
							//  1) Update its weight (using the likelihood of
							//  the observation linearized at the mean)
							//  2) Update its mean/cov (as in the simple EKF)
							double max_w = -1e9;
							for (auto& mode : beac->m_locationSOG)
							{
								double expectedRange =
									sensorPnt.distanceTo(mode.val.mean);

								// An EKF for updating the Gaussian:
								double y = sensedRange - expectedRange;

								// Compute the Jacobian H and varZ
								CMatrixDouble13 H;
								double varZ;
								double Ax = (mode.val.mean.x() - sensorPnt.x());
								double Ay = (mode.val.mean.y() - sensorPnt.y());
								double Az = (mode.val.mean.z() - sensorPnt.z());
								H(0, 0) = Ax;
								H(0, 1) = Ay;
								H(0, 2) = Az;
								H.asEigen() *= 1.0 / expectedRange;
								varZ = mrpt::math::multiply_HCHt_scalar(
									H, mode.val.cov);
								varZ += varR;
								Eigen::Vector3d K =
									mode.val.cov.asEigen() * H.transpose();
								K *= 1.0 / varZ;

								// Update stage of the EKF:
								mode.val.mean.x_incr(K(0, 0) * y);
								mode.val.mean.y_incr(K(1, 0) * y);
								mode.val.mean.z_incr(K(2, 0) * y);

								mode.val.cov = (Eigen::Matrix3d::Identity() -
												K * H.asEigen()) *
											   mode.val.cov.asEigen();

								// Update the weight of this mode:
								// ----------------------------------
								mode.log_w += -0.5 * square(y) / varZ;

								// keep the maximum mode weight
								max_w = max(max_w, mode.log_w);
							}  // end for each mode

							// Remove modes with negligible weights:
							// -----------------------------------------------------------
							for (auto it_p = beac->m_locationSOG.begin();
								 it_p != beac->m_locationSOG.end();)
							{
								if (max_w - it_p->log_w >
									insertionOptions.SOG_thresholdNegligible)
									it_p = beac->m_locationSOG.erase(it_p);
								else
									++it_p;
							}

							// Normalize the weights:
							beac->m_locationSOG.normalizeWeights();

							// Should we pass this beacon to a single Gaussian
							// mode?
							// -----------------------------------------------------------
							const auto [curCov, curMean] =
								beac->m_locationSOG.getCovarianceAndMean();

							double D1 = sqrt(curCov(0, 0));
							double D2 = sqrt(curCov(1, 1));
							double D3 = sqrt(curCov(2, 2));
							float maxDiag = max3(D1, D2, D3);

							if (maxDiag < 0.10f)
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

				}  // end fuse
			}  // end if range makes sense
		}  // end for each observation

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
	const mrpt::maps::CMetricMap* otherMap, const CPose2D& otherMapPose,
	TMatchingPairList& correspondences, const TMatchingParams& params,
	TMatchingExtraResults& extraResults) const
{
	MRPT_UNUSED_PARAM(params);
	MRPT_START
	extraResults = TMatchingExtraResults();

	CBeaconMap auxMap;
	CPose3D otherMapPose3D(otherMapPose);

	// Check the other map class:
	ASSERT_(otherMap->GetRuntimeClass() == CLASS_ID(CBeaconMap));
	const auto* otherMap2 = dynamic_cast<const CBeaconMap*>(otherMap);
	vector<bool> otherCorrespondences;

	// Coordinates change:
	auxMap.changeCoordinatesReference(otherMapPose3D, otherMap2);

	// Use the 3D matching method:
	computeMatchingWith3DLandmarks(
		otherMap2, correspondences, extraResults.correspondencesRatio,
		otherCorrespondences);

	MRPT_END
}

/*---------------------------------------------------------------
				changeCoordinatesReference
  ---------------------------------------------------------------*/
void CBeaconMap::changeCoordinatesReference(const CPose3D& newOrg)
{
	// Change the reference of each individual beacon:
	for (auto& m_beacon : m_beacons)
		m_beacon.changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
				changeCoordinatesReference
  ---------------------------------------------------------------*/
void CBeaconMap::changeCoordinatesReference(
	const CPose3D& newOrg, const mrpt::maps::CBeaconMap* otherMap)
{
	// In this object we cannot apply any special speed-up: Just copy and change
	// coordinates:
	(*this) = *otherMap;
	changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
						computeMatchingWith3DLandmarks
  ---------------------------------------------------------------*/
void CBeaconMap::computeMatchingWith3DLandmarks(
	const mrpt::maps::CBeaconMap* anotherMap,
	TMatchingPairList& correspondences, float& correspondencesRatio,
	vector<bool>& otherCorrespondences) const
{
	MRPT_START

	TSequenceBeacons::const_iterator thisIt, otherIt;
	size_t nThis, nOther;
	unsigned int j, k;
	TMatchingPair match;
	CPointPDFGaussian pointPDF_k, pointPDF_j;
	vector<bool> thisLandmarkAssigned;

	// Get the number of landmarks:
	nThis = m_beacons.size();
	nOther = anotherMap->m_beacons.size();

	// Initially no LM has a correspondence:
	thisLandmarkAssigned.resize(nThis, false);

	// Initially, set all landmarks without correspondences:
	correspondences.clear();
	otherCorrespondences.clear();
	otherCorrespondences.resize(nOther, false);
	correspondencesRatio = 0;

	for (k = 0, otherIt = anotherMap->m_beacons.begin();
		 otherIt != anotherMap->m_beacons.end(); ++otherIt, ++k)
	{
		for (j = 0, thisIt = m_beacons.begin(); thisIt != m_beacons.end();
			 ++thisIt, ++j)
		{
			// Is it a correspondence?
			if ((otherIt)->m_ID == (thisIt)->m_ID)
			{
				// If a previous correspondence for this LM was found, discard
				// this one!
				if (!thisLandmarkAssigned[j])
				{
					thisLandmarkAssigned[j] = true;

					// OK: A correspondence found!!
					otherCorrespondences[k] = true;

					match.this_idx = j;

					CPoint3D mean_j = m_beacons[j].getMeanVal();

					match.this_x = mean_j.x();
					match.this_y = mean_j.y();
					match.this_z = mean_j.z();

					CPoint3D mean_k = anotherMap->m_beacons[k].getMeanVal();
					match.other_idx = k;
					match.other_x = mean_k.x();
					match.other_y = mean_k.y();
					match.other_z = mean_k.z();

					correspondences.push_back(match);
				}
			}

		}  // end of "otherIt" is SIFT

	}  // end of other it., k

	// Compute the corrs ratio:
	correspondencesRatio = 2.0f * correspondences.size() / d2f(nThis + nOther);

	MRPT_END
}

/*---------------------------------------------------------------
						saveToMATLABScript3D
  ---------------------------------------------------------------*/
bool CBeaconMap::saveToMATLABScript3D(
	const string& file, const char* style, float confInterval) const
{
	MRPT_UNUSED_PARAM(style);
	MRPT_UNUSED_PARAM(confInterval);

	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	// Header:
	os::fprintf(
		f, "%%-------------------------------------------------------\n");
	os::fprintf(f, "%% File automatically generated using the MRPT method:\n");
	os::fprintf(f, "%%   'CBeaconMap::saveToMATLABScript3D'\n");
	os::fprintf(f, "%%\n");
	os::fprintf(f, "%%                        ~ MRPT ~\n");
	os::fprintf(
		f, "%%  Jose Luis Blanco Claraco, University of Malaga @ 2006\n");
	os::fprintf(f, "%%  http://www.isa.uma.es/ \n");
	os::fprintf(
		f, "%%-------------------------------------------------------\n\n");

	// Main code:
	os::fprintf(f, "hold on;\n\n");
	std::vector<std::string> strs;
	string s;

	for (const auto& m_beacon : m_beacons)
	{
		m_beacon.getAsMatlabDrawCommands(strs);
		mrpt::system::stringListAsString(strs, s);
		os::fprintf(f, "%s", s.c_str());
	}

	os::fprintf(f, "axis equal;grid on;");

	os::fclose(f);
	return true;
}

void CBeaconMap::TLikelihoodOptions::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [CBeaconMap::TLikelihoodOptions] ------------ \n\n";
	out << mrpt::format(
		"rangeStd                                = %f\n", rangeStd);
	out << "\n";
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CBeaconMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
	rangeStd = iniFile.read_float(section.c_str(), "rangeStd", rangeStd);
}

void CBeaconMap::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [CBeaconMap::TInsertionOptions] ------------ \n\n";

	out << mrpt::format(
		"insertAsMonteCarlo                      = %c\n",
		insertAsMonteCarlo ? 'Y' : 'N');
	out << mrpt::format(
		"minElevation_deg                        = %.03f\n", minElevation_deg);
	out << mrpt::format(
		"maxElevation_deg                        = %.03f\n", maxElevation_deg);
	out << mrpt::format(
		"MC_numSamplesPerMeter                   = %d\n",
		MC_numSamplesPerMeter);
	out << mrpt::format(
		"MC_maxStdToGauss                        = %.03f\n", MC_maxStdToGauss);
	out << mrpt::format(
		"MC_thresholdNegligible                  = %.03f\n",
		MC_thresholdNegligible);
	out << mrpt::format(
		"MC_performResampling                    = %c\n",
		MC_performResampling ? 'Y' : 'N');
	out << mrpt::format(
		"MC_afterResamplingNoise                 = %.03f\n",
		MC_afterResamplingNoise);
	out << mrpt::format(
		"SOG_thresholdNegligible                 = %.03f\n",
		SOG_thresholdNegligible);
	out << mrpt::format(
		"SOG_maxDistBetweenGaussians             = %.03f\n",
		SOG_maxDistBetweenGaussians);
	out << mrpt::format(
		"SOG_separationConstant                  = %.03f\n",
		SOG_separationConstant);

	out << "\n";
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CBeaconMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
	MRPT_LOAD_CONFIG_VAR(insertAsMonteCarlo, bool, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(maxElevation_deg, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(minElevation_deg, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_numSamplesPerMeter, int, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_maxStdToGauss, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		MC_thresholdNegligible, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_performResampling, bool, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		MC_afterResamplingNoise, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		SOG_thresholdNegligible, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		SOG_maxDistBetweenGaussians, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		SOG_separationConstant, float, iniFile, section.c_str());
}

/*---------------------------------------------------------------
					 isEmpty
  ---------------------------------------------------------------*/
bool CBeaconMap::isEmpty() const { return size() == 0; }
/*---------------------------------------------------------------
					 simulateBeaconReadings
  ---------------------------------------------------------------*/
void CBeaconMap::simulateBeaconReadings(
	const CPose3D& in_robotPose, const CPoint3D& in_sensorLocationOnRobot,
	CObservationBeaconRanges& out_Observations) const
{
	TSequenceBeacons::const_iterator it;
	mrpt::obs::CObservationBeaconRanges::TMeasurement newMeas;
	CPoint3D point3D, beacon3D;
	CPointPDFGaussian beaconPDF;

	// Compute the 3D position of the sensor:
	point3D = in_robotPose + in_sensorLocationOnRobot;

	// Clear output data:
	out_Observations.sensedData.clear();

	// For each BEACON landmark in the map:
	for (it = m_beacons.begin(); it != m_beacons.end(); ++it)
	{
		it->getMean(beacon3D);

		float range = point3D.distanceTo(beacon3D);

		if (range < out_Observations.maxSensorDistance &&
			range > out_Observations.minSensorDistance)
		{
			// Add noise:
			range += getRandomGenerator().drawGaussian1D(
				0, out_Observations.stdError);

			// Fill out:
			newMeas.beaconID = it->m_ID;
			newMeas.sensorLocationOnRobot = in_sensorLocationOnRobot;
			newMeas.sensedDistance = range;

			// Insert:
			out_Observations.sensedData.push_back(newMeas);
		}
	}  // end for it
	// Done!
}

/*---------------------------------------------------------------
					 saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void CBeaconMap::saveMetricMapRepresentationToFile(
	const string& filNamePrefix) const
{
	MRPT_START

	// Matlab:
	string fil1(filNamePrefix + string("_3D.m"));
	saveToMATLABScript3D(fil1);

	// 3D Scene:
	opengl::COpenGLScene scene;
	opengl::CSetOfObjects::Ptr obj3D =
		std::make_shared<opengl::CSetOfObjects>();

	getAs3DObject(obj3D);
	auto objGround = opengl::CGridPlaneXY::Create(
		-100.0f, 100.0f, -100.0f, 100.0f, .0f, 1.f);

	scene.insert(obj3D);
	scene.insert(objGround);

	string fil2(filNamePrefix + string("_3D.3Dscene"));
	scene.saveToFile(fil2);

	// Textual representation:
	string fil3(filNamePrefix + string("_covs.txt"));
	saveToTextFile(fil3);

	// Total number of particles / modes:
	string fil4(filNamePrefix + string("_population.txt"));
	{
		FILE* f = os::fopen(fil4.c_str(), "wt");
		if (f)
		{
			size_t nParts = 0, nGaussians = 0;

			for (const auto& m_beacon : m_beacons)
			{
				switch (m_beacon.m_typePDF)
				{
					case CBeacon::pdfMonteCarlo:
						nParts += m_beacon.m_locationMC.size();
						break;
					case CBeacon::pdfSOG:
						nGaussians += m_beacon.m_locationSOG.size();
						break;
					case CBeacon::pdfGauss:
						nGaussians++;
						break;
				};
			}

			fprintf(
				f, "%u %u", static_cast<unsigned>(nParts),
				static_cast<unsigned>(nGaussians));
			os::fclose(f);
		}
	}

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
  ---------------------------------------------------------------*/
void CBeaconMap::getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START

	if (!genericMapParams.enableSaveAs3DObject) return;

	// ------------------------------------------------
	//  Add the XYZ corner for the current area:
	// ------------------------------------------------
	outObj->insert(opengl::stock_objects::CornerXYZ());

	// Save 3D ellipsoids or whatever representation:
	for (const auto& m_beacon : m_beacons) m_beacon.getAs3DObject(outObj);

	MRPT_END
}

/*---------------------------------------------------------------
   Computes the ratio in [0,1] of correspondences between "this" and the
 "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the
 maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching
 with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen
 from "this".
 * \param  maxDistForCorr			  [IN] The minimum distance between 2
 non-probabilistic map elements for counting them as a correspondence.
 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance
 between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
 ----------------------------------------------------------------*/
float CBeaconMap::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap2,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_START

	// Compare to a similar map only:
	const CBeaconMap* otherMap = nullptr;

	if (otherMap2->GetRuntimeClass() == CLASS_ID(CBeaconMap))
		otherMap = dynamic_cast<const CBeaconMap*>(otherMap2);

	if (!otherMap) return 0;

	TMatchingPairList matchList;
	vector<bool> otherCorrespondences;
	float out_corrsRatio;

	CBeaconMap modMap;

	modMap.changeCoordinatesReference(otherMapPose, otherMap);

	computeMatchingWith3DLandmarks(
		&modMap, matchList, out_corrsRatio, otherCorrespondences);

	return out_corrsRatio;

	MRPT_END
}

/*---------------------------------------------------------------
					getBeaconByID
 ---------------------------------------------------------------*/
const CBeacon* CBeaconMap::getBeaconByID(CBeacon::TBeaconID id) const
{
	for (const auto& m_beacon : m_beacons)
		if (m_beacon.m_ID == id) return &m_beacon;
	return nullptr;
}

/*---------------------------------------------------------------
					getBeaconByID
 ---------------------------------------------------------------*/
CBeacon* CBeaconMap::getBeaconByID(CBeacon::TBeaconID id)
{
	for (auto& m_beacon : m_beacons)
		if (m_beacon.m_ID == id) return &m_beacon;
	return nullptr;
}

/*---------------------------------------------------------------
					saveToTextFile
- VX VY VZ: Variances of each dimension (C11, C22, C33)
- DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
- C12, C13, C23: Cross covariances
 ---------------------------------------------------------------*/
void CBeaconMap::saveToTextFile(const string& fil) const
{
	MRPT_START
	FILE* f = os::fopen(fil.c_str(), "wt");
	ASSERT_(f != nullptr);

	for (const auto& m_beacon : m_beacons)
	{
		const auto [C, p] = m_beacon.getCovarianceAndMean();

		float D3 = C.det();
		float D2 = C(0, 0) * C(1, 1) - square(C(0, 1));
		os::fprintf(
			f, "%i %f %f %f %e %e %e %e %e %e %e %e\n",
			static_cast<int>(m_beacon.m_ID), p.x(), p.y(), p.z(), C(0, 0),
			C(1, 1), C(2, 2), D2, D3, C(0, 1), C(1, 2), C(1, 2));
	}

	os::fclose(f);
	MRPT_END
}
