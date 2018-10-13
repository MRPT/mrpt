/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/io/CFileStream.h>
#include <mrpt/system/os.h>

#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CLandmarksMap.h>

#include <mrpt/slam/PF_aux_structs.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CMultiMetricMapPDF, CSerializable, mrpt::maps)
IMPLEMENTS_SERIALIZABLE(CRBPFParticleData, CSerializable, mrpt::maps)

/*---------------------------------------------------------------
				Constructor
  ---------------------------------------------------------------*/
CMultiMetricMapPDF::CMultiMetricMapPDF(
	const bayes::CParticleFilter::TParticleFilterOptions& opts,
	const mrpt::maps::TSetOfMetricMapInitializers* mapsInitializers,
	const TPredictionParams* predictionOptions)
	: averageMap(mapsInitializers),

	  SFs(),
	  SF2robotPath(),
	  options()

{
	m_particles.resize(opts.sampleSize);
	for (auto& m_particle : m_particles)
	{
		m_particle.log_w = 0;
		m_particle.d.reset(new CRBPFParticleData(mapsInitializers));
	}

	// Initialize:
	const CPose3D nullPose(0, 0, 0);
	clear(nullPose);

	// If provided, copy the whole set of params now:
	if (predictionOptions != nullptr) options = *predictionOptions;
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CMultiMetricMapPDF::clear(const CPose2D& initialPose)
{
	CPose3D p(initialPose);
	clear(p);
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CMultiMetricMapPDF::clear(const CPose3D& initialPose)
{
	const size_t M = m_particles.size();
	for (size_t i = 0; i < M; i++)
	{
		m_particles[i].log_w = 0;

		m_particles[i].d->mapTillNow.clear();

		m_particles[i].d->robotPath.resize(1);
		m_particles[i].d->robotPath[0] = initialPose.asTPose();
	}

	SFs.clear();
	SF2robotPath.clear();

	averageMapIsUpdated = false;
}

void CMultiMetricMapPDF::clear(
	const mrpt::maps::CSimpleMap& prevMap,
	const mrpt::poses::CPose3D& currentPose)
{
	const size_t nParts = m_particles.size(), nOldKeyframes = prevMap.size();
	if (nOldKeyframes == 0)
	{
		// prevMap is empty, so reset the map
		clear(currentPose);
		return;
	}
	for (size_t idxPart = 0; idxPart < nParts; idxPart++)
	{
		auto& p = m_particles[idxPart];
		p.log_w = 0;

		p.d->mapTillNow.clear();

		p.d->robotPath.resize(nOldKeyframes);
		for (size_t i = 0; i < nOldKeyframes; i++)
		{
			CPose3DPDF::Ptr keyframe_pose;
			CSensoryFrame::Ptr sfkeyframe_sf;
			prevMap.get(i, keyframe_pose, sfkeyframe_sf);

			// as pose, use: if the PDF is also a PF with the same number of
			// samples, use those particles;
			// otherwise, simply use the mean for all particles as an
			// approximation (with loss of uncertainty).
			mrpt::poses::CPose3D kf_pose;
			bool kf_pose_set = false;
			if (IS_CLASS(keyframe_pose, CPose3DPDFParticles))
			{
				const auto pdf_parts = dynamic_cast<const CPose3DPDFParticles*>(
					keyframe_pose.get());
				ASSERT_(pdf_parts);
				if (pdf_parts->particlesCount() == nParts)
				{
					kf_pose = CPose3D(pdf_parts->m_particles[idxPart].d);
					kf_pose_set = true;
				}
			}
			if (!kf_pose_set)
			{
				kf_pose = keyframe_pose->getMeanVal();
			}
			p.d->robotPath[i] = kf_pose.asTPose();
			for (const auto& obs : *sfkeyframe_sf)
			{
				p.d->mapTillNow.insertObservation(&(*obs), &kf_pose);
			}
		}
	}

	SFs = prevMap;  // copy
	SF2robotPath.clear();
	SF2robotPath.reserve(nOldKeyframes);
	for (size_t i = 0; i < nOldKeyframes; i++) SF2robotPath.push_back(i);

	averageMapIsUpdated = false;
}

/*---------------------------------------------------------------
						getEstimatedPosePDF
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF), computed
		as a weighted average over all m_particles.
 ---------------------------------------------------------------*/
void CMultiMetricMapPDF::getEstimatedPosePDF(
	CPose3DPDFParticles& out_estimation) const
{
	ASSERT_(m_particles[0].d->robotPath.size() > 0);
	getEstimatedPosePDFAtTime(
		m_particles[0].d->robotPath.size() - 1, out_estimation);
}

/*---------------------------------------------------------------
						getEstimatedPosePDFAtTime
 ---------------------------------------------------------------*/
void CMultiMetricMapPDF::getEstimatedPosePDFAtTime(
	size_t timeStep, CPose3DPDFParticles& out_estimation) const
{
	// CPose3D	p;
	size_t i, n = m_particles.size();

	// Delete current content of "out_estimation":
	out_estimation.clearParticles();

	// Create new m_particles:
	out_estimation.m_particles.resize(n);
	for (i = 0; i < n; i++)
	{
		out_estimation.m_particles[i].d = m_particles[i].d->robotPath[timeStep];
		out_estimation.m_particles[i].log_w = m_particles[i].log_w;
	}
}

uint8_t CRBPFParticleData::serializeGetVersion() const { return 0; }
void CRBPFParticleData::serializeTo(mrpt::serialization::CArchive&) const
{
	THROW_EXCEPTION("Shouldn't arrive here");
}
void CRBPFParticleData::serializeFrom(mrpt::serialization::CArchive&, uint8_t)
{
	THROW_EXCEPTION("Shouldn't arrive here");
}

uint8_t CMultiMetricMapPDF::serializeGetVersion() const { return 0; }
void CMultiMetricMapPDF::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_particles.size());
	for (const auto& part : m_particles)
	{
		out << part.log_w << part.d->mapTillNow;
		out.WriteAs<uint32_t>(part.d->robotPath.size());
		for (const auto& p : part.d->robotPath) out << p;
	}
	out << SFs << SF2robotPath;
}

void CMultiMetricMapPDF::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t i, n, j, m;

			// Delete current contents:
			// --------------------------
			clearParticles();
			SFs.clear();
			SF2robotPath.clear();

			averageMapIsUpdated = false;

			// Load the new data:
			// --------------------
			in >> n;

			m_particles.resize(n);
			for (i = 0; i < n; i++)
			{
				m_particles[i].d.reset(new CRBPFParticleData());

				// Load
				in >> m_particles[i].log_w >> m_particles[i].d->mapTillNow;

				in >> m;
				m_particles[i].d->robotPath.resize(m);
				for (j = 0; j < m; j++) in >> m_particles[i].d->robotPath[j];
			}

			in >> SFs >> SF2robotPath;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

TPose3D CMultiMetricMapPDF::getLastPose(
	const size_t i, bool& is_valid_pose) const
{
	if (i >= m_particles.size())
		THROW_EXCEPTION("Particle index out of bounds!");

	if (m_particles[i].d->robotPath.empty())
	{
		is_valid_pose = false;
		return TPose3D(0, 0, 0, 0, 0, 0);
	}
	else
	{
		return *m_particles[i].d->robotPath.rbegin();
	}
}

const CMultiMetricMap* CMultiMetricMapPDF::getAveragedMetricMapEstimation()
{
	rebuildAverageMap();
	return &averageMap;
}

/*---------------------------------------------------------------
						getWeightedAveragedMap
 ---------------------------------------------------------------*/
void CMultiMetricMapPDF::rebuildAverageMap()
{
	float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
	CParticleList::iterator part;

	if (averageMapIsUpdated) return;

	// ---------------------------------------------------------
	//					GRID
	// ---------------------------------------------------------
	for (part = m_particles.begin(); part != m_particles.end(); ++part)
	{
		ASSERT_(part->d->mapTillNow.m_gridMaps.size() > 0);

		min_x = min(min_x, part->d->mapTillNow.m_gridMaps[0]->getXMin());
		max_x = max(max_x, part->d->mapTillNow.m_gridMaps[0]->getXMax());
		min_y = min(min_y, part->d->mapTillNow.m_gridMaps[0]->getYMin());
		max_y = max(max_y, part->d->mapTillNow.m_gridMaps[0]->getYMax());
	}

	// Asure all maps have the same dimensions:
	for (part = m_particles.begin(); part != m_particles.end(); ++part)
		part->d->mapTillNow.m_gridMaps[0]->resizeGrid(
			min_x, max_x, min_y, max_y, 0.5f, false);

	for (part = m_particles.begin(); part != m_particles.end(); ++part)
	{
		min_x = min(min_x, part->d->mapTillNow.m_gridMaps[0]->getXMin());
		max_x = max(max_x, part->d->mapTillNow.m_gridMaps[0]->getXMax());
		min_y = min(min_y, part->d->mapTillNow.m_gridMaps[0]->getYMin());
		max_y = max(max_y, part->d->mapTillNow.m_gridMaps[0]->getYMax());
	}

	// Prepare target map:
	ASSERT_(averageMap.m_gridMaps.size() > 0);
	averageMap.m_gridMaps[0]->setSize(
		min_x, max_x, min_y, max_y,
		m_particles[0].d->mapTillNow.m_gridMaps[0]->getResolution(), 0);

	// Compute the sum of weights:
	double sumLinearWeights = 0;
	for (part = m_particles.begin(); part != m_particles.end(); ++part)
		sumLinearWeights += exp(part->log_w);

	// CHECK:
	for (part = m_particles.begin(); part != m_particles.end(); ++part)
	{
		ASSERT_(
			part->d->mapTillNow.m_gridMaps[0]->getSizeX() ==
			averageMap.m_gridMaps[0]->getSizeX());
		ASSERT_(
			part->d->mapTillNow.m_gridMaps[0]->getSizeY() ==
			averageMap.m_gridMaps[0]->getSizeY());
	}

	{
		// ******************************************************
		//     Implementation WITHOUT the SSE Instructions Set
		// ******************************************************
		MRPT_START

		// Reserve a float grid-map, add weight all maps
		// -------------------------------------------------------------------------------------------
		std::vector<float> floatMap;
		floatMap.resize(averageMap.m_gridMaps[0]->map.size(), 0);

		// For each particle in the RBPF:
		double sumW = 0;
		for (part = m_particles.begin(); part != m_particles.end(); ++part)
			sumW += exp(part->log_w);

		if (sumW == 0) sumW = 1;

		for (part = m_particles.begin(); part != m_particles.end(); ++part)
		{
			// Variables:
			std::vector<COccupancyGridMap2D::cellType>::iterator srcCell;
			auto firstSrcCell = part->d->mapTillNow.m_gridMaps[0]->map.begin();
			auto lastSrcCell = part->d->mapTillNow.m_gridMaps[0]->map.end();
			std::vector<float>::iterator destCell;

			// The weight of particle:
			float w = exp(part->log_w) / sumW;

			ASSERT_(
				part->d->mapTillNow.m_gridMaps[0]->map.size() ==
				floatMap.size());

			// For each cell in individual maps:
			for (srcCell = firstSrcCell, destCell = floatMap.begin();
				 srcCell != lastSrcCell; srcCell++, destCell++)
				(*destCell) += w * (*srcCell);
		}

		// Copy to fixed point map:
		std::vector<float>::iterator srcCell;
		auto destCell = averageMap.m_gridMaps[0]->map.begin();

		ASSERT_(averageMap.m_gridMaps[0]->map.size() == floatMap.size());

		for (srcCell = floatMap.begin(); srcCell != floatMap.end();
			 srcCell++, destCell++)
			*destCell = static_cast<COccupancyGridMap2D::cellType>(*srcCell);

		MRPT_END
	}  // End of SSE not supported

	// Don't calculate again until really necesary.
	averageMapIsUpdated = true;
}

/*---------------------------------------------------------------
						insertObservation
 ---------------------------------------------------------------*/
bool CMultiMetricMapPDF::insertObservation(CSensoryFrame& sf)
{
	const size_t M = particlesCount();

	// Insert into SFs:
	CPose3DPDFParticles::Ptr posePDF =
		mrpt::make_aligned_shared<CPose3DPDFParticles>();
	getEstimatedPosePDF(*posePDF);

	// Insert it into the SFs and the SF2robotPath list:
	const uint32_t new_sf_id = SFs.size();
	SFs.insert(posePDF, CSensoryFrame::Create(sf));
	SF2robotPath.resize(new_sf_id + 1);
	SF2robotPath[new_sf_id] = m_particles[0].d->robotPath.size() - 1;

	bool anymap = false;
	for (size_t i = 0; i < M; i++)
	{
		bool pose_is_valid;
		const CPose3D robotPose = CPose3D(getLastPose(i, pose_is_valid));
		// ASSERT_(pose_is_valid); // if not, use the default (0,0,0)
		const bool map_modified = sf.insertObservationsInto(
			&m_particles[i].d->mapTillNow, &robotPose);
		anymap = anymap || map_modified;
	}

	averageMapIsUpdated = false;
	return anymap;
}

/*---------------------------------------------------------------
						getPath
 ---------------------------------------------------------------*/
void CMultiMetricMapPDF::getPath(
	size_t i, std::deque<math::TPose3D>& out_path) const
{
	if (i >= m_particles.size()) THROW_EXCEPTION("Index out of bounds");
	out_path = m_particles[i].d->robotPath;
}

/*---------------------------------------------------------------
					getCurrentEntropyOfPaths
  ---------------------------------------------------------------*/
double CMultiMetricMapPDF::getCurrentEntropyOfPaths()
{
	size_t i;
	size_t N =
		m_particles[0].d->robotPath.size();  // The poses count along the paths

	// Compute paths entropy:
	// ---------------------------
	double H_paths = 0;

	if (N)
	{
		// For each pose along the path:
		for (i = 0; i < N; i++)
		{
			// Get pose est. as m_particles:
			CPose3DPDFParticles posePDFParts;
			getEstimatedPosePDFAtTime(i, posePDFParts);

			// Approximate to gaussian and compute entropy of covariance:
			H_paths += posePDFParts.getCovarianceEntropy();
		}
		H_paths /= N;
	}
	return H_paths;
}

/*---------------------------------------------------------------
					getCurrentJointEntropy
  ---------------------------------------------------------------*/
double CMultiMetricMapPDF::getCurrentJointEntropy()
{
	double H_joint, H_paths, H_maps;
	size_t i, M = m_particles.size();
	COccupancyGridMap2D::TEntropyInfo entropy;

	// Entropy of the paths:
	H_paths = getCurrentEntropyOfPaths();

	float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
	CParticleList::iterator part;

	// ---------------------------------------------------------
	//			ASSURE ALL THE GRIDS ARE THE SAME SIZE!
	// ---------------------------------------------------------
	for (part = m_particles.begin(); part != m_particles.end(); ++part)
	{
		ASSERT_(part->d->mapTillNow.m_gridMaps.size() > 0);

		min_x = min(min_x, part->d->mapTillNow.m_gridMaps[0]->getXMin());
		max_x = max(max_x, part->d->mapTillNow.m_gridMaps[0]->getXMax());
		min_y = min(min_y, part->d->mapTillNow.m_gridMaps[0]->getYMin());
		max_y = max(max_y, part->d->mapTillNow.m_gridMaps[0]->getYMax());
	}

	// Asure all maps have the same dimensions:
	for (part = m_particles.begin(); part != m_particles.end(); ++part)
		part->d->mapTillNow.m_gridMaps[0]->resizeGrid(
			min_x, max_x, min_y, max_y, 0.5f, false);

	// Sum of linear weights:
	double sumLinearWeights = 0;
	for (i = 0; i < M; i++) sumLinearWeights += exp(m_particles[i].log_w);

	// Compute weighted maps entropy:
	// --------------------------------
	H_maps = 0;
	for (i = 0; i < M; i++)
	{
		ASSERT_(m_particles[i].d->mapTillNow.m_gridMaps.size() > 0);

		m_particles[i].d->mapTillNow.m_gridMaps[0]->computeEntropy(entropy);
		H_maps += exp(m_particles[i].log_w) * entropy.H / sumLinearWeights;
	}

	printf("H_paths=%e\n", H_paths);
	printf("H_maps=%e\n", H_maps);

	H_joint = H_paths + H_maps;
	return H_joint;
}

const CMultiMetricMap* CMultiMetricMapPDF::getCurrentMostLikelyMetricMap() const
{
	size_t i, max_i = 0, n = m_particles.size();
	double max_w = m_particles[0].log_w;

	for (i = 0; i < n; i++)
	{
		if (m_particles[i].log_w > max_w)
		{
			max_w = m_particles[i].log_w;
			max_i = i;
		}
	}

	// Return its map:
	return &m_particles[max_i].d->mapTillNow;
}

/*---------------------------------------------------------------
				updateSensoryFrameSequence
  ---------------------------------------------------------------*/
void CMultiMetricMapPDF::updateSensoryFrameSequence()
{
	MRPT_START
	CPose3DPDFParticles posePartsPDF;
	CPose3DPDF::Ptr previousPosePDF;
	CSensoryFrame::Ptr dummy;

	for (size_t i = 0; i < SFs.size(); i++)
	{
		// Get last estimation:
		SFs.get(i, previousPosePDF, dummy);

		// Compute the new one:
		getEstimatedPosePDFAtTime(SF2robotPath[i], posePartsPDF);

		// Copy into SFs:
		previousPosePDF->copyFrom(posePartsPDF);
	}

	MRPT_END
}

/*---------------------------------------------------------------
				saveCurrentPathEstimationToTextFile
  ---------------------------------------------------------------*/
void CMultiMetricMapPDF::saveCurrentPathEstimationToTextFile(
	const std::string& fil)
{
	FILE* f = os::fopen(fil.c_str(), "wt");
	if (!f) return;

	for (auto& m_particle : m_particles)
	{
		for (size_t i = 0; i < m_particle.d->robotPath.size(); i++)
		{
			const mrpt::math::TPose3D& p = m_particle.d->robotPath[i];

			os::fprintf(
				f, "%.04f %.04f %.04f %.04f %.04f %.04f ", p.x, p.y, p.z, p.yaw,
				p.pitch, p.roll);
		}
		os::fprintf(f, " %e\n", m_particle.log_w);
	}

	os::fclose(f);
}

/*---------------------------------------------------------------
				TPredictionParams
  ---------------------------------------------------------------*/
CMultiMetricMapPDF::TPredictionParams::TPredictionParams()
	: update_gridMapLikelihoodOptions(), KLD_params(), icp_params()
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void CMultiMetricMapPDF::TPredictionParams::dumpToTextStream(
	std::ostream& out) const
{
	out << mrpt::format(
		"\n----------- [CMultiMetricMapPDF::TPredictionParams] ------------ "
		"\n\n");

	out << mrpt::format(
		"pfOptimalProposal_mapSelection          = %i\n",
		pfOptimalProposal_mapSelection);
	out << mrpt::format(
		"ICPGlobalAlign_MinQuality               = %f\n",
		ICPGlobalAlign_MinQuality);

	KLD_params.dumpToTextStream(out);
	icp_params.dumpToTextStream(out);
	out << mrpt::format("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CMultiMetricMapPDF::TPredictionParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	pfOptimalProposal_mapSelection = iniFile.read_int(
		section, "pfOptimalProposal_mapSelection",
		pfOptimalProposal_mapSelection, true);

	MRPT_LOAD_CONFIG_VAR(ICPGlobalAlign_MinQuality, float, iniFile, section);

	KLD_params.loadFromConfigFile(iniFile, section);
	icp_params.loadFromConfigFile(iniFile, section);
}
