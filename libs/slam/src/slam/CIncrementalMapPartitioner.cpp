/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/slam/observations_overlap.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/graphs/CGraphPartitioner.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CSimpleLine.h>

using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CIncrementalMapPartitioner, CSerializable, mrpt::slam)

static double eval_similarity_metric_map_matching(
	const CIncrementalMapPartitioner* parent, const map_keyframe_t& kf1,
	const map_keyframe_t& kf2, const mrpt::poses::CPose3D& relPose2wrt1)
{
	return kf1.metric_map->compute3DMatchingRatio(
		kf2.metric_map.get(), relPose2wrt1, parent->options.mrp);
}
static double eval_similarity_observation_overlap(
	const map_keyframe_t& kf1, const map_keyframe_t& kf2,
	const mrpt::poses::CPose3D& relPose2wrt1)
{
	return observationsOverlap(
		kf1.raw_observations, kf2.raw_observations, &relPose2wrt1);
}

CIncrementalMapPartitioner::TOptions::TOptions()
{
	CSimplePointsMap::TMapDefinition def;
	metricmap.push_back(def);
}

void CIncrementalMapPartitioner::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const string& section)
{
	MRPT_START

	MRPT_LOAD_CONFIG_VAR(partitionThreshold, double, source, section);
	MRPT_LOAD_CONFIG_VAR(forceBisectionOnly, bool, source, section);
	MRPT_LOAD_CONFIG_VAR(simil_method, enum, source, section);
	MRPT_LOAD_CONFIG_VAR(
		minimumNumberElementsEachCluster, uint64_t, source, section);
	MRPT_LOAD_HERE_CONFIG_VAR(
		"minDistForCorrespondence", double, mrp.maxDistForCorr, source,
		section);
	MRPT_LOAD_HERE_CONFIG_VAR(
		"minMahaDistForCorrespondence", double, mrp.maxMahaDistForCorr, source,
		section);
	MRPT_LOAD_CONFIG_VAR(maxKeyFrameDistanceToEval, uint64_t, source, section);

	mrpt::config::CConfigFilePrefixer cfp(
		source, section + std::string("."), "");
	metricmap.loadFromConfigFile(cfp, "metricmap");
	MRPT_TODO("Add link to example INI file");

	MRPT_END
}

void CIncrementalMapPartitioner::TOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		partitionThreshold, "N-cut partition threshold [0,2]");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		forceBisectionOnly,
		"Force bisection (true) or automatically determine number of "
		"partitions(false = default)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(simil_method, "Similarity method");
	MRPT_SAVE_CONFIG_VAR_COMMENT(minimumNumberElementsEachCluster, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		maxKeyFrameDistanceToEval, "Max KF ID distance");
	c.write(
		s, "minDistForCorrespondence", mrp.maxDistForCorr,
		mrpt::config::MRPT_SAVE_NAME_PADDING(),
		mrpt::config::MRPT_SAVE_VALUE_PADDING());
	c.write(
		s, "minMahaDistForCorrespondence", mrp.maxMahaDistForCorr,
		mrpt::config::MRPT_SAVE_NAME_PADDING(),
		mrpt::config::MRPT_SAVE_VALUE_PADDING());

	mrpt::config::CConfigFilePrefixer cfp(c, s + std::string("."), "");
	metricmap.saveToConfigFile(cfp, "metricmap");
}

void CIncrementalMapPartitioner::clear()
{
	m_last_last_partition_are_new_ones = false;
	m_A.setSize(0, 0);
	m_individualFrames.clear();  // Free the map...
	m_individualMaps.clear();
	m_last_partition.clear();  // Delete last partitions
}

uint32_t CIncrementalMapPartitioner::addMapFrame(
	const mrpt::obs::CSensoryFrame& frame,
	const mrpt::poses::CPose3DPDF& robotPose)
{
	MRPT_START

	const uint32_t new_id = m_individualMaps.size();
	const size_t n = new_id + 1;  // new size

	// Create new new metric map:
	m_individualMaps.push_back(CMultiMetricMap::Create());
	auto& newMetricMap = m_individualMaps.back();
	newMetricMap->setListOfMaps(&options.metricmap);

	// Build robo-centric map for each keyframe:
	frame.insertObservationsInto(newMetricMap.get());

	// Add tuple (pose,SF) to "simplemap":
	m_individualFrames.insert(&robotPose, frame);

	// Expand the adjacency matrix (pads with 0)
	m_A.setSize(n, n);

	ASSERT_(m_individualMaps.size() == n);
	ASSERT_(m_individualFrames.size() == n);

	// Select method to evaluate similarity:
	similarity_func_t sim_func;
	using namespace std::placeholders;  // for _1, _2 etc.
	switch (options.simil_method)
	{
		case smMETRIC_MAP_MATCHING:
			sim_func = std::bind(
				&eval_similarity_metric_map_matching, this, _1, _2, _3);
			break;
		case smOBSERVATION_OVERLAP:
			sim_func = &eval_similarity_observation_overlap;
			break;
		case smCUSTOM_FUNCTION:
			sim_func = m_sim_func;
			break;
		default:
			THROW_EXCEPTION("Invalid value for `simil_method`");
	};

	// Evaluate the similarity metric for the last row & column:
	// (0:new_id, new_id)  and (new_id, 0:new_id)
	// ----------------------------------------------------------------
	{
		auto i = new_id;

		// KF "i":
		map_keyframe_t map_i;
		map_i.kf_id = i;
		map_i.metric_map = m_individualMaps[i];
		CPose3DPDF::Ptr posePDF_i;
		m_individualFrames.get(i, posePDF_i, map_i.raw_observations);
		auto pose_i = posePDF_i->getMeanVal();

		for (uint32_t j = 0; j < new_id; j++)
		{
			const auto id_diff = new_id - j;
			double s_sym;
			if (id_diff > options.maxKeyFrameDistanceToEval)
			{
				// skip evaluation
				s_sym = .0;
			}
			else
			{
				// KF "j":
				map_keyframe_t map_j;
				CPose3DPDF::Ptr posePDF_j;
				map_j.kf_id = j;
				m_individualFrames.get(j, posePDF_j, map_j.raw_observations);
				auto pose_j = posePDF_j->getMeanVal();
				map_j.metric_map = m_individualMaps[j];

				auto relPose = pose_j - pose_i;

				// Evaluate similarity metric & make it symetric:
				const auto s_ij = sim_func(map_i, map_j, relPose);
				const auto s_ji = sim_func(map_j, map_i, relPose);
				s_sym = 0.5 * (s_ij + s_ji);
			}
			m_A(i, j) = m_A(j, i) = s_sym;
		}  // for j
	}  // i=n-1=new_id

	// Self-similatity: Not used
	m_A(new_id, new_id) = 0;

	// If a partition has been already computed, add these new keyframes
	// into a new partition on its own. When the user calls updatePartitions()
	// all keyframes will be re-distributed according to the real similarity
	// scores.
	if (m_last_last_partition_are_new_ones)
	{
		// Insert into the "new_ones" partition:
		m_last_partition[m_last_partition.size() - 1].push_back(n - 1);
	}
	else
	{
		// Add a new partition:
		std::vector<uint32_t> dummyPart;
		dummyPart.push_back(n - 1);
		m_last_partition.emplace_back(dummyPart);

		// The last one is the new_ones partition:
		m_last_last_partition_are_new_ones = true;
	}

	return n - 1;  // Index of the new node

	MRPT_END
}

void CIncrementalMapPartitioner::updatePartitions(
	vector<std::vector<uint32_t>>& partitions)
{
	MRPT_START

	partitions.clear();
	CGraphPartitioner<CMatrixD>::RecursiveSpectralPartition(
		m_A, partitions, options.partitionThreshold, true, true,
		!options.forceBisectionOnly, options.minimumNumberElementsEachCluster,
		false /* verbose */
	);

	m_last_partition = partitions;
	m_last_last_partition_are_new_ones = false;

	MRPT_END
}

size_t CIncrementalMapPartitioner::getNodesCount()
{
	return m_individualFrames.size();
}

void CIncrementalMapPartitioner::removeSetOfNodes(
	std::vector<uint32_t> indexesToRemove, bool changeCoordsRef)
{
	MRPT_START

	size_t nOld = m_A.cols();
	size_t nNew = nOld - indexesToRemove.size();
	size_t i, j;

	// Assure indexes are sorted:
	std::sort(indexesToRemove.begin(), indexesToRemove.end());

	ASSERT_(nNew >= 1);

	// Build the vector with the nodes that REMAINS;
	std::vector<uint32_t> indexesToStay;
	indexesToStay.reserve(nNew);
	for (i = 0; i < nOld; i++)
	{
		bool remov = false;
		for (j = 0; !remov && j < indexesToRemove.size(); j++)
		{
			if (indexesToRemove[j] == i) remov = true;
		}

		if (!remov) indexesToStay.push_back(i);
	}

	ASSERT_(indexesToStay.size() == nNew);

	// Update the A matrix:
	// ---------------------------------------------------
	CMatrixDouble newA(nNew, nNew);
	for (i = 0; i < nNew; i++)
		for (j = 0; j < nNew; j++)
			newA(i, j) = m_A(indexesToStay[i], indexesToStay[j]);

	// Substitute "A":
	m_A = newA;

	// The last partitioning is all the nodes together:
	// --------------------------------------------------
	m_last_partition.resize(1);
	m_last_partition[0].resize(nNew);
	for (i = 0; i < nNew; i++) m_last_partition[0][i] = i;

	m_last_last_partition_are_new_ones = false;

	// The new sequence of maps:
	// --------------------------------------------------
	for (auto it = indexesToRemove.rbegin(); it != indexesToRemove.rend(); ++it)
	{
		auto itM = m_individualMaps.begin() + *it;
		m_individualMaps.erase(itM);  // Delete from list
	}

	// The new sequence of localized SFs:
	// --------------------------------------------------
	for (auto it = indexesToRemove.rbegin(); it != indexesToRemove.rend(); ++it)
		m_individualFrames.remove(*it);

	// Change coordinates reference of frames:
	CSensoryFrame::Ptr SF;
	CPose3DPDF::Ptr posePDF;

	if (changeCoordsRef)
	{
		ASSERT_(m_individualFrames.size() > 0);
		m_individualFrames.get(0, posePDF, SF);

		CPose3D p;
		posePDF->getMean(p);
		m_individualFrames.changeCoordinatesOrigin(p);
	}

	// All done!
	MRPT_END
}

void CIncrementalMapPartitioner::changeCoordinatesOrigin(
	const CPose3D& newOrigin)
{
	m_individualFrames.changeCoordinatesOrigin(newOrigin);
}

void CIncrementalMapPartitioner::changeCoordinatesOriginPoseIndex(
	unsigned int newOriginPose)
{
	MRPT_START

	CPose3DPDF::Ptr pdf;
	CSensoryFrame::Ptr sf;
	m_individualFrames.get(newOriginPose, pdf, sf);

	CPose3D p;
	pdf->getMean(p);
	changeCoordinatesOrigin(-p);

	MRPT_END
}

void CIncrementalMapPartitioner::getAs3DScene(
	mrpt::opengl::CSetOfObjects::Ptr& objs,
	const std::map<uint32_t, int64_t>* renameIndexes) const
{
	objs->clear();
	ASSERT_((int)m_individualFrames.size() == m_A.cols());

	auto gl_grid = opengl::CGridPlaneXY::Create();
	objs->insert(gl_grid);
	int bbminx = std::numeric_limits<int>::max(),
		bbminy = std::numeric_limits<int>::max();
	int bbmaxx = -bbminx, bbmaxy = -bbminy;

	for (size_t i = 0; i < m_individualFrames.size(); i++)
	{
		CPose3DPDF::Ptr i_pdf;
		CSensoryFrame::Ptr i_sf;
		m_individualFrames.get(i, i_pdf, i_sf);

		CPose3D i_mean;
		i_pdf->getMean(i_mean);

		mrpt::keep_min(bbminx, (int)floor(i_mean.x()));
		mrpt::keep_min(bbminy, (int)floor(i_mean.y()));
		mrpt::keep_max(bbmaxx, (int)ceil(i_mean.x()));
		mrpt::keep_max(bbmaxy, (int)ceil(i_mean.y()));

		opengl::CSphere::Ptr i_sph =
			mrpt::make_aligned_shared<opengl::CSphere>();
		i_sph->setRadius(0.02f);
		i_sph->setColor(0, 0, 1);

		if (!renameIndexes)
			i_sph->setName(format("%u", static_cast<unsigned int>(i)));
		else
		{
			auto itName = renameIndexes->find(i);
			ASSERT_(itName != renameIndexes->end());
			i_sph->setName(
				format("%lu", static_cast<unsigned long>(itName->second)));
		}

		i_sph->enableShowName();
		i_sph->setPose(i_mean);

		objs->insert(i_sph);

		// Arcs:
		for (size_t j = i + 1; j < m_individualFrames.size(); j++)
		{
			CPose3DPDF::Ptr j_pdf;
			CSensoryFrame::Ptr j_sf;
			m_individualFrames.get(j, j_pdf, j_sf);

			CPose3D j_mean;
			j_pdf->getMean(j_mean);

			float SSO_ij = m_A(i, j);

			if (SSO_ij > 0.01)
			{
				opengl::CSimpleLine::Ptr lin =
					mrpt::make_aligned_shared<opengl::CSimpleLine>();
				lin->setLineCoords(
					i_mean.x(), i_mean.y(), i_mean.z(), j_mean.x(), j_mean.y(),
					j_mean.z());

				lin->setColor(SSO_ij, 0, 1 - SSO_ij, SSO_ij * 0.6);
				lin->setLineWidth(SSO_ij * 10);

				objs->insert(lin);
			}
		}
	}
	gl_grid->setPlaneLimits(bbminx, bbmaxx, bbminy, bbmaxy);
	gl_grid->setGridFrequency(5);
}

void CIncrementalMapPartitioner::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			in >> m_individualFrames >> m_individualMaps >> m_A >>
				m_last_partition >> m_last_last_partition_are_new_ones;
			if (version == 0)
			{
				// field removed in v1
				std::vector<uint8_t> old_modified_nodes;
				in >> old_modified_nodes;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

uint8_t CIncrementalMapPartitioner::serializeGetVersion() const { return 1; }
void CIncrementalMapPartitioner::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << m_individualFrames << m_individualMaps << m_A << m_last_partition
		<< m_last_last_partition_are_new_ones;
}
