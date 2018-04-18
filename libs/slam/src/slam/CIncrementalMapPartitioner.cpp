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
	const CIncrementalMapPartitioner *parent,
	const mrpt::maps::CMultiMetricMap &m1,
	const mrpt::maps::CMultiMetricMap &m2,
	const mrpt::obs::CSensoryFrame &sf1,
	const mrpt::obs::CSensoryFrame &sf2,
	uint32_t id_kf1, uint32_t id_kf2,
	const mrpt::poses::CPose3D &relPose2wrt1
)
{
	return m1.compute3DMatchingRatio(&m2, relPose2wrt1, parent->options.mrp);
}
static double eval_similarity_observation_overlap(
	const mrpt::maps::CMultiMetricMap &m1,
	const mrpt::maps::CMultiMetricMap &m2,
	const mrpt::obs::CSensoryFrame &sf1,
	const mrpt::obs::CSensoryFrame &sf2,
	uint32_t id_kf1, uint32_t id_kf2,
	const mrpt::poses::CPose3D &relPose2wrt1
)
{
	return observationsOverlap(sf1, sf2, &relPose2wrt1);
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
		"minDistForCorrespondence", double, mrp.maxDistForCorr,
		source, section);
	MRPT_LOAD_HERE_CONFIG_VAR(
		"minMahaDistForCorrespondence", double, mrp.maxMahaDistForCorr,
		source, section);

	mrpt::config::CConfigFilePrefixer cfp(source, section + std::string("."), "");
	metricmap.loadFromConfigFile(cfp, "metricmap");
	MRPT_TODO("Add link to example INI file");

	MRPT_END
}

void CIncrementalMapPartitioner::TOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c,
	const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(partitionThreshold, "N-cut partition threshold [0,2]");
	MRPT_SAVE_CONFIG_VAR_COMMENT(forceBisectionOnly, "Force bisection (true) or automatically determine number of partitions(false = default)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(simil_method, "Similarity method");
	MRPT_SAVE_CONFIG_VAR_COMMENT(minimumNumberElementsEachCluster, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(, "");

	MRPT_TODO("Write me");
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

	const size_t n = m_individualMaps.size() + 1; // new size

	// Create new new metric map:
	m_individualMaps.resize(n);
	CMultiMetricMap& newMetricMap = m_individualMaps.back();
	newMetricMap.setListOfMaps(&options.metricmap);

	// Build robo-centric map for each keyframe:
	frame.insertObservationsInto(&newMetricMap);

	// Add tuple (pose,SF) to "simplemap":
	m_individualFrames.insert(&robotPose, frame);

	// Expand the adjacency matrix (pads with 0)
	m_A.setSize(n, n);

	ASSERT_(m_individualMaps.size() == n);
	ASSERT_(m_individualFrames.size() == n);

	// Select method to evaluate similarity:
	similarity_func_t sim_func;
	using namespace std::placeholders; // for _1, _2 etc.
	switch (options.simil_method)
	{
	case smMETRIC_MAP_MATCHING:
		sim_func = std::bind(&eval_similarity_metric_map_matching, this, _1, _2, _3, _4, _5, _6, _7);
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

	MRPT_TODO("unify in one loop:");
	// Calculate the new matches - put them in the matrix
	// ----------------------------------------------------------------
	size_t i = 0, j = 0, n = 0;
	// for (i=n-1;i<n;i++)
	i = n - 1;  // Execute procedure until "i=n-1"; Last row/column is empty
	{
		CPose3DPDF::Ptr posePDF_i, posePDF_j;
		CSensoryFrame::Ptr sf_i, sf_j;

		// Get node "i":
		m_individualFrames.get(i, posePDF_i, sf_i);
		auto pose_i = posePDF_i->getMeanVal();

		// And its points map:
		CMultiMetricMap &map_i = m_individualMaps[i];

		for (j = 0; j < n - 1; j++)
		{
			// Get node "j":
			m_individualFrames.get(j, posePDF_j, sf_j);
			auto pose_j = posePDF_j->getMeanVal();
			auto relPose = pose_j - pose_i;

			// And its points map:
			CMultiMetricMap &map_j = m_individualMaps[j];

			// Compute matching ratio:
			m_A(i, j) = sim_func(map_i, map_j, *sf_i, *sf_j, i, j, relPose);

		}  // for j

	}  // for i

	for (i = 0; i < n - 1;
		 i++)  // Execute procedure until "i=n-1"; Last row/column is empty
	{
		// Get node "i":
		m_individualFrames.get(i, posePDF_i, sf_i);
		posePDF_i->getMean(pose_i);

		// And its points map:
		CMultiMetricMap *map_i = &m_individualMaps[i];

		j = n - 1;  // for (j=n-1;j<n;j++)
		{
			// Get node "j":
			m_individualFrames.get(j, posePDF_j, sf_j);
			posePDF_j->getMean(pose_j);

			relPose = pose_j - pose_i;

			// And its points map:
			CMultiMetricMap *map_j = &m_individualMaps[j];

			// Compute matching ratio:
			m_A(i, j) = sim_func(*map_i, *map_j, *sf_i, *sf_j, i, j, relPose);
		}  // for j
	}  // for i

	// Self-similatity: Not used
	m_A(n - 1, n - 1) = 0;

	// make matrix symetric
	// -----------------------------------------------------------------
	for (i = 0; i < n; i++)
		for (j = i + 1; j < n; j++)
			m_A(i, j) = m_A(j, i) = 0.5 * (m_A(i, j) + m_A(j, i));

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

	unsigned int i, j;
	unsigned int n_nodes;
	unsigned int n_clusters_last;
	std::vector<uint32_t>
		mods;  // The list of nodes that will have been regrouped
	std::vector<bool> last_parts_are_mods;

	n_nodes = m_modified_nodes.size();  // total number of nodes (scans)
	n_clusters_last =
		m_last_partition.size();  // Number of clusters in the last partition

	last_parts_are_mods.resize(n_clusters_last);

	// If a single scan of the cluster is affected, the whole cluster is
	// affected
	// -------------------------------------------------------------------
	for (i = 0; i < n_clusters_last; i++)
	{
		std::vector<uint32_t> p = m_last_partition[i];

		// Recorrer esta particion:
		last_parts_are_mods[i] = false;
		//			for (j=0;j<p.size();j++)
		//				if (m_modified_nodes[ p[j] ])
		//					last_parts_are_mods[i] = true;
		last_parts_are_mods[i] = true;

		// If changed mark all the nodes
		if (last_parts_are_mods[i])
			for (j = 0; j < p.size(); j++) m_modified_nodes[p[j]] = true;
	}

	// How many nodes are going to be partitioned?
	mods.clear();
	for (i = 0; i < n_nodes; i++)
		if (m_modified_nodes[i]) mods.push_back(i);

	// printf("[%u nodes to be recomputed]", mods.size());

	if (mods.size() > 0)
	{
		// Construct submatrix of adjacencies only with the nodes that are going
		// to be regrouped
		// -------------------------------------------------------------------
		CMatrix A_mods;
		A_mods.setSize(mods.size(), mods.size());
		for (i = 0; i < mods.size(); i++)
		{
			for (j = 0; j < mods.size(); j++)
			{
				A_mods(i, j) = m_A(mods[i], mods[j]);
			}
		}

		// Partitions of the modified nodes
		vector<std::vector<uint32_t>> mods_parts;
		mods_parts.clear();

		CGraphPartitioner<CMatrix>::RecursiveSpectralPartition(
			A_mods, mods_parts, options.partitionThreshold, true, true,
			!options.forceBisectionOnly,
			options.minimumNumberElementsEachCluster, false /* verbose */
			);

		// Aggregate the results with the clusters that were not used and return
		// them
		// --------------------------------------------------------------------------
		partitions.clear();

		// 1) Add the partitions that have not been modified
		// -----------------------------------------------
		for (i = 0; i < m_last_partition.size(); i++)
			if (!last_parts_are_mods[i])
				partitions.push_back(m_last_partition[i]);

		// 2) Add the modified partitions
		// WARNING: Translate the indices acordingly
		// -----------------------------------------------
		for (i = 0; i < mods_parts.size(); i++)
		{
			std::vector<uint32_t> v;
			v.clear();
			for (j = 0; j < mods_parts[i].size(); j++)
				v.push_back(mods[mods_parts[i][j]]);

			partitions.push_back(v);
		}
	}

	// Update all nodes
	for (i = 0; i < n_nodes; i++) m_modified_nodes[i] = false;

	// Save partition so that we take it into account in the next iteration
	// ------------------------------------------------------------------------
	size_t n = partitions.size();
	m_last_partition.resize(n);
	for (i = 0; i < n; i++) m_last_partition[i] = partitions[i];

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
	std::vector<uint32_t>::reverse_iterator it;
	for (it = indexesToRemove.rbegin(); it != indexesToRemove.rend(); ++it)
	{
		deque<mrpt::maps::CMultiMetricMap>::iterator itM =
			m_individualMaps.begin() + *it;
		m_individualMaps.erase(itM);  // Delete from list
	}

	// The new sequence of localized SFs:
	// --------------------------------------------------
	for (it = indexesToRemove.rbegin(); it != indexesToRemove.rend(); ++it)
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

	for (size_t i = 0; i < m_individualFrames.size(); i++)
	{
		CPose3DPDF::Ptr i_pdf;
		CSensoryFrame::Ptr i_sf;
		m_individualFrames.get(i, i_pdf, i_sf);

		CPose3D i_mean;
		i_pdf->getMean(i_mean);

		opengl::CSphere::Ptr i_sph =
			mrpt::make_aligned_shared<opengl::CSphere>();
		i_sph->setRadius(0.02f);
		i_sph->setColor(0, 0, 1);

		if (!renameIndexes)
			i_sph->setName(format("%u", static_cast<unsigned int>(i)));
		else
		{
			std::map<uint32_t, int64_t>::const_iterator itName =
				renameIndexes->find(i);
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
	MRPT_TODO("Autodetermine bounding box");
	gl_grid->setPlaneLimits(-100, 100, -100, 100);
	gl_grid->setGridFrequency(5);
}

void CIncrementalMapPartitioner::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> m_individualFrames >> m_individualMaps >> m_A >>
				m_last_partition >> m_last_last_partition_are_new_ones >>
				m_modified_nodes;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

uint8_t CIncrementalMapPartitioner::serializeGetVersion() const { return 0; }
void CIncrementalMapPartitioner::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << m_individualFrames << m_individualMaps << m_A << m_last_partition
		<< m_last_last_partition_are_new_ones << m_modified_nodes;
}

/*---------------------------------------------------------------
					addMapFrame
  ---------------------------------------------------------------*/
unsigned int CIncrementalMapPartitioner::addMapFrame(
	const CSensoryFrame& frame, const CPose3DPDF& robotPose3D)
{
	return addMapFrame(
		CSensoryFrame::Ptr(new CSensoryFrame(frame)),
		std::dynamic_pointer_cast<CPose3DPDF>(
			robotPose3D.duplicateGetSmartPtr()));
}
