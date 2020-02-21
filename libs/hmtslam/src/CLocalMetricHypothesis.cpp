/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/os.h>

#include <mrpt/hmtslam/CRobotPosesGraph.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::opengl;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CLocalMetricHypothesis, CSerializable, mrpt::hmtslam)
IMPLEMENTS_SERIALIZABLE(CLSLAMParticleData, CSerializable, mrpt::hmtslam)

/*---------------------------------------------------------------
				Normal constructor
  ---------------------------------------------------------------*/
CLocalMetricHypothesis::CLocalMetricHypothesis(CHMTSLAM* parent)
	: m_parent(parent), m_log_w_metric_history()
// m_log_w_topol_history()
{
	m_log_w = 0;
	m_accumRobotMovementIsValid = false;
}

/*---------------------------------------------------------------
				Destructor
  ---------------------------------------------------------------*/
CLocalMetricHypothesis::~CLocalMetricHypothesis() { clearParticles(); }
/*---------------------------------------------------------------
				   getAs3DScene

	Returns a 3D representation of the current robot pose, all
	the poses in the auxiliary graph, and each of the areas
	they belong to.
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getAs3DScene(
	opengl::CSetOfObjects::Ptr& objs) const
{
	objs->clear();

	// -------------------------------------------
	// Draw a grid on the ground:
	// -------------------------------------------
	{
		opengl::CGridPlaneXY::Ptr obj =
			std::make_shared<opengl::CGridPlaneXY>(-100, 100, -100, 100, 0, 5);
		obj->setColor(0.4, 0.4, 0.4);

		objs->insert(obj);  // it will free the memory
	}

	// ---------------------------------------------------------
	// Draw each of the robot poses as 2D/3D ellipsoids
	//  For the current pose, draw the robot itself as well.
	// ---------------------------------------------------------
	std::map<TPoseID, CPose3DPDFParticles> lstPoses;
	std::map<TPoseID, CPose3DPDFParticles>::iterator it;
	getPathParticles(lstPoses);

	// -----------------------------------------------
	// Draw a 3D coordinates corner for each cluster
	// -----------------------------------------------
	{
		std::lock_guard<std::mutex> lock(m_parent->m_map_cs);

		for (unsigned long m_neighbor : m_neighbors)
		{
			const CHMHMapNode::Ptr node =
				m_parent->m_map.getNodeByID(m_neighbor);
			ASSERT_(node);
			TPoseID poseID_origin;
			CPose3D originPose;
			if (node->m_annotations.getElemental(
					NODE_ANNOTATION_REF_POSEID, poseID_origin, m_ID))
			{
				lstPoses[poseID_origin].getMean(originPose);

				opengl::CSetOfObjects::Ptr corner = stock_objects::CornerXYZ();
				corner->setPose(originPose);
				objs->insert(corner);
			}
		}
	}  // end of lock on map_cs

	// Add a camera following the robot:
	// -----------------------------------------
	const CPose3D meanCurPose = lstPoses[m_currentRobotPose].getMeanVal();
	{
		opengl::CCamera::Ptr cam = std::make_shared<opengl::CCamera>();
		cam->setZoomDistance(85);
		cam->setAzimuthDegrees(45 + RAD2DEG(meanCurPose.yaw()));
		cam->setElevationDegrees(45);
		cam->setPointingAt(meanCurPose);
		objs->insert(cam);
	}

	map<CHMHMapNode::TNodeID, CPoint3D>
		areas_mean;  // Store the mean pose of each area
	map<CHMHMapNode::TNodeID, int> areas_howmany;

	for (it = lstPoses.begin(); it != lstPoses.end(); it++)
	{
		opengl::CEllipsoid3D::Ptr ellip = std::make_shared<opengl::CEllipsoid3D>();
		// Color depending on being into the current area:
		if (m_nodeIDmemberships.find(it->first)->second ==
			m_nodeIDmemberships.find(m_currentRobotPose)->second)
			ellip->setColor(0, 0, 1);
		else
			ellip->setColor(1, 0, 0);

		const CPose3DPDFParticles* pdfParts = &it->second;
		CPose3DPDFGaussian pdf;
		pdf.copyFrom(*pdfParts);

		// Mean:
		ellip->setLocation(
			pdf.mean.x(), pdf.mean.y(),
			pdf.mean.z() + 0.005);  // To be above the ground gridmap...

		// Cov:
		CMatrixDouble C = CMatrixDouble(pdf.cov);  // 6x6 cov.
		C.setSize(3, 3);

		// Is a 2D pose??
		if (pdf.mean.pitch() == 0 && pdf.mean.roll() == 0 &&
			pdf.cov(2, 2) < 1e-4f)
			C.setSize(2, 2);

		ellip->setCovMatrix(C);

		ellip->enableDrawSolid3D(false);

		// Name:
		ellip->setName(format("P%u", (unsigned int)it->first));
		ellip->enableShowName();

		// Add it:
		objs->insert(ellip);

		// Add an arrow for the mean direction also:
		{
			mrpt::opengl::CArrow::Ptr obj = mrpt::opengl::CArrow::Create(
				0, 0, 0, 0.20f, 0, 0, 0.25f, 0.005f, 0.02f);
			obj->setColor(1, 0, 0);
			obj->setPose(pdf.mean);
			objs->insert(obj);
		}

		// Is this the current robot pose?
		if (it->first == m_currentRobotPose)
		{
			// Draw robot:
			opengl::CSetOfObjects::Ptr robot = stock_objects::RobotPioneer();
			robot->setPose(pdf.mean);

			// Add it:
			objs->insert(robot);
		}
		else  // The current robot pose does not count
		{
			// Compute the area means:
			auto itAreaId = m_nodeIDmemberships.find(it->first);
			ASSERT_(itAreaId != m_nodeIDmemberships.end());
			CHMHMapNode::TNodeID areaId = itAreaId->second;
			areas_howmany[areaId]++;
			areas_mean[areaId].x_incr(pdf.mean.x());
			areas_mean[areaId].y_incr(pdf.mean.y());
			areas_mean[areaId].z_incr(pdf.mean.z());
		}
	}  // end for it

	// Average area means:
	map<CHMHMapNode::TNodeID, CPoint3D>::iterator itMeans;
	map<CHMHMapNode::TNodeID, int>::iterator itHowMany;
	ASSERT_(areas_howmany.size() == areas_mean.size());
	for (itMeans = areas_mean.begin(), itHowMany = areas_howmany.begin();
		 itMeans != areas_mean.end(); itMeans++, itHowMany++)
	{
		ASSERT_(itHowMany->second > 0);

		float f = 1.0f / itHowMany->second;
		itMeans->second *= f;
	}

	// -------------------------------------------------------------------
	// Draw lines between robot poses & their corresponding area sphere
	// -------------------------------------------------------------------
	for (it = lstPoses.begin(); it != lstPoses.end(); it++)
	{
		if (it->first != m_currentRobotPose)
		{
			CPoint3D areaPnt(
				areas_mean[m_nodeIDmemberships.find(it->first)->second]);
			areaPnt.z_incr(m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT);

			const CPose3DPDFParticles* pdfParts = &it->second;
			CPose3DPDFGaussian pdf;
			pdf.copyFrom(*pdfParts);

			opengl::CSimpleLine::Ptr line =
				std::make_shared<opengl::CSimpleLine>();
			line->setColor(0.8, 0.8, 0.8, 0.3);
			line->setLineWidth(2);

			line->setLineCoords(
				pdf.mean.x(), pdf.mean.y(), pdf.mean.z(), areaPnt.x(),
				areaPnt.y(), areaPnt.z());
			objs->insert(line);
		}
	}  // end for it

	// -------------------------------------------------------------------
	// Draw lines for links between robot poses
	// -------------------------------------------------------------------
	//	for (it=m_robotPoses.begin(); it!=m_robotPoses.end();it++)
	/*	for (it=lstPoses.begin(); it!=lstPoses.end();it++)
		{
			const CPose3DPDFParticles  *myPdfParts = &it->second;
			CPose3DPDFGaussian   myPdf;
			myPdf.copyFrom( *myPdfParts );

			std::map<TPoseID,TInterRobotPosesInfo>::const_iterator itLink;
			for
	   (itLink=it->second.m_links.begin();itLink!=it->second.m_links.end();itLink++)
			{
				if (itLink->second.SSO>0.7)
				{
					CRobotPosesAuxiliaryGraph::const_iterator hisIt =
	   m_robotPoses.find( itLink->first );
					ASSERT_( hisIt !=m_robotPoses.end() );

					const CPose3DPDFGaussian  *hisPdf = & hisIt->second.m_pose;

					opengl::CSimpleLine::Ptr line =
	   std::make_shared<opengl::CSimpleLine>();
					line->m_color_R = 0.2f;
					line->m_color_G = 0.8f;
					line->m_color_B = 0.2f;
					line->m_color_A = 0.3f;
					line->m_lineWidth = 3.0f;

					line->m_x0 = myPdf->mean.x;
					line->m_y0 = myPdf->mean.y;
					line->m_z0 = myPdf->mean.z;

					line->m_x1 = hisPdf->mean.x;
					line->m_y1 = hisPdf->mean.y;
					line->m_z1 = hisPdf->mean.z;

					objs->insert( line );
				}
			}
		} // end for it
	*/

	// ---------------------------------------------------------
	// Draw each of the areas in the neighborhood:
	// ---------------------------------------------------------
	{
		std::lock_guard<std::mutex> lock(
			m_parent->m_map_cs);  // To access nodes' labels.

		for (itMeans = areas_mean.begin(); itMeans != areas_mean.end();
			 itMeans++)
		{
			opengl::CSphere::Ptr sphere = std::make_shared<opengl::CSphere>();

			if (itMeans->first ==
				m_nodeIDmemberships.find(m_currentRobotPose)->second)
			{  // Color of current area
				sphere->setColor(0.1, 0.1, 0.7);
			}
			else
			{  // Color of other areas
				sphere->setColor(0.7, 0, 0);
			}

			sphere->setLocation(
				itMeans->second.x(), itMeans->second.y(),
				itMeans->second.z() +
					m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT);

			sphere->setRadius(m_parent->m_options.VIEW3D_AREA_SPHERES_RADIUS);

			// Add it:
			objs->insert(sphere);

			// And text label:
			opengl::CText::Ptr txt = std::make_shared<opengl::CText>();
			txt->setColor(1, 1, 1);

			const CHMHMapNode::Ptr node =
				m_parent->m_map.getNodeByID(itMeans->first);
			ASSERT_(node);

			txt->setLocation(
				itMeans->second.x(), itMeans->second.y(),
				itMeans->second.z() +
					m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT);

			//			txt->m_str = node->m_label;
			txt->setString(format("%li", (long int)node->getID()));

			objs->insert(txt);
		}
	}  // end of lock on map_cs

	// ---------------------------------------------------------
	// Draw links between areas:
	// ---------------------------------------------------------
	{
		std::lock_guard<std::mutex> lock(m_parent->m_map_cs);

		for (itMeans = areas_mean.begin(); itMeans != areas_mean.end();
			 itMeans++)
		{
			CHMHMapNode::TNodeID srcAreaID = itMeans->first;
			const CHMHMapNode::Ptr srcArea =
				m_parent->m_map.getNodeByID(srcAreaID);
			ASSERT_(srcArea);

			TArcList lstArcs;
			srcArea->getArcs(lstArcs);
			for (auto a = lstArcs.begin(); a != lstArcs.end(); ++a)
			{
				// target is in the neighborhood of LMH:
				if ((*a)->getNodeFrom() == srcAreaID)
				{
					auto trgAreaPoseIt = areas_mean.find((*a)->getNodeTo());
					if (trgAreaPoseIt != areas_mean.end())
					{
						// Yes, target node of the arc is in the LMH: Draw it:
						opengl::CSimpleLine::Ptr line =
							std::make_shared<opengl::CSimpleLine>();
						line->setColor(0.8, 0.8, 0);
						line->setLineWidth(3);

						line->setLineCoords(
							areas_mean[srcAreaID].x(),
							areas_mean[srcAreaID].y(),
							areas_mean[srcAreaID].z() +
								m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT,
							trgAreaPoseIt->second.x(),
							trgAreaPoseIt->second.y(),
							trgAreaPoseIt->second.z() +
								m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT);

						objs->insert(line);
					}
				}
			}  // end for each arc
		}  // end for each area

	}  // end of lock on map_cs
}

/** The PF algorithm implementation.  */
void CLocalMetricHypothesis::prediction_and_update_pfAuxiliaryPFOptimal(
	const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	ASSERT_(m_parent.get());
	ASSERT_(m_parent->m_LSLAM_method);
	m_parent->m_LSLAM_method->prediction_and_update_pfAuxiliaryPFOptimal(
		this, action, observation, PF_options);
}

void CLocalMetricHypothesis::prediction_and_update_pfOptimalProposal(
	const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	ASSERT_(m_parent.get());
	ASSERT_(m_parent->m_LSLAM_method);
	m_parent->m_LSLAM_method->prediction_and_update_pfOptimalProposal(
		this, action, observation, PF_options);
}

/*---------------------------------------------------------------
					getMeans
   Returns the mean of each robot pose in this LMH, as
	 computed from the set of particles.
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getMeans(TMapPoseID2Pose3D& outList) const
{
	MRPT_START

	outList.clear();

	// Build list of particles pdfs:
	std::map<TPoseID, CPose3DPDFParticles> parts;
	getPathParticles(parts);

	std::map<TPoseID, CPose3DPDFParticles>::iterator it;

	for (it = parts.begin(); it != parts.end(); it++)
		it->second.getMean(outList[it->first]);

	MRPT_END
}

/*---------------------------------------------------------------
					getPathParticles
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getPathParticles(
	std::map<TPoseID, CPose3DPDFParticles>& outList) const
{
	MRPT_START

	outList.clear();

	if (m_particles.empty()) return;

	// For each poseID:
	for (auto itPoseID = m_particles.begin()->d->robotPoses.begin();
		 itPoseID != m_particles.begin()->d->robotPoses.end(); ++itPoseID)
	{
		CPose3DPDFParticles auxPDF(m_particles.size());
		CParticleList::const_iterator it;
		CPose3DPDFParticles::CParticleList::iterator itP;
		for (it = m_particles.begin(), itP = auxPDF.m_particles.begin();
			 it != m_particles.end(); it++, itP++)
		{
			itP->log_w = it->log_w;
			itP->d = it->d->robotPoses.find(itPoseID->first)->second.asTPose();
		}

		// Save PDF:
		outList[itPoseID->first] = auxPDF;
	}  // end for itPoseID

	MRPT_END
}

/*---------------------------------------------------------------
					getPoseParticles
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getPoseParticles(
	const TPoseID& poseID, CPose3DPDFParticles& outPDF) const
{
	MRPT_START

	ASSERT_(!m_particles.empty());

	CParticleList::const_iterator it;
	outPDF.resetDeterministic(TPose3D(0, 0, 0, 0, 0, 0), m_particles.size());
	CPose3DPDFParticles::CParticleList::iterator itP;
	for (it = m_particles.begin(), itP = outPDF.m_particles.begin();
		 it != m_particles.end(); it++, itP++)
	{
		itP->log_w = it->log_w;
		auto itPose = it->d->robotPoses.find(poseID);
		ASSERT_(itPose != it->d->robotPoses.end());
		itP->d = itPose->second.asTPose();
	}

	MRPT_END
}

/*---------------------------------------------------------------
					clearRobotPoses
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::clearRobotPoses()
{
	clearParticles();
	m_particles.resize(m_parent->m_options.pf_options.sampleSize);
	for (auto& m_particle : m_particles)
	{
		// Create particle:
		m_particle.log_w = 0;
		m_particle.d.reset(new CLSLAMParticleData(
			&m_parent->m_options.defaultMapsInitializers));

		// Fill in:
		m_particle.d->robotPoses.clear();
	}
}

/*---------------------------------------------------------------
						getCurrentPose
 ---------------------------------------------------------------*/
const CPose3D* CLocalMetricHypothesis::getCurrentPose(size_t particleIdx) const
{
	if (particleIdx >= m_particles.size())
		THROW_EXCEPTION("Particle index out of bounds!");

	auto it = m_particles[particleIdx].d->robotPoses.find(m_currentRobotPose);
	ASSERT_(it != m_particles[particleIdx].d->robotPoses.end());
	return &it->second;
}

/*---------------------------------------------------------------
						getCurrentPose
 ---------------------------------------------------------------*/
CPose3D* CLocalMetricHypothesis::getCurrentPose(size_t particleIdx)
{
	if (particleIdx >= m_particles.size())
		THROW_EXCEPTION("Particle index out of bounds!");

	auto it = m_particles[particleIdx].d->robotPoses.find(m_currentRobotPose);
	ASSERT_(it != m_particles[particleIdx].d->robotPoses.end());
	return &it->second;
}

/*---------------------------------------------------------------
						getRelativePose
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getRelativePose(
	const TPoseID& reference, const TPoseID& pose,
	CPose3DPDFParticles& outPDF) const
{
	MRPT_START

	// Resize output:
	outPDF.resetDeterministic(TPose3D(0, 0, 0, 0, 0, 0), m_particles.size());

	CParticleList::const_iterator it;
	CPose3DPDFParticles::CParticleList::iterator itP;
	for (it = m_particles.begin(), itP = outPDF.m_particles.begin();
		 it != m_particles.end(); it++, itP++)
	{
		itP->log_w = it->log_w;

		auto srcPose = it->d->robotPoses.find(reference);
		auto trgPose = it->d->robotPoses.find(pose);

		ASSERT_(srcPose != it->d->robotPoses.end());
		ASSERT_(trgPose != it->d->robotPoses.end());
		itP->d =
			(CPose3D(trgPose->second) - CPose3D(srcPose->second)).asTPose();
	}

	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinateOrigin
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::changeCoordinateOrigin(const TPoseID& newOrigin)
{
	CPose3DPDFParticles originPDF(m_particles.size());

	CParticleList::iterator it;
	CPose3DPDFParticles::CParticleList::iterator itOrgPDF;

	for (it = m_particles.begin(), itOrgPDF = originPDF.m_particles.begin();
		 it != m_particles.end(); it++, itOrgPDF++)
	{
		auto refPoseIt = it->d->robotPoses.find(newOrigin);
		ASSERT_(refPoseIt != it->d->robotPoses.end());
		const CPose3D& refPose = refPoseIt->second;

		// Save in pdf to compute mean:
		itOrgPDF->d = refPose.asTPose();
		itOrgPDF->log_w = it->log_w;

		auto End = it->d->robotPoses.end();
		// Change all other poses first:
		for (auto itP = it->d->robotPoses.begin(); itP != End; ++itP)
			if (itP != refPoseIt) itP->second = itP->second - refPose;

		// Now set new origin to 0:
		refPoseIt->second.setFromValues(0, 0, 0);
	}

	// Rebuild metric maps for consistency:
	rebuildMetricMaps();

	// Change coords in incr. partitioning as well:
	{
		std::lock_guard<std::mutex> locker(m_robotPosesGraph.lock);

		CSimpleMap* SFseq = m_robotPosesGraph.partitioner.getSequenceOfFrames();
		for (auto& p : m_robotPosesGraph.idx2pose)
		{
			CPose3DPDF::Ptr pdf;
			CSensoryFrame::Ptr sf;
			SFseq->get(p.first, pdf, sf);

			// Copy from particles:
			ASSERT_(pdf->GetRuntimeClass() == CLASS_ID(CPose3DPDFParticles));
			auto pdfParts = std::dynamic_pointer_cast<CPose3DPDFParticles>(pdf);
			getPoseParticles(p.second, *pdfParts);
		}
	}
}

/*---------------------------------------------------------------
						rebuildMetricMaps
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::rebuildMetricMaps()
{
	for (auto& m_particle : m_particles)
	{
		m_particle.d->metricMaps.clear();

		// Follow all robot poses:
		auto End = m_particle.d->robotPoses.end();
		for (auto itP = m_particle.d->robotPoses.begin(); itP != End; ++itP)
		{
			if (itP->first !=
				m_currentRobotPose)  // Current robot pose has no SF stored.
			{
				auto SFit = m_SFs.find(itP->first);
				ASSERT_(SFit != m_SFs.end());
				SFit->second.insertObservationsInto(
					&m_particle.d->metricMaps, &itP->second);
			}
		}
	}
}

/** Removes a given area from the LMH:
 *	- The corresponding node in the HMT map is updated with the robot poses &
 *SFs in the LMH.
 *	- Robot poses belonging to that area are removed from:
 *		- the particles.
 *		- the graph partitioner.
 *		- the list of SFs.
 *		- the list m_nodeIDmemberships.
 *		- The weights of all particles are changed to remove the effects of the
 *removed metric observations.
 *	- After calling this the metric maps should be updated.
 */
void CLocalMetricHypothesis::removeAreaFromLMH(
	const CHMHMapNode::TNodeID areaID)
{
	MRPT_START

	// Remove from m_neighbors:
	// -----------------------------------
	auto itNeig = m_neighbors.find(areaID);
	if (itNeig != m_neighbors.end()) m_neighbors.erase(itNeig);

	// Build the list with the poses in the area to be removed from LMH:
	// ----------------------------------------------------------------------
	TNodeIDList lstPoseIDs;
	for (auto& m_nodeIDmembership : m_nodeIDmemberships)
		if (m_nodeIDmembership.second == areaID)
			lstPoseIDs.insert(m_nodeIDmembership.first);

	ASSERT_(!lstPoseIDs.empty());

	// ----------------------------------------------------------------------
	// The corresponding node in the HMT map is updated with the
	//   robot poses & SFs in the LMH.
	// ----------------------------------------------------------------------
	updateAreaFromLMH(areaID, true);

	// - Robot poses belonging to that area are removed from:
	// 	- the particles.
	// ----------------------------------------------------------------------
	for (auto it = lstPoseIDs.begin(); it != lstPoseIDs.end(); ++it)
		for (auto& m_particle : m_particles)
			m_particle.d->robotPoses.erase(m_particle.d->robotPoses.find(*it));

	// - The weights of all particles are changed to remove the effects of the
	// removed metric observations.
	// ----------------------------------------------------------------------
	{
		CParticleList::iterator p;
		vector<map<TPoseID, double>>::iterator ws_it;
		ASSERT_(m_log_w_metric_history.size() == m_particles.size());

		for (ws_it = m_log_w_metric_history.begin(), p = m_particles.begin();
			 p != m_particles.end(); ++p, ++ws_it)
		{
			for (auto it = lstPoseIDs.begin(); it != lstPoseIDs.end(); ++it)
			{
				auto itW = ws_it->find(*it);
				if (itW != ws_it->end())
				{
					MRPT_CHECK_NORMAL_NUMBER(itW->second);

					p->log_w -= itW->second;
					// No longer required:
					ws_it->erase(itW);
				}
			}
		}
	}

	// - Robot poses belonging to that area are removed from:
	// 	- the graph partitioner.
	// ----------------------------------------------------------------------
	{
		std::lock_guard<std::mutex> locker(m_robotPosesGraph.lock);

		std::vector<uint32_t> indexesToRemove;
		indexesToRemove.reserve(lstPoseIDs.size());

		for (auto it = m_robotPosesGraph.idx2pose.begin();
			 it != m_robotPosesGraph.idx2pose.end();)
		{
			if (lstPoseIDs.find(it->second) != lstPoseIDs.end())
			{
				indexesToRemove.push_back(it->first);

				// Remove from the mapping indexes->nodeIDs as well:
				auto it2 = it;
				it2++;
				// it = m_robotPosesGraph.idx2pose.erase( it );
				m_robotPosesGraph.idx2pose.erase(it);
				it = it2;
			}
			else
				it++;
		}

		m_robotPosesGraph.partitioner.removeSetOfNodes(indexesToRemove);

		// Renumbering of indexes<->posesIDs to be the same than in
		// "m_robotPosesGraph.partitioner":
		unsigned idx = 0;
		map<uint32_t, TPoseID> newList;
		for (auto i = m_robotPosesGraph.idx2pose.begin();
			 i != m_robotPosesGraph.idx2pose.end(); ++i, idx++)
			newList[idx] = i->second;
		m_robotPosesGraph.idx2pose = newList;
	}

	// - Robot poses belonging to that area are removed from:
	// - the list of SFs.
	// ----------------------------------------------------------------------
	// Already done above.

	// - Robot poses belonging to that area are removed from:
	// - the list m_nodeIDmemberships.
	// ----------------------------------------------------------------------
	for (auto it = lstPoseIDs.begin(); it != lstPoseIDs.end(); ++it)
		m_nodeIDmemberships.erase(m_nodeIDmemberships.find(*it));

	double out_max_log_w;
	normalizeWeights(&out_max_log_w);  // Normalize weights:

	MRPT_END
}

/*---------------------------------------------------------------
					TRobotPosesPartitioning::pose2idx
 ---------------------------------------------------------------*/
unsigned int CLocalMetricHypothesis::TRobotPosesPartitioning::pose2idx(
	const TPoseID& id) const
{
	for (auto it : idx2pose)
		if (it.second == id) return it.first;
	THROW_EXCEPTION_FMT("PoseID=%i not found.", static_cast<int>(id));
}

/*---------------------------------------------------------------
					updateAreaFromLMH

// The corresponding node in the HMT map is updated with the
//   robot poses & SFs in the LMH.
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::updateAreaFromLMH(
	const CHMHMapNode::TNodeID areaID, bool eraseSFsFromLMH)
{
	// Build the list with the poses belonging to that area from LMH:
	// ----------------------------------------------------------------------
	TNodeIDList lstPoseIDs;
	for (auto it = m_nodeIDmemberships.begin(); it != m_nodeIDmemberships.end();
		 ++it)
		if (it->second == areaID) lstPoseIDs.insert(it->first);

	ASSERT_(!lstPoseIDs.empty());

	CHMHMapNode::Ptr node;
	{
		std::lock_guard<std::mutex> lock(m_parent->m_map_cs);
		node = m_parent->m_map.getNodeByID(areaID);
		ASSERT_(node);
		ASSERT_(node->m_hypotheses.has(m_ID));
	}  // end of HMT map cs

	// The pose to become the origin:
	TPoseID poseID_origin;
	node->m_annotations.getElemental(
		NODE_ANNOTATION_REF_POSEID, poseID_origin, m_ID, true);

	// 1) The set of robot poses and SFs
	//    In annotation: 					NODE_ANNOTATION_POSES_GRAPH
	// ---------------------------------------------------------------------
	CRobotPosesGraph::Ptr posesGraph;
	{
		CSerializable::Ptr annot =
			node->m_annotations.get(NODE_ANNOTATION_POSES_GRAPH, m_ID);
		if (!annot)
		{
			// Add it now:
			posesGraph = std::make_shared<CRobotPosesGraph>();
			node->m_annotations.setMemoryReference(
				NODE_ANNOTATION_POSES_GRAPH, posesGraph, m_ID);
		}
		else
		{
			posesGraph = std::dynamic_pointer_cast<CRobotPosesGraph>(annot);
			posesGraph->clear();
		}
	}

	// For each pose in the area:
	CPose3DPDFParticles pdfOrigin;
	bool pdfOrigin_ok = false;
	for (auto it = lstPoseIDs.begin(); it != lstPoseIDs.end(); ++it)
	{
		TPoseInfo& poseInfo = (*posesGraph)[*it];
		getPoseParticles(*it, poseInfo.pdf);  // Save pose particles

		// Save the PDF of the origin:
		if (*it == poseID_origin)
		{
			pdfOrigin.copyFrom(poseInfo.pdf);
			pdfOrigin_ok = true;
		}

		if (*it != m_currentRobotPose)  // The current robot pose has no SF
		{
			auto itSF = m_SFs.find(*it);
			ASSERT_(itSF != m_SFs.end());

			if (eraseSFsFromLMH)
			{
				poseInfo.sf = std::move(itSF->second);
				m_SFs.erase(itSF);
			}
			else
			{
				poseInfo.sf = itSF->second;  // Copy observations
			}
		}
	}

	// Readjust to set the origin pose ID:
	ASSERT_(pdfOrigin_ok);
	CPose3DPDFParticles pdfOriginInv;
	pdfOrigin.inverse(pdfOriginInv);
	for (auto it = posesGraph->begin(); it != posesGraph->end(); ++it)
	{
		CPose3DPDFParticles::CParticleList::iterator orgIt, pdfIt;
		ASSERT_(it->second.pdf.size() == pdfOriginInv.size());
		for (pdfIt = it->second.pdf.m_particles.begin(),
			orgIt = pdfOriginInv.m_particles.begin();
			 orgIt != pdfOriginInv.m_particles.end(); orgIt++, pdfIt++)
			pdfIt->d = (CPose3D(orgIt->d) + CPose3D(pdfIt->d)).asTPose();
	}

	// 2) One single metric map built from the most likelily robot poses
	//    In annotation: 					NODE_ANNOTATION_METRIC_MAPS
	// ---------------------------------------------------------------------
	CMultiMetricMap::Ptr metricMap = node->m_annotations.getAs<CMultiMetricMap>(
		NODE_ANNOTATION_METRIC_MAPS, m_ID, false);
	metricMap->clear();
	posesGraph->insertIntoMetricMap(*metricMap);
}

/*---------------------------------------------------------------
				dumpAsText
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::dumpAsText(std::vector<std::string>& st) const
{
	st.clear();
	st.emplace_back("LIST OF POSES IN LMH");
	st.emplace_back("====================");

	string s;
	s = "Neighbors: ";
	for (unsigned long m_neighbor : m_neighbors)
		s += format("%i ", (int)m_neighbor);
	st.push_back(s);

	TMapPoseID2Pose3D lst;
	getMeans(lst);

	for (auto it = lst.begin(); it != lst.end(); ++it)
	{
		auto area = m_nodeIDmemberships.find(it->first);
		st.emplace_back(mrpt::format(
			"  ID: %i \t AREA: %i \t %.03f,%.03f,%.03fdeg", (int)it->first,
			(int)area->second, it->second.x(), it->second.y(),
			RAD2DEG(it->second.yaw())));
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> m_ID >> m_currentRobotPose >> m_neighbors >>
				m_nodeIDmemberships >> m_SFs >> m_posesPendingAddPartitioner >>
				m_areasPendingTBI >> m_log_w >> m_log_w_metric_history >>
				m_robotPosesGraph.partitioner >> m_robotPosesGraph.idx2pose;

			// particles:
			readParticlesFromStream(in);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

uint8_t CLocalMetricHypothesis::serializeGetVersion() const { return 0; }
void CLocalMetricHypothesis::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << m_ID << m_currentRobotPose << m_neighbors << m_nodeIDmemberships
		<< m_SFs << m_posesPendingAddPartitioner << m_areasPendingTBI << m_log_w
		<< m_log_w_metric_history << m_robotPosesGraph.partitioner
		<< m_robotPosesGraph.idx2pose;

	// particles:
	writeParticlesToStream(out);
}

void CLSLAMParticleData::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> metricMaps >> robotPoses;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

uint8_t CLSLAMParticleData::serializeGetVersion() const { return 0; }
void CLSLAMParticleData::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << metricMaps << robotPoses;
}
