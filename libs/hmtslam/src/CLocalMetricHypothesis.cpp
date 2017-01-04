/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/utils/stl_serialization.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/hmtslam/CRobotPosesGraph.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::opengl;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CLocalMetricHypothesis, CSerializable,mrpt::hmtslam)
IMPLEMENTS_SERIALIZABLE(CLSLAMParticleData, CSerializable,mrpt::hmtslam)


/*---------------------------------------------------------------
				Normal constructor
  ---------------------------------------------------------------*/
CLocalMetricHypothesis::CLocalMetricHypothesis( CHMTSLAM  * parent ) :
	m_lock(),
	m_parent(parent),
	m_log_w_metric_history()
	//m_log_w_topol_history()
{
	m_log_w 				= 0;
	m_accumRobotMovementIsValid	= false;
}


/*---------------------------------------------------------------
				Destructor
  ---------------------------------------------------------------*/
CLocalMetricHypothesis::~CLocalMetricHypothesis()
{
	clearParticles();
}

/*---------------------------------------------------------------
				   getAs3DScene

	Returns a 3D representation of the current robot pose, all
	the poses in the auxiliary graph, and each of the areas
	they belong to.
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getAs3DScene( opengl::CSetOfObjectsPtr &objs ) const
{
	objs->clear();

	// -------------------------------------------
	// Draw a grid on the ground:
	// -------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-100,100,-100,100,0,5);
		obj->setColor(0.4,0.4,0.4);

		objs->insert(obj);  // it will free the memory
	}

	// ---------------------------------------------------------
	// Draw each of the robot poses as 2D/3D ellipsoids
	//  For the current pose, draw the robot itself as well.
	// ---------------------------------------------------------
	std::map< TPoseID, CPose3DPDFParticles > lstPoses;
	std::map< TPoseID, CPose3DPDFParticles >::iterator	it;
	getPathParticles( lstPoses );

	// -----------------------------------------------
	// Draw a 3D coordinates corner for each cluster
	// -----------------------------------------------
	{
		CCriticalSectionLocker	locker( &m_parent->m_map_cs );

		for ( TNodeIDSet::const_iterator n=m_neighbors.begin();n!=m_neighbors.end();++n)
		{
			const CHMHMapNodePtr node = m_parent->m_map.getNodeByID( *n );
			ASSERT_(node);
			TPoseID poseID_origin;
			CPose3D originPose;
			if (node->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID,poseID_origin, m_ID))
			{
				lstPoses[ poseID_origin ].getMean(originPose);

				opengl::CSetOfObjectsPtr corner = stock_objects::CornerXYZ();
				corner->setPose(originPose);
				objs->insert( corner );
			}
		}
	} // end of lock on map_cs

	// Add a camera following the robot:
	// -----------------------------------------
	const CPose3D meanCurPose = lstPoses[ m_currentRobotPose ].getMeanVal();
	{
		opengl::CCameraPtr cam = opengl::CCamera::Create();
		cam->setZoomDistance(85);
		cam->setAzimuthDegrees(45 + RAD2DEG(meanCurPose.yaw()));
		cam->setElevationDegrees(45);
		cam->setPointingAt(meanCurPose);
		objs->insert(cam);
	}



	map<CHMHMapNode::TNodeID,CPoint3D>      areas_mean;  // Store the mean pose of each area
	map<CHMHMapNode::TNodeID,int>			areas_howmany;

	for (it=lstPoses.begin(); it!=lstPoses.end();it++)
	{
		opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();
		// Color depending on being into the current area:
		if ( m_nodeIDmemberships.find(it->first)->second == m_nodeIDmemberships.find(m_currentRobotPose)->second )
			ellip->setColor(0,0,1);
		else
			ellip->setColor(1,0,0);

		const CPose3DPDFParticles  *pdfParts = &it->second;
		CPose3DPDFGaussian   pdf;
		pdf.copyFrom( *pdfParts );

		// Mean:
		ellip->setLocation(pdf.mean.x(), pdf.mean.y(), pdf.mean.z()+0.005);  // To be above the ground gridmap...

		// Cov:
		CMatrixDouble C = CMatrixDouble(pdf.cov); // 6x6 cov.
		C.setSize(3,3);

		// Is a 2D pose??
		if ( pdf.mean.pitch() == 0 && pdf.mean.roll() == 0 && pdf.cov(2,2) < 1e-4f )
			C.setSize(2,2);

		ellip->setCovMatrix(C);

		ellip->enableDrawSolid3D(false);

		// Name:
		ellip->setName( format("P%u", (unsigned int) it->first ) );
		ellip->enableShowName();

		// Add it:
		objs->insert( ellip );

		// Add an arrow for the mean direction also:
		{
			mrpt::opengl::CArrowPtr obj = mrpt::opengl::CArrow::Create(
				0,0,0,
				0.20f,0,0,
				0.25f,0.005f,0.02f);
			obj->setColor(1,0,0);
			obj->setPose(pdf.mean);
			objs->insert( obj );
	    }



		// Is this the current robot pose?
		if (it->first == m_currentRobotPose )
		{
			// Draw robot:
			opengl::CSetOfObjectsPtr robot = stock_objects::RobotPioneer();
			robot->setPose(pdf.mean);

			// Add it:
			objs->insert( robot );
		}
		else // The current robot pose does not count
		{
			// Compute the area means:
			std::map<TPoseID,CHMHMapNode::TNodeID>::const_iterator itAreaId = m_nodeIDmemberships.find( it->first );
			ASSERT_( itAreaId != m_nodeIDmemberships.end() );
			CHMHMapNode::TNodeID  areaId = itAreaId->second;
			areas_howmany[ areaId  ]++;
			areas_mean[ areaId  ].x_incr( pdf.mean.x() );
			areas_mean[ areaId  ].y_incr( pdf.mean.y() );
			areas_mean[ areaId  ].z_incr( pdf.mean.z() );
		}
	} // end for it

	// Average area means:
	map<CHMHMapNode::TNodeID,CPoint3D>::iterator itMeans;
	map<CHMHMapNode::TNodeID,int>::iterator      itHowMany;
	ASSERT_( areas_howmany.size() == areas_mean.size() );
	for ( itMeans = areas_mean.begin(), itHowMany = areas_howmany.begin(); itMeans!=areas_mean.end(); itMeans++, itHowMany++ )
	{
		ASSERT_( itHowMany->second >0);

		float f = 1.0f / itHowMany->second;
		itMeans->second *= f;
	}


	// -------------------------------------------------------------------
	// Draw lines between robot poses & their corresponding area sphere
	// -------------------------------------------------------------------
	for (it=lstPoses.begin(); it!=lstPoses.end();it++)
	{
		if (it->first != m_currentRobotPose )
		{
			CPoint3D  areaPnt( areas_mean[ m_nodeIDmemberships.find( it->first )->second ] );
			areaPnt.z_incr( m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT );

			const CPose3DPDFParticles  *pdfParts = &it->second;
			CPose3DPDFGaussian   pdf;
			pdf.copyFrom( *pdfParts );

			opengl::CSimpleLinePtr line = opengl::CSimpleLine::Create();
			line->setColor(0.8,0.8,0.8, 0.3);
			line->setLineWidth(2);

			line->setLineCoords(
				pdf.mean.x(), pdf.mean.y(), pdf.mean.z(),
				areaPnt.x(), areaPnt.y(), areaPnt.z() );
			objs->insert( line );
		}
	} // end for it

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
		for (itLink=it->second.m_links.begin();itLink!=it->second.m_links.end();itLink++)
		{
			if (itLink->second.SSO>0.7)
			{
				CRobotPosesAuxiliaryGraph::const_iterator hisIt = m_robotPoses.find( itLink->first );
				ASSERT_( hisIt !=m_robotPoses.end() );

				const CPose3DPDFGaussian  *hisPdf = & hisIt->second.m_pose;

				opengl::CSimpleLinePtr line = opengl::CSimpleLine::Create();
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
		CCriticalSectionLocker	locker( &m_parent->m_map_cs ); //To access nodes' labels.

		for ( itMeans = areas_mean.begin(); itMeans!=areas_mean.end(); itMeans++ )
		{
			opengl::CSpherePtr sphere = opengl::CSphere::Create();

			if (itMeans->first == m_nodeIDmemberships.find( m_currentRobotPose)->second )
			{   // Color of current area
				sphere->setColor(0.1,0.1,0.7);
			}
			else
			{   // Color of other areas
				sphere->setColor(0.7,0,0);
			}

			sphere->setLocation(itMeans->second.x(), itMeans->second.y(), itMeans->second.z() + m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT);

			sphere->setRadius( m_parent->m_options.VIEW3D_AREA_SPHERES_RADIUS );

			// Add it:
			objs->insert( sphere );

			// And text label:
			opengl::CTextPtr txt = opengl::CText::Create();
			txt->setColor(1,1,1);

			const CHMHMapNodePtr node = m_parent->m_map.getNodeByID( itMeans->first );
			ASSERT_(node);

			txt->setLocation( itMeans->second.x(), itMeans->second.y(), itMeans->second.z() + m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT );

//			txt->m_str = node->m_label;
			txt->setString( format("%li",(long int)node->getID()) );

			objs->insert( txt );
		}
	} // end of lock on map_cs

	// ---------------------------------------------------------
	// Draw links between areas:
	// ---------------------------------------------------------
	{
		CCriticalSectionLocker	locker( &m_parent->m_map_cs );

		for ( itMeans = areas_mean.begin(); itMeans!=areas_mean.end(); itMeans++ )
		{
			CHMHMapNode::TNodeID  srcAreaID = itMeans->first;
			const CHMHMapNodePtr srcArea = m_parent->m_map.getNodeByID( srcAreaID );
			ASSERT_(srcArea);

			TArcList		lstArcs;
			srcArea->getArcs(lstArcs);
			for (TArcList::const_iterator a=lstArcs.begin();a!=lstArcs.end();++a)
			{
				// target is in the neighborhood of LMH:
				if ( (*a)->getNodeFrom() == srcAreaID )
				{
					map<CHMHMapNode::TNodeID,CPoint3D>::const_iterator trgAreaPoseIt = areas_mean.find( (*a)->getNodeTo() );
					if ( trgAreaPoseIt != areas_mean.end() )
					{
						// Yes, target node of the arc is in the LMH: Draw it:
						opengl::CSimpleLinePtr line = opengl::CSimpleLine::Create();
						line->setColor(0.8,0.8,0);
						line->setLineWidth(3);

						line->setLineCoords(
							areas_mean[srcAreaID].x(), areas_mean[srcAreaID].y(), areas_mean[srcAreaID].z() + m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT,
							trgAreaPoseIt->second.x(), trgAreaPoseIt->second.y(), trgAreaPoseIt->second.z() + m_parent->m_options.VIEW3D_AREA_SPHERES_HEIGHT );

						objs->insert( line );
					}
				}
			} // end for each arc
		} // end for each area

	} // end of lock on map_cs


}

/** The PF algorithm implementation.  */
void  CLocalMetricHypothesis::prediction_and_update_pfAuxiliaryPFOptimal(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	ASSERT_(m_parent.get());
	ASSERT_(m_parent->m_LSLAM_method);
	m_parent->m_LSLAM_method->prediction_and_update_pfAuxiliaryPFOptimal(this, action, observation,PF_options);
}

void  CLocalMetricHypothesis::prediction_and_update_pfOptimalProposal(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	ASSERT_(m_parent.get());
	ASSERT_(m_parent->m_LSLAM_method);
	m_parent->m_LSLAM_method->prediction_and_update_pfOptimalProposal(this, action, observation,PF_options);
}


/*---------------------------------------------------------------
					getMeans
   Returns the mean of each robot pose in this LMH, as
     computed from the set of particles.
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getMeans( TMapPoseID2Pose3D &outList ) const
{
	MRPT_START

	outList.clear();

	// Build list of particles pdfs:
	std::map< TPoseID, CPose3DPDFParticles >	parts;
	getPathParticles( parts );

	std::map< TPoseID, CPose3DPDFParticles >::iterator   it;

	for (it=parts.begin();it!=parts.end();it++)
		it->second.getMean( outList[ it->first ] );

	MRPT_END
}


/*---------------------------------------------------------------
					getPathParticles
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getPathParticles( std::map< TPoseID, CPose3DPDFParticles > &outList ) const
{
	MRPT_START

	outList.clear();

	if (m_particles.empty()) return;

	// For each poseID:
	for ( TMapPoseID2Pose3D::const_iterator itPoseID= m_particles.begin()->d->robotPoses.begin(); itPoseID!=m_particles.begin()->d->robotPoses.end();++itPoseID )
	{
		CPose3DPDFParticles								auxPDF( m_particles.size() );
		CParticleList::const_iterator  					it;
		CPose3DPDFParticles::CParticleList::iterator 	itP;
		for ( it = m_particles.begin(), itP = auxPDF.m_particles.begin(); it!=m_particles.end(); it++, itP++ )
		{
			itP->log_w = it->log_w;
			*itP->d    = it->d->robotPoses.find( itPoseID->first )->second;
		}

		// Save PDF:
		outList[ itPoseID->first ] = auxPDF;
	} // end for itPoseID

	MRPT_END
}


/*---------------------------------------------------------------
					getPoseParticles
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getPoseParticles( const TPoseID &poseID, CPose3DPDFParticles &outPDF ) const
{
	MRPT_START

	ASSERT_(!m_particles.empty());

	CParticleList::const_iterator  					it;
	outPDF.resetDeterministic( CPose3D(), m_particles.size() );
	CPose3DPDFParticles::CParticleList::iterator 	itP;
	for ( it = m_particles.begin(), itP = outPDF.m_particles.begin(); it!=m_particles.end(); it++, itP++ )
	{
		itP->log_w = it->log_w;
		TMapPoseID2Pose3D::const_iterator	itPose = it->d->robotPoses.find(poseID);
		ASSERT_( itPose!=it->d->robotPoses.end() );
		*itP->d = itPose->second;
	}

	MRPT_END
}


/*---------------------------------------------------------------
					clearRobotPoses
  ---------------------------------------------------------------*/
void CLocalMetricHypothesis::clearRobotPoses()
{
	clearParticles();
	m_particles.resize( m_parent->m_options.pf_options.sampleSize );
	for (CParticleList::iterator it = m_particles.begin();it!=m_particles.end();++it)
	{
		// Create particle:
		it->log_w = 0;
		it->d.reset(new CLSLAMParticleData( &m_parent->m_options.defaultMapsInitializers ));

		// Fill in:
		it->d->robotPoses.clear();
	}
}

/*---------------------------------------------------------------
						getCurrentPose
 ---------------------------------------------------------------*/
const CPose3D * CLocalMetricHypothesis::getCurrentPose(const size_t &particleIdx) const
{
	if (particleIdx>=m_particles.size()) THROW_EXCEPTION("Particle index out of bounds!");

	TMapPoseID2Pose3D::const_iterator it = m_particles[particleIdx].d->robotPoses.find( m_currentRobotPose );
	ASSERT_( it!=m_particles[particleIdx].d->robotPoses.end() );
	return & it->second;
}

/*---------------------------------------------------------------
						getCurrentPose
 ---------------------------------------------------------------*/
CPose3D * CLocalMetricHypothesis::getCurrentPose(const size_t &particleIdx)
{
	if (particleIdx>=m_particles.size()) THROW_EXCEPTION("Particle index out of bounds!");

	TMapPoseID2Pose3D::iterator it = m_particles[particleIdx].d->robotPoses.find( m_currentRobotPose );
	ASSERT_( it!=m_particles[particleIdx].d->robotPoses.end() );
	return & it->second;
}

/*---------------------------------------------------------------
						getRelativePose
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::getRelativePose(
	const  TPoseID &reference,
	const  TPoseID &pose,
	CPose3DPDFParticles  &outPDF ) const
{
	MRPT_START

	// Resize output:
	outPDF.resetDeterministic( CPose3D(), m_particles.size() );

	CParticleList::const_iterator  					it;
	CPose3DPDFParticles::CParticleList::iterator 	itP;
	for ( it = m_particles.begin(), itP = outPDF.m_particles.begin(); it!=m_particles.end(); it++, itP++ )
	{
		itP->log_w = it->log_w;

		TMapPoseID2Pose3D::const_iterator  srcPose =  it->d->robotPoses.find( reference );
		TMapPoseID2Pose3D::const_iterator  trgPose =  it->d->robotPoses.find( pose );

		ASSERT_( srcPose != it->d->robotPoses.end() )
		ASSERT_( trgPose != it->d->robotPoses.end() )

		*itP->d = trgPose->second - srcPose->second;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinateOrigin
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::changeCoordinateOrigin( const TPoseID &newOrigin )
{
	CPose3DPDFParticles		originPDF( m_particles.size() );

	CParticleList::iterator it;
	CPose3DPDFParticles::CParticleList::iterator	itOrgPDF;

	for ( it = m_particles.begin(), itOrgPDF=originPDF.m_particles.begin(); it!=m_particles.end(); it++, itOrgPDF++ )
	{
		TMapPoseID2Pose3D::iterator  refPoseIt =  it->d->robotPoses.find( newOrigin );
		ASSERT_( refPoseIt != it->d->robotPoses.end() )
		const CPose3D  &refPose = refPoseIt->second;

		// Save in pdf to compute mean:
		*itOrgPDF->d = refPose;
		itOrgPDF->log_w = it->log_w;

		TMapPoseID2Pose3D::iterator   End = it->d->robotPoses.end();
		// Change all other poses first:
		for (TMapPoseID2Pose3D::iterator  itP=it->d->robotPoses.begin();itP!=End;++itP)
			if (itP!=refPoseIt)
				itP->second = itP->second - refPose;

		// Now set new origin to 0:
		refPoseIt->second.setFromValues(0,0,0);
	}

	// Rebuild metric maps for consistency:
	rebuildMetricMaps();

	// Change coords in incr. partitioning as well:
	{
		synch::CCriticalSectionLocker locker ( &m_robotPosesGraph.lock );

		CSimpleMap *SFseq = m_robotPosesGraph.partitioner.getSequenceOfFrames();
		for (std::map<uint32_t,TPoseID>::const_iterator it=m_robotPosesGraph.idx2pose.begin();it!=m_robotPosesGraph.idx2pose.end();++it)
		{
			CPose3DPDFPtr		pdf;
			CSensoryFramePtr	sf;
			SFseq->get( it->first, pdf, sf);

			// Copy from particles:
			ASSERT_( pdf->GetRuntimeClass() == CLASS_ID(CPose3DPDFParticles) );
			CPose3DPDFParticlesPtr pdfParts = CPose3DPDFParticlesPtr(pdf);
			getPoseParticles( it->second, *pdfParts);
		}
	}
}

/*---------------------------------------------------------------
						rebuildMetricMaps
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::rebuildMetricMaps()
{
	for ( CParticleList::iterator it = m_particles.begin(); it!=m_particles.end(); ++it )
	{
		it->d->metricMaps.clear();

		// Follow all robot poses:
		TMapPoseID2Pose3D::iterator   End = it->d->robotPoses.end();
		for (TMapPoseID2Pose3D::iterator  itP=it->d->robotPoses.begin();itP!=End;++itP)
		{
			if ( itP->first != m_currentRobotPose )  // Current robot pose has no SF stored.
			{
				std::map<TPoseID,CSensoryFrame>::const_iterator SFit = m_SFs.find( itP->first );
				ASSERT_(SFit!=m_SFs.end());
				SFit->second.insertObservationsInto( &it->d->metricMaps, &itP->second );
			}
		}
	}
}


/** Removes a given area from the LMH:
  *	- The corresponding node in the HMT map is updated with the robot poses & SFs in the LMH.
  *	- Robot poses belonging to that area are removed from:
  *		- the particles.
  *		- the graph partitioner.
  *		- the list of SFs.
  *		- the list m_nodeIDmemberships.
  *		- The weights of all particles are changed to remove the effects of the removed metric observations.
  *	- After calling this the metric maps should be updated.
  */
void CLocalMetricHypothesis::removeAreaFromLMH( const CHMHMapNode::TNodeID areaID )
{
	MRPT_START

	// Remove from m_neighbors:
	// -----------------------------------
	TNodeIDSet::iterator itNeig = m_neighbors.find(areaID);
	if (itNeig!=m_neighbors.end() )
		m_neighbors.erase(itNeig);

	// Build the list with the poses in the area to be removed from LMH:
	// ----------------------------------------------------------------------
	TNodeIDList   lstPoseIDs;
	for (map<TPoseID,CHMHMapNode::TNodeID>::iterator it=m_nodeIDmemberships.begin();it!=m_nodeIDmemberships.end();++it)
		if ( it->second==areaID )
			lstPoseIDs.insert(it->first);

	ASSERT_( !lstPoseIDs.empty() );

	// ----------------------------------------------------------------------
	// The corresponding node in the HMT map is updated with the
	//   robot poses & SFs in the LMH.
	// ----------------------------------------------------------------------
	updateAreaFromLMH( areaID, true );

	// - Robot poses belonging to that area are removed from:
	// 	- the particles.
	// ----------------------------------------------------------------------
	for (TNodeIDList::const_iterator it=lstPoseIDs.begin();it!=lstPoseIDs.end();++it)
		for ( CParticleList::iterator p = m_particles.begin(); p!=m_particles.end(); ++p)
			p->d->robotPoses.erase( p->d->robotPoses.find( *it ) );

	// - The weights of all particles are changed to remove the effects of the removed metric observations.
	// ----------------------------------------------------------------------
	{
		CParticleList::iterator		p;
		vector<map<TPoseID,double> >::iterator  ws_it;
		ASSERT_( m_log_w_metric_history.size() == m_particles.size() );

		for ( ws_it=m_log_w_metric_history.begin(),p = m_particles.begin();
				p!=m_particles.end();
			   ++p,++ws_it)
		{
			for (TNodeIDList::const_iterator it=lstPoseIDs.begin();it!=lstPoseIDs.end();++it)
			{
				map<TPoseID,double>::iterator itW = ws_it->find(*it);
				if (itW!=ws_it->end())
				{
					MRPT_CHECK_NORMAL_NUMBER( itW->second );

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
		synch::CCriticalSectionLocker locker ( &m_robotPosesGraph.lock );

		vector_uint	indexesToRemove;
		indexesToRemove.reserve( lstPoseIDs.size() );

		for ( std::map<uint32_t,TPoseID>::iterator it=m_robotPosesGraph.idx2pose.begin();it!=m_robotPosesGraph.idx2pose.end(); )
		{
			if (lstPoseIDs.find(it->second)!=lstPoseIDs.end())
			{
				indexesToRemove.push_back( it->first );

				// Remove from the mapping indexes->nodeIDs as well:
				std::map<uint32_t,TPoseID>::iterator it2 = it; it2++;
				//it = m_robotPosesGraph.idx2pose.erase( it );
				m_robotPosesGraph.idx2pose.erase( it );
				it = it2;
			}
			else it++;
		}

		m_robotPosesGraph.partitioner.removeSetOfNodes(indexesToRemove);

		// Renumbering of indexes<->posesIDs to be the same than in  "m_robotPosesGraph.partitioner":
		unsigned idx = 0;
		map<uint32_t,TPoseID>		newList;
		for (map<uint32_t,TPoseID>::iterator i=m_robotPosesGraph.idx2pose.begin();i!=m_robotPosesGraph.idx2pose.end();++i,idx++)
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
	for (TNodeIDList::const_iterator it=lstPoseIDs.begin();it!=lstPoseIDs.end();++it)
		m_nodeIDmemberships.erase( m_nodeIDmemberships.find( *it ) );


	double out_max_log_w ;
	normalizeWeights( &out_max_log_w );	// Normalize weights:

	MRPT_END
}

/*---------------------------------------------------------------
					TRobotPosesPartitioning::pose2idx
 ---------------------------------------------------------------*/
unsigned int CLocalMetricHypothesis::TRobotPosesPartitioning::pose2idx(const TPoseID &id) const
{
	for (std::map<uint32_t,TPoseID>::const_iterator it=idx2pose.begin();it!=idx2pose.end();++it)
		if (it->second == id)
			return it->first;
	THROW_EXCEPTION_CUSTOM_MSG1("PoseID=%i not found.", static_cast<int>(id) );
}

/*---------------------------------------------------------------
					updateAreaFromLMH

// The corresponding node in the HMT map is updated with the
//   robot poses & SFs in the LMH.
 ---------------------------------------------------------------*/
void CLocalMetricHypothesis::updateAreaFromLMH(
	const CHMHMapNode::TNodeID areaID,
	bool eraseSFsFromLMH )
{
	// Build the list with the poses belonging to that area from LMH:
	// ----------------------------------------------------------------------
	TNodeIDList   lstPoseIDs;
	for (map<TPoseID,CHMHMapNode::TNodeID>::const_iterator it=m_nodeIDmemberships.begin();it!=m_nodeIDmemberships.end();++it)
		if ( it->second==areaID )
			lstPoseIDs.insert(it->first);

	ASSERT_( !lstPoseIDs.empty() );

	CHMHMapNodePtr node;
	{
		synch::CCriticalSectionLocker  ( &m_parent->m_map_cs );
		node = m_parent->m_map.getNodeByID( areaID );
		ASSERT_(node);
		ASSERT_(node->m_hypotheses.has( m_ID ));
	} // end of HMT map cs

	// The pose to become the origin:
	TPoseID		poseID_origin;
	node->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID,poseID_origin, m_ID, true);

	// 1) The set of robot poses and SFs
	//    In annotation: 					NODE_ANNOTATION_POSES_GRAPH
	// ---------------------------------------------------------------------
	CRobotPosesGraphPtr posesGraph;
	{
		CSerializablePtr annot = node->m_annotations.get(NODE_ANNOTATION_POSES_GRAPH,m_ID);
		if (!annot)
		{
			// Add it now:
			posesGraph = CRobotPosesGraph::Create();
			node->m_annotations.setMemoryReference(NODE_ANNOTATION_POSES_GRAPH, posesGraph, m_ID);
		}
		else
		{
			posesGraph = CRobotPosesGraphPtr(annot);
			posesGraph->clear();
		}
	}

	// For each pose in the area:
	CPose3DPDFParticles		pdfOrigin;
	bool					pdfOrigin_ok=false;
	for (TNodeIDList::const_iterator it=lstPoseIDs.begin();it!=lstPoseIDs.end();++it)
	{
		TPoseInfo &poseInfo = (*posesGraph)[*it];
		getPoseParticles( *it, poseInfo.pdf );    // Save pose particles

		// Save the PDF of the origin:
		if (*it==poseID_origin)
		{
			pdfOrigin.copyFrom( poseInfo.pdf );
			pdfOrigin_ok = true;
		}

		if ( *it != m_currentRobotPose )  // The current robot pose has no SF
		{
			std::map<TPoseID,CSensoryFrame>::iterator itSF = m_SFs.find(*it);
			ASSERT_(itSF!=m_SFs.end());

			if (eraseSFsFromLMH)
			{
				poseInfo.sf.moveFrom( itSF->second );   // This leaves m_SFs[*it] without observations, but it is being erased just now:
				m_SFs.erase( itSF );
			}
			else
			{
				poseInfo.sf = itSF->second;  // Copy observations
			}
		}
	}

	// Readjust to set the origin pose ID:
	ASSERT_(pdfOrigin_ok);
	CPose3DPDFParticles		pdfOriginInv;
	pdfOrigin.inverse(pdfOriginInv);
	for (CRobotPosesGraph::iterator it=posesGraph->begin();it!=posesGraph->end();++it)
	{
		CPose3DPDFParticles::CParticleList::iterator orgIt,pdfIt;
		ASSERT_( it->second.pdf.size() == pdfOriginInv.size() );
		for ( pdfIt=it->second.pdf.m_particles.begin(), orgIt= pdfOriginInv.m_particles.begin();orgIt!=pdfOriginInv.m_particles.end();orgIt++,pdfIt++)
			*pdfIt->d = *orgIt->d + *pdfIt->d;
	}

	// 2) One single metric map built from the most likelily robot poses
	//    In annotation: 					NODE_ANNOTATION_METRIC_MAPS
	// ---------------------------------------------------------------------
	CMultiMetricMapPtr metricMap = node->m_annotations.getAs<CMultiMetricMap>(NODE_ANNOTATION_METRIC_MAPS,m_ID, false);
	metricMap->clear();
	posesGraph->insertIntoMetricMap( *metricMap );

}

/*---------------------------------------------------------------
				dumpAsText
  ---------------------------------------------------------------*/
void  CLocalMetricHypothesis::dumpAsText(utils::CStringList &st) const
{
	st.clear();
	st << "LIST OF POSES IN LMH";
	st << "====================";

	string s;
	s = "Neighbors: ";
	for (TNodeIDSet::const_iterator it=m_neighbors.begin();it!=m_neighbors.end();++it)
		s+=format("%i ",(int)*it);
	st << s;

	TMapPoseID2Pose3D lst;
	getMeans( lst );

	for (TMapPoseID2Pose3D::const_iterator it=lst.begin();it!=lst.end();++it)
	{
		map<TPoseID,CHMHMapNode::TNodeID>::const_iterator	area = m_nodeIDmemberships.find(it->first);

		string s = format("  ID: %i \t AREA: %i \t %.03f,%.03f,%.03fdeg",
			(int)it->first,
			(int)area->second,
			it->second.x(),it->second.y(),RAD2DEG(it->second.yaw()) );
		st << s;
	}
}


/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLocalMetricHypothesis::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in  >> m_ID >> m_currentRobotPose
				>> m_neighbors
				>> m_nodeIDmemberships
				>> m_SFs
				>> m_posesPendingAddPartitioner
				>> m_areasPendingTBI
				>> m_log_w
				>> m_log_w_metric_history
				>> m_robotPosesGraph.partitioner
				>> m_robotPosesGraph.idx2pose;

			// particles:
			readParticlesFromStream(in);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CLocalMetricHypothesis::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_ID << m_currentRobotPose
			<< m_neighbors
			<< m_nodeIDmemberships
			<< m_SFs
			<< m_posesPendingAddPartitioner
			<< m_areasPendingTBI
			<< m_log_w
			<< m_log_w_metric_history
			<< m_robotPosesGraph.partitioner
			<< m_robotPosesGraph.idx2pose;

        // particles:
        writeParticlesToStream(out);

	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLSLAMParticleData::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
            in >> metricMaps >> robotPoses;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CLSLAMParticleData::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << metricMaps << robotPoses;
    }
}
