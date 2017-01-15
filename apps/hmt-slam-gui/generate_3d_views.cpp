/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "hmt_slam_guiMain.h"
#include <mrpt/opengl.h>

#include <wx/msgdlg.h>

//(*InternalHeaders(hmt_slam_guiFrame)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/tglbtn.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)


using namespace std;
using namespace mrpt;
using namespace mrpt::hmtslam;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::math;

void hmt_slam_guiFrame::updateLocalMapView()
{
    WX_START_TRY

	m_glLocalArea->m_openGLScene->clear();

	// Get the hypothesis ID:
	THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );
	if ( m_hmtslam->m_LMHs.find(hypID)==m_hmtslam->m_LMHs.end() )
	{
		wxMessageBox(_U( format("No LMH has hypothesis ID %i!", (int)hypID).c_str() ), _("Error with topological hypotesis"));
		return;
	}

	// Get the selected area or LMH in the tree view:
	wxArrayTreeItemIds	lstSelect;
	size_t	nSel = treeView->GetSelections(lstSelect);
	if (!nSel) return;

	CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(0) ) );
	if (!data1) return;
	if (!data1->m_ptr) return;

	CSerializablePtr obj = data1->m_ptr;
	if (obj->GetRuntimeClass()==CLASS_ID(CHMHMapNode))
	{
		// The 3D view:
		opengl::CSetOfObjectsPtr objs = opengl::CSetOfObjects::Create();

		// -------------------------------------------
		// Draw a grid on the ground:
		// -------------------------------------------
		{
			opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-100,100,-100,100,0,5);
			obj->setColor(0.4,0.4,0.4);
			objs->insert(obj);  // it will free the memory
		}


		// Two passes: 1st draw the map on the ground, then the rest.
		for (int nRound=0;nRound<2;nRound++)
		{
			CHMHMapNodePtr firstArea;
			CPose3DPDFGaussian		refPoseThisArea;

			for (size_t  nSelItem = 0; nSelItem<nSel;nSelItem++)
			{
				CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(nSelItem) ) );
				if (!data1) continue;
				if (!data1->m_ptr) continue;

				CHMHMapNodePtr area= CHMHMapNodePtr(data1->m_ptr);
				if (!area) continue;

				// Is this the first rendered area??
				if ( !firstArea )
				{
					firstArea = area;
				}
				else
				{
					// Compute the translation btw. ref. and current area:
					CPose3DPDFParticles		pdf;

					m_hmtslam->m_map.computeCoordinatesTransformationBetweenNodes(
						firstArea->getID(),
						area->getID(),
						pdf,
						hypID,
						200 );
						/*0.15f,
						DEG2RAD(5.0f) );*/

					refPoseThisArea.copyFrom( pdf );
					cout << "Pose " << firstArea->getID() << " - " << area->getID() << refPoseThisArea << endl;
				}

				CMultiMetricMapPtr obj_mmap = area->m_annotations.getAs<CMultiMetricMap>( NODE_ANNOTATION_METRIC_MAPS, hypID, false );

				CRobotPosesGraphPtr obj_robposes = area->m_annotations.getAs<CRobotPosesGraph>( NODE_ANNOTATION_POSES_GRAPH, hypID, false );

				TPoseID	refPoseID;
				area->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, refPoseID, hypID, true);

				// ---------------------------------------------------------
				// The metric map:
				// ---------------------------------------------------------
				if (nRound==0)
				{
					opengl::CSetOfObjectsPtr objMap= opengl::CSetOfObjects::Create();
					obj_mmap->getAs3DObject(objMap);
					objMap->setPose( refPoseThisArea.mean );
					objs->insert(objMap);
				}

				if (nRound==1)
				{
					// ---------------------------------------------------------
					// Bounding boxes for grid maps:
					// ---------------------------------------------------------
					if (obj_mmap->m_gridMaps.size())
					{
						float x_min = obj_mmap->m_gridMaps[0]->getXMin();
						float x_max = obj_mmap->m_gridMaps[0]->getXMax();
						float y_min = obj_mmap->m_gridMaps[0]->getYMin();
						float y_max = obj_mmap->m_gridMaps[0]->getYMax();

						opengl::CSetOfLinesPtr objBB = opengl::CSetOfLines::Create();
						objBB->setColor(0,0,1);
						objBB->setLineWidth( 4.0f );

						objBB->appendLine(x_min,y_min,0,  x_max,y_min,0);
						objBB->appendLine(x_max,y_min,0,  x_max,y_max,0);
						objBB->appendLine(x_max,y_max,0,  x_min,y_max,0);
						objBB->appendLine(x_min,y_max,0,  x_min,y_min,0);

						objBB->setPose( refPoseThisArea.mean );
						objs->insert(objBB);
					}

					// -----------------------------------------------
					// Draw a 3D coordinates corner for the ref. pose
					// -----------------------------------------------
					{
						CPose3D	p;
						(*obj_robposes)[refPoseID].pdf.getMean(p);

						opengl::CSetOfObjectsPtr corner = stock_objects::CornerXYZ();
						corner->setPose( refPoseThisArea.mean + p);
						corner->setName(format("AREA %i",(int)area->getID() ));
						corner->enableShowName();

						objs->insert( corner );
					}

					// -----------------------------------------------
					// Draw ellipsoid with uncertainty of pose transformation
					// -----------------------------------------------
					if (refPoseThisArea.cov(0,0)!=0 || refPoseThisArea.cov(1,1)!=0)
					{
						opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();
						ellip->setPose( refPoseThisArea.mean );
						ellip->enableDrawSolid3D(false);

						CMatrixDouble C = CMatrixDouble(refPoseThisArea.cov);

						if (C(2,2)<1e6)
								C.setSize(2,2);
						else	C.setSize(3,3);

						ellip->setCovMatrix(C);
						ellip->setQuantiles(3);
						ellip->setLocation( ellip->getPoseX(), ellip->getPoseY(), ellip->getPoseZ()+0.5);
						ellip->setColor(1,0,0);
						ellip->setLineWidth(3);

						objs->insert( ellip );
					}

					// ---------------------------------------------------------
					// Draw each of the robot poses as 2D/3D ellipsoids
					// ---------------------------------------------------------
					for (CRobotPosesGraph::iterator it=obj_robposes->begin();it!=obj_robposes->end();++it)
					{

					}
				}

			} // end for nSelItem

		} // two pass

		// Add to the scene:
		m_glLocalArea->m_openGLScene->insert(objs);
	}
	else
	if (obj->GetRuntimeClass()==CLASS_ID( CLocalMetricHypothesis ))
	{
		//CLocalMetricHypothesis *lmh = static_cast<CLocalMetricHypothesis*>( obj );

	}

	m_glLocalArea->Refresh();

	WX_END_TRY
}

void hmt_slam_guiFrame::updateGlobalMapView()
{
	m_glGlobalHMTMap->m_openGLScene->clear();

	wxBusyCursor busy;


	// Dump text representation to log window:
	{
		CStringList	strLst;
		m_hmtslam->m_map.dumpAsText( strLst );
		string str;
		strLst.getText( str );
		cout << str << endl;
	}

	if (m_hmtslam->m_map.getFirstNode())
	{
		CHMHMapNode::TNodeID	refID = m_hmtslam->m_map.getFirstNode()->getID();

		THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );

		if ( m_hmtslam->m_LMHs.find(hypID)==m_hmtslam->m_LMHs.end() )
		{
			wxMessageBox(_U( format("No LMH has hypothesis ID %i!", (int)hypID).c_str() ), _("Error with topological hypotesis"));
			return;
		}
//		cout << "Showing hypothesis ID: " << hypID  << endl;

		m_hmtslam->m_map.getAs3DScene(
			*m_glGlobalHMTMap->m_openGLScene,
			refID,
			hypID );

		m_glGlobalHMTMap->Refresh();
	}
}
