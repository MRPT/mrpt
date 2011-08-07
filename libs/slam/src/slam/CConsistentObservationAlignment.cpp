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


#include <mrpt/slam/CConsistentObservationAlignment.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CMemoryStream.h>

using namespace mrpt;
using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;
using namespace mrpt::utils;


/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CConsistentObservationAlignment::CConsistentObservationAlignment() :
	options()
{
	options.gridInsertOptions.maxOccupancyUpdateCertainty = 1.0f;
}

/*---------------------------------------------------------------
						execute
  ---------------------------------------------------------------*/
void  CConsistentObservationAlignment::execute(
		CSimpleMap		&inputMap,
		CSimpleMap		&outputMap )
{
	MRPT_START

	// A matrix of gaussian prob. poses: Used to store both,
	//   the estimated Dij and Cij covariances matrices.
	std::vector<vector_posesPdf>	DCij;
	std::vector<vector_posesPdf>	DCij_dif;	// Just the diferences between ICP estimation
												//  and map stored one, in global coordinates
	int								i,j,n;
	CICP							icp;
	CPose3DPDFPtr					pose_i,pose_j;
	CSensoryFramePtr				sf_i,sf_j;
	CMatrixD						G,B;
	std::vector<CMultiMetricMap*>	maps;

	// Clear the output:
	outputMap.clear();

	// The number of nodes is "n+1"
	n = inputMap.size() - 1;

	G.setSize(n*3,n*3);		// Remember!! nodes count = n+1
	CMatrixDouble33		m33;

	// If the map has only ONE observation, we can not do anymore:
	if (n<1)
	{
		CMemoryStream	aux;
		aux << inputMap;
		aux.Seek(0, CStream::sFromBeginning);
		aux >> outputMap;
		return;
	}

	// Build the local points maps for each sensory frame:
	// ----------------------------------------------------------------------
	maps.resize(n+1,NULL);		// Alloc mem. for the points map:

	// ---------------------------------------------
	//				Set map options:
	// ---------------------------------------------
	TSetOfMetricMapInitializers			mapInitializer;
	TMetricMapInitializer				mapElement;

	mapElement.metricMapClassType = CLASS_ID( CSimplePointsMap );
	mapElement.pointsMapOptions_options.insertionOpts = options.pointsMapOptions;
	mapInitializer.push_back( mapElement );

	if (options.matchAgainstGridmap)
	{
		mapElement.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
		mapElement.occupancyGridMap2D_options.resolution = options.gridMapsResolution;
		mapElement.occupancyGridMap2D_options.insertionOpts = options.gridInsertOptions;
		mapInitializer.push_back( mapElement );
	}

	for (i=0;i<=n;i++)
	{
		bool	atLeastOne = false;
		inputMap.get(i, pose_i,sf_i);

		// Create maps:
		maps[i] = new CMultiMetricMap( &mapInitializer );

		atLeastOne = sf_i->insertObservationsInto( maps[i] );

		//maps[i]->gridMap->saveAsBitmapFile("debugMap.bmp");

		ASSERT_(atLeastOne);
	}

	/*
	maps[i]: (n+1)x1 , The "n+1" points map representation of nodes
	DCij[i][j]: [0,n]x[0,n] , The ICP-derived "links" between nodes.
						but only elements at the right of diagonal are used.
	*/

	// Compute the gauss. probabilistic pose of each pair of observations,
	//  based on the ICP algorithm: (Fill the DCij matrix)
	// ----------------------------------------------------------------------
	// Alloc mem. for the gauss. prob. poses:
	DCij.resize(n+1);
	DCij_dif.resize(n+1);
	for (i=0;i<=n;i++)
	{
		DCij[i].resize(n+1);
		for (int k=0;k<=n;k++)
			DCij[i][k] = CPosePDFGaussian::Create();

		DCij_dif[i].resize(n+1);
		for (int k=0;k<=n;k++)
			DCij_dif[i][k] = CPosePDFGaussian::Create();
	}

	CMatrixDouble		debug_C(3*(n+1),3*(n+1));
	debug_C.zeros();

	std::vector<bool>	atLeastOneStrongLink;

	atLeastOneStrongLink.resize(n+1,false);

	// ---------------------------------------------
	//				Set ICP options:
	// ---------------------------------------------
	icp.options = options.icpOptions;

	for (i=0;i<n;i++)
	{
		printf_debug("Computing aligment, row %u/%u...\n",i,n);

		//printf(
		inputMap.get(i, pose_i,sf_i);

		for (j=i+1;j<=n;j++)
		{
			inputMap.get(j, pose_j,sf_j);

			// The initial "gross estimated" pose of "i" relative to "j":
			// -------------------------------------------------------------
			TPose2D		D_initial = TPose2D( CPose2D( pose_j->getMeanVal() - pose_i->getMeanVal() ) );

			CICP::TReturnInfo	icpInfo;


			// Perform the alignment:
			// -------------------------------
			CMetricMap	*alignWith;

			if (options.matchAgainstGridmap)
			{
				ASSERT_(maps[i]->m_gridMaps.size());
				alignWith = static_cast<CMetricMap*>(maps[i]->m_gridMaps[0].pointer());
			}
			else
			{
				ASSERT_(maps[i]->m_pointsMaps.size());
				alignWith = static_cast<CMetricMap*>(maps[i]->m_pointsMaps[0].pointer());
			}

			ASSERT_(maps[j]->m_pointsMaps.size());

            CPosePDFPtr icpEst = icp.Align(
                alignWith,
                maps[j]->m_pointsMaps[0].pointer(),
                D_initial,
                NULL,
                &icpInfo );

			icpEst->getMean( DCij[j][i]->mean );
            icpEst->getCovariance( DCij[j][i]->cov );


			float orientationChange = fabs( DCij[j][i]->mean.phi() - D_initial.phi );

			// The "D_estimated" is in corrdinates of "i", rotate to
			//   accomodate to global coordinates.
			// -----------------------------------------------------------
			if (icpInfo.goodness<0.6 || orientationChange>DEG2RAD(15))
			{
				// Not enought matching to take it into account:
				// -------------------------------------------------
				DCij[j][i]->mean = D_initial;
				DCij[j][i]->cov.setIdentity();
				DCij[j][i]->cov(0,0) = 100.0f;
				DCij[j][i]->cov(1,1) = 100.0f;
				DCij[j][i]->cov(2,2) = 1.0f;
			}
			else
			{
				// Ok, a good matching, keep the ICP result as valid.
				// -------------------------------------------------
				atLeastOneStrongLink[i] = true;
				atLeastOneStrongLink[j] = true;
				DCij[j][i]->cov.setIdentity();
				DCij[j][i]->cov(0,0) = 0.01f;
				DCij[j][i]->cov(1,1) = 0.01f;
				DCij[j][i]->cov(2,2) = 0.01f;
			}

			// Save difference in the relative poses:
			CPose2D		Dij_dif(
				DCij[j][i]->mean.x() - D_initial.x,
				DCij[j][i]->mean.y() - D_initial.y,
				DCij[j][i]->mean.phi() - D_initial.phi );

			/** DEBUG ** /
			printf("(%u,%u): Alignment %.1f%% (%.03f,%.03f,%.03fdeg)->(%.03f,%.03f,%.03f)\n",
						i,
						j,
						icpInfo.goodness*100,
						D_initial.x,
						D_initial.y,
						RAD2DEG(D_initial.phi),
						DCij[j][i]->mean.x,
						DCij[j][i]->mean.y,
						RAD2DEG(DCij[j][i]->mean.phi));
            / **/

			// Reference to global coordinates:
			TPose2D		Dij_global(Dij_dif - CPose2D( 0,0, -pose_i->getMeanVal().yaw() ) );
			Dij_global.phi = Dij_dif.phi(); //DCij[j][i]->mean.phi;

			/** DEBUG ** /
			printf("(%u,%u): Alignment %.1f%% dif=(%.03f,%.03f,%.03fdeg)\n",
						i,
						j,
						icpInfo.goodness*100,
						Dij_global.x,
						Dij_global.y,
						RAD2DEG(Dij_dif.phi));
			/ **/

			// Save in Dji
			DCij[j][i]->mean.x( Dij_global.x );
			DCij[j][i]->mean.y( Dij_global.y );
			DCij[j][i]->mean.phi( Dij_global.phi );

			// Dij = -Dji
			DCij[i][j]->cov = DCij[j][i]->cov;
			DCij[i][j]->mean.x( -Dij_global.x );
			DCij[i][j]->mean.y( -Dij_global.y );
			DCij[i][j]->mean.phi( -Dij_global.phi );
		}

	}
//	debug_C.saveToTextFile("C.txt");

	// Assert we have a connected graph:
	bool	errorDetected = false;
	for (i=0;i<n;i++)
	{
		if (!atLeastOneStrongLink[i])
		{
			errorDetected = true;
			printf_debug("execute: Node %i has no strong links!\n", i);
		}
	}
	ASSERT_(!errorDetected);

	// Compute the G matrix
	// ----------------------------------------------------------------------
	for (i=0;i<n;i++)
	{
		for (j=i;j<n;j++)
		{
			if (i==j)
			{
				m33.zeros();
				for (int k=0;k<=n;k++)
					if (k!=(i+1))
					{
						m33 += !DCij[i+1][k]->cov;
					}
			}
			else
			{
				m33 = !DCij[i+1][j+1]->cov;
				m33 *= -1;
			}

			// Copy submatrix to G:
			G(j*3+0,i*3+0) = m33(0,0); G(i*3+0,j*3+0) = m33(0,0);
			G(j*3+0,i*3+1) = m33(0,1); G(i*3+0,j*3+1) = m33(0,1);
			G(j*3+0,i*3+2) = m33(0,2); G(i*3+0,j*3+2) = m33(0,2);
			G(j*3+1,i*3+0) = m33(1,0); G(i*3+1,j*3+0) = m33(1,0);
			G(j*3+1,i*3+1) = m33(1,1); G(i*3+1,j*3+1) = m33(1,1);
			G(j*3+1,i*3+2) = m33(1,2); G(i*3+1,j*3+2) = m33(1,2);
			G(j*3+2,i*3+0) = m33(2,0); G(i*3+2,j*3+0) = m33(2,0);
			G(j*3+2,i*3+1) = m33(2,1); G(i*3+2,j*3+1) = m33(2,1);
			G(j*3+2,i*3+2) = m33(2,2); G(i*3+2,j*3+2) = m33(2,2);
		}

	}

//	G.saveToTextFile("G.txt");

	// Compute the B matrix
	// ----------------------------------------------------------------------
	B.setSize(n*3,1);
	CMatrixDouble31	m31;
	for (i=0;i<n;i++)
	{
		m31.zeros();

		for (j=0;j<=n;j++)
		{
			if ((i+1)!=j)
			{
				CMatrixDouble31	pose = CMatrixDouble31( DCij[i+1][j]->mean );

				m31 += (!DCij[i+1][j]->cov )*pose;
			}
		}

		// Copy sub-vector to B:
		B(i*3+0,0) = m31(0,0);
		B(i*3+1,0) = m31(1,0);
		B(i*3+2,0) = m31(2,0);
	}

//	B.saveToTextFile("B.txt");

	// Perform the consistent, global estimation:
	// ----------------------------------------------------------------------
	CMatrixD  CovX = G.inv();
	CMatrixD  X = CovX * B;

	// Generate the output map:
	// ----------------------------------------------------------------------
	outputMap.clear();
	double meanPosesChange = 0;
	for (i=0;i<=n;i++)
	{
		// Get sensory frame: this is never modified
		inputMap.get(i, pose_i,sf_i);

		printf_debug(
					"Node#%u: (%.03f,%.03f,%.03fdeg)->",
					i,
					pose_i->getMeanVal().x(),
					pose_i->getMeanVal().y(),
					RAD2DEG( pose_i->getMeanVal().yaw() ) );

		// Set new estimated pose:
		CPosePDFGaussian	nodeNewPose;
		if (i==0)
		{
			// Mean value:
			nodeNewPose.mean.x(0);
			nodeNewPose.mean.y(0);
			nodeNewPose.mean.phi(0);

			// And covariance matrix:
			nodeNewPose.cov.zeros();
		}
		else
		{
			int		i_1 = 3*(i-1);

			// Mean value:
//			nodeNewPose.mean.x = X(i_1+0,0);
//			nodeNewPose.mean.y = X(i_1+1,0);
//			nodeNewPose.mean.phi = X(i_1+2,0);
			nodeNewPose.mean.x( X(i_1+0,0) + pose_i->getMeanVal().x() );
			nodeNewPose.mean.y( X(i_1+1,0) + pose_i->getMeanVal().y() );
			nodeNewPose.mean.phi( X(i_1+2,0) + pose_i->getMeanVal().yaw() );

			nodeNewPose.mean.normalizePhi();

			// And covariance matrix:
			nodeNewPose.cov(0,0) = CovX(i_1+0,i_1+0);
			nodeNewPose.cov(0,1) = CovX(i_1+0,i_1+1);
			nodeNewPose.cov(0,2) = CovX(i_1+0,i_1+2);
			nodeNewPose.cov(1,0) = CovX(i_1+1,i_1+0);
			nodeNewPose.cov(1,1) = CovX(i_1+1,i_1+1);
			nodeNewPose.cov(1,2) = CovX(i_1+1,i_1+2);
			nodeNewPose.cov(2,0) = CovX(i_1+2,i_1+0);
			nodeNewPose.cov(2,1) = CovX(i_1+2,i_1+1);
			nodeNewPose.cov(2,2) = CovX(i_1+2,i_1+2);
		}

		// Add node to output map:
		outputMap.insert( &nodeNewPose, sf_i);

		printf_debug(
					" (%.03f,%.03f,%.03fdeg)",
					nodeNewPose.getMeanVal().x(),
					nodeNewPose.getMeanVal().y(),
					RAD2DEG(nodeNewPose.getMeanVal().phi() ));

		// Calculate mean poses change:
		double poseChange =
			sqrt( square(nodeNewPose.getMeanVal().x()-pose_i->getMeanVal().x())+
				square(nodeNewPose.getMeanVal().y()-pose_i->getMeanVal().y())+
				square(nodeNewPose.getMeanVal().phi()-pose_i->getMeanVal().yaw() ) );

		printf_debug(" chang=%f\n",poseChange );

		meanPosesChange+= poseChange;
	}

	meanPosesChange/= (n+1);

	printf_debug("Mean change in map poses: %f\n", meanPosesChange);

	// Free the mem. of local maps:
	for (std::vector<CMultiMetricMap*>::iterator	it=maps.begin();it!=maps.end();++it)
		delete	*it;

	maps.clear();

	MRPT_END
}

/*---------------------------------------------------------------
					TOptions
  ---------------------------------------------------------------*/
CConsistentObservationAlignment::TOptions::TOptions() :
	matchAgainstGridmap ( true ),
	gridMapsResolution  ( 0.03f),
	pointsMapOptions(),
	gridInsertOptions(),
	icpOptions()
{
}

/*---------------------------------------------------------------
 This alternate method provides the basic consistent alignment
 algorithm to any user-supplied matrix of pose constrainsts,
 returning the optimal poses of all the nodes relative to
 the first one.

	in_PoseConstraints This is a NxN matrix where element M(i,j) is the pose constrainst between node "i" and "j". Please, fill out only the upper-triangle part of the matrix (diagonal and lowe-part entries are not used).
	out_OptimalPoses The 1xN vector with the consistent global poses of all nodes, where the first node is always at (0,0,0deg).
  ---------------------------------------------------------------*/
void  CConsistentObservationAlignment::optimizeUserSuppliedData(
			CMatrixTemplateObjects<CPosePDFGaussian>		&in_PoseConstraints,
			CMatrixTemplateObjects<CPosePDFGaussian>		&out_OptimalPoses )
{
	CMatrixD						G,B;

	MRPT_START

	int								i,j,n;

	ASSERT_( in_PoseConstraints.getColCount() == in_PoseConstraints.getRowCount() );
	// The number of nodes is "n+1"
	n = in_PoseConstraints.getColCount() - 1;

	// Clear the output:
	out_OptimalPoses.setSize(1,n+1);
	out_OptimalPoses.freeAllObjects();
	out_OptimalPoses.allocAllObjects();


	G.setSize(n*3,n*3);		// Remember!! nodes count = n+1
	CMatrixDouble33		m33;

	// If the map has only ONE observation, we can not do anymore:
	if (n<1)
	{
		out_OptimalPoses(0,0)->mean = CPose2D(0,0,0);
		out_OptimalPoses(0,0)->cov.setConstant(0);
		return;
	}

	/*
 	 	in_PoseConstraints[i][j]: [0,n]x[0,n] , The "links/edges" between nodes:
						(only elements at the right of diagonal are used)
	*/

	for (i=0;i<n;i++)
	{
		for (j=i+1;j<=n;j++)
		{
			// Dij = -Dji
			ASSERT_( in_PoseConstraints(j,i)->cov.getColCount()==3 );  // This is a 2D algorithm!

			in_PoseConstraints(j,i)->cov = in_PoseConstraints(i,j)->cov;
			in_PoseConstraints(j,i)->mean.x( -in_PoseConstraints(i,j)->mean.x() );
			in_PoseConstraints(j,i)->mean.y( -in_PoseConstraints(i,j)->mean.y() );
			in_PoseConstraints(j,i)->mean.phi( -in_PoseConstraints(i,j)->mean.phi() );
		} // end for j
	}	// end for i

	// Compute the G matrix
	// ----------------------------------------------------------------------
	for (i=0;i<n;i++)
	{
		for (j=i;j<n;j++)
		{
			if (i==j)
			{
				m33.zeros();
				for (int k=0;k<=n;k++)
					if (k!=(i+1))
					{
						m33 += !in_PoseConstraints(i+1,k)->cov;
					}
			}
			else
			{
				m33 = !in_PoseConstraints(i+1,j+1)->cov;
				m33 *= -1;
			}

			// Copy submatrix to G:
			G.insertMatrix(j*3,i*3, m33);
			G.insertMatrixTranspose(i*3,j*3, m33);
		}
	}

	// Compute the B matrix
	// ----------------------------------------------------------------------
	B.setSize(n*3,1);
	CMatrixDouble31		m31;
	for (i=0;i<n;i++)
	{
		m31.zeros();

		for (j=0;j<=n;j++)
		{
			if ((i+1)!=j)
			{
				CMatrixDouble31	pose = CMatrixDouble31(in_PoseConstraints(i+1,j)->mean);
				pose *= -1;
				CMatrixDouble33	cov2 = in_PoseConstraints(i+1,j)->cov;

				CMatrixDouble33	aux;
				cov2.inv_fast(aux);	// aux = ((!cov2)*pose);

				m31 += aux * pose;
			}
		}

		// Copy sub-vector to B:
		B.insertMatrix(i*3,0, m31);
	}


	// Perform the consistent, global estimation:
	// ----------------------------------------------------------------------
	CMatrixD		CovX ( G.inv() );
	CMatrixD		X; X.multiply(CovX,B);

	// Generate the output poses:
	// ----------------------------------------------------------------------
	for (i=0;i<=n;i++)
	{
		// Set new estimated pose:
		CPosePDFGaussian	nodeNewPose;
		if (i==0)
		{
			// Mean value:
			out_OptimalPoses(0,i)->mean = CPose2D(0,0,0);

			// And covariance matrix:
			out_OptimalPoses(0,i)->cov.zeros();
		}
		else
		{
			size_t		i_1 = 3*(i-1);

			// Mean value:
			out_OptimalPoses(0,i)->mean.x(  X(i_1+0,0) );
			out_OptimalPoses(0,i)->mean.y( X(i_1+1,0) );
			out_OptimalPoses(0,i)->mean.phi(  X(i_1+2,0) );
			out_OptimalPoses(0,i)->mean.normalizePhi();

			// And covariance matrix:
			CovX.extractMatrix(i_1,i_1,  out_OptimalPoses(0,i)->cov );
		}
	}

	MRPT_END_WITH_CLEAN_UP( G.saveToTextFile("debug_G.txt"); B.saveToTextFile("debug_B.txt"); );
}
