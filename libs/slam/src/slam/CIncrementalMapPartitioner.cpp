/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/slam/observations_overlap.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/graphs/CGraphPartitioner.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/stl_serialization.h>
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
using namespace mrpt::utils;
using namespace mrpt;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CIncrementalMapPartitioner, CSerializable,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::CIncrementalMapPartitioner( ) :
	COutputLogger("CIncrementalMapPartitioner"),
	options(),
	m_individualFrames(),
	m_individualMaps(),
	m_A(0,0),
	m_last_partition(),
	m_last_last_partition_are_new_ones(false),
	m_modified_nodes()
{
	clear();
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::~CIncrementalMapPartitioner()
{
	clear();
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::TOptions::TOptions() :
	partitionThreshold				( 1.0f ),
	gridResolution					( 0.10f ),
	minDistForCorrespondence		( 0.20f ),
	minMahaDistForCorrespondence	( 2.0f ),
	forceBisectionOnly				( false ),
	useMapMatching				    ( true ),
	minimumNumberElementsEachCluster( 1 )
{
}

/*---------------------------------------------------------------
						loadFromConfigFile
  ---------------------------------------------------------------*/
void  CIncrementalMapPartitioner::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const string		&section)
{
	MRPT_START

	MRPT_LOAD_CONFIG_VAR(partitionThreshold,		  float,source,section);
	MRPT_LOAD_CONFIG_VAR(gridResolution,		      float,source,section);
	MRPT_LOAD_CONFIG_VAR(minDistForCorrespondence,	  float,source,section);
	MRPT_LOAD_CONFIG_VAR(forceBisectionOnly,          bool,source,section);
	MRPT_LOAD_CONFIG_VAR(useMapMatching,		      bool,source,section);
	MRPT_LOAD_CONFIG_VAR(minimumNumberElementsEachCluster, int, source,section);


	MRPT_END
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void  CIncrementalMapPartitioner::TOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CIncrementalMapPartitioner::TOptions] ------------ \n\n");

	out.printf("partitionThreshold                      = %f\n",partitionThreshold);
	out.printf("gridResolution                          = %f\n",gridResolution);
	out.printf("minDistForCorrespondence                = %f\n",minDistForCorrespondence);
	out.printf("forceBisectionOnly                      = %c\n",forceBisectionOnly ? 'Y':'N');
	out.printf("useMapMatching                          = %c\n",useMapMatching ? 'Y':'N');
	out.printf("minimumNumberElementsEachCluster        = %i\n",minimumNumberElementsEachCluster);
}


/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CIncrementalMapPartitioner::clear()
{
	m_last_last_partition_are_new_ones = false;

	m_A.setSize(0,0);

	m_individualFrames.clear();	// Free the map...

	// Free individual maps:
	//for (deque_serializable<mrpt::maps::CMultiMetricMap>::iterator it=m_individualMaps.begin();it!=m_individualMaps.end();++it)	delete (*it);
	m_individualMaps.clear();

	m_last_partition.clear();		// Delete last partitions
	m_modified_nodes.clear();		// Delete modified nodes
}

/*---------------------------------------------------------------
						addMapFrame
  ---------------------------------------------------------------*/
unsigned int CIncrementalMapPartitioner::addMapFrame(
	const CSensoryFramePtr &frame,
	const CPosePDFPtr &robotPose )
{
	MRPT_START
	return addMapFrame( frame,CPose3DPDFPtr( CPose3DPDF::createFrom2D( *robotPose ) ));
	MRPT_END
}


/*---------------------------------------------------------------
						addMapFrame
  ---------------------------------------------------------------*/
unsigned int CIncrementalMapPartitioner::addMapFrame(
	const CSensoryFramePtr &frame,
	const CPose3DPDFPtr &robotPose )
{
	size_t								i=0,j=0,n=0;
	CPose3D								pose_i, pose_j, relPose;
	CPose3DPDFPtr						posePDF_i, posePDF_j;
	CSensoryFramePtr					sf_i, sf_j;
	CMultiMetricMap						*map_i=NULL,*map_j=NULL;
	mrpt::utils::TMatchingPairList		corrs;
	static CPose3D						nullPose(0,0,0);

	// Create the maps:
	TSetOfMetricMapInitializers			mapInitializer;

	{
		CSimplePointsMap::TMapDefinition def;
		mapInitializer.push_back(def);
	}

	{
		CLandmarksMap::TMapDefinition def;
		mapInitializer.push_back(def);
	}

	// Add new metric map to "m_individualMaps"
	// --------------------------------------------
	m_individualMaps.push_back( CMultiMetricMap() );
	CMultiMetricMap		&newMetricMap = m_individualMaps.back();
	newMetricMap.setListOfMaps( &mapInitializer );

	MRPT_START

	// Create the metric map:
	// -----------------------------------------------------------------
	ASSERT_(newMetricMap.m_pointsMaps.size()>0);
	newMetricMap.m_pointsMaps[0]->insertionOptions.isPlanarMap		= false;  // true
	newMetricMap.m_pointsMaps[0]->insertionOptions.minDistBetweenLaserPoints = 0.20f;
	options.minDistForCorrespondence = max(options.minDistForCorrespondence,1.3f*newMetricMap.m_pointsMaps[0]->insertionOptions.minDistBetweenLaserPoints);

	TMatchingRatioParams mrp;
	mrp.maxDistForCorr = options.minDistForCorrespondence;
	mrp.maxMahaDistForCorr = options.minMahaDistForCorrespondence;

	// JLBC,17/AGO/2006: "m_individualMaps" were created from the robot pose, but it is
	//   more convenient now to save them as the robot being at (0,0,0).

	//frame->insertObservationsInto( newMetricMap.m_pointsMaps[0] );
	newMetricMap.m_pointsMaps[0]->copyFrom( * frame->buildAuxPointsMap<CPointsMap>(&newMetricMap.m_pointsMaps[0]->insertionOptions));	// Faster :-)

	// Insert just the VisualLandmarkObservations:
	mrpt::maps::CLandmarksMap &lm = *newMetricMap.m_landmarksMap;
	lm.insertionOptions.insert_SIFTs_from_monocular_images = false;
	lm.insertionOptions.insert_SIFTs_from_stereo_images    = false;
	lm.insertionOptions.insert_Landmarks_from_range_scans  = false;
	frame->insertObservationsInto( &lm );

	// Add to corresponding vectors:
	m_individualFrames.insert(robotPose, frame);
	// Already added to "m_individualMaps" above

	// Expand the adjacency matrix
	// -----------------------------------------------------------------
	n = m_A.getColCount();
	n++;
	m_A.setSize(n,n);

	ASSERT_(m_individualMaps.size() == n);
	ASSERT_(m_individualFrames.size() == n);

	// Adjust size of vector containing the modified nodes
	// ---------------------------------------------------
	// The new must be taken into account as well.
	m_modified_nodes.push_back(true);

	// Methods to compute adjacency matrix:
	// true:  matching between maps
	// false: matching between observations through "CObservation::likelihoodWith"
	// ------------------------------------------------------------------------------
	bool useMapOrSF = options.useMapMatching;

	// Calculate the new matches - put them in the matrix
	// ----------------------------------------------------------------
	//for (i=n-1;i<n;i++)
	i=n-1;   // Execute procedure until "i=n-1"; Last row/column is empty
	{
		// Get node "i":
		m_individualFrames.get(i, posePDF_i, sf_i);
		posePDF_i->getMean(pose_i);

		// And its points map:
		map_i = &m_individualMaps[i];

		for (j=0;j<n-1;j++)
		{
			// Get node "j":
			m_individualFrames.get(j, posePDF_j, sf_j);
			posePDF_j->getMean( pose_j );

			relPose = pose_j - pose_i;

			// And its points map:
			map_j = &m_individualMaps[j];

			// Compute matching ratio:
			if (useMapOrSF)
			{
				m_A(i,j) = map_i->compute3DMatchingRatio(map_j,relPose,mrp);
			}
			else
			{
				//m_A(i,j) = sf_i->likelihoodWith(sf_j.pointer());
				m_A(i,j) = observationsOverlap(sf_i, sf_j, &relPose );
			}

		} // for j

	} // for i


	for (i=0;i<n-1;i++) // Execute procedure until "i=n-1"; Last row/column is empty
	{
		// Get node "i":
		m_individualFrames.get(i, posePDF_i, sf_i);
		posePDF_i->getMean(pose_i);

		// And its points map:
		map_i = &m_individualMaps[i];

		j=n-1; //for (j=n-1;j<n;j++)
		{
			// Get node "j":
			m_individualFrames.get(j, posePDF_j, sf_j);
			posePDF_j->getMean(pose_j);

			relPose = pose_j - pose_i;

			// And its points map:
			map_j = &m_individualMaps[j];

			// Compute matching ratio:
			if (useMapOrSF)
			{
				m_A(i,j) = map_i->compute3DMatchingRatio(map_j,CPose3D(relPose),mrp);
			}
			else
			{
				//m_A(i,j) = sf_i->likelihoodWith(sf_j.pointer());
				m_A(i,j) = observationsOverlap(sf_i, sf_j, &relPose );
			}
		} // for j
	} // for i

	// Self-similatity: Not used
	m_A(n-1,n-1) = 0;

	// Hacer que la matriz sea simetrica:
	// -----------------------------------------------------------------
	for (i=0;i<n;i++)
		for (j=i+1;j<n;j++)
			m_A(i,j) = m_A(j,i) = 0.5f * (m_A(i,j) + m_A(j,i) );

	/* DEBUG: Save the matrix: * /
	A.saveToTextFile("debug_matriz.txt",1);
	/ **/

	// Add the affected nodes to the list of modified ones
	// -----------------------------------------------------------------
	for (i=0;i<n;i++)
		m_modified_nodes[i] = m_A(i,n-1) > 0;

	if (m_last_last_partition_are_new_ones)
	{
		// Insert into the "new_ones" partition:
		m_last_partition[m_last_partition.size()-1].push_back( n-1 );
	}
	else
	{
		// Add a new partition:
		vector_uint  dummyPart;
		dummyPart.push_back(n-1);
		m_last_partition.push_back( dummyPart );

		// The last one is the new_ones partition:
		m_last_last_partition_are_new_ones = true;
	}

	return n-1; // Index of the new node

	MRPT_END_WITH_CLEAN_UP( \
		cout << "Unexpected runtime error:\n"; \
		cout << "\tn=" << n << "\n"; \
		cout << "\ti=" << i << "\n"; \
		cout << "\tj=" << j << "\n"; \
		cout << "\tmap_i=" << map_i << "\n"; \
		cout << "\tmap_j=" << map_j << "\n"; \
		cout << "relPose: "<< relPose << endl; \
		cout << "map_i.size()=" << map_i->m_pointsMaps[0]->size() << "\n"; \
		cout << "map_j.size()=" << map_j->m_pointsMaps[0]->size() << "\n"; \
		map_i->m_pointsMaps[0]->save2D_to_text_file(string("debug_DUMP_map_i.txt")); \
		map_j->m_pointsMaps[0]->save2D_to_text_file(string("debug_DUMP_map_j.txt")); \
		m_A.saveToTextFile("debug_DUMP_exception_A.txt"); \
		);

}

/*---------------------------------------------------------------
						updatePartitions
  ---------------------------------------------------------------*/
  void CIncrementalMapPartitioner::updatePartitions(
	  vector<vector_uint> &partitions )
{
	MRPT_START

	unsigned int			i,j;
	unsigned int			n_nodes;
	unsigned int			n_clusters_last;
	vector_uint				mods;	// The list of nodes that will have been regrouped
	vector_bool				last_parts_are_mods;

	n_nodes = m_modified_nodes.size();			// total number of nodes (scans)
	n_clusters_last = m_last_partition.size();	// Number of clusters in the last partition

	last_parts_are_mods.resize( n_clusters_last );

	// If a single scan of the cluster is affected, the whole cluster is affected
	// -------------------------------------------------------------------
	for (i=0;i<n_clusters_last;i++)
	{
		vector_uint	p = m_last_partition[i];

		// Recorrer esta particion:
		last_parts_are_mods[i] = false;

//			for (j=0;j<p.size();j++)
//				if ( m_modified_nodes[ p[j] ] )
//					last_parts_are_mods[i] = true;

		last_parts_are_mods[i] = true;

		// If changed mark all the nodes
		if (last_parts_are_mods[i])
			for (j=0;j<p.size();j++)
				m_modified_nodes[ p[j] ] = true;
	}

	// How many nodes are going to be partitioned?
	mods.clear();
	for (i=0;i<n_nodes;i++)
		if ( m_modified_nodes[i] )
			mods.push_back(i);

	// printf("[%u nodes to be recomputed]", mods.size());

	if (mods.size()>0)
	{

		// Construct submatrix of adjacencies only with the nodes that are going
		// to be regrouped
		// -------------------------------------------------------------------
		CMatrix		A_mods;
		A_mods.setSize(mods.size(),mods.size());
		for (i=0;i<mods.size();i++)
		{
			for (j=0;j<mods.size();j++)
			{
				A_mods(i,j) = m_A(mods[i],mods[j]);
			}
		}

		// Partitions of the modified nodes
		vector<vector_uint>		mods_parts;
		mods_parts.clear();

		CGraphPartitioner<CMatrix>::RecursiveSpectralPartition(
			A_mods,
			mods_parts,
			options.partitionThreshold,
			true,
			true,
			!options.forceBisectionOnly,
			options.minimumNumberElementsEachCluster,
			false /* verbose */
			);

		// Aggregate the results with the clusters that were not used and return them
		// --------------------------------------------------------------------------
		partitions.clear();

		// 1) Add the partitions that have not been modified
		// -----------------------------------------------
		for (i=0;i<m_last_partition.size();i++)
			if (!last_parts_are_mods[i])
				partitions.push_back( m_last_partition[i] );


		// 2) Add the modified partitions
		// WARNING: Translate the indices acordingly
		// -----------------------------------------------
		for (i=0;i<mods_parts.size();i++)
		{
			vector_uint		v;
			v.clear();
			for (j=0;j<mods_parts[i].size();j++)
				v.push_back( mods[mods_parts[i][j]] );

			partitions.push_back( v );
		}
	}

	// Update all nodes
	for (i=0;i<n_nodes;i++)
		m_modified_nodes[i] = false;

	// Save partition so that we take it into account in the next iteration
	// ------------------------------------------------------------------------
	size_t n = partitions.size();
	m_last_partition.resize(n);
	for (i=0;i<n;i++)	m_last_partition[i] = partitions[i];

	m_last_last_partition_are_new_ones = false;

	MRPT_END
}



/*---------------------------------------------------------------
						getNodesCount
  ------------------------------------------------------------	---*/
unsigned int CIncrementalMapPartitioner::getNodesCount()
{
	return m_individualFrames.size();
}

/*---------------------------------------------------------------
				removeSetOfNodes
  ---------------------------------------------------------------*/
void  CIncrementalMapPartitioner::removeSetOfNodes(vector_uint	indexesToRemove, bool changeCoordsRef)
{
	MRPT_START

	size_t				nOld = m_A.getColCount();
	size_t				nNew = nOld - indexesToRemove.size();
	size_t				i,j;

	// Assure indexes are sorted:
	std::sort( indexesToRemove.begin(), indexesToRemove.end() );

	ASSERT_(nNew>=1);

	// Build the vector with the nodes that REMAINS;
	vector_uint			indexesToStay;
	indexesToStay.reserve( nNew );
	for (i=0;i<nOld;i++)
	{
		bool	remov = false;
		for (j=0;!remov && j<indexesToRemove.size();j++)
		{
			if ( indexesToRemove[j]==i )
				remov = true;
		}

		if (!remov)
			indexesToStay.push_back( i );
	}

	ASSERT_( indexesToStay.size() == nNew );

	// Update the A matrix:
	// ---------------------------------------------------
	CMatrixDouble  newA(nNew,nNew);
	for (i=0;i<nNew;i++)
		for (j=0;j<nNew;j++)
			newA(i,j)=m_A(indexesToStay[i],indexesToStay[j]);

	// Substitute "A":
	m_A = newA;

	// The last partitioning is all the nodes together:
	// --------------------------------------------------
	m_last_partition.resize(1);
	m_last_partition[0].resize(nNew);
	for (i=0;i<nNew;i++) m_last_partition[0][i] = i;

	m_last_last_partition_are_new_ones = false;

	// The matrix "A" is supposed to be right, thus recomputing is not required:
	m_modified_nodes.assign(nNew,false);



	// The new sequence of maps:
	// --------------------------------------------------
	vector_uint::reverse_iterator it;
	for (it= indexesToRemove.rbegin(); it!=indexesToRemove.rend(); ++it)
	{
		deque<mrpt::maps::CMultiMetricMap>::iterator  itM = m_individualMaps.begin() + *it;
		// delete *itM; // Delete map
		m_individualMaps.erase( itM ); // Delete from list
	}

	// The new sequence of localized SFs:
	// --------------------------------------------------
	for (it = indexesToRemove.rbegin(); it!=indexesToRemove.rend(); ++it)
		m_individualFrames.remove( *it );

	// Change coordinates reference of frames:
	CSensoryFramePtr	SF;
	CPose3DPDFPtr		posePDF;

	if (changeCoordsRef)
	{
		ASSERT_(m_individualFrames.size()>0);
		m_individualFrames.get( 0, posePDF, SF);

		CPose3D p;
		posePDF->getMean(p);
		m_individualFrames.changeCoordinatesOrigin(p);
	}

	// All done!

	MRPT_END
}

/*---------------------------------------------------------------
				markAllNodesForReconsideration
  ---------------------------------------------------------------*/
void  CIncrementalMapPartitioner::markAllNodesForReconsideration( )
{
	m_last_last_partition_are_new_ones = false;
	m_last_partition.clear();		// No partitions in last step

	for (vector<uint8_t>::iterator it = m_modified_nodes.begin();it!=m_modified_nodes.end();++it)
		*it = 1; //true;
}

/*---------------------------------------------------------------
				changeCoordinatesOrigin
  ---------------------------------------------------------------*/
void CIncrementalMapPartitioner::changeCoordinatesOrigin( const CPose3D  &newOrigin )
{
	m_individualFrames.changeCoordinatesOrigin( newOrigin );
}

/*---------------------------------------------------------------
				changeCoordinatesOriginPoseIndex
  ---------------------------------------------------------------*/
void CIncrementalMapPartitioner::changeCoordinatesOriginPoseIndex( const unsigned &newOriginPose )
{
	MRPT_START

	CPose3DPDFPtr pdf;
	CSensoryFramePtr sf;
	m_individualFrames.get(newOriginPose,pdf,sf);

	CPose3D p;
	pdf->getMean(p);
	changeCoordinatesOrigin(-p);

	MRPT_END
}

/*---------------------------------------------------------------
				getAs3DScene
  ---------------------------------------------------------------*/
void CIncrementalMapPartitioner::getAs3DScene(
	mrpt::opengl::CSetOfObjectsPtr &objs,
	const std::map<uint32_t,int64_t>  *renameIndexes
	) const
{
	objs->clear();
	ASSERT_(m_individualFrames.size() == m_A.getColCount());

	objs->insert( opengl::CGridPlaneXY::Create(-100,100,-100,100,0,5) );

	for (size_t i=0;i<m_individualFrames.size();i++)
	{
		CPose3DPDFPtr i_pdf;
		CSensoryFramePtr i_sf;
		m_individualFrames.get(i,i_pdf,i_sf);

		CPose3D  i_mean;
		i_pdf->getMean(i_mean);

		opengl::CSpherePtr   i_sph = opengl::CSphere::Create();
		i_sph->setRadius(0.02f);
		i_sph->setColor(0,0,1);

		if (!renameIndexes)
				i_sph->setName( format("%u",static_cast<unsigned int>(i)) );
		else
		{
			std::map<uint32_t,int64_t>::const_iterator itName = renameIndexes->find(i);
			ASSERT_( itName != renameIndexes->end() );
			i_sph->setName( format("%lu",static_cast<unsigned long>(itName->second)) );
		}

		i_sph->enableShowName();
		i_sph->setPose(i_mean);

		objs->insert(i_sph);


		// Arcs:
		for (size_t j=i+1;j<m_individualFrames.size();j++)
		{
			CPose3DPDFPtr j_pdf;
			CSensoryFramePtr j_sf;
			m_individualFrames.get(j,j_pdf,j_sf);

			CPose3D  j_mean;
			j_pdf->getMean(j_mean);

			float  SSO_ij = m_A(i,j);

			if (SSO_ij>0.01)
			{
				opengl::CSimpleLinePtr lin = opengl::CSimpleLine::Create();
				lin->setLineCoords(
					i_mean.x(), i_mean.y(), i_mean.z(),
					j_mean.x(), j_mean.y(), j_mean.z() );

				lin->setColor( SSO_ij, 0, 1-SSO_ij, SSO_ij*0.6 );
				lin->setLineWidth( SSO_ij * 10 );

				objs->insert(lin);
			}
		}

	}

}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CIncrementalMapPartitioner::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
		in  >> m_individualFrames
			>> m_individualMaps
			>> m_A
			>> m_last_partition
			>> m_last_last_partition_are_new_ones
			>> m_modified_nodes;

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
void  CIncrementalMapPartitioner::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_individualFrames
			<< m_individualMaps
			<< m_A
			<< m_last_partition
			<< m_last_last_partition_are_new_ones
			<< m_modified_nodes;
	}
}

/*---------------------------------------------------------------
					addMapFrame
  ---------------------------------------------------------------*/
unsigned int CIncrementalMapPartitioner::addMapFrame( const CSensoryFrame &frame, const CPose3DPDF &robotPose3D )
{
	return addMapFrame(
		CSensoryFramePtr(new CSensoryFrame(frame)),
		CPose3DPDFPtr( robotPose3D.duplicateGetSmartPtr() )
		);
}
