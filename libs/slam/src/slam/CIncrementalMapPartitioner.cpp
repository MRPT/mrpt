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



#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/observations_overlap.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/graphs/CGraphPartitioner.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl.h>

using namespace mrpt::slam;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CIncrementalMapPartitioner, CSerializable,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::CIncrementalMapPartitioner( ) :
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
	debugSaveAllEigenvectors        ( false ),
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
	MRPT_LOAD_CONFIG_VAR(debugSaveAllEigenvectors,    bool,source,section);
	MRPT_LOAD_CONFIG_VAR(minimumNumberElementsEachCluster, int, source,section);


	MRPT_END
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void  CIncrementalMapPartitioner::TOptions::dumpToTextStream(CStream	&out) const
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

	m_individualFrames.clear();	// Liberar el mapa hasta ahora:

	// Free individual maps:
	//for (deque_serializable<mrpt::slam::CMultiMetricMap>::iterator it=m_individualMaps.begin();it!=m_individualMaps.end();++it)	delete (*it);
	m_individualMaps.clear();

	m_last_partition.clear();		// Borrar las ultimas particiones
	m_modified_nodes.clear();		//  y estos nodos a actualizar...
}

/*---------------------------------------------------------------
						Constructor
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
	int									debug_CheckPoint = 0;
	mrpt::utils::TMatchingPairList		corrs;
	static CPose3D						nullPose(0,0,0);

	// Set options in graph partition:
	CGraphPartitioner::DEBUG_SAVE_EIGENVECTOR_FILES = options.debugSaveAllEigenvectors;

	// Create the maps:
	TSetOfMetricMapInitializers			mapInitializer;
	TMetricMapInitializer				mapElement;

	mapElement.metricMapClassType = CLASS_ID( CSimplePointsMap );
	mapInitializer.push_back( mapElement );

//	mapElement.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
//	mapElement.occupancyGridMap2D_options.resolution = options.gridResolution;
//	mapInitializer.push_back( mapElement );

	mapElement.metricMapClassType = CLASS_ID( CLandmarksMap );
//	mapElement.landmarksMap_options.insertionOpts.
	mapInitializer.push_back( mapElement );

	// Add new metric map to "m_individualMaps"
	// --------------------------------------------
	//CMultiMetricMap			*newMetricMap = new CMultiMetricMap( &mapInitializer );

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

	// JLBC,17/AGO/2006: "m_individualMaps" were created from the robot pose, but it is
	//   more convenient now to save them as the robot being at (0,0,0).

	//frame->insertObservationsInto( newMetricMap.m_pointsMaps[0] );
	newMetricMap.m_pointsMaps[0]->copyFrom( * frame->buildAuxPointsMap<CPointsMap>(&newMetricMap.m_pointsMaps[0]->insertionOptions));	// Faster :-)

	// Insert just the VisualLandmarkObservations:
	newMetricMap.m_landmarksMap->insertionOptions.insert_SIFTs_from_monocular_images = false;
	newMetricMap.m_landmarksMap->insertionOptions.insert_SIFTs_from_stereo_images    = false;
	newMetricMap.m_landmarksMap->insertionOptions.insert_Landmarks_from_range_scans  = false;
	frame->insertObservationsInto( newMetricMap.m_landmarksMap );

	debug_CheckPoint=1;

	// Add to corresponding vectors:
	m_individualFrames.insert(robotPose, frame);
	// Already added to "m_individualMaps" above

	debug_CheckPoint=2;

	// Ampliar la matriz de adyacencias:
	// -----------------------------------------------------------------
	n = m_A.getColCount();
	n++;
	m_A.setSize(n,n);

	debug_CheckPoint=3;

	ASSERT_(m_individualMaps.size() == n);
	ASSERT_(m_individualFrames.size() == n);

	// Ajustar tamaño del vector de nodos modificados:
	// ---------------------------------------------------
	// El nuevo evidentemente debe ser tenido en cuenta:
	m_modified_nodes.push_back(true);

	// Methods to compute adjacency matrix:
	// true:  matching between maps
	// false: matching between observations through "CObservation::likelihoodWith"
	// ------------------------------------------------------------------------------
	bool useMapOrSF = options.useMapMatching;

	debug_CheckPoint=4;

	// Calcular los nuevos matchings y meterlos en la matriz:
	// ----------------------------------------------------------------
	//for (i=n-1;i<n;i++)
	i=n-1;   // Solo ejecutar para "i=n-1" la ultima fila/columna que esta vacia
	{
		// Get node "i":
		m_individualFrames.get(i, posePDF_i, sf_i);
		posePDF_i->getMean(pose_i);

		// And its points map:
		map_i = &m_individualMaps[i];

		debug_CheckPoint=5;

//			for (j=0;j<n;j++)
		for (j=0;j<n-1;j++)
		{
			// Get node "j":
			m_individualFrames.get(j, posePDF_j, sf_j);
			posePDF_j->getMean( pose_j );

			debug_CheckPoint=6;

			relPose = pose_j - pose_i;

			// And its points map:
			map_j = &m_individualMaps[j];

			debug_CheckPoint=66;

			// Compute matching ratio:
			if (useMapOrSF)
			{
				m_A(i,j) = map_i->compute3DMatchingRatio(
					map_j,
					relPose,
					options.minDistForCorrespondence,
					options.minMahaDistForCorrespondence );
			}
			else
			{
				//m_A(i,j) = sf_i->likelihoodWith(sf_j.pointer());
				m_A(i,j) = observationsOverlap(sf_i, sf_j, &relPose );
			}

		} // for j

	} // for i


	for (i=0;i<n-1;i++)  // Solo ejecutar para "i=n-1" la ultima fila/columna que esta vacia
	{
		debug_CheckPoint=8;

		// Get node "i":
		m_individualFrames.get(i, posePDF_i, sf_i);
		posePDF_i->getMean(pose_i);

		// And its points map:
		map_i = &m_individualMaps[i];

		debug_CheckPoint=9;

		j=n-1; //for (j=n-1;j<n;j++)
		{
			// Get node "j":
			m_individualFrames.get(j, posePDF_j, sf_j);
			posePDF_j->getMean(pose_j);

			debug_CheckPoint=10;

			relPose = pose_j - pose_i;

			// And its points map:
			map_j = &m_individualMaps[j];

			// Compute matching ratio:
			if (useMapOrSF)
			{
				m_A(i,j) = map_i->compute3DMatchingRatio(
					map_j,
					CPose3D(relPose),
					options.minDistForCorrespondence,
					options.minMahaDistForCorrespondence );
			}
			else
			{
				//m_A(i,j) = sf_i->likelihoodWith(sf_j.pointer());
				m_A(i,j) = observationsOverlap(sf_i, sf_j, &relPose );
			}

			debug_CheckPoint=12;

		} // for j

	} // for i

	debug_CheckPoint=13;

	// Self-similatity: Not used
	m_A(n-1,n-1) = 0;

	// Hacer que la matriz sea simetrica:
	// -----------------------------------------------------------------
	for (i=0;i<n;i++)
		for (j=i+1;j<n;j++)
			m_A(i,j) = m_A(j,i) = 0.5f * (m_A(i,j) + m_A(j,i) );

	debug_CheckPoint=14;

	/* DEBUG: Guardar la matriz: * /
	A.saveToTextFile("debug_matriz.txt",1);
	/ **/

	// Añadir a la lista de nodos modificados los nodos afectados:
	// -----------------------------------------------------------------
	for (i=0;i<n;i++)
		m_modified_nodes[i] = m_A(i,n-1) > 0;

	debug_CheckPoint=15;

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
		cout << "Unexpected runtime error at checkPoint="<< debug_CheckPoint << "\n"; \
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
						Constructor
  ---------------------------------------------------------------*/
  void CIncrementalMapPartitioner::updatePartitions(
	  vector<vector_uint> &partitions )
{
	MRPT_START

	unsigned int			i,j;
	unsigned int			n_nodes;
	unsigned int			n_clusters_last;
	vector_uint				mods;	// La lista con los nodos que finalmente seran reagrupados
	vector_bool				last_parts_are_mods;

	n_nodes = m_modified_nodes.size();			// El numero de nodos (scans) en total:
	n_clusters_last = m_last_partition.size();	// El numero de clusters en la ult. particion:

	last_parts_are_mods.resize( n_clusters_last );

	// Si un solo scan de un cluster se ve afectado, su cluster completo
	//   se vera afectado:
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

		// Si ha cambiado, marcar todos sus nodos:
		if (last_parts_are_mods[i])
			for (j=0;j<p.size();j++)
				m_modified_nodes[ p[j] ] = true;
	}

	// Cuantos nodos en total entran en el algoritmo de particionado??
	mods.clear();
	for (i=0;i<n_nodes;i++)
		if ( m_modified_nodes[i] )
			mods.push_back(i);

	// printf("[%u nodes to be recomputed]", mods.size());

	if (mods.size()>0)
	{

		// Construir sub-matriz de adyacencias con solo los nodos que tengan
		//   que ser reagrupados:
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

		// Las particiones de los nodos modificados:
		vector<vector_uint>		mods_parts;
		mods_parts.clear();

		CGraphPartitioner::RecursiveSpectralPartition(
			A_mods,
			mods_parts,
			options.partitionThreshold,
			true,
			true,
			!options.forceBisectionOnly,
			options.minimumNumberElementsEachCluster );

		// Mezclar los resultados con los clusters que no se tocaron y devolverlos
		// --------------------------------------------------------------------------
		partitions.clear();

		// 1) Añadir las particiones que no han cambiado:
		// -----------------------------------------------
		for (i=0;i<m_last_partition.size();i++)
			if (!last_parts_are_mods[i])
				partitions.push_back( m_last_partition[i] );


		// 2) Añadir las particiones actualizadas:
		//  ¡¡ CUIDADO: Hay que traducir los indices !!
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

	// Todos los nodos estan actualizados ahora:
	for (i=0;i<n_nodes;i++)
		m_modified_nodes[i] = false;

	// Guardar particion para tenerla en cuenta en la siguiente iteracion:
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
	for (it= indexesToRemove.rbegin(); it!=indexesToRemove.rend(); it++)
	{
		deque<mrpt::slam::CMultiMetricMap>::iterator  itM = m_individualMaps.begin() + *it;
		// delete *itM; // Delete map
		m_individualMaps.erase( itM ); // Delete from list
	}

	// The new sequence of localized SFs:
	// --------------------------------------------------
	for (it = indexesToRemove.rbegin(); it!=indexesToRemove.rend(); it++)
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
		i_sph->setRadius(0.02);
		i_sph->setColor(0,0,1);

		if (!renameIndexes)
				i_sph->setName( format("%"PRIuPTR,i) );
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
void  CIncrementalMapPartitioner::readFromStream(CStream &in,int version)
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
void  CIncrementalMapPartitioner::writeToStream(CStream &out, int *version) const
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
