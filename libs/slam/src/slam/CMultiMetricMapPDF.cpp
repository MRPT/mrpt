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


#include <mrpt/random.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileStream.h>

#include <mrpt/slam/CMultiMetricMapPDF.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/slam/CObservationBeaconRanges.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/math.h>

#include <mrpt/slam/PF_aux_structs.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CMultiMetricMapPDF, CSerializable, mrpt::slam )
IMPLEMENTS_SERIALIZABLE( CRBPFParticleData,  CSerializable, mrpt::slam )


#if MRPT_HAS_SSE2
	/*---------------------------------------------------------------
	Auxiliary function: Detect the processor support of SSE.
		Only executed the first time.
	---------------------------------------------------------------*/
	bool checkSuportSSE()
	{
		return false;

/*		static	bool	firstTime = true;
		static	bool	SSE_Supported;

		// Already checked:
		if (!firstTime)
			return SSE_Supported;

		printf("[MRPT] Probing processor support for SSE...");

		firstTime = false;

		try
		{
			__asm
				{
					xorps xmm0, xmm0
				}
		}
		catch(...)
		{
			// Error: Not supported:
			printf("*NOT PASS*\n");
			SSE_Supported = false;
			return SSE_Supported;
		}

		// Ok, SSE suported:
		printf("PASS!\n");
		SSE_Supported = true;
		return SSE_Supported;
*/
	}
#endif


//#if defined(_MSC_VER)
//#	pragma warning(push)
//#	pragma warning(disable:4355) // for the "this" argument below
//#endif

/*---------------------------------------------------------------
				Constructor
  ---------------------------------------------------------------*/
CMultiMetricMapPDF::CMultiMetricMapPDF(
	const bayes::CParticleFilter::TParticleFilterOptions    &opts,
	const  mrpt::slam::TSetOfMetricMapInitializers		        *mapsInitializers,
	const  TPredictionParams						        *predictionOptions) :
//		PF_implementation<CRBPFParticleData>(static_cast<mrpt::bayes::CParticleFilterData<CRBPFParticleData>&>(*this),static_cast<mrpt::bayes::CParticleFilterCapable&>(*this) ),
		averageMap( mapsInitializers ),
		averageMapIsUpdated(false),
		SFs(),
		SF2robotPath(),
		options(),
		newInfoIndex(0)
{
	m_particles.resize( opts.sampleSize );
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		it->log_w = 0;
		it->d = new CRBPFParticleData( mapsInitializers );
	}

	// Initialize:
	static const CPose3D nullPose(0,0,0);
	clear(nullPose);

	// If provided, copy the whole set of params now:
	if (predictionOptions!=NULL)
		options = *predictionOptions;
}

//#if defined(_MSC_VER)
//#	pragma warning(pop)
//#endif

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::clear( const CPose2D &initialPose )
{
	CPose3D	p(initialPose);
	clear(p);
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::clear( const CPose3D &initialPose )
{
	size_t	i,M = m_particles.size();

	for (i=0;i<M;i++)
	{
		m_particles[i].log_w = 0;

		m_particles[i].d->mapTillNow.clear();

		m_particles[i].d->robotPath.resize(1);
		m_particles[i].d->robotPath[0]=initialPose;
	}

	SFs.clear();
	SF2robotPath.clear();

	averageMapIsUpdated = false;
}

/*---------------------------------------------------------------
	Destructor
  ---------------------------------------------------------------*/
CMultiMetricMapPDF::~CMultiMetricMapPDF( )
{
	clearParticles();
	SFs.clear();
	SF2robotPath.clear();
}

/*---------------------------------------------------------------
						getEstimatedPosePDF
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed
		as a weighted average over all m_particles.
 ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::getEstimatedPosePDF( CPose3DPDFParticles	&out_estimation ) const
{
	ASSERT_(m_particles[0].d->robotPath.size()>0);
	getEstimatedPosePDFAtTime( m_particles[0].d->robotPath.size()-1, out_estimation );
}

/*---------------------------------------------------------------
						getEstimatedPosePDFAtTime
 ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::getEstimatedPosePDFAtTime(
			size_t				timeStep,
			CPose3DPDFParticles	&out_estimation ) const
{
	//CPose3D	p;
	size_t	i,n = m_particles.size();

	// Delete current content of "out_estimation":
	out_estimation.clearParticles();

	// Create new m_particles:
	out_estimation.m_particles.resize(n);
	for (i=0;i<n;i++)
	{
		out_estimation.m_particles[i].d = new CPose3D( m_particles[i].d->robotPath[ timeStep ] );
		out_estimation.m_particles[i].log_w = m_particles[i].log_w;
	}

}


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CRBPFParticleData::writeToStream(CStream &out,int *version) const
{
	THROW_EXCEPTION("Shouldn't arrive here")
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CRBPFParticleData::readFromStream(CStream &in, int version)
{
	THROW_EXCEPTION("Shouldn't arrive here")
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	i,n,j,m;

		// The data
		n = static_cast<uint32_t>(m_particles.size());
		out << n;
		for (i=0;i<n;i++)
		{
			out << m_particles[i].log_w << m_particles[i].d->mapTillNow;
			m = static_cast<uint32_t>(m_particles[i].d->robotPath.size());
			out << m;
			for (j=0;j<m;j++)
				out << m_particles[i].d->robotPath[j];
		}
		out << SFs << SF2robotPath;
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n,j,m;

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
			for (i=0;i<n;i++)
			{
				m_particles[i].d = new CRBPFParticleData();

				// Load
				in >> m_particles[i].log_w >> m_particles[i].d->mapTillNow;

				in >> m;
				m_particles[i].d->robotPath.resize(m);
				for (j=0;j<m;j++)
					in >> m_particles[i].d->robotPath[j];
			}

			in >> SFs >> SF2robotPath;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
						getLastPose
 ---------------------------------------------------------------*/
const TPose3D* CMultiMetricMapPDF::getLastPose(const size_t i) const
{
	if (i>=m_particles.size()) THROW_EXCEPTION("Particle index out of bounds!");

	size_t	n = m_particles[i].d->robotPath.size();

	if (n)
			return &m_particles[i].d->robotPath[n-1];
	else	return NULL;
}

/*---------------------------------------------------------------
						getCurrentMetricMapEstimation
 ---------------------------------------------------------------*/
CMultiMetricMap *  CMultiMetricMapPDF::getCurrentMetricMapEstimation( )
{
	rebuildAverageMap();

	return &averageMap;
}


/*---------------------------------------------------------------
						getWeightedAveragedMap
 ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::rebuildAverageMap()
{
	//size_t				M = particlesCount();
	float				min_x = 1e6, max_x=-1e6, min_y = 1e6, max_y = -1e6;
	CParticleList::iterator	part;

	if (averageMapIsUpdated)
		return;

//	CTicTac		tictac;
//	tictac.Tic();
//	printf("[EM...");

	// ---------------------------------------------------------
	//					GRID
	// ---------------------------------------------------------
	for (part=m_particles.begin();part!=m_particles.end();part++)
	{
		ASSERT_( part->d->mapTillNow.m_gridMaps.size()>0 );

		min_x = min( min_x, part->d->mapTillNow.m_gridMaps[0]->getXMin() );
		max_x = max( max_x, part->d->mapTillNow.m_gridMaps[0]->getXMax() );
		min_y = min( min_y, part->d->mapTillNow.m_gridMaps[0]->getYMin() );
		max_y = max( max_y, part->d->mapTillNow.m_gridMaps[0]->getYMax() );
	}

	// Asure all maps have the same dimensions:
	for (part=m_particles.begin();part!=m_particles.end();part++)
		part->d->mapTillNow.m_gridMaps[0]->resizeGrid(min_x,max_x,min_y,max_y,0.5f,false);

	for (part=m_particles.begin();part!=m_particles.end();part++)
	{
		min_x = min( min_x, part->d->mapTillNow.m_gridMaps[0]->getXMin() );
		max_x = max( max_x, part->d->mapTillNow.m_gridMaps[0]->getXMax() );
		min_y = min( min_y, part->d->mapTillNow.m_gridMaps[0]->getYMin() );
		max_y = max( max_y, part->d->mapTillNow.m_gridMaps[0]->getYMax() );
	}

	// Prepare target map:
	ASSERT_( averageMap.m_gridMaps.size()>0 );
	averageMap.m_gridMaps[0]->setSize(
		min_x,
		max_x,
		min_y,
		max_y,
		m_particles[0].d->mapTillNow.m_gridMaps[0]->getResolution(),
		0);

	// Compute the sum of weights:
	double		sumLinearWeights = 0;
	for (part=m_particles.begin();part!=m_particles.end();part++)
		sumLinearWeights += exp( part->log_w );

	// CHECK:
	for (part=m_particles.begin();part!=m_particles.end();part++)
	{
		ASSERT_(part->d->mapTillNow.m_gridMaps[0]->getSizeX() == averageMap.m_gridMaps[0]->getSizeX());
		ASSERT_(part->d->mapTillNow.m_gridMaps[0]->getSizeY() == averageMap.m_gridMaps[0]->getSizeY());
	}


#if MRPT_HAS_SSE2 && defined(MRPT_OS_WINDOWS) && (MRPT_WORD_SIZE==32)
	// Autodetect SSE support:
	bool	SSE_SUPORTED = checkSuportSSE();
	if (SSE_SUPORTED)
	{
        MRPT_START

		// ******************************************************
		//     Implementation with the SSE Instructions Set
		//  http://www.rz.uni-karlsruhe.de/rz/docs/VTune/reference/
		//  JLBC - 16/MAR/2006
		// ******************************************************

		// For each particle in the RBPF:
		part=m_particles.begin();
		do
		{
			// The cells in the source map:
			unsigned short*		srcCell;
			unsigned short*		firstSrcCell = (unsigned short*)&(*part->d->mapTillNow.m_gridMaps[0]->map.begin());
			unsigned short*		lastSrcCell = (unsigned short*)&(*part->d->mapTillNow.m_gridMaps[0]->map.end());

			// Assure sizes:
			ASSERT_(0==(part->d->mapTillNow.m_gridMaps[0]->size_x % 8));
			ASSERT_(0==(averageMap.m_gridMaps[0]->size_x % 8));
			ASSERT_(averageMap.m_gridMaps[0]->map.size()==part->d->mapTillNow.m_gridMaps[0]->map.size());

			// The destination cells:
			unsigned short*		destCell;
			unsigned short*		firstDestCell = (unsigned short*)&averageMap.m_gridMaps[0]->map[0];

			// The weight of particle:
			MRPT_ALIGN16 unsigned short weights_array[8];
			weights_array[0] = weights_array[1] = weights_array[2] = weights_array[3] =
			weights_array[4] = weights_array[5] = weights_array[6] = weights_array[7] = (unsigned short)(exp(part->log_w) * 65535 / sumLinearWeights);
			unsigned short*		weights_8 = weights_array;

			srcCell = firstSrcCell;
			destCell = firstDestCell;
			//size_t	cellsCount = averageMap.gridMap->map.size();

			// For each cell in individual maps:
			__asm
			{
				push	eax
				push	edx
				push	ecx

				mov		eax, dword ptr[srcCell]
				mov		edx, dword ptr[destCell]
				mov     ecx, dword ptr[weights_8]

				_loop:

				// Load 8 cells from the particle's map:
				movdqu	xmm0, [eax]

				// Move 8 copies of the weight to XMM1
				movdqu	xmm1, [ecx]

				// Packed multiply, Unsigned, 16bits
				//  Multiply the 8 cells in XMM1 with the associated weight in XMM0, and save it in XMM1
				pmulhuw	xmm1, xmm0

				// Packed ADD with saturation, Unsigned, 16bits
				//  Accumulate the Expected Map in XMM3:
				paddusw	xmm1, [edx]

				// Store again in the expected map:
				movdqu	[edx],xmm1

				// Increment pointers:
				add		eax, 16
				add		edx, 16

				// End of loop:
				cmp		eax, dword ptr[lastSrcCell]
				jnz		_loop

				// Clear MMX flags
				emms

				pop		ecx
				pop		edx
				pop		eax
			}

			// Next particle:
			part++;
		}  while (part!=m_particles.end());

		MRPT_END_WITH_CLEAN_UP( printf("Exception in Expected Map computation with SSE!! Dumping variables:\n EM size in cells: %ux%u\n",averageMap.m_gridMaps[0]->size_x,averageMap.m_gridMaps[0]->size_y); printf("Particle map: %ux%u\n",part->d->mapTillNow.m_gridMaps[0]->size_x,part->d->mapTillNow.m_gridMaps[0]->size_y); );

	}	// End of SSE supported
	else
#endif
	{
		// ******************************************************
		//     Implementation WITHOUT the SSE Instructions Set
		// ******************************************************

		MRPT_START

		// Reserve a float grid-map, add weight all maps
		// -------------------------------------------------------------------------------------------
		std::vector<float>	floatMap;
		floatMap.resize(averageMap.m_gridMaps[0]->map.size(),0);

		// For each particle in the RBPF:
		double		sumW = 0;
		for (part=m_particles.begin();part!=m_particles.end();part++)
			sumW+=exp(part->log_w);

		if (sumW==0) sumW=1;

		for (part=m_particles.begin();part!=m_particles.end();part++)
		{
			// Variables:
			std::vector<COccupancyGridMap2D::cellType>::iterator	srcCell;
			std::vector<COccupancyGridMap2D::cellType>::iterator	firstSrcCell = part->d->mapTillNow.m_gridMaps[0]->map.begin();
			std::vector<COccupancyGridMap2D::cellType>::iterator	lastSrcCell = part->d->mapTillNow.m_gridMaps[0]->map.end();
			std::vector<float>::iterator							destCell;

			// The weight of particle:
			float		w =  exp(part->log_w) / sumW;

			ASSERT_( part->d->mapTillNow.m_gridMaps[0]->map.size() == floatMap.size() );

			// For each cell in individual maps:
			for (srcCell = firstSrcCell, destCell = floatMap.begin();srcCell!=lastSrcCell;srcCell++,destCell++)
				(*destCell) += w * (*srcCell);

		}

		// Copy to fixed point map:
		std::vector<float>::iterator							srcCell;
		std::vector<COccupancyGridMap2D::cellType>::iterator	destCell = averageMap.m_gridMaps[0]->map.begin();

		ASSERT_( averageMap.m_gridMaps[0]->map.size() == floatMap.size() );

		for (srcCell=floatMap.begin();srcCell!=floatMap.end();srcCell++,destCell++)
			*destCell = static_cast<COccupancyGridMap2D::cellType>( *srcCell );

		MRPT_END
	}	// End of SSE not supported

	// ---------------------------------------------------------
	//					POINTS
	// ---------------------------------------------------------
/*	averageMap.pointsMap->clear();
	averageMap.pointsMap->insertionOptions.fuseWithExisting = false;
	averageMap.pointsMap->insertionOptions.matchStaticPointsOnly = false;

	for (i=0;i<M;i++)
	{
		averageMap.pointsMap->fuseWith( m_particles[i].d->mapTillNow.pointsMap );
	}
*/

//	printf("%.03fms]",1000*tictac.Tac());

	// Don't calculate again until really necesary.
	averageMapIsUpdated = true;
}

/*---------------------------------------------------------------
						insertObservation
 ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::insertObservation(CSensoryFrame	&sf)
{
	const size_t M = particlesCount();

	// Insert into SFs:
	CPose3DPDFParticlesPtr posePDF = CPose3DPDFParticles::Create();
	getEstimatedPosePDF(*posePDF);

	// Insert it into the SFs and the SF2robotPath list:
	SFs.insert(
		posePDF,
		CSensoryFramePtr( new CSensoryFrame(sf) ) );
	SF2robotPath.push_back( m_particles[0].d->robotPath.size()-1 );

	for (size_t i=0;i<M;i++)
	{
		const CPose3D robotPose(*getLastPose(i));
		sf.insertObservationsInto( &m_particles[i].d->mapTillNow, &robotPose );
	}

	averageMapIsUpdated = false;
}

/*---------------------------------------------------------------
						getPath
 ---------------------------------------------------------------*/
void	 CMultiMetricMapPDF::getPath(size_t i, std::deque<math::TPose3D> &out_path) const
{
	if (i>=m_particles.size())
		THROW_EXCEPTION("Index out of bounds");
	out_path = m_particles[i].d->robotPath;
}

/*---------------------------------------------------------------
					getCurrentEntropyOfPaths
  ---------------------------------------------------------------*/
double  CMultiMetricMapPDF::getCurrentEntropyOfPaths()
{
	size_t				i;
	size_t				N=m_particles[0].d->robotPath.size();			// The poses count along the paths


	// Compute paths entropy:
	// ---------------------------
	double	H_paths = 0;

	if (N)
	{
		// For each pose along the path:
		for (i=0;i<N;i++)
		{
			// Get pose est. as m_particles:
			CPose3DPDFParticles	posePDFParts;
			getEstimatedPosePDFAtTime(i,posePDFParts);

			// Approximate to gaussian and compute entropy of covariance:
			H_paths+= posePDFParts.getCovarianceEntropy();
		}
		H_paths /= N;
	}
	return H_paths;
}

/*---------------------------------------------------------------
					getCurrentJointEntropy
  ---------------------------------------------------------------*/
double  CMultiMetricMapPDF::getCurrentJointEntropy()
{
	double								H_joint,H_paths,H_maps;
	size_t								i,M = m_particles.size();
	COccupancyGridMap2D::TEntropyInfo	entropy;

	// Entropy of the paths:
	H_paths = getCurrentEntropyOfPaths();


	float				min_x = 1e6, max_x=-1e6, min_y = 1e6, max_y = -1e6;
	CParticleList::iterator	part;

	// ---------------------------------------------------------
	//			ASSURE ALL THE GRIDS ARE THE SAME SIZE!
	// ---------------------------------------------------------
	for (part=m_particles.begin();part!=m_particles.end();part++)
	{
		ASSERT_( part->d->mapTillNow.m_gridMaps.size()>0 );

		min_x = min( min_x, part->d->mapTillNow.m_gridMaps[0]->getXMin() );
		max_x = max( max_x, part->d->mapTillNow.m_gridMaps[0]->getXMax() );
		min_y = min( min_y, part->d->mapTillNow.m_gridMaps[0]->getYMin() );
		max_y = max( max_y, part->d->mapTillNow.m_gridMaps[0]->getYMax() );
	}

	// Asure all maps have the same dimensions:
	for (part=m_particles.begin();part!=m_particles.end();part++)
		part->d->mapTillNow.m_gridMaps[0]->resizeGrid(min_x,max_x,min_y,max_y,0.5f,false);


	// Sum of linear weights:
	double	sumLinearWeights = 0;
	for (i=0;i<M;i++)
		sumLinearWeights += exp(m_particles[i].log_w);

	// Compute weighted maps entropy:
	// --------------------------------
	H_maps = 0;
	for (i=0;i<M;i++)
	{
		ASSERT_( m_particles[i].d->mapTillNow.m_gridMaps.size()>0 );

		m_particles[i].d->mapTillNow.m_gridMaps[0]->computeEntropy( entropy );
		H_maps += exp(m_particles[i].log_w) * entropy.H / sumLinearWeights;
	}

	printf("H_paths=%e\n",H_paths);
	printf("H_maps=%e\n",H_maps);

	H_joint = H_paths + H_maps;
	return	H_joint;
}

/*---------------------------------------------------------------
					Entropy aux. function
 ---------------------------------------------------------------*/
float  CMultiMetricMapPDF::H(float p)
{
	if (p==0 || p==1)	return 0;
	else				return -p*log(p);
}


/*---------------------------------------------------------------
					getCurrentMostLikelyMetricMap
  ---------------------------------------------------------------*/
CMultiMetricMap  * CMultiMetricMapPDF::getCurrentMostLikelyMetricMap( )
{
	size_t		i,max_i=0, n = m_particles.size();
	double		max_w = m_particles[0].log_w;

	for (i=0;i<n;i++)
	{
		if ( m_particles[i].log_w > max_w )
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
void  CMultiMetricMapPDF::updateSensoryFrameSequence()
{
	MRPT_START
	CPose3DPDFParticles	posePartsPDF;
	CPose3DPDFPtr		previousPosePDF;
	CSensoryFramePtr	dummy;

	for (size_t i=0;i<SFs.size();i++)
	{
		// Get last estimation:
		SFs.get(i,previousPosePDF,dummy);

		// Compute the new one:
		getEstimatedPosePDFAtTime(SF2robotPath[i], posePartsPDF);

		// Copy into SFs:
		previousPosePDF->copyFrom( posePartsPDF );
	}

	MRPT_END
}

/*---------------------------------------------------------------
				saveCurrentPathEstimationToTextFile
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::saveCurrentPathEstimationToTextFile( const std::string  &fil )
{
	FILE	*f=os::fopen( fil.c_str(), "wt");
	if (!f) return;

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		for (size_t i=0;i<it->d->robotPath.size();i++)
		{
			const CPose3D  &p = it->d->robotPath[i];

			os::fprintf(f,"%.04f %.04f %.04f %.04f %.04f %.04f ",
				p.x(),p.y(),p.z(),
				p.yaw(), p.pitch(), p.roll() );
		}
		os::fprintf(f," %e\n", it->log_w );
	}

	os::fclose(f);
}

/*---------------------------------------------------------------
				TPredictionParams
  ---------------------------------------------------------------*/
CMultiMetricMapPDF::TPredictionParams::TPredictionParams() :
	pfOptimalProposal_mapSelection(0),
	ICPGlobalAlign_MinQuality(0.70f),
	update_gridMapLikelihoodOptions(),
	KLD_params(),
	icp_params()
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::TPredictionParams::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CMultiMetricMapPDF::TPredictionParams] ------------ \n\n");

	out.printf("pfOptimalProposal_mapSelection          = %i\n", pfOptimalProposal_mapSelection );
	out.printf("ICPGlobalAlign_MinQuality               = %f\n", ICPGlobalAlign_MinQuality );

	KLD_params.dumpToTextStream(out);
	icp_params.dumpToTextStream(out);
	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::TPredictionParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	pfOptimalProposal_mapSelection = iniFile.read_int(section,"pfOptimalProposal_mapSelection",pfOptimalProposal_mapSelection, true);

	MRPT_LOAD_CONFIG_VAR( ICPGlobalAlign_MinQuality, float,   iniFile,section );

	KLD_params.loadFromConfigFile(iniFile, section);
	icp_params.loadFromConfigFile(iniFile, section);
}
