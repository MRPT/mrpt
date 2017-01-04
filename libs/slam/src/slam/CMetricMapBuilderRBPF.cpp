/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/utils/CEnhancedMetaFile.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::bayes;

#include <mrpt/utils/metaprogramming.h>
using namespace mrpt::utils::metaprogramming;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CMetricMapBuilderRBPF::CMetricMapBuilderRBPF(  const TConstructionOptions &initializationOptions ) :
	mapPDF(
		initializationOptions.PF_options,
		&initializationOptions.mapsInitializers,
		&initializationOptions.predictionOptions ),
	m_PF_options( initializationOptions.PF_options ),
	insertionLinDistance(initializationOptions.insertionLinDistance),
	insertionAngDistance(initializationOptions.insertionAngDistance),
	localizeLinDistance(initializationOptions.localizeLinDistance),
	localizeAngDistance(initializationOptions.localizeAngDistance),
	odoIncrementSinceLastLocalization(),
	odoIncrementSinceLastMapUpdate()
{
	setLoggerName("CMetricMapBuilderRBPF");
	setVerbosityLevel(initializationOptions.verbosity_level);
	// Reset:
	clear();
}

CMetricMapBuilderRBPF::CMetricMapBuilderRBPF()
{
	this->setLoggerName("CMetricMapBuilderRBPF");
	MRPT_LOG_WARN("Empty constructor invoked!\n");
}

/*---------------------------------------------------------------
				     Copy operator
  ---------------------------------------------------------------*/
CMetricMapBuilderRBPF & CMetricMapBuilderRBPF::operator =(const CMetricMapBuilderRBPF &src)
{
	if (this==&src) {
		return *this;
	}
	mapPDF = src.mapPDF;
	m_PF_options = src.m_PF_options;
	insertionLinDistance = src.insertionLinDistance;
	insertionAngDistance = src.insertionAngDistance;
	localizeLinDistance = src.localizeLinDistance;
	localizeAngDistance = src.localizeAngDistance;
	odoIncrementSinceLastLocalization = src.odoIncrementSinceLastLocalization;
	odoIncrementSinceLastMapUpdate = src.odoIncrementSinceLastMapUpdate;
	m_statsLastIteration = src.m_statsLastIteration;
	return *this;
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CMetricMapBuilderRBPF::~CMetricMapBuilderRBPF()
{

}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::clear()
{
	MRPT_LOG_DEBUG("CMetricMapBuilderRBPF::clear() called.");
	static CPose2D		nullPose(0,0,0);

	// Reset traveled distances counters:
	odoIncrementSinceLastLocalization = CPose3DPDFGaussian();

	odoIncrementSinceLastMapUpdate.setFromValues(0,0,0,0,0,0);

	// Clear maps for each particle:
	mapPDF.clear( nullPose );
}

/*---------------------------------------------------------------
					processActionObservation
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::processActionObservation(
					CActionCollection	&action,
					CSensoryFrame		&observations )
{
	MRPT_START
	mrpt::synch::CCriticalSectionLocker csl(&critZoneChangingMap); // Enter critical section (updating map)

	// Update the traveled distance estimations:
	{
		CActionRobotMovement3DPtr act3D = action.getActionByClass<CActionRobotMovement3D>();
		CActionRobotMovement2DPtr act2D = action.getActionByClass<CActionRobotMovement2D>();
		if (act3D)
		{
			MRPT_LOG_DEBUG("processActionObservation(): Input action is CActionRobotMovement3D");
			odoIncrementSinceLastMapUpdate += act3D->poseChange.getMeanVal();
			odoIncrementSinceLastLocalization += act3D->poseChange;
		}
		else if (act2D)
		{
			MRPT_LOG_DEBUG("processActionObservation(): Input action is CActionRobotMovement2D");
			odoIncrementSinceLastMapUpdate += act2D->poseChange->getMeanVal();
			odoIncrementSinceLastLocalization.mean += act2D->poseChange->getMeanVal();
		}
		else
		{
			MRPT_LOG_WARN("Action contains no odometry.\n");
		}
	}

	// Execute particle filter:
	//   But only if the traveled distance since the last update is long enough,
	//    or this is the first observation, etc...
	// ----------------------------------------------------------------------------
	bool do_localization = (
			!mapPDF.SFs.size() ||	// This is the first observation!
			options.debugForceInsertion ||
			odoIncrementSinceLastLocalization.mean.norm()>localizeLinDistance ||
			std::abs(odoIncrementSinceLastLocalization.mean.yaw())>localizeAngDistance);

	bool do_map_update = (
			!mapPDF.SFs.size() ||	// This is the first observation!
			options.debugForceInsertion ||
			odoIncrementSinceLastMapUpdate.norm()>insertionLinDistance ||
			std::abs(odoIncrementSinceLastMapUpdate.yaw())>insertionAngDistance);

	// Used any "options.alwaysInsertByClass" ??
	for (CListOfClasses::const_iterator itCl=options.alwaysInsertByClass.begin();!do_map_update && itCl!=options.alwaysInsertByClass.end();++itCl)
		for ( CSensoryFrame::iterator it=observations.begin();it!=observations.end();++it)
			if ((*it)->GetRuntimeClass()==*itCl)
			{
				do_map_update = true;
				do_localization = true;
				break;
			}


	if (do_map_update)
		do_localization = true;

	MRPT_LOG_DEBUG(mrpt::format("do_map_update=%s do_localization=%s",do_map_update ? "YES":"NO", do_localization ? "YES":"NO" ));

	if (do_localization)
	{
		// Create an artificial action object, since
		//  we've been collecting them until a threshold:
		// ------------------------------------------------
		CActionCollection	fakeActs;
		{
			CActionRobotMovement3DPtr act3D = action.getActionByClass<CActionRobotMovement3D>();
			if (act3D)
			{
				CActionRobotMovement3D newAct;
				newAct.estimationMethod = act3D->estimationMethod;
				newAct.poseChange = odoIncrementSinceLastLocalization;
				newAct.timestamp = act3D->timestamp;
				fakeActs.insert(newAct);
			}
			else
			{
				// It must be 2D odometry:
				CActionRobotMovement2DPtr act2D = action.getActionByClass<CActionRobotMovement2D>();
				ASSERT_(act2D)
				CActionRobotMovement2D newAct;
				newAct.computeFromOdometry( CPose2D(odoIncrementSinceLastLocalization.mean), act2D->motionModelConfiguration );
				newAct.timestamp = act2D->timestamp;
				fakeActs.insert(newAct);
			}
		}

		MRPT_LOG_DEBUG_STREAM << "odoIncrementSinceLastLocalization before resetting = " << odoIncrementSinceLastLocalization.mean;
		// Reset distance counters:
		odoIncrementSinceLastLocalization.mean.setFromValues(0,0,0,0,0,0);
		odoIncrementSinceLastLocalization.cov.zeros();

		CParticleFilter	pf;
		pf.m_options = m_PF_options;
		pf.setVerbosityLevel( this->getMinLoggingLevel() );

		pf.executeOn( mapPDF, &fakeActs, &observations );

		if (isLoggingLevelVisible(LVL_INFO))
		{
			// Get current pose estimation:
			CPose3DPDFParticles  poseEstimation;
			CPose3D		meanPose;
			mapPDF.getEstimatedPosePDF(poseEstimation);
			poseEstimation.getMean(meanPose);

			CPose3D		estPos;
			CMatrixDouble66	cov;
			poseEstimation.getCovarianceAndMean(cov,estPos);

			MRPT_LOG_INFO_STREAM << "New pose=" << estPos << std::endl << "New ESS:"<< mapPDF.ESS() << std::endl;
			MRPT_LOG_INFO( format("   STDs: x=%2.3f y=%2.3f z=%.03f yaw=%2.3fdeg\n", sqrt(cov(0,0)),sqrt(cov(1,1)),sqrt(cov(2,2)),RAD2DEG(sqrt(cov(3,3)))) );
		}
	}

	if (do_map_update)
	{
		odoIncrementSinceLastMapUpdate.setFromValues(0,0,0,0,0,0);

		// Update the particles' maps:
		// -------------------------------------------------
		MRPT_LOG_INFO("New observation inserted into the map.");

		// Add current observation to the map:
		const bool anymap_update = mapPDF.insertObservation(observations);
		if (!anymap_update)
			MRPT_LOG_WARN_STREAM << "**No map was updated** after inserting a CSensoryFrame with "<< observations.size();

		m_statsLastIteration.observationsInserted = true;
	}
	else
	{
		m_statsLastIteration.observationsInserted = false;
	}

	// Added 29/JUN/2007 JLBC: Tell all maps that they can now free aux. variables
	//  (if any) since one PF cycle is over:
	for (CMultiMetricMapPDF::CParticleList::iterator	it = mapPDF.m_particles.begin(); it!=mapPDF.m_particles.end();++it)
		it->d->mapTillNow.auxParticleFilterCleanUp();

	MRPT_END;
}

/*---------------------------------------------------------------
					initialize
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::initialize(
		const CSimpleMap		&initialMap,
		CPosePDF					*x0 )
{
	// Enter critical section (updating map)
	enterCriticalSection();

	MRPT_LOG_INFO_STREAM << "[initialize] Called with " << initialMap.size() << " nodes in fixed map\n"; 
	if (x0)
		MRPT_LOG_INFO_STREAM << "[initialize] x0: " << x0->getMeanVal() << "\n";
	else
		MRPT_LOG_INFO_STREAM << "[initialize] x0: (Not supplied)\n";

	this->clear();
	if (x0) {
		const CPose2D meanPose = x0->getMeanVal();
		// Clear maps for each particle & set pose:
		mapPDF.clear( meanPose );
	}

	// Leaving critical section (updating map)
	leaveCriticalSection();
}


/*---------------------------------------------------------------
						getMarkovLocalization
  ---------------------------------------------------------------*/
CPose3DPDFPtr CMetricMapBuilderRBPF::getCurrentPoseEstimation() const
{
	CPose3DPDFParticlesPtr posePDF = CPose3DPDFParticles::Create();
	mapPDF.getEstimatedPosePDF(*posePDF);
	return posePDF;
}

/*---------------------------------------------------------------
						getCurrentMostLikelyPath
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::getCurrentMostLikelyPath( std::deque<TPose3D> &outPath ) const
{
	double maxW = -1, w;
	size_t mostLik=0;
	for (size_t i=0;i<mapPDF.particlesCount();i++)
	{
		w = mapPDF.getW(i);
		if (w>maxW)
		{
			maxW = w;
			mostLik = i;
		}
	}

	mapPDF.getPath( mostLik, outPath );
}

/*---------------------------------------------------------------
						getCurrentlyBuiltMap
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::getCurrentlyBuiltMap(
			CSimpleMap		&out_map) const
{
	const_cast<CMetricMapBuilderRBPF*>(this)->mapPDF.updateSensoryFrameSequence();
	out_map = mapPDF.SFs;
}

const CMultiMetricMap* CMetricMapBuilderRBPF::getCurrentlyBuiltMetricMap() const
{
	return mapPDF.getCurrentMostLikelyMetricMap();
}

/*---------------------------------------------------------------
			getCurrentlyBuiltMapSize
  ---------------------------------------------------------------*/
unsigned int  CMetricMapBuilderRBPF::getCurrentlyBuiltMapSize()
{
	return mapPDF.SFs.size();
}


/*---------------------------------------------------------------
					drawCurrentEstimationToImage
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::drawCurrentEstimationToImage( utils::CCanvas *img )
{
	using mrpt::utils::round;

	unsigned int			i, M = mapPDF.particlesCount();
	std::deque<TPose3D>		path;
	unsigned int			imgHeight=0;

	MRPT_START

	const mrpt::maps::CMultiMetricMap * currentMetricMapEstimation = mapPDF.getCurrentMostLikelyMetricMap( );

	ASSERT_( currentMetricMapEstimation->m_gridMaps.size()>0 );


	// Find which is the most likely path index:
	unsigned int	bestPath = 0;
	double			bestPathLik = -1;
	for (i=0;i<M;i++)
	{
		if (mapPDF.getW(i)>bestPathLik)
		{
			bestPathLik = mapPDF.getW(i);
			bestPath = i;
		}
	}

	// Compute the length of the paths:
	mapPDF.getPath(0, path);

	// Adapt the canvas size:
	bool   alreadyCopiedImage = false;
	{
		CImage *obj = dynamic_cast<CImage*>( img );
		if (obj)
			obj->resize(
				currentMetricMapEstimation->m_gridMaps[0]->getSizeX(),
				currentMetricMapEstimation->m_gridMaps[0]->getSizeY(),
				1,
				true);
	}
	if (!alreadyCopiedImage)
	{
		CImage imgGrid;

		// grid map as bitmap:
		// ----------------------------------
		currentMetricMapEstimation->m_gridMaps[0]->getAsImage( imgGrid );

		img->drawImage( 0,0, imgGrid );
		imgHeight = imgGrid.getHeight();
	}

	int		x1=0,x2=0,y1=0,y2=0;
	float	x_min = currentMetricMapEstimation->m_gridMaps[0]->getXMin();
	float	y_min = currentMetricMapEstimation->m_gridMaps[0]->getYMin();
	float   resolution = currentMetricMapEstimation->m_gridMaps[0]->getResolution();

	// Paths hypothesis:
	// ----------------------------------
	/***/
	for (i=0;i<=M;i++)
	{
		if (i!=bestPath || i==M)
		{
			mapPDF.getPath(i==M ? bestPath:i, path);

			size_t	nPoses = path.size();

			// First point: (0,0)
			x2 = round( ( path[0].x - x_min) / resolution);
			y2 = round( ( path[0].y - y_min) / resolution );

			// Draw path in the bitmap:
			for (size_t j=0;j<nPoses;j++)
			{
				// For next segment
				x1 = x2;
				y1 = y2;

				// Coordinates -> pixels
				x2 = round( ( path[j].x - x_min) / resolution );
				y2 = round( ( path[j].y - y_min) / resolution );

				// Draw line:
				img->line(
					x1, round( (imgHeight-1)-y1 ),
					x2, round( (imgHeight-1)-y2 ),
					i==M ? TColor(0,0,0) : TColor(0x50,0x50,0x50),	 // Color, gray levels,
					i==M ? 3 : 1				 // Line width
					);
			}
		}
	}

	MRPT_END
}

/*---------------------------------------------------------------
					saveCurrentEstimationToImage
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP )
{
	MRPT_START

	if (formatEMF_BMP)
	{
		// Draw paths (using vectorial plots!) over the EMF file:
		// --------------------------------------------------------
		CEnhancedMetaFile		EMF( file,  100 /* Scale */ );
		drawCurrentEstimationToImage( &EMF );
	}
	else
	{
		CImage  img(1,1,  CH_GRAY );
		drawCurrentEstimationToImage( &img );
		img.saveToFile(file);
	}

	MRPT_END
}

/*---------------------------------------------------------------
					getCurrentJointEntropy
  ---------------------------------------------------------------*/
double  CMetricMapBuilderRBPF::getCurrentJointEntropy()
{
	return	mapPDF.getCurrentJointEntropy();
}

/*---------------------------------------------------------------
					saveCurrentPathEstimationToTextFile
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::saveCurrentPathEstimationToTextFile( const std::string  &fil )
{
	mapPDF.saveCurrentPathEstimationToTextFile( fil );
}

/*---------------------------------------------------------------
						TConstructionOptions
  ---------------------------------------------------------------*/
CMetricMapBuilderRBPF::TConstructionOptions::TConstructionOptions() :
	insertionLinDistance	( 1.0f ),
	insertionAngDistance	( DEG2RAD(30) ),
	localizeLinDistance		( 0.4f ),
	localizeAngDistance		( DEG2RAD(10) ),
	PF_options(),
	mapsInitializers(),
	predictionOptions(),
	verbosity_level(mrpt::utils::LVL_INFO)
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::TConstructionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CMetricMapBuilderRBPF::TConstructionOptions] ------------ \n\n");

	out.printf("insertionLinDistance                    = %f m\n", insertionLinDistance );
	out.printf("insertionAngDistance                    = %f deg\n", RAD2DEG(insertionAngDistance) );
	out.printf("localizeLinDistance                     = %f m\n", localizeLinDistance );
	out.printf("localizeAngDistance                     = %f deg\n", RAD2DEG(localizeAngDistance) );
	out.printf("verbosity_level                         = %s\n", mrpt::utils::TEnumType<mrpt::utils::VerbosityLevel>::value2name(verbosity_level).c_str());

	PF_options.dumpToTextStream(out);

	out.printf("  Now showing 'mapsInitializers' and 'predictionOptions':\n");
	out.printf("\n");

	mapsInitializers.dumpToTextStream(out);
	predictionOptions.dumpToTextStream(out);

}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CMetricMapBuilderRBPF::TConstructionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_START

	PF_options.loadFromConfigFile(iniFile,section);

	MRPT_LOAD_CONFIG_VAR(insertionLinDistance, float, iniFile,section);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(insertionAngDistance_deg, double, insertionAngDistance,  iniFile,section);

	MRPT_LOAD_CONFIG_VAR(localizeLinDistance, float, iniFile,section);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(localizeAngDistance_deg, double, localizeAngDistance,  iniFile,section);
	verbosity_level = iniFile.read_enum<mrpt::utils::VerbosityLevel>(section,"verbosity_level", verbosity_level );

	mapsInitializers.loadFromConfigFile(iniFile,section);
	predictionOptions.loadFromConfigFile(iniFile,section);

	MRPT_END
}
