/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers
#include <mrpt/gui.h>
#include <mrpt/maps/CColouredPointsMap.h>

#include <mrpt/detectors/CFaceDetection.h>
#include <mrpt/math.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CAxis.h>

#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/slam/CICP.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>


using namespace std;
using namespace mrpt;
using namespace mrpt::detectors;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::maps;
using namespace mrpt::obs;


//------------------------------------------------------------------------
//							CFaceDetection
//------------------------------------------------------------------------
CFaceDetection::CFaceDetection() :
	m_end_threads(false),
	m_enter_checkIfFaceRegions(0,1),
	m_enter_checkIfFacePlaneCov(0,1),
	m_enter_checkIfDiagonalSurface(0,1),
	m_leave_checkIfFaceRegions(0,1),
	m_leave_checkIfFacePlaneCov(0,1),
	m_leave_checkIfDiagonalSurface(0,1)
{
	m_measure.numPossibleFacesDetected = 0;
	m_measure.numRealFacesDetected = 0;

	m_measure.faceNum = 0;

	m_timeLog.enable();
}


//------------------------------------------------------------------------
//							~CFaceDetection
//------------------------------------------------------------------------
CFaceDetection::~CFaceDetection()
{
	// Stop filters threads

	m_end_threads	= true;

	m_enter_checkIfFacePlaneCov.release();
	m_enter_checkIfFaceRegions.release();
	m_enter_checkIfDiagonalSurface.release();

	joinThread(m_thread_checkIfFaceRegions);
	joinThread(m_thread_checkIfFacePlaneCov);
	joinThread(m_thread_checkIfDiagonalSurface);
}

//------------------------------------------------------------------------
//								init
//------------------------------------------------------------------------
void CFaceDetection::init(const mrpt::utils::CConfigFileBase &cfg )
{
	m_options.confidenceThreshold	= cfg.read_int( "FaceDetection", "confidenceThreshold", 240 );
	m_options.multithread			= cfg.read_bool( "FaceDetection", "multithread", true );
	m_options.useCovFilter			= cfg.read_bool( "FaceDetection", "useCovFilter", true );
	m_options.useRegionsFilter		= cfg.read_bool( "FaceDetection", "useRegionsFilter", true );
	m_options.useSizeDistanceRelationFilter	= cfg.read_bool( "FaceDetection", "useSizeDistanceRelationFilter", true );
	m_options.useDiagonalDistanceFilter		= cfg.read_bool( "FaceDetection", "useDiagonalDistanceFilter", true );

	m_testsOptions.planeThreshold		= cfg.read_double( "FaceDetection", "planeThreshold", 50 );
	m_testsOptions.planeTest_eigenVal_top		= cfg.read_double( "FaceDetection", "planeTest_eigenVal_top", 0.011 );
	m_testsOptions.planeTest_eigenVal_bottom	= cfg.read_double( "FaceDetection", "planeTest_eigenVal_bottom", 0.0002 );
	m_testsOptions.regionsTest_sumDistThreshold_top		= cfg.read_double( "FaceDetection", "regionsTest_sumDistThreshold_top", 0.5 );
	m_testsOptions.regionsTest_sumDistThreshold_bottom		= cfg.read_double( "FaceDetection", "regionsTest_sumDistThreshold_bottom", 0.04 );

	m_measure.takeTime				= cfg.read_bool( "FaceDetection", "takeTime", false );
	m_measure.takeMeasures			= cfg.read_bool( "FaceDetection", "takeMeasures", false );
	m_measure.saveMeasurementsToFile= cfg.read_bool( "FaceDetection", "saveMeasurementsToFile", false );

	// Run filters threads
	if ( m_options.multithread )
	{
		if ( m_options.useRegionsFilter )
			m_thread_checkIfFaceRegions = createThread( dummy_checkIfFaceRegions, this );
		if ( m_options.useCovFilter )
			m_thread_checkIfFacePlaneCov = createThread( dummy_checkIfFacePlaneCov, this );
		if ( m_options.useSizeDistanceRelationFilter || m_options.useDiagonalDistanceFilter )
			m_thread_checkIfDiagonalSurface = createThread( dummy_checkIfDiagonalSurface, this );

		m_checkIfFacePlaneCov_res	= false;
		m_checkIfFaceRegions_res	= true;
		m_checkIfDiagonalSurface_res = true;
	}

	cascadeClassifier.init( cfg );
}


//------------------------------------------------------------------------
//							detectObjects
//------------------------------------------------------------------------
void CFaceDetection::detectObjects_Impl(const mrpt::obs::CObservation *obs, vector_detectable_object &detected)
{
	MRPT_START

	// Detect possible faces
	vector_detectable_object localDetected;

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
			m_timeLog.enter("Detection time");
	}

	cascadeClassifier.detectObjects( obs, localDetected );

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
			m_timeLog.leave("Detection time");

		//if ( m_measure.takeMeasures )
		m_measure.numPossibleFacesDetected += localDetected.size();
	}


	// Check if we are using a 3D Camera and 3D points are saved
	if ( (IS_CLASS(obs, CObservation3DRangeScan )) && ( localDetected.size() > 0 ) )
	{
		// To obtain experimental results
		{
			if ( m_measure.takeTime )
				m_timeLog.enter("Check if real face time");
		}

		CObservation3DRangeScan* o = static_cast<CObservation3DRangeScan*>( const_cast<CObservation*>(obs) );

		if ( o->hasPoints3D )
		{
			// Vector to save detected objects to delete if they aren't a face
			vector<size_t> deleteDetected;

			// Check if all possible detected faces satisfy a serial of constrains
			for ( unsigned int i = 0; i < localDetected.size(); i++ )
			{
				CDetectable2DPtr rec	= CDetectable2DPtr(localDetected[i]);

				// Calculate initial and final rows and columns
				unsigned int r1 = rec->m_y;
				unsigned int r2 = rec->m_y + rec->m_height;
				unsigned int c1 = rec->m_x;
				unsigned int c2 = rec->m_x + rec->m_width;

				o->getZoneAsObs( m_lastFaceDetected, r1, r2, c1, c2 );

				if ( m_options.multithread )
				{
					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.enter("Multithread filters application");
					}

					// Semaphores signal
					if ( m_options.useCovFilter )
						m_enter_checkIfFacePlaneCov.release();
					if ( m_options.useRegionsFilter )
						m_enter_checkIfFaceRegions.release();
					if ( m_options.useSizeDistanceRelationFilter || m_options.useDiagonalDistanceFilter )
						m_enter_checkIfDiagonalSurface.release();

					// Semaphores wait
					if ( m_options.useCovFilter )
						m_leave_checkIfFacePlaneCov.waitForSignal();
					if ( m_options.useRegionsFilter )
						m_leave_checkIfFaceRegions.waitForSignal();
					if ( m_options.useSizeDistanceRelationFilter || m_options.useDiagonalDistanceFilter )
						m_leave_checkIfDiagonalSurface.waitForSignal();

					// Check resutls
					if ( !m_checkIfFacePlaneCov_res || !m_checkIfFaceRegions_res
						|| !m_checkIfDiagonalSurface_res )
						deleteDetected.push_back( i );

					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.leave("Multithread filters application");
					}

					m_measure.faceNum++;

				}
				else
				{
					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.enter("Secuential filters application");
					}

					/////////////////////////////////////////////////////
					//CMatrixTemplate<bool> region;
					//experimental_segmentFace( m_lastFaceDetected,  region);
					/////////////////////////////////////////////////////

					//m_lastFaceDetected.intensityImage.saveToFile(format("%i.jpg",m_measure.faceNum));

					bool remove = false;

					// First check if we can adjust a plane to detected region as face, if yes it isn't a face!
					if ( m_options.useCovFilter && !checkIfFacePlaneCov( &m_lastFaceDetected ) )
					{
						deleteDetected.push_back( i );
						remove = true;
					}
					else if ( m_options.useRegionsFilter && !checkIfFaceRegions( &m_lastFaceDetected ) )
					{
						deleteDetected.push_back( i );
						remove = true;
					}
					else if ( ( m_options.useSizeDistanceRelationFilter || m_options.useDiagonalDistanceFilter )
						&& !checkIfDiagonalSurface( &m_lastFaceDetected ) )
					{
						deleteDetected.push_back( i );
						remove = true;
					}

					if ( remove )
					{
						/*ofstream f;
						f.open("deleted.txt", ofstream::app);
						f << "Deleted: " << m_measure.faceNum << endl;
						f.close();*/
						m_measure.deletedRegions.push_back( m_measure.faceNum );

					}

					m_measure.faceNum++;

					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.leave("Secuential filters application");
					}

				}

			}

			// Delete non faces
			for ( unsigned int i = deleteDetected.size(); i > 0; i-- )
				localDetected.erase( localDetected.begin() + deleteDetected[i-1] );
		}

		// Convert 2d detected objects to 3d
		for ( unsigned int i = 0; i < localDetected.size(); i++ )
		{
			CDetectable3DPtr object3d =
				CDetectable3DPtr( new CDetectable3D((CDetectable2DPtr)localDetected[i]) );
			detected.push_back( object3d );
		}

		// To obtain experimental results
		{
			if ( m_measure.takeTime )
				m_timeLog.leave("Check if real face time");
		}
	}
	else // Not using a 3D camera
	{
		detected = localDetected;
	}

	// To obtain experimental results
	{
		//if ( m_measure.takeMeasures )
			m_measure.numRealFacesDetected += detected.size();
	}

	MRPT_END

}


//------------------------------------------------------------------------
//  						checkIfFacePlane
//------------------------------------------------------------------------
bool CFaceDetection::checkIfFacePlane( CObservation3DRangeScan *face )
{

	vector<TPoint3D> points;

	size_t N = face->points3D_x.size();

	points.resize( N );

	for ( size_t i = 0; i < N; i++ )
		points[i] = TPoint3D( face->points3D_x.at(i), face->points3D_y.at(i), face->points3D_z.at(i) );

	// Try to ajust a plane
	TPlane plane;

	// To obtain experimental results
	{
		if ( m_measure.takeMeasures )
			m_measure.errorEstimations.push_back( (double)getRegressionPlane(points,plane) );
	}

	if ( getRegressionPlane(points,plane) < m_testsOptions.planeThreshold )
		return true;

	return false;
}

void CFaceDetection::dummy_checkIfFacePlaneCov( CFaceDetection *obj )
{
	obj->thread_checkIfFacePlaneCov( );
}

void CFaceDetection::thread_checkIfFacePlaneCov( )
{
	for(;;)
	{
		m_enter_checkIfFacePlaneCov.waitForSignal();

		if ( m_end_threads )
			break;

		// Perform filter
		m_checkIfFacePlaneCov_res = checkIfFacePlaneCov( &m_lastFaceDetected );

		m_leave_checkIfFacePlaneCov.release();
	}
}

//------------------------------------------------------------------------
//  					 checkIfFacePlaneCov
//------------------------------------------------------------------------
bool CFaceDetection::checkIfFacePlaneCov( CObservation3DRangeScan* face )
{
	MRPT_TRY_START

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.enter("Check if face plane: covariance");
	}

	// Get face region size
	const unsigned int faceWidth = face->intensityImage.getWidth();
	const unsigned int faceHeight = face->intensityImage.getHeight();

	// We work with a confidence image?
	const bool confidence = face->hasConfidenceImage;

	// To fill with valid points
	vector<CArrayDouble<3> > pointsVector;

	CMatrixTemplate<bool> region; // To save the segmented region
	experimental_segmentFace( *face,  region);

	for ( unsigned int j = 0; j < faceHeight; j++ )
	{
		for ( unsigned int k = 0; k < faceWidth; k++ )
		{
			CArrayDouble<3> aux;

			if ( region.get_unsafe( j,k ) && (( (!confidence) || (( confidence ) &&
				( *(face->confidenceImage.get_unsafe( k, j, 0 )) > m_options.confidenceThreshold )
				&& ( *(face->intensityImage.get_unsafe( k, j )) > 50 )))))	// Don't take in account dark pixels
			{
				int position = faceWidth*j + k;
				aux[0] = face->points3D_x[position];
				aux[1] = face->points3D_y[position];
				aux[2] = face->points3D_z[position];
				pointsVector.push_back( aux );
			}
		}
	}

	// Check if points vector is empty to avoid a future crash
	if ( pointsVector.empty() )
		return false;

	//experimental_viewFacePointsScanned( *face );

	// To obtain the covariance vector and eigenvalues
	CMatrixDouble cov;
	CMatrixDouble eVects, m_eVals;
	CVectorDouble eVals;

	cov = covVector<vector<CArrayDouble<3> >,CMatrixDouble>( pointsVector );

	cov.eigenValues( eVals );

	cov.eigenVectors( eVects, m_eVals );

	// To obtain experimental results
	{
		if ( m_measure.takeMeasures )
			m_measure.lessEigenVals.push_back(eVals[0]);

		if ( m_measure.takeTime )
			m_timeLog.leave("Check if face plane: covariance");

		// Uncomment if you want to analyze the calculated eigenvalues
		//ofstream f;
		/*f.open("eigenvalues.txt", ofstream::app);
		f << m_measure.faceNum << " " << eVals[0] << endl;
		f.close();*/

		//f.open("eigenvalues2.txt", ofstream::app);
		cout << eVals[0] << " " << eVals[1] << " " << eVals[2] << " > " ;
		cout << eVals[0]/eVals[2] << endl;
		//f << eVals[0]/eVals[2] << endl;
		//f.close();
	}

	if ( m_measure.faceNum >= 314 )
		experimental_viewFacePointsAndEigenVects( pointsVector, eVects, eVals );

	// Check if the less eigenvalue is out of the permited area
	//if ( ( eVals[0] > m_options.planeEigenValThreshold_down )
	//	&& ( eVals[0] < m_options.planeEigenValThreshold_up ) )
	if ( eVals[0]/eVals[2] > 0.06 )
	{

		//Uncomment if you want to save the face regions discarted by this filter
		/*ofstream f;
		f.open("deletedCOV.txt", ofstream::app);
		f << m_measure.faceNum << endl;
		f.close();*/

		return true;	// Filter not passed
	}

	return false;	// Filter passed

	MRPT_TRY_END
}


void CFaceDetection::dummy_checkIfFaceRegions( CFaceDetection *obj )
{
		obj->thread_checkIfFaceRegions( );
}

void CFaceDetection::thread_checkIfFaceRegions( )
{
	for(;;)
	{
		m_enter_checkIfFaceRegions.waitForSignal();

		if ( m_end_threads )
			break;

		// Perform filter
		m_checkIfFaceRegions_res = checkIfFaceRegions( &m_lastFaceDetected );

		m_leave_checkIfFaceRegions.release();
	}
}


//------------------------------------------------------------------------
//							checkIfFaceRegions
//------------------------------------------------------------------------

bool CFaceDetection::checkIfFaceRegions( CObservation3DRangeScan* face )
{
	MRPT_START

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.enter("Check if face plane: regions");
	}

	// To obtain region size
	const unsigned int faceWidth = face->intensityImage.getWidth();
	const unsigned int faceHeight = face->intensityImage.getHeight();

	// Initial vertical size of a region
	unsigned int sectionVSize = faceHeight/3.0;

	// Steps of this filter
	//	1. To segment the region detected as face using a regions growing algorithm
	//	2. To obtain the first and last column to work (a profile face detected can have a lateral area without to use)
	//	3. To calculate the histogram of the upper zone of the region for determine if we use it (if this zone present
	//		a lot of dark pixels the measurements can be wrong)
	//	4. To obtain the coordinates of pixels that form each subregion
	//	5. To calculate medians or means of each subregion
	//	6. To check subregions constrains

	vector<TPoint3D> points;

	TPoint3D meanPos[3][3] = {
		{ TPoint3D(0,0,0),TPoint3D(0,0,0),TPoint3D(0,0,0)},
		{ TPoint3D(0,0,0),TPoint3D(0,0,0),TPoint3D(0,0,0)},
		{ TPoint3D(0,0,0),TPoint3D(0,0,0),TPoint3D(0,0,0)} };
	int numPoints[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };

	vector<TPoint3D> regions2[9];

	//
	//	1. To segment the region detected as face using a regions growing algorithm
	//

	CMatrixTemplate<bool> region; // To save the segmented region
	experimental_segmentFace( *face,  region);

	//
	//	2. To obtain the first and last column to work (a profile face detected can have a lateral area without to use)
	//

	size_t start=faceWidth, end=0;

	for ( size_t r = 0; r < region.getRowCount() ; r++ )
		for ( size_t c = 1; c < region.getColCount() ; c++ )
		{
			if ( ( !(region.get_unsafe( r, c-1 )) ) && ( region.get_unsafe( r, c )) )
			{
				if ( c < start )
					start = c;
			}
			else if ( ( region.get_unsafe( r, c-1 ) ) && ( !(region.get_unsafe( r, c )) ) )
				if ( c > end )
					end = c;

			if ( ( c > end ) && ( region.get_unsafe( r, c ) ) )
				end = c;

		}

	if ( end == 0 ) end = faceWidth-1;	// Check if the end has't changed
	if ( end < 3*(faceWidth/4) ) end = 3*(faceWidth/4); // To avoid spoiler
	if ( start == faceWidth ) start = 0;	// Check if the start has't changed
	if ( start > faceWidth/4 ) start = faceWidth/4;	// To avoid spoiler

	//cout << "Start: " << start << " End: " << end << endl;

	// To use the start and end calculated to obtain the final regions limits
	unsigned int utilWidth = faceWidth - start - ( faceWidth - end );
	unsigned int c1 = ceil( utilWidth/3.0 + start );
	unsigned int c2 = ceil( 2*(utilWidth/3.0) + start );

	//
	//	3. To calculate the histogram of the upper zone of the region for determine if we use it
	//

	CMatrixTemplate<unsigned int> hist;
	hist.setSize(1,256,true);
	experimental_calcHist( face->intensityImage, start, 0, end, ceil(faceHeight*0.1), hist );

	size_t countHist = 0;
	for ( size_t i = 0; i < 60; i++ )
	{
		countHist += hist.get_unsafe(0,i);
	}

	size_t upLimit = 0;
	size_t downLimit = faceHeight-1;

	if ( countHist > 10 )
	{
		upLimit = floor(faceHeight*0.1);
		downLimit = floor(faceHeight*0.9);
	}

	// Uncomment it if you want to analyze the number of pixels that have more dark that the 60 gray tone
	//m_meanHist.push_back( countHist );

	//
	//	4. To obtain the coordinates of pixels that form each region
	//

	unsigned int cont = 0;

	for ( unsigned int r = 0; r < faceHeight; r++ )
	{
		for ( unsigned int c = 0; c < faceWidth; c++, cont++ )
		{
			if ( ( r >= upLimit ) && ( r <= downLimit )
				&& ( region.get_unsafe( r, c ) )
				&& (*(face->confidenceImage.get_unsafe( c, r, 0 )) > m_options.confidenceThreshold )
				&& ( *(face->intensityImage.get_unsafe( c, r )) > 50) )
			{
				unsigned int row, col;
				if ( r < sectionVSize + upLimit*0.3)
					row = 0;
				else if ( r < sectionVSize*2 - upLimit*0.15 )
					row = 1;
				else
					row = 2;

				if ( c < c1)
					col = 0;
				else if ( c < c2 )
					col = 1;
				else
					col = 2;


				TPoint3D point( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont]);
				meanPos[row][col] = meanPos[row][col] + point;

				++numPoints[row][col];

				if ( row == 0 && col == 0 )
					regions2[0].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 0 && col == 1 )
					regions2[1].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 0 && col == 2 )
					regions2[2].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 1 && col == 0 )
					regions2[3].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 1 && col == 1 )
					regions2[4].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 1 && col == 2 )
					regions2[5].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 2 && col == 0 )
					regions2[6].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else if ( row == 2 && col == 1 )
					regions2[7].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				else
					regions2[8].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );

			}

		}
	}

	//
	//	5. To calculate medians or means of each subregion
	//

	vector<double> oldPointsX1;

	size_t middle1=0;
	size_t middle2=0;

	if ( regions2[0].size() > 0 )
	{
		for ( size_t i = 0; i < regions2[0].size(); i++ )
			oldPointsX1.push_back( regions2[0][i].x );

		middle1 = floor((double)oldPointsX1.size()/2);
		nth_element(oldPointsX1.begin(),oldPointsX1.begin()+middle1,oldPointsX1.end()); // Obtain center element
	}

	vector<double> oldPointsX2;

	if ( regions2[2].size() > 0 )
	{
		for ( size_t i = 0; i < regions2[2].size(); i++ )
			oldPointsX2.push_back( regions2[2][i].x );

		middle2 = floor((double)oldPointsX2.size()/2);
		nth_element(oldPointsX2.begin(),oldPointsX2.begin()+middle2,oldPointsX2.end()); // Obtain center element
	}

	for ( size_t i = 0; i < 3; i++ )
		for ( size_t j = 0; j < 3; j++ )
			if ( !numPoints[i][j] )
				meanPos[i][j] = TPoint3D( 0, 0, 0 );
			else
				meanPos[i][j] = meanPos[i][j] / numPoints[i][j];

	if ( regions2[0].size() > 0  )
		meanPos[0][0].x = oldPointsX1.at(middle1);

	if ( regions2[2].size() > 0 )
		meanPos[0][2].x = oldPointsX2.at(middle2);

	//
	//	6. To check subregions constrains
	//
	vector<double> dist(5);
	size_t res = checkRelativePosition( meanPos[1][0], meanPos[1][2], meanPos[1][1], dist[0] );
	res	 += res && checkRelativePosition( meanPos[2][0], meanPos[2][2], meanPos[2][1], dist[1] );
	res	 += res && checkRelativePosition( meanPos[0][0], meanPos[0][2], meanPos[0][1], dist[2] );
	res	 += res && checkRelativePosition( meanPos[0][0], meanPos[2][2], meanPos[1][1], dist[3] );
	res	 += res && checkRelativePosition( meanPos[2][0], meanPos[0][2], meanPos[1][1], dist[4] );

	ofstream f;
	f.open("dist.txt", ofstream::app);
	f << sum(dist) << endl;
	f.close();

	bool real = false;
	if ( !res )
		real = true;
	else if (( res = 1 ) && ( sum(dist) > 0.04 ))
		real = true;

	f.open("tam.txt",ofstream::app);
	f << meanPos[0][1].distanceTo( meanPos[2][1] ) << endl;
	f.close();


	//experimental_viewRegions( regions2, meanPos );

	//cout << endl << meanPos[0][0] << "\t" << meanPos[0][1] << "\t" << meanPos[0][2];
	//	cout << endl << meanPos[1][0] << "\t" << meanPos[1][1] << "\t" << meanPos[1][2];
	//	cout << endl << meanPos[2][0] << "\t" << meanPos[2][1] << "\t" << meanPos[2][2] << endl;


	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.leave("Check if face plane: regions");
	}

	if ( real )
		return true; // Filter passed
	else
	{

		// Uncomment if you want to known what regions was discarted by this filter
		/*ofstream f;
		f.open("deletedSTRUCTURES.txt", ofstream::app);
		f << m_measure.faceNum << endl;
		f.close();*/

		return false; // Filter not passed
	}

	MRPT_END
}


//------------------------------------------------------------------------
//							checkRelativePosition
//------------------------------------------------------------------------

size_t CFaceDetection::checkRelativePosition( const TPoint3D &p1, const TPoint3D &p2, const TPoint3D &p, double &dist )
{
	double x1 = -p1.y;
	double y1 = p1.x;

	double x2 = -p2.y;
	double y2 = p2.x;

	double x = -p.y;
	double y = p.x;

	double yIdeal = y1 + ( ((x-x1)*(y2-y1)) / (x2-x1) );

	//////////////////////////////////

	/*double xaux = x2;
	double yaux = y1;

	cout << "Grados= " << RAD2DEG(acos( (xaux-x1)/(sqrt(pow(x1-x2,2)+pow(y1-y2,2))) )) << endl;*/

	///////////////////////////////////////

	dist = yIdeal-y;

	if (  y < yIdeal )
		return 0;
	else
		return 1;
}


void CFaceDetection::dummy_checkIfDiagonalSurface( CFaceDetection *obj )
{
	obj->thread_checkIfDiagonalSurface( );
}

void CFaceDetection::thread_checkIfDiagonalSurface( )
{
	for(;;)
	{
		m_enter_checkIfDiagonalSurface.waitForSignal();

		if ( m_end_threads )
			break;

		// Perform filter
		m_checkIfDiagonalSurface_res = checkIfDiagonalSurface( &m_lastFaceDetected );

		m_leave_checkIfDiagonalSurface.release();
	}
}


//------------------------------------------------------------------------
//							checkIfDiagonalSurface
//------------------------------------------------------------------------

bool CFaceDetection::checkIfDiagonalSurface( CObservation3DRangeScan* face )
{
	MRPT_START

	// To obtain experimental results
	{
		if ( m_options.useDiagonalDistanceFilter && m_measure.takeTime )
			m_timeLog.enter("Check if face plane: diagonal distances");

		if ( m_options.useSizeDistanceRelationFilter && m_measure.takeTime )
			m_timeLog.enter("Check if face plane: size-distance relation");
	}

	const unsigned int faceWidth = face->intensityImage.getWidth();
	const unsigned int faceHeight = face->intensityImage.getHeight();

	//const float max_desv = 0.2;

	unsigned int x1 = ceil(faceWidth*0.25);
	unsigned int x2 = floor(faceWidth*0.75);
	unsigned int y1 = ceil(faceHeight*0.15);
	unsigned int y2 = floor(faceHeight*0.85);

	vector<TPoint3D> points;
	unsigned int cont = ( y1 == 0 ? 0 : faceHeight*(y1-1));
	CMatrixBool valids;

	valids.setSize(faceHeight,faceWidth);

	int total = 0;
	double sumDepth = 0;

	for ( unsigned int i = y1; i <= y2; i++ )
	{
		cont += x1;

		for ( unsigned int j = x1; j <= x2; j++, cont++ )
		{
			if (*(face->confidenceImage.get_unsafe( j, i, 0 )) > m_options.confidenceThreshold )
			{
				sumDepth += face->points3D_x[cont];
				total++;
				points.push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
			}
		}
		cont += faceWidth-x2-1;
	}

	double meanDepth = sumDepth / total;



	/*if (  m_measure.faceNum == 434  )
		experimental_viewFacePointsScanned( *face );*/

	//experimental_viewFacePointsScanned( points );

	bool res = true;

	if ( m_options.useSizeDistanceRelationFilter )
	{
		double maxFaceDistance = 0.5 + 1000 / ( pow( faceWidth, 1.9 ) );

		// To obtain experimental results
		{
			if ( m_measure.takeTime )
				m_timeLog.leave("Check if face plane: size-distance relation");

			if ( m_options.useDiagonalDistanceFilter && m_measure.takeTime )
				m_timeLog.leave("Check if face plane: diagonal distances");
		}

		/*if ( maxFaceDistance > meanDepth )
			return true;

		if ( !m_options.useDiagonalDistanceFilter )
			return false;*/

		if ( maxFaceDistance < meanDepth )
		{
			// Uncomment if you want to analyze the regions discarted by this filter
			/*ofstream f;
			f.open("deletedSIZEDISTANCE.txt", ofstream::app);
			f << m_measure.faceNum << endl;
			f.close();*/

			//if ( !m_options.useDiagonalDistanceFilter )
				return false;
			//else
			//	res = false;
		}

		if ( !m_options.useDiagonalDistanceFilter )
			return true;
	}

	ofstream f;
	/*f.open("relaciones1.txt", ofstream::app);
	f << faceWidth << endl;
	f.close();*/

	f.open("relaciones2.txt", ofstream::app);
	f << meanDepth << endl;
	f.close();

	//cout << m_measure.faceNum ;

	//experimental_viewFacePointsScanned( points );

	points.clear();

	cont = ( y1 == 1 ? 0 : faceHeight*(y1-1));

	for ( unsigned int i = y1; i <= y2; i++ )
	{
		cont += x1;

		for ( unsigned int j = x1; j <= x2; j++, cont++ )
		{
			if ( (*(face->confidenceImage.get_unsafe( j, i, 0 )) > m_options.confidenceThreshold ) )
				//&& ( face->points3D_x[cont] > meanDepth - max_desv )
				//&& ( face->points3D_x[cont] < meanDepth + max_desv ) )
			{
				valids.set_unsafe( i, j, true );
				points.push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
			}
			else
				valids.set_unsafe( i, j, false );


		}
		cont += faceWidth-x2-1;
	}

	/*if (  m_measure.faceNum > 838 )
		experimental_viewFacePointsScanned( points );*/

	//if ( ( m_measure.faceNum == 225 ) || ( m_measure.faceNum == 226 ) )
	//experimental_viewFacePointsScanned( points );

	double sumDistances = 0;
	double distance;
	int offsetIndex;

	cont = 0;

	for ( unsigned int i = y1; i <= y2; i++ )
	{
		cont += x1;

		for ( unsigned int j = x1; j <= x2; j++, cont++ )
		{
			if ( valids.get_unsafe( i, j ) )
			{
				//experimental_calcDiagDist( face, i, j, faceWidth, faceHeight, valids, distance );

				distance = 0;
				if ( ( i+1 <= y2 ) && ( j+1 <= x2 ) )
				{
					if ( valids.get_unsafe( i+1, j+1 ) )
					{
						TPoint3D p1( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] );
						offsetIndex = cont + faceWidth + 1;
						distance = p1.distanceTo( TPoint3D(face->points3D_x[offsetIndex], face->points3D_y[offsetIndex], face->points3D_z[offsetIndex]) );
					}
					else
					{
						bool validOffset = true;
						int offset = 2;

						while ( validOffset )
						{
							if ( ( i + offset <= y2 ) && ( j + offset <= x2 ) )
							{
								if ( valids.get_unsafe( i+offset, j+offset ) )
								{
									TPoint3D p1( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] );
									offsetIndex = cont + faceWidth + offset;
									distance = p1.distanceTo( TPoint3D(face->points3D_x[offsetIndex], face->points3D_y[offsetIndex], face->points3D_z[offsetIndex]) );
									break;
								}
								offset++;
							}
							else
								validOffset = false;
						}
					}
				}

				sumDistances += distance;
			}
		}
		cont += faceWidth-x2-1;
	}

	// For experimental results
	{
		if ( m_measure.takeMeasures )
			m_measure.sumDistances.push_back( sumDistances );

		ofstream f;
		f.open("distances.txt", ofstream::app);
		//f << m_measure.faceNum << " " << sumDistances << endl;
		f << sumDistances << endl;
		f.close();

		f.open("distances2.txt", ofstream::app);
		f << m_measure.faceNum << " " << sumDistances << endl;
		f.close();
	}

	//double yMax = 3 + 3.8 / ( pow( meanDepth, 2 ) );
	//double yMax = 3 + 7 /( pow( meanDepth, 2) ) ;
	double yMax = 3 + 6 /( pow( meanDepth, 2) ) ;
	double yMin = 1 + 3.8 / ( pow( meanDepth+1.2, 2 ) );

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.leave("Check if face plane: diagonal distances");
	}

	if ((( sumDistances <= yMax ) && ( sumDistances >= yMin  ))&&( res ) )
	{
		/* Uncomment if you want to analyze the real size of each studied region
		/ *ofstream f;
		f.open("sizes.txt", ofstream::app);
		double h = meanDepth/cos(DEG2RAD(faceHeight*0.2361111111111111));
		double realHigh = sin(DEG2RAD(faceHeight*0.2361111111111111))*h;
		f << realHigh << endl;
		f.close();*/

		return true;
	}

	// Uncomment if you want to analyze regions discarted by this filter
	/*if (( sumDistances > yMax ) || ( sumDistances < yMin  ))
	{
		ofstream f;
		f.open("deletedDIAGONAL.txt", ofstream::app);
		f << m_measure.faceNum << endl;
		f.close();
	}*/

	return false;


	MRPT_END
}


//------------------------------------------------------------------------
//							checkIfDiagonalSurface2
//------------------------------------------------------------------------

bool CFaceDetection::checkIfDiagonalSurface2( CObservation3DRangeScan* face )
{
	MRPT_START

	// To obtain experimental results
	{
		if ( m_options.useDiagonalDistanceFilter && m_measure.takeTime )
			m_timeLog.enter("Check if face plane: diagonal distances");

		if ( m_options.useSizeDistanceRelationFilter && m_measure.takeTime )
			m_timeLog.enter("Check if face plane: size-distance relation");
	}

	const unsigned int faceWidth = face->intensityImage.getWidth();
	const unsigned int faceHeight = face->intensityImage.getHeight();

	CMatrixTemplate<bool> region; // To save the segmented region
	experimental_segmentFace( *face,  region);

	size_t cont = 0;
	size_t total = 0;
	float sumDepth = 0;

	vector<TPoint3D> points;

	for ( unsigned int row = 0; row < faceHeight; row++ )
	{
		for ( unsigned int col = 0; col < faceWidth; col++, cont++ )
		{
			if ( ( region.get_unsafe( row, col ) ) &&
				(*(face->confidenceImage.get_unsafe( col, row, 0 )) > m_options.confidenceThreshold ))
			{
				sumDepth += face->points3D_x[cont];
				total++;
				points.push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
			}
		}
	}

	double meanDepth = sumDepth / total;

	bool res = true;

	if ( m_options.useSizeDistanceRelationFilter )
	{
		double maxFaceDistance = 0.5 + 1000 / ( pow( faceWidth, 1.9 ) );

		// To obtain experimental results
		{
			if ( m_measure.takeTime )
				m_timeLog.leave("Check if face plane: size-distance relation");

			if ( m_options.useDiagonalDistanceFilter && m_measure.takeTime )
				m_timeLog.leave("Check if face plane: diagonal distances");
		}

		/*if ( maxFaceDistance > meanDepth )
			return true;

		if ( !m_options.useDiagonalDistanceFilter )
			return false;*/

		if ( maxFaceDistance < meanDepth )
		{
			// Uncomment if you want to analyze the regions discarted by this filter
			/*ofstream f;
			f.open("deletedSIZEDISTANCE.txt", ofstream::app);
			f << m_measure.faceNum << endl;
			f.close();*/

			//if ( !m_options.useDiagonalDistanceFilter )
				return false;
			//else
			//	res = false;
		}

		if ( !m_options.useDiagonalDistanceFilter )
			return true;
	}

	ofstream f;
	/*f.open("relaciones1.txt", ofstream::app);
	f << faceWidth << endl;
	f.close();*/

	f.open("relaciones2.txt", ofstream::app);
	f << meanDepth << endl;
	f.close();

	//cout << m_measure.faceNum ;

	//experimental_viewFacePointsScanned( points );

	points.clear();

	/*if (  m_measure.faceNum > 838 )
		experimental_viewFacePointsScanned( points );*/

	//if ( ( m_measure.faceNum == 225 ) || ( m_measure.faceNum == 226 ) )
	//experimental_viewFacePointsScanned( points );

	double sumDistances = 0;
	double distance;
	size_t offsetIndex = 0;

	cont = 0;

	for ( unsigned int i = 0; i < faceHeight; i++ )
	{
		for ( unsigned int j = 0; j < faceWidth; j++, cont++ )
		{
			if ( region.get_unsafe( i, j ) )
			{
				distance = 0;
				if ( ( i+1 < faceHeight ) && ( j+1 < faceWidth ) )
				{
					if ( region.get_unsafe( i+1, j+1 ) )
					{
						TPoint3D p1( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] );
						offsetIndex = cont + faceWidth + 1;
						distance = p1.distanceTo( TPoint3D(face->points3D_x[offsetIndex], face->points3D_y[offsetIndex], face->points3D_z[offsetIndex]) );
					}
					else
					{
						bool validOffset = true;
						int offset = 2;

						while ( validOffset )
						{
							if ( ( i + offset < faceHeight ) && ( j + offset < faceWidth ) )
							{
								if ( region.get_unsafe( i+offset, j+offset ) )
								{
									TPoint3D p1( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] );
									offsetIndex = cont + faceWidth + offset;
									distance = p1.distanceTo( TPoint3D(face->points3D_x[offsetIndex], face->points3D_y[offsetIndex], face->points3D_z[offsetIndex]) );
									break;
								}
								offset++;
							}
							else
								validOffset = false;
						}
					}
				}

				sumDistances += distance;
			}
		}
	}

	// For experimental results
	{
		if ( m_measure.takeMeasures )
			m_measure.sumDistances.push_back( sumDistances );

		ofstream f;
		f.open("distances.txt", ofstream::app);
		//f << m_measure.faceNum << " " << sumDistances << endl;
		f << sumDistances << endl;
		f.close();

		/*f.open("distances2.txt", ofstream::app);
		f << m_measure.faceNum << " " << sumDistances << endl;
		f.close();*/
	}

	//double yMax = 3 + 3.8 / ( pow( meanDepth, 2 ) );
	//double yMax = 3 + 7 /( pow( meanDepth, 2) ) ;
	double yMax = 3 + 11.8 /( pow( meanDepth, 0.9) ) ;
	double yMin = 1 + 3.8 / ( pow( meanDepth+7, 6 ) );

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.leave("Check if face plane: diagonal distances");
	}

	if ((( sumDistances <= yMax ) && ( sumDistances >= yMin  ))&&( res ) )
	{
		/* Uncomment if you want to analyze the real size of each studied region
		/ *ofstream f;
		f.open("sizes.txt", ofstream::app);
		double h = meanDepth/cos(DEG2RAD(faceHeight*0.2361111111111111));
		double realHigh = sin(DEG2RAD(faceHeight*0.2361111111111111))*h;
		f << realHigh << endl;
		f.close();*/

		return true;
	}

	// Uncomment if you want to analyze regions discarted by this filter
	/*if (( sumDistances > yMax ) || ( sumDistances < yMin  ))
	{
		ofstream f;
		f.open("deletedDIAGONAL.txt", ofstream::app);
		f << m_measure.faceNum << endl;
		f.close();
	}*/

	return false;


	MRPT_END
}


//------------------------------------------------------------------------
//					experimental_viewFacePointsScanned
//------------------------------------------------------------------------

void CFaceDetection::experimental_viewFacePointsScanned( const CObservation3DRangeScan &face )
{
	vector<float> xs, ys, zs;

	unsigned int N = face.points3D_x.size();

	xs.resize(N);
	ys.resize(N);
	zs.resize(N);

	for ( unsigned int i = 0; i < N; i++ )
	{
		xs[i] = face.points3D_x[i];
		ys[i] = face.points3D_y[i];
		zs[i] = face.points3D_z[i];
	}

	experimental_viewFacePointsScanned( xs, ys, zs );
}


//------------------------------------------------------------------------
//					experimental_ViewFacePointsScanned
//------------------------------------------------------------------------

void CFaceDetection::experimental_viewFacePointsScanned( const vector<TPoint3D> &points )
{
	vector<float> xs, ys, zs;

	unsigned int N = points.size();

	xs.resize(N);
	ys.resize(N);
	zs.resize(N);

	for ( unsigned int i = 0; i < N; i++ )
	{
		xs[i] = points[i].x;
		ys[i] = points[i].y;
		zs[i] = points[i].z;
	}

	experimental_viewFacePointsScanned( xs, ys, zs );
}


//------------------------------------------------------------------------
//					experimental_viewFacePointsScanned
//------------------------------------------------------------------------

void CFaceDetection::experimental_viewFacePointsScanned( const vector<float> &xs, const vector<float> &ys, const vector<float> &zs )
{
	mrpt::gui::CDisplayWindow3D  win3D;

	win3D.setWindowTitle("3D Face detected (Scanned points)");

	win3D.resize(400,300);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(6.0);
	win3D.setCameraPointingToPoint(2.5,0,0);

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(4.5);

	mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

	scene->insert( gl_points );
	scene->insert( mrpt::opengl::CGridPlaneXY::Create() );

	CColouredPointsMap pntsMap;

	pntsMap.setAllPoints( xs, ys, zs );

	gl_points->loadFromPointsMap(&pntsMap);

	//gl_points->setColor(0,0.7,0.7,1);

	/*static int i = 0;

	if ( i == 2 )
	{
		mapa.setAllPoints( xs, ys, zs );
		i++;
	}
	else if ( i > 2 )
	{
		float run_time;
		CICP	icp;
		CICP::TReturnInfo	icp_info;

		icp.options.thresholdDist = 0.40;
		icp.options.thresholdAng = 0.40;

		CPose3DPDFPtr pdf= icp.Align3D(
		&mapa,    // Map to align
		&pntsMap,          // Reference map
		CPose3D(),    // Initial gross estimate
		&run_time,
		&icp_info);

		cout << "ICP run took " << run_time << " secs." << endl;
		cout << "Goodness: " << 100*icp_info.goodness << "%" << endl;
	}

	i++;*/

	win3D.unlockAccess3DScene();
	win3D.repaint();

	system::pause();
}


//------------------------------------------------------------------------
//				experimental_viewFacePointsAndEigenVects
//------------------------------------------------------------------------

void CFaceDetection::experimental_viewFacePointsAndEigenVects(  const vector<CArrayDouble<3> > &pointsVector, const CMatrixDouble &eigenVect, const CVectorDouble &eigenVal )
{

	vector<float> xs, ys, zs;

	const size_t size = pointsVector.size();

	xs.resize( size );
	ys.resize( size );
	zs.resize( size );

	for ( size_t i = 0; i < size; i++ )
	{
		xs[i] = pointsVector[i][0];
		ys[i] = pointsVector[i][1];
		zs[i] = pointsVector[i][2];
	}

	TPoint3D center( sum(xs)/size, sum(ys)/size, sum(zs)/size );

	mrpt::gui::CDisplayWindow3D  win3D;

	win3D.setWindowTitle("3D Face detected (Scanned points)");

	win3D.resize(400,300);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(6.0);
	win3D.setCameraPointingToPoint(2.5,0,0);

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(4.5);

	mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

	CSpherePtr sphere = CSphere::Create(0.005f);
	sphere->setLocation( center );
	sphere->setColor( TColorf(0,1,0) );
	scene->insert( sphere );

	//TPoint3D E1( eigenVect.get_unsafe(0,0), eigenVect.get_unsafe(1,0), eigenVect.get_unsafe(2,0) );
	//TPoint3D E2( eigenVect.get_unsafe(0,1), eigenVect.get_unsafe(1,1), eigenVect.get_unsafe(2,1) );
	//TPoint3D E3( eigenVect.get_unsafe(0,2), eigenVect.get_unsafe(1,2), eigenVect.get_unsafe(2,2) );

	TPoint3D E1( eigenVect.get_unsafe(0,0), eigenVect.get_unsafe(0,1), eigenVect.get_unsafe(0,2) );
	TPoint3D E2( eigenVect.get_unsafe(1,0), eigenVect.get_unsafe(1,1), eigenVect.get_unsafe(1,2) );
	TPoint3D E3( eigenVect.get_unsafe(2,0), eigenVect.get_unsafe(2,1), eigenVect.get_unsafe(2,2) );

	//vector<TSegment3D> sgms;

	TPoint3D p1( center + E1*eigenVal[0]*100 );
	TPoint3D p2( center + E2*eigenVal[1]*100 );
	TPoint3D p3( center + E3*eigenVal[2]*100 );

	CArrowPtr arrow1 = CArrow::Create( center.x, center.y, center.z, p1.x, p1.y, p1.z );
	CArrowPtr arrow2 = CArrow::Create( center.x, center.y, center.z, p2.x, p2.y, p2.z );
	CArrowPtr arrow3 = CArrow::Create( center.x, center.y, center.z, p3.x, p3.y, p3.z );

	arrow1->setColor( TColorf(0,1,0) );
	arrow2->setColor( TColorf(1,0,0) );
	arrow3->setColor( TColorf(0,0,1) );

	scene->insert( arrow1 );
	scene->insert( arrow2 );
	scene->insert( arrow3 );


	//sgms.push_back( TSegment3D(center,center + E1*eigenVal[0]*100) );
	//sgms.push_back( TSegment3D(center,center + E2*eigenVal[1]*100) );
	//sgms.push_back( TSegment3D(center,center + E3*eigenVal[2]*100) );
	//mrpt::opengl::CSetOfLinesPtr lines = mrpt::opengl::CSetOfLines::Create( sgms );
	//lines->setColor(0,0,1,1);
	//lines->setLineWidth( 10 );

	//scene->insert( lines );

	scene->insert( gl_points );
	scene->insert( mrpt::opengl::CGridPlaneXY::Create() );

	CColouredPointsMap pntsMap;

	pntsMap.setAllPoints( xs, ys, zs );

	gl_points->loadFromPointsMap(&pntsMap);

	win3D.unlockAccess3DScene();
	win3D.repaint();

	system::pause();
}


//------------------------------------------------------------------------
//						experimental_viewRegions
//------------------------------------------------------------------------

void CFaceDetection::experimental_viewRegions( const vector<TPoint3D> regions[9], const TPoint3D meanPos[3][3] )
{
	mrpt::gui::CDisplayWindow3D  win3D;

	win3D.setWindowTitle("3D Face detected (Scanned points)");

	win3D.resize(400,300);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(6.0);
	win3D.setCameraPointingToPoint(2.5,0,0);

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(6);

	mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

	if ( meanPos != NULL )
	{
		for ( size_t i = 0; i < 3; i++ )
			for ( size_t j = 0; j < 3; j++ )
			{
				CSpherePtr sphere = CSphere::Create(0.005f);
				sphere->setLocation( meanPos[i][j] );
				sphere->setColor( TColorf(0,1,0) );
				scene->insert( sphere );
			}
	}

	vector<TSegment3D> sgms;
	sgms.push_back( TSegment3D(meanPos[0][0],meanPos[0][1]) );
	sgms.push_back( TSegment3D(meanPos[0][1],meanPos[0][2]) );
	sgms.push_back( TSegment3D(meanPos[1][0],meanPos[1][1]) );
	sgms.push_back( TSegment3D(meanPos[1][1],meanPos[1][2]) );
	sgms.push_back( TSegment3D(meanPos[2][0],meanPos[2][1]) );
	sgms.push_back( TSegment3D(meanPos[2][1],meanPos[2][2]) );
	sgms.push_back( TSegment3D(meanPos[0][0],meanPos[1][1]) );
	sgms.push_back( TSegment3D(meanPos[1][1],meanPos[2][2]) );
	sgms.push_back( TSegment3D(meanPos[2][0],meanPos[1][1]) );
	sgms.push_back( TSegment3D(meanPos[1][1],meanPos[0][2]) );
	mrpt::opengl::CSetOfLinesPtr lines = mrpt::opengl::CSetOfLines::Create( sgms );
	lines->setColor(0,0,1,1);
	lines->setLineWidth( 10 );

	scene->insert( lines );

	scene->insert( gl_points );
	scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
	scene->insert(mrpt::opengl::CAxis::Create(-5,-5,-5,5,5,5,2.5,3,true));

	CColouredPointsMap pntsMap;

	vector<float> xs, ys, zs;

	for ( size_t i = 0; i < 9; i++ )
		for ( unsigned int j = 0; j < regions[i].size(); j++ )
		{
			xs.push_back( regions[i][j].x );
			ys.push_back( regions[i][j].y );
			zs.push_back( regions[i][j].z );
		}

	pntsMap.setAllPoints( xs, ys, zs );

	int cont = 0;
	float colors[9][3] = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1},{0.5f,0.25f,0},{0.5f,0,0.25f},{0,0.35f,0.5f}};
	for ( size_t i = 0; i < 9; i++ )
	{
		float R = colors[i][0];
		float G = colors[i][1];
		float B = colors[i][2];

		for ( unsigned int j = 0; j < regions[i].size(); j++, cont++ )
			pntsMap.setPointColor( cont, R, G, B);
	}

	gl_points->loadFromPointsMap(&pntsMap);
	//gl_points->setColorA(0.5);

	win3D.unlockAccess3DScene();
	win3D.repaint();

	system::pause();
}


//------------------------------------------------------------------------
//						experimental_segmentFace
//------------------------------------------------------------------------

void CFaceDetection::experimental_segmentFace( const CObservation3DRangeScan &face, CMatrixTemplate<bool> &region )
{
	const unsigned int faceWidth = face.intensityImage.getWidth();
	const unsigned int faceHeight = face.intensityImage.getHeight();

	region.setSize( faceWidth, faceHeight, true);

	unsigned int x1 = ceil(faceWidth*0.4);
	unsigned int x2 = floor(faceWidth*0.6);
	unsigned int y1 = ceil(faceHeight*0.4);
	unsigned int y2 = floor(faceHeight*0.6);

	region.setSize(faceHeight,faceWidth);
	CMatrixTemplate<size_t> toExpand;
	toExpand.setSize(faceHeight,faceWidth,true);

	unsigned int cont = ( y1 <= 1 ? 0 : faceHeight*(y1-1));

	//int total = 0;  // JL: Unused var
	//int numPoints = 0; // JL: Unused var

	mrpt::utils::CImage  img;
	// Normalize the image
	CMatrixFloat  range2D = m_lastFaceDetected.rangeImage;
	range2D *= 1.0f/5;
	img.setFromMatrix(range2D);

	// INITIALIZATION
	for ( unsigned int i = y1; i <= y2; i++ )
	{
		cont += x1;

		for ( unsigned int j = x1; j <= x2; j++, cont++ )
		{
			if (*(face.confidenceImage.get_unsafe( j, i, 0 )) > m_options.confidenceThreshold )
			{
				//unsigned char *c = img.get_unsafe(i,j);
				//size_t value = (size_t)*c;
				//total += value;
				//++numPoints;
				toExpand.set_unsafe(i,j,1);
			}
		}
		cont += faceWidth-x2;
	}

	//int mean = total / numPoints;

	//cout << "Mean: " << mean << endl;
	//system::pause();

	// UMBRALIZATION
/*
	for ( unsigned int row = 0; row < faceWidth; row++ )
	{
		for ( unsigned int col = 0; col < faceHeight; col++ )
		{
			unsigned char *c = img.get_unsafe(row,col);
			size_t value = (size_t)*c;

			if ( ( value < mean+7 ) && ( value > mean-7 ) )
			{
				region.set_unsafe( row, col, true );
			}else{
				img.setPixel( row, col, 0 );
			}
		}
	}
*/

	// REGIONS GROWING

	bool newExpanded = true;

	while ( newExpanded )
	{
		newExpanded = false;

		for ( size_t row = 0; row < faceHeight; row++ )
		{
			for ( size_t col = 0; col < faceWidth; col++ )
			{
				//cout << toExpand.get_unsafe( row, col ) << "" ;

				if ( toExpand.get_unsafe( row, col ) == 1 )
				{
					region.set_unsafe( row, col, true );

					unsigned char *c = img.get_unsafe(col,row);
					int value = (int)*c;

					if (( row > 0 ) && ( toExpand.get_unsafe(row-1,col) != 2 ))
					{
						unsigned char *c = img.get_unsafe(col, row-1);
						int value2 = (int)*c;
						if ( abs( value - value2 ) < 2 )
						{
							toExpand.set_unsafe(row-1,col,1);
							newExpanded = true;
						}
					}

					if (( row < faceWidth-1 ) && ( toExpand.get_unsafe(row+1,col) != 2 ))
					{
						unsigned char *c = img.get_unsafe(col, row+1);
						int value2 = (int)*c;
						if ( abs( value - value2 ) < 2 )
						{
							toExpand.set_unsafe(row+1,col,1);
							newExpanded = true;
						}
					}

					if (( col > 0) && ( toExpand.get_unsafe(row,col-1) != 2 ))
					{
						unsigned char *c = img.get_unsafe(col-1,row);
						int value2 = (int)*c;
						if ( abs( value - value2 ) < 2 )
						{
							toExpand.set_unsafe(row,col-1,1);
							newExpanded = true;
						}
					}

					if (( col < faceHeight-1) && ( toExpand.get_unsafe(row,col+1) != 2 ))
					{
						unsigned char *c = img.get_unsafe(col+1,row);
						int value2 = (int)*c;
						if ( abs( value - value2 ) < 2 )
						{
							toExpand.set_unsafe(row,col+1,1);
							newExpanded = true;
						}
					}

					toExpand.set_unsafe( row, col, 2 );
				}
			}
		}
	}

	for ( unsigned int row = 0; row < faceHeight; row++ )
	{
		for ( unsigned int col = 0; col < faceWidth; col++ )
		{
			if ( !(region.get_unsafe( row, col)) )
			{
				img.setPixel( col, row, 0 );
			}
		}
	}

	// Uncomment if you want to see the resultant region segmented
	if ( m_measure.faceNum >= 314 )
	{
		CDisplayWindow  win("Live video");

	win.showImage( img );
	system::pause();
	}
}


//------------------------------------------------------------------------
//						experimental_calcHist
//------------------------------------------------------------------------

void CFaceDetection::experimental_calcHist( const CImage &face, const size_t &c1, const size_t &r1, const size_t &c2,
											const size_t &r2, CMatrixTemplate<unsigned int> &hist )
{
	TImageSize size;
	face.getSize( size );
	for ( size_t row = r1; row <= r2 ; row++ )
		for ( size_t col = c1; col <= c2; col++ )
		{
			unsigned char *c = face.get_unsafe( col, row );
			size_t value = (size_t)*c;
			int count = hist.get_unsafe( 0, value ) + 1;
			hist.set_unsafe( 0, value, count );
		}

}


//------------------------------------------------------------------------
//					experimental_showMeasurements
//------------------------------------------------------------------------

void CFaceDetection::experimental_showMeasurements()
{
	// This method execution time is not critical because it's executed only at the end
	// or a few times in user application

	ofstream f;
	f.open("statistics.txt", ofstream::app);

	if ( m_measure.lessEigenVals.size() > 0 )
	{
		double meanEigenVal, stdEigenVal;
		double minEigenVal = *min_element( m_measure.lessEigenVals.begin(), m_measure.lessEigenVals.end() );
		double maxEigenVal = *max_element( m_measure.lessEigenVals.begin(), m_measure.lessEigenVals.end() );

		meanAndStd( m_measure.lessEigenVals, meanEigenVal, stdEigenVal );

		cout << endl << "Statistical data about eigen values calculated of regions detected as faces" << endl;
		cout << "Min eigenVal: " << minEigenVal << endl;
		cout << "Max eigenVal: " << maxEigenVal << endl;
		cout << "Mean eigenVal: " << meanEigenVal << endl;
		cout << "Standard Desv: " << stdEigenVal << endl;

		if ( m_measure.saveMeasurementsToFile )
		{
			f << endl << "Statistical data about eigen values calculated of regions detected as faces" << endl;
			f << "Min eigenVal: " << minEigenVal << endl;
			f << "Max eigenVal: " << maxEigenVal << endl;
			f << "Mean eigenVal: " << meanEigenVal << endl;
			f << "Standard Desv: " << stdEigenVal << endl;
		}
	}

	if ( m_measure.sumDistances.size() > 0 )
	{
		double meanSumDist, stdSumDist;
		double minSumDist = *min_element( m_measure.sumDistances.begin(), m_measure.sumDistances.end() );
		double maxSumDist = *max_element( m_measure.sumDistances.begin(), m_measure.sumDistances.end() );

		meanAndStd( m_measure.sumDistances, meanSumDist, stdSumDist );

		cout << endl << "Statistical data about sum of distances" << endl;
		cout << "Min sumDistances: " << minSumDist << endl;
		cout << "Max sumDistances: " << maxSumDist << endl;
		cout << "Mean sumDistances: " << meanSumDist << endl;
		cout << "Standard Desv: " << stdSumDist << endl;

		if ( m_measure.saveMeasurementsToFile )
		{
			f << endl << "Statistical data about sum of distances" << endl;
			f << "Min sumDistances: " << minSumDist << endl;
			f << "Max sumDistances: " << maxSumDist << endl;
			f << "Mean sumDistances: " << meanSumDist << endl;
			f << "Standard Desv: " << stdSumDist << endl;
		}
	}

	if ( m_measure.errorEstimations.size() > 0 )
	{
		double meanEstimationErr, stdEstimationErr;
		double minEstimationErr = *min_element( m_measure.errorEstimations.begin(), m_measure.errorEstimations.end() );
		double maxEstimationErr = *max_element( m_measure.errorEstimations.begin(), m_measure.errorEstimations.end() );

		meanAndStd( m_measure.errorEstimations, meanEstimationErr, stdEstimationErr );

		cout << endl << "Statistical data about estimation error adjusting a plane of regions detected as faces" << endl;
		cout << "Min estimation: " << minEstimationErr << endl;
		cout << "Max estimation: " << maxEstimationErr << endl;
		cout << "Mean estimation: " << meanEstimationErr << endl;
		cout << "Standard Desv: " << stdEstimationErr << endl;

		if ( m_measure.saveMeasurementsToFile )
		{
			f << endl << "Statistical data about estimation error adjusting a plane of regions detected as faces" << endl;
			f << "Min estimation: " << minEstimationErr << endl;
			f << "Max estimation: " << maxEstimationErr << endl;
			f << "Mean estimation: " << meanEstimationErr << endl;
			f << "Standard Desv: " << stdEstimationErr << endl;
		}
	}

	cout << endl << "Data about number of faces" << endl;
	cout << "Possible faces detected: " << m_measure.numPossibleFacesDetected << endl;
	cout << "Real faces detected: " << m_measure.numRealFacesDetected << endl;

	if ( m_meanHist.size() > 0 )
	{
		double minHist = *min_element( m_meanHist.begin(), m_meanHist.end() );
		double maxHist = *max_element( m_meanHist.begin(), m_meanHist.end() );
		double meanHist;
		double stdHist;
		meanAndStd( m_meanHist, meanHist, stdHist );


		cout << endl << "Mean hist: " << meanHist << endl;
		cout << "Min hist: " << minHist << endl;
		cout << "Max hist: " << maxHist << endl;
		cout << "Stdv: " << stdHist << endl;
	}

	if ( m_measure.saveMeasurementsToFile )
	{
		f << endl << "Data about number of faces" << endl;
		f << "Possible faces detected: " << m_measure.numPossibleFacesDetected << endl;
		f << "Real faces detected: " << m_measure.numRealFacesDetected << endl;
	}

	if ( m_measure.takeTime && m_measure.saveMeasurementsToFile )
		f << endl << m_timeLog.getStatsAsText();

	f.close();

	mrpt::system::pause();
}


//------------------------------------------------------------------------
//						debug_returnResults
//------------------------------------------------------------------------

void CFaceDetection::debug_returnResults( const vector_uint &falsePositives, const vector_uint &ignore, unsigned int &falsePositivesDeleted, unsigned int &realFacesDeleted )
{
	const unsigned int numDeleted = m_measure.deletedRegions.size();
	const unsigned int numFalsePositives = falsePositives.size();
	const unsigned int numIgnored = ignore.size();
	unsigned int ignoredDetected = 0;

	falsePositivesDeleted = 0;

	for ( unsigned int i = 0; i < numDeleted; i++ )
	{
		unsigned int region = m_measure.deletedRegions[i];

		bool falsePositive = false;

		unsigned int j = 0;
		while (!falsePositive && ( j < numFalsePositives ) )
		{
			if ( region == falsePositives[j] ) falsePositive = true;
			j++;
		}

		if ( falsePositive )
			falsePositivesDeleted++;
		else
		{
			bool igno = false;

			j = 0;
			while (!igno && ( j < numIgnored ) )
			{
				if ( region == ignore[j] ) igno = true;
				j++;
			}

			if ( igno )
				ignoredDetected++;
		}
	}

	realFacesDeleted = numDeleted - falsePositivesDeleted - ignoredDetected;

	m_measure.faceNum = 0;
	m_measure.deletedRegions.clear();
}
