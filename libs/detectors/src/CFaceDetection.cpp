/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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


#include <mrpt/detectors.h>  // Precompiled headers
#include <mrpt/gui.h>
#include <mrpt/slam/CColouredPointsMap.h>

#include <mrpt/detectors/CFaceDetection.h>
#include <mrpt/math/geometry.h>



using namespace mrpt::detectors;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::system;

//------------------------------------------------------------------------
//							CFaceDetection
//------------------------------------------------------------------------
CFaceDetection::CFaceDetection() : m_enter_checkIfFacePlaneCov(0,1,"enter_checkIfFacePlaneCov"),
								m_enter_checkIfFaceRegions(0,1,"enter_checkIfFaceRegions"),
								m_leave_checkIfFacePlaneCov(0,1,"leave_checkIfFacePlaneCov"),
								m_leave_checkIfFaceRegions(0,1,"enter_checkIfFaceRegions"),	
								m_end_checkIfFaceRegions(false),
								m_end_checkIfFacePlaneCov(false)

			
{
	// Run filters threads

	m_thread_checkIfFaceRegions = createThread( dummy_checkIfFaceRegions, this );
	m_thread_checkIfFacePlaneCov = createThread( dummy_checkIfFacePlaneCov, this );	

	m_measure.numPossibleFacesDetected = 0;
	m_measure.numRealFacesDetected = 0;

	m_timeLog.enable();
}


//------------------------------------------------------------------------
//							~CFaceDetection
//------------------------------------------------------------------------
CFaceDetection::~CFaceDetection() 			
{
	// Stop filters threads

	m_end_checkIfFaceRegions	= true;
	m_end_checkIfFacePlaneCov	= true;

	m_enter_checkIfFacePlaneCov.release();
	m_enter_checkIfFaceRegions.release();

	joinThread(m_thread_checkIfFaceRegions);
	joinThread(m_thread_checkIfFacePlaneCov);
}

//------------------------------------------------------------------------
//								init
//------------------------------------------------------------------------
void CFaceDetection::init(const mrpt::utils::CConfigFileBase &cfg )
{
	m_options.confidenceThreshold	= cfg.read_int( "FaceDetection", "confidenceThreshold", 200 );
	m_options.planeThreshold		= cfg.read_double( "FaceDetection", "planeThreshold", 50 );
	m_options.planeEigenValThreshold= cfg.read_double( "FaceDetection", "planeEigenValThreshold", 0 );
	m_options.regionsThreshold		= cfg.read_double( "FaceDetection", "regionsThreshold", 0.5 );
	m_options.multithread			= cfg.read_bool( "FaceDetection", "multithread", true );

	m_measure.takeTime				= cfg.read_bool( "FaceDetection", "takeTime", false );
	m_measure.takeMeasures			= cfg.read_bool( "FaceDetection", "takeMeasures", false );
	m_measure.saveMeasurementsToFile= cfg.read_bool( "FaceDetection", "saveMeasurementsToFile", false );
	
	cascadeClassifier.init( cfg );
}


//------------------------------------------------------------------------
//							detectObjects
//------------------------------------------------------------------------
void CFaceDetection::detectObjects_Impl(const mrpt::slam::CObservation *obs, vector_detectable_object &detected)
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

		if ( m_measure.takeMeasures )
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
						m_timeLog.enter("Multithread filters aplication");
					}

					/*m_timeLog.enter("parte1");
					CObservation3DRangeScan face;
					o->getZoneAsObs( face, r1, r2, c1, c2 );
					m_timeLog.leave("parte1");
					
					m_timeLog.enter("parte2");
					TThreadParams t_params1;
					t_params1.res = false;
					t_params1.obj = this;
					t_params1.face = &face;
					TThreadHandle thread1 = createThread( dummy_checkIfFacePlaneCov, static_cast<void *>(&t_params1) );
					m_timeLog.leave("parte2");


					m_timeLog.enter("parte3");
					TThreadParams t_params2;
					t_params2.res = false;
					t_params2.obj = this;
					t_params2.face = &face;
					TThreadHandle thread2 = createThread( dummy_checkIfFaceRegions, static_cast<void *>(&t_params2) );
					m_timeLog.leave("parte3");
					
					m_timeLog.enter("parte4");
					joinThread( thread2 );
					joinThread( thread1 );
					m_timeLog.leave("parte4");

					m_timeLog.enter("parte5");
					if ( t_params1.res || !t_params2.res )
						deleteDetected.push_back( i );
					m_timeLog.leave("parte5");*/

					m_enter_checkIfFacePlaneCov.release();
					m_enter_checkIfFaceRegions.release();

					m_leave_checkIfFacePlaneCov.waitForSignal();
					m_leave_checkIfFaceRegions.waitForSignal();
	
					if ( m_checkIfFacePlaneCov_res || m_checkIfFaceRegions_res )
						deleteDetected.push_back( i );

					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.leave("Multithread filters aplication");
					}
				}
				else
				{				
					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.enter("Secuential filters aplication");
					}

					static int picNum = 0;

					if ( picNum == 326 )
						int a = 3;

					//face.intensityImage.saveToFile(format("%ia.jpg",picNum));

					bool remove = false;

					// First check if we can adjust a plane to detected region as face, if yes it isn't a face!
					if ( checkIfFacePlaneCov( &m_lastFaceDetected ) )
					{
						deleteDetected.push_back( i );
						remove = true;
					}
					else
					{			
						//experimental_viewFacePointsScanned( face );
						if ( !checkIfFaceRegions( &m_lastFaceDetected ) )
						{
							deleteDetected.push_back( i );
							remove = true;
						}
					}

					/*if (!remove)
						face.intensityImage.saveToFile(format("%ib.jpg",picNum));*/

					picNum++;

					// To obtain experimental results
					{
						if ( m_measure.takeTime )
						m_timeLog.leave("Secuential filters aplication");
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
		if ( m_measure.takeMeasures )
			m_measure.numRealFacesDetected += detected.size();
	}

	MRPT_END

}


//------------------------------------------------------------------------
//  						checkIfFacePlane
//------------------------------------------------------------------------
bool CFaceDetection::checkIfFacePlane( const vector<TPoint3D> &points )
{
	// Try to ajust a plane
	TPlane plane;

	// To obtain experimental results
	{
		if ( m_measure.takeMeasures )
			m_measure.errorEstimations.push_back( (double)getRegressionPlane(points,plane) );		
	}

	if ( getRegressionPlane(points,plane) < m_options.planeThreshold )
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

		if ( m_end_checkIfFacePlaneCov )
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

	const unsigned int faceWidth = face->intensityImage.getWidth();
	const unsigned int faceHeight = face->intensityImage.getHeight();	

	const bool confidence = face->hasConfidenceImage;

	vector<CArrayDouble<3> > pointsVector;

	for ( unsigned int j = 0; j < faceHeight; j++ )
	{
		for ( unsigned int k = 0; k < faceWidth; k++ )
		{
			CArrayDouble<3> aux;

			if ( (!confidence) || (( confidence ) && 
				( *(face->confidenceImage.get_unsafe( k, j, 0 )) > m_options.confidenceThreshold )) )
			{
				int position = faceWidth*j + k;
				aux[0] = face->points3D_x[position];
				aux[1] = face->points3D_y[position];
				aux[2] = face->points3D_z[position];
				pointsVector.push_back( aux );
			}
		}
	}

	CMatrixDouble cov;
	vector_double eVals;

	cov = covVector( pointsVector ); 

	cov.eigenValues( eVals );

	CMatrixDouble eVec;
	CMatrixDouble eVal;
	cov.eigenVectors( eVec, eVal );

	/*cout << "Size: " << eVec.size() << endl;
	cout << eVec.get_unsafe(0,0) << eVec.get_unsafe(0,1) << eVec.get_unsafe(0,2) << endl;
	cout << eVec.get_unsafe(1,0) << eVec.get_unsafe(1,1) << eVec.get_unsafe(1,2) << endl;
	cout << eVec.get_unsafe(2,0) << eVec.get_unsafe(2,1) << eVec.get_unsafe(2,2) << endl;*/

	// To obtain experimental results
	{
		if ( m_measure.takeMeasures )
			m_measure.lessEigenVals.push_back(eVals[0]);
	}

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.leave("Check if face plane: covariance");
	}

	if ( eVals[0] < m_options.planeEigenValThreshold )
		return true;

	return false;

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

		if ( m_end_checkIfFaceRegions )
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

	const unsigned int faceWidth = face->intensityImage.getWidth();
	const unsigned int faceHeight = face->intensityImage.getHeight();

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.enter("Check if face plane: regions");
	}
	
	unsigned int x1 = ceil(faceWidth*0.1);
	unsigned int x2 = floor(faceWidth*0.9);
	unsigned int y1 = ceil(faceHeight*0.1);
	unsigned int y2 = floor(faceHeight*0.9);

	unsigned int sectionVSize = floor((y2-y1)/3.0);
	unsigned int sectionHSize = floor((x2-x1)/3.0);

	vector<TPoint3D> points;
	unsigned int cont = 0;

	double meanDepth[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
	int numPoints[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };

	//vector<TPoint3D> regions2[9];

	for ( unsigned int i = y1; i <= y2; i++ )
	{
		cont += x1;

		for ( unsigned int j = x1; j <= x2; j++, cont++ )
		{
			if (*(face->confidenceImage.get_unsafe( j, i, 0 )) > m_options.confidenceThreshold )
			{
				unsigned int row, col;
				if ( i-y1 < sectionVSize - floor(sectionVSize*0.1) )
					row = 0;
				else if ( i-y1 < sectionVSize*2 + floor(sectionVSize*0.1) )
					row = 1;
				else
					row = 2;

				if ( j-x1 < sectionHSize - floor(sectionHSize*0.1) )
					col = 0;
				else if ( j-x1 < sectionHSize*2 + floor(sectionHSize*0.1) )
					col = 1;
				else
					col = 2;


				//cout << ">" << face->points3D_x[cont] << "." << face->points3D_y[cont] << "." << face->points3D_z[cont] << endl;

				meanDepth[row][col]+=face->points3D_x[cont];
				++numPoints[row][col];

				//if ( row == 0 && col == 0 )
				//	regions2[0].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 0 && col == 1 )
				//	regions2[1].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 0 && col == 2 )
				//	regions2[2].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 1 && col == 0 )
				//	regions2[3].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 1 && col == 1 )
				//	regions2[4].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 1 && col == 2 )
				//	regions2[5].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 2 && col == 0 )
				//	regions2[6].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else if ( row == 2 && col == 1 )
				//	regions2[7].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
				//else
				//	regions2[8].push_back( TPoint3D( face->points3D_x[cont], face->points3D_y[cont], face->points3D_z[cont] ) );
			
			}
		}

		cont += faceWidth-x2;
	}
	
	// Create 9 regions and calculate
	vector<vector<TPoint3D> > regions;

	for ( size_t i = 0; i < 3; i++ )
		for ( size_t j = 0; j < 3; j++ )
			if ( !numPoints[i][j] )
				meanDepth[i][j] = 0;
			else
				meanDepth[i][j] /= numPoints[i][j];

	/*cout << endl << meanDepth[0][0] << "\t" << meanDepth[0][1] << "\t" << meanDepth[0][2] << endl;
	cout << endl << meanDepth[1][0] << "\t" << meanDepth[1][1] << "\t" << meanDepth[1][2] << endl;
	cout << endl << meanDepth[2][0] << "\t" << meanDepth[2][1] << "\t" << meanDepth[2][2] << endl;*/

	//experimental_viewRegions( regions2 );

	// For experimental results
	{
		if ( m_measure.takeMeasures )
		{
			m_measure.meanRegions.push_back( meanDepth[0][0] );
			m_measure.meanRegions.push_back( meanDepth[0][1] );
			m_measure.meanRegions.push_back( meanDepth[0][2] );
			m_measure.meanRegions.push_back( meanDepth[1][0] );
			m_measure.meanRegions.push_back( meanDepth[1][1] );
			m_measure.meanRegions.push_back( meanDepth[1][2] );
			m_measure.meanRegions.push_back( meanDepth[2][0] );
			m_measure.meanRegions.push_back( meanDepth[2][1] );
			m_measure.meanRegions.push_back( meanDepth[2][2] );
		}
	}

	// To obtain experimental results
	{
		if ( m_measure.takeTime )
		m_timeLog.leave("Check if face plane: regions");
	}

	return checkRegionsConstrains( meanDepth );

	MRPT_END
}


//------------------------------------------------------------------------
//						 checkRegionsConstrains
//------------------------------------------------------------------------
bool CFaceDetection::checkRegionsConstrains( const double values[3][3] )
{
	// This matrix put, for instance, in 0,0 an 1 if this region is farther that 0,1 region
	double satisfy[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };

	if ( values[0][0] > values[0][1] )
		satisfy[0][0] = 1;
	if ( values[0][2] > values[0][1] )
		satisfy[0][2] = 1;
	if ( values[1][0] > values[1][1] )
		satisfy[1][0] = 1;
	if ( values[1][2] > values[1][1] )
		satisfy[1][2] = 1;
	if ( values[2][0] > values[2][1] )
		satisfy[2][0] = 1;
	if ( values[2][2] > values[2][1] )
		satisfy[2][2] = 1;

	size_t sumCol0	= satisfy[0][0] + satisfy[1][0] + satisfy[2][0];
	size_t sumCol2	= satisfy[0][2] + satisfy[1][2] + satisfy[2][2];
	//size_t sumFil0 = satisfy[0][0] + satisfy[0][1] + satisfy[0][2];
	//size_t sumFil2 = satisfy[2][0] + satisfy[2][1] + satisfy[2][2];

	// We can choose any constrain to require to candidate face scan
	if ( ( sumCol0 >= 2 ) && ( sumCol2 >= 2 ) ) // Frontal faces
		return true;
	if ( ( sumCol0 == 3 ) || ( sumCol2 == 3 ) ) // Profile faces
		return true;

	return false;
}

void CFaceDetection::experimental_viewFacePointsScanned( const CObservation3DRangeScan &face )
{
	vector_float xs, ys, zs;

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

void CFaceDetection::experimental_viewFacePointsScanned( const vector<TPoint3D> &points )
{
	vector_float xs, ys, zs;

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

void CFaceDetection::experimental_viewFacePointsScanned( const vector_float &xs, const vector_float &ys, const vector_float &zs )
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

	win3D.unlockAccess3DScene();
	win3D.repaint();

	system::pause();
}

void CFaceDetection::experimental_viewRegions( const vector<TPoint3D> regions[9] )
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

	scene->insert( gl_points );
	scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
	scene->insert(mrpt::opengl::CAxis::Create(-5,-5,-5,5,5,5,2.5,3,true));
		
	CColouredPointsMap pntsMap;

	vector_float xs, ys, zs;

	for ( size_t i = 0; i < 9; i++ )
		for ( unsigned int j = 0; j < regions[i].size(); j++ )
		{
			xs.push_back( regions[i][j].x );
			ys.push_back( regions[i][j].y );
			zs.push_back( regions[i][j].z );
		}

	pntsMap.setAllPoints( xs, ys, zs );

	int cont = 0;
	float colors[9][3] = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1},{0.5,0.25,0},{0.5,0,0.25},{0,0.35,0.5}};
	for ( size_t i = 0; i < 9; i++ )
	{
		float R = colors[i][0];
		float G = colors[i][1];
		float B = colors[i][2];

		for ( unsigned int j = 0; j < regions[i].size(); j++, cont++ )
			pntsMap.setPointColor( cont, R, G, B);
	}
		
	gl_points->loadFromPointsMap(&pntsMap);
	
	win3D.unlockAccess3DScene();
	win3D.repaint();

	system::pause();
}

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


	if ( m_measure.meanRegions.size() > 0 )
	{
		vector_double	meanRegions[9];
		double			sumRegionsConstrains[9] = {0,0,0,0,0,0,0,0,0};

		unsigned int N = m_measure.meanRegions.size();

		for ( unsigned int i = 0; i < m_measure.meanRegions.size(); i++ )
			meanRegions[i%9].push_back( m_measure.meanRegions[i] );

		/*	Regions
			0	1	2
			3	4	5
			6	7	8
		*/
		int passed1 = 0, passed2 = 0;
		for ( unsigned int i = 0; i < (unsigned int)(N/9) ; i++ )
		{
			int col0 = 0, col2 = 0;
			if ( meanRegions[0][i] > meanRegions[1][i] )
			{
				sumRegionsConstrains[0]+=1;
				col0++;
			}
			if ( meanRegions[2][i] > meanRegions[1][i] )
			{
				sumRegionsConstrains[1]+=1;
				col2++;
			}
			if ( meanRegions[3][i] > meanRegions[4][i] )
			{
				col0++;
				sumRegionsConstrains[2]+=1;
			}
			if ( meanRegions[5][i] > meanRegions[4][i] )
			{
				sumRegionsConstrains[3]+=1;
				col2++;
			}
			if ( meanRegions[6][i] > meanRegions[7][i] )
			{
				col0++;
				sumRegionsConstrains[4]+=1;
			}
			if ( meanRegions[8][i] > meanRegions[7][i] )
			{
				sumRegionsConstrains[5]+=1;
				col2++;
			}

			if ( ( col0 >= 2) && (col2 >=2) ) 
				passed1++;				
			else if( ( col0 == 3 ) || (col2 == 3) )
				passed2++;

			
		}

		cout << endl << "Information about face regions restrictions: " << endl;
		cout << "Regions:" << endl;
		cout << "0 1 2" << endl;
		cout << "3 4 5" << endl;
		cout << "6 7 8" << endl;
		cout << "Restriction #1 (0>1): " << sumRegionsConstrains[0]/(N/9) << endl;
		cout << "Restriction #2 (2>1): " << sumRegionsConstrains[1]/(N/9) << endl;
		cout << "Restriction #3 (3>4): " << sumRegionsConstrains[2]/(N/9) << endl;
		cout << "Restriction #4 (5>4): " << sumRegionsConstrains[3]/(N/9) << endl;
		cout << "Restriction #5 (6>7): " << sumRegionsConstrains[4]/(N/9) << endl;
		cout << "Restriction #6 (8>7): " << sumRegionsConstrains[5]/(N/9) << endl;
		cout << "Satisfy frontal restrictions: " << passed1 << endl;
		cout << "Satisfy profile restrictions: " << passed2 << endl;

		if ( m_measure.saveMeasurementsToFile )
		{
			f << endl << "Information about face regions restrictions: " << endl;
			f << "Regions:" << endl;
			f << "0 1 2" << endl;
			f << "3 4 5" << endl;
			f << "6 7 8" << endl;
			f << "Restriction #1 (0>1): " << sumRegionsConstrains[0]/(N/9) << endl;
			f << "Restriction #2 (2>1): " << sumRegionsConstrains[1]/(N/9) << endl;
			f << "Restriction #3 (3>4): " << sumRegionsConstrains[2]/(N/9) << endl;
			f << "Restriction #4 (5>4): " << sumRegionsConstrains[3]/(N/9) << endl;
			f << "Restriction #5 (6>7): " << sumRegionsConstrains[4]/(N/9) << endl;
			f << "Restriction #6 (8>7): " << sumRegionsConstrains[5]/(N/9) << endl;
			f << "Satisfy frontal restrictions: " << passed1 << endl;
			f << "Satisfy profile restrictions: " << passed2 << endl;
		}
	}

	cout << endl << "Data about number of faces" << endl;
	cout << "Possible faces detected: " << m_measure.numPossibleFacesDetected << endl;
	cout << "Real faces detected: " << m_measure.numRealFacesDetected << endl;

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
