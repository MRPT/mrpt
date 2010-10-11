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

#ifndef CFaceDetection_H
#define CFaceDetection_H

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/detectors/CCascadeClassifierDetection.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system.h>
#include <mrpt/synch.h>
#include <mrpt/slam/CObservation3DRangeScan.h>

namespace mrpt
{

	namespace slam { class CObservation3DRangeScan; }

	namespace detectors
	{
		using namespace mrpt::slam;
		using namespace mrpt::system;
		using namespace mrpt::synch;
		
		/** Specific class for face detection.
		  * Methods and variables labeled as experimentals are temporals (for debug or testing
		  * purposes) and may disappear in future versions.
		  */
		class DETECTORS_IMPEXP CFaceDetection: public CObjectDetection
		{
		public:
		
			CCascadeClassifierDetection cascadeClassifier;

			CFaceDetection();

			~CFaceDetection();

			virtual void init(const mrpt::utils::CConfigFileBase &cfg );

			virtual void detectObjects_Impl(const CObservation *obs, vector_detectable_object &detected);
			
			struct TOptions
			{
				int		confidenceThreshold;
				double	planeThreshold;
				double	planeEigenValThreshold_down;
				double	planeEigenValThreshold_up;
				double	regionsThreshold;
				bool	multithread;

				bool	useCovFilter;
				bool	useRegionsFilter;
				bool	useSizeDistanceRelationFilter;
				bool	useDiagonalDistanceFilter;

			}m_options;

			// Experimental methods
			void experimental_showMeasurements();
			
		private:

			TThreadHandle		m_thread_checkIfFaceRegions;	//!< Thread that execute checkIfFaceRegions filter
			TThreadHandle		m_thread_checkIfFacePlaneCov;	//!< Thread that execute checkIfFacePlaneCov filter
			TThreadHandle		m_thread_checkIfDiagonalSurface;	//!< Thread that execute checkIfDiagonalSurface filter

			bool	m_checkIfFaceRegions_res;	//!< Save result of checkIfFaceRegions filter
			bool	m_checkIfFacePlaneCov_res;	//!< Save result of checkIfFacePlaneCov filter
			bool	m_checkIfDiagonalSurface_res;	//!< Save result of checkIfDiagonalSurface filter

			bool	m_end_threads;	//!< Indicates to all threads that must finish their execution
			
			CSemaphore m_enter_checkIfFaceRegions;	//!< Indicates to thread_checkIfFaceRegions that exist a new face to analyze
			CSemaphore m_enter_checkIfFacePlaneCov;	//!< Indicates to thread_checkIfFacePlaneCov that exist a new face to analyze
			CSemaphore m_enter_checkIfDiagonalSurface;	//!< Indicates to thread_checkIfDiagonalSurface that exist a new face to analyze

			CSemaphore m_leave_checkIfFaceRegions;	//!< Indicates to main thread that thread_checkIfFaceRegions has been completed analisis of the last face detected
			CSemaphore m_leave_checkIfFacePlaneCov;	//!< Indicates to main thread that thread_checkIfFacePlaneCov has been completed analisis of the last face detected
			CSemaphore m_leave_checkIfDiagonalSurface;	//!< Indicates to main thread that thread_checkIfDiagonalSurface has been completed analisis of the last face detected

			CObservation3DRangeScan m_lastFaceDetected;	//!< Last face detected

			struct TMeasurement
			{	
				bool			takeMeasures;

				vector_double	lessEigenVals;
				vector_double	errorEstimations;
				vector_double	meanRegions;

				vector_double	sumDistances;

				int				faceNum;
				int				numPossibleFacesDetected;
				int				numRealFacesDetected;

				bool			takeTime;

				bool			saveMeasurementsToFile;

			}m_measure;

			CTimeLogger	m_timeLog;

			vector<double> m_meanHist;


			bool checkIfFacePlane( const vector<TPoint3D> &points );

			bool checkIfFacePlaneCov( CObservation3DRangeScan* face );

			void thread_checkIfFacePlaneCov( );

			static void dummy_checkIfFacePlaneCov( CFaceDetection *obj );

			bool checkIfFaceRegions2( CObservation3DRangeScan* face );

			bool checkIfFaceRegions( CObservation3DRangeScan* face );

			void thread_checkIfFaceRegions( );

			static void dummy_checkIfFaceRegions( CFaceDetection *obj );

			bool checkRegionsConstrains( const double values[3][3] );

			void thread_checkIfDiagonalSurface( );

			bool checkIfDiagonalSurface( CObservation3DRangeScan* face );

			static void dummy_checkIfDiagonalSurface( CFaceDetection *obj );

			bool checkRelativePosition( const TPoint3D &p1, const TPoint3D &p2, const TPoint3D &p );

			// Experimental methods
			void experimental_viewFacePointsScanned( const vector_float &xs, const vector_float &ys, const vector_float &zs );

			void experimental_viewFacePointsScanned( const CObservation3DRangeScan &face );
			
			void experimental_viewFacePointsScanned( const vector<TPoint3D> &points );

			void experimental_viewRegions( const vector<TPoint3D> regions[9], const TPoint3D meanPos[3][3] );		

			void experimental_segmentFace( const CObservation3DRangeScan &face, CMatrixTemplate<bool> &region );

			void experimental_calcHist( const CImage &face, CMatrixTemplate<unsigned int> &hist );

		}; // End of class
	}

}

#endif