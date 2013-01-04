/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

	/** \ingroup mrpt_detectors_grp  */
	namespace detectors
	{
		using namespace mrpt::slam;
		using namespace mrpt::system;
		using namespace mrpt::synch;
		
		/** Specific class for face detection.
		  * Methods and variables labeled as experimentals are temporals (for debug or testing
		  * purposes) and may disappear in future versions.
		  * \ingroup mrpt_detectors_grp
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
				bool	multithread;

				bool	useCovFilter;
				bool	useRegionsFilter;
				bool	useSizeDistanceRelationFilter;
				bool	useDiagonalDistanceFilter;

				bool	batchMode;

			}m_options;

			struct TTestsOptions
			{
				double	planeThreshold;
				double	planeTest_eigenVal_top;
				double	planeTest_eigenVal_bottom;
				double	regionsTest_sumDistThreshold_top;				
				double	regionsTest_sumDistThreshold_bottom;				

			}m_testsOptions;

			// Experimental methods
			void experimental_showMeasurements();

			void debug_returnResults( const vector_uint &falsePositives, const vector_uint &ignore, unsigned int &falsePositivesDeleted, unsigned int &realFacesDeleted );
			
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
				vector_uint		deletedRegions;
				int				numPossibleFacesDetected;
				int				numRealFacesDetected;

				bool			takeTime;

				bool			saveMeasurementsToFile;

			}m_measure;

			// To take measures abaout execution time
			CTimeLogger	m_timeLog;

			std::vector<double> m_meanHist;


			// Test to check if a candidate region is a real face

			bool checkIfFacePlane( CObservation3DRangeScan* face );

			bool checkIfFacePlaneCov( CObservation3DRangeScan* face );

			void thread_checkIfFacePlaneCov( );

			static void dummy_checkIfFacePlaneCov( CFaceDetection *obj );


			bool checkIfFaceRegions( CObservation3DRangeScan* face );

			void thread_checkIfFaceRegions( );

			static void dummy_checkIfFaceRegions( CFaceDetection *obj );

			size_t checkRelativePosition( const TPoint3D &p1, const TPoint3D &p2, const TPoint3D &p, double &dist );


			void thread_checkIfDiagonalSurface( );

			bool checkIfDiagonalSurface( CObservation3DRangeScan* face );

			bool checkIfDiagonalSurface2( CObservation3DRangeScan* face );

			static void dummy_checkIfDiagonalSurface( CFaceDetection *obj );

			// Experimental methods to view 3D points

			void experimental_viewFacePointsScanned( const std::vector<float> &xs, const std::vector<float> &ys, const std::vector<float> &zs );

			void experimental_viewFacePointsScanned( const CObservation3DRangeScan &face );
			
			void experimental_viewFacePointsScanned( const std::vector<TPoint3D> &points );

			void experimental_viewFacePointsAndEigenVects(  const std::vector<CArrayDouble<3> > &pointsVector, const CMatrixDouble &eigenVect, const vector_double &eigenVal );

			void experimental_viewRegions( const std::vector<TPoint3D> regions[9], const TPoint3D meanPos[3][3] );		

			// Segmentation methods

			void experimental_segmentFace( const CObservation3DRangeScan &face, CMatrixTemplate<bool> &region );

			// Histogram methods

			void experimental_calcHist( const CImage &face, const size_t &c1, const size_t &r1, const size_t &c2, 
										const size_t &r2, CMatrixTemplate<unsigned int> &hist );

			

		}; // End of class
	}

}

#endif
