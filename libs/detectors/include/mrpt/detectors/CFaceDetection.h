/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CFaceDetection_H
#define CFaceDetection_H

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/detectors/CCascadeClassifierDetection.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/threads.h>
#include <mrpt/synch/CSemaphore.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
	/** \ingroup mrpt_detectors_grp  */
	namespace detectors
	{
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

			virtual void detectObjects_Impl(const mrpt::obs::CObservation *obs, vector_detectable_object &detected);
			
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

			mrpt::system::TThreadHandle		m_thread_checkIfFaceRegions;	//!< Thread that execute checkIfFaceRegions filter
			mrpt::system::TThreadHandle		m_thread_checkIfFacePlaneCov;	//!< Thread that execute checkIfFacePlaneCov filter
			mrpt::system::TThreadHandle		m_thread_checkIfDiagonalSurface;	//!< Thread that execute checkIfDiagonalSurface filter

			bool	m_checkIfFaceRegions_res;	//!< Save result of checkIfFaceRegions filter
			bool	m_checkIfFacePlaneCov_res;	//!< Save result of checkIfFacePlaneCov filter
			bool	m_checkIfDiagonalSurface_res;	//!< Save result of checkIfDiagonalSurface filter

			bool	m_end_threads;	//!< Indicates to all threads that must finish their execution
			
			mrpt::synch::CSemaphore m_enter_checkIfFaceRegions;	//!< Indicates to thread_checkIfFaceRegions that exist a new face to analyze
			mrpt::synch::CSemaphore m_enter_checkIfFacePlaneCov;	//!< Indicates to thread_checkIfFacePlaneCov that exist a new face to analyze
			mrpt::synch::CSemaphore m_enter_checkIfDiagonalSurface;	//!< Indicates to thread_checkIfDiagonalSurface that exist a new face to analyze

			mrpt::synch::CSemaphore m_leave_checkIfFaceRegions;	//!< Indicates to main thread that thread_checkIfFaceRegions has been completed analisis of the last face detected
			mrpt::synch::CSemaphore m_leave_checkIfFacePlaneCov;	//!< Indicates to main thread that thread_checkIfFacePlaneCov has been completed analisis of the last face detected
			mrpt::synch::CSemaphore m_leave_checkIfDiagonalSurface;	//!< Indicates to main thread that thread_checkIfDiagonalSurface has been completed analisis of the last face detected

			mrpt::obs::CObservation3DRangeScan m_lastFaceDetected;	//!< Last face detected

			struct TMeasurement
			{	
				bool			takeMeasures;

				mrpt::math::CVectorDouble	lessEigenVals;
				mrpt::math::CVectorDouble	errorEstimations;
				mrpt::math::CVectorDouble	meanRegions;

				mrpt::math::CVectorDouble	sumDistances;

				int				faceNum;
				vector_uint		deletedRegions;
				int				numPossibleFacesDetected;
				int				numRealFacesDetected;

				bool			takeTime;

				bool			saveMeasurementsToFile;

			}m_measure;

			// To take measures abaout execution time
			mrpt::utils::CTimeLogger	m_timeLog;

			std::vector<double> m_meanHist;


			// Test to check if a candidate region is a real face

			bool checkIfFacePlane( mrpt::obs::CObservation3DRangeScan* face );

			bool checkIfFacePlaneCov( mrpt::obs::CObservation3DRangeScan* face );

			void thread_checkIfFacePlaneCov( );

			static void dummy_checkIfFacePlaneCov( CFaceDetection *obj );


			bool checkIfFaceRegions( mrpt::obs::CObservation3DRangeScan* face );

			void thread_checkIfFaceRegions( );

			static void dummy_checkIfFaceRegions( CFaceDetection *obj );

			size_t checkRelativePosition( const mrpt::math::TPoint3D &p1, const mrpt::math::TPoint3D &p2, const mrpt::math::TPoint3D &p, double &dist );


			void thread_checkIfDiagonalSurface( );

			bool checkIfDiagonalSurface( mrpt::obs::CObservation3DRangeScan* face );

			bool checkIfDiagonalSurface2( mrpt::obs::CObservation3DRangeScan* face );

			static void dummy_checkIfDiagonalSurface( CFaceDetection *obj );

			// Experimental methods to view 3D points

			void experimental_viewFacePointsScanned( const std::vector<float> &xs, const std::vector<float> &ys, const std::vector<float> &zs );

			void experimental_viewFacePointsScanned( const mrpt::obs::CObservation3DRangeScan &face );
			
			void experimental_viewFacePointsScanned( const std::vector<mrpt::math::TPoint3D> &points );

			void experimental_viewFacePointsAndEigenVects(  const std::vector<mrpt::math::CArrayDouble<3> > &pointsVector, const mrpt::math::CMatrixDouble &eigenVect, const mrpt::math::CVectorDouble &eigenVal );

			void experimental_viewRegions( const std::vector<mrpt::math::TPoint3D> regions[9], const mrpt::math::TPoint3D meanPos[3][3] );		

			// Segmentation methods
			void experimental_segmentFace( const mrpt::obs::CObservation3DRangeScan &face, mrpt::math::CMatrixTemplate<bool> &region );

			// Histogram methods
			void experimental_calcHist( const mrpt::utils::CImage &face, const size_t &c1, const size_t &r1, const size_t &c2, 
										const size_t &r2, mrpt::math::CMatrixTemplate<unsigned int> &hist );

			

		}; // End of class
	}

}

#endif
