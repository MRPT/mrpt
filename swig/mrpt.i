/* File mrpt.i */
%module mrpt
%include "std_string.i"
%include "std_deque.i"
%include "exception.i"
%{
# define SWIG_FILE_WITH_INIT
#include "mrpt/slam.h"
#include "mrpt/utils.h"
#include "mrpt/poses.h"
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::utils;
%}

class CPose2D : public CPose{
	 public:
		 CPose2D(const double& x=0,const double&y=0,const double& phi=0);

		 double phi() const;

		 void phi_incr(const double Aphi);
         
         /*
         %exception operator[](unsigned int)
         {
             try {
                 $action
             }
             catch (std::runtime_error)
             {
                 SWIG_exception(SWIG_IndexError, "Index out of range");
             }
         }
         */
         %extend{
             double __getitem__(unsigned int i) throw (std::runtime_error)
		 {
  		 	return $self->operator[](i);
		 }
         }
};

class CPose3D : public CPose
{
	 public:
		 CPose3D(const double& x=0,const double& y=0,const double& z=0,const double& yaw=0, const double& pitch=0, const double& roll=0);
		 CPose3D( const CPose3D &o);
 		 CPose3D(const CPose2D &);
};

class CObservationRange : public CObservation
{
	 public:
		%extend {
		   CObservationRange(float minSensorDist, float	maxSensorDist, float	sensorConeAperture) {
		       CObservationRange *ret = new CObservationRange();
		       ret->minSensorDistance = minSensorDist;
		       ret->maxSensorDistance = maxSensorDist;
		       ret->sensorConeApperture = sensorConeAperture;
		       return ret;
		   }
		   
		   void insert(int sensorID, CPose3D pose, float sensedDistance) {
		       CObservationRange::TMeasurement *msr = new CObservationRange::TMeasurement();
		       msr->sensorID = sensorID;
		       msr->sensorPose = pose;
		       msr->sensedDistance = sensedDistance;
		       $self->sensedData.push_back(*msr);
		   }
		}
		float	minSensorDistance;
		float	maxSensorDistance;
		float	sensorConeApperture;  //!< Cone aperture of each ultrasonic beam, in radians.
		
		
};

class CSensoryFrame : public mrpt::utils::CSerializable
{
    public:
			 CSensoryFrame();
			 CSensoryFrame( const CSensoryFrame &);
 			 virtual ~CSensoryFrame();
 			 
			 %extend {
			     void insert(CObservationRange &obs)
			     {
			         CObservationPtr ptr = CObservationPtr(new CObservationRange(obs));
			         $self->operator+=(ptr);
			     }
			 }
};

class CAction : public mrpt::utils::CSerializable
	    {
	    
    	    CAction();
	    	virtual ~CAction();
	    };

class CActionCollection : public mrpt::utils::CSerializable
{
public:
    CActionCollection();
    
    CActionCollection(const CActionCollection &o );
			virtual ~CActionCollection();
  			void  insert(CAction	&action);

};

class CActionRobotMovement2D : public CAction
		{
        public:
	
			CPose2D					rawOdometryIncrementReading;
	
        	CActionRobotMovement2D();
			CActionRobotMovement2D(const CActionRobotMovement2D &o);
			~CActionRobotMovement2D();
        
        
	        %extend{
	            void computeFromOdometry(const CPose2D	&odometryIncrement)
	            {
				    $self->computeFromOdometry(odometryIncrement, $self->motionModelConfiguration);
				}
		    }
	    };
%{
// SWIG thinks that TMotionModelOptions is a global class, so we need to trick the C++
// compiler into understanding this so called global type.
typedef CActionRobotMovement2D::TMotionModelOptions TMotionModelOptions;
%}

class COccupancyGridMap2D : public CMetricMap
{
public:
		COccupancyGridMap2D( float min_x = -20.0f,
							 float max_x = 20.0f,
							 float min_y = -20.0f,
							 float max_y = 20.0f,
							 float resolution = 0.05f
							 );
		virtual ~COccupancyGridMap2D();

		void  clear( );
		void  fill(float default_value = 0.5f );
		inline void   setPos(float x,float y,float value);
		inline float  getPos(float x,float y) const;
		
        bool  insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );
		
		void  sonarSimulator(
				CObservationRange	        &inout_observation,
				const CPose2D				&robotPose,
				float						threshold = 0.5f,
				float						rangeNoiseStd = 0,
				float						angleNoiseStd = DEG2RAD(0) ) const;
				
		bool  saveAsBitmapFile(const std::string &file) const;
		bool  saveAsBitmapFileWithLandmarks(
			const std::string		&file,
			const CLandmarksMap		*landmarks,
			bool  addTextLabels = false ) const;
			
		bool  loadFromBitmapFile(const std::string	&file, float resolution, float xCentralPixel = -1, float yCentralPixel =-1 );

};

	    class CRawlog : public mrpt::utils::CSerializable
		{
		public:
			std::string getCommentText() const;			//!< Returns the block of comment text for the rawlog
			void setCommentText( const std::string &t);	//!< Changes the block of comment text for the rawlog
			enum TEntryType
			{
				etSensoryFrame = 0,
				etActionCollection,
				etObservation
			};
         	CRawlog();
			virtual ~CRawlog();
			void  clear();
			void  addAction( CAction &action );
			void  addActions( CActionCollection &action );
			void  addObservations( CSensoryFrame &observations );
			bool saveToRawLogFile( const std::string &fileName ) const;
        };


struct TMetricMapInitializer
	{
public:
		TMetricMapInitializer();
		bool				m_disableSaveAs3DObject;
		%extend {
		    void set_COccupancyGridMap2D(float	min_x, float max_x, float min_y, float max_y, float resolution) {	//!< See COccupancyGridMap2D::COccupancyGridMap2D
		        $self->metricMapClassType = CLASS_ID(COccupancyGridMap2D);
		        $self->occupancyGridMap2D_options.min_x = min_x;
		        $self->occupancyGridMap2D_options.max_x = max_x;
		        $self->occupancyGridMap2D_options.min_y = min_y;
		        $self->occupancyGridMap2D_options.max_y = max_y;
		        $self->occupancyGridMap2D_options.resolution = resolution;
		    }
		}
};

class TSetOfMetricMapInitializers : public utils::CLoadableOptions
{
public:
    TSetOfMetricMapInitializers() : m_list(), options();
	void push_back( const TMetricMapInitializer &o );
};

%inline {
    COccupancyGridMap2D *ptr2map(COccupancyGridMap2DPtr &p)
    {
        return &(*p);
    }
}

%template(GridMapDeque) std::deque<COccupancyGridMap2DPtr>;

class CMultiMetricMap : public CMetricMap
	{
	public:
	%extend {
	    CMultiMetricMap() {
	        return new CMultiMetricMap();
	    }
    }
	CMultiMetricMap(const mrpt::slam::CMultiMetricMap &other );
		virtual ~CMultiMetricMap( );
		std::deque<COccupancyGridMap2DPtr>	m_gridMaps;

};

	class CMetricMapBuilderICP : public CMetricMapBuilder
	{
	 public:
		 /** Constructor.
		   */

		 CMetricMapBuilderICP(
			TSetOfMetricMapInitializers	*mapInitializers,
			float						insertionLinDistance = 1.0f,
			float						insertionAngDistance = DEG2RAD(30),
			CICP::TConfigParams			*icpParams = NULL );

		 /** Destructor:
		   */
		 virtual ~CMetricMapBuilderICP();
		 
		CPose3DPDFPtr  getCurrentPoseEstimation() const;
        void  processActionObservation(
					CActionCollection	&action,
					CSensoryFrame		&in_SF );
CMultiMetricMap*   getCurrentlyBuiltMetricMap();
		void  saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP = true);
};

class CMetricMapBuilderRBPF : public CMetricMapBuilder
{
public:
	%extend {
	    CMetricMapBuilderRBPF(float insertionLinDistance, float	insertionAngDistance, TSetOfMetricMapInitializers mapsInitializers)
	    { // The map is based on 1 GridMap2D because I'm too lazy to un-nest nested classes
	        CMetricMapBuilderRBPF::TConstructionOptions init;
	        init.insertionLinDistance = insertionLinDistance;
	        init.insertionAngDistance = insertionAngDistance;
	        init.mapsInitializers = mapsInitializers;
	        return new CMetricMapBuilderRBPF(init);
	    }
	}
		void  initialize(
				CSensFrameProbSequence		&initialMap,
				CPosePDF					*x0 = NULL
				);
						void  clear();
						
		CPose3DPDFPtr  getCurrentPoseEstimation() const;
		
		void  processActionObservation(
					CActionCollection	&action,
					CSensoryFrame		&observations );		
					
		CMultiMetricMap*   getCurrentlyBuiltMetricMap();
		unsigned int  getCurrentlyBuiltMapSize();
		
		void  saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP = true);
};

