/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGenericSensor_H
#define CGenericSensor_H

#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CUncopiable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>
#include <map>

#include <mrpt/hwdrivers/link_pragmas.h>
#include <map>


namespace mrpt
{
	/**   Contains classes for various device interfaces.
	 * \ingroup mrpt_hwdrivers_grp
	 */
	namespace hwdrivers
	{
		class HWDRIVERS_IMPEXP CGenericSensor;

		/** A structure for runtime ID class type information in the context of hwdrivers::CGenericSensor.
		  */
		struct HWDRIVERS_IMPEXP TSensorClassId
		{
			const char*			className;						//!< Class name
			CGenericSensor*		(*ptrCreateObject)();			//!< Pointer to class constructor
		};

		typedef stlplus::smart_ptr<CGenericSensor>	CGenericSensorPtr;

		/** A generic interface for a wide-variety of sensors designed to be used in the application RawLogGrabber.
		  *  Derived classes should be designed with the following execution flow in mind:
		  *		- Object constructor
		  *		- CGenericSensor::loadConfig: The following parameters are common to all sensors in rawlog-grabber (they are automatically loaded by rawlog-grabber) - see each class documentation for additional parameters:
		  *			- "process_rate": (Mandatory) The rate in Hertz (Hz) at which the sensor thread should invoke "doProcess".
		  *			- "max_queue_len": (Optional) The maximum number of objects in the observations queue (default is 200). If overflow occurs, an error message will be issued at run-time.
		  *			- "grab_decimation": (Optional) Grab only 1 out of N observations captured by the sensor (default is 1, i.e. do not decimate).
		  *		- CGenericSensor::initialize
		  *		- CGenericSensor::doProcess
		  *		- CGenericSensor::getObservations
		  *
		  *  Notice that there are helper methods for managing the internal list of objects (see CGenericSensor::appendObservation).
		  *
		  *  <b>Class Factory:</b> This is also a factory of derived classes, through the static method CGenericSensor::createSensor
		  *
		  *
		  *  For more details on RawLogGrabber refer to the wiki page:
		  *    http://www.mrpt.org/Application:RawLogGrabber
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CGenericSensor: public mrpt::utils::CUncopiable
		{
		public:
			virtual const mrpt::hwdrivers::TSensorClassId* GetRuntimeClass() const = 0;

			typedef std::multimap< mrpt::system::TTimeStamp, mrpt::utils::CSerializablePtr > TListObservations;
			typedef std::pair< mrpt::system::TTimeStamp, mrpt::utils::CSerializablePtr > TListObsPair;

			/** The current state of the sensor
			  * \sa CGenericSensor::getState
			  */
			enum TSensorState
			{
				ssInitializing = 0,
				ssWorking,
				ssError
			};

			/** The current state of the sensor  */
			inline TSensorState getState() const { return m_state; }

			inline double getProcessRate() const { return m_process_rate; }

			inline std::string getSensorLabel() const { return m_sensorLabel; }
			inline void setSensorLabel(const std::string& sensorLabel) { m_sensorLabel=sensorLabel; }

			/** Enable or disable extra debug info dumped to std::cout during sensor operation.
			  * Default: disabled unless the environment variable "MRPT_HWDRIVERS_VERBOSE" is set to "1" during object creation.
			  */
			inline void enableVerbose(bool enabled=true) { m_verbose=enabled; }
			inline bool isVerboseEnabled() const { return m_verbose; }

			/** Register a class into the internal list of "CGenericSensor" descendents.
			  *  Used internally in the macros DEFINE_GENERIC_SENSOR, etc...
			  *
			  *  Can be used as "CGenericSensor::registerClass( SENSOR_CLASS_ID(CMySensor) );" if
			  *    building custom sensors outside mrpt libraries in user code.
			  */
			static void registerClass(const TSensorClassId* pNewClass);

		private:
			synch::CCriticalSection			m_csObjList;		//!< The critical section for m_objList
			TListObservations				m_objList;		//!< The queue of objects to be returned by getObservations

			/** Used in registerClass */
			typedef std::map< std::string , const TSensorClassId *> registered_sensor_classes_t;
			static registered_sensor_classes_t & get_registered_sensor_classes(); //!< Access to singleton

		protected:
			/** @name Common settings to any sensor, loaded in "loadConfig"
			    @{ */

			double	m_process_rate;  //!< See CGenericSensor
			size_t	m_max_queue_len; //!< See CGenericSensor
			size_t	m_grab_decimation;	//!< If set to N>=2, only 1 out of N observations will be saved to m_objList.
			std::string  m_sensorLabel; //!< See CGenericSensor

			/** @} */

			size_t	m_grab_decimation_counter; //!< Used when "m_grab_decimation" is enabled

			TSensorState    m_state;
			bool            m_verbose;

			// === Data for off-rawlog file external image directory ====
			//  Only used by a few sensor classes.
			std::string			m_path_for_external_images; //!< The path where to save off-rawlog images: empty means save images embedded in the rawlog.
			std::string			m_external_images_format; //!< The extension ("jpg","gif","png",...) that determines the format of images saved externally \sa setPathForExternalImages
			unsigned int		m_external_images_jpeg_quality; //!< For JPEG images, the quality (default=95%).
			// ======================================

			/** This method must be called by derived classes to enqueue a new observation in the list to be returned by getObservations.
			  *  Passed objects must be created in dynamic memory and a smart pointer passed. Example of creation:
			  \code
				mrpt::obs::CObservationGPSPtr  o = CObservationGPSPtr( new CObservationGPS() );
				o-> .... // Set data
				appendObservation(o);
			  \endcode
			  * If several observations are passed at once in the vector, they'll be considered as a block regarding the grabbing decimation factor.
			  */
			void appendObservations( const std::vector<mrpt::utils::CSerializablePtr> &obj);

			//! Like appendObservations() but for just one observation.
			void appendObservation( const mrpt::utils::CSerializablePtr &obj)
			{
				appendObservations(std::vector<mrpt::utils::CSerializablePtr>(1, obj));
			}

			/** Auxiliary structure used for CSerializable runtime class ID support.
			  */
			struct CLASSINIT_GENERIC_SENSOR
			{
				CLASSINIT_GENERIC_SENSOR(const TSensorClassId* pNewClass)
				{
					CGenericSensor::registerClass(pNewClass);
				}
			};

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  \exception This method must throw an exception with a descriptive message if some critical parameter is missing or has an invalid value.
			  */
			virtual void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section ) = 0;

		public:
			/** Creates a sensor by a name of the class.
			  *  Typically the user may want to create a smart pointer around the returned pointer, whis is made with:
			  *  \code
			  *   CGenericSensorPtr sensor = CGenericSensorPtr( CGenericSensor::createSensor("XXX") );
			  *  \endcode
			  * \return A pointer to a new class, or NULL if class name is unknown.
			  */
			static CGenericSensor* createSensor(const std::string &className);

			/** Just like createSensor, but returning a smart pointer to the newly created sensor object. */
			static inline CGenericSensorPtr createSensorPtr(const std::string &className)
			{
				return CGenericSensorPtr(createSensor(className));
			}

			/** Constructor */
			CGenericSensor( );

			/** Destructor */
			virtual ~CGenericSensor();

			/** Loads the generic settings common to any sensor (See CGenericSensor), then call to "loadConfig_sensorSpecific"
			  *  \exception This method throws an exception with a descriptive message if some critical parameter is missing or has an invalid value.
			  */
			void  loadConfig(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

			/** This method can or cannot be implemented in the derived class, depending on the need for it.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize()
			{  }	// Default method does nothing.

			/** This method will be invoked at a minimum rate of "process_rate" (Hz)
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void doProcess() = 0;

			/** Returns a list of enqueued objects, emptying it (thread-safe). The objects must be freed by the invoker.
			  */
			void getObservations( TListObservations		&lstObjects );

			/**  Set the path where to save off-rawlog image files (will be ignored in those sensors where this is not applicable).
			  *  An  empty string (the default value at construction) means to save images embedded in the rawlog, instead of on separate files.
			  * \exception std::exception If the directory doesn't exists and cannot be created.
			  */
			virtual void setPathForExternalImages( const std::string &directory ) {
				MRPT_UNUSED_PARAM(directory);
				// In this base class, the default is to ignore image paths.
			}

			/**  Set the extension ("jpg","gif","png",...) that determines the format of images saved externally
			  *   The default is "jpg".
			  * \sa setPathForExternalImages, setExternalImageJPEGQuality
			  */
			void setExternalImageFormat( const std::string &ext ) {
				m_external_images_format = ext;
			}

			/** The quality of JPEG compression, when external images is enabled and the format is "jpg". \sa setExternalImageFormat */
			void setExternalImageJPEGQuality(const unsigned int quality) {
				m_external_images_jpeg_quality = quality;
			}
			unsigned int getExternalImageJPEGQuality()const  {
				return m_external_images_jpeg_quality;
			}

		public:
			MRPT_MAKE_ALIGNED_OPERATOR_NEW

		}; // end of class


		#define SENSOR_CLASS_ID(class_name) \
			static_cast<const mrpt::hwdrivers::TSensorClassId*>(& mrpt::hwdrivers::class_name::class##class_name)

		#define SENSOR_IS_CLASS( ptrObj, class_name )  (ptrObj->GetRuntimeClass()==SENSOR_CLASS_ID(class_name))


		/** This declaration must be inserted in all CGenericSensor classes definition, within the class declaration.
		  */
		#define DEFINE_GENERIC_SENSOR(class_name) \
		protected: \
			static mrpt::hwdrivers::CGenericSensor::CLASSINIT_GENERIC_SENSOR _init_##class_name;\
		public: \
			static  mrpt::hwdrivers::TSensorClassId class##class_name; \
			virtual const mrpt::hwdrivers::TSensorClassId* GetRuntimeClass() const; \
			static  mrpt::hwdrivers::CGenericSensor* CreateObject(); \
			static void doRegister() \
			{ 	CGenericSensor::registerClass( SENSOR_CLASS_ID( class_name ) ); }

		/** This must be inserted in all CGenericSensor classes implementation files:
		  */
		#define IMPLEMENTS_GENERIC_SENSOR(class_name, NameSpace) \
			mrpt::hwdrivers::CGenericSensor* NameSpace::class_name::CreateObject() \
				{ return static_cast<hwdrivers::CGenericSensor*>( new NameSpace::class_name ); } \
			mrpt::hwdrivers::TSensorClassId NameSpace::class_name::class##class_name = { \
				#class_name, NameSpace::class_name::CreateObject }; \
			const mrpt::hwdrivers::TSensorClassId* NameSpace::class_name::GetRuntimeClass() const \
				{ return SENSOR_CLASS_ID(class_name); }


	} // end of namespace
} // end of namespace

#endif
