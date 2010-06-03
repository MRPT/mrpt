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

#ifndef CGenericSensor_H
#define CGenericSensor_H

#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/synch.h>
#include <mrpt/system/threads.h>

#include <mrpt/hwdrivers/link_pragmas.h>


namespace mrpt
{
	/**   Contains classes for various device interfaces.
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
			TSensorState getState() const { return m_state; }

			double getProcessRate() const { return m_process_rate; }

		private:
			synch::CCriticalSection			m_csObjList;		//!< The critical section for m_objList
			TListObservations				m_objList;		//!< The queue of objects to be returned by getObservations

			/** Used in registerClass */
			static std::map< std::string , const TSensorClassId *>	m_knownClasses;

			// DECLARE_UNCOPIABLE( CGenericSensor )


		protected:
			// === Common settings to any sensor, loaded in "loadConfig" ====
			double	m_process_rate;  //!< See CGenericSensor
			size_t	m_max_queue_len; //!< See CGenericSensor
			size_t	m_grab_decimation;	//!< If set to N>=2, only 1 out of N observations will be saved to m_objList.
			// ======================================

			size_t	m_grab_decimation_counter; //!< Used when "m_grab_decimation" is enabled


			TSensorState    m_state;

			/** This method must be called by derived classes to enqueue a new observation in the list to be returned by getObservations.
			  *  Passed objects must be created in dynamic memory and a smart pointer passed. Example of creation:
			  \code
				CObservationGPSPtr  o = CObservationGPSPtr( new CObservationGPS() );
				o-> .... // Set data
				appendObservation(o);
			  \endcode
			  */
			void appendObservation( const mrpt::utils::CSerializablePtr &obj);

			/** Register a class into the internal list of "CGenericSensor" descendents.
			  *  Used internally in the macros DEFINE_GENERIC_SENSOR, etc...
			  */
			static void registerClass(const TSensorClassId* pNewClass);


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
