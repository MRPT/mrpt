/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/obs/CObservation.h>
#include <map>
#include <mutex>

namespace mrpt
{
/**   Contains classes for various device interfaces.
 * \ingroup mrpt_hwdrivers_grp
 */
namespace hwdrivers
{
class CGenericSensor;

/** A structure for runtime ID class type information in the context of
 * hwdrivers::CGenericSensor.
 */
struct TSensorClassId
{
	/** Class name */
	const char* className;
	/** Pointer to class constructor */
	CGenericSensor* (*ptrCreateObject)();
};

/** A generic interface for a wide-variety of sensors designed to be used in the
 *application RawLogGrabber.
 *  Derived classes should be designed with the following execution flow in
 *mind:
 *		- Object constructor
 *		- CGenericSensor::loadConfig: The following parameters are common to all
 *sensors in rawlog-grabber (they are automatically loaded by rawlog-grabber) -
 *see each class documentation for additional parameters:
 *			- "process_rate": (Mandatory) The rate in Hertz (Hz) at which the
 *sensor
 *thread should invoke "doProcess".
 *			- "max_queue_len": (Optional) The maximum number of objects in the
 *observations queue (default is 200). If overflow occurs, an error message
 *will be issued at run-time.
 *			- "grab_decimation": (Optional) Grab only 1 out of N observations
 *captured
 *by the sensor (default is 1, i.e. do not decimate).
 *		- CGenericSensor::initialize
 *		- CGenericSensor::doProcess
 *		- CGenericSensor::getObservations
 *
 *  Notice that there are helper methods for managing the internal list of
 *objects (see CGenericSensor::appendObservation).
 *
 *  <b>Class Factory:</b> This is also a factory of derived classes, through
 *the static method CGenericSensor::createSensor
 *
 *
 *  For more details on RawLogGrabber refer to the wiki page:
 *    http://www.mrpt.org/Application:RawLogGrabber
 * \ingroup mrpt_hwdrivers_grp
 */
class CGenericSensor
{
   public:
	using Ptr = std::shared_ptr<CGenericSensor>;
	virtual const mrpt::hwdrivers::TSensorClassId* GetRuntimeClass() const = 0;

	using TListObservations = std::multimap<
		mrpt::system::TTimeStamp, mrpt::serialization::CSerializable::Ptr>;
	using TListObsPair = std::pair<
		mrpt::system::TTimeStamp, mrpt::serialization::CSerializable::Ptr>;

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
	inline void setSensorLabel(const std::string& sensorLabel)
	{
		m_sensorLabel = sensorLabel;
	}

	/** Enable or disable extra debug info dumped to std::cout during sensor
	 * operation.
	 * Default: disabled unless the environment variable
	 * "MRPT_HWDRIVERS_VERBOSE" is set to "1" during object creation.
	 */
	inline void enableVerbose(bool enabled = true) { m_verbose = enabled; }
	inline bool isVerboseEnabled() const { return m_verbose; }
	/** Register a class into the internal list of "CGenericSensor" descendents.
	 *  Used internally in the macros DEFINE_GENERIC_SENSOR, etc...
	 *
	 *  Can be used as "CGenericSensor::registerClass(
	 * SENSOR_CLASS_ID(CMySensor) );" if
	 *    building custom sensors outside mrpt libraries in user code.
	 */
	static void registerClass(const TSensorClassId* pNewClass);

   private:
	/** The critical section for m_objList */
	std::mutex m_csObjList;
	/** The queue of objects to be returned by getObservations */
	TListObservations m_objList;

	/** Used in registerClass */
	using registered_sensor_classes_t =
		std::map<std::string, const TSensorClassId*>;
	/** Access to singleton */
	static registered_sensor_classes_t& get_registered_sensor_classes();

   protected:
	/** @name Common settings to any sensor, loaded in "loadConfig"
		@{ */

	/** See CGenericSensor */
	double m_process_rate{0};
	/** See CGenericSensor */
	size_t m_max_queue_len{200};
	/** If set to N>=2, only 1 out of N observations will be saved to m_objList.
	 */
	size_t m_grab_decimation{0};
	/** See CGenericSensor */
	std::string m_sensorLabel;

	/** @} */

	/** Used when "m_grab_decimation" is enabled */
	size_t m_grab_decimation_counter{0};

	TSensorState m_state{ssInitializing};
	bool m_verbose{false};

	// === Data for off-rawlog file external image directory ====
	//  Only used by a few sensor classes.
	/** The path where to save off-rawlog images: empty means save images
	 * embedded in the rawlog. */
	std::string m_path_for_external_images;
	/** The extension ("jpg","gif","png",...) that determines the format of
	 * images saved externally \sa setPathForExternalImages */
	std::string m_external_images_format;
	/** For JPEG images, the quality (default=95%). */
	unsigned int m_external_images_jpeg_quality{95};
	// ======================================

	/** This method must be called by derived classes to enqueue a new
	  observation in the list to be returned by getObservations.
	  *  Passed objects must be created in dynamic memory and a smart pointer
	  passed. Example of creation:
	  \code
		mrpt::obs::CObservationGPS::Ptr  o = CObservationGPS::Ptr( new
	  CObservationGPS() );
		o-> .... // Set data
		appendObservation(o);
	  \endcode
	  * If several observations are passed at once in the vector, they'll be
	  considered as a block regarding the grabbing decimation factor.
	  */
	void appendObservations(
		const std::vector<mrpt::serialization::CSerializable::Ptr>& obj);

	//! Like appendObservations() but for just one observation.
	void appendObservation(const mrpt::serialization::CSerializable::Ptr& obj)
	{
		appendObservations(
			std::vector<mrpt::serialization::CSerializable::Ptr>(1, obj));
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

	/** Loads specific configuration for the device from a given source of
	 * configuration parameters, for example, an ".ini" file, loading from the
	 * section "[iniSection]" (see config::CConfigFileBase and derived classes)
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical parameter is missing or has an invalid value.
	 */
	virtual void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& section) = 0;

   public:
	/** Creates a sensor by a name of the class.
	 *  Typically the user may want to create a smart pointer around the
	 * returned pointer, whis is made with:
	 *  \code
	 *   CGenericSensor::Ptr sensor = CGenericSensor::Ptr(
	 * CGenericSensor::createSensor("XXX") );
	 *  \endcode
	 * \return A pointer to a new class, or nullptr if class name is unknown.
	 */
	static CGenericSensor* createSensor(const std::string& className);

	/** Just like createSensor, but returning a smart pointer to the newly
	 * created sensor object. */
	static inline CGenericSensor::Ptr createSensorPtr(
		const std::string& className)
	{
		return CGenericSensor::Ptr(createSensor(className));
	}

	/** Constructor */
	CGenericSensor();

	CGenericSensor(const CGenericSensor&) = delete;
	CGenericSensor& operator=(const CGenericSensor&) = delete;

	/** Destructor */
	virtual ~CGenericSensor();

	/** Loads the generic settings common to any sensor (See CGenericSensor),
	 * then call to "loadConfig_sensorSpecific"
	 *  \exception This method throws an exception with a descriptive message
	 * if some critical parameter is missing or has an invalid value.
	 */
	void loadConfig(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& section);

	/** This method can or cannot be implemented in the derived class, depending
	 * on the need for it.
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	 */
	virtual void initialize() {}  // Default method does nothing.
	/** This method will be invoked at a minimum rate of "process_rate" (Hz)
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	 */
	virtual void doProcess() = 0;

	/** Returns a list of enqueued objects, emptying it (thread-safe). The
	 * objects must be freed by the invoker.
	 */
	void getObservations(TListObservations& lstObjects);

	/**  Set the path where to save off-rawlog image files (will be ignored in
	 * those sensors where this is not applicable).
	 *  An  empty string (the default value at construction) means to save
	 * images embedded in the rawlog, instead of on separate files.
	 * \exception std::exception If the directory doesn't exists and cannot be
	 * created.
	 */
	virtual void setPathForExternalImages(const std::string& directory)
	{
		MRPT_UNUSED_PARAM(directory);
		// In this base class, the default is to ignore image paths.
	}

	/**  Set the extension ("jpg","gif","png",...) that determines the format of
	 * images saved externally
	 *   The default is "jpg".
	 * \sa setPathForExternalImages, setExternalImageJPEGQuality
	 */
	void setExternalImageFormat(const std::string& ext)
	{
		m_external_images_format = ext;
	}

	/** The quality of JPEG compression, when external images is enabled and the
	 * format is "jpg". \sa setExternalImageFormat */
	void setExternalImageJPEGQuality(const unsigned int quality)
	{
		m_external_images_jpeg_quality = quality;
	}
	unsigned int getExternalImageJPEGQuality() const
	{
		return m_external_images_jpeg_quality;
	}

   public:
	MRPT_MAKE_ALIGNED_OPERATOR_NEW

};  // end of class

static_assert(
	!std::is_copy_constructible_v<CGenericSensor> &&
		!std::is_copy_assignable_v<CGenericSensor>,
	"Copy Check");

#define SENSOR_CLASS_ID(class_name)                      \
	static_cast<const mrpt::hwdrivers::TSensorClassId*>( \
		&mrpt::hwdrivers::class_name::class##class_name)

#define SENSOR_IS_CLASS(ptrObj, class_name) \
	(ptrObj->GetRuntimeClass() == SENSOR_CLASS_ID(class_name))

/** This declaration must be inserted in all CGenericSensor classes definition,
 * within the class declaration.
 */
#define DEFINE_GENERIC_SENSOR(class_name)                                    \
   protected:                                                                \
	static mrpt::hwdrivers::CGenericSensor::CLASSINIT_GENERIC_SENSOR         \
		_init_##class_name;                                                  \
                                                                             \
   public:                                                                   \
	static mrpt::hwdrivers::TSensorClassId class##class_name;                \
	const mrpt::hwdrivers::TSensorClassId* GetRuntimeClass() const override; \
	static mrpt::hwdrivers::CGenericSensor* CreateObject();                  \
	static void doRegister()                                                 \
	{                                                                        \
		CGenericSensor::registerClass(SENSOR_CLASS_ID(class_name));          \
	}

/** This must be inserted in all CGenericSensor classes implementation files:
 */
#define IMPLEMENTS_GENERIC_SENSOR(class_name, NameSpace)                       \
	mrpt::hwdrivers::CGenericSensor* NameSpace::class_name::CreateObject()     \
	{                                                                          \
		return static_cast<hwdrivers::CGenericSensor*>(                        \
			new NameSpace::class_name);                                        \
	}                                                                          \
	mrpt::hwdrivers::TSensorClassId NameSpace::class_name::class##class_name = \
		{#class_name, NameSpace::class_name::CreateObject};                    \
	const mrpt::hwdrivers::TSensorClassId*                                     \
		NameSpace::class_name::GetRuntimeClass() const                         \
	{                                                                          \
		return SENSOR_CLASS_ID(class_name);                                    \
	}

}  // namespace hwdrivers
}  // namespace mrpt
