#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/VelodyneCalibration.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::obs::CObservationVelodyneScan file:mrpt/obs/CObservationVelodyneScan.h line:82
struct PyCallBack_mrpt_obs_CObservationVelodyneScan : public mrpt::obs::CObservationVelodyneScan {
	using mrpt::obs::CObservationVelodyneScan::CObservationVelodyneScan;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationVelodyneScan::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationVelodyneScan::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationVelodyneScan::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationVelodyneScan::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationVelodyneScan::serializeFrom(a0, a1);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservationVelodyneScan::getOriginalReceivedTimeStamp();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationVelodyneScan::unload();
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationVelodyneScan::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationVelodyneScan::setSensorPose(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::asString();
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservation::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtDataRow();
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load_impl();
	}
};

// mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper file:mrpt/obs/CObservationVelodyneScan.h line:298
struct PyCallBack_mrpt_obs_CObservationVelodyneScan_PointCloudStorageWrapper : public mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper {
	using mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::PointCloudStorageWrapper;

	void reserve(std::size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper *>(this), "reserve");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return PointCloudStorageWrapper::reserve(a0);
	}
	void resizeLaserCount(std::size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper *>(this), "resizeLaserCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return PointCloudStorageWrapper::resizeLaserCount(a0);
	}
	void add_point(float a0, float a1, float a2, uint8_t a3, const mrpt::Clock::time_point & a4, const float a5, uint16_t a6) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper *>(this), "add_point");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PointCloudStorageWrapper::add_point\"");
	}
};

void bind_mrpt_obs_VelodyneCalibration(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::VelodyneCalibration file:mrpt/obs/VelodyneCalibration.h line:28
		pybind11::class_<mrpt::obs::VelodyneCalibration, std::shared_ptr<mrpt::obs::VelodyneCalibration>> cl(M("mrpt::obs"), "VelodyneCalibration", "Velodyne calibration data, for usage in mrpt::obs::CObservationVelodyneScan\n\n It is mandatory to use some calibration data to convert Velodyne scans into\n 3D point clouds. Users should\n normally use the XML files provided by the manufacturer with each scanner,\n but default calibration files can be\n loaded with \n\n \n New in MRPT 1.4.0\n \n\n CObservationVelodyneScan, CVelodyneScanner\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::VelodyneCalibration(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::VelodyneCalibration const &o){ return new mrpt::obs::VelodyneCalibration(o); } ) );
		cl.def_readwrite("laser_corrections", &mrpt::obs::VelodyneCalibration::laser_corrections);
		cl.def("empty", (bool (mrpt::obs::VelodyneCalibration::*)() const) &mrpt::obs::VelodyneCalibration::empty, "Returns true if no calibration has been loaded yet \n\nC++: mrpt::obs::VelodyneCalibration::empty() const --> bool");
		cl.def("clear", (void (mrpt::obs::VelodyneCalibration::*)()) &mrpt::obs::VelodyneCalibration::clear, "Clear all previous contents \n\nC++: mrpt::obs::VelodyneCalibration::clear() --> void");
		cl.def_static("LoadDefaultCalibration", (const struct mrpt::obs::VelodyneCalibration & (*)(const std::string &)) &mrpt::obs::VelodyneCalibration::LoadDefaultCalibration, "Loads default calibration files for common LIDAR models.\n \n\n Valid model names are: `VLP16`, `HDL32`, `HDL64`\n \n\n It always return a calibration structure, but it may be empty if\n the model name is unknown. See \n \n\n Default files can be inspected in `[MRPT_SRC or\n /usr]/share/mrpt/config_files/rawlog-grabber/velodyne_default_calib_{*}.xml`\n\nC++: mrpt::obs::VelodyneCalibration::LoadDefaultCalibration(const std::string &) --> const struct mrpt::obs::VelodyneCalibration &", pybind11::return_value_policy::automatic, pybind11::arg("lidar_model"));
		cl.def("loadFromXMLFile", (bool (mrpt::obs::VelodyneCalibration::*)(const std::string &)) &mrpt::obs::VelodyneCalibration::loadFromXMLFile, "Loads calibration from file, in the format supplied by the manufacturer.\n \n\n false on any error, true on success \n\nC++: mrpt::obs::VelodyneCalibration::loadFromXMLFile(const std::string &) --> bool", pybind11::arg("velodyne_calibration_xml_filename"));
		cl.def("loadFromXMLText", (bool (mrpt::obs::VelodyneCalibration::*)(const std::string &)) &mrpt::obs::VelodyneCalibration::loadFromXMLText, "Loads calibration from a string containing an entire XML calibration\n file. \n\n loadFromXMLFile \n false on any error, true on success \n\nC++: mrpt::obs::VelodyneCalibration::loadFromXMLText(const std::string &) --> bool", pybind11::arg("xml_file_contents"));
		cl.def("loadFromYAMLText", (bool (mrpt::obs::VelodyneCalibration::*)(const std::string &)) &mrpt::obs::VelodyneCalibration::loadFromYAMLText, "Loads calibration from a string containing an entire YAML calibration\n file. \n\n loadFromYAMLFile, loadFromXMLFile \n false on any error,\n true on success \n\nC++: mrpt::obs::VelodyneCalibration::loadFromYAMLText(const std::string &) --> bool", pybind11::arg("yaml_file_contents"));
		cl.def("loadFromYAMLFile", (bool (mrpt::obs::VelodyneCalibration::*)(const std::string &)) &mrpt::obs::VelodyneCalibration::loadFromYAMLFile, "Loads calibration from a YAML calibration file.\n \n\n loadFromYAMLText, loadFromXMLFile\n \n\n false on any error, true on success \n\nC++: mrpt::obs::VelodyneCalibration::loadFromYAMLFile(const std::string &) --> bool", pybind11::arg("velodyne_calib_yaml_filename"));
		cl.def("assign", (struct mrpt::obs::VelodyneCalibration & (mrpt::obs::VelodyneCalibration::*)(const struct mrpt::obs::VelodyneCalibration &)) &mrpt::obs::VelodyneCalibration::operator=, "C++: mrpt::obs::VelodyneCalibration::operator=(const struct mrpt::obs::VelodyneCalibration &) --> struct mrpt::obs::VelodyneCalibration &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::VelodyneCalibration::PerLaserCalib file:mrpt/obs/VelodyneCalibration.h line:64
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::VelodyneCalibration::PerLaserCalib, std::shared_ptr<mrpt::obs::VelodyneCalibration::PerLaserCalib>> cl(enclosing_class, "PerLaserCalib", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::VelodyneCalibration::PerLaserCalib(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::VelodyneCalibration::PerLaserCalib const &o){ return new mrpt::obs::VelodyneCalibration::PerLaserCalib(o); } ) );
			cl.def_readwrite("azimuthCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::azimuthCorrection);
			cl.def_readwrite("verticalCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::verticalCorrection);
			cl.def_readwrite("distanceCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::distanceCorrection);
			cl.def_readwrite("verticalOffsetCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::verticalOffsetCorrection);
			cl.def_readwrite("horizontalOffsetCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::horizontalOffsetCorrection);
			cl.def_readwrite("sinVertCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::sinVertCorrection);
			cl.def_readwrite("cosVertCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::cosVertCorrection);
			cl.def_readwrite("sinVertOffsetCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::sinVertOffsetCorrection);
			cl.def_readwrite("cosVertOffsetCorrection", &mrpt::obs::VelodyneCalibration::PerLaserCalib::cosVertOffsetCorrection);
			cl.def("assign", (struct mrpt::obs::VelodyneCalibration::PerLaserCalib & (mrpt::obs::VelodyneCalibration::PerLaserCalib::*)(const struct mrpt::obs::VelodyneCalibration::PerLaserCalib &)) &mrpt::obs::VelodyneCalibration::PerLaserCalib::operator=, "C++: mrpt::obs::VelodyneCalibration::PerLaserCalib::operator=(const struct mrpt::obs::VelodyneCalibration::PerLaserCalib &) --> struct mrpt::obs::VelodyneCalibration::PerLaserCalib &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::CObservationVelodyneScan file:mrpt/obs/CObservationVelodyneScan.h line:82
		pybind11::class_<mrpt::obs::CObservationVelodyneScan, std::shared_ptr<mrpt::obs::CObservationVelodyneScan>, PyCallBack_mrpt_obs_CObservationVelodyneScan, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationVelodyneScan", "A `CObservation`-derived class for RAW DATA (and optionally, point cloud) of\n scans from 3D Velodyne LIDAR scanners.\n A scan comprises one or more \"velodyne packets\" (refer to Velodyne user\n manual). Normally, a full 360 degrees sweep is included in one observation\n object. Note that if a pointcloud is generated inside this class, each point\n is annotated with per-point information about its exact azimuth and laser_id\n (ring number), an information that is loss when inserting the observation in\n a regular mrpt::maps::CPointsMap.\n\n Main data fields:\n - CObservationVelodyneScan::scan_packets with raw data packets.\n - CObservationVelodyneScan::point_cloud normally empty after grabbing for\n efficiency, can be generated calling \n\n Dual return mode is supported (see mrpt::hwdrivers::CVelodyneScanner).\n\n Axes convention for point cloud (x,y,z) coordinates:\n\n    \n\n If it can be assumed that the sensor moves SLOWLY through the environment\n (i.e. its pose can be approximated to be the same since the beginning to the\n end of one complete scan)\n then this observation can be converted / loaded into the following other\n classes:\n  - Maps of points (these require first generating the pointcloud in this\n observation object with\n mrpt::obs::CObservationVelodyneScan::generatePointCloud() ):\n    - mrpt::maps::CPointsMap::loadFromVelodyneScan() (available in all\n derived classes)\n    - and the generic method:mrpt::maps::CPointsMap::insertObservation()\n  - mrpt::opengl::CPointCloud and mrpt::opengl::CPointCloudColoured is\n supported by first converting\n    this scan to a mrpt::maps::CPointsMap-derived class, then loading it into\n the opengl object.\n\n Otherwise, the following API exists for accurate reconstruction of the\n sensor path in SE(3) over time:\n  - CObservationVelodyneScan::generatePointCloudAlongSE3Trajectory()\n\n  Note that this object has  timestamp fields:\n  - The standard CObservation::timestamp field in the base class, which\n should contain the accurate satellite-based UTC timestamp, and\n  - the field CObservationVelodyneScan::originalReceivedTimestamp, with the\n local computer-based timestamp based on the reception of the message in the\n computer.\n  Both timestamps correspond to the firing of the first laser in the\n first CObservationVelodyneScan::scan_packets packet.\n\n \n New in MRPT 1.4.0\n \n\n CObservation, mrpt::maps::CPointsMap, mrpt::hwdrivers::CVelodyneScanner\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan(); }, [](){ return new PyCallBack_mrpt_obs_CObservationVelodyneScan(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationVelodyneScan const &o){ return new PyCallBack_mrpt_obs_CObservationVelodyneScan(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationVelodyneScan const &o){ return new mrpt::obs::CObservationVelodyneScan(o); } ) );
		cl.def_readwrite("minRange", &mrpt::obs::CObservationVelodyneScan::minRange);
		cl.def_readwrite("maxRange", &mrpt::obs::CObservationVelodyneScan::maxRange);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationVelodyneScan::sensorPose);
		cl.def_readwrite("scan_packets", &mrpt::obs::CObservationVelodyneScan::scan_packets);
		cl.def_readwrite("calibration", &mrpt::obs::CObservationVelodyneScan::calibration);
		cl.def_readwrite("originalReceivedTimestamp", &mrpt::obs::CObservationVelodyneScan::originalReceivedTimestamp);
		cl.def_readwrite("has_satellite_timestamp", &mrpt::obs::CObservationVelodyneScan::has_satellite_timestamp);
		cl.def_readwrite("point_cloud", &mrpt::obs::CObservationVelodyneScan::point_cloud);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationVelodyneScan::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationVelodyneScan::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationVelodyneScan::*)() const) &mrpt::obs::CObservationVelodyneScan::GetRuntimeClass, "C++: mrpt::obs::CObservationVelodyneScan::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationVelodyneScan::*)() const) &mrpt::obs::CObservationVelodyneScan::clone, "C++: mrpt::obs::CObservationVelodyneScan::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationVelodyneScan::CreateObject, "C++: mrpt::obs::CObservationVelodyneScan::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getOriginalReceivedTimeStamp", (mrpt::Clock::time_point (mrpt::obs::CObservationVelodyneScan::*)() const) &mrpt::obs::CObservationVelodyneScan::getOriginalReceivedTimeStamp, "C++: mrpt::obs::CObservationVelodyneScan::getOriginalReceivedTimeStamp() const --> mrpt::Clock::time_point");
		cl.def("generatePointCloud", [](mrpt::obs::CObservationVelodyneScan &o) -> void { return o.generatePointCloud(); }, "");
		cl.def("generatePointCloud", (void (mrpt::obs::CObservationVelodyneScan::*)(const struct mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters &)) &mrpt::obs::CObservationVelodyneScan::generatePointCloud, "Generates the point cloud into the point cloud data fields in \n where it is stored in local coordinates wrt the sensor (neither the\n vehicle nor the world).\n So, this method does not take into account the possible motion of the\n sensor through the world as it collects LIDAR scans.\n For high dynamics, see the more costly API\n generatePointCloudAlongSE3Trajectory()\n \n\n Points with ranges out of [minRange,maxRange] are discarded; as\n well, other filters are available in \n \n\n generatePointCloudAlongSE3Trajectory(),\n TGeneratePointCloudParameters\n\nC++: mrpt::obs::CObservationVelodyneScan::generatePointCloud(const struct mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters &) --> void", pybind11::arg("params"));
		cl.def("generatePointCloud", [](mrpt::obs::CObservationVelodyneScan &o, struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper & a0) -> void { return o.generatePointCloud(a0); }, "", pybind11::arg("dest"));
		cl.def("generatePointCloud", (void (mrpt::obs::CObservationVelodyneScan::*)(struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper &, const struct mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters &)) &mrpt::obs::CObservationVelodyneScan::generatePointCloud, "C++: mrpt::obs::CObservationVelodyneScan::generatePointCloud(struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper &, const struct mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters &) --> void", pybind11::arg("dest"), pybind11::arg("params"));
		cl.def("unload", (void (mrpt::obs::CObservationVelodyneScan::*)() const) &mrpt::obs::CObservationVelodyneScan::unload, "@{ \n\n Frees the memory of cached point clouds \n\nC++: mrpt::obs::CObservationVelodyneScan::unload() const --> void");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationVelodyneScan::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationVelodyneScan::getSensorPose, "@} \n\nC++: mrpt::obs::CObservationVelodyneScan::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationVelodyneScan::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationVelodyneScan::setSensorPose, "C++: mrpt::obs::CObservationVelodyneScan::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationVelodyneScan & (mrpt::obs::CObservationVelodyneScan::*)(const class mrpt::obs::CObservationVelodyneScan &)) &mrpt::obs::CObservationVelodyneScan::operator=, "C++: mrpt::obs::CObservationVelodyneScan::operator=(const class mrpt::obs::CObservationVelodyneScan &) --> class mrpt::obs::CObservationVelodyneScan &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservationVelodyneScan::laser_return_t file:mrpt/obs/CObservationVelodyneScan.h line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::laser_return_t, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::laser_return_t>> cl(enclosing_class, "laser_return_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::laser_return_t(); } ) );
			cl.def("distance", (uint16_t (mrpt::obs::CObservationVelodyneScan::laser_return_t::*)() const) &mrpt::obs::CObservationVelodyneScan::laser_return_t::distance, "C++: mrpt::obs::CObservationVelodyneScan::laser_return_t::distance() const --> uint16_t");
			cl.def("intensity", (uint8_t (mrpt::obs::CObservationVelodyneScan::laser_return_t::*)() const) &mrpt::obs::CObservationVelodyneScan::laser_return_t::intensity, "C++: mrpt::obs::CObservationVelodyneScan::laser_return_t::intensity() const --> uint8_t");
		}

		{ // mrpt::obs::CObservationVelodyneScan::raw_block_t file:mrpt/obs/CObservationVelodyneScan.h line:135
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::raw_block_t, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::raw_block_t>> cl(enclosing_class, "raw_block_t", "Raw Velodyne data block.\n  Each block contains data from either the upper or lower laser\n  bank.  The device returns three times as many upper bank blocks. ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::raw_block_t(); } ) );
			cl.def("header", (uint16_t (mrpt::obs::CObservationVelodyneScan::raw_block_t::*)() const) &mrpt::obs::CObservationVelodyneScan::raw_block_t::header, "C++: mrpt::obs::CObservationVelodyneScan::raw_block_t::header() const --> uint16_t");
			cl.def("rotation", (uint16_t (mrpt::obs::CObservationVelodyneScan::raw_block_t::*)() const) &mrpt::obs::CObservationVelodyneScan::raw_block_t::rotation, "C++: mrpt::obs::CObservationVelodyneScan::raw_block_t::rotation() const --> uint16_t");
		}

		{ // mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket file:mrpt/obs/CObservationVelodyneScan.h line:149
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket>> cl(enclosing_class, "TVelodyneRawPacket", "One unit of data from the scanner (the payload of one UDP DATA packet)");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket(); } ) );
			cl.def_readwrite("laser_return_mode", &mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket::laser_return_mode);
			cl.def_readwrite("velodyne_model_ID", &mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket::velodyne_model_ID);
			cl.def("gps_timestamp", (uint32_t (mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket::*)() const) &mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket::gps_timestamp, "C++: mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket::gps_timestamp() const --> uint32_t");
		}

		{ // mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket file:mrpt/obs/CObservationVelodyneScan.h line:168
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket>> cl(enclosing_class, "TVelodynePositionPacket", "Payload of one POSITION packet ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket(); } ) );
			cl.def("gps_timestamp", (uint32_t (mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket::*)() const) &mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket::gps_timestamp, "C++: mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket::gps_timestamp() const --> uint32_t");
			cl.def("unused2", (uint32_t (mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket::*)() const) &mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket::unused2, "C++: mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket::unused2() const --> uint32_t");
		}

		{ // mrpt::obs::CObservationVelodyneScan::TPointCloud file:mrpt/obs/CObservationVelodyneScan.h line:212
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::TPointCloud, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::TPointCloud>> cl(enclosing_class, "TPointCloud", "See  and  ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::TPointCloud(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservationVelodyneScan::TPointCloud const &o){ return new mrpt::obs::CObservationVelodyneScan::TPointCloud(o); } ) );
			cl.def_readwrite("x", &mrpt::obs::CObservationVelodyneScan::TPointCloud::x);
			cl.def_readwrite("y", &mrpt::obs::CObservationVelodyneScan::TPointCloud::y);
			cl.def_readwrite("z", &mrpt::obs::CObservationVelodyneScan::TPointCloud::z);
			cl.def_readwrite("intensity", &mrpt::obs::CObservationVelodyneScan::TPointCloud::intensity);
			cl.def_readwrite("timestamp", &mrpt::obs::CObservationVelodyneScan::TPointCloud::timestamp);
			cl.def_readwrite("azimuth", &mrpt::obs::CObservationVelodyneScan::TPointCloud::azimuth);
			cl.def_readwrite("laser_id", &mrpt::obs::CObservationVelodyneScan::TPointCloud::laser_id);
			cl.def_readwrite("pointsForLaserID", &mrpt::obs::CObservationVelodyneScan::TPointCloud::pointsForLaserID);
			cl.def("size", (std::size_t (mrpt::obs::CObservationVelodyneScan::TPointCloud::*)() const) &mrpt::obs::CObservationVelodyneScan::TPointCloud::size, "C++: mrpt::obs::CObservationVelodyneScan::TPointCloud::size() const --> std::size_t");
			cl.def("reserve", (void (mrpt::obs::CObservationVelodyneScan::TPointCloud::*)(std::size_t)) &mrpt::obs::CObservationVelodyneScan::TPointCloud::reserve, "C++: mrpt::obs::CObservationVelodyneScan::TPointCloud::reserve(std::size_t) --> void", pybind11::arg("n"));
			cl.def("clear", (void (mrpt::obs::CObservationVelodyneScan::TPointCloud::*)()) &mrpt::obs::CObservationVelodyneScan::TPointCloud::clear, "Sets all vectors to zero length \n\nC++: mrpt::obs::CObservationVelodyneScan::TPointCloud::clear() --> void");
			cl.def("clear_deep", (void (mrpt::obs::CObservationVelodyneScan::TPointCloud::*)()) &mrpt::obs::CObservationVelodyneScan::TPointCloud::clear_deep, "Like clear(), but also enforcing freeing memory \n\nC++: mrpt::obs::CObservationVelodyneScan::TPointCloud::clear_deep() --> void");
			cl.def("assign", (struct mrpt::obs::CObservationVelodyneScan::TPointCloud & (mrpt::obs::CObservationVelodyneScan::TPointCloud::*)(const struct mrpt::obs::CObservationVelodyneScan::TPointCloud &)) &mrpt::obs::CObservationVelodyneScan::TPointCloud::operator=, "C++: mrpt::obs::CObservationVelodyneScan::TPointCloud::operator=(const struct mrpt::obs::CObservationVelodyneScan::TPointCloud &) --> struct mrpt::obs::CObservationVelodyneScan::TPointCloud &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters file:mrpt/obs/CObservationVelodyneScan.h line:249
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters>> cl(enclosing_class, "TGeneratePointCloudParameters", "@{ ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters const &o){ return new mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters(o); } ) );
			cl.def_readwrite("minAzimuth_deg", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::minAzimuth_deg);
			cl.def_readwrite("maxAzimuth_deg", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::maxAzimuth_deg);
			cl.def_readwrite("minDistance", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::minDistance);
			cl.def_readwrite("maxDistance", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::maxDistance);
			cl.def_readwrite("ROI_x_min", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::ROI_x_min);
			cl.def_readwrite("ROI_x_max", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::ROI_x_max);
			cl.def_readwrite("ROI_y_min", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::ROI_y_min);
			cl.def_readwrite("ROI_y_max", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::ROI_y_max);
			cl.def_readwrite("ROI_z_min", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::ROI_z_min);
			cl.def_readwrite("ROI_z_max", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::ROI_z_max);
			cl.def_readwrite("nROI_x_min", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::nROI_x_min);
			cl.def_readwrite("nROI_x_max", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::nROI_x_max);
			cl.def_readwrite("nROI_y_min", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::nROI_y_min);
			cl.def_readwrite("nROI_y_max", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::nROI_y_max);
			cl.def_readwrite("nROI_z_min", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::nROI_z_min);
			cl.def_readwrite("nROI_z_max", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::nROI_z_max);
			cl.def_readwrite("isolatedPointsFilterDistance", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::isolatedPointsFilterDistance);
			cl.def_readwrite("filterByROI", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::filterByROI);
			cl.def_readwrite("filterBynROI", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::filterBynROI);
			cl.def_readwrite("filterOutIsolatedPoints", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::filterOutIsolatedPoints);
			cl.def_readwrite("dualKeepStrongest", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::dualKeepStrongest);
			cl.def_readwrite("dualKeepLast", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::dualKeepLast);
			cl.def_readwrite("generatePerPointTimestamp", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::generatePerPointTimestamp);
			cl.def_readwrite("generatePerPointAzimuth", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::generatePerPointAzimuth);
			cl.def_readwrite("generatePointsForLaserID", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters::generatePointsForLaserID);
		}

		{ // mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper file:mrpt/obs/CObservationVelodyneScan.h line:298
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper>, PyCallBack_mrpt_obs_CObservationVelodyneScan_PointCloudStorageWrapper> cl(enclosing_class, "PointCloudStorageWrapper", "Derive from this class to generate pointclouds into custom containers.\n \n\n generatePointCloud() ");
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_obs_CObservationVelodyneScan_PointCloudStorageWrapper(); } ) );
			cl.def("reserve", (void (mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::*)(std::size_t)) &mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::reserve, "C++: mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::reserve(std::size_t) --> void", pybind11::arg("n"));
			cl.def("resizeLaserCount", (void (mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::*)(std::size_t)) &mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::resizeLaserCount, "C++: mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::resizeLaserCount(std::size_t) --> void", pybind11::arg("n"));
			cl.def("add_point", (void (mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::*)(float, float, float, uint8_t, const mrpt::Clock::time_point &, const float, uint16_t)) &mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::add_point, "Process the insertion of a new (x,y,z) point to the cloud, in\n sensor-centric coordinates, with the exact timestamp of that LIDAR\n ray \n\nC++: mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::add_point(float, float, float, uint8_t, const mrpt::Clock::time_point &, const float, uint16_t) --> void", pybind11::arg("pt_x"), pybind11::arg("pt_y"), pybind11::arg("pt_z"), pybind11::arg("pt_intensity"), pybind11::arg("tim"), pybind11::arg("azimuth"), pybind11::arg("laser_id"));
			cl.def("assign", (struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper & (mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::*)(const struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper &)) &mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::operator=, "C++: mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper::operator=(const struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper &) --> struct mrpt::obs::CObservationVelodyneScan::PointCloudStorageWrapper &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudSE3Results file:mrpt/obs/CObservationVelodyneScan.h line:338
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudSE3Results, std::shared_ptr<mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudSE3Results>> cl(enclosing_class, "TGeneratePointCloudSE3Results", "Results for generatePointCloudAlongSE3Trajectory() ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudSE3Results(); } ) );
			cl.def_readwrite("num_points", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudSE3Results::num_points);
			cl.def_readwrite("num_correctly_inserted_points", &mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudSE3Results::num_correctly_inserted_points);
		}

	}
}
