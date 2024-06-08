#include <chrono>
#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TPixelLabelInfo.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

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

// mrpt::obs::CObservation3DRangeScan file:mrpt/obs/CObservation3DRangeScan.h line:149
struct PyCallBack_mrpt_obs_CObservation3DRangeScan : public mrpt::obs::CObservation3DRangeScan {
	using mrpt::obs::CObservation3DRangeScan::CObservation3DRangeScan;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservation3DRangeScan::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservation3DRangeScan::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservation3DRangeScan::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DRangeScan::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DRangeScan::serializeFrom(a0, a1);
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DRangeScan::load_impl();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DRangeScan::unload();
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DRangeScan::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DRangeScan::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservation::getOriginalReceivedTimeStamp();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DRangeScan *>(this), "exportTxtDataRow");
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
};

void bind_mrpt_obs_TPixelLabelInfo(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::TPixelLabelInfoBase file:mrpt/obs/TPixelLabelInfo.h line:25
		pybind11::class_<mrpt::obs::TPixelLabelInfoBase, std::shared_ptr<mrpt::obs::TPixelLabelInfoBase>> cl(M("mrpt::obs"), "TPixelLabelInfoBase", "Virtual interface to all pixel-label semantic information structs.\n\n See CObservation3DRangeScan::pixelLabels\n \n\n\n ");
		cl.def_readwrite("pixelLabelNames", &mrpt::obs::TPixelLabelInfoBase::pixelLabelNames);
		cl.def_readonly("BITFIELD_BYTES", &mrpt::obs::TPixelLabelInfoBase::BITFIELD_BYTES);
		cl.def("getLabelName", (const std::string & (mrpt::obs::TPixelLabelInfoBase::*)(unsigned int) const) &mrpt::obs::TPixelLabelInfoBase::getLabelName, "C++: mrpt::obs::TPixelLabelInfoBase::getLabelName(unsigned int) const --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("label_idx"));
		cl.def("setLabelName", (void (mrpt::obs::TPixelLabelInfoBase::*)(unsigned int, const std::string &)) &mrpt::obs::TPixelLabelInfoBase::setLabelName, "C++: mrpt::obs::TPixelLabelInfoBase::setLabelName(unsigned int, const std::string &) --> void", pybind11::arg("label_idx"), pybind11::arg("name"));
		cl.def("checkLabelNameExistence", (int (mrpt::obs::TPixelLabelInfoBase::*)(const std::string &) const) &mrpt::obs::TPixelLabelInfoBase::checkLabelNameExistence, "Check the existence of a label by returning its associated index.\n -1 if it does not exist. \n\nC++: mrpt::obs::TPixelLabelInfoBase::checkLabelNameExistence(const std::string &) const --> int", pybind11::arg("name"));
		cl.def("setSize", (void (mrpt::obs::TPixelLabelInfoBase::*)(const int, const int)) &mrpt::obs::TPixelLabelInfoBase::setSize, "Resizes the matrix pixelLabels to the given size, setting all\n bitfields to zero (that is, all pixels are assigned NONE category).\n\nC++: mrpt::obs::TPixelLabelInfoBase::setSize(const int, const int) --> void", pybind11::arg("NROWS"), pybind11::arg("NCOLS"));
		cl.def("setLabel", (void (mrpt::obs::TPixelLabelInfoBase::*)(const int, const int, uint8_t)) &mrpt::obs::TPixelLabelInfoBase::setLabel, "Mark the pixel(row,col) as classified in the category \n which may be in the range 0 to MAX_NUM_LABELS-1\n Note that 0 is a valid label index, it does not mean \"no label\" \n\n\n unsetLabel, unsetAll \n\nC++: mrpt::obs::TPixelLabelInfoBase::setLabel(const int, const int, uint8_t) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("label_idx"));
		cl.def("getLabels", (void (mrpt::obs::TPixelLabelInfoBase::*)(const int, const int, unsigned char &)) &mrpt::obs::TPixelLabelInfoBase::getLabels, "C++: mrpt::obs::TPixelLabelInfoBase::getLabels(const int, const int, unsigned char &) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("labels"));
		cl.def("unsetLabel", (void (mrpt::obs::TPixelLabelInfoBase::*)(const int, const int, uint8_t)) &mrpt::obs::TPixelLabelInfoBase::unsetLabel, "For the pixel(row,col), removes its classification into the category\n  which may be in the range 0 to 7\n Note that 0 is a valid label index, it does not mean \"no label\" \n\n\n setLabel, unsetAll \n\nC++: mrpt::obs::TPixelLabelInfoBase::unsetLabel(const int, const int, uint8_t) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("label_idx"));
		cl.def("unsetAll", (void (mrpt::obs::TPixelLabelInfoBase::*)(const int, const int, uint8_t)) &mrpt::obs::TPixelLabelInfoBase::unsetAll, "Removes all categories for pixel(row,col)  \n setLabel, unsetLabel\n\nC++: mrpt::obs::TPixelLabelInfoBase::unsetAll(const int, const int, uint8_t) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("label_idx"));
		cl.def("checkLabel", (bool (mrpt::obs::TPixelLabelInfoBase::*)(const int, const int, uint8_t) const) &mrpt::obs::TPixelLabelInfoBase::checkLabel, "Checks whether pixel(row,col) has been clasified into category \n which may be in the range 0 to 7\n \n\n unsetLabel, unsetAll \n\nC++: mrpt::obs::TPixelLabelInfoBase::checkLabel(const int, const int, uint8_t) const --> bool", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("label_idx"));
		cl.def("writeToStream", (void (mrpt::obs::TPixelLabelInfoBase::*)(class mrpt::serialization::CArchive &) const) &mrpt::obs::TPixelLabelInfoBase::writeToStream, "C++: mrpt::obs::TPixelLabelInfoBase::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def_static("readAndBuildFromStream", (struct mrpt::obs::TPixelLabelInfoBase * (*)(class mrpt::serialization::CArchive &)) &mrpt::obs::TPixelLabelInfoBase::readAndBuildFromStream, "C++: mrpt::obs::TPixelLabelInfoBase::readAndBuildFromStream(class mrpt::serialization::CArchive &) --> struct mrpt::obs::TPixelLabelInfoBase *", pybind11::return_value_policy::automatic, pybind11::arg("in"));

		cl.def("__str__", [](mrpt::obs::TPixelLabelInfoBase const &o) -> std::string { std::ostringstream s; using namespace mrpt::obs; s << o; return s.str(); } );
	}
	{ // mrpt::obs::TRangeImageFilterParams file:mrpt/obs/TRangeImageFilter.h line:17
		pybind11::class_<mrpt::obs::TRangeImageFilterParams, std::shared_ptr<mrpt::obs::TRangeImageFilterParams>> cl(M("mrpt::obs"), "TRangeImageFilterParams", "Used in CObservation3DRangeScan::unprojectInto() ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::TRangeImageFilterParams(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::TRangeImageFilterParams const &o){ return new mrpt::obs::TRangeImageFilterParams(o); } ) );
		cl.def_readwrite("rangeCheckBetween", &mrpt::obs::TRangeImageFilterParams::rangeCheckBetween);
		cl.def_readwrite("mark_invalid_ranges", &mrpt::obs::TRangeImageFilterParams::mark_invalid_ranges);
	}
	{ // mrpt::obs::TRangeImageFilter file:mrpt/obs/TRangeImageFilter.h line:48
		pybind11::class_<mrpt::obs::TRangeImageFilter, std::shared_ptr<mrpt::obs::TRangeImageFilter>> cl(M("mrpt::obs"), "TRangeImageFilter", "Mainly for internal use within\n CObservation3DRangeScan::unprojectInto() ");
		cl.def( pybind11::init<const struct mrpt::obs::TRangeImageFilterParams &>(), pybind11::arg("filter_params") );

		cl.def( pybind11::init( [](){ return new mrpt::obs::TRangeImageFilter(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::TRangeImageFilter const &o){ return new mrpt::obs::TRangeImageFilter(o); } ) );
		cl.def_readwrite("fp", &mrpt::obs::TRangeImageFilter::fp);
		cl.def("do_range_filter", (bool (mrpt::obs::TRangeImageFilter::*)(size_t, size_t, const float) const) &mrpt::obs::TRangeImageFilter::do_range_filter, "Returns true if the point (r,c) with depth D passes all filters. \n\nC++: mrpt::obs::TRangeImageFilter::do_range_filter(size_t, size_t, const float) const --> bool", pybind11::arg("r"), pybind11::arg("c"), pybind11::arg("D"));
	}
	{ // mrpt::obs::CObservation3DRangeScan file:mrpt/obs/CObservation3DRangeScan.h line:149
		pybind11::class_<mrpt::obs::CObservation3DRangeScan, std::shared_ptr<mrpt::obs::CObservation3DRangeScan>, PyCallBack_mrpt_obs_CObservation3DRangeScan, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservation3DRangeScan", "A depth or RGB+D image from a time-of-flight or structured-light sensor.\n\n This kind of observations can carry one or more of these data fields:\n  - 3D point cloud (as {x,y,z} `float` vectors).\n  - Each 3D point has its associated `(u,v)` pixel coordinates in \n &  (New in MRPT 1.4.0)\n  - 2D range image (as a matrix): Each entry in the matrix\n    `rangeImage(ROW,COLUMN)` contains a distance or a depth, depending\n    on  Ranges are stored as uint16_t for efficiency.\n    The units of ranges are stored separately in `rangeUnits`.\n  - 2D intensity (grayscale or RGB) image (as a mrpt::img::CImage).\n  - 2D confidence image (as a mrpt::img::CImage): For each pixel, a 0x00\n    and a 0xFF mean the lowest and highest confidence levels, respectively.\n  - Semantic labels: Stored as a matrix of bitfields, each bit having a\n    user-defined meaning.\n  - For cameras supporting multiple returns per pixels, different layers of\n    range images are available in the map \n\n The coordinates of the 3D point cloud are in meters with respect to the\n depth camera origin of coordinates,\n with the +X axis pointing forward, +Y pointing left-hand and +Z pointing\n up. By convention, a 3D point with its coordinates set to (0,0,0), will be\n considered as invalid.\n\n The field CObservation3DRangeScan::relativePoseIntensityWRTDepth describes\n the change of coordinates from the depth camera to the intensity\n (RGB or grayscale) camera. In some cameras both cameras coincide,\n so this pose would be just a rotation (0,0,0,-90deg,0,-90deg).\n In Kinect-like cameras there is also an offset, as shown in this figure:\n\n ![CObservation3DRangeScan axes](CObservation3DRangeScan_figRefSystem.png)\n\n In any case, check the field  or the method\n  to determine if both frames of\n reference coincide, since even for Kinect cameras both can coincide if the\n images have been rectified.\n\n The 2D images and matrices are stored as common images, with an up->down\n rows order and left->right, as usual.\n Optionally, the intensity and confidence channels can be set to\n delayed-load images for off-rawlog storage so it saves\n memory by having loaded in memory just the needed images. See the methods\n load() and unload().\n Due to the intensive storage requirements of this kind of observations, this\n observation is the only one in MRPT\n for which it's recommended to always call \"load()\" and \"unload()\" before\n and after using the observation, *ONLY* when\n the observation was read from a rawlog dataset, in order to make sure that\n all the externally stored data fields are\n loaded and ready in memory.\n\n Some classes that grab observations of this type are:\n  - mrpt::hwdrivers::CSwissRanger3DCamera\n  - mrpt::hwdrivers::CKinect\n  - mrpt::hwdrivers::COpenNI2Sensor\n\n There are two sets of calibration parameters (see\n mrpt::vision::checkerBoardStereoCalibration() or the ready-to-use GUI program\n [kinect-calibrate](https://www.mrpt.org/list-of-mrpt-apps/application-kinect-stereo-calib/):\n  - cameraParams: Intrinsics of the depth camera.\n  - cameraParamsIntensity: Intrinsics of the intensity (RGB) camera.\n\n In some cameras, like SwissRanger, both are the same. It is possible in\n Kinect to rectify the range images such both cameras\n seem to coincide and then both sets of camera parameters will be identical.\n\n Range data can be interpreted in two different ways depending on the 3D\n camera (this field is already set to the correct setting when grabbing\n observations from an mrpt::hwdrivers sensor):\n  - `range_is_depth==true`: Kinect-like ranges: entries of \n    are distances along the +X (front-facing) axis.\n  - `range_is_depth==false`: Ranges in  are actual distances\n    in 3D, i.e. a bit larger than the depth.\n\n The `intensity` channel may come from different channels in sensors as\n Kinect. Look at field `intensityImageChannel` to find out if the image was\n grabbed from the visible (RGB) or IR channels.\n\n 3D point clouds can be generated at any moment after grabbing with\n CObservation3DRangeScan::unprojectInto(), provided the correct calibration\n parameters. Note that unprojectInto() will store the point cloud in\n sensor-centric local coordinates by default, but changing its parameters you\n can obtain a vehicle-centric, or world-frame point cloud instead.\n\n Examples of how to assign labels to pixels (for object segmentation, semantic\n information, etc.):\n\n \n\n\n\n\n\n\n\n\n \n Since MRPT 1.5.0, external files format can be selected at runtime\n       with `CObservation3DRangeScan::EXTERNALS_AS_TEXT`\n\n \n mrpt::hwdrivers::CSwissRanger3DCamera, mrpt::hwdrivers::CKinect,\n     mrpt::hwdrivers::COpenNI2Sensor, mrpt::obs::CObservation\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation3DRangeScan(); }, [](){ return new PyCallBack_mrpt_obs_CObservation3DRangeScan(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservation3DRangeScan const &o){ return new PyCallBack_mrpt_obs_CObservation3DRangeScan(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservation3DRangeScan const &o){ return new mrpt::obs::CObservation3DRangeScan(o); } ) );

		pybind11::enum_<mrpt::obs::CObservation3DRangeScan::TIntensityChannelID>(cl, "TIntensityChannelID", pybind11::arithmetic(), "@{ \n\n Enum type for intensityImageChannel ")
			.value("CH_VISIBLE", mrpt::obs::CObservation3DRangeScan::CH_VISIBLE)
			.value("CH_IR", mrpt::obs::CObservation3DRangeScan::CH_IR)
			.export_values();

		cl.def_readwrite("hasPoints3D", &mrpt::obs::CObservation3DRangeScan::hasPoints3D);
		cl.def_readwrite("points3D_x", &mrpt::obs::CObservation3DRangeScan::points3D_x);
		cl.def_readwrite("points3D_y", &mrpt::obs::CObservation3DRangeScan::points3D_y);
		cl.def_readwrite("points3D_z", &mrpt::obs::CObservation3DRangeScan::points3D_z);
		cl.def_readwrite("points3D_idxs_x", &mrpt::obs::CObservation3DRangeScan::points3D_idxs_x);
		cl.def_readwrite("points3D_idxs_y", &mrpt::obs::CObservation3DRangeScan::points3D_idxs_y);
		cl.def_readwrite("hasRangeImage", &mrpt::obs::CObservation3DRangeScan::hasRangeImage);
		cl.def_readwrite("rangeImage", &mrpt::obs::CObservation3DRangeScan::rangeImage);
		cl.def_readwrite("rangeImageOtherLayers", &mrpt::obs::CObservation3DRangeScan::rangeImageOtherLayers);
		cl.def_readwrite("rangeUnits", &mrpt::obs::CObservation3DRangeScan::rangeUnits);
		cl.def_readwrite("range_is_depth", &mrpt::obs::CObservation3DRangeScan::range_is_depth);
		cl.def_readwrite("hasIntensityImage", &mrpt::obs::CObservation3DRangeScan::hasIntensityImage);
		cl.def_readwrite("intensityImage", &mrpt::obs::CObservation3DRangeScan::intensityImage);
		cl.def_readwrite("intensityImageChannel", &mrpt::obs::CObservation3DRangeScan::intensityImageChannel);
		cl.def_readwrite("hasConfidenceImage", &mrpt::obs::CObservation3DRangeScan::hasConfidenceImage);
		cl.def_readwrite("confidenceImage", &mrpt::obs::CObservation3DRangeScan::confidenceImage);
		cl.def_readwrite("pixelLabels", &mrpt::obs::CObservation3DRangeScan::pixelLabels);
		cl.def_readwrite("cameraParams", &mrpt::obs::CObservation3DRangeScan::cameraParams);
		cl.def_readwrite("cameraParamsIntensity", &mrpt::obs::CObservation3DRangeScan::cameraParamsIntensity);
		cl.def_readwrite("relativePoseIntensityWRTDepth", &mrpt::obs::CObservation3DRangeScan::relativePoseIntensityWRTDepth);
		cl.def_readwrite("maxRange", &mrpt::obs::CObservation3DRangeScan::maxRange);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservation3DRangeScan::sensorPose);
		cl.def_readwrite("stdError", &mrpt::obs::CObservation3DRangeScan::stdError);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservation3DRangeScan::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservation3DRangeScan::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::GetRuntimeClass, "C++: mrpt::obs::CObservation3DRangeScan::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::clone, "C++: mrpt::obs::CObservation3DRangeScan::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservation3DRangeScan::CreateObject, "C++: mrpt::obs::CObservation3DRangeScan::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("load_impl", (void (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::load_impl, "@{ \n\n Makes sure all images and other fields which may be externally stored\n are loaded in memory.\n  Note that for all CImages, calling load() is not required since the\n images will be automatically loaded upon first access, so load()\n shouldn't be needed to be called in normal cases by the user.\n  If all the data were alredy loaded or this object has no externally\n stored data fields, calling this method has no effects.\n \n\n unload\n\nC++: mrpt::obs::CObservation3DRangeScan::load_impl() const --> void");
		cl.def("unload", (void (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::unload, "Unload all images, for the case they being delayed-load images stored in\n external files (othewise, has no effect).\n \n\n load\n\nC++: mrpt::obs::CObservation3DRangeScan::unload() const --> void");
		cl.def("convertTo2DScan", [](mrpt::obs::CObservation3DRangeScan &o, class mrpt::obs::CObservation2DRangeScan & a0, const struct mrpt::obs::T3DPointsTo2DScanParams & a1) -> void { return o.convertTo2DScan(a0, a1); }, "", pybind11::arg("out_scan2d"), pybind11::arg("scanParams"));
		cl.def("convertTo2DScan", (void (mrpt::obs::CObservation3DRangeScan::*)(class mrpt::obs::CObservation2DRangeScan &, const struct mrpt::obs::T3DPointsTo2DScanParams &, const struct mrpt::obs::TRangeImageFilterParams &)) &mrpt::obs::CObservation3DRangeScan::convertTo2DScan, "Convert this 3D observation into an \"equivalent 2D fake laser scan\",\n with a configurable vertical FOV.\n\n  The result is a 2D laser scan with more \"rays\" (N) than columns has the\n 3D observation (W), exactly: N = W * oversampling_ratio.\n  This oversampling is required since laser scans sample the space at\n evenly-separated angles, while\n  a range camera follows a tangent-like distribution. By oversampling we\n make sure we don't leave \"gaps\" unseen by the virtual \"2D laser\".\n\n  All obstacles within a frustum are considered and the minimum distance\n is kept in each direction.\n  The horizontal FOV of the frustum is automatically computed from the\n intrinsic parameters, but the\n  vertical FOV must be provided by the user, and can be set to be\n assymetric which may be useful\n  depending on the zone of interest where to look for obstacles.\n\n  All spatial transformations are riguorosly taken into account in this\n class, using the depth camera\n  intrinsic calibration parameters.\n\n  The timestamp of the new object is copied from the 3D object.\n  Obviously, a requisite for calling this method is the 3D observation\n having range data,\n  i.e. hasRangeImage must be true. It's not needed to have RGB data nor\n the raw 3D point clouds\n  for this method to work.\n\n  If `scanParams.use_origin_sensor_pose` is `true`, the points will be\n projected to 3D and then reprojected\n  as seen from a different sensorPose at the vehicle frame origin.\n Otherwise (the default), the output 2D observation will share the\n sensorPose of the input 3D scan\n  (using a more efficient algorithm that avoids trigonometric functions).\n\n  \n The resulting 2D equivalent scan.\n\n \n The example in\n https://www.mrpt.org/tutorials/mrpt-examples/example-kinect-to-2d-laser-demo/\n\nC++: mrpt::obs::CObservation3DRangeScan::convertTo2DScan(class mrpt::obs::CObservation2DRangeScan &, const struct mrpt::obs::T3DPointsTo2DScanParams &, const struct mrpt::obs::TRangeImageFilterParams &) --> void", pybind11::arg("out_scan2d"), pybind11::arg("scanParams"), pybind11::arg("filterParams"));
		cl.def_static("EXTERNALS_AS_TEXT", (void (*)(bool)) &mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT, "Whether external files (3D points, range and confidence) are to be\n saved as `.txt` text files (MATLAB compatible) or `*.bin` binary\n(faster).\n Loading always will determine the type by inspecting the file extension.\n \n\n Default=false\n\nC++: mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT(bool) --> void", pybind11::arg("value"));
		cl.def_static("EXTERNALS_AS_TEXT", (bool (*)()) &mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT, "C++: mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT() --> bool");
		cl.def("resizePoints3DVectors", (void (mrpt::obs::CObservation3DRangeScan::*)(size_t)) &mrpt::obs::CObservation3DRangeScan::resizePoints3DVectors, "Use this method instead of resizing all three  \n &  to allow the usage of the internal memory\n pool. \n\nC++: mrpt::obs::CObservation3DRangeScan::resizePoints3DVectors(size_t) --> void", pybind11::arg("nPoints"));
		cl.def("getScanSize", (size_t (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::getScanSize, "Get the size of the scan pointcloud. \n Method is added for\n compatibility with its CObservation2DRangeScan counterpart \n\nC++: mrpt::obs::CObservation3DRangeScan::getScanSize() const --> size_t");
		cl.def("points3D_isExternallyStored", (bool (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::points3D_isExternallyStored, "@{ \n\nC++: mrpt::obs::CObservation3DRangeScan::points3D_isExternallyStored() const --> bool");
		cl.def("points3D_getExternalStorageFile", (std::string (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::points3D_getExternalStorageFile, "C++: mrpt::obs::CObservation3DRangeScan::points3D_getExternalStorageFile() const --> std::string");
		cl.def("points3D_getExternalStorageFileAbsolutePath", (void (mrpt::obs::CObservation3DRangeScan::*)(std::string &) const) &mrpt::obs::CObservation3DRangeScan::points3D_getExternalStorageFileAbsolutePath, "C++: mrpt::obs::CObservation3DRangeScan::points3D_getExternalStorageFileAbsolutePath(std::string &) const --> void", pybind11::arg("out_path"));
		cl.def("points3D_getExternalStorageFileAbsolutePath", (std::string (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::points3D_getExternalStorageFileAbsolutePath, "C++: mrpt::obs::CObservation3DRangeScan::points3D_getExternalStorageFileAbsolutePath() const --> std::string");
		cl.def("points3D_convertToExternalStorage", (void (mrpt::obs::CObservation3DRangeScan::*)(const std::string &, const std::string &)) &mrpt::obs::CObservation3DRangeScan::points3D_convertToExternalStorage, "Users won't normally want to call this, it's only used from internal\n MRPT programs. \n\n EXTERNALS_AS_TEXT \n\nC++: mrpt::obs::CObservation3DRangeScan::points3D_convertToExternalStorage(const std::string &, const std::string &) --> void", pybind11::arg("fileName"), pybind11::arg("use_this_base_dir"));
		cl.def("points3D_overrideExternalStoredFlag", (void (mrpt::obs::CObservation3DRangeScan::*)(bool)) &mrpt::obs::CObservation3DRangeScan::points3D_overrideExternalStoredFlag, "Users normally won't need to use this \n\nC++: mrpt::obs::CObservation3DRangeScan::points3D_overrideExternalStoredFlag(bool) --> void", pybind11::arg("isExternal"));
		cl.def("rangeImage_setSize", (void (mrpt::obs::CObservation3DRangeScan::*)(const int, const int)) &mrpt::obs::CObservation3DRangeScan::rangeImage_setSize, "Similar to calling \"rangeImage.setSize(H,W)\" but this method provides\n memory pooling to speed-up the memory allocation. \n\nC++: mrpt::obs::CObservation3DRangeScan::rangeImage_setSize(const int, const int) --> void", pybind11::arg("HEIGHT"), pybind11::arg("WIDTH"));
		cl.def("rangeImage_isExternallyStored", (bool (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::rangeImage_isExternallyStored, "@{ \n\nC++: mrpt::obs::CObservation3DRangeScan::rangeImage_isExternallyStored() const --> bool");
		cl.def("rangeImage_getExternalStorageFile", (std::string (mrpt::obs::CObservation3DRangeScan::*)(const std::string &) const) &mrpt::obs::CObservation3DRangeScan::rangeImage_getExternalStorageFile, "C++: mrpt::obs::CObservation3DRangeScan::rangeImage_getExternalStorageFile(const std::string &) const --> std::string", pybind11::arg("rangeImageLayer"));
		cl.def("rangeImage_getExternalStorageFileAbsolutePath", (void (mrpt::obs::CObservation3DRangeScan::*)(std::string &, const std::string &) const) &mrpt::obs::CObservation3DRangeScan::rangeImage_getExternalStorageFileAbsolutePath, "rangeImageLayer: Empty for the main rangeImage matrix, otherwise, a key\n of rangeImageOtherLayers \n\nC++: mrpt::obs::CObservation3DRangeScan::rangeImage_getExternalStorageFileAbsolutePath(std::string &, const std::string &) const --> void", pybind11::arg("out_path"), pybind11::arg("rangeImageLayer"));
		cl.def("rangeImage_getExternalStorageFileAbsolutePath", (std::string (mrpt::obs::CObservation3DRangeScan::*)(const std::string &) const) &mrpt::obs::CObservation3DRangeScan::rangeImage_getExternalStorageFileAbsolutePath, "C++: mrpt::obs::CObservation3DRangeScan::rangeImage_getExternalStorageFileAbsolutePath(const std::string &) const --> std::string", pybind11::arg("rangeImageLayer"));
		cl.def("rangeImage_convertToExternalStorage", (void (mrpt::obs::CObservation3DRangeScan::*)(const std::string &, const std::string &)) &mrpt::obs::CObservation3DRangeScan::rangeImage_convertToExternalStorage, "Users won't normally want to call this, it's only used from internal\n MRPT programs. \n\n EXTERNALS_AS_TEXT \n\nC++: mrpt::obs::CObservation3DRangeScan::rangeImage_convertToExternalStorage(const std::string &, const std::string &) --> void", pybind11::arg("fileName"), pybind11::arg("use_this_base_dir"));
		cl.def("rangeImage_forceResetExternalStorage", (void (mrpt::obs::CObservation3DRangeScan::*)()) &mrpt::obs::CObservation3DRangeScan::rangeImage_forceResetExternalStorage, "Forces marking this observation as non-externally stored - it doesn't\n anything else apart from reseting the corresponding flag (Users won't\n normally want to call this, it's only used from internal MRPT programs)\n\nC++: mrpt::obs::CObservation3DRangeScan::rangeImage_forceResetExternalStorage() --> void");
		cl.def("hasPixelLabels", (bool (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::hasPixelLabels, "@{ \n\n Returns true if the field CObservation3DRangeScan::pixelLabels contains\n a non-NULL smart pointer.\n To enhance a 3D point cloud with labeling info, just assign an\n appropiate object to \n   \n\nC++: mrpt::obs::CObservation3DRangeScan::hasPixelLabels() const --> bool");
		cl.def("doDepthAndIntensityCamerasCoincide", (bool (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::doDepthAndIntensityCamerasCoincide, "Return true if  equals the pure rotation\n (0,0,0,-90deg,0,-90deg) (with a small comparison epsilon)\n \n\n relativePoseIntensityWRTDepth\n\nC++: mrpt::obs::CObservation3DRangeScan::doDepthAndIntensityCamerasCoincide() const --> bool");
		cl.def("getSensorPose", (void (mrpt::obs::CObservation3DRangeScan::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservation3DRangeScan::getSensorPose, "C++: mrpt::obs::CObservation3DRangeScan::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservation3DRangeScan::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservation3DRangeScan::setSensorPose, "C++: mrpt::obs::CObservation3DRangeScan::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("undistort", (void (mrpt::obs::CObservation3DRangeScan::*)()) &mrpt::obs::CObservation3DRangeScan::undistort, "Removes the distortion in both, depth and intensity images. Intrinsics\n (fx,fy,cx,cy) remains the same for each image after undistortion.\n\nC++: mrpt::obs::CObservation3DRangeScan::undistort() --> void");
		cl.def("swap", (void (mrpt::obs::CObservation3DRangeScan::*)(class mrpt::obs::CObservation3DRangeScan &)) &mrpt::obs::CObservation3DRangeScan::swap, "Very efficient method to swap the contents of two observations. \n\nC++: mrpt::obs::CObservation3DRangeScan::swap(class mrpt::obs::CObservation3DRangeScan &) --> void", pybind11::arg("o"));
		cl.def("getZoneAsObs", (void (mrpt::obs::CObservation3DRangeScan::*)(class mrpt::obs::CObservation3DRangeScan &, const unsigned int &, const unsigned int &, const unsigned int &, const unsigned int &)) &mrpt::obs::CObservation3DRangeScan::getZoneAsObs, "Extract a ROI of the 3D observation as a new one. \n PixelLabels are\n *not* copied to the output subimage. \n\nC++: mrpt::obs::CObservation3DRangeScan::getZoneAsObs(class mrpt::obs::CObservation3DRangeScan &, const unsigned int &, const unsigned int &, const unsigned int &, const unsigned int &) --> void", pybind11::arg("obs"), pybind11::arg("r1"), pybind11::arg("r2"), pybind11::arg("c1"), pybind11::arg("c2"));
		cl.def_static("recoverCameraCalibrationParameters", [](const class mrpt::obs::CObservation3DRangeScan & a0, class mrpt::img::TCamera & a1) -> double { return mrpt::obs::CObservation3DRangeScan::recoverCameraCalibrationParameters(a0, a1); }, "", pybind11::arg("in_obs"), pybind11::arg("out_camParams"));
		cl.def_static("recoverCameraCalibrationParameters", (double (*)(const class mrpt::obs::CObservation3DRangeScan &, class mrpt::img::TCamera &, const double)) &mrpt::obs::CObservation3DRangeScan::recoverCameraCalibrationParameters, "A Levenberg-Marquart-based optimizer to recover the calibration\n parameters of a 3D camera given a range (depth) image and the\n corresponding 3D point cloud.\n \n\n The offset (in meters) in the +X direction of the\n point cloud.\n \n\n The final average reprojection error per pixel (typ <0.05 px)\n\nC++: mrpt::obs::CObservation3DRangeScan::recoverCameraCalibrationParameters(const class mrpt::obs::CObservation3DRangeScan &, class mrpt::img::TCamera &, const double) --> double", pybind11::arg("in_obs"), pybind11::arg("out_camParams"), pybind11::arg("camera_offset"));
		cl.def("get_unproj_lut", (const struct mrpt::obs::CObservation3DRangeScan::unproject_LUT_t & (mrpt::obs::CObservation3DRangeScan::*)() const) &mrpt::obs::CObservation3DRangeScan::get_unproj_lut, "Gets (or generates upon first request) the 3D point cloud projection\n look-up-table for the current depth camera intrinsics & distortion\n parameters.\n Returns a const reference to a global variable. Multithread safe.\n \n\n unprojectInto \n\nC++: mrpt::obs::CObservation3DRangeScan::get_unproj_lut() const --> const struct mrpt::obs::CObservation3DRangeScan::unproject_LUT_t &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::obs::CObservation3DRangeScan & (mrpt::obs::CObservation3DRangeScan::*)(const class mrpt::obs::CObservation3DRangeScan &)) &mrpt::obs::CObservation3DRangeScan::operator=, "C++: mrpt::obs::CObservation3DRangeScan::operator=(const class mrpt::obs::CObservation3DRangeScan &) --> class mrpt::obs::CObservation3DRangeScan &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservation3DRangeScan::unproject_LUT_t file:mrpt/obs/CObservation3DRangeScan.h line:557
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservation3DRangeScan::unproject_LUT_t, std::shared_ptr<mrpt::obs::CObservation3DRangeScan::unproject_LUT_t>> cl(enclosing_class, "unproject_LUT_t", "Look-up-table struct for unprojectInto() ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation3DRangeScan::unproject_LUT_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservation3DRangeScan::unproject_LUT_t const &o){ return new mrpt::obs::CObservation3DRangeScan::unproject_LUT_t(o); } ) );
			cl.def_readwrite("Kxs", &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::Kxs);
			cl.def_readwrite("Kys", &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::Kys);
			cl.def_readwrite("Kzs", &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::Kzs);
			cl.def_readwrite("Kxs_rot", &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::Kxs_rot);
			cl.def_readwrite("Kys_rot", &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::Kys_rot);
			cl.def_readwrite("Kzs_rot", &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::Kzs_rot);
			cl.def("assign", (struct mrpt::obs::CObservation3DRangeScan::unproject_LUT_t & (mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::*)(const struct mrpt::obs::CObservation3DRangeScan::unproject_LUT_t &)) &mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::operator=, "C++: mrpt::obs::CObservation3DRangeScan::unproject_LUT_t::operator=(const struct mrpt::obs::CObservation3DRangeScan::unproject_LUT_t &) --> struct mrpt::obs::CObservation3DRangeScan::unproject_LUT_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
