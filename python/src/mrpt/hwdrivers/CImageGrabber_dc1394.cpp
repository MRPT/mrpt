#include <memory>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <string>

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

void bind_mrpt_hwdrivers_CImageGrabber_dc1394(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::TCaptureOptions_dc1394 file:mrpt/hwdrivers/CImageGrabber_dc1394.h line:47
		pybind11::class_<mrpt::hwdrivers::TCaptureOptions_dc1394, std::shared_ptr<mrpt::hwdrivers::TCaptureOptions_dc1394>> cl(M("mrpt::hwdrivers"), "TCaptureOptions_dc1394", "Options used when creating an dc1394 capture object\n   All but the frame size, framerate, and color_coding can be changed\n dynamically by CImageGrabber_dc1394::changeCaptureOptions\n \n\n CImageGrabber_dc1394\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TCaptureOptions_dc1394(); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::TCaptureOptions_dc1394 const &o){ return new mrpt::hwdrivers::TCaptureOptions_dc1394(o); } ) );
		cl.def_readwrite("frame_width", &mrpt::hwdrivers::TCaptureOptions_dc1394::frame_width);
		cl.def_readwrite("frame_height", &mrpt::hwdrivers::TCaptureOptions_dc1394::frame_height);
		cl.def_readwrite("framerate", &mrpt::hwdrivers::TCaptureOptions_dc1394::framerate);
		cl.def_readwrite("color_coding", &mrpt::hwdrivers::TCaptureOptions_dc1394::color_coding);
		cl.def_readwrite("mode7", &mrpt::hwdrivers::TCaptureOptions_dc1394::mode7);
		cl.def_readwrite("shutter", &mrpt::hwdrivers::TCaptureOptions_dc1394::shutter);
		cl.def_readwrite("gain", &mrpt::hwdrivers::TCaptureOptions_dc1394::gain);
		cl.def_readwrite("gamma", &mrpt::hwdrivers::TCaptureOptions_dc1394::gamma);
		cl.def_readwrite("brightness", &mrpt::hwdrivers::TCaptureOptions_dc1394::brightness);
		cl.def_readwrite("exposure", &mrpt::hwdrivers::TCaptureOptions_dc1394::exposure);
		cl.def_readwrite("sharpness", &mrpt::hwdrivers::TCaptureOptions_dc1394::sharpness);
		cl.def_readwrite("white_balance", &mrpt::hwdrivers::TCaptureOptions_dc1394::white_balance);
		cl.def_readwrite("shutter_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::shutter_mode);
		cl.def_readwrite("gain_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::gain_mode);
		cl.def_readwrite("gamma_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::gamma_mode);
		cl.def_readwrite("brightness_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::brightness_mode);
		cl.def_readwrite("exposure_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::exposure_mode);
		cl.def_readwrite("sharpness_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::sharpness_mode);
		cl.def_readwrite("white_balance_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::white_balance_mode);
		cl.def_readwrite("deinterlace_stereo", &mrpt::hwdrivers::TCaptureOptions_dc1394::deinterlace_stereo);
		cl.def_readwrite("trigger_power", &mrpt::hwdrivers::TCaptureOptions_dc1394::trigger_power);
		cl.def_readwrite("trigger_mode", &mrpt::hwdrivers::TCaptureOptions_dc1394::trigger_mode);
		cl.def_readwrite("trigger_source", &mrpt::hwdrivers::TCaptureOptions_dc1394::trigger_source);
		cl.def_readwrite("trigger_polarity", &mrpt::hwdrivers::TCaptureOptions_dc1394::trigger_polarity);
		cl.def_readwrite("ring_buffer_size", &mrpt::hwdrivers::TCaptureOptions_dc1394::ring_buffer_size);
		cl.def("assign", (struct mrpt::hwdrivers::TCaptureOptions_dc1394 & (mrpt::hwdrivers::TCaptureOptions_dc1394::*)(const struct mrpt::hwdrivers::TCaptureOptions_dc1394 &)) &mrpt::hwdrivers::TCaptureOptions_dc1394::operator=, "C++: mrpt::hwdrivers::TCaptureOptions_dc1394::operator=(const struct mrpt::hwdrivers::TCaptureOptions_dc1394 &) --> struct mrpt::hwdrivers::TCaptureOptions_dc1394 &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::CImageGrabber_dc1394 file:mrpt/hwdrivers/CImageGrabber_dc1394.h line:125
		pybind11::class_<mrpt::hwdrivers::CImageGrabber_dc1394, std::shared_ptr<mrpt::hwdrivers::CImageGrabber_dc1394>> cl(M("mrpt::hwdrivers"), "CImageGrabber_dc1394", "A class for grabing images from a IEEE1394 (Firewire) camera using the\n libdc1394-2 library.\n   See the constructor for the options when opening the camera. Notice that\n you may have\n    to carefully set the resolution, framerate and color_mode. See the\n verbose parameter of\n    the constructor, which can display a list of supported modes in your\n camera.\n\n  This class is able to manage any Firewire cameras, including Stereo or\n multi-cameras in general,\n    so this can be used to open the Bumblebee camera (not tested yet).\n\n A static method (CImageGrabber_dc1394::enumerateCameras) is provided to\n enumerate all existing cameras and their properties. It can be used\n  to find the GUID of the desired camera, then open it at the constructor.\n\n \n This class requires MRPT compiled with \"libdc1394-2\" (Only works under\n Linux for now) and \"opencv\".\n \n\n In Linux you may need to execute \"chmod 666 /dev/video1394/ * \" and\n \"chmod 666 /dev/raw1394\" for allowing any user R/W access to firewire\n cameras.\n \n\n [New in MRPT 1.3.0] Length of ring buffer is now configurable via\n TCaptureOptions_dc1394::ring_buffer_size\n \n\n The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CImageGrabber_dc1394(); } ), "doc" );
		cl.def( pybind11::init( [](uint64_t const & a0){ return new mrpt::hwdrivers::CImageGrabber_dc1394(a0); } ), "doc" , pybind11::arg("cameraGUID"));
		cl.def( pybind11::init( [](uint64_t const & a0, uint16_t const & a1){ return new mrpt::hwdrivers::CImageGrabber_dc1394(a0, a1); } ), "doc" , pybind11::arg("cameraGUID"), pybind11::arg("cameraUnit"));
		cl.def( pybind11::init( [](uint64_t const & a0, uint16_t const & a1, const struct mrpt::hwdrivers::TCaptureOptions_dc1394 & a2){ return new mrpt::hwdrivers::CImageGrabber_dc1394(a0, a1, a2); } ), "doc" , pybind11::arg("cameraGUID"), pybind11::arg("cameraUnit"), pybind11::arg("options"));
		cl.def( pybind11::init<uint64_t, uint16_t, const struct mrpt::hwdrivers::TCaptureOptions_dc1394 &, bool>(), pybind11::arg("cameraGUID"), pybind11::arg("cameraUnit"), pybind11::arg("options"), pybind11::arg("verbose") );

		cl.def("isOpen", (bool (mrpt::hwdrivers::CImageGrabber_dc1394::*)() const) &mrpt::hwdrivers::CImageGrabber_dc1394::isOpen, "Check whether the camera has been open successfully. \n\nC++: mrpt::hwdrivers::CImageGrabber_dc1394::isOpen() const --> bool");
		cl.def("changeCaptureOptions", (bool (mrpt::hwdrivers::CImageGrabber_dc1394::*)(const struct mrpt::hwdrivers::TCaptureOptions_dc1394 &)) &mrpt::hwdrivers::CImageGrabber_dc1394::changeCaptureOptions, "Changes the capture properties (brightness, gain, shutter, etc)\n The frame size, framerate, and color_coding fields in options are\n ignored since they can be only set at construction time.\n \n\n false on error\n\nC++: mrpt::hwdrivers::CImageGrabber_dc1394::changeCaptureOptions(const struct mrpt::hwdrivers::TCaptureOptions_dc1394 &) --> bool", pybind11::arg("options"));
		cl.def("getObservation", (bool (mrpt::hwdrivers::CImageGrabber_dc1394::*)(class mrpt::obs::CObservationImage &)) &mrpt::hwdrivers::CImageGrabber_dc1394::getObservation, "Grab an image from the opened camera (for monocular cameras).\n \n\n The object to be filled with sensed data.\n \n\n This may be blocking when using software trigger and no frame is\n available yet. Ensure trigger before getObservation() or take into\n account that this call may block.\n \n\n false on any error, true if all go fine.\n\nC++: mrpt::hwdrivers::CImageGrabber_dc1394::getObservation(class mrpt::obs::CObservationImage &) --> bool", pybind11::arg("out_observation"));
		cl.def("getObservation", (bool (mrpt::hwdrivers::CImageGrabber_dc1394::*)(class mrpt::obs::CObservationStereoImages &)) &mrpt::hwdrivers::CImageGrabber_dc1394::getObservation, "Grab an image from the opened camera (for stereo cameras).\n \n\n The object to be filled with sensed data.\n\n \n false on any error, true if all go fine.\n\nC++: mrpt::hwdrivers::CImageGrabber_dc1394::getObservation(class mrpt::obs::CObservationStereoImages &) --> bool", pybind11::arg("out_observation"));
		cl.def("setSoftwareTriggerLevel", (bool (mrpt::hwdrivers::CImageGrabber_dc1394::*)(bool)) &mrpt::hwdrivers::CImageGrabber_dc1394::setSoftwareTriggerLevel, "Changes the boolean level associated to Software Trigger (ON/OFF)\n Can be used to control camera triggering trough software\n \n\n false on error\n\nC++: mrpt::hwdrivers::CImageGrabber_dc1394::setSoftwareTriggerLevel(bool) --> bool", pybind11::arg("level"));
		cl.def("assign", (class mrpt::hwdrivers::CImageGrabber_dc1394 & (mrpt::hwdrivers::CImageGrabber_dc1394::*)(const class mrpt::hwdrivers::CImageGrabber_dc1394 &)) &mrpt::hwdrivers::CImageGrabber_dc1394::operator=, "C++: mrpt::hwdrivers::CImageGrabber_dc1394::operator=(const class mrpt::hwdrivers::CImageGrabber_dc1394 &) --> class mrpt::hwdrivers::CImageGrabber_dc1394 &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo file:mrpt/hwdrivers/CImageGrabber_dc1394.h line:193
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo, std::shared_ptr<mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo>> cl(enclosing_class, "TCameraInfo", "Used in enumerateCameras ");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo const &o){ return new mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo(o); } ) );
			cl.def_readwrite("guid", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::guid);
			cl.def_readwrite("unit", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::unit);
			cl.def_readwrite("unit_spec_ID", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::unit_spec_ID);
			cl.def_readwrite("unit_sw_version", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::unit_sw_version);
			cl.def_readwrite("unit_sub_sw_version", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::unit_sub_sw_version);
			cl.def_readwrite("command_registers_base", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::command_registers_base);
			cl.def_readwrite("unit_directory", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::unit_directory);
			cl.def_readwrite("unit_dependent_directory", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::unit_dependent_directory);
			cl.def_readwrite("advanced_features_csr", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::advanced_features_csr);
			cl.def_readwrite("PIO_control_csr", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::PIO_control_csr);
			cl.def_readwrite("SIO_control_csr", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::SIO_control_csr);
			cl.def_readwrite("strobe_control_csr", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::strobe_control_csr);
			cl.def_readwrite("iidc_version", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::iidc_version);
			cl.def_readwrite("vendor", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::vendor);
			cl.def_readwrite("model", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::model);
			cl.def_readwrite("vendor_id", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::vendor_id);
			cl.def_readwrite("model_id", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::model_id);
			cl.def_readwrite("bmode_capable", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::bmode_capable);
			cl.def_readwrite("one_shot_capable", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::one_shot_capable);
			cl.def_readwrite("multi_shot_capable", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::multi_shot_capable);
			cl.def_readwrite("can_switch_on_off", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::can_switch_on_off);
			cl.def_readwrite("has_vmode_error_status", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::has_vmode_error_status);
			cl.def_readwrite("has_feature_error_status", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::has_feature_error_status);
			cl.def_readwrite("max_mem_channel", &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::max_mem_channel);
			cl.def("assign", (struct mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo & (mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::*)(const struct mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo &)) &mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::operator=, "C++: mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo::operator=(const struct mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo &) --> struct mrpt::hwdrivers::CImageGrabber_dc1394::TCameraInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
