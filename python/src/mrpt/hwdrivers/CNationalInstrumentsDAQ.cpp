#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CNationalInstrumentsDAQ.h>
#include <mrpt/obs/CObservationRawDAQ.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <utility>
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

// mrpt::hwdrivers::CNationalInstrumentsDAQ file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:209
struct PyCallBack_mrpt_hwdrivers_CNationalInstrumentsDAQ : public mrpt::hwdrivers::CNationalInstrumentsDAQ {
	using mrpt::hwdrivers::CNationalInstrumentsDAQ::CNationalInstrumentsDAQ;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CNationalInstrumentsDAQ::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNationalInstrumentsDAQ::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNationalInstrumentsDAQ::doProcess();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNationalInstrumentsDAQ::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "loadConfig");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::loadConfig(a0, a1);
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "getObservations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CGenericSensor::getObservations();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setPathForExternalImages(a0);
	}
	void setExternalImageFormat(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "setExternalImageFormat");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageFormat(a0);
	}
	void setExternalImageJPEGQuality(const unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "setExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageJPEGQuality(a0);
	}
	unsigned int getExternalImageJPEGQuality() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNationalInstrumentsDAQ *>(this), "getExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		return CGenericSensor::getExternalImageJPEGQuality();
	}
};

void bind_mrpt_hwdrivers_CNationalInstrumentsDAQ(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CNationalInstrumentsDAQ file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:209
		pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ>, PyCallBack_mrpt_hwdrivers_CNationalInstrumentsDAQ, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CNationalInstrumentsDAQ", "An interface to read from data acquisition boards compatible with National\n Instruments \"DAQmx Base\" or \"DAQmx\".\n Refer to DAQmx Base C API reference online to learn more on the concepts of\n \"channels\", \"tasks\" (which in this MRPT class\n  are mapped to independent grabbing threads), etc.\n If both DAQmx and DAQmxBase are installed in the system, DAQmx will be used.\n This class API isolate the user from the usage of one or another specific\n library.\n\n  This class can be used as a sensor from the application \"rawlog-grabber\", or\n directly as a C++ class from a user program.\n  Refer to the example:  [MRPT]/samples/NIDAQ_test\n\n  Samples will be returned inside mrpt::obs::CObservationRawDAQ in \"packets\"\n of a predefined number of samples, which can be changed by the user through\n the \"samplesPerChannelToRead\" parameter of each task.\n\n  For multichannels tasks, samples will be **interleaved**. For example, the\n readings from successive timesteps for 4 ADC channels\n  will be available in the ADC vector inside mrpt::obs::CObservationRawDAQ in\n this order:\n\n   - A0[0] A1[0] A2[0] A3[0]  A0[1] A1[1] A2[1] A3[1]  A0[2] A1[2] A2[2] A3[2]\n ...\n\n  The sensor label (field \"m_sensorLabel\") of each grabbed observation will be\n the concatenation of this class sensor label,\n  a dot (\".\") and the task label (default=\"task###\", with ### the task index).\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n See also:\n  - [MRPT]/samples/NIDAQ_test\n  - Sample .ini files for rawlog-grabber in\n [MRPT]/share/mrpt/config_files/rawlog-grabber/\n  - NI DAQmx C reference:\n http://others-help.mrpt.org/ni-daqmx_c_reference_help/\n  - NI DAQmx Base 3.x C reference:\n http://others-help.mrpt.org/ni-daqmx_base_3.x_c_function_reference/\n\n DAQmx Base Installation\n ------------------------\n Go to http://ni.com and download the \"DAQmx Base\" package for your OS.\n Install following NI's instructions. As of 2013, the latest version is 3.7.\n\n \n This class requires compiling MRPT with support for \"NI DAQmx\" or \"NI\n DAQmx Base\". While compiling MRPT,\n        check the \"MRPT_HAS_NI_DAQmx\"/\"MRPT_HAS_NI_DAQmxBASE\" option and\n correctly set the new variables to\n        the library include directory and library file.\n\n \n As of 2013, NI seems not to support compiling 64bit programs, so you\n can must build MRPT for 32bits if you need this class.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CNationalInstrumentsDAQ(); } ) );
		cl.def_readwrite("task_definitions", &mrpt::hwdrivers::CNationalInstrumentsDAQ::task_definitions);
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)() const) &mrpt::hwdrivers::CNationalInstrumentsDAQ::GetRuntimeClass, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CNationalInstrumentsDAQ::CreateObject, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CNationalInstrumentsDAQ::doRegister, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::doRegister() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)()) &mrpt::hwdrivers::CNationalInstrumentsDAQ::initialize, "Setup and launch the DAQ tasks, in parallel threads.\n Access to grabbed data with CNationalInstrumentsDAQ::readFromDAQ() or\n the standard CGenericSensor::doProcess() \n\nC++: mrpt::hwdrivers::CNationalInstrumentsDAQ::initialize() --> void");
		cl.def("stop", (void (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)()) &mrpt::hwdrivers::CNationalInstrumentsDAQ::stop, "Stop the grabbing threads for DAQ tasks. It is automatically called at\n destruction. \n\nC++: mrpt::hwdrivers::CNationalInstrumentsDAQ::stop() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)()) &mrpt::hwdrivers::CNationalInstrumentsDAQ::doProcess, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::doProcess() --> void");
		cl.def("writeAnalogOutputTask", (void (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)(size_t, size_t, const double *, double, bool)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::writeAnalogOutputTask, "Set voltage outputs to all the outputs in an AOUT task\n For the meaning of parameters, refere to NI DAQmx docs for\n DAQmxBaseWriteAnalogF64()\n \n\n The number of samples in  must match the number of\n channels in the task when it was initiated.\n\nC++: mrpt::hwdrivers::CNationalInstrumentsDAQ::writeAnalogOutputTask(size_t, size_t, const double *, double, bool) --> void", pybind11::arg("task_index"), pybind11::arg("nSamplesPerChannel"), pybind11::arg("volt_values"), pybind11::arg("timeout"), pybind11::arg("groupedByChannel"));
		cl.def("writeDigitalOutputTask", (void (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)(size_t, bool, double)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::writeDigitalOutputTask, "Changes the boolean state of one digital output line.\n For the meaning of parameters, refere to NI DAQmx docs for\n DAQmxBaseWriteAnalogF64()\n \n\n The number of samples in  must match the number of\n channels in the task when it was initiated.\n\nC++: mrpt::hwdrivers::CNationalInstrumentsDAQ::writeDigitalOutputTask(size_t, bool, double) --> void", pybind11::arg("task_index"), pybind11::arg("line_value"), pybind11::arg("timeout"));
		cl.def("checkDAQIsWorking", (bool (mrpt::hwdrivers::CNationalInstrumentsDAQ::*)() const) &mrpt::hwdrivers::CNationalInstrumentsDAQ::checkDAQIsWorking, "Returns true if initialize() was called and at least one task is\n running. \n\nC++: mrpt::hwdrivers::CNationalInstrumentsDAQ::checkDAQIsWorking() const --> bool");

		{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:274
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription>> cl(enclosing_class, "TaskDescription", "Each of the tasks to create in CNationalInstrumentsDAQ::initialize().\n Refer to the docs on config file formats of\n mrpt::hwdrivers::CNationalInstrumentsDAQ to learn on the meaning\n of each field. Also, see National Instruments' DAQmx API docs online.");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription(); } ) );
			cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription(o); } ) );
			cl.def_readwrite("has_ai", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ai);
			cl.def_readwrite("has_ao", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ao);
			cl.def_readwrite("has_di", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_di);
			cl.def_readwrite("has_do", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_do);
			cl.def_readwrite("has_ci_period", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ci_period);
			cl.def_readwrite("has_ci_count_edges", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ci_count_edges);
			cl.def_readwrite("has_ci_pulse_width", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ci_pulse_width);
			cl.def_readwrite("has_ci_lin_encoder", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ci_lin_encoder);
			cl.def_readwrite("has_ci_ang_encoder", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_ci_ang_encoder);
			cl.def_readwrite("has_co_pulses", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::has_co_pulses);
			cl.def_readwrite("samplesPerSecond", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::samplesPerSecond);
			cl.def_readwrite("sampleClkSource", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::sampleClkSource);
			cl.def_readwrite("bufferSamplesPerChannel", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::bufferSamplesPerChannel);
			cl.def_readwrite("samplesPerChannelToRead", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::samplesPerChannelToRead);
			cl.def_readwrite("taskLabel", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::taskLabel);
			cl.def_readwrite("ai", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ai);
			cl.def_readwrite("ao", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ao);
			cl.def_readwrite("di", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::di);
			cl.def_readwrite("douts", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::douts);
			cl.def_readwrite("ci_period", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ci_period);
			cl.def_readwrite("ci_count_edges", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ci_count_edges);
			cl.def_readwrite("ci_pulse_width", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ci_pulse_width);
			cl.def_readwrite("ci_lin_encoder", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ci_lin_encoder);
			cl.def_readwrite("ci_ang_encoder", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::ci_ang_encoder);
			cl.def_readwrite("co_pulses", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::co_pulses);
			cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:300
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t>> cl(enclosing_class, "desc_ai_t", "Analog inputs ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t(o); } ) );
				cl.def_readwrite("physicalChannel", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::physicalChannel);
				cl.def_readwrite("terminalConfig", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::terminalConfig);
				cl.def_readwrite("minVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::minVal);
				cl.def_readwrite("maxVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::maxVal);
				cl.def_readwrite("physicalChannelCount", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::physicalChannelCount);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ai_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:312
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t>> cl(enclosing_class, "desc_ao_t", "Analog outputs ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t(o); } ) );
				cl.def_readwrite("physicalChannel", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::physicalChannel);
				cl.def_readwrite("physicalChannelCount", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::physicalChannelCount);
				cl.def_readwrite("minVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::minVal);
				cl.def_readwrite("maxVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::maxVal);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ao_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:323
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t>> cl(enclosing_class, "desc_di_t", "Digital inputs (di) ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t(o); } ) );
				cl.def_readwrite("line", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t::line);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_di_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:330
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t>> cl(enclosing_class, "desc_do_t", "Digital outs (do) ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t(o); } ) );
				cl.def_readwrite("line", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t::line);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_do_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:336
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t>> cl(enclosing_class, "desc_ci_period_t", "");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t(o); } ) );
				cl.def_readwrite("counter", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::counter);
				cl.def_readwrite("units", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::units);
				cl.def_readwrite("edge", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::edge);
				cl.def_readwrite("minVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::minVal);
				cl.def_readwrite("maxVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::maxVal);
				cl.def_readwrite("measTime", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::measTime);
				cl.def_readwrite("divisor", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::divisor);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_period_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:349
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t>> cl(enclosing_class, "desc_ci_count_edges_t", "Counter: period of a digital signal ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t(o); } ) );
				cl.def_readwrite("counter", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::counter);
				cl.def_readwrite("edge", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::edge);
				cl.def_readwrite("countDirection", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::countDirection);
				cl.def_readwrite("initialCount", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::initialCount);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_count_edges_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:358
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t>> cl(enclosing_class, "desc_ci_pulse_width_t", "Counter: measure the width of a digital pulse ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t(o); } ) );
				cl.def_readwrite("counter", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::counter);
				cl.def_readwrite("units", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::units);
				cl.def_readwrite("startingEdge", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::startingEdge);
				cl.def_readwrite("minVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::minVal);
				cl.def_readwrite("maxVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::maxVal);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_pulse_width_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:366
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t>> cl(enclosing_class, "desc_ci_lin_encoder_t", "Counter: uses a linear encoder to measure linear position ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t(o); } ) );
				cl.def_readwrite("counter", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::counter);
				cl.def_readwrite("decodingType", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::decodingType);
				cl.def_readwrite("ZidxPhase", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::ZidxPhase);
				cl.def_readwrite("units", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::units);
				cl.def_readwrite("ZidxEnable", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::ZidxEnable);
				cl.def_readwrite("ZidxVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::ZidxVal);
				cl.def_readwrite("distPerPulse", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::distPerPulse);
				cl.def_readwrite("initialPos", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::initialPos);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_lin_encoder_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:378
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t>> cl(enclosing_class, "desc_ci_ang_encoder_t", "Counter: uses an angular encoder to measure angular position ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t(o); } ) );
				cl.def_readwrite("counter", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::counter);
				cl.def_readwrite("decodingType", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::decodingType);
				cl.def_readwrite("ZidxPhase", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::ZidxPhase);
				cl.def_readwrite("units", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::units);
				cl.def_readwrite("ZidxEnable", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::ZidxEnable);
				cl.def_readwrite("ZidxVal", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::ZidxVal);
				cl.def_readwrite("pulsesPerRev", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::pulsesPerRev);
				cl.def_readwrite("initialAngle", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::initialAngle);
				cl.def_readwrite("decimate", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::decimate);
				cl.def_readwrite("decimate_cnt", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::decimate_cnt);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_ci_ang_encoder_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t file:mrpt/hwdrivers/CNationalInstrumentsDAQ.h line:391
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t, std::shared_ptr<mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t>> cl(enclosing_class, "desc_co_pulses_t", "Output counter: digital pulses output ");
				cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t(); } ) );
				cl.def( pybind11::init( [](mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t const &o){ return new mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t(o); } ) );
				cl.def_readwrite("counter", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::counter);
				cl.def_readwrite("idleState", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::idleState);
				cl.def_readwrite("initialDelay", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::initialDelay);
				cl.def_readwrite("freq", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::freq);
				cl.def_readwrite("dutyCycle", &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::dutyCycle);
				cl.def("assign", (struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t & (mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::*)(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t &)) &mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::operator=, "C++: mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t::operator=(const struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t &) --> struct mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription::desc_co_pulses_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

		}

	}
}
