#include <iterator>
#include <memory>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
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

// mrpt::obs::CSensoryFrame file:mrpt/obs/CSensoryFrame.h line:51
struct PyCallBack_mrpt_obs_CSensoryFrame : public mrpt::obs::CSensoryFrame {
	using mrpt::obs::CSensoryFrame::CSensoryFrame;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CSensoryFrame *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSensoryFrame::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CSensoryFrame *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSensoryFrame::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CSensoryFrame *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSensoryFrame::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CSensoryFrame *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSensoryFrame::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CSensoryFrame *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSensoryFrame::serializeFrom(a0, a1);
	}
};

void bind_mrpt_obs_CSensoryFrame(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CSensoryFrame file:mrpt/obs/CSensoryFrame.h line:51
		pybind11::class_<mrpt::obs::CSensoryFrame, std::shared_ptr<mrpt::obs::CSensoryFrame>, PyCallBack_mrpt_obs_CSensoryFrame, mrpt::serialization::CSerializable> cl(M("mrpt::obs"), "CSensoryFrame", "A \"sensory frame\" is a set of observations taken by the robot\n  approximately at the same time, so they can be considered as a multi-sensor\n  \"snapshot\" of the environment.\n It can contain \"observations\" of many different kinds.\n\n  New observations can be added using:\n\n \n\n\n\n\n\n\n The following methods are equivalent for adding new observations to a\n \"sensory frame\":\n - CSensoryFrame::operator +=\n - CSensoryFrame::push_back\n - CSensoryFrame::insert\n\n To examine the objects within a sensory frame, the following methods exist:\n - CSensoryFrame::getObservationByClass : Looks for some specific observation\n class.\n - CSensoryFrame::begin : To iterate over all observations.\n - CSensoryFrame::getObservationByIndex : To query by index.\n\n Note that `shared_ptr<>`s to the observations are stored, so\n a copy of a CSensoryFrame will contain references to the **same** objects,\n i.e. copies are shallows copies, not deep copies.\n\n \n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CSensoryFrame(); }, [](){ return new PyCallBack_mrpt_obs_CSensoryFrame(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CSensoryFrame const &o){ return new PyCallBack_mrpt_obs_CSensoryFrame(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CSensoryFrame const &o){ return new mrpt::obs::CSensoryFrame(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CSensoryFrame::GetRuntimeClassIdStatic, "C++: mrpt::obs::CSensoryFrame::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CSensoryFrame::*)() const) &mrpt::obs::CSensoryFrame::GetRuntimeClass, "C++: mrpt::obs::CSensoryFrame::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CSensoryFrame::*)() const) &mrpt::obs::CSensoryFrame::clone, "C++: mrpt::obs::CSensoryFrame::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CSensoryFrame::CreateObject, "C++: mrpt::obs::CSensoryFrame::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::obs::CSensoryFrame::*)()) &mrpt::obs::CSensoryFrame::clear, "Clear the container, so it holds no observations. \n\nC++: mrpt::obs::CSensoryFrame::clear() --> void");
		cl.def("__iadd__", (void (mrpt::obs::CSensoryFrame::*)(const class mrpt::obs::CSensoryFrame &)) &mrpt::obs::CSensoryFrame::operator+=, "You can use \"sf1+=sf2;\" to add all observations in sf2 to sf1.\n\nC++: mrpt::obs::CSensoryFrame::operator+=(const class mrpt::obs::CSensoryFrame &) --> void", pybind11::arg("sf"));
		cl.def("__iadd__", (void (mrpt::obs::CSensoryFrame::*)(const class std::shared_ptr<class mrpt::obs::CObservation> &)) &mrpt::obs::CSensoryFrame::operator+=, "You can use \"sf+=obs;\" to add the observation \"obs\" to the \"sf1\".\n\nC++: mrpt::obs::CSensoryFrame::operator+=(const class std::shared_ptr<class mrpt::obs::CObservation> &) --> void", pybind11::arg("obs"));
		cl.def("push_back", (void (mrpt::obs::CSensoryFrame::*)(const class std::shared_ptr<class mrpt::obs::CObservation> &)) &mrpt::obs::CSensoryFrame::push_back, "Insert a new observation to the sensory frame. \n\nC++: mrpt::obs::CSensoryFrame::push_back(const class std::shared_ptr<class mrpt::obs::CObservation> &) --> void", pybind11::arg("obs"));
		cl.def("insert", (void (mrpt::obs::CSensoryFrame::*)(const class std::shared_ptr<class mrpt::obs::CObservation> &)) &mrpt::obs::CSensoryFrame::insert, "Synonym with push_back()\n\nC++: mrpt::obs::CSensoryFrame::insert(const class std::shared_ptr<class mrpt::obs::CObservation> &) --> void", pybind11::arg("obs"));
		cl.def("size", (size_t (mrpt::obs::CSensoryFrame::*)() const) &mrpt::obs::CSensoryFrame::size, "Returns the number of observations in the list. \n\nC++: mrpt::obs::CSensoryFrame::size() const --> size_t");
		cl.def("empty", (bool (mrpt::obs::CSensoryFrame::*)() const) &mrpt::obs::CSensoryFrame::empty, "Returns true if there are no observations in the list. \n\nC++: mrpt::obs::CSensoryFrame::empty() const --> bool");
		cl.def("eraseByIndex", (void (mrpt::obs::CSensoryFrame::*)(size_t)) &mrpt::obs::CSensoryFrame::eraseByIndex, "Removes the i'th observation in the list (0=first). \n\nC++: mrpt::obs::CSensoryFrame::eraseByIndex(size_t) --> void", pybind11::arg("idx"));
		cl.def("eraseByLabel", (void (mrpt::obs::CSensoryFrame::*)(const std::string &)) &mrpt::obs::CSensoryFrame::eraseByLabel, "Removes all the observations that match a given sensorLabel.\n\nC++: mrpt::obs::CSensoryFrame::eraseByLabel(const std::string &) --> void", pybind11::arg("label"));
		cl.def("getObservationByIndex", (class std::shared_ptr<class mrpt::obs::CObservation> & (mrpt::obs::CSensoryFrame::*)(size_t)) &mrpt::obs::CSensoryFrame::getObservationByIndex, "C++: mrpt::obs::CSensoryFrame::getObservationByIndex(size_t) --> class std::shared_ptr<class mrpt::obs::CObservation> &", pybind11::return_value_policy::automatic, pybind11::arg("idx"));
		cl.def("getObservationBySensorLabel", [](mrpt::obs::CSensoryFrame const &o, const std::string & a0) -> std::shared_ptr<class mrpt::obs::CObservation> { return o.getObservationBySensorLabel(a0); }, "", pybind11::arg("label"));
		cl.def("getObservationBySensorLabel", (class std::shared_ptr<class mrpt::obs::CObservation> (mrpt::obs::CSensoryFrame::*)(const std::string &, size_t) const) &mrpt::obs::CSensoryFrame::getObservationBySensorLabel, "Returns the i'th observation in the list with the given \"sensorLabel\"\n (0=first).\n \n\n The observation, or nullptr if not found.\n \n\n begin, size\n\nC++: mrpt::obs::CSensoryFrame::getObservationBySensorLabel(const std::string &, size_t) const --> class std::shared_ptr<class mrpt::obs::CObservation>", pybind11::arg("label"), pybind11::arg("idx"));
		cl.def("swap", (void (mrpt::obs::CSensoryFrame::*)(class mrpt::obs::CSensoryFrame &)) &mrpt::obs::CSensoryFrame::swap, "Efficiently swaps the contents of two objects.\n\nC++: mrpt::obs::CSensoryFrame::swap(class mrpt::obs::CSensoryFrame &) --> void", pybind11::arg("sf"));
		cl.def("assign", (class mrpt::obs::CSensoryFrame & (mrpt::obs::CSensoryFrame::*)(const class mrpt::obs::CSensoryFrame &)) &mrpt::obs::CSensoryFrame::operator=, "C++: mrpt::obs::CSensoryFrame::operator=(const class mrpt::obs::CSensoryFrame &) --> class mrpt::obs::CSensoryFrame &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
