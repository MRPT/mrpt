#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMetricMapEvents.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <optional>
#include <ostream>
#include <ratio>
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

// mrpt::maps::mrptEventMetricMapClear file:mrpt/maps/CMetricMapEvents.h line:26
struct PyCallBack_mrpt_maps_mrptEventMetricMapClear : public mrpt::maps::mrptEventMetricMapClear {
	using mrpt::maps::mrptEventMetricMapClear::mrptEventMetricMapClear;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::mrptEventMetricMapClear *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventMetricMapClear::do_nothing();
	}
};

// mrpt::maps::mrptEventMetricMapInsert file:mrpt/maps/CMetricMapEvents.h line:42
struct PyCallBack_mrpt_maps_mrptEventMetricMapInsert : public mrpt::maps::mrptEventMetricMapInsert {
	using mrpt::maps::mrptEventMetricMapInsert::mrptEventMetricMapInsert;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::mrptEventMetricMapInsert *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventMetricMapInsert::do_nothing();
	}
};

void bind_mrpt_maps_CMetricMapEvents(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::mrptEventMetricMapClear file:mrpt/maps/CMetricMapEvents.h line:26
		pybind11::class_<mrpt::maps::mrptEventMetricMapClear, std::shared_ptr<mrpt::maps::mrptEventMetricMapClear>, PyCallBack_mrpt_maps_mrptEventMetricMapClear, mrpt::system::mrptEvent> cl(M("mrpt::maps"), "mrptEventMetricMapClear", "Event emitted by a metric up upon call of clear()\n \n\n CMetricMap\n \n\n\n ");
		cl.def( pybind11::init<const class mrpt::maps::CMetricMap *>(), pybind11::arg("smap") );

		cl.def("assign", (class mrpt::maps::mrptEventMetricMapClear & (mrpt::maps::mrptEventMetricMapClear::*)(const class mrpt::maps::mrptEventMetricMapClear &)) &mrpt::maps::mrptEventMetricMapClear::operator=, "C++: mrpt::maps::mrptEventMetricMapClear::operator=(const class mrpt::maps::mrptEventMetricMapClear &) --> class mrpt::maps::mrptEventMetricMapClear &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::mrptEventMetricMapInsert file:mrpt/maps/CMetricMapEvents.h line:42
		pybind11::class_<mrpt::maps::mrptEventMetricMapInsert, std::shared_ptr<mrpt::maps::mrptEventMetricMapInsert>, PyCallBack_mrpt_maps_mrptEventMetricMapInsert, mrpt::system::mrptEvent> cl(M("mrpt::maps"), "mrptEventMetricMapInsert", "Event emitted by a metric up upon a succesful call to insertObservation()\n \n\n CMetricMap\n \n\n\n ");
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_mrptEventMetricMapInsert const &o){ return new PyCallBack_mrpt_maps_mrptEventMetricMapInsert(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::mrptEventMetricMapInsert const &o){ return new mrpt::maps::mrptEventMetricMapInsert(o); } ) );
		cl.def_readonly("inserted_robotPose", &mrpt::maps::mrptEventMetricMapInsert::inserted_robotPose);
	}
}
