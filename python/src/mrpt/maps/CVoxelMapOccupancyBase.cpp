#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CVoxelMapOccupancyBase.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <sstream> // __str__
#include <string>
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

// mrpt::maps::TVoxelMap_InsertionOptions file:mrpt/maps/CVoxelMapOccupancyBase.h line:24
struct PyCallBack_mrpt_maps_TVoxelMap_InsertionOptions : public mrpt::maps::TVoxelMap_InsertionOptions {
	using mrpt::maps::TVoxelMap_InsertionOptions::TVoxelMap_InsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TVoxelMap_InsertionOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TVoxelMap_InsertionOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TVoxelMap_InsertionOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TVoxelMap_InsertionOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::TVoxelMap_LikelihoodOptions file:mrpt/maps/CVoxelMapOccupancyBase.h line:54
struct PyCallBack_mrpt_maps_TVoxelMap_LikelihoodOptions : public mrpt::maps::TVoxelMap_LikelihoodOptions {
	using mrpt::maps::TVoxelMap_LikelihoodOptions::TVoxelMap_LikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TVoxelMap_LikelihoodOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TVoxelMap_LikelihoodOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TVoxelMap_LikelihoodOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TVoxelMap_LikelihoodOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_maps_CVoxelMapOccupancyBase(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::TVoxelMap_InsertionOptions file:mrpt/maps/CVoxelMapOccupancyBase.h line:24
		pybind11::class_<mrpt::maps::TVoxelMap_InsertionOptions, std::shared_ptr<mrpt::maps::TVoxelMap_InsertionOptions>, PyCallBack_mrpt_maps_TVoxelMap_InsertionOptions, mrpt::config::CLoadableOptions> cl(M("mrpt::maps"), "TVoxelMap_InsertionOptions", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TVoxelMap_InsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_TVoxelMap_InsertionOptions(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_TVoxelMap_InsertionOptions const &o){ return new PyCallBack_mrpt_maps_TVoxelMap_InsertionOptions(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::TVoxelMap_InsertionOptions const &o){ return new mrpt::maps::TVoxelMap_InsertionOptions(o); } ) );
		cl.def_readwrite("max_range", &mrpt::maps::TVoxelMap_InsertionOptions::max_range);
		cl.def_readwrite("prob_miss", &mrpt::maps::TVoxelMap_InsertionOptions::prob_miss);
		cl.def_readwrite("prob_hit", &mrpt::maps::TVoxelMap_InsertionOptions::prob_hit);
		cl.def_readwrite("clamp_min", &mrpt::maps::TVoxelMap_InsertionOptions::clamp_min);
		cl.def_readwrite("clamp_max", &mrpt::maps::TVoxelMap_InsertionOptions::clamp_max);
		cl.def_readwrite("ray_trace_free_space", &mrpt::maps::TVoxelMap_InsertionOptions::ray_trace_free_space);
		cl.def_readwrite("decimation", &mrpt::maps::TVoxelMap_InsertionOptions::decimation);
		cl.def_readwrite("remove_voxels_farther_than", &mrpt::maps::TVoxelMap_InsertionOptions::remove_voxels_farther_than);
		cl.def("loadFromConfigFile", (void (mrpt::maps::TVoxelMap_InsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::TVoxelMap_InsertionOptions::loadFromConfigFile, "C++: mrpt::maps::TVoxelMap_InsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("saveToConfigFile", (void (mrpt::maps::TVoxelMap_InsertionOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::TVoxelMap_InsertionOptions::saveToConfigFile, "C++: mrpt::maps::TVoxelMap_InsertionOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
		cl.def("writeToStream", (void (mrpt::maps::TVoxelMap_InsertionOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::TVoxelMap_InsertionOptions::writeToStream, "C++: mrpt::maps::TVoxelMap_InsertionOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def("readFromStream", (void (mrpt::maps::TVoxelMap_InsertionOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::TVoxelMap_InsertionOptions::readFromStream, "C++: mrpt::maps::TVoxelMap_InsertionOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
		cl.def("assign", (struct mrpt::maps::TVoxelMap_InsertionOptions & (mrpt::maps::TVoxelMap_InsertionOptions::*)(const struct mrpt::maps::TVoxelMap_InsertionOptions &)) &mrpt::maps::TVoxelMap_InsertionOptions::operator=, "C++: mrpt::maps::TVoxelMap_InsertionOptions::operator=(const struct mrpt::maps::TVoxelMap_InsertionOptions &) --> struct mrpt::maps::TVoxelMap_InsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::TVoxelMap_LikelihoodOptions file:mrpt/maps/CVoxelMapOccupancyBase.h line:54
		pybind11::class_<mrpt::maps::TVoxelMap_LikelihoodOptions, std::shared_ptr<mrpt::maps::TVoxelMap_LikelihoodOptions>, PyCallBack_mrpt_maps_TVoxelMap_LikelihoodOptions, mrpt::config::CLoadableOptions> cl(M("mrpt::maps"), "TVoxelMap_LikelihoodOptions", "Options used when evaluating \"computeObservationLikelihood\"\n \n\n CObservation::computeObservationLikelihood");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TVoxelMap_LikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_TVoxelMap_LikelihoodOptions(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_TVoxelMap_LikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_TVoxelMap_LikelihoodOptions(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::TVoxelMap_LikelihoodOptions const &o){ return new mrpt::maps::TVoxelMap_LikelihoodOptions(o); } ) );
		cl.def_readwrite("decimate_up_to", &mrpt::maps::TVoxelMap_LikelihoodOptions::decimate_up_to);
		cl.def_readwrite("occupiedThreshold", &mrpt::maps::TVoxelMap_LikelihoodOptions::occupiedThreshold);
		cl.def("loadFromConfigFile", (void (mrpt::maps::TVoxelMap_LikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::TVoxelMap_LikelihoodOptions::loadFromConfigFile, "C++: mrpt::maps::TVoxelMap_LikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("saveToConfigFile", (void (mrpt::maps::TVoxelMap_LikelihoodOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::TVoxelMap_LikelihoodOptions::saveToConfigFile, "C++: mrpt::maps::TVoxelMap_LikelihoodOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
		cl.def("writeToStream", (void (mrpt::maps::TVoxelMap_LikelihoodOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::TVoxelMap_LikelihoodOptions::writeToStream, "C++: mrpt::maps::TVoxelMap_LikelihoodOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def("readFromStream", (void (mrpt::maps::TVoxelMap_LikelihoodOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::TVoxelMap_LikelihoodOptions::readFromStream, "C++: mrpt::maps::TVoxelMap_LikelihoodOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
		cl.def("assign", (struct mrpt::maps::TVoxelMap_LikelihoodOptions & (mrpt::maps::TVoxelMap_LikelihoodOptions::*)(const struct mrpt::maps::TVoxelMap_LikelihoodOptions &)) &mrpt::maps::TVoxelMap_LikelihoodOptions::operator=, "C++: mrpt::maps::TVoxelMap_LikelihoodOptions::operator=(const struct mrpt::maps::TVoxelMap_LikelihoodOptions &) --> struct mrpt::maps::TVoxelMap_LikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::TVoxelMap_RenderingOptions file:mrpt/maps/CVoxelMapOccupancyBase.h line:78
		pybind11::class_<mrpt::maps::TVoxelMap_RenderingOptions, std::shared_ptr<mrpt::maps::TVoxelMap_RenderingOptions>> cl(M("mrpt::maps"), "TVoxelMap_RenderingOptions", "Options for the conversion of a mrpt::maps::COctoMap into a\n mrpt::opengl::COctoMapVoxels ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TVoxelMap_RenderingOptions(); } ) );
		cl.def( pybind11::init( [](mrpt::maps::TVoxelMap_RenderingOptions const &o){ return new mrpt::maps::TVoxelMap_RenderingOptions(o); } ) );
		cl.def_readwrite("generateOccupiedVoxels", &mrpt::maps::TVoxelMap_RenderingOptions::generateOccupiedVoxels);
		cl.def_readwrite("occupiedThreshold", &mrpt::maps::TVoxelMap_RenderingOptions::occupiedThreshold);
		cl.def_readwrite("visibleOccupiedVoxels", &mrpt::maps::TVoxelMap_RenderingOptions::visibleOccupiedVoxels);
		cl.def_readwrite("generateFreeVoxels", &mrpt::maps::TVoxelMap_RenderingOptions::generateFreeVoxels);
		cl.def_readwrite("freeThreshold", &mrpt::maps::TVoxelMap_RenderingOptions::freeThreshold);
		cl.def_readwrite("visibleFreeVoxels", &mrpt::maps::TVoxelMap_RenderingOptions::visibleFreeVoxels);
		cl.def("writeToStream", (void (mrpt::maps::TVoxelMap_RenderingOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::TVoxelMap_RenderingOptions::writeToStream, "Binary dump to stream \n\nC++: mrpt::maps::TVoxelMap_RenderingOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def("readFromStream", (void (mrpt::maps::TVoxelMap_RenderingOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::TVoxelMap_RenderingOptions::readFromStream, "Binary dump to stream \n\nC++: mrpt::maps::TVoxelMap_RenderingOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
		cl.def("assign", (struct mrpt::maps::TVoxelMap_RenderingOptions & (mrpt::maps::TVoxelMap_RenderingOptions::*)(const struct mrpt::maps::TVoxelMap_RenderingOptions &)) &mrpt::maps::TVoxelMap_RenderingOptions::operator=, "C++: mrpt::maps::TVoxelMap_RenderingOptions::operator=(const struct mrpt::maps::TVoxelMap_RenderingOptions &) --> struct mrpt::maps::TVoxelMap_RenderingOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
