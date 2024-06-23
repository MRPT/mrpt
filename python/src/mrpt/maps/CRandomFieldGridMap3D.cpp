#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

// mrpt::maps::CRandomFieldGridMap3D file:mrpt/maps/CRandomFieldGridMap3D.h line:71
struct PyCallBack_mrpt_maps_CRandomFieldGridMap3D : public mrpt::maps::CRandomFieldGridMap3D {
	using mrpt::maps::CRandomFieldGridMap3D::CRandomFieldGridMap3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CRandomFieldGridMap3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CRandomFieldGridMap3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CRandomFieldGridMap3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap3D::serializeFrom(a0, a1);
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap3D::clear();
	}
	void resize(double a0, double a1, double a2, double a3, double a4, double a5, const struct mrpt::maps::TRandomFieldVoxel & a6, double a7) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap3D::resize(a0, a1, a2, a3, a4, a5, a6, a7);
	}
	void setSize(const double a0, const double a1, const double a2, const double a3, const double a4, const double a5, const double a6, const double a7, const struct mrpt::maps::TRandomFieldVoxel * a8) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7, a8);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap3D::setSize(a0, a1, a2, a3, a4, a5, a6, a7, a8);
	}
};

// mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions file:mrpt/maps/CRandomFieldGridMap3D.h line:120
struct PyCallBack_mrpt_maps_CRandomFieldGridMap3D_TInsertionOptions : public mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions {
	using mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TInsertionOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor file:mrpt/maps/CRandomFieldGridMap3D.h line:174
struct PyCallBack_mrpt_maps_CRandomFieldGridMap3D_ConnectivityDescriptor : public mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor {
	using mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::ConnectivityDescriptor;

	bool getEdgeInformation(const class mrpt::maps::CRandomFieldGridMap3D * a0, size_t a1, size_t a2, size_t a3, size_t a4, size_t a5, size_t a6, double & a7) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor *>(this), "getEdgeInformation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"ConnectivityDescriptor::getEdgeInformation\"");
	}
};

void bind_mrpt_maps_CRandomFieldGridMap3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::TRandomFieldVoxel file:mrpt/maps/CRandomFieldGridMap3D.h line:30
		pybind11::class_<mrpt::maps::TRandomFieldVoxel, std::shared_ptr<mrpt::maps::TRandomFieldVoxel>> cl(M("mrpt::maps"), "TRandomFieldVoxel", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TRandomFieldVoxel(); } ), "doc" );
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::maps::TRandomFieldVoxel(a0); } ), "doc" , pybind11::arg("_mean_value"));
		cl.def( pybind11::init<double, double>(), pybind11::arg("_mean_value"), pybind11::arg("_stddev_value") );

		cl.def_readwrite("mean_value", &mrpt::maps::TRandomFieldVoxel::mean_value);
		cl.def_readwrite("stddev_value", &mrpt::maps::TRandomFieldVoxel::stddev_value);
	}
	{ // mrpt::maps::CRandomFieldGridMap3D file:mrpt/maps/CRandomFieldGridMap3D.h line:71
		pybind11::class_<mrpt::maps::CRandomFieldGridMap3D, std::shared_ptr<mrpt::maps::CRandomFieldGridMap3D>, PyCallBack_mrpt_maps_CRandomFieldGridMap3D, mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>, mrpt::serialization::CSerializable> cl(M("mrpt::maps"), "CRandomFieldGridMap3D", "CRandomFieldGridMap3D represents a 3D regular grid where each voxel is\n associated one real-valued property which is to be estimated by this class.\n\n  This class implements a Gaussian Markov Random Field (GMRF) estimator, with\n each voxel being connected to its\n   6 immediate neighbors (Up, down, left, right, front, back).\n  - See papers:\n    - \"Time-variant gas distribution mapping with obstacle information\",\n Monroy, J. G., Blanco, J. L., & Gonzalez-Jimenez, J. Autonomous Robots,\n 40(1), 1-16, 2016.\n\n  Note that this class does not derive from mrpt::maps::CMetricMap since the\n estimated values do not have sensor-especific semantics,\n  i.e. the grid can be used to estimate temperature, gas concentration, etc.\n\n  Usage:\n  - Define grid size with either constructor or via `setSize()`.\n  - Initialize the map with `initialize()`. This resets the contents of the\n map, so previously-added observations will be lost.\n  - Add observations of 3D voxels with `insertIndividualReading()`\n\n Custom connectivity patterns can be defined with setVoxelsConnectivity().\n\n \n mrpt::maps::CRandomFieldGridMap3D\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CRandomFieldGridMap3D(); }, [](){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::maps::CRandomFieldGridMap3D(a0); }, [](double const & a0){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::maps::CRandomFieldGridMap3D(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::maps::CRandomFieldGridMap3D(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::maps::CRandomFieldGridMap3D(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::maps::CRandomFieldGridMap3D(a0, a1, a2, a3, a4); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new mrpt::maps::CRandomFieldGridMap3D(a0, a1, a2, a3, a4, a5); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new mrpt::maps::CRandomFieldGridMap3D(a0, a1, a2, a3, a4, a5, a6); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double, double, double, bool>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("voxel_size"), pybind11::arg("call_initialize_now") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CRandomFieldGridMap3D const &o){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CRandomFieldGridMap3D const &o){ return new mrpt::maps::CRandomFieldGridMap3D(o); } ) );

		pybind11::enum_<mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod>(cl, "TVoxelInterpolationMethod", pybind11::arithmetic(), "")
			.value("gimNearest", mrpt::maps::CRandomFieldGridMap3D::gimNearest)
			.value("gimBilinear", mrpt::maps::CRandomFieldGridMap3D::gimBilinear)
			.export_values();

		cl.def_readwrite("insertionOptions", &mrpt::maps::CRandomFieldGridMap3D::insertionOptions);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CRandomFieldGridMap3D::GetRuntimeClassIdStatic, "C++: mrpt::maps::CRandomFieldGridMap3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CRandomFieldGridMap3D::*)() const) &mrpt::maps::CRandomFieldGridMap3D::GetRuntimeClass, "C++: mrpt::maps::CRandomFieldGridMap3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CRandomFieldGridMap3D::*)() const) &mrpt::maps::CRandomFieldGridMap3D::clone, "C++: mrpt::maps::CRandomFieldGridMap3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CRandomFieldGridMap3D::CreateObject, "C++: mrpt::maps::CRandomFieldGridMap3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::maps::CRandomFieldGridMap3D::*)()) &mrpt::maps::CRandomFieldGridMap3D::clear, "Erases all added observations and start again with an empty gridmap. \n\nC++: mrpt::maps::CRandomFieldGridMap3D::clear() --> void");
		cl.def("saveAsCSV", [](mrpt::maps::CRandomFieldGridMap3D const &o, const std::string & a0) -> bool { return o.saveAsCSV(a0); }, "", pybind11::arg("filName_mean"));
		cl.def("saveAsCSV", (bool (mrpt::maps::CRandomFieldGridMap3D::*)(const std::string &, const std::string &) const) &mrpt::maps::CRandomFieldGridMap3D::saveAsCSV, "Save the current estimated mean values to a CSV file (compatible with\n Paraview) with fields `x y z mean_value`.\n Optionally, std deviations can be also saved to another file with fields\n `x y z stddev_value`, if `filName_stddev` is provided.\n \n\n false on error writing to file\n\nC++: mrpt::maps::CRandomFieldGridMap3D::saveAsCSV(const std::string &, const std::string &) const --> bool", pybind11::arg("filName_mean"), pybind11::arg("filName_stddev"));
		cl.def("resize", [](mrpt::maps::CRandomFieldGridMap3D &o, double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, const struct mrpt::maps::TRandomFieldVoxel & a6) -> void { return o.resize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_z_min"), pybind11::arg("new_z_max"), pybind11::arg("defaultValueNewvoxels"));
		cl.def("resize", (void (mrpt::maps::CRandomFieldGridMap3D::*)(double, double, double, double, double, double, const struct mrpt::maps::TRandomFieldVoxel &, double)) &mrpt::maps::CRandomFieldGridMap3D::resize, "Changes the size of the grid, maintaining previous contents. \n setSize\n\nC++: mrpt::maps::CRandomFieldGridMap3D::resize(double, double, double, double, double, double, const struct mrpt::maps::TRandomFieldVoxel &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_z_min"), pybind11::arg("new_z_max"), pybind11::arg("defaultValueNewvoxels"), pybind11::arg("additionalMarginMeters"));
		cl.def("setSize", [](mrpt::maps::CRandomFieldGridMap3D &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4, const double & a5, const double & a6) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"));
		cl.def("setSize", [](mrpt::maps::CRandomFieldGridMap3D &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4, const double & a5, const double & a6, const double & a7) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z"));
		cl.def("setSize", (void (mrpt::maps::CRandomFieldGridMap3D::*)(const double, const double, const double, const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldVoxel *)) &mrpt::maps::CRandomFieldGridMap3D::setSize, "Changes the size of the grid, erasing previous contents.If\n `resolution_z`<0, the same resolution will be used for all dimensions\n x,y,z as given in `resolution_xy` \n\n resize.\n\nC++: mrpt::maps::CRandomFieldGridMap3D::setSize(const double, const double, const double, const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldVoxel *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z"), pybind11::arg("fill_value"));
		cl.def("setVoxelsConnectivity", (void (mrpt::maps::CRandomFieldGridMap3D::*)(const class std::shared_ptr<struct mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor> &)) &mrpt::maps::CRandomFieldGridMap3D::setVoxelsConnectivity, "Sets a custom object to define the connectivity between voxels. Must\n call clear() or setSize() afterwards for the changes to take place. \n\nC++: mrpt::maps::CRandomFieldGridMap3D::setVoxelsConnectivity(const class std::shared_ptr<struct mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor> &) --> void", pybind11::arg("new_connectivity_descriptor"));
		cl.def("insertIndividualReading", (bool (mrpt::maps::CRandomFieldGridMap3D::*)(const double, const double, const struct mrpt::math::TPoint3D_<double> &, const enum mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod, const bool)) &mrpt::maps::CRandomFieldGridMap3D::insertIndividualReading, "Direct update of the map with a reading in a given position of the map.\n \n\n false if point is out of the grid extension.\n\nC++: mrpt::maps::CRandomFieldGridMap3D::insertIndividualReading(const double, const double, const struct mrpt::math::TPoint3D_<double> &, const enum mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod, const bool) --> bool", pybind11::arg("sensorReading"), pybind11::arg("sensorVariance"), pybind11::arg("point"), pybind11::arg("method"), pybind11::arg("update_map"));
		cl.def("updateMapEstimation", (void (mrpt::maps::CRandomFieldGridMap3D::*)()) &mrpt::maps::CRandomFieldGridMap3D::updateMapEstimation, "Run the method-specific procedure required to ensure that the mean &\n variances are up-to-date with all inserted observations, using parameters\n in insertionOptions \n\nC++: mrpt::maps::CRandomFieldGridMap3D::updateMapEstimation() --> void");
		cl.def("assign", (class mrpt::maps::CRandomFieldGridMap3D & (mrpt::maps::CRandomFieldGridMap3D::*)(const class mrpt::maps::CRandomFieldGridMap3D &)) &mrpt::maps::CRandomFieldGridMap3D::operator=, "C++: mrpt::maps::CRandomFieldGridMap3D::operator=(const class mrpt::maps::CRandomFieldGridMap3D &) --> class mrpt::maps::CRandomFieldGridMap3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions file:mrpt/maps/CRandomFieldGridMap3D.h line:120
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions, std::shared_ptr<mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions>, PyCallBack_mrpt_maps_CRandomFieldGridMap3D_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "Parameters common to any derived class.\n  Derived classes should derive a new struct from this one, plus\n mrpt::config::CLoadableOptions,\n  and call the internal_* methods where appropiate to deal with the\n variables declared here.\n  Derived classes instantions of their \"TInsertionOptions\" MUST set the\n pointer \"m_insertOptions_common\" upon construction.");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CRandomFieldGridMap3D_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions const &o){ return new mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions(o); } ) );
			cl.def_readwrite("GMRF_lambdaPrior", &mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::GMRF_lambdaPrior);
			cl.def_readwrite("GMRF_skip_variance", &mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::GMRF_skip_variance);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::loadFromConfigFile, "See mrpt::config::CLoadableOptions \n\nC++: mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions & (mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::*)(const struct mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions &)) &mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::operator=, "C++: mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions::operator=(const struct mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions &) --> struct mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor file:mrpt/maps/CRandomFieldGridMap3D.h line:174
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor, std::shared_ptr<mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor>, PyCallBack_mrpt_maps_CRandomFieldGridMap3D_ConnectivityDescriptor> cl(enclosing_class, "ConnectivityDescriptor", "Base class for user-supplied objects capable of describing voxels\n connectivity, used to build prior factors of the MRF graph. \n\n\n setvoxelsConnectivity() ");
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap3D_ConnectivityDescriptor(); } ) );
			cl.def("getEdgeInformation", (bool (mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::*)(const class mrpt::maps::CRandomFieldGridMap3D *, size_t, size_t, size_t, size_t, size_t, size_t, double &)) &mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::getEdgeInformation, "Implement the check of whether node i=(icx,icy,icz) is connected\n with node j=(jcx,jcy,jcy).\n This visitor method will be called only for immediate neighbors.\n \n\n true if connected (and the \"information\" value should be also\n updated in out_edge_information), false otherwise.\n\nC++: mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::getEdgeInformation(const class mrpt::maps::CRandomFieldGridMap3D *, size_t, size_t, size_t, size_t, size_t, size_t, double &) --> bool", pybind11::arg("parent"), pybind11::arg("icx"), pybind11::arg("icy"), pybind11::arg("icz"), pybind11::arg("jcx"), pybind11::arg("jcy"), pybind11::arg("jcz"), pybind11::arg("out_edge_information"));
			cl.def("assign", (struct mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor & (mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::*)(const struct mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor &)) &mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::operator=, "C++: mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor::operator=(const struct mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor &) --> struct mrpt::maps::CRandomFieldGridMap3D::ConnectivityDescriptor &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
