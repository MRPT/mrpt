#include <iterator>
#include <memory>
#include <mrpt/containers/CDynamicGrid3D.h>
#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <sstream> // __str__
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

// mrpt::containers::CDynamicGrid3D file:mrpt/containers/CDynamicGrid3D.h line:27
struct PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t : public mrpt::containers::CDynamicGrid3D<signed char,double> {
	using mrpt::containers::CDynamicGrid3D<signed char,double>::CDynamicGrid3D;

	void resize(double a0, double a1, double a2, double a3, double a4, double a5, const signed char & a6, double a7) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid3D<signed char,double> *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid3D::resize(a0, a1, a2, a3, a4, a5, a6, a7);
	}
	void setSize(const double a0, const double a1, const double a2, const double a3, const double a4, const double a5, const double a6, const double a7, const signed char * a8) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid3D<signed char,double> *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7, a8);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid3D::setSize(a0, a1, a2, a3, a4, a5, a6, a7, a8);
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid3D<signed char,double> *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid3D::clear();
	}
};

// mrpt::containers::CDynamicGrid3D file:mrpt/containers/CDynamicGrid3D.h line:27
struct PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t : public mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> {
	using mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::CDynamicGrid3D;

	void resize(double a0, double a1, double a2, double a3, double a4, double a5, const struct mrpt::maps::TRandomFieldVoxel & a6, double a7) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid3D::resize(a0, a1, a2, a3, a4, a5, a6, a7);
	}
	void setSize(const double a0, const double a1, const double a2, const double a3, const double a4, const double a5, const double a6, const double a7, const struct mrpt::maps::TRandomFieldVoxel * a8) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6, a7, a8);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid3D::setSize(a0, a1, a2, a3, a4, a5, a6, a7, a8);
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid3D::clear();
	}
};

void bind_mrpt_containers_CDynamicGrid3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::CDynamicGrid3D file:mrpt/containers/CDynamicGrid3D.h line:27
		pybind11::class_<mrpt::containers::CDynamicGrid3D<signed char,double>, std::shared_ptr<mrpt::containers::CDynamicGrid3D<signed char,double>>, PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t> cl(M("mrpt::containers"), "CDynamicGrid3D_signed_char_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0, a1, a2, a3, a4); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0, a1, a2, a3, a4, a5); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(a0, a1, a2, a3, a4, a5, a6); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_signed_char_double_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid3D<signed char,double> const &o){ return new mrpt::containers::CDynamicGrid3D<signed char,double>(o); } ) );
		cl.def("resize", [](mrpt::containers::CDynamicGrid3D<signed char,double> &o, double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, const signed char & a6) -> void { return o.resize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_z_min"), pybind11::arg("new_z_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(double, double, double, double, double, double, const signed char &, double)) &mrpt::containers::CDynamicGrid3D<signed char>::resize, "C++: mrpt::containers::CDynamicGrid3D<signed char>::resize(double, double, double, double, double, double, const signed char &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_z_min"), pybind11::arg("new_z_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("setSize", [](mrpt::containers::CDynamicGrid3D<signed char,double> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4, const double & a5, const double & a6) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"));
		cl.def("setSize", [](mrpt::containers::CDynamicGrid3D<signed char,double> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4, const double & a5, const double & a6, const double & a7) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z_"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(const double, const double, const double, const double, const double, const double, const double, const double, const signed char *)) &mrpt::containers::CDynamicGrid3D<signed char>::setSize, "C++: mrpt::containers::CDynamicGrid3D<signed char>::setSize(const double, const double, const double, const double, const double, const double, const double, const double, const signed char *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z_"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid3D<signed char,double>::*)()) &mrpt::containers::CDynamicGrid3D<signed char>::clear, "C++: mrpt::containers::CDynamicGrid3D<signed char>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(const signed char &)) &mrpt::containers::CDynamicGrid3D<signed char>::fill, "C++: mrpt::containers::CDynamicGrid3D<signed char>::fill(const signed char &) --> void", pybind11::arg("value"));
		cl.def("isOutOfBounds", (bool (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(const int, const int, const int) const) &mrpt::containers::CDynamicGrid3D<signed char>::isOutOfBounds, "C++: mrpt::containers::CDynamicGrid3D<signed char>::isOutOfBounds(const int, const int, const int) const --> bool", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("cellAbsIndexFromCXCYCZ", (size_t (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(const int, const int, const int) const) &mrpt::containers::CDynamicGrid3D<signed char>::cellAbsIndexFromCXCYCZ, "C++: mrpt::containers::CDynamicGrid3D<signed char>::cellAbsIndexFromCXCYCZ(const int, const int, const int) const --> size_t", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("cellByPos", (signed char * (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(double, double, double)) &mrpt::containers::CDynamicGrid3D<signed char>::cellByPos, "C++: mrpt::containers::CDynamicGrid3D<signed char>::cellByPos(double, double, double) --> signed char *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("cellRefByPos", (signed char & (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(double, double, double)) &mrpt::containers::CDynamicGrid3D<signed char>::cellRefByPos, "C++: mrpt::containers::CDynamicGrid3D<signed char>::cellRefByPos(double, double, double) --> signed char &", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("cellByIndex", (signed char * (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(unsigned int, unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid3D<signed char>::cellByIndex, "C++: mrpt::containers::CDynamicGrid3D<signed char>::cellByIndex(unsigned int, unsigned int, unsigned int) --> signed char *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("cellByIndex", (signed char * (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(size_t)) &mrpt::containers::CDynamicGrid3D<signed char>::cellByIndex, "C++: mrpt::containers::CDynamicGrid3D<signed char>::cellByIndex(size_t) --> signed char *", pybind11::return_value_policy::automatic, pybind11::arg("cidx"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getSizeX, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getSizeY, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getSizeY() const --> size_t");
		cl.def("getSizeZ", (size_t (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getSizeZ, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getSizeZ() const --> size_t");
		cl.def("getVoxelCount", (size_t (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getVoxelCount, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getVoxelCount() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getXMin, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getXMax, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getYMin, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getYMax, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getYMax() const --> double");
		cl.def("getZMin", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getZMin, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getZMin() const --> double");
		cl.def("getZMax", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getZMax, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getZMax() const --> double");
		cl.def("getResolutionXY", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getResolutionXY, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getResolutionXY() const --> double");
		cl.def("getResolutionZ", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)() const) &mrpt::containers::CDynamicGrid3D<signed char>::getResolutionZ, "C++: mrpt::containers::CDynamicGrid3D<signed char>::getResolutionZ() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(double) const) &mrpt::containers::CDynamicGrid3D<signed char>::x2idx, "C++: mrpt::containers::CDynamicGrid3D<signed char>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(double) const) &mrpt::containers::CDynamicGrid3D<signed char>::y2idx, "C++: mrpt::containers::CDynamicGrid3D<signed char>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("z2idx", (int (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(double) const) &mrpt::containers::CDynamicGrid3D<signed char>::z2idx, "C++: mrpt::containers::CDynamicGrid3D<signed char>::z2idx(double) const --> int", pybind11::arg("z"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(int) const) &mrpt::containers::CDynamicGrid3D<signed char>::idx2x, "C++: mrpt::containers::CDynamicGrid3D<signed char>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(int) const) &mrpt::containers::CDynamicGrid3D<signed char>::idx2y, "C++: mrpt::containers::CDynamicGrid3D<signed char>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("idx2z", (double (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(int) const) &mrpt::containers::CDynamicGrid3D<signed char>::idx2z, "C++: mrpt::containers::CDynamicGrid3D<signed char>::idx2z(int) const --> double", pybind11::arg("cz"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid3D<signed char> & (mrpt::containers::CDynamicGrid3D<signed char,double>::*)(const class mrpt::containers::CDynamicGrid3D<signed char> &)) &mrpt::containers::CDynamicGrid3D<signed char>::operator=, "C++: mrpt::containers::CDynamicGrid3D<signed char>::operator=(const class mrpt::containers::CDynamicGrid3D<signed char> &) --> class mrpt::containers::CDynamicGrid3D<signed char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::containers::CDynamicGrid3D file:mrpt/containers/CDynamicGrid3D.h line:27
		pybind11::class_<mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>, std::shared_ptr<mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>>, PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t> cl(M("mrpt::containers"), "CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0, a1, a2, a3, a4); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0, a1, a2, a3, a4, a5); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(a0, a1, a2, a3, a4, a5, a6); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid3D_mrpt_maps_TRandomFieldVoxel_double_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> const &o){ return new mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>(o); } ) );
		cl.def("resize", [](mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> &o, double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, const struct mrpt::maps::TRandomFieldVoxel & a6) -> void { return o.resize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_z_min"), pybind11::arg("new_z_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(double, double, double, double, double, double, const struct mrpt::maps::TRandomFieldVoxel &, double)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::resize, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::resize(double, double, double, double, double, double, const struct mrpt::maps::TRandomFieldVoxel &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_z_min"), pybind11::arg("new_z_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("setSize", [](mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4, const double & a5, const double & a6) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"));
		cl.def("setSize", [](mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4, const double & a5, const double & a6, const double & a7) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z_"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(const double, const double, const double, const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldVoxel *)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::setSize, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::setSize(const double, const double, const double, const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldVoxel *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("z_min"), pybind11::arg("z_max"), pybind11::arg("resolution_xy"), pybind11::arg("resolution_z_"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)()) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::clear, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(const struct mrpt::maps::TRandomFieldVoxel &)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::fill, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::fill(const struct mrpt::maps::TRandomFieldVoxel &) --> void", pybind11::arg("value"));
		cl.def("isOutOfBounds", (bool (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(const int, const int, const int) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::isOutOfBounds, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::isOutOfBounds(const int, const int, const int) const --> bool", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("cellAbsIndexFromCXCYCZ", (size_t (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(const int, const int, const int) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellAbsIndexFromCXCYCZ, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellAbsIndexFromCXCYCZ(const int, const int, const int) const --> size_t", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("cellByPos", (struct mrpt::maps::TRandomFieldVoxel * (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(double, double, double)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellByPos, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellByPos(double, double, double) --> struct mrpt::maps::TRandomFieldVoxel *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("cellRefByPos", (struct mrpt::maps::TRandomFieldVoxel & (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(double, double, double)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellRefByPos, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellRefByPos(double, double, double) --> struct mrpt::maps::TRandomFieldVoxel &", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("cellByIndex", (struct mrpt::maps::TRandomFieldVoxel * (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(unsigned int, unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellByIndex, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellByIndex(unsigned int, unsigned int, unsigned int) --> struct mrpt::maps::TRandomFieldVoxel *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("cellByIndex", (struct mrpt::maps::TRandomFieldVoxel * (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(size_t)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellByIndex, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::cellByIndex(size_t) --> struct mrpt::maps::TRandomFieldVoxel *", pybind11::return_value_policy::automatic, pybind11::arg("cidx"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getSizeX, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getSizeY, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getSizeY() const --> size_t");
		cl.def("getSizeZ", (size_t (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getSizeZ, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getSizeZ() const --> size_t");
		cl.def("getVoxelCount", (size_t (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getVoxelCount, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getVoxelCount() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getXMin, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getXMax, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getYMin, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getYMax, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getYMax() const --> double");
		cl.def("getZMin", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getZMin, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getZMin() const --> double");
		cl.def("getZMax", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getZMax, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getZMax() const --> double");
		cl.def("getResolutionXY", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getResolutionXY, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getResolutionXY() const --> double");
		cl.def("getResolutionZ", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)() const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getResolutionZ, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::getResolutionZ() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(double) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::x2idx, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(double) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::y2idx, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("z2idx", (int (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(double) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::z2idx, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::z2idx(double) const --> int", pybind11::arg("z"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(int) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::idx2x, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(int) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::idx2y, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("idx2z", (double (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(int) const) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::idx2z, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::idx2z(int) const --> double", pybind11::arg("cz"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid3D<struct mrpt::maps::TRandomFieldVoxel> & (mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel,double>::*)(const class mrpt::containers::CDynamicGrid3D<struct mrpt::maps::TRandomFieldVoxel> &)) &mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::operator=, "C++: mrpt::containers::CDynamicGrid3D<mrpt::maps::TRandomFieldVoxel>::operator=(const class mrpt::containers::CDynamicGrid3D<struct mrpt::maps::TRandomFieldVoxel> &) --> class mrpt::containers::CDynamicGrid3D<struct mrpt::maps::TRandomFieldVoxel> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
