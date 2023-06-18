#include <iterator>
#include <memory>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <sstream> // __str__
#include <string>
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

// mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
struct PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t : public mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell> {
	using mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::CDynamicGrid;

	void resize(double a0, double a1, double a2, double a3, const struct mrpt::maps::TRandomFieldCell & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell> *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid::resize(a0, a1, a2, a3, a4, a5);
	}
	float cell2float(const struct mrpt::maps::TRandomFieldCell & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell> *>(this), "cell2float");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CDynamicGrid::cell2float(a0);
	}
};

// mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
struct PyCallBack_mrpt_containers_CDynamicGrid_double_t : public mrpt::containers::CDynamicGrid<double> {
	using mrpt::containers::CDynamicGrid<double>::CDynamicGrid;

	void resize(double a0, double a1, double a2, double a3, const double & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<double> *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid::resize(a0, a1, a2, a3, a4, a5);
	}
	float cell2float(const double & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<double> *>(this), "cell2float");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CDynamicGrid::cell2float(a0);
	}
};

void bind_mrpt_containers_CDynamicGrid(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
		pybind11::class_<mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>, std::shared_ptr<mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>>, PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t> cl(M("mrpt::containers"), "CDynamicGrid_mrpt_maps_TRandomFieldCell_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_TRandomFieldCell_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell> const &o){ return new mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>(o); } ) );
		cl.def("setSize", [](mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldCell *)) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::setSize, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::setSize(const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldCell *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)()) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::clear, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(const struct mrpt::maps::TRandomFieldCell &)) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::fill, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::fill(const struct mrpt::maps::TRandomFieldCell &) --> void", pybind11::arg("value"));
		cl.def("resize", [](mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell> &o, double const & a0, double const & a1, double const & a2, double const & a3, const struct mrpt::maps::TRandomFieldCell & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(double, double, double, double, const struct mrpt::maps::TRandomFieldCell &, double)) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::resize, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::resize(double, double, double, double, const struct mrpt::maps::TRandomFieldCell &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("cellByPos", (struct mrpt::maps::TRandomFieldCell * (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(double, double)) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::cellByPos, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::cellByPos(double, double) --> struct mrpt::maps::TRandomFieldCell *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cellByIndex", (struct mrpt::maps::TRandomFieldCell * (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::cellByIndex, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::cellByIndex(unsigned int, unsigned int) --> struct mrpt::maps::TRandomFieldCell *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getSizeX, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getSizeY, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getSizeY() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getXMin, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getXMax, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getYMin, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getYMax, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getResolution, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::getResolution() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(double) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::x2idx, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(double) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::y2idx, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("xy2idx", (int (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(double, double) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::xy2idx, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::xy2idx(double, double) const --> int", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("idx2cxcy", (void (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(int, int &, int &) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::idx2cxcy, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::idx2cxcy(int, int &, int &) const --> void", pybind11::arg("idx"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(int) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::idx2x, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(int) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::idx2y, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("cell2float", (float (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(const struct mrpt::maps::TRandomFieldCell &) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::cell2float, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::cell2float(const struct mrpt::maps::TRandomFieldCell &) const --> float", pybind11::arg(""));
		cl.def("saveToTextFile", (bool (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(const std::string &) const) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::saveToTextFile, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid<struct mrpt::maps::TRandomFieldCell> & (mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::*)(const class mrpt::containers::CDynamicGrid<struct mrpt::maps::TRandomFieldCell> &)) &mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::operator=, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>::operator=(const class mrpt::containers::CDynamicGrid<struct mrpt::maps::TRandomFieldCell> &) --> class mrpt::containers::CDynamicGrid<struct mrpt::maps::TRandomFieldCell> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
		pybind11::class_<mrpt::containers::CDynamicGrid<double>, std::shared_ptr<mrpt::containers::CDynamicGrid<double>>, PyCallBack_mrpt_containers_CDynamicGrid_double_t> cl(M("mrpt::containers"), "CDynamicGrid_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid<double>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid_double_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid<double>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid_double_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid<double>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid_double_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid<double>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid_double_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid<double>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid_double_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid_double_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid_double_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid<double> const &o){ return new mrpt::containers::CDynamicGrid<double>(o); } ) );
		cl.def("setSize", [](mrpt::containers::CDynamicGrid<double> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid<double>::*)(const double, const double, const double, const double, const double, const double *)) &mrpt::containers::CDynamicGrid<double>::setSize, "C++: mrpt::containers::CDynamicGrid<double>::setSize(const double, const double, const double, const double, const double, const double *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid<double>::*)()) &mrpt::containers::CDynamicGrid<double>::clear, "C++: mrpt::containers::CDynamicGrid<double>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid<double>::*)(const double &)) &mrpt::containers::CDynamicGrid<double>::fill, "C++: mrpt::containers::CDynamicGrid<double>::fill(const double &) --> void", pybind11::arg("value"));
		cl.def("resize", [](mrpt::containers::CDynamicGrid<double> &o, double const & a0, double const & a1, double const & a2, double const & a3, const double & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid<double>::*)(double, double, double, double, const double &, double)) &mrpt::containers::CDynamicGrid<double>::resize, "C++: mrpt::containers::CDynamicGrid<double>::resize(double, double, double, double, const double &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("cellByPos", (double * (mrpt::containers::CDynamicGrid<double>::*)(double, double)) &mrpt::containers::CDynamicGrid<double>::cellByPos, "C++: mrpt::containers::CDynamicGrid<double>::cellByPos(double, double) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cellByIndex", (double * (mrpt::containers::CDynamicGrid<double>::*)(unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid<double>::cellByIndex, "C++: mrpt::containers::CDynamicGrid<double>::cellByIndex(unsigned int, unsigned int) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getSizeX, "C++: mrpt::containers::CDynamicGrid<double>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getSizeY, "C++: mrpt::containers::CDynamicGrid<double>::getSizeY() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getXMin, "C++: mrpt::containers::CDynamicGrid<double>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getXMax, "C++: mrpt::containers::CDynamicGrid<double>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getYMin, "C++: mrpt::containers::CDynamicGrid<double>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getYMax, "C++: mrpt::containers::CDynamicGrid<double>::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::containers::CDynamicGrid<double>::*)() const) &mrpt::containers::CDynamicGrid<double>::getResolution, "C++: mrpt::containers::CDynamicGrid<double>::getResolution() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid<double>::*)(double) const) &mrpt::containers::CDynamicGrid<double>::x2idx, "C++: mrpt::containers::CDynamicGrid<double>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid<double>::*)(double) const) &mrpt::containers::CDynamicGrid<double>::y2idx, "C++: mrpt::containers::CDynamicGrid<double>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("xy2idx", (int (mrpt::containers::CDynamicGrid<double>::*)(double, double) const) &mrpt::containers::CDynamicGrid<double>::xy2idx, "C++: mrpt::containers::CDynamicGrid<double>::xy2idx(double, double) const --> int", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("idx2cxcy", (void (mrpt::containers::CDynamicGrid<double>::*)(int, int &, int &) const) &mrpt::containers::CDynamicGrid<double>::idx2cxcy, "C++: mrpt::containers::CDynamicGrid<double>::idx2cxcy(int, int &, int &) const --> void", pybind11::arg("idx"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid<double>::*)(int) const) &mrpt::containers::CDynamicGrid<double>::idx2x, "C++: mrpt::containers::CDynamicGrid<double>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid<double>::*)(int) const) &mrpt::containers::CDynamicGrid<double>::idx2y, "C++: mrpt::containers::CDynamicGrid<double>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("cell2float", (float (mrpt::containers::CDynamicGrid<double>::*)(const double &) const) &mrpt::containers::CDynamicGrid<double>::cell2float, "C++: mrpt::containers::CDynamicGrid<double>::cell2float(const double &) const --> float", pybind11::arg(""));
		cl.def("saveToTextFile", (bool (mrpt::containers::CDynamicGrid<double>::*)(const std::string &) const) &mrpt::containers::CDynamicGrid<double>::saveToTextFile, "C++: mrpt::containers::CDynamicGrid<double>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid<double> & (mrpt::containers::CDynamicGrid<double>::*)(const class mrpt::containers::CDynamicGrid<double> &)) &mrpt::containers::CDynamicGrid<double>::operator=, "C++: mrpt::containers::CDynamicGrid<double>::operator=(const class mrpt::containers::CDynamicGrid<double> &) --> class mrpt::containers::CDynamicGrid<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
