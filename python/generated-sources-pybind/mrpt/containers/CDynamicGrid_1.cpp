#include <iterator>
#include <memory>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CHeightGridMap2D.h>
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
struct PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t : public mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell> {
	using mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::CDynamicGrid;

	void resize(double a0, double a1, double a2, double a3, const struct mrpt::maps::THeightGridmapCell & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell> *>(this), "resize");
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
	float cell2float(const struct mrpt::maps::THeightGridmapCell & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell> *>(this), "cell2float");
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
struct PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t : public mrpt::containers::CDynamicGrid<unsigned char> {
	using mrpt::containers::CDynamicGrid<unsigned char>::CDynamicGrid;

	void resize(double a0, double a1, double a2, double a3, const unsigned char & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<unsigned char> *>(this), "resize");
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
	float cell2float(const unsigned char & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<unsigned char> *>(this), "cell2float");
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

void bind_mrpt_containers_CDynamicGrid_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
		pybind11::class_<mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>, std::shared_ptr<mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>>, PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t> cl(M("mrpt::containers"), "CDynamicGrid_mrpt_maps_THeightGridmapCell_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid_mrpt_maps_THeightGridmapCell_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell> const &o){ return new mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>(o); } ) );
		cl.def("setSize", [](mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(const double, const double, const double, const double, const double, const struct mrpt::maps::THeightGridmapCell *)) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::setSize, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::setSize(const double, const double, const double, const double, const double, const struct mrpt::maps::THeightGridmapCell *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)()) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::clear, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(const struct mrpt::maps::THeightGridmapCell &)) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::fill, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::fill(const struct mrpt::maps::THeightGridmapCell &) --> void", pybind11::arg("value"));
		cl.def("resize", [](mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell> &o, double const & a0, double const & a1, double const & a2, double const & a3, const struct mrpt::maps::THeightGridmapCell & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(double, double, double, double, const struct mrpt::maps::THeightGridmapCell &, double)) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::resize, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::resize(double, double, double, double, const struct mrpt::maps::THeightGridmapCell &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("cellByPos", (struct mrpt::maps::THeightGridmapCell * (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(double, double)) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::cellByPos, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::cellByPos(double, double) --> struct mrpt::maps::THeightGridmapCell *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cellByIndex", (struct mrpt::maps::THeightGridmapCell * (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::cellByIndex, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::cellByIndex(unsigned int, unsigned int) --> struct mrpt::maps::THeightGridmapCell *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getSizeX, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getSizeY, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getSizeY() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getXMin, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getXMax, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getYMin, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getYMax, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)() const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getResolution, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::getResolution() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(double) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::x2idx, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(double) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::y2idx, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("xy2idx", (int (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(double, double) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::xy2idx, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::xy2idx(double, double) const --> int", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("idx2cxcy", (void (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(int, int &, int &) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::idx2cxcy, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::idx2cxcy(int, int &, int &) const --> void", pybind11::arg("idx"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(int) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::idx2x, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(int) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::idx2y, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("cell2float", (float (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(const struct mrpt::maps::THeightGridmapCell &) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::cell2float, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::cell2float(const struct mrpt::maps::THeightGridmapCell &) const --> float", pybind11::arg(""));
		cl.def("saveToTextFile", (bool (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(const std::string &) const) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::saveToTextFile, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid<struct mrpt::maps::THeightGridmapCell> & (mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::*)(const class mrpt::containers::CDynamicGrid<struct mrpt::maps::THeightGridmapCell> &)) &mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::operator=, "C++: mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>::operator=(const class mrpt::containers::CDynamicGrid<struct mrpt::maps::THeightGridmapCell> &) --> class mrpt::containers::CDynamicGrid<struct mrpt::maps::THeightGridmapCell> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
		pybind11::class_<mrpt::containers::CDynamicGrid<unsigned char>, std::shared_ptr<mrpt::containers::CDynamicGrid<unsigned char>>, PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t> cl(M("mrpt::containers"), "CDynamicGrid_unsigned_char_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid<unsigned char>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid<unsigned char>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid<unsigned char>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid<unsigned char>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid<unsigned char>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_char_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid<unsigned char> const &o){ return new mrpt::containers::CDynamicGrid<unsigned char>(o); } ) );
		cl.def("setSize", [](mrpt::containers::CDynamicGrid<unsigned char> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid<unsigned char>::*)(const double, const double, const double, const double, const double, const unsigned char *)) &mrpt::containers::CDynamicGrid<unsigned char>::setSize, "C++: mrpt::containers::CDynamicGrid<unsigned char>::setSize(const double, const double, const double, const double, const double, const unsigned char *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid<unsigned char>::*)()) &mrpt::containers::CDynamicGrid<unsigned char>::clear, "C++: mrpt::containers::CDynamicGrid<unsigned char>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid<unsigned char>::*)(const unsigned char &)) &mrpt::containers::CDynamicGrid<unsigned char>::fill, "C++: mrpt::containers::CDynamicGrid<unsigned char>::fill(const unsigned char &) --> void", pybind11::arg("value"));
		cl.def("resize", [](mrpt::containers::CDynamicGrid<unsigned char> &o, double const & a0, double const & a1, double const & a2, double const & a3, const unsigned char & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid<unsigned char>::*)(double, double, double, double, const unsigned char &, double)) &mrpt::containers::CDynamicGrid<unsigned char>::resize, "C++: mrpt::containers::CDynamicGrid<unsigned char>::resize(double, double, double, double, const unsigned char &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("cellByPos", (unsigned char * (mrpt::containers::CDynamicGrid<unsigned char>::*)(double, double)) &mrpt::containers::CDynamicGrid<unsigned char>::cellByPos, "C++: mrpt::containers::CDynamicGrid<unsigned char>::cellByPos(double, double) --> unsigned char *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cellByIndex", (unsigned char * (mrpt::containers::CDynamicGrid<unsigned char>::*)(unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid<unsigned char>::cellByIndex, "C++: mrpt::containers::CDynamicGrid<unsigned char>::cellByIndex(unsigned int, unsigned int) --> unsigned char *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getSizeX, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getSizeY, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getSizeY() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getXMin, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getXMax, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getYMin, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getYMax, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)() const) &mrpt::containers::CDynamicGrid<unsigned char>::getResolution, "C++: mrpt::containers::CDynamicGrid<unsigned char>::getResolution() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid<unsigned char>::*)(double) const) &mrpt::containers::CDynamicGrid<unsigned char>::x2idx, "C++: mrpt::containers::CDynamicGrid<unsigned char>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid<unsigned char>::*)(double) const) &mrpt::containers::CDynamicGrid<unsigned char>::y2idx, "C++: mrpt::containers::CDynamicGrid<unsigned char>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("xy2idx", (int (mrpt::containers::CDynamicGrid<unsigned char>::*)(double, double) const) &mrpt::containers::CDynamicGrid<unsigned char>::xy2idx, "C++: mrpt::containers::CDynamicGrid<unsigned char>::xy2idx(double, double) const --> int", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("idx2cxcy", (void (mrpt::containers::CDynamicGrid<unsigned char>::*)(int, int &, int &) const) &mrpt::containers::CDynamicGrid<unsigned char>::idx2cxcy, "C++: mrpt::containers::CDynamicGrid<unsigned char>::idx2cxcy(int, int &, int &) const --> void", pybind11::arg("idx"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)(int) const) &mrpt::containers::CDynamicGrid<unsigned char>::idx2x, "C++: mrpt::containers::CDynamicGrid<unsigned char>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid<unsigned char>::*)(int) const) &mrpt::containers::CDynamicGrid<unsigned char>::idx2y, "C++: mrpt::containers::CDynamicGrid<unsigned char>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("cell2float", (float (mrpt::containers::CDynamicGrid<unsigned char>::*)(const unsigned char &) const) &mrpt::containers::CDynamicGrid<unsigned char>::cell2float, "C++: mrpt::containers::CDynamicGrid<unsigned char>::cell2float(const unsigned char &) const --> float", pybind11::arg(""));
		cl.def("saveToTextFile", (bool (mrpt::containers::CDynamicGrid<unsigned char>::*)(const std::string &) const) &mrpt::containers::CDynamicGrid<unsigned char>::saveToTextFile, "C++: mrpt::containers::CDynamicGrid<unsigned char>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid<unsigned char> & (mrpt::containers::CDynamicGrid<unsigned char>::*)(const class mrpt::containers::CDynamicGrid<unsigned char> &)) &mrpt::containers::CDynamicGrid<unsigned char>::operator=, "C++: mrpt::containers::CDynamicGrid<unsigned char>::operator=(const class mrpt::containers::CDynamicGrid<unsigned char> &) --> class mrpt::containers::CDynamicGrid<unsigned char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
