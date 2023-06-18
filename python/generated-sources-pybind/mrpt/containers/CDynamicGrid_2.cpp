#include <iterator>
#include <memory>
#include <mrpt/containers/CDynamicGrid.h>
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
struct PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t : public mrpt::containers::CDynamicGrid<unsigned short> {
	using mrpt::containers::CDynamicGrid<unsigned short>::CDynamicGrid;

	void resize(double a0, double a1, double a2, double a3, const unsigned short & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<unsigned short> *>(this), "resize");
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
	float cell2float(const unsigned short & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<unsigned short> *>(this), "cell2float");
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
struct PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t : public mrpt::containers::CDynamicGrid<signed char> {
	using mrpt::containers::CDynamicGrid<signed char>::CDynamicGrid;

	void resize(double a0, double a1, double a2, double a3, const signed char & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<signed char> *>(this), "resize");
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
	float cell2float(const signed char & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::containers::CDynamicGrid<signed char> *>(this), "cell2float");
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

void bind_mrpt_containers_CDynamicGrid_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
		pybind11::class_<mrpt::containers::CDynamicGrid<unsigned short>, std::shared_ptr<mrpt::containers::CDynamicGrid<unsigned short>>, PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t> cl(M("mrpt::containers"), "CDynamicGrid_unsigned_short_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid<unsigned short>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid<unsigned short>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid<unsigned short>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid<unsigned short>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid<unsigned short>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid_unsigned_short_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid<unsigned short> const &o){ return new mrpt::containers::CDynamicGrid<unsigned short>(o); } ) );
		cl.def("setSize", [](mrpt::containers::CDynamicGrid<unsigned short> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid<unsigned short>::*)(const double, const double, const double, const double, const double, const unsigned short *)) &mrpt::containers::CDynamicGrid<unsigned short>::setSize, "C++: mrpt::containers::CDynamicGrid<unsigned short>::setSize(const double, const double, const double, const double, const double, const unsigned short *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid<unsigned short>::*)()) &mrpt::containers::CDynamicGrid<unsigned short>::clear, "C++: mrpt::containers::CDynamicGrid<unsigned short>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid<unsigned short>::*)(const unsigned short &)) &mrpt::containers::CDynamicGrid<unsigned short>::fill, "C++: mrpt::containers::CDynamicGrid<unsigned short>::fill(const unsigned short &) --> void", pybind11::arg("value"));
		cl.def("resize", [](mrpt::containers::CDynamicGrid<unsigned short> &o, double const & a0, double const & a1, double const & a2, double const & a3, const unsigned short & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid<unsigned short>::*)(double, double, double, double, const unsigned short &, double)) &mrpt::containers::CDynamicGrid<unsigned short>::resize, "C++: mrpt::containers::CDynamicGrid<unsigned short>::resize(double, double, double, double, const unsigned short &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("cellByPos", (unsigned short * (mrpt::containers::CDynamicGrid<unsigned short>::*)(double, double)) &mrpt::containers::CDynamicGrid<unsigned short>::cellByPos, "C++: mrpt::containers::CDynamicGrid<unsigned short>::cellByPos(double, double) --> unsigned short *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cellByIndex", (unsigned short * (mrpt::containers::CDynamicGrid<unsigned short>::*)(unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid<unsigned short>::cellByIndex, "C++: mrpt::containers::CDynamicGrid<unsigned short>::cellByIndex(unsigned int, unsigned int) --> unsigned short *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getSizeX, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getSizeY, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getSizeY() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getXMin, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getXMax, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getYMin, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getYMax, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)() const) &mrpt::containers::CDynamicGrid<unsigned short>::getResolution, "C++: mrpt::containers::CDynamicGrid<unsigned short>::getResolution() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid<unsigned short>::*)(double) const) &mrpt::containers::CDynamicGrid<unsigned short>::x2idx, "C++: mrpt::containers::CDynamicGrid<unsigned short>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid<unsigned short>::*)(double) const) &mrpt::containers::CDynamicGrid<unsigned short>::y2idx, "C++: mrpt::containers::CDynamicGrid<unsigned short>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("xy2idx", (int (mrpt::containers::CDynamicGrid<unsigned short>::*)(double, double) const) &mrpt::containers::CDynamicGrid<unsigned short>::xy2idx, "C++: mrpt::containers::CDynamicGrid<unsigned short>::xy2idx(double, double) const --> int", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("idx2cxcy", (void (mrpt::containers::CDynamicGrid<unsigned short>::*)(int, int &, int &) const) &mrpt::containers::CDynamicGrid<unsigned short>::idx2cxcy, "C++: mrpt::containers::CDynamicGrid<unsigned short>::idx2cxcy(int, int &, int &) const --> void", pybind11::arg("idx"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)(int) const) &mrpt::containers::CDynamicGrid<unsigned short>::idx2x, "C++: mrpt::containers::CDynamicGrid<unsigned short>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid<unsigned short>::*)(int) const) &mrpt::containers::CDynamicGrid<unsigned short>::idx2y, "C++: mrpt::containers::CDynamicGrid<unsigned short>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("cell2float", (float (mrpt::containers::CDynamicGrid<unsigned short>::*)(const unsigned short &) const) &mrpt::containers::CDynamicGrid<unsigned short>::cell2float, "C++: mrpt::containers::CDynamicGrid<unsigned short>::cell2float(const unsigned short &) const --> float", pybind11::arg(""));
		cl.def("saveToTextFile", (bool (mrpt::containers::CDynamicGrid<unsigned short>::*)(const std::string &) const) &mrpt::containers::CDynamicGrid<unsigned short>::saveToTextFile, "C++: mrpt::containers::CDynamicGrid<unsigned short>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid<unsigned short> & (mrpt::containers::CDynamicGrid<unsigned short>::*)(const class mrpt::containers::CDynamicGrid<unsigned short> &)) &mrpt::containers::CDynamicGrid<unsigned short>::operator=, "C++: mrpt::containers::CDynamicGrid<unsigned short>::operator=(const class mrpt::containers::CDynamicGrid<unsigned short> &) --> class mrpt::containers::CDynamicGrid<unsigned short> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::containers::CDynamicGrid file:mrpt/containers/CDynamicGrid.h line:40
		pybind11::class_<mrpt::containers::CDynamicGrid<signed char>, std::shared_ptr<mrpt::containers::CDynamicGrid<signed char>>, PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t> cl(M("mrpt::containers"), "CDynamicGrid_signed_char_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::CDynamicGrid<signed char>(); }, [](){ return new PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::containers::CDynamicGrid<signed char>(a0); }, [](double const & a0){ return new PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::containers::CDynamicGrid<signed char>(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::containers::CDynamicGrid<signed char>(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::containers::CDynamicGrid<signed char>(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t const &o){ return new PyCallBack_mrpt_containers_CDynamicGrid_signed_char_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::containers::CDynamicGrid<signed char> const &o){ return new mrpt::containers::CDynamicGrid<signed char>(o); } ) );
		cl.def("setSize", [](mrpt::containers::CDynamicGrid<signed char> &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::containers::CDynamicGrid<signed char>::*)(const double, const double, const double, const double, const double, const signed char *)) &mrpt::containers::CDynamicGrid<signed char>::setSize, "C++: mrpt::containers::CDynamicGrid<signed char>::setSize(const double, const double, const double, const double, const double, const signed char *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("clear", (void (mrpt::containers::CDynamicGrid<signed char>::*)()) &mrpt::containers::CDynamicGrid<signed char>::clear, "C++: mrpt::containers::CDynamicGrid<signed char>::clear() --> void");
		cl.def("fill", (void (mrpt::containers::CDynamicGrid<signed char>::*)(const signed char &)) &mrpt::containers::CDynamicGrid<signed char>::fill, "C++: mrpt::containers::CDynamicGrid<signed char>::fill(const signed char &) --> void", pybind11::arg("value"));
		cl.def("resize", [](mrpt::containers::CDynamicGrid<signed char> &o, double const & a0, double const & a1, double const & a2, double const & a3, const signed char & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::containers::CDynamicGrid<signed char>::*)(double, double, double, double, const signed char &, double)) &mrpt::containers::CDynamicGrid<signed char>::resize, "C++: mrpt::containers::CDynamicGrid<signed char>::resize(double, double, double, double, const signed char &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("cellByPos", (signed char * (mrpt::containers::CDynamicGrid<signed char>::*)(double, double)) &mrpt::containers::CDynamicGrid<signed char>::cellByPos, "C++: mrpt::containers::CDynamicGrid<signed char>::cellByPos(double, double) --> signed char *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"));
		cl.def("cellByIndex", (signed char * (mrpt::containers::CDynamicGrid<signed char>::*)(unsigned int, unsigned int)) &mrpt::containers::CDynamicGrid<signed char>::cellByIndex, "C++: mrpt::containers::CDynamicGrid<signed char>::cellByIndex(unsigned int, unsigned int) --> signed char *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getSizeX", (size_t (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getSizeX, "C++: mrpt::containers::CDynamicGrid<signed char>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getSizeY, "C++: mrpt::containers::CDynamicGrid<signed char>::getSizeY() const --> size_t");
		cl.def("getXMin", (double (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getXMin, "C++: mrpt::containers::CDynamicGrid<signed char>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getXMax, "C++: mrpt::containers::CDynamicGrid<signed char>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getYMin, "C++: mrpt::containers::CDynamicGrid<signed char>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getYMax, "C++: mrpt::containers::CDynamicGrid<signed char>::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::containers::CDynamicGrid<signed char>::*)() const) &mrpt::containers::CDynamicGrid<signed char>::getResolution, "C++: mrpt::containers::CDynamicGrid<signed char>::getResolution() const --> double");
		cl.def("x2idx", (int (mrpt::containers::CDynamicGrid<signed char>::*)(double) const) &mrpt::containers::CDynamicGrid<signed char>::x2idx, "C++: mrpt::containers::CDynamicGrid<signed char>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::containers::CDynamicGrid<signed char>::*)(double) const) &mrpt::containers::CDynamicGrid<signed char>::y2idx, "C++: mrpt::containers::CDynamicGrid<signed char>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("xy2idx", (int (mrpt::containers::CDynamicGrid<signed char>::*)(double, double) const) &mrpt::containers::CDynamicGrid<signed char>::xy2idx, "C++: mrpt::containers::CDynamicGrid<signed char>::xy2idx(double, double) const --> int", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("idx2cxcy", (void (mrpt::containers::CDynamicGrid<signed char>::*)(int, int &, int &) const) &mrpt::containers::CDynamicGrid<signed char>::idx2cxcy, "C++: mrpt::containers::CDynamicGrid<signed char>::idx2cxcy(int, int &, int &) const --> void", pybind11::arg("idx"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("idx2x", (double (mrpt::containers::CDynamicGrid<signed char>::*)(int) const) &mrpt::containers::CDynamicGrid<signed char>::idx2x, "C++: mrpt::containers::CDynamicGrid<signed char>::idx2x(int) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::containers::CDynamicGrid<signed char>::*)(int) const) &mrpt::containers::CDynamicGrid<signed char>::idx2y, "C++: mrpt::containers::CDynamicGrid<signed char>::idx2y(int) const --> double", pybind11::arg("cy"));
		cl.def("cell2float", (float (mrpt::containers::CDynamicGrid<signed char>::*)(const signed char &) const) &mrpt::containers::CDynamicGrid<signed char>::cell2float, "C++: mrpt::containers::CDynamicGrid<signed char>::cell2float(const signed char &) const --> float", pybind11::arg(""));
		cl.def("saveToTextFile", (bool (mrpt::containers::CDynamicGrid<signed char>::*)(const std::string &) const) &mrpt::containers::CDynamicGrid<signed char>::saveToTextFile, "C++: mrpt::containers::CDynamicGrid<signed char>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("assign", (class mrpt::containers::CDynamicGrid<signed char> & (mrpt::containers::CDynamicGrid<signed char>::*)(const class mrpt::containers::CDynamicGrid<signed char> &)) &mrpt::containers::CDynamicGrid<signed char>::operator=, "C++: mrpt::containers::CDynamicGrid<signed char>::operator=(const class mrpt::containers::CDynamicGrid<signed char> &) --> class mrpt::containers::CDynamicGrid<signed char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
