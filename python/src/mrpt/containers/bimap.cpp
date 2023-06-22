#include <functional>
#include <map>
#include <memory>
#include <mrpt/containers/bimap.h>
#include <sstream> // __str__
#include <utility>

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

void bind_mrpt_containers_bimap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::bimap file:mrpt/containers/bimap.h line:42
		pybind11::class_<mrpt::containers::bimap<long,unsigned int>, std::shared_ptr<mrpt::containers::bimap<long,unsigned int>>> cl(M("mrpt::containers"), "bimap_long_unsigned_int_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::containers::bimap<long,unsigned int>(); } ) );
		cl.def( pybind11::init( [](mrpt::containers::bimap<long,unsigned int> const &o){ return new mrpt::containers::bimap<long,unsigned int>(o); } ) );
		cl.def("size", (size_t (mrpt::containers::bimap<long,unsigned int>::*)() const) &mrpt::containers::bimap<long, unsigned int>::size, "C++: mrpt::containers::bimap<long, unsigned int>::size() const --> size_t");
		cl.def("empty", (bool (mrpt::containers::bimap<long,unsigned int>::*)() const) &mrpt::containers::bimap<long, unsigned int>::empty, "C++: mrpt::containers::bimap<long, unsigned int>::empty() const --> bool");
		cl.def("clear", (void (mrpt::containers::bimap<long,unsigned int>::*)()) &mrpt::containers::bimap<long, unsigned int>::clear, "C++: mrpt::containers::bimap<long, unsigned int>::clear() --> void");
		cl.def("insert", (void (mrpt::containers::bimap<long,unsigned int>::*)(const long &, const unsigned int &)) &mrpt::containers::bimap<long, unsigned int>::insert, "C++: mrpt::containers::bimap<long, unsigned int>::insert(const long &, const unsigned int &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("direct", (bool (mrpt::containers::bimap<long,unsigned int>::*)(const long &, unsigned int &) const) &mrpt::containers::bimap<long, unsigned int>::direct, "C++: mrpt::containers::bimap<long, unsigned int>::direct(const long &, unsigned int &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("hasKey", (bool (mrpt::containers::bimap<long,unsigned int>::*)(const long &) const) &mrpt::containers::bimap<long, unsigned int>::hasKey, "C++: mrpt::containers::bimap<long, unsigned int>::hasKey(const long &) const --> bool", pybind11::arg("k"));
		cl.def("hasValue", (bool (mrpt::containers::bimap<long,unsigned int>::*)(const unsigned int &) const) &mrpt::containers::bimap<long, unsigned int>::hasValue, "C++: mrpt::containers::bimap<long, unsigned int>::hasValue(const unsigned int &) const --> bool", pybind11::arg("v"));
		cl.def("direct", (unsigned int (mrpt::containers::bimap<long,unsigned int>::*)(const long &) const) &mrpt::containers::bimap<long, unsigned int>::direct, "C++: mrpt::containers::bimap<long, unsigned int>::direct(const long &) const --> unsigned int", pybind11::arg("k"));
		cl.def("inverse", (bool (mrpt::containers::bimap<long,unsigned int>::*)(const unsigned int &, long &) const) &mrpt::containers::bimap<long, unsigned int>::inverse, "C++: mrpt::containers::bimap<long, unsigned int>::inverse(const unsigned int &, long &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("inverse", (long (mrpt::containers::bimap<long,unsigned int>::*)(const unsigned int &) const) &mrpt::containers::bimap<long, unsigned int>::inverse, "C++: mrpt::containers::bimap<long, unsigned int>::inverse(const unsigned int &) const --> long", pybind11::arg("v"));
		cl.def("erase_by_key", (void (mrpt::containers::bimap<long,unsigned int>::*)(const long &)) &mrpt::containers::bimap<long, unsigned int>::erase_by_key, "C++: mrpt::containers::bimap<long, unsigned int>::erase_by_key(const long &) --> void", pybind11::arg("k"));
		cl.def("erase_by_value", (void (mrpt::containers::bimap<long,unsigned int>::*)(const unsigned int &)) &mrpt::containers::bimap<long, unsigned int>::erase_by_value, "C++: mrpt::containers::bimap<long, unsigned int>::erase_by_value(const unsigned int &) --> void", pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::bimap<long, unsigned int> & (mrpt::containers::bimap<long,unsigned int>::*)(const class mrpt::containers::bimap<long, unsigned int> &)) &mrpt::containers::bimap<long, unsigned int>::operator=, "C++: mrpt::containers::bimap<long, unsigned int>::operator=(const class mrpt::containers::bimap<long, unsigned int> &) --> class mrpt::containers::bimap<long, unsigned int> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
