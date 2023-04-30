#include <mrpt/containers/circular_buffer.h>
#include <sstream> // __str__

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_containers_circular_buffer(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::circular_buffer file:mrpt/containers/circular_buffer.h line:22
		pybind11::class_<mrpt::containers::circular_buffer<unsigned char>, std::shared_ptr<mrpt::containers::circular_buffer<unsigned char>>> cl(M("mrpt::containers"), "circular_buffer_unsigned_char_t", "");
		cl.def( pybind11::init<const unsigned long>(), pybind11::arg("size") );

		cl.def( pybind11::init( [](mrpt::containers::circular_buffer<unsigned char> const &o){ return new mrpt::containers::circular_buffer<unsigned char>(o); } ) );
		cl.def("push", (void (mrpt::containers::circular_buffer<unsigned char>::*)(unsigned char)) &mrpt::containers::circular_buffer<unsigned char>::push, "C++: mrpt::containers::circular_buffer<unsigned char>::push(unsigned char) --> void", pybind11::arg("d"));
		cl.def("push_ref", (void (mrpt::containers::circular_buffer<unsigned char>::*)(const unsigned char &)) &mrpt::containers::circular_buffer<unsigned char>::push_ref, "C++: mrpt::containers::circular_buffer<unsigned char>::push_ref(const unsigned char &) --> void", pybind11::arg("d"));
		cl.def("push_many", (void (mrpt::containers::circular_buffer<unsigned char>::*)(unsigned char *, unsigned long)) &mrpt::containers::circular_buffer<unsigned char>::push_many, "C++: mrpt::containers::circular_buffer<unsigned char>::push_many(unsigned char *, unsigned long) --> void", pybind11::arg("array_elements"), pybind11::arg("count"));
		cl.def("pop", (unsigned char (mrpt::containers::circular_buffer<unsigned char>::*)()) &mrpt::containers::circular_buffer<unsigned char>::pop, "C++: mrpt::containers::circular_buffer<unsigned char>::pop() --> unsigned char");
		cl.def("pop", (void (mrpt::containers::circular_buffer<unsigned char>::*)(unsigned char &)) &mrpt::containers::circular_buffer<unsigned char>::pop, "C++: mrpt::containers::circular_buffer<unsigned char>::pop(unsigned char &) --> void", pybind11::arg("out_val"));
		cl.def("pop_many", (void (mrpt::containers::circular_buffer<unsigned char>::*)(unsigned char *, unsigned long)) &mrpt::containers::circular_buffer<unsigned char>::pop_many, "C++: mrpt::containers::circular_buffer<unsigned char>::pop_many(unsigned char *, unsigned long) --> void", pybind11::arg("out_array"), pybind11::arg("count"));
		cl.def("peek", (unsigned char (mrpt::containers::circular_buffer<unsigned char>::*)() const) &mrpt::containers::circular_buffer<unsigned char>::peek, "C++: mrpt::containers::circular_buffer<unsigned char>::peek() const --> unsigned char");
		cl.def("peek", (unsigned char (mrpt::containers::circular_buffer<unsigned char>::*)(unsigned long) const) &mrpt::containers::circular_buffer<unsigned char>::peek, "C++: mrpt::containers::circular_buffer<unsigned char>::peek(unsigned long) const --> unsigned char", pybind11::arg("index"));
		cl.def("peek_many", (void (mrpt::containers::circular_buffer<unsigned char>::*)(unsigned char *, unsigned long) const) &mrpt::containers::circular_buffer<unsigned char>::peek_many, "C++: mrpt::containers::circular_buffer<unsigned char>::peek_many(unsigned char *, unsigned long) const --> void", pybind11::arg("out_array"), pybind11::arg("count"));
		cl.def("size", (unsigned long (mrpt::containers::circular_buffer<unsigned char>::*)() const) &mrpt::containers::circular_buffer<unsigned char>::size, "C++: mrpt::containers::circular_buffer<unsigned char>::size() const --> unsigned long");
		cl.def("capacity", (unsigned long (mrpt::containers::circular_buffer<unsigned char>::*)() const) &mrpt::containers::circular_buffer<unsigned char>::capacity, "C++: mrpt::containers::circular_buffer<unsigned char>::capacity() const --> unsigned long");
		cl.def("available", (unsigned long (mrpt::containers::circular_buffer<unsigned char>::*)() const) &mrpt::containers::circular_buffer<unsigned char>::available, "C++: mrpt::containers::circular_buffer<unsigned char>::available() const --> unsigned long");
		cl.def("clear", (void (mrpt::containers::circular_buffer<unsigned char>::*)()) &mrpt::containers::circular_buffer<unsigned char>::clear, "C++: mrpt::containers::circular_buffer<unsigned char>::clear() --> void");
		cl.def("assign", (class mrpt::containers::circular_buffer<unsigned char> & (mrpt::containers::circular_buffer<unsigned char>::*)(const class mrpt::containers::circular_buffer<unsigned char> &)) &mrpt::containers::circular_buffer<unsigned char>::operator=, "C++: mrpt::containers::circular_buffer<unsigned char>::operator=(const class mrpt::containers::circular_buffer<unsigned char> &) --> class mrpt::containers::circular_buffer<unsigned char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
