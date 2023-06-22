#include <iterator>
#include <memory>
#include <mrpt/containers/MT_buffer.h>
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

void bind_mrpt_containers_MT_buffer(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::MT_buffer file:mrpt/containers/MT_buffer.h line:22
		pybind11::class_<mrpt::containers::MT_buffer, std::shared_ptr<mrpt::containers::MT_buffer>> cl(M("mrpt::containers"), "MT_buffer", "This class is a bulk sequence of bytes with MultiThread (MT)-safe read and\n write operations.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::containers::MT_buffer(); } ) );
		cl.def("clear", (void (mrpt::containers::MT_buffer::*)()) &mrpt::containers::MT_buffer::clear, "Empty the buffer \n\nC++: mrpt::containers::MT_buffer::clear() --> void");
		cl.def("size", (size_t (mrpt::containers::MT_buffer::*)()) &mrpt::containers::MT_buffer::size, "Return the number of available bytes at this moment. \n\nC++: mrpt::containers::MT_buffer::size() --> size_t");
	}
}
