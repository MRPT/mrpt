#include <array>
#include <iterator>
#include <memory>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
#include <vector>

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

void bind_std_array(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::array file:array line:95
		pybind11::class_<std::array<unsigned long,2>, std::shared_ptr<std::array<unsigned long,2>>> cl(M("std"), "array_unsigned_long_2_t", "");
		cl.def( pybind11::init( [](std::array<unsigned long,2> const &o){ return new std::array<unsigned long,2>(o); } ) );
		cl.def( pybind11::init( [](){ return new std::array<unsigned long,2>(); } ) );
		cl.def("fill", (void (std::array<unsigned long,2>::*)(const unsigned long &)) &std::array<unsigned long, 2>::fill, "C++: std::array<unsigned long, 2>::fill(const unsigned long &) --> void", pybind11::arg("__u"));
		cl.def("swap", (void (std::array<unsigned long,2>::*)(struct std::array<unsigned long, 2> &)) &std::array<unsigned long, 2>::swap, "C++: std::array<unsigned long, 2>::swap(struct std::array<unsigned long, 2> &) --> void", pybind11::arg("__other"));
		cl.def("begin", (unsigned long * (std::array<unsigned long,2>::*)()) &std::array<unsigned long, 2>::begin, "C++: std::array<unsigned long, 2>::begin() --> unsigned long *", pybind11::return_value_policy::automatic);
		cl.def("end", (unsigned long * (std::array<unsigned long,2>::*)()) &std::array<unsigned long, 2>::end, "C++: std::array<unsigned long, 2>::end() --> unsigned long *", pybind11::return_value_policy::automatic);
		cl.def("cbegin", (const unsigned long * (std::array<unsigned long,2>::*)() const) &std::array<unsigned long, 2>::cbegin, "C++: std::array<unsigned long, 2>::cbegin() const --> const unsigned long *", pybind11::return_value_policy::automatic);
		cl.def("cend", (const unsigned long * (std::array<unsigned long,2>::*)() const) &std::array<unsigned long, 2>::cend, "C++: std::array<unsigned long, 2>::cend() const --> const unsigned long *", pybind11::return_value_policy::automatic);
		cl.def("size", (unsigned long (std::array<unsigned long,2>::*)() const) &std::array<unsigned long, 2>::size, "C++: std::array<unsigned long, 2>::size() const --> unsigned long");
		cl.def("max_size", (unsigned long (std::array<unsigned long,2>::*)() const) &std::array<unsigned long, 2>::max_size, "C++: std::array<unsigned long, 2>::max_size() const --> unsigned long");
		cl.def("empty", (bool (std::array<unsigned long,2>::*)() const) &std::array<unsigned long, 2>::empty, "C++: std::array<unsigned long, 2>::empty() const --> bool");
		cl.def("__getitem__", (unsigned long & (std::array<unsigned long,2>::*)(unsigned long)) &std::array<unsigned long, 2>::operator[], "C++: std::array<unsigned long, 2>::operator[](unsigned long) --> unsigned long &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (unsigned long & (std::array<unsigned long,2>::*)(unsigned long)) &std::array<unsigned long, 2>::at, "C++: std::array<unsigned long, 2>::at(unsigned long) --> unsigned long &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (unsigned long & (std::array<unsigned long,2>::*)()) &std::array<unsigned long, 2>::front, "C++: std::array<unsigned long, 2>::front() --> unsigned long &", pybind11::return_value_policy::automatic);
		cl.def("back", (unsigned long & (std::array<unsigned long,2>::*)()) &std::array<unsigned long, 2>::back, "C++: std::array<unsigned long, 2>::back() --> unsigned long &", pybind11::return_value_policy::automatic);
		cl.def("data", (unsigned long * (std::array<unsigned long,2>::*)()) &std::array<unsigned long, 2>::data, "C++: std::array<unsigned long, 2>::data() --> unsigned long *", pybind11::return_value_policy::automatic);
	}
	{ // std::array file:array line:95
		pybind11::class_<std::array<mrpt::img::CImage,6>, std::shared_ptr<std::array<mrpt::img::CImage,6>>> cl(M("std"), "array_mrpt_img_CImage_6_t", "");
		cl.def( pybind11::init( [](std::array<mrpt::img::CImage,6> const &o){ return new std::array<mrpt::img::CImage,6>(o); } ) );
		cl.def( pybind11::init( [](){ return new std::array<mrpt::img::CImage,6>(); } ) );
		cl.def("fill", (void (std::array<mrpt::img::CImage,6>::*)(const class mrpt::img::CImage &)) &std::array<mrpt::img::CImage, 6>::fill, "C++: std::array<mrpt::img::CImage, 6>::fill(const class mrpt::img::CImage &) --> void", pybind11::arg("__u"));
		cl.def("swap", (void (std::array<mrpt::img::CImage,6>::*)(struct std::array<class mrpt::img::CImage, 6> &)) &std::array<mrpt::img::CImage, 6>::swap, "C++: std::array<mrpt::img::CImage, 6>::swap(struct std::array<class mrpt::img::CImage, 6> &) --> void", pybind11::arg("__other"));
		cl.def("begin", (class mrpt::img::CImage * (std::array<mrpt::img::CImage,6>::*)()) &std::array<mrpt::img::CImage, 6>::begin, "C++: std::array<mrpt::img::CImage, 6>::begin() --> class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("end", (class mrpt::img::CImage * (std::array<mrpt::img::CImage,6>::*)()) &std::array<mrpt::img::CImage, 6>::end, "C++: std::array<mrpt::img::CImage, 6>::end() --> class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("cbegin", (const class mrpt::img::CImage * (std::array<mrpt::img::CImage,6>::*)() const) &std::array<mrpt::img::CImage, 6>::cbegin, "C++: std::array<mrpt::img::CImage, 6>::cbegin() const --> const class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("cend", (const class mrpt::img::CImage * (std::array<mrpt::img::CImage,6>::*)() const) &std::array<mrpt::img::CImage, 6>::cend, "C++: std::array<mrpt::img::CImage, 6>::cend() const --> const class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("size", (unsigned long (std::array<mrpt::img::CImage,6>::*)() const) &std::array<mrpt::img::CImage, 6>::size, "C++: std::array<mrpt::img::CImage, 6>::size() const --> unsigned long");
		cl.def("max_size", (unsigned long (std::array<mrpt::img::CImage,6>::*)() const) &std::array<mrpt::img::CImage, 6>::max_size, "C++: std::array<mrpt::img::CImage, 6>::max_size() const --> unsigned long");
		cl.def("empty", (bool (std::array<mrpt::img::CImage,6>::*)() const) &std::array<mrpt::img::CImage, 6>::empty, "C++: std::array<mrpt::img::CImage, 6>::empty() const --> bool");
		cl.def("__getitem__", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6>::*)(unsigned long)) &std::array<mrpt::img::CImage, 6>::operator[], "C++: std::array<mrpt::img::CImage, 6>::operator[](unsigned long) --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6>::*)(unsigned long)) &std::array<mrpt::img::CImage, 6>::at, "C++: std::array<mrpt::img::CImage, 6>::at(unsigned long) --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6>::*)()) &std::array<mrpt::img::CImage, 6>::front, "C++: std::array<mrpt::img::CImage, 6>::front() --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic);
		cl.def("back", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6>::*)()) &std::array<mrpt::img::CImage, 6>::back, "C++: std::array<mrpt::img::CImage, 6>::back() --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic);
		cl.def("data", (class mrpt::img::CImage * (std::array<mrpt::img::CImage,6>::*)()) &std::array<mrpt::img::CImage, 6>::data, "C++: std::array<mrpt::img::CImage, 6>::data() --> class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("assign", (struct std::array<class mrpt::img::CImage, 6> & (std::array<mrpt::img::CImage,6>::*)(const struct std::array<class mrpt::img::CImage, 6> &)) &std::array<mrpt::img::CImage, 6>::operator=, "C++: std::array<mrpt::img::CImage, 6>::operator=(const struct std::array<class mrpt::img::CImage, 6> &) --> struct std::array<class mrpt::img::CImage, 6> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
