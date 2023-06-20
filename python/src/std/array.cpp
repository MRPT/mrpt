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
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
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

PYBIND11_MAKE_OPAQUE(std::array<mrpt::img::CImage,6UL>)

void bind_std_array(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::array file:array line:95
		pybind11::class_<std::array<mrpt::img::CImage,6UL>, std::shared_ptr<std::array<mrpt::img::CImage,6UL>>> cl(M("std"), "array_mrpt_img_CImage_6UL_t", "");
		cl.def( pybind11::init( [](std::array<mrpt::img::CImage,6UL> const &o){ return new std::array<mrpt::img::CImage,6UL>(o); } ) );
		cl.def( pybind11::init( [](){ return new std::array<mrpt::img::CImage,6UL>(); } ) );
		cl.def("fill", (void (std::array<mrpt::img::CImage,6UL>::*)(const class mrpt::img::CImage &)) &std::array<mrpt::img::CImage, 6>::fill, "C++: std::array<mrpt::img::CImage, 6>::fill(const class mrpt::img::CImage &) --> void", pybind11::arg("__u"));
		cl.def("swap", (void (std::array<mrpt::img::CImage,6UL>::*)(struct std::array<class mrpt::img::CImage, 6> &)) &std::array<mrpt::img::CImage, 6>::swap, "C++: std::array<mrpt::img::CImage, 6>::swap(struct std::array<class mrpt::img::CImage, 6> &) --> void", pybind11::arg("__other"));
		cl.def("begin", (class mrpt::img::CImage * (std::array<mrpt::img::CImage,6UL>::*)()) &std::array<mrpt::img::CImage, 6>::begin, "C++: std::array<mrpt::img::CImage, 6>::begin() --> class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("end", (class mrpt::img::CImage * (std::array<mrpt::img::CImage,6UL>::*)()) &std::array<mrpt::img::CImage, 6>::end, "C++: std::array<mrpt::img::CImage, 6>::end() --> class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("cbegin", (const class mrpt::img::CImage * (std::array<mrpt::img::CImage,6UL>::*)() const) &std::array<mrpt::img::CImage, 6>::cbegin, "C++: std::array<mrpt::img::CImage, 6>::cbegin() const --> const class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("cend", (const class mrpt::img::CImage * (std::array<mrpt::img::CImage,6UL>::*)() const) &std::array<mrpt::img::CImage, 6>::cend, "C++: std::array<mrpt::img::CImage, 6>::cend() const --> const class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("size", (size_t (std::array<mrpt::img::CImage,6UL>::*)() const) &std::array<mrpt::img::CImage, 6>::size, "C++: std::array<mrpt::img::CImage, 6>::size() const --> size_t");
		cl.def("max_size", (size_t (std::array<mrpt::img::CImage,6UL>::*)() const) &std::array<mrpt::img::CImage, 6>::max_size, "C++: std::array<mrpt::img::CImage, 6>::max_size() const --> size_t");
		cl.def("empty", (bool (std::array<mrpt::img::CImage,6UL>::*)() const) &std::array<mrpt::img::CImage, 6>::empty, "C++: std::array<mrpt::img::CImage, 6>::empty() const --> bool");
		cl.def("__getitem__", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6UL>::*)(size_t)) &std::array<mrpt::img::CImage, 6>::operator[], "C++: std::array<mrpt::img::CImage, 6>::operator[](size_t) --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6UL>::*)(size_t)) &std::array<mrpt::img::CImage, 6>::at, "C++: std::array<mrpt::img::CImage, 6>::at(size_t) --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6UL>::*)()) &std::array<mrpt::img::CImage, 6>::front, "C++: std::array<mrpt::img::CImage, 6>::front() --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic);
		cl.def("back", (class mrpt::img::CImage & (std::array<mrpt::img::CImage,6UL>::*)()) &std::array<mrpt::img::CImage, 6>::back, "C++: std::array<mrpt::img::CImage, 6>::back() --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic);
		cl.def("data", (class mrpt::img::CImage * (std::array<mrpt::img::CImage,6UL>::*)()) &std::array<mrpt::img::CImage, 6>::data, "C++: std::array<mrpt::img::CImage, 6>::data() --> class mrpt::img::CImage *", pybind11::return_value_policy::automatic);
		cl.def("assign", (struct std::array<class mrpt::img::CImage, 6> & (std::array<mrpt::img::CImage,6UL>::*)(const struct std::array<class mrpt::img::CImage, 6> &)) &std::array<mrpt::img::CImage, 6>::operator=, "C++: std::array<mrpt::img::CImage, 6>::operator=(const struct std::array<class mrpt::img::CImage, 6> &) --> struct std::array<class mrpt::img::CImage, 6> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
