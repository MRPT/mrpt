#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>

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

void bind_mrpt_img_color_maps(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::img::hsv2rgb(float, float, float, float &, float &, float &) file:mrpt/img/color_maps.h line:23
	M("mrpt::img").def("hsv2rgb", (void (*)(float, float, float, float &, float &, float &)) &mrpt::img::hsv2rgb, "Transform HSV color components to RGB, all of them in the range [0,1]  \n\n rgb2hsv \n\nC++: mrpt::img::hsv2rgb(float, float, float, float &, float &, float &) --> void", pybind11::arg("h"), pybind11::arg("s"), pybind11::arg("v"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));

	// mrpt::img::rgb2hsv(float, float, float, float &, float &, float &) file:mrpt/img/color_maps.h line:27
	M("mrpt::img").def("rgb2hsv", (void (*)(float, float, float, float &, float &, float &)) &mrpt::img::rgb2hsv, "Transform RGB color components to HSV, all of them in the range [0,1] \n\n hsv2rgb \n\nC++: mrpt::img::rgb2hsv(float, float, float, float &, float &, float &) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("h"), pybind11::arg("s"), pybind11::arg("v"));

	// mrpt::img::TColormap file:mrpt/img/color_maps.h line:30
	pybind11::enum_<mrpt::img::TColormap>(M("mrpt::img"), "TColormap", pybind11::arithmetic(), "Different colormaps for use in mrpt::img::colormap() ")
		.value("cmNONE", mrpt::img::cmNONE)
		.value("cmGRAYSCALE", mrpt::img::cmGRAYSCALE)
		.value("cmJET", mrpt::img::cmJET)
		.value("cmHOT", mrpt::img::cmHOT)
		.export_values();

;

	// mrpt::img::colormap(const enum mrpt::img::TColormap &, const float, float &, float &, float &) file:mrpt/img/color_maps.h line:41
	M("mrpt::img").def("colormap", (void (*)(const enum mrpt::img::TColormap &, const float, float &, float &, float &)) &mrpt::img::colormap, "Transform a float number in the range [0,1] into RGB components. Different\n colormaps are available. \n\nC++: mrpt::img::colormap(const enum mrpt::img::TColormap &, const float, float &, float &, float &) --> void", pybind11::arg("color_map"), pybind11::arg("color_index"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));

	// mrpt::img::colormap(const enum mrpt::img::TColormap &, const float) file:mrpt/img/color_maps.h line:44
	M("mrpt::img").def("colormap", (struct mrpt::img::TColor (*)(const enum mrpt::img::TColormap &, const float)) &mrpt::img::colormap, "C++: mrpt::img::colormap(const enum mrpt::img::TColormap &, const float) --> struct mrpt::img::TColor", pybind11::arg("color_map"), pybind11::arg("color_index"));

	// mrpt::img::jet2rgb(const float, float &, float &, float &) file:mrpt/img/color_maps.h line:48
	M("mrpt::img").def("jet2rgb", (void (*)(const float, float &, float &, float &)) &mrpt::img::jet2rgb, "Computes the RGB color components (range [0,1]) for the corresponding color\n index in the range [0,1] using the MATLAB 'jet' colormap.  \n\n colormap  \n\nC++: mrpt::img::jet2rgb(const float, float &, float &, float &) --> void", pybind11::arg("color_index"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));

	// mrpt::img::hot2rgb(const float, float &, float &, float &) file:mrpt/img/color_maps.h line:52
	M("mrpt::img").def("hot2rgb", (void (*)(const float, float &, float &, float &)) &mrpt::img::hot2rgb, "Computes the RGB color components (range [0,1]) for the corresponding color\n index in the range [0,1] using the MATLAB 'hot' colormap.  \n\n colormap  \n\nC++: mrpt::img::hot2rgb(const float, float &, float &, float &) --> void", pybind11::arg("color_index"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));

}
