#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
#include <type_traits>
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

// mrpt::img::CCanvas file:mrpt/img/CCanvas.h line:42
struct PyCallBack_mrpt_img_CCanvas : public mrpt::img::CCanvas {
	using mrpt::img::CCanvas::CCanvas;

	void setPixel(int a0, int a1, size_t a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "setPixel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CCanvas::setPixel\"");
	}
	size_t getWidth() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "getWidth");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CCanvas::getWidth\"");
	}
	size_t getHeight() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "getHeight");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CCanvas::getHeight\"");
	}
	void line(int a0, int a1, int a2, int a3, const struct mrpt::img::TColor a4, unsigned int a5, enum mrpt::img::CCanvas::TPenStyle a6) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "line");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::line(a0, a1, a2, a3, a4, a5, a6);
	}
	void filledRectangle(int a0, int a1, int a2, int a3, const struct mrpt::img::TColor a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "filledRectangle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::filledRectangle(a0, a1, a2, a3, a4);
	}
	void textOut(int a0, int a1, const std::string & a2, const struct mrpt::img::TColor a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "textOut");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::textOut(a0, a1, a2, a3);
	}
	void selectTextFont(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "selectTextFont");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::selectTextFont(a0);
	}
	void drawImage(int a0, int a1, const class mrpt::img::CImage & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "drawImage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::drawImage(a0, a1, a2);
	}
	void drawImage(int a0, int a1, const class mrpt::img::CImage & a2, float a3, float a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "drawImage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::drawImage(a0, a1, a2, a3, a4);
	}
	void drawCircle(int a0, int a1, int a2, const struct mrpt::img::TColor & a3, unsigned int a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CCanvas *>(this), "drawCircle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCanvas::drawCircle(a0, a1, a2, a3, a4);
	}
};

void bind_mrpt_img_CCanvas(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::img::CCanvas file:mrpt/img/CCanvas.h line:42
		pybind11::class_<mrpt::img::CCanvas, std::shared_ptr<mrpt::img::CCanvas>, PyCallBack_mrpt_img_CCanvas> cl(M("mrpt::img"), "CCanvas", "This virtual class defines the interface of any object accepting drawing\n primitives on it.\n\n  A number of text fonts can be selected with CCanvas::selectTextFont(). These\n are the\n   implemented font names:\n\n  - \"6x13\"\n  - \"6x13B\" (bold)\n  - \"6x13O\" (italic)\n  - \"9x15\"\n  - \"9x15B\" (bold)\n  - \"10x20\"\n  - \"18x18ja\" (Japanese, UNICODE character values)\n\n  For an example of each font check the \n* href=\"http://www.mrpt.org/Implemented_2D_Fonts\">corresponding wiki page.\n\n \n CImage\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_img_CCanvas(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_img_CCanvas const &>());

		pybind11::enum_<mrpt::img::CCanvas::TPenStyle>(cl, "TPenStyle", pybind11::arithmetic(), "Definition of pen styles ")
			.value("psSolid", mrpt::img::CCanvas::psSolid)
			.value("psDash", mrpt::img::CCanvas::psDash)
			.value("psDot", mrpt::img::CCanvas::psDot)
			.value("psDashDot", mrpt::img::CCanvas::psDashDot)
			.value("psDashDotDot", mrpt::img::CCanvas::psDashDotDot)
			.export_values();

		cl.def("setPixel", (void (mrpt::img::CCanvas::*)(int, int, size_t)) &mrpt::img::CCanvas::setPixel, "Changes the value of the pixel (x,y).\n  Pixel coordinates starts at the left-top corner of the image, and start\n in (0,0).\n  The meaning of the parameter \"color\" depends on the implementation: it\n will usually\n   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray\n level.\n\n  You can also use a TColor() type as input and it will be automatically\n converted to size_t.\n\n  This method must support (x,y) values OUT of the actual image size\n without neither\n   raising exceptions, nor leading to memory access errors.\n\n   \n\nC++: mrpt::img::CCanvas::setPixel(int, int, size_t) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("color"));
		cl.def("getWidth", (size_t (mrpt::img::CCanvas::*)() const) &mrpt::img::CCanvas::getWidth, "Returns the width of the image in pixels\n\nC++: mrpt::img::CCanvas::getWidth() const --> size_t");
		cl.def("getHeight", (size_t (mrpt::img::CCanvas::*)() const) &mrpt::img::CCanvas::getHeight, "Returns the height of the image in pixels\n\nC++: mrpt::img::CCanvas::getHeight() const --> size_t");
		cl.def("line", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2, int const & a3, const struct mrpt::img::TColor & a4) -> void { return o.line(a0, a1, a2, a3, a4); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"));
		cl.def("line", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2, int const & a3, const struct mrpt::img::TColor & a4, unsigned int const & a5) -> void { return o.line(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"), pybind11::arg("width"));
		cl.def("line", (void (mrpt::img::CCanvas::*)(int, int, int, int, const struct mrpt::img::TColor, unsigned int, enum mrpt::img::CCanvas::TPenStyle)) &mrpt::img::CCanvas::line, "Draws a line.\n \n\n The starting point x coordinate\n \n\n The starting point y coordinate\n \n\n The end point x coordinate\n \n\n The end point y coordinate\n \n\n The color of the line\n \n\n The desired width of the line (this is IGNORED in this\n virtual class)\n  This method may be redefined in some classes implementing this\n interface in a more appropiate manner.\n\nC++: mrpt::img::CCanvas::line(int, int, int, int, const struct mrpt::img::TColor, unsigned int, enum mrpt::img::CCanvas::TPenStyle) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"), pybind11::arg("width"), pybind11::arg("penStyle"));
		cl.def("rectangle", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2, int const & a3, const struct mrpt::img::TColor & a4) -> void { return o.rectangle(a0, a1, a2, a3, a4); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"));
		cl.def("rectangle", (void (mrpt::img::CCanvas::*)(int, int, int, int, const struct mrpt::img::TColor, unsigned int)) &mrpt::img::CCanvas::rectangle, "Draws a rectangle (an empty rectangle, without filling)\n \n\n The top-left x coordinate\n \n\n The top-left y coordinate\n \n\n The right-bottom x coordinate\n \n\n The right-bottom y coordinate\n \n\n The color of the line\n \n\n The desired width of the line.\n \n\n filledRectangle\n\nC++: mrpt::img::CCanvas::rectangle(int, int, int, int, const struct mrpt::img::TColor, unsigned int) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"), pybind11::arg("width"));
		cl.def("triangle", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2, const struct mrpt::img::TColor & a3) -> void { return o.triangle(a0, a1, a2, a3); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("size"), pybind11::arg("color"));
		cl.def("triangle", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2, const struct mrpt::img::TColor & a3, bool const & a4) -> void { return o.triangle(a0, a1, a2, a3, a4); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("size"), pybind11::arg("color"), pybind11::arg("inferior"));
		cl.def("triangle", (void (mrpt::img::CCanvas::*)(int, int, int, const struct mrpt::img::TColor, bool, unsigned int)) &mrpt::img::CCanvas::triangle, "Draws a triangle\n \n\n The triangle center x coordinate\n \n\n The triangle center y coordinate\n \n\n The size of the triangle\n \n\n The color of the line\n	\n\n The position of the triangle\n \n\n The desired width of the line.\n \n\n triangle\n\nC++: mrpt::img::CCanvas::triangle(int, int, int, const struct mrpt::img::TColor, bool, unsigned int) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("size"), pybind11::arg("color"), pybind11::arg("inferior"), pybind11::arg("width"));
		cl.def("filledRectangle", (void (mrpt::img::CCanvas::*)(int, int, int, int, const struct mrpt::img::TColor)) &mrpt::img::CCanvas::filledRectangle, "Draws a filled rectangle.\n \n\n The top-left x coordinate\n \n\n The top-left y coordinate\n \n\n The right-bottom x coordinate\n \n\n The right-bottom y coordinate\n \n\n The color of the rectangle fill\n  This method may be redefined in some classes implementing this\n interface in a more appropiate manner.\n \n\n rectangle\n\nC++: mrpt::img::CCanvas::filledRectangle(int, int, int, int, const struct mrpt::img::TColor) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"));
		cl.def("textOut", (void (mrpt::img::CCanvas::*)(int, int, const std::string &, const struct mrpt::img::TColor)) &mrpt::img::CCanvas::textOut, "Renders 2D text using bitmap fonts.\n \n\n The x coordinates\n \n\n The y coordinates\n \n\n The string to put. If using UNICODE characters, use UTF-8\n encoding.\n \n\n The text color\n\n \n selectTextFont\n\nC++: mrpt::img::CCanvas::textOut(int, int, const std::string &, const struct mrpt::img::TColor) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("str"), pybind11::arg("color"));
		cl.def("selectTextFont", (void (mrpt::img::CCanvas::*)(const std::string &)) &mrpt::img::CCanvas::selectTextFont, "Select the current font used when drawing text.\n \n\n The name of the font\n\n  Valid font names:\n  - 5x7\n  - 6x13\n  - 6x13B\n  - 6x13O\n  - 9x15   (Default at start-up)\n  - 9x15B\n  - 10x20\n  - 18x18ja (Asian characters for UTF-8 strings - Only available if MRPT\n is built with MRPT_HAS_ASIAN_FONTS = true)\n\n   \n\n \n textOut, The example in \n* href=\"http://www.mrpt.org/Implemented_2D_Fonts\">this page.\n\nC++: mrpt::img::CCanvas::selectTextFont(const std::string &) --> void", pybind11::arg("fontName"));
		cl.def("drawImage", (void (mrpt::img::CCanvas::*)(int, int, const class mrpt::img::CImage &)) &mrpt::img::CCanvas::drawImage, "Draws an image as a bitmap at a given position.\n \n\n The top-left corner x coordinates on this canvas where the\n image is to be drawn\n \n\n The top-left corner y coordinates on this canvas where the\n image is to be drawn\n \n\n The image to be drawn in this canvas\n  This method may be redefined in some classes implementing this\n interface in a more appropiate manner.\n\nC++: mrpt::img::CCanvas::drawImage(int, int, const class mrpt::img::CImage &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("img"));
		cl.def("drawMark", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, const struct mrpt::img::TColor & a2, char const & a3) -> void { return o.drawMark(a0, a1, a2, a3); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("color"), pybind11::arg("type"));
		cl.def("drawMark", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, const struct mrpt::img::TColor & a2, char const & a3, int const & a4) -> void { return o.drawMark(a0, a1, a2, a3, a4); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("color"), pybind11::arg("type"), pybind11::arg("size"));
		cl.def("drawMark", (void (mrpt::img::CCanvas::*)(int, int, const struct mrpt::img::TColor, char, int, unsigned int)) &mrpt::img::CCanvas::drawMark, "Draw a mark.\n \n\n The point x coordinate\n \n\n The point y coordinate\n \n\n The color of the cross\n \n\n The size of the cross\n \n\n The cross type. It could be: 'x', '+', ':'(like '+' but\n clear at the center dot), or 's' (square)\n \n\n The desired width of the cross (this is IGNORED yet)\n\nC++: mrpt::img::CCanvas::drawMark(int, int, const struct mrpt::img::TColor, char, int, unsigned int) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("color"), pybind11::arg("type"), pybind11::arg("size"), pybind11::arg("width"));
		cl.def("drawImage", (void (mrpt::img::CCanvas::*)(int, int, const class mrpt::img::CImage &, float, float)) &mrpt::img::CCanvas::drawImage, "Draws an image as a bitmap at a given position, with some custom scale\n and rotation changes.\n \n\n The top-left corner x coordinates on this canvas where the\n image is to be drawn\n \n\n The top-left corner y coordinates on this canvas where the\n image is to be drawn\n \n\n The rotation in radians, positive values being\n anti-clockwise direction, 0 is the normal position.\n \n\n The scale factor, e.g. 2 means twice the original size.\n \n\n The image to be drawn in this canvas\n  This method may be redefined in some classes implementing this\n interface in a more appropiate manner.\n\nC++: mrpt::img::CCanvas::drawImage(int, int, const class mrpt::img::CImage &, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("img"), pybind11::arg("rotation"), pybind11::arg("scale"));
		cl.def("drawCircle", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2) -> void { return o.drawCircle(a0, a1, a2); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"));
		cl.def("drawCircle", [](mrpt::img::CCanvas &o, int const & a0, int const & a1, int const & a2, const struct mrpt::img::TColor & a3) -> void { return o.drawCircle(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"), pybind11::arg("color"));
		cl.def("drawCircle", (void (mrpt::img::CCanvas::*)(int, int, int, const struct mrpt::img::TColor &, unsigned int)) &mrpt::img::CCanvas::drawCircle, "Draws a circle of a given radius.\n \n\n The center - x coordinate in pixels.\n \n\n The center - y coordinate in pixels.\n \n\n The radius - in pixels.\n \n\n The color of the circle.\n \n\n The desired width of the line (this is IGNORED in this\n virtual class)\n\nC++: mrpt::img::CCanvas::drawCircle(int, int, int, const struct mrpt::img::TColor &, unsigned int) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"), pybind11::arg("color"), pybind11::arg("width"));
		cl.def("ellipseGaussian", [](mrpt::img::CCanvas &o, const class mrpt::math::CMatrixFixed<double, 2, 2> & a0, const double & a1, const double & a2) -> void { return o.ellipseGaussian(a0, a1, a2); }, "", pybind11::arg("cov2D"), pybind11::arg("mean_x"), pybind11::arg("mean_y"));
		cl.def("ellipseGaussian", [](mrpt::img::CCanvas &o, const class mrpt::math::CMatrixFixed<double, 2, 2> & a0, const double & a1, const double & a2, double const & a3) -> void { return o.ellipseGaussian(a0, a1, a2, a3); }, "", pybind11::arg("cov2D"), pybind11::arg("mean_x"), pybind11::arg("mean_y"), pybind11::arg("confIntervalStds"));
		cl.def("ellipseGaussian", [](mrpt::img::CCanvas &o, const class mrpt::math::CMatrixFixed<double, 2, 2> & a0, const double & a1, const double & a2, double const & a3, const struct mrpt::img::TColor & a4) -> void { return o.ellipseGaussian(a0, a1, a2, a3, a4); }, "", pybind11::arg("cov2D"), pybind11::arg("mean_x"), pybind11::arg("mean_y"), pybind11::arg("confIntervalStds"), pybind11::arg("color"));
		cl.def("ellipseGaussian", [](mrpt::img::CCanvas &o, const class mrpt::math::CMatrixFixed<double, 2, 2> & a0, const double & a1, const double & a2, double const & a3, const struct mrpt::img::TColor & a4, unsigned int const & a5) -> void { return o.ellipseGaussian(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("cov2D"), pybind11::arg("mean_x"), pybind11::arg("mean_y"), pybind11::arg("confIntervalStds"), pybind11::arg("color"), pybind11::arg("width"));
		cl.def("ellipseGaussian", (void (mrpt::img::CCanvas::*)(const class mrpt::math::CMatrixFixed<double, 2, 2> &, const double, const double, double, const struct mrpt::img::TColor &, unsigned int, int)) &mrpt::img::CCanvas::ellipseGaussian, "Draws an ellipse representing a given confidence interval of a 2D\n Gaussian distribution.\n \n\n The x coordinate of the center point of the ellipse.\n \n\n The y coordinate of the center point of the ellipse.\n \n\n A 2x2 covariance matrix.\n \n\n How many \"sigmas\" for the confidence level (i.e.\n 2->95%, 3=99.97%,...)\n \n\n The color of the ellipse\n \n\n The desired width of the line (this is IGNORED in this\n virtual class)\n \n\n The number of points to generate to approximate\n the ellipse shape.\n \n\n std::exception On an invalid matrix.\n\nC++: mrpt::img::CCanvas::ellipseGaussian(const class mrpt::math::CMatrixFixed<double, 2, 2> &, const double, const double, double, const struct mrpt::img::TColor &, unsigned int, int) --> void", pybind11::arg("cov2D"), pybind11::arg("mean_x"), pybind11::arg("mean_y"), pybind11::arg("confIntervalStds"), pybind11::arg("color"), pybind11::arg("width"), pybind11::arg("nEllipsePoints"));
		cl.def("assign", (class mrpt::img::CCanvas & (mrpt::img::CCanvas::*)(const class mrpt::img::CCanvas &)) &mrpt::img::CCanvas::operator=, "C++: mrpt::img::CCanvas::operator=(const class mrpt::img::CCanvas &) --> class mrpt::img::CCanvas &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
