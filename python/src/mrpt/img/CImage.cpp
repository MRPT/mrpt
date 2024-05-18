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
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
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

// mrpt::img::CExceptionExternalImageNotFound file:mrpt/img/CImage.h line:83
struct PyCallBack_mrpt_img_CExceptionExternalImageNotFound : public mrpt::img::CExceptionExternalImageNotFound {
	using mrpt::img::CExceptionExternalImageNotFound::CExceptionExternalImageNotFound;

	const char * what() const throw() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CExceptionExternalImageNotFound *>(this), "what");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const char *>::value) {
				static pybind11::detail::override_caster_t<const char *> caster;
				return pybind11::detail::cast_ref<const char *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const char *>(std::move(o));
		}
		return runtime_error::what();
	}
};

// mrpt::img::CImage file:mrpt/img/CImage.h line:149
struct PyCallBack_mrpt_img_CImage : public mrpt::img::CImage {
	using mrpt::img::CImage::CImage;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CImage::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CImage::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CImage::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::serializeFrom(a0, a1);
	}
	void setPixel(int a0, int a1, size_t a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "setPixel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::setPixel(a0, a1, a2);
	}
	void line(int a0, int a1, int a2, int a3, const struct mrpt::img::TColor a4, unsigned int a5, enum mrpt::img::CCanvas::TPenStyle a6) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "line");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::line(a0, a1, a2, a3, a4, a5, a6);
	}
	void drawCircle(int a0, int a1, int a2, const struct mrpt::img::TColor & a3, unsigned int a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "drawCircle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::drawCircle(a0, a1, a2, a3, a4);
	}
	void drawImage(int a0, int a1, const class mrpt::img::CImage & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "drawImage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::drawImage(a0, a1, a2);
	}
	void filledRectangle(int a0, int a1, int a2, int a3, const struct mrpt::img::TColor a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "filledRectangle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImage::filledRectangle(a0, a1, a2, a3, a4);
	}
	size_t getWidth() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "getWidth");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CImage::getWidth();
	}
	size_t getHeight() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "getHeight");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CImage::getHeight();
	}
	void textOut(int a0, int a1, const std::string & a2, const struct mrpt::img::TColor a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "textOut");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "selectTextFont");
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
	void drawImage(int a0, int a1, const class mrpt::img::CImage & a2, float a3, float a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::CImage *>(this), "drawImage");
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
};

void bind_mrpt_img_CImage(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::img::PixelDepth file:mrpt/img/CImage.h line:33
	pybind11::enum_<mrpt::img::PixelDepth>(M("mrpt::img"), "PixelDepth", "")
		.value("D8U", mrpt::img::PixelDepth::D8U)
		.value("D8S", mrpt::img::PixelDepth::D8S)
		.value("D16U", mrpt::img::PixelDepth::D16U)
		.value("D16S", mrpt::img::PixelDepth::D16S)
		.value("D32S", mrpt::img::PixelDepth::D32S)
		.value("D32F", mrpt::img::PixelDepth::D32F)
		.value("D64F", mrpt::img::PixelDepth::D64F);

;

	// mrpt::img::TInterpolationMethod file:mrpt/img/CImage.h line:51
	pybind11::enum_<mrpt::img::TInterpolationMethod>(M("mrpt::img"), "TInterpolationMethod", pybind11::arithmetic(), "Interpolation methods for images.\n  Used for OpenCV related operations with images, but also with MRPT native\n classes.\n \n\n mrpt::img::CMappedImage, CImage::scaleImage\n \n\n These are numerically compatible to cv::InterpolationFlags\n \n\n\n ")
		.value("IMG_INTERP_NN", mrpt::img::IMG_INTERP_NN)
		.value("IMG_INTERP_LINEAR", mrpt::img::IMG_INTERP_LINEAR)
		.value("IMG_INTERP_CUBIC", mrpt::img::IMG_INTERP_CUBIC)
		.value("IMG_INTERP_AREA", mrpt::img::IMG_INTERP_AREA)
		.export_values();

;

	// mrpt::img::TImageChannels file:mrpt/img/CImage.h line:60
	pybind11::enum_<mrpt::img::TImageChannels>(M("mrpt::img"), "TImageChannels", pybind11::arithmetic(), "For use in mrpt::img::CImage ")
		.value("CH_GRAY", mrpt::img::CH_GRAY)
		.value("CH_RGB", mrpt::img::CH_RGB)
		.export_values();

;

	// mrpt::img::ctor_CImage_ref_or_gray file:mrpt/img/CImage.h line:67
	pybind11::enum_<mrpt::img::ctor_CImage_ref_or_gray>(M("mrpt::img"), "ctor_CImage_ref_or_gray", pybind11::arithmetic(), "For usage in one of the CImage constructors ")
		.value("FAST_REF_OR_CONVERT_TO_GRAY", mrpt::img::FAST_REF_OR_CONVERT_TO_GRAY)
		.export_values();

;

	// mrpt::img::copy_type_t file:mrpt/img/CImage.h line:73
	pybind11::enum_<mrpt::img::copy_type_t>(M("mrpt::img"), "copy_type_t", pybind11::arithmetic(), "Define kind of copies  ")
		.value("SHALLOW_COPY", mrpt::img::SHALLOW_COPY)
		.value("DEEP_COPY", mrpt::img::DEEP_COPY)
		.export_values();

;

	{ // mrpt::img::CExceptionExternalImageNotFound file:mrpt/img/CImage.h line:83
		pybind11::class_<mrpt::img::CExceptionExternalImageNotFound, std::shared_ptr<mrpt::img::CExceptionExternalImageNotFound>, PyCallBack_mrpt_img_CExceptionExternalImageNotFound> cl(M("mrpt::img"), "CExceptionExternalImageNotFound", "Used in mrpt::img::CImage ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_img_CExceptionExternalImageNotFound const &o){ return new PyCallBack_mrpt_img_CExceptionExternalImageNotFound(o); } ) );
		cl.def( pybind11::init( [](mrpt::img::CExceptionExternalImageNotFound const &o){ return new mrpt::img::CExceptionExternalImageNotFound(o); } ) );
		cl.def("assign", (class mrpt::img::CExceptionExternalImageNotFound & (mrpt::img::CExceptionExternalImageNotFound::*)(const class mrpt::img::CExceptionExternalImageNotFound &)) &mrpt::img::CExceptionExternalImageNotFound::operator=, "C++: mrpt::img::CExceptionExternalImageNotFound::operator=(const class mrpt::img::CExceptionExternalImageNotFound &) --> class mrpt::img::CExceptionExternalImageNotFound &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::img::CImage file:mrpt/img/CImage.h line:149
		pybind11::class_<mrpt::img::CImage, std::shared_ptr<mrpt::img::CImage>, PyCallBack_mrpt_img_CImage, mrpt::serialization::CSerializable, mrpt::img::CCanvas> cl(M("mrpt::img"), "CImage", "A class for storing images as grayscale or RGB bitmaps.\n I/O is supported as:\n - Binary dump using the CSerializable interface(<< and >> operators),\njust as most objects in MRPT. This format is not compatible with any\nstandarized image format but it is fast.\n - Saving/loading from files of different formats (bmp,jpg,png,...) using\nthe methods CImage::loadFromFile and CImage::saveToFile. See OpenCV for the\nlist of supported formats.\n - Importing from an XPM array (.xpm file format) using CImage::loadFromXPM()\n - Importing TGA images. See CImage::loadTGA()\n\n  How to create color/grayscale images:\n  \n\n\n\n\n Additional notes:\n - The OpenCV `cv::Mat` format is used internally for compatibility with\n all OpenCV functions. Use CImage::asCvMat() to retrieve it. Example:\n \n\n\n\n\n\n - By default, all images use unsigned 8-bit storage format for pixels (on\neach channel), but it can be changed by flags in the constructor.\n - An **external storage mode** can be enabled by calling\nCImage::setExternalStorage, useful for storing large collections of image\nobjects in memory while loading the image data itself only for the relevant\nimages at any time. See CImage::forceLoad() and CImage::unload().\n - Operator = and copy ctor make shallow copies. For deep copies, see\n CImage::makeDeepCopy() or CImage(const CImage&, copy_type_t), e.g:\n \n\n\n\n\n\n\n - If you are interested in a smart pointer to an image, use:\n \n\n\n\n\n\n - To set a CImage from an OpenCV `cv::Mat` use\nCImage::CImage(cv::Mat,copy_type_t).\n\n Some functions are implemented in MRPT with highly optimized SSE2/SSE3\nroutines, in suitable platforms and compilers. To see the list of\n optimizations refer to  falling back to default OpenCV\nmethods where unavailable.\n\n For computer vision functions that use CImage as its image data type,\nsee mrpt::vision.\n\n \n mrpt::vision, mrpt::vision::CFeatureExtractor,\nmrpt::vision::CImagePyramid, CSerializable, CCanvas\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::img::CImage(); }, [](){ return new PyCallBack_mrpt_img_CImage(); } ) );
		cl.def( pybind11::init( [](unsigned int const & a0, unsigned int const & a1){ return new mrpt::img::CImage(a0, a1); }, [](unsigned int const & a0, unsigned int const & a1){ return new PyCallBack_mrpt_img_CImage(a0, a1); } ), "doc");
		cl.def( pybind11::init<unsigned int, unsigned int, enum mrpt::img::TImageChannels>(), pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("nChannels") );

		cl.def( pybind11::init<const class mrpt::img::CImage &, enum mrpt::img::ctor_CImage_ref_or_gray>(), pybind11::arg("other_img"), pybind11::arg("") );

		cl.def( pybind11::init<const class mrpt::img::CImage &, enum mrpt::img::copy_type_t>(), pybind11::arg("img"), pybind11::arg("copy_type") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_img_CImage const &o){ return new PyCallBack_mrpt_img_CImage(o); } ) );
		cl.def( pybind11::init( [](mrpt::img::CImage const &o){ return new mrpt::img::CImage(o); } ) );
		cl.def("ptrLine", (const unsigned char * (mrpt::img::CImage::*)(unsigned int) const) &mrpt::img::CImage::ptrLine<unsigned char>, "C++: mrpt::img::CImage::ptrLine(unsigned int) const --> const unsigned char *", pybind11::return_value_policy::automatic, pybind11::arg("row"));
		cl.def("ptrLine", (unsigned char * (mrpt::img::CImage::*)(unsigned int)) &mrpt::img::CImage::ptrLine<unsigned char>, "C++: mrpt::img::CImage::ptrLine(unsigned int) --> unsigned char *", pybind11::return_value_policy::automatic, pybind11::arg("row"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::img::CImage::GetRuntimeClassIdStatic, "C++: mrpt::img::CImage::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::img::CImage::*)() const) &mrpt::img::CImage::GetRuntimeClass, "C++: mrpt::img::CImage::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::img::CImage::*)() const) &mrpt::img::CImage::clone, "C++: mrpt::img::CImage::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::img::CImage::CreateObject, "C++: mrpt::img::CImage::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def_static("DISABLE_JPEG_COMPRESSION", (void (*)(bool)) &mrpt::img::CImage::DISABLE_JPEG_COMPRESSION, "By default, when storing images through the CSerializable interface, RGB\n images are JPEG-compressed to save space. If for some reason you prefer\n storing RAW image data, disable this feature by setting this flag to\n true.\n  (Default = true) \n\nC++: mrpt::img::CImage::DISABLE_JPEG_COMPRESSION(bool) --> void", pybind11::arg("val"));
		cl.def_static("DISABLE_JPEG_COMPRESSION", (bool (*)()) &mrpt::img::CImage::DISABLE_JPEG_COMPRESSION, "C++: mrpt::img::CImage::DISABLE_JPEG_COMPRESSION() --> bool");
		cl.def_static("SERIALIZATION_JPEG_QUALITY", (void (*)(int)) &mrpt::img::CImage::SERIALIZATION_JPEG_QUALITY, "Unless DISABLE_JPEG_COMPRESSION=true, this sets the JPEG quality (range\n 1-100) of serialized RGB images.\n  (Default = 95) \n\nC++: mrpt::img::CImage::SERIALIZATION_JPEG_QUALITY(int) --> void", pybind11::arg("q"));
		cl.def_static("SERIALIZATION_JPEG_QUALITY", (int (*)()) &mrpt::img::CImage::SERIALIZATION_JPEG_QUALITY, "C++: mrpt::img::CImage::SERIALIZATION_JPEG_QUALITY() --> int");
		cl.def("clear", (void (mrpt::img::CImage::*)()) &mrpt::img::CImage::clear, "Resets the image to the state after a default ctor. Accessing the image\n after will throw an exception, unless it is formerly initialized somehow:\n loading an image from disk, calling rezize(), etc. \n\nC++: mrpt::img::CImage::clear() --> void");
		cl.def("resize", [](mrpt::img::CImage &o, std::size_t const & a0, std::size_t const & a1, enum mrpt::img::TImageChannels const & a2) -> void { return o.resize(a0, a1, a2); }, "", pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("nChannels"));
		cl.def("resize", (void (mrpt::img::CImage::*)(std::size_t, std::size_t, enum mrpt::img::TImageChannels, enum mrpt::img::PixelDepth)) &mrpt::img::CImage::resize, "Changes the size of the image, erasing previous contents (does NOT scale\n its current content, for that, see scaleImage).\n  - nChannels: Can be 3 for RGB images or 1 for grayscale images.\n \n\n scaleImage\n\nC++: mrpt::img::CImage::resize(std::size_t, std::size_t, enum mrpt::img::TImageChannels, enum mrpt::img::PixelDepth) --> void", pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("nChannels"), pybind11::arg("depth"));
		cl.def("getPixelDepth", (enum mrpt::img::PixelDepth (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getPixelDepth, "C++: mrpt::img::CImage::getPixelDepth() const --> enum mrpt::img::PixelDepth");
		cl.def("scaleImage", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, unsigned int const & a1, unsigned int const & a2) -> void { return o.scaleImage(a0, a1, a2); }, "", pybind11::arg("out_img"), pybind11::arg("width"), pybind11::arg("height"));
		cl.def("scaleImage", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, unsigned int, unsigned int, enum mrpt::img::TInterpolationMethod) const) &mrpt::img::CImage::scaleImage, "Scales this image to a new size, interpolating as needed, saving the new\n image in a different output object, or operating in-place if\n `out_img==this`. \n\n resize, rotateImage\n\nC++: mrpt::img::CImage::scaleImage(class mrpt::img::CImage &, unsigned int, unsigned int, enum mrpt::img::TInterpolationMethod) const --> void", pybind11::arg("out_img"), pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("interp"));
		cl.def("rotateImage", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, double const & a1, unsigned int const & a2, unsigned int const & a3) -> void { return o.rotateImage(a0, a1, a2, a3); }, "", pybind11::arg("out_img"), pybind11::arg("ang"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("rotateImage", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, double, unsigned int, unsigned int, double) const) &mrpt::img::CImage::rotateImage, "Rotates the image by the given angle around the given center point, with\n an optional scale factor.\n \n\n resize, scaleImage\n\nC++: mrpt::img::CImage::rotateImage(class mrpt::img::CImage &, double, unsigned int, unsigned int, double) const --> void", pybind11::arg("out_img"), pybind11::arg("ang"), pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("scale"));
		cl.def("setPixel", (void (mrpt::img::CImage::*)(int, int, size_t)) &mrpt::img::CImage::setPixel, "Changes the value of the pixel (x,y).\n  Pixel coordinates starts at the left-top corner of the image, and start\n in (0,0).\n  The meaning of the parameter \"color\" depends on the implementation: it\n will usually\n   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray\n level.\n  This method must support (x,y) values OUT of the actual image size\n without neither\n   raising exceptions, nor leading to memory access errors.\n \n\n at, ptr\n\nC++: mrpt::img::CImage::setPixel(int, int, size_t) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("color"));
		cl.def("line", [](mrpt::img::CImage &o, int const & a0, int const & a1, int const & a2, int const & a3, const struct mrpt::img::TColor & a4) -> void { return o.line(a0, a1, a2, a3, a4); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"));
		cl.def("line", [](mrpt::img::CImage &o, int const & a0, int const & a1, int const & a2, int const & a3, const struct mrpt::img::TColor & a4, unsigned int const & a5) -> void { return o.line(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"), pybind11::arg("width"));
		cl.def("line", (void (mrpt::img::CImage::*)(int, int, int, int, const struct mrpt::img::TColor, unsigned int, enum mrpt::img::CCanvas::TPenStyle)) &mrpt::img::CImage::line, "C++: mrpt::img::CImage::line(int, int, int, int, const struct mrpt::img::TColor, unsigned int, enum mrpt::img::CCanvas::TPenStyle) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"), pybind11::arg("width"), pybind11::arg("penStyle"));
		cl.def("drawCircle", [](mrpt::img::CImage &o, int const & a0, int const & a1, int const & a2) -> void { return o.drawCircle(a0, a1, a2); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"));
		cl.def("drawCircle", [](mrpt::img::CImage &o, int const & a0, int const & a1, int const & a2, const struct mrpt::img::TColor & a3) -> void { return o.drawCircle(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"), pybind11::arg("color"));
		cl.def("drawCircle", (void (mrpt::img::CImage::*)(int, int, int, const struct mrpt::img::TColor &, unsigned int)) &mrpt::img::CImage::drawCircle, "C++: mrpt::img::CImage::drawCircle(int, int, int, const struct mrpt::img::TColor &, unsigned int) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("radius"), pybind11::arg("color"), pybind11::arg("width"));
		cl.def("drawImage", (void (mrpt::img::CImage::*)(int, int, const class mrpt::img::CImage &)) &mrpt::img::CImage::drawImage, "C++: mrpt::img::CImage::drawImage(int, int, const class mrpt::img::CImage &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("img"));
		cl.def("filledRectangle", (void (mrpt::img::CImage::*)(int, int, int, int, const struct mrpt::img::TColor)) &mrpt::img::CImage::filledRectangle, "C++: mrpt::img::CImage::filledRectangle(int, int, int, int, const struct mrpt::img::TColor) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("color"));
		cl.def("equalizeHist", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &) const) &mrpt::img::CImage::equalizeHist, "Equalize the image histogram, saving the new image in the given output\n object.  \n\n RGB images are first converted to HSV color space, then\n equalized for brightness (V) \n\nC++: mrpt::img::CImage::equalizeHist(class mrpt::img::CImage &) const --> void", pybind11::arg("out_img"));
		cl.def("scaleHalf", (class mrpt::img::CImage (mrpt::img::CImage::*)(enum mrpt::img::TInterpolationMethod) const) &mrpt::img::CImage::scaleHalf, "Returns a new image scaled down to half its original size\n \n\n std::exception On odd size\n \n\n scaleDouble, scaleImage, scaleHalfSmooth\n\nC++: mrpt::img::CImage::scaleHalf(enum mrpt::img::TInterpolationMethod) const --> class mrpt::img::CImage", pybind11::arg("interp"));
		cl.def("scaleHalf", (bool (mrpt::img::CImage::*)(class mrpt::img::CImage &, enum mrpt::img::TInterpolationMethod) const) &mrpt::img::CImage::scaleHalf, "true if an optimized SSE2/SSE3 version could be used. \n\nC++: mrpt::img::CImage::scaleHalf(class mrpt::img::CImage &, enum mrpt::img::TInterpolationMethod) const --> bool", pybind11::arg("out_image"), pybind11::arg("interp"));
		cl.def("scaleDouble", (class mrpt::img::CImage (mrpt::img::CImage::*)(enum mrpt::img::TInterpolationMethod) const) &mrpt::img::CImage::scaleDouble, "Returns a new image scaled up to double its original size.\n \n\n std::exception On odd size\n \n\n scaleHalf, scaleImage\n\nC++: mrpt::img::CImage::scaleDouble(enum mrpt::img::TInterpolationMethod) const --> class mrpt::img::CImage", pybind11::arg("interp"));
		cl.def("scaleDouble", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, enum mrpt::img::TInterpolationMethod) const) &mrpt::img::CImage::scaleDouble, "C++: mrpt::img::CImage::scaleDouble(class mrpt::img::CImage &, enum mrpt::img::TInterpolationMethod) const --> void", pybind11::arg("out_image"), pybind11::arg("interp"));
		cl.def("update_patch", (void (mrpt::img::CImage::*)(const class mrpt::img::CImage &, const unsigned int, const unsigned int)) &mrpt::img::CImage::update_patch, "Update a part of this image with the \"patch\" given as argument.\n The \"patch\" will be \"pasted\" at the (col,row) coordinates of this image.\n \n\n std::exception if patch pasted on the pixel (_row, _column)\n jut out\n of the image.\n \n\n extract_patch\n\nC++: mrpt::img::CImage::update_patch(const class mrpt::img::CImage &, const unsigned int, const unsigned int) --> void", pybind11::arg("patch"), pybind11::arg("col"), pybind11::arg("row"));
		cl.def("extract_patch", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0) -> void { return o.extract_patch(a0); }, "", pybind11::arg("patch"));
		cl.def("extract_patch", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, const unsigned int & a1) -> void { return o.extract_patch(a0, a1); }, "", pybind11::arg("patch"), pybind11::arg("col"));
		cl.def("extract_patch", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, const unsigned int & a1, const unsigned int & a2) -> void { return o.extract_patch(a0, a1, a2); }, "", pybind11::arg("patch"), pybind11::arg("col"), pybind11::arg("row"));
		cl.def("extract_patch", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, const unsigned int & a1, const unsigned int & a2, const unsigned int & a3) -> void { return o.extract_patch(a0, a1, a2, a3); }, "", pybind11::arg("patch"), pybind11::arg("col"), pybind11::arg("row"), pybind11::arg("width"));
		cl.def("extract_patch", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, const unsigned int, const unsigned int, const unsigned int, const unsigned int) const) &mrpt::img::CImage::extract_patch, "Extract a patch from this image, saveing it into \"patch\" (its previous\n contents will be overwritten).\n  The patch to extract starts at (col,row) and has the given dimensions.\n \n\n update_patch\n\nC++: mrpt::img::CImage::extract_patch(class mrpt::img::CImage &, const unsigned int, const unsigned int, const unsigned int, const unsigned int) const --> void", pybind11::arg("patch"), pybind11::arg("col"), pybind11::arg("row"), pybind11::arg("width"), pybind11::arg("height"));
		cl.def("correlate", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0) -> float { return o.correlate(a0); }, "", pybind11::arg("img2int"));
		cl.def("correlate", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, int const & a1) -> float { return o.correlate(a0, a1); }, "", pybind11::arg("img2int"), pybind11::arg("width_init"));
		cl.def("correlate", (float (mrpt::img::CImage::*)(const class mrpt::img::CImage &, int, int) const) &mrpt::img::CImage::correlate, "Computes the correlation coefficient (returned as val), between two\nimages\n	This function use grayscale images only\n	img1, img2 must be same size\n (by AJOGD @ DEC-2006)\n\nC++: mrpt::img::CImage::correlate(const class mrpt::img::CImage &, int, int) const --> float", pybind11::arg("img2int"), pybind11::arg("width_init"), pybind11::arg("height_init"));
		cl.def("cross_correlation_FFT", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, class mrpt::math::CMatrixDynamic<float> & a1) -> void { return o.cross_correlation_FFT(a0, a1); }, "", pybind11::arg("in_img"), pybind11::arg("out_corr"));
		cl.def("cross_correlation_FFT", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, class mrpt::math::CMatrixDynamic<float> & a1, int const & a2) -> void { return o.cross_correlation_FFT(a0, a1, a2); }, "", pybind11::arg("in_img"), pybind11::arg("out_corr"), pybind11::arg("u_search_ini"));
		cl.def("cross_correlation_FFT", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, class mrpt::math::CMatrixDynamic<float> & a1, int const & a2, int const & a3) -> void { return o.cross_correlation_FFT(a0, a1, a2, a3); }, "", pybind11::arg("in_img"), pybind11::arg("out_corr"), pybind11::arg("u_search_ini"), pybind11::arg("v_search_ini"));
		cl.def("cross_correlation_FFT", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, class mrpt::math::CMatrixDynamic<float> & a1, int const & a2, int const & a3, int const & a4) -> void { return o.cross_correlation_FFT(a0, a1, a2, a3, a4); }, "", pybind11::arg("in_img"), pybind11::arg("out_corr"), pybind11::arg("u_search_ini"), pybind11::arg("v_search_ini"), pybind11::arg("u_search_size"));
		cl.def("cross_correlation_FFT", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, class mrpt::math::CMatrixDynamic<float> & a1, int const & a2, int const & a3, int const & a4, int const & a5) -> void { return o.cross_correlation_FFT(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("in_img"), pybind11::arg("out_corr"), pybind11::arg("u_search_ini"), pybind11::arg("v_search_ini"), pybind11::arg("u_search_size"), pybind11::arg("v_search_size"));
		cl.def("cross_correlation_FFT", [](mrpt::img::CImage const &o, const class mrpt::img::CImage & a0, class mrpt::math::CMatrixDynamic<float> & a1, int const & a2, int const & a3, int const & a4, int const & a5, float const & a6) -> void { return o.cross_correlation_FFT(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("in_img"), pybind11::arg("out_corr"), pybind11::arg("u_search_ini"), pybind11::arg("v_search_ini"), pybind11::arg("u_search_size"), pybind11::arg("v_search_size"), pybind11::arg("biasThisImg"));
		cl.def("cross_correlation_FFT", (void (mrpt::img::CImage::*)(const class mrpt::img::CImage &, class mrpt::math::CMatrixDynamic<float> &, int, int, int, int, float, float) const) &mrpt::img::CImage::cross_correlation_FFT, "	Computes the correlation matrix between this image and another one.\n   This implementation uses the 2D FFT for achieving reduced computation\n time.\n \n\n The \"patch\" image, which must be equal, or smaller than\n \"this\" image. This function supports gray-scale (1 channel only) images.\n \n\n The \"x\" coordinate of the search window.\n \n\n The \"y\" coordinate of the search window.\n \n\n The width of the search window.\n \n\n The height of the search window.\n \n\n The output for the correlation matrix, which will be\n \"u_search_size\" x \"v_search_size\"\n \n\n This optional parameter is a fixed \"bias\" value to be\n substracted to the pixels of \"this\" image before performing correlation.\n \n\n This optional parameter is a fixed \"bias\" value to be\n substracted to the pixels of \"in_img\" image before performing\n correlation. Note: By default, the search area is the whole (this) image.\n (by JLBC @ JAN-2006)\n \n\n cross_correlation\n\nC++: mrpt::img::CImage::cross_correlation_FFT(const class mrpt::img::CImage &, class mrpt::math::CMatrixDynamic<float> &, int, int, int, int, float, float) const --> void", pybind11::arg("in_img"), pybind11::arg("out_corr"), pybind11::arg("u_search_ini"), pybind11::arg("v_search_ini"), pybind11::arg("u_search_size"), pybind11::arg("v_search_size"), pybind11::arg("biasThisImg"), pybind11::arg("biasInImg"));
		cl.def("normalize", (void (mrpt::img::CImage::*)()) &mrpt::img::CImage::normalize, "Optimize the brightness range of an image without using histogram\n Only for one channel images.\n \n\n equalizeHist\n\nC++: mrpt::img::CImage::normalize() --> void");
		cl.def("flipVertical", (void (mrpt::img::CImage::*)()) &mrpt::img::CImage::flipVertical, "Flips the image vertically. \n swapRB(), flipHorizontal() \n\nC++: mrpt::img::CImage::flipVertical() --> void");
		cl.def("flipHorizontal", (void (mrpt::img::CImage::*)()) &mrpt::img::CImage::flipHorizontal, "Flips the image horizontally \n swapRB(), flipVertical() \n\nC++: mrpt::img::CImage::flipHorizontal() --> void");
		cl.def("swapRB", (void (mrpt::img::CImage::*)()) &mrpt::img::CImage::swapRB, "Swaps red and blue channels. \n\nC++: mrpt::img::CImage::swapRB() --> void");
		cl.def("undistort", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, const class mrpt::img::TCamera &) const) &mrpt::img::CImage::undistort, "Undistort the image according to some camera parameters, and\n returns an output undistorted image.\n \n\n The output undistorted image\n \n\n The input camera params (containing the intrinsic\n and distortion parameters of the camera)\n \n\n The intrinsic parameters (fx,fy,cx,cy) of the output image are the\n same than in the input image.\n \n\n mrpt::vision::CUndistortMap\n\nC++: mrpt::img::CImage::undistort(class mrpt::img::CImage &, const class mrpt::img::TCamera &) const --> void", pybind11::arg("out_img"), pybind11::arg("cameraParams"));
		cl.def("rectifyImageInPlace", (void (mrpt::img::CImage::*)(void *, void *)) &mrpt::img::CImage::rectifyImageInPlace, "Rectify an image (undistorts and rectification) from a stereo pair\n according to a pair of precomputed rectification maps\n \n\n mapY   [IN] The pre-computed maps of the rectification\n (should be computed beforehand)\n \n\n mrpt::vision::CStereoRectifyMap,\n mrpt::vision::computeStereoRectificationMaps\n\nC++: mrpt::img::CImage::rectifyImageInPlace(void *, void *) --> void", pybind11::arg("mapX"), pybind11::arg("mapY"));
		cl.def("filterMedian", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0) -> void { return o.filterMedian(a0); }, "", pybind11::arg("out_img"));
		cl.def("filterMedian", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, int) const) &mrpt::img::CImage::filterMedian, "Filter the image with a Median filter with a window size WxW, returning\n the filtered image in out_img. For inplace operation, set out_img to\n this. \n\nC++: mrpt::img::CImage::filterMedian(class mrpt::img::CImage &, int) const --> void", pybind11::arg("out_img"), pybind11::arg("W"));
		cl.def("filterGaussian", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0) -> void { return o.filterGaussian(a0); }, "", pybind11::arg("out_img"));
		cl.def("filterGaussian", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, int const & a1) -> void { return o.filterGaussian(a0, a1); }, "", pybind11::arg("out_img"), pybind11::arg("W"));
		cl.def("filterGaussian", [](mrpt::img::CImage const &o, class mrpt::img::CImage & a0, int const & a1, int const & a2) -> void { return o.filterGaussian(a0, a1, a2); }, "", pybind11::arg("out_img"), pybind11::arg("W"), pybind11::arg("H"));
		cl.def("filterGaussian", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &, int, int, double) const) &mrpt::img::CImage::filterGaussian, "Filter the image with a Gaussian filter with a window size WxH,\n replacing \"this\" image by the filtered one. For inplace operation, set\n out_img to this. \n\nC++: mrpt::img::CImage::filterGaussian(class mrpt::img::CImage &, int, int, double) const --> void", pybind11::arg("out_img"), pybind11::arg("W"), pybind11::arg("H"), pybind11::arg("sigma"));
		cl.def("joinImagesHorz", (void (mrpt::img::CImage::*)(const class mrpt::img::CImage &, const class mrpt::img::CImage &)) &mrpt::img::CImage::joinImagesHorz, "Joins two images side-by-side horizontally. Both images must have the\n same number of rows and be of the same type (i.e. depth and color mode)\n\n \n [IN] The first image.\n \n\n [IN] The other image.\n\nC++: mrpt::img::CImage::joinImagesHorz(const class mrpt::img::CImage &, const class mrpt::img::CImage &) --> void", pybind11::arg("im1"), pybind11::arg("im2"));
		cl.def("KLT_response", (float (mrpt::img::CImage::*)(const unsigned int, const unsigned int, const unsigned int) const) &mrpt::img::CImage::KLT_response, "Compute the KLT response at a given pixel (x,y) - Only for grayscale\n images (for efficiency it avoids converting to grayscale internally).\n  See KLT_response() for more details on the internal\n optimizations of this method, but this graph shows a general view:\n  \n\nC++: mrpt::img::CImage::KLT_response(const unsigned int, const unsigned int, const unsigned int) const --> float", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("half_window_size"));
		cl.def("makeShallowCopy", (class mrpt::img::CImage (mrpt::img::CImage::*)() const) &mrpt::img::CImage::makeShallowCopy, "Returns a shallow copy of the original image \n\nC++: mrpt::img::CImage::makeShallowCopy() const --> class mrpt::img::CImage");
		cl.def("makeDeepCopy", (class mrpt::img::CImage (mrpt::img::CImage::*)() const) &mrpt::img::CImage::makeDeepCopy, "Returns a deep copy of this image.\n If the image is externally-stored, there is no difference with a shallow\n copy. \n\n makeShallowCopy() \n\nC++: mrpt::img::CImage::makeDeepCopy() const --> class mrpt::img::CImage");
		cl.def("copyFromForceLoad", (void (mrpt::img::CImage::*)(const class mrpt::img::CImage &)) &mrpt::img::CImage::copyFromForceLoad, "Copies from another image (shallow copy), and, if it is externally\n stored, the image file will be actually loaded into memory in \"this\"\n object. \n\n operator = \n CExceptionExternalImageNotFound If the\n external image couldn't be loaded.\n\nC++: mrpt::img::CImage::copyFromForceLoad(const class mrpt::img::CImage &) --> void", pybind11::arg("o"));
		cl.def("swap", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &)) &mrpt::img::CImage::swap, "Efficiently swap of two images \n\nC++: mrpt::img::CImage::swap(class mrpt::img::CImage &) --> void", pybind11::arg("o"));
		cl.def("getAsFloat", (float (mrpt::img::CImage::*)(unsigned int, unsigned int, unsigned int) const) &mrpt::img::CImage::getAsFloat, "Returns the contents of a given pixel at the desired channel, in float\n format: [0,255]->[0,1]\n   The coordinate origin is pixel(0,0)=top-left corner of the image.\n \n\n std::exception On pixel coordinates out of bounds\n \n\n operator()\n\nC++: mrpt::img::CImage::getAsFloat(unsigned int, unsigned int, unsigned int) const --> float", pybind11::arg("col"), pybind11::arg("row"), pybind11::arg("channel"));
		cl.def("getAsFloat", (float (mrpt::img::CImage::*)(unsigned int, unsigned int) const) &mrpt::img::CImage::getAsFloat, "Returns the contents of a given pixel (for gray-scale images, in color\n images the gray scale equivalent is computed for the pixel), in float\n format: [0,255]->[0,1]\n   The coordinate origin is pixel(0,0)=top-left corner of the image.\n \n\n std::exception On pixel coordinates out of bounds\n \n\n operator()\n\nC++: mrpt::img::CImage::getAsFloat(unsigned int, unsigned int) const --> float", pybind11::arg("col"), pybind11::arg("row"));
		cl.def("__call__", [](mrpt::img::CImage const &o, unsigned int const & a0, unsigned int const & a1) -> unsigned char * { return o.operator()(a0, a1); }, "", pybind11::return_value_policy::automatic, pybind11::arg("col"), pybind11::arg("row"));
		cl.def("__call__", (unsigned char * (mrpt::img::CImage::*)(unsigned int, unsigned int, unsigned int) const) &mrpt::img::CImage::operator(), "Returns a pointer to a given pixel information.\n   The coordinate origin is pixel(0,0)=top-left corner of the image.\n \n\n std::exception On pixel coordinates out of bounds\n\nC++: mrpt::img::CImage::operator()(unsigned int, unsigned int, unsigned int) const --> unsigned char *", pybind11::return_value_policy::automatic, pybind11::arg("col"), pybind11::arg("row"), pybind11::arg("channel"));
		cl.def("getWidth", (size_t (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getWidth, "Returns the width of the image in pixels \n getSize \n\nC++: mrpt::img::CImage::getWidth() const --> size_t");
		cl.def("getHeight", (size_t (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getHeight, "Returns the height of the image in pixels \n getSize \n\nC++: mrpt::img::CImage::getHeight() const --> size_t");
		cl.def("getSize", (void (mrpt::img::CImage::*)(struct mrpt::img::TPixelCoord &) const) &mrpt::img::CImage::getSize, "Return the size of the image \n getWidth, getHeight \n\nC++: mrpt::img::CImage::getSize(struct mrpt::img::TPixelCoord &) const --> void", pybind11::arg("s"));
		cl.def("getSize", (struct mrpt::img::TPixelCoord (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getSize, "Return the size of the image \n getWidth, getHeight \n\nC++: mrpt::img::CImage::getSize() const --> struct mrpt::img::TPixelCoord");
		cl.def("getRowStride", (size_t (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getRowStride, "Returns the row stride of the image: this is the number of *bytes*\n between two consecutive rows. You can access the pointer to the first row\n with ptrLine(0)\n \n\n getSize, as, ptr, ptrLine \n\nC++: mrpt::img::CImage::getRowStride() const --> size_t");
		cl.def("getChannelsOrder", (std::string (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getChannelsOrder, "As of mrpt 2.0.0, this returns either \"GRAY\" or \"BGR\". \n\nC++: mrpt::img::CImage::getChannelsOrder() const --> std::string");
		cl.def("getMaxAsFloat", (float (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getMaxAsFloat, "Return the maximum pixel value of the image, as a float value in the\n range [0,1]\n \n\n getAsFloat \n\nC++: mrpt::img::CImage::getMaxAsFloat() const --> float");
		cl.def("channelCount", (int (mrpt::img::CImage::*)() const) &mrpt::img::CImage::channelCount, "Returns 1 (grayscale), 3 (RGB) or 4 (RGBA) [New in MRPT 2.1.1] \n\nC++: mrpt::img::CImage::channelCount() const --> int");
		cl.def("isColor", (bool (mrpt::img::CImage::*)() const) &mrpt::img::CImage::isColor, "Returns true if the image is RGB or RGBA, false if it is grayscale \n\nC++: mrpt::img::CImage::isColor() const --> bool");
		cl.def("isEmpty", (bool (mrpt::img::CImage::*)() const) &mrpt::img::CImage::isEmpty, "Returns true if the object is in the state after default constructor.\n Returns false for delay-loaded images, disregarding whether the image is\n actually on disk or memory.\n\nC++: mrpt::img::CImage::isEmpty() const --> bool");
		cl.def("isOriginTopLeft", (bool (mrpt::img::CImage::*)() const) &mrpt::img::CImage::isOriginTopLeft, "Returns true (as of MRPT v2.0.0, it's fixed) \n\nC++: mrpt::img::CImage::isOriginTopLeft() const --> bool");
		cl.def("getChannelCount", (enum mrpt::img::TImageChannels (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getChannelCount, "Returns the number of channels, typically 1 (GRAY) or 3 (RGB)\n \n\n isColor\n\nC++: mrpt::img::CImage::getChannelCount() const --> enum mrpt::img::TImageChannels");
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0) -> void { return o.getAsMatrix(a0); }, "", pybind11::arg("outMatrix"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, bool const & a1) -> void { return o.getAsMatrix(a0, a1); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, bool const & a1, int const & a2) -> void { return o.getAsMatrix(a0, a1, a2); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, bool const & a1, int const & a2, int const & a3) -> void { return o.getAsMatrix(a0, a1, a2, a3); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, bool const & a1, int const & a2, int const & a3, int const & a4) -> void { return o.getAsMatrix(a0, a1, a2, a3, a4); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, bool const & a1, int const & a2, int const & a3, int const & a4, int const & a5) -> void { return o.getAsMatrix(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"), pybind11::arg("y_max"));
		cl.def("getAsMatrix", (void (mrpt::img::CImage::*)(class mrpt::math::CMatrixDynamic<float> &, bool, int, int, int, int, bool) const) &mrpt::img::CImage::getAsMatrix, "	Returns the image as a matrix with pixel grayscale values in the range\n [0,1]. Matrix indexes in this order: M(row,column)\n  \n\n If set to true (default), the output matrix will be\n always the size of the image at output. If set to false, the matrix will\n be enlarged to the size of the image, but it will not be cropped if it\n has room enough (useful for FFT2D,...)\n  \n\n The starting \"x\" coordinate to extract (default=0=the\n first column)\n  \n\n The starting \"y\" coordinate to extract (default=0=the\n first row)\n  \n\n The final \"x\" coordinate (inclusive) to extract\n (default=-1=the last column)\n  \n\n The final \"y\" coordinate (inclusive) to extract\n (default=-1=the last row)\n \n\n Normalize the image values such that they fall in the\n range [0,1] (default: true). If set to false, the matrix will hold\n numbers in the range [0,255]. \n\n setFromMatrix\n\nC++: mrpt::img::CImage::getAsMatrix(class mrpt::math::CMatrixDynamic<float> &, bool, int, int, int, int, bool) const --> void", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"), pybind11::arg("y_max"), pybind11::arg("normalize_01"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0) -> void { return o.getAsMatrix(a0); }, "", pybind11::arg("outMatrix"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, bool const & a1) -> void { return o.getAsMatrix(a0, a1); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, bool const & a1, int const & a2) -> void { return o.getAsMatrix(a0, a1, a2); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, bool const & a1, int const & a2, int const & a3) -> void { return o.getAsMatrix(a0, a1, a2, a3); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"));
		cl.def("getAsMatrix", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, bool const & a1, int const & a2, int const & a3, int const & a4) -> void { return o.getAsMatrix(a0, a1, a2, a3, a4); }, "", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"));
		cl.def("getAsMatrix", (void (mrpt::img::CImage::*)(class mrpt::math::CMatrixDynamic<unsigned char> &, bool, int, int, int, int) const) &mrpt::img::CImage::getAsMatrix, "C++: mrpt::img::CImage::getAsMatrix(class mrpt::math::CMatrixDynamic<unsigned char> &, bool, int, int, int, int) const --> void", pybind11::arg("outMatrix"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"), pybind11::arg("y_max"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, class mrpt::math::CMatrixDynamic<float> & a1, class mrpt::math::CMatrixDynamic<float> & a2) -> void { return o.getAsRGBMatrices(a0, a1, a2); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, class mrpt::math::CMatrixDynamic<float> & a1, class mrpt::math::CMatrixDynamic<float> & a2, bool const & a3) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, class mrpt::math::CMatrixDynamic<float> & a1, class mrpt::math::CMatrixDynamic<float> & a2, bool const & a3, int const & a4) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3, a4); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, class mrpt::math::CMatrixDynamic<float> & a1, class mrpt::math::CMatrixDynamic<float> & a2, bool const & a3, int const & a4, int const & a5) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<float> & a0, class mrpt::math::CMatrixDynamic<float> & a1, class mrpt::math::CMatrixDynamic<float> & a2, bool const & a3, int const & a4, int const & a5, int const & a6) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"));
		cl.def("getAsRGBMatrices", (void (mrpt::img::CImage::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, bool, int, int, int, int) const) &mrpt::img::CImage::getAsRGBMatrices, "	Returns the image as RGB matrices with pixel values in the range [0,1].\n Matrix indexes in this order: M(row,column)\n  \n\n If set to true (default), the output matrix will be\n always the size of the image at output. If set to false, the matrix will\n be enlarged to the size of the image, but it will not be cropped if it\n has room enough (useful for FFT2D,...)\n  \n\n The starting \"x\" coordinate to extract (default=0=the\n first column)\n  \n\n The starting \"y\" coordinate to extract (default=0=the\n first row)\n  \n\n The final \"x\" coordinate (inclusive) to extract\n (default=-1=the last column)\n  \n\n The final \"y\" coordinate (inclusive) to extract\n (default=-1=the last row)\n \n\n setFromRGBMatrices\n\nC++: mrpt::img::CImage::getAsRGBMatrices(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, bool, int, int, int, int) const --> void", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"), pybind11::arg("y_max"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, class mrpt::math::CMatrixDynamic<unsigned char> & a1, class mrpt::math::CMatrixDynamic<unsigned char> & a2) -> void { return o.getAsRGBMatrices(a0, a1, a2); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, class mrpt::math::CMatrixDynamic<unsigned char> & a1, class mrpt::math::CMatrixDynamic<unsigned char> & a2, bool const & a3) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, class mrpt::math::CMatrixDynamic<unsigned char> & a1, class mrpt::math::CMatrixDynamic<unsigned char> & a2, bool const & a3, int const & a4) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3, a4); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, class mrpt::math::CMatrixDynamic<unsigned char> & a1, class mrpt::math::CMatrixDynamic<unsigned char> & a2, bool const & a3, int const & a4, int const & a5) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"));
		cl.def("getAsRGBMatrices", [](mrpt::img::CImage const &o, class mrpt::math::CMatrixDynamic<unsigned char> & a0, class mrpt::math::CMatrixDynamic<unsigned char> & a1, class mrpt::math::CMatrixDynamic<unsigned char> & a2, bool const & a3, int const & a4, int const & a5, int const & a6) -> void { return o.getAsRGBMatrices(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"));
		cl.def("getAsRGBMatrices", (void (mrpt::img::CImage::*)(class mrpt::math::CMatrixDynamic<unsigned char> &, class mrpt::math::CMatrixDynamic<unsigned char> &, class mrpt::math::CMatrixDynamic<unsigned char> &, bool, int, int, int, int) const) &mrpt::img::CImage::getAsRGBMatrices, "C++: mrpt::img::CImage::getAsRGBMatrices(class mrpt::math::CMatrixDynamic<unsigned char> &, class mrpt::math::CMatrixDynamic<unsigned char> &, class mrpt::math::CMatrixDynamic<unsigned char> &, bool, int, int, int, int) const --> void", pybind11::arg("outMatrixR"), pybind11::arg("outMatrixG"), pybind11::arg("outMatrixB"), pybind11::arg("doResize"), pybind11::arg("x_min"), pybind11::arg("y_min"), pybind11::arg("x_max"), pybind11::arg("y_max"));
		cl.def("getAsMatrixTiled", (void (mrpt::img::CImage::*)(class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::img::CImage::getAsMatrixTiled, "	Returns the image as a matrix, where the image is \"tiled\" (repeated)\n the required number of times to fill the entire size of the matrix on\n input.\n\nC++: mrpt::img::CImage::getAsMatrixTiled(class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("outMatrix"));
		cl.def("setExternalStorage", (void (mrpt::img::CImage::*)(const std::string &)) &mrpt::img::CImage::setExternalStorage, "By using this method the image is marked as referenced to an external\n file, which will be loaded only under demand.\n   A CImage with external storage does not consume memory until some\n method trying to access the image is invoked (e.g. getWidth(),\n isColor(),...)\n   At any moment, the image can be unloaded from memory again by invoking\n unload.\n   An image becomes of type \"external storage\" only through calling\n setExternalStorage. This property remains after serializing the object.\n   File names can be absolute, or relative to the\n CImage::getImagesPathBase() directory. Filenames staring with \"X:\\\" or\n \"/\"\n are considered absolute paths.\n   By calling this method the current contents of the image are NOT saved\n to that file, because this method can be also called\n    to let the object know where to load the image in case its contents\n are required. Thus, for saving images in this format (not when loading)\n    the proper order of commands should be:\n   \n\n\n\n\n   \n Modifications to the memory copy of the image are not\n automatically saved to disk.\n  \n\n unload, isExternallyStored\n\nC++: mrpt::img::CImage::setExternalStorage(const std::string &) --> void", pybind11::arg("fileName"));
		cl.def_static("getImagesPathBase", (const std::string & (*)()) &mrpt::img::CImage::getImagesPathBase, "By default, \".\"  \n setExternalStorage\n  \n\n Since MRPT 2.3.3 this is a synonym\n        with mrpt::io::getLazyLoadPathBase()\n\nC++: mrpt::img::CImage::getImagesPathBase() --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def_static("setImagesPathBase", (void (*)(const std::string &)) &mrpt::img::CImage::setImagesPathBase, "Since MRPT 2.3.3 this is a synonym\n        with mrpt::io::setLazyLoadPathBase()\n\nC++: mrpt::img::CImage::setImagesPathBase(const std::string &) --> void", pybind11::arg("path"));
		cl.def("isExternallyStored", (bool (mrpt::img::CImage::*)() const) &mrpt::img::CImage::isExternallyStored, "See setExternalStorage(). \n\nC++: mrpt::img::CImage::isExternallyStored() const --> bool");
		cl.def("getExternalStorageFile", (std::string (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getExternalStorageFile, "Only if isExternallyStored() returns true. \n\n getExternalStorageFileAbsolutePath \n\nC++: mrpt::img::CImage::getExternalStorageFile() const --> std::string");
		cl.def("getExternalStorageFileAbsolutePath", (void (mrpt::img::CImage::*)(std::string &) const) &mrpt::img::CImage::getExternalStorageFileAbsolutePath, "Only if isExternallyStored() returns true. \n getExternalStorageFile \n\nC++: mrpt::img::CImage::getExternalStorageFileAbsolutePath(std::string &) const --> void", pybind11::arg("out_path"));
		cl.def("getExternalStorageFileAbsolutePath", (std::string (mrpt::img::CImage::*)() const) &mrpt::img::CImage::getExternalStorageFileAbsolutePath, "Only if isExternallyStored() returns true. \n getExternalStorageFile \n\nC++: mrpt::img::CImage::getExternalStorageFileAbsolutePath() const --> std::string");
		cl.def("forceLoad", (void (mrpt::img::CImage::*)() const) &mrpt::img::CImage::forceLoad, "For external storage image objects only, this method makes sure the\n image is loaded in memory. Note that usually images are loaded on-the-fly\n on first access and there's no need to call this.\n \n\nC++: mrpt::img::CImage::forceLoad() const --> void");
		cl.def("unload", (void (mrpt::img::CImage::*)() const) &mrpt::img::CImage::unload, "For external storage image objects only, this method unloads the image\n from memory (or does nothing if already unloaded).\n  It does not need to be called explicitly, unless the user wants to save\n memory for images that will not be used often.\n  If called for an image without the flag \"external storage\", it is\n simply ignored.\n \n\n setExternalStorage, forceLoad\n\nC++: mrpt::img::CImage::unload() const --> void");
		cl.def("loadFromMemoryBuffer", [](mrpt::img::CImage &o, unsigned int const & a0, unsigned int const & a1, bool const & a2, unsigned char * a3) -> void { return o.loadFromMemoryBuffer(a0, a1, a2, a3); }, "", pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("color"), pybind11::arg("rawpixels"));
		cl.def("loadFromMemoryBuffer", (void (mrpt::img::CImage::*)(unsigned int, unsigned int, bool, unsigned char *, bool)) &mrpt::img::CImage::loadFromMemoryBuffer, "Reads the image from raw pixels buffer in memory.\n\nC++: mrpt::img::CImage::loadFromMemoryBuffer(unsigned int, unsigned int, bool, unsigned char *, bool) --> void", pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("color"), pybind11::arg("rawpixels"), pybind11::arg("swapRedBlue"));
		cl.def("loadFromMemoryBuffer", (void (mrpt::img::CImage::*)(unsigned int, unsigned int, unsigned int, unsigned char *, unsigned char *, unsigned char *)) &mrpt::img::CImage::loadFromMemoryBuffer, "Reads a color image from three raw pixels buffers in memory.\n bytesPerRow is the number of bytes per row per channel, i.e. the row\n increment.\n\nC++: mrpt::img::CImage::loadFromMemoryBuffer(unsigned int, unsigned int, unsigned int, unsigned char *, unsigned char *, unsigned char *) --> void", pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("bytesPerRow"), pybind11::arg("red"), pybind11::arg("green"), pybind11::arg("blue"));
		cl.def("loadFromStreamAsJPEG", (void (mrpt::img::CImage::*)(class mrpt::io::CStream &)) &mrpt::img::CImage::loadFromStreamAsJPEG, "Reads the image from a binary stream containing a binary jpeg file.\n \n\n std::exception On pixel coordinates out of bounds\n\nC++: mrpt::img::CImage::loadFromStreamAsJPEG(class mrpt::io::CStream &) --> void", pybind11::arg("in"));
		cl.def("loadFromFile", [](mrpt::img::CImage &o, const std::string & a0) -> bool { return o.loadFromFile(a0); }, "", pybind11::arg("fileName"));
		cl.def("loadFromFile", (bool (mrpt::img::CImage::*)(const std::string &, int)) &mrpt::img::CImage::loadFromFile, "Load image from a file, whose format is determined from the extension\n (internally uses OpenCV).\n \n\n The file to read from.\n \n\n Specifies colorness of the loaded image:\n  - if ==1, the loaded image is forced to be color 3-channel image;\n  - if ==0, the loaded image is forced to be grayscale;\n  - if ==-1, the loaded image will be loaded as is (with number of\n channels depends on the file). The supported formats are:\n\n - Windows bitmaps - BMP, DIB;\n - JPEG files - JPEG, JPG, JPE;\n - Portable Network Graphics - PNG;\n - Portable image format - PBM, PGM, PPM;\n - Sun rasters - SR, RAS;\n - TIFF files - TIFF, TIF.\n\n MRPT also provides the special loaders loadFromXPM() and loadTGA().\n\n Note that this function uses cv::imdecode() internally to reuse the\n memory buffer used by the image already loaded into this CImage, if\n possible, minimizing the number of memory allocations.\n\n \n False on any error\n \n\n saveToFile, setExternalStorage,loadFromXPM, loadTGA\n\nC++: mrpt::img::CImage::loadFromFile(const std::string &, int) --> bool", pybind11::arg("fileName"), pybind11::arg("isColor"));
		cl.def_static("LoadFromFile", [](const std::string & a0) -> mrpt::img::CImage { return mrpt::img::CImage::LoadFromFile(a0); }, "", pybind11::arg("fileName"));
		cl.def_static("LoadFromFile", (class mrpt::img::CImage (*)(const std::string &, int)) &mrpt::img::CImage::LoadFromFile, "Static method to construct an CImage object from a file.\n See CImage::loadFromFile() for meaning of parameters.\n\n \n std::exception On load error.\n \n\n New in MRPT 2.4.2\n\nC++: mrpt::img::CImage::LoadFromFile(const std::string &, int) --> class mrpt::img::CImage", pybind11::arg("fileName"), pybind11::arg("isColor"));
		cl.def_static("loadTGA", (bool (*)(const std::string &, class mrpt::img::CImage &, class mrpt::img::CImage &)) &mrpt::img::CImage::loadTGA, "Loads a TGA true-color RGBA image as two CImage objects, one for the RGB\n channels plus a separate gray-level image with A channel.\n \n\n true on success\n\nC++: mrpt::img::CImage::loadTGA(const std::string &, class mrpt::img::CImage &, class mrpt::img::CImage &) --> bool", pybind11::arg("fileName"), pybind11::arg("out_RGB"), pybind11::arg("out_alpha"));
		cl.def("saveToFile", [](mrpt::img::CImage const &o, const std::string & a0) -> bool { return o.saveToFile(a0); }, "", pybind11::arg("fileName"));
		cl.def("saveToFile", (bool (mrpt::img::CImage::*)(const std::string &, int) const) &mrpt::img::CImage::saveToFile, "Save the image to a file, whose format is determined from the extension\n (internally uses OpenCV).\n \n\n The file to write to.\n\n The supported formats are:\n\n - Windows bitmaps - BMP, DIB;\n - JPEG files - JPEG, JPG, JPE;\n - Portable Network Graphics - PNG;\n - Portable image format - PBM, PGM, PPM;\n - Sun rasters - SR, RAS;\n - TIFF files - TIFF, TIF.\n\n \n Only for JPEG files, the quality of the compression\n in the range [0-100]. Larger is better quality but slower.\n \n\n jpeg_quality is only effective if MRPT is compiled against OpenCV\n 1.1.0 or newer.\n \n\n False on any error\n \n\n loadFromFile\n\nC++: mrpt::img::CImage::saveToFile(const std::string &, int) const --> bool", pybind11::arg("fileName"), pybind11::arg("jpeg_quality"));
		cl.def("saveToStreamAsJPEG", [](mrpt::img::CImage const &o, class mrpt::io::CStream & a0) -> void { return o.saveToStreamAsJPEG(a0); }, "", pybind11::arg("out"));
		cl.def("saveToStreamAsJPEG", (void (mrpt::img::CImage::*)(class mrpt::io::CStream &, const int) const) &mrpt::img::CImage::saveToStreamAsJPEG, "Save image to binary stream as a JPEG (.jpg) compressed format.\n \n\n std::exception On number of rows or cols equal to zero or\n other errors.\n \n\n saveToJPEG\n\nC++: mrpt::img::CImage::saveToStreamAsJPEG(class mrpt::io::CStream &, const int) const --> void", pybind11::arg("out"), pybind11::arg("jpeg_quality"));
		cl.def("grayscale", (class mrpt::img::CImage (mrpt::img::CImage::*)() const) &mrpt::img::CImage::grayscale, "Returns a grayscale version of the image, or a shallow copy of itself if\n it is already a grayscale image.\n\nC++: mrpt::img::CImage::grayscale() const --> class mrpt::img::CImage");
		cl.def("grayscale", (bool (mrpt::img::CImage::*)(class mrpt::img::CImage &) const) &mrpt::img::CImage::grayscale, "In-place is supported by setting `ret=*this`.\n \n\n true if SSE2 version has been run (or if the image was already\n grayscale)\n\nC++: mrpt::img::CImage::grayscale(class mrpt::img::CImage &) const --> bool", pybind11::arg("ret"));
		cl.def("colorImage", (class mrpt::img::CImage (mrpt::img::CImage::*)() const) &mrpt::img::CImage::colorImage, "Returns a color (RGB) version of the grayscale image, or a shallow copy\n of itself if it is already a color image.\n \n\n grayscale\n\nC++: mrpt::img::CImage::colorImage() const --> class mrpt::img::CImage");
		cl.def("colorImage", (void (mrpt::img::CImage::*)(class mrpt::img::CImage &) const) &mrpt::img::CImage::colorImage, "In-place is supported by setting `ret=*this`. \n\nC++: mrpt::img::CImage::colorImage(class mrpt::img::CImage &) const --> void", pybind11::arg("ret"));
		cl.def("assign", (class mrpt::img::CImage & (mrpt::img::CImage::*)(const class mrpt::img::CImage &)) &mrpt::img::CImage::operator=, "C++: mrpt::img::CImage::operator=(const class mrpt::img::CImage &) --> class mrpt::img::CImage &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
