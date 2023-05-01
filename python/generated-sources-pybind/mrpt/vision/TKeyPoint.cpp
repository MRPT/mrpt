#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/TKeyPoint.h>
#include <mrpt/vision/types.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

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

// mrpt::vision::CFeature file:mrpt/vision/CFeature.h line:54
struct PyCallBack_mrpt_vision_CFeature : public mrpt::vision::CFeature {
	using mrpt::vision::CFeature::CFeature;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::CFeature *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CFeature::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::CFeature *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CFeature::clone();
	}
	unsigned char serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::CFeature *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned char>::value) {
				static pybind11::detail::override_caster_t<unsigned char> caster;
				return pybind11::detail::cast_ref<unsigned char>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned char>(std::move(o));
		}
		return CFeature::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::CFeature *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFeature::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, unsigned char a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::CFeature *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFeature::serializeFrom(a0, a1);
	}
};

void bind_mrpt_vision_TKeyPoint(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::vision::TKeyPoint_templ file:mrpt/vision/TKeyPoint.h line:29
		pybind11::class_<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>, std::shared_ptr<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>>> cl(M("mrpt::vision"), "TKeyPoint_templ_mrpt_img_TPixelCoordf_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>(); } ) );
		cl.def( pybind11::init( [](mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf> const &o){ return new mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>(o); } ) );
		cl.def_readwrite("pt", &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::pt);
		cl.def_readwrite("ID", &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::ID);
		cl.def_readwrite("track_status", &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::track_status);
		cl.def_readwrite("response", &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::response);
		cl.def_readwrite("octave", &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::octave);
		cl.def_readwrite("user_flags", &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::user_flags);
		cl.def("assign", (struct mrpt::vision::TKeyPoint_templ<struct mrpt::img::TPixelCoordf> & (mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::*)(const struct mrpt::vision::TKeyPoint_templ<struct mrpt::img::TPixelCoordf> &)) &mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::operator=, "C++: mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>::operator=(const struct mrpt::vision::TKeyPoint_templ<struct mrpt::img::TPixelCoordf> &) --> struct mrpt::vision::TKeyPoint_templ<struct mrpt::img::TPixelCoordf> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::TKeyPointTraits file:mrpt/vision/TKeyPoint.h line:88
		pybind11::class_<mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoord>>, std::shared_ptr<mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoord>>>> cl(M("mrpt::vision"), "TKeyPointTraits_mrpt_vision_TKeyPoint_templ_mrpt_img_TPixelCoord_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoord>>(); } ) );
		cl.def_static("f2coord", (int (*)(float)) &mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoord>>::f2coord, "C++: mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoord>>::f2coord(float) --> int", pybind11::arg("f"));
	}
	{ // mrpt::vision::TKeyPointTraits file:mrpt/vision/TKeyPoint.h line:96
		pybind11::class_<mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>>, std::shared_ptr<mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>>>> cl(M("mrpt::vision"), "TKeyPointTraits_mrpt_vision_TKeyPoint_templ_mrpt_img_TPixelCoordf_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>>(); } ) );
		cl.def_static("f2coord", (float (*)(float)) &mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>>::f2coord, "C++: mrpt::vision::TKeyPointTraits<mrpt::vision::TKeyPoint_templ<mrpt::img::TPixelCoordf>>::f2coord(float) --> float", pybind11::arg("f"));
	}
	// mrpt::vision::TListIdx file:mrpt/vision/CFeature.h line:27
	pybind11::enum_<mrpt::vision::TListIdx>(M("mrpt::vision"), "TListIdx", pybind11::arithmetic(), "")
		.value("firstList", mrpt::vision::firstList)
		.value("secondList", mrpt::vision::secondList)
		.value("bothLists", mrpt::vision::bothLists)
		.export_values();

;

	{ // mrpt::vision::CFeature file:mrpt/vision/CFeature.h line:54
		pybind11::class_<mrpt::vision::CFeature, std::shared_ptr<mrpt::vision::CFeature>, PyCallBack_mrpt_vision_CFeature, mrpt::serialization::CSerializable> cl(M("mrpt::vision"), "CFeature", "A generic 2D feature from an image, extracted with \n Each feature may have one or more descriptors (see  in\n addition to an image patch.\n The (Euclidean) distance between descriptors in a pair of features can be\n computed with  descriptorDistanceTo,\n  while the similarity of the patches is given by patchCorrelationTo.\n\n  \n CFeatureList, TKeyPoint, TKeyPointList");
		cl.def( pybind11::init( [](){ return new mrpt::vision::CFeature(); }, [](){ return new PyCallBack_mrpt_vision_CFeature(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_vision_CFeature const &o){ return new PyCallBack_mrpt_vision_CFeature(o); } ) );
		cl.def( pybind11::init( [](mrpt::vision::CFeature const &o){ return new mrpt::vision::CFeature(o); } ) );
		cl.def_readwrite("keypoint", &mrpt::vision::CFeature::keypoint);
		cl.def_readwrite("patch", &mrpt::vision::CFeature::patch);
		cl.def_readwrite("patchSize", &mrpt::vision::CFeature::patchSize);
		cl.def_readwrite("type", &mrpt::vision::CFeature::type);
		cl.def_readwrite("track_status", &mrpt::vision::CFeature::track_status);
		cl.def_readwrite("response", &mrpt::vision::CFeature::response);
		cl.def_readwrite("orientation", &mrpt::vision::CFeature::orientation);
		cl.def_readwrite("user_flags", &mrpt::vision::CFeature::user_flags);
		cl.def_readwrite("depth", &mrpt::vision::CFeature::depth);
		cl.def_readwrite("initialDepth", &mrpt::vision::CFeature::initialDepth);
		cl.def_readwrite("p3D", &mrpt::vision::CFeature::p3D);
		cl.def_readwrite("descriptors", &mrpt::vision::CFeature::descriptors);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<22> (*)()) &mrpt::vision::CFeature::getClassName, "C++: mrpt::vision::CFeature::getClassName() --> class mrpt::typemeta::string_literal<22>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::vision::CFeature::GetRuntimeClassIdStatic, "C++: mrpt::vision::CFeature::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::vision::CFeature::*)() const) &mrpt::vision::CFeature::GetRuntimeClass, "C++: mrpt::vision::CFeature::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::vision::CFeature::*)() const) &mrpt::vision::CFeature::clone, "C++: mrpt::vision::CFeature::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::vision::CFeature::CreateObject, "C++: mrpt::vision::CFeature::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("isPointFeature", (bool (mrpt::vision::CFeature::*)() const) &mrpt::vision::CFeature::isPointFeature, "Return false only for Blob detectors (SIFT, SURF) \n\nC++: mrpt::vision::CFeature::isPointFeature() const --> bool");
		cl.def("getFirstDescriptorAsMatrix", (bool (mrpt::vision::CFeature::*)(class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::vision::CFeature::getFirstDescriptorAsMatrix, "Return the first found descriptor, as a matrix.\n \n\n false on error, i.e. there is no valid descriptor.\n\nC++: mrpt::vision::CFeature::getFirstDescriptorAsMatrix(class mrpt::math::CMatrixDynamic<float> &) const --> bool", pybind11::arg("desc"));
		cl.def("patchCorrelationTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &) const) &mrpt::vision::CFeature::patchCorrelationTo, "Computes the normalized cross-correlation between the patches of this\n and another feature (normalized in the range [0,1], such as 0=best,\n 1=worst).\n  \n\n If this or the other features does not have patches or they are\n of different sizes, an exception will be raised.\n \n\n descriptorDistanceTo\n\nC++: mrpt::vision::CFeature::patchCorrelationTo(const class mrpt::vision::CFeature &) const --> float", pybind11::arg("oFeature"));
		cl.def("descriptorDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0) -> float { return o.descriptorDistanceTo(a0); }, "", pybind11::arg("oFeature"));
		cl.def("descriptorDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0, enum mrpt::vision::TDescriptorType const & a1) -> float { return o.descriptorDistanceTo(a0, a1); }, "", pybind11::arg("oFeature"), pybind11::arg("descriptorToUse"));
		cl.def("descriptorDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, enum mrpt::vision::TDescriptorType, bool) const) &mrpt::vision::CFeature::descriptorDistanceTo, "Computes the Euclidean Distance between this feature's and other\n feature's descriptors, using the given descriptor or the first present\n one.\n  \n\n If descriptorToUse is not descAny and that descriptor is not\n present in one of the features, an exception will be raised.\n \n\n patchCorrelationTo\n\nC++: mrpt::vision::CFeature::descriptorDistanceTo(const class mrpt::vision::CFeature &, enum mrpt::vision::TDescriptorType, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("descriptorToUse"), pybind11::arg("normalize_distances"));
		cl.def("descriptorSIFTDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0) -> float { return o.descriptorSIFTDistanceTo(a0); }, "", pybind11::arg("oFeature"));
		cl.def("descriptorSIFTDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, bool) const) &mrpt::vision::CFeature::descriptorSIFTDistanceTo, "Computes the Euclidean Distance between \"this\" and the \"other\"\n descriptors \n\nC++: mrpt::vision::CFeature::descriptorSIFTDistanceTo(const class mrpt::vision::CFeature &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("normalize_distances"));
		cl.def("descriptorSURFDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0) -> float { return o.descriptorSURFDistanceTo(a0); }, "", pybind11::arg("oFeature"));
		cl.def("descriptorSURFDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, bool) const) &mrpt::vision::CFeature::descriptorSURFDistanceTo, "Computes the Euclidean Distance between \"this\" and the \"other\"\n descriptors \n\nC++: mrpt::vision::CFeature::descriptorSURFDistanceTo(const class mrpt::vision::CFeature &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("normalize_distances"));
		cl.def("descriptorSpinImgDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0) -> float { return o.descriptorSpinImgDistanceTo(a0); }, "", pybind11::arg("oFeature"));
		cl.def("descriptorSpinImgDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, bool) const) &mrpt::vision::CFeature::descriptorSpinImgDistanceTo, "Computes the Euclidean Distance between \"this\" and the \"other\"\n descriptors \n\nC++: mrpt::vision::CFeature::descriptorSpinImgDistanceTo(const class mrpt::vision::CFeature &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("normalize_distances"));
		cl.def("descriptorPolarImgDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0, float & a1) -> float { return o.descriptorPolarImgDistanceTo(a0, a1); }, "", pybind11::arg("oFeature"), pybind11::arg("minDistAngle"));
		cl.def("descriptorPolarImgDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, float &, bool) const) &mrpt::vision::CFeature::descriptorPolarImgDistanceTo, "Returns the minimum Euclidean Distance between \"this\" and the \"other\"\n polar image descriptor, for the best shift in orientation.\n \n\n The other feature to compare with.\n \n\n The placeholder for the angle at which the smallest\n distance is found.\n \n\n The distance for the best orientation (minimum distance).\n\nC++: mrpt::vision::CFeature::descriptorPolarImgDistanceTo(const class mrpt::vision::CFeature &, float &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("minDistAngle"), pybind11::arg("normalize_distances"));
		cl.def("descriptorLogPolarImgDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0, float & a1) -> float { return o.descriptorLogPolarImgDistanceTo(a0, a1); }, "", pybind11::arg("oFeature"), pybind11::arg("minDistAngle"));
		cl.def("descriptorLogPolarImgDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, float &, bool) const) &mrpt::vision::CFeature::descriptorLogPolarImgDistanceTo, "Returns the minimum Euclidean Distance between \"this\" and the \"other\"\n log-polar image descriptor, for the best shift in orientation.\n \n\n The other feature to compare with.\n \n\n The placeholder for the angle at which the smallest\n distance is found.\n \n\n The distance for the best orientation (minimum distance).\n\nC++: mrpt::vision::CFeature::descriptorLogPolarImgDistanceTo(const class mrpt::vision::CFeature &, float &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("minDistAngle"), pybind11::arg("normalize_distances"));
		cl.def("descriptorORBDistanceTo", (unsigned char (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &) const) &mrpt::vision::CFeature::descriptorORBDistanceTo, "Computes the Hamming distance \"this\" and the \"other\" descriptor ORB\n descriptor \n\nC++: mrpt::vision::CFeature::descriptorORBDistanceTo(const class mrpt::vision::CFeature &) const --> unsigned char", pybind11::arg("oFeature"));
		cl.def("descriptorBLDDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0) -> float { return o.descriptorBLDDistanceTo(a0); }, "", pybind11::arg("oFeature"));
		cl.def("descriptorBLDDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, bool) const) &mrpt::vision::CFeature::descriptorBLDDistanceTo, "Computes the Euclidean Distance between \"this\" and the \"other\"\n descriptors \n\nC++: mrpt::vision::CFeature::descriptorBLDDistanceTo(const class mrpt::vision::CFeature &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("normalize_distances"));
		cl.def("descriptorLATCHDistanceTo", [](mrpt::vision::CFeature const &o, const class mrpt::vision::CFeature & a0) -> float { return o.descriptorLATCHDistanceTo(a0); }, "", pybind11::arg("oFeature"));
		cl.def("descriptorLATCHDistanceTo", (float (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &, bool) const) &mrpt::vision::CFeature::descriptorLATCHDistanceTo, "Computes the Euclidean Distance between \"this\" and the \"other\"\n descriptors \n\nC++: mrpt::vision::CFeature::descriptorLATCHDistanceTo(const class mrpt::vision::CFeature &, bool) const --> float", pybind11::arg("oFeature"), pybind11::arg("normalize_distances"));
		cl.def("saveToTextFile", [](mrpt::vision::CFeature &o, const std::string & a0) -> void { return o.saveToTextFile(a0); }, "", pybind11::arg("filename"));
		cl.def("saveToTextFile", (void (mrpt::vision::CFeature::*)(const std::string &, bool)) &mrpt::vision::CFeature::saveToTextFile, "Save the feature to a text file in this format:\n    \"%% Dump of mrpt::vision::CFeatureList. Each line format is:\"\n    \"%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT\n[SIFT] HAS_SURF [SURF] HAS_MULTI [MULTI_i] HAS_ORB [ORB]\"\n    \"%% |---------------------- feature ------------------|\n|---------------------- descriptors ------------------------|\"\n    \"%% with:\"\n    \"%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4:\nSURF, 5: Beacon, 6: FAST, 7: ORB\"\n    \"%%  HAS_* : 1 if a descriptor of that type is associated to the\nfeature.\"\n    \"%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1\"\n    \"%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1\"\n    \"%%  MULTI : Present if HAS_MULTI=1: SCALE ORI N DESC_0 ... DESC_N-1\"\n	   \"%%  ORB   : Present if HAS_ORB=1: DESC_0 ... DESC_31\n    \"%%-----------------------------------------------------------------------------\");\n\nC++: mrpt::vision::CFeature::saveToTextFile(const std::string &, bool) --> void", pybind11::arg("filename"), pybind11::arg("APPEND"));
		cl.def("get_type", (enum mrpt::vision::TKeyPointMethod (mrpt::vision::CFeature::*)() const) &mrpt::vision::CFeature::get_type, "Get the type of the feature\n\nC++: mrpt::vision::CFeature::get_type() const --> enum mrpt::vision::TKeyPointMethod");
		cl.def("dumpToConsole", (void (mrpt::vision::CFeature::*)() const) &mrpt::vision::CFeature::dumpToConsole, "C++: mrpt::vision::CFeature::dumpToConsole() const --> void");
		cl.def("assign", (class mrpt::vision::CFeature & (mrpt::vision::CFeature::*)(const class mrpt::vision::CFeature &)) &mrpt::vision::CFeature::operator=, "C++: mrpt::vision::CFeature::operator=(const class mrpt::vision::CFeature &) --> class mrpt::vision::CFeature &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::vision::CFeature::TDescriptors file:mrpt/vision/CFeature.h line:106
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::vision::CFeature::TDescriptors, std::shared_ptr<mrpt::vision::CFeature::TDescriptors>> cl(enclosing_class, "TDescriptors", "All the possible descriptors this feature may have ");
			cl.def( pybind11::init( [](){ return new mrpt::vision::CFeature::TDescriptors(); } ) );
			cl.def( pybind11::init( [](mrpt::vision::CFeature::TDescriptors const &o){ return new mrpt::vision::CFeature::TDescriptors(o); } ) );
			cl.def_readwrite("SIFT", &mrpt::vision::CFeature::TDescriptors::SIFT);
			cl.def_readwrite("SURF", &mrpt::vision::CFeature::TDescriptors::SURF);
			cl.def_readwrite("SpinImg", &mrpt::vision::CFeature::TDescriptors::SpinImg);
			cl.def_readwrite("SpinImg_range_rows", &mrpt::vision::CFeature::TDescriptors::SpinImg_range_rows);
			cl.def_readwrite("PolarImg", &mrpt::vision::CFeature::TDescriptors::PolarImg);
			cl.def_readwrite("LogPolarImg", &mrpt::vision::CFeature::TDescriptors::LogPolarImg);
			cl.def_readwrite("polarImgsNoRotation", &mrpt::vision::CFeature::TDescriptors::polarImgsNoRotation);
			cl.def_readwrite("ORB", &mrpt::vision::CFeature::TDescriptors::ORB);
			cl.def_readwrite("BLD", &mrpt::vision::CFeature::TDescriptors::BLD);
			cl.def_readwrite("LATCH", &mrpt::vision::CFeature::TDescriptors::LATCH);
			cl.def("hasDescriptorSIFT", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorSIFT, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorSIFT() const --> bool");
			cl.def("hasDescriptorSURF", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorSURF, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorSURF() const --> bool");
			cl.def("hasDescriptorSpinImg", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorSpinImg, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorSpinImg() const --> bool");
			cl.def("hasDescriptorPolarImg", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorPolarImg, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorPolarImg() const --> bool");
			cl.def("hasDescriptorLogPolarImg", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorLogPolarImg, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorLogPolarImg() const --> bool");
			cl.def("hasDescriptorORB", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorORB, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorORB() const --> bool");
			cl.def("hasDescriptorBLD", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorBLD, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorBLD() const --> bool");
			cl.def("hasDescriptorLATCH", (bool (mrpt::vision::CFeature::TDescriptors::*)() const) &mrpt::vision::CFeature::TDescriptors::hasDescriptorLATCH, "C++: mrpt::vision::CFeature::TDescriptors::hasDescriptorLATCH() const --> bool");
			cl.def("assign", (struct mrpt::vision::CFeature::TDescriptors & (mrpt::vision::CFeature::TDescriptors::*)(const struct mrpt::vision::CFeature::TDescriptors &)) &mrpt::vision::CFeature::TDescriptors::operator=, "C++: mrpt::vision::CFeature::TDescriptors::operator=(const struct mrpt::vision::CFeature::TDescriptors &) --> struct mrpt::vision::CFeature::TDescriptors &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
