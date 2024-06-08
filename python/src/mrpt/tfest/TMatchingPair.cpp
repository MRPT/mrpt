#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
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

void bind_mrpt_tfest_TMatchingPair(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::tfest::TMatchingPairListTempl file:mrpt/tfest/TMatchingPair.h line:129
		pybind11::class_<mrpt::tfest::TMatchingPairListTempl<float>, std::shared_ptr<mrpt::tfest::TMatchingPairListTempl<float>>> cl(M("mrpt::tfest"), "TMatchingPairListTempl_float_t", "");
		cl.def( pybind11::init( [](mrpt::tfest::TMatchingPairListTempl<float> const &o){ return new mrpt::tfest::TMatchingPairListTempl<float>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::tfest::TMatchingPairListTempl<float>(); } ) );
		cl.def("indexOtherMapHasCorrespondence", (bool (mrpt::tfest::TMatchingPairListTempl<float>::*)(size_t) const) &mrpt::tfest::TMatchingPairListTempl<float>::indexOtherMapHasCorrespondence, "C++: mrpt::tfest::TMatchingPairListTempl<float>::indexOtherMapHasCorrespondence(size_t) const --> bool", pybind11::arg("idx"));
		cl.def("dumpToFile", (void (mrpt::tfest::TMatchingPairListTempl<float>::*)(const std::string &) const) &mrpt::tfest::TMatchingPairListTempl<float>::dumpToFile, "C++: mrpt::tfest::TMatchingPairListTempl<float>::dumpToFile(const std::string &) const --> void", pybind11::arg("fileName"));
		cl.def("saveAsMATLABScript", (void (mrpt::tfest::TMatchingPairListTempl<float>::*)(const std::string &) const) &mrpt::tfest::TMatchingPairListTempl<float>::saveAsMATLABScript, "C++: mrpt::tfest::TMatchingPairListTempl<float>::saveAsMATLABScript(const std::string &) const --> void", pybind11::arg("filName"));
		cl.def("overallSquareError", (float (mrpt::tfest::TMatchingPairListTempl<float>::*)(const class mrpt::poses::CPose2D &) const) &mrpt::tfest::TMatchingPairListTempl<float>::overallSquareError, "C++: mrpt::tfest::TMatchingPairListTempl<float>::overallSquareError(const class mrpt::poses::CPose2D &) const --> float", pybind11::arg("q"));
		cl.def("overallSquareError", (float (mrpt::tfest::TMatchingPairListTempl<float>::*)(const class mrpt::poses::CPose3D &) const) &mrpt::tfest::TMatchingPairListTempl<float>::overallSquareError, "C++: mrpt::tfest::TMatchingPairListTempl<float>::overallSquareError(const class mrpt::poses::CPose3D &) const --> float", pybind11::arg("q"));
		cl.def("filterUniqueRobustPairs", (void (mrpt::tfest::TMatchingPairListTempl<float>::*)(size_t, class mrpt::tfest::TMatchingPairListTempl<float> &) const) &mrpt::tfest::TMatchingPairListTempl<float>::filterUniqueRobustPairs, "C++: mrpt::tfest::TMatchingPairListTempl<float>::filterUniqueRobustPairs(size_t, class mrpt::tfest::TMatchingPairListTempl<float> &) const --> void", pybind11::arg("num_elements_this_map"), pybind11::arg("out_filtered_list"));
		cl.def("assign", (class mrpt::tfest::TMatchingPairListTempl<float> & (mrpt::tfest::TMatchingPairListTempl<float>::*)(const class mrpt::tfest::TMatchingPairListTempl<float> &)) &mrpt::tfest::TMatchingPairListTempl<float>::operator=, "C++: mrpt::tfest::TMatchingPairListTempl<float>::operator=(const class mrpt::tfest::TMatchingPairListTempl<float> &) --> class mrpt::tfest::TMatchingPairListTempl<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
