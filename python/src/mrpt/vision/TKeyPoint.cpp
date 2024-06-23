#include <iterator>
#include <memory>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/TKeyPoint.h>
#include <mrpt/vision/types.h>
#include <ostream>
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

	{ // mrpt::vision::CFeatureList file:mrpt/vision/CFeature.h line:268
		pybind11::class_<mrpt::vision::CFeatureList, std::shared_ptr<mrpt::vision::CFeatureList>> cl(M("mrpt::vision"), "CFeatureList", "A list of visual features, to be used as output by detectors, as\n input/output by trackers, etc.");
		cl.def( pybind11::init( [](){ return new mrpt::vision::CFeatureList(); } ) );
		cl.def( pybind11::init( [](mrpt::vision::CFeatureList const &o){ return new mrpt::vision::CFeatureList(o); } ) );
		cl.def("get_type", (enum mrpt::vision::TKeyPointMethod (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::get_type, "The type of the first feature in the list \n\nC++: mrpt::vision::CFeatureList::get_type() const --> enum mrpt::vision::TKeyPointMethod");
		cl.def("saveToTextFile", [](mrpt::vision::CFeatureList &o, const std::string & a0) -> void { return o.saveToTextFile(a0); }, "", pybind11::arg("fileName"));
		cl.def("saveToTextFile", (void (mrpt::vision::CFeatureList::*)(const std::string &, bool)) &mrpt::vision::CFeatureList::saveToTextFile, "Save feature list to a text file \n\nC++: mrpt::vision::CFeatureList::saveToTextFile(const std::string &, bool) --> void", pybind11::arg("fileName"), pybind11::arg("APPEND"));
		cl.def("loadFromTextFile", (void (mrpt::vision::CFeatureList::*)(const std::string &)) &mrpt::vision::CFeatureList::loadFromTextFile, "Save feature list to a text file \n\nC++: mrpt::vision::CFeatureList::loadFromTextFile(const std::string &) --> void", pybind11::arg("fileName"));
		cl.def("copyListFrom", (void (mrpt::vision::CFeatureList::*)(const class mrpt::vision::CFeatureList &)) &mrpt::vision::CFeatureList::copyListFrom, "Copies the content of another CFeatureList inside this one. The inner\n features are also copied. \n\nC++: mrpt::vision::CFeatureList::copyListFrom(const class mrpt::vision::CFeatureList &) --> void", pybind11::arg("otherList"));
		cl.def("getMaxID", (unsigned long (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::getMaxID, "Get the maximum ID into the list \n\nC++: mrpt::vision::CFeatureList::getMaxID() const --> unsigned long");
		cl.def("mark_kdtree_as_outdated", (void (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::mark_kdtree_as_outdated, "Call this when the list of features has been modified so the KD-tree is\n marked as outdated. \n\nC++: mrpt::vision::CFeatureList::mark_kdtree_as_outdated() const --> void");
		cl.def("empty", (bool (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::empty, "C++: mrpt::vision::CFeatureList::empty() const --> bool");
		cl.def("size", (size_t (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::size, "C++: mrpt::vision::CFeatureList::size() const --> size_t");
		cl.def("clear", (void (mrpt::vision::CFeatureList::*)()) &mrpt::vision::CFeatureList::clear, "C++: mrpt::vision::CFeatureList::clear() --> void");
		cl.def("resize", (void (mrpt::vision::CFeatureList::*)(size_t)) &mrpt::vision::CFeatureList::resize, "C++: mrpt::vision::CFeatureList::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("kdtree_get_point_count", (size_t (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::kdtree_get_point_count, "Must return the number of data points\n\nC++: mrpt::vision::CFeatureList::kdtree_get_point_count() const --> size_t");
		cl.def("kdtree_get_pt", (float (mrpt::vision::CFeatureList::*)(size_t, int) const) &mrpt::vision::CFeatureList::kdtree_get_pt, "Returns the dim'th component of the idx'th point in the class:\n\nC++: mrpt::vision::CFeatureList::kdtree_get_pt(size_t, int) const --> float", pybind11::arg("idx"), pybind11::arg("dim"));
		cl.def("kdtree_distance", (float (mrpt::vision::CFeatureList::*)(const float *, size_t, size_t) const) &mrpt::vision::CFeatureList::kdtree_distance, "Returns the distance between the vector \"p1[0:size-1]\" and the data\n point with index \"idx_p2\" stored in the class:\n\nC++: mrpt::vision::CFeatureList::kdtree_distance(const float *, size_t, size_t) const --> float", pybind11::arg("p1"), pybind11::arg("idx_p2"), pybind11::arg("size"));
		cl.def("getFeatureX", (float (mrpt::vision::CFeatureList::*)(size_t) const) &mrpt::vision::CFeatureList::getFeatureX, "@{ \n\nC++: mrpt::vision::CFeatureList::getFeatureX(size_t) const --> float", pybind11::arg("i"));
		cl.def("getFeatureY", (float (mrpt::vision::CFeatureList::*)(size_t) const) &mrpt::vision::CFeatureList::getFeatureY, "C++: mrpt::vision::CFeatureList::getFeatureY(size_t) const --> float", pybind11::arg("i"));
		cl.def("getFeatureID", (unsigned long (mrpt::vision::CFeatureList::*)(size_t) const) &mrpt::vision::CFeatureList::getFeatureID, "C++: mrpt::vision::CFeatureList::getFeatureID(size_t) const --> unsigned long", pybind11::arg("i"));
		cl.def("getFeatureResponse", (float (mrpt::vision::CFeatureList::*)(size_t) const) &mrpt::vision::CFeatureList::getFeatureResponse, "C++: mrpt::vision::CFeatureList::getFeatureResponse(size_t) const --> float", pybind11::arg("i"));
		cl.def("isPointFeature", (bool (mrpt::vision::CFeatureList::*)(size_t) const) &mrpt::vision::CFeatureList::isPointFeature, "C++: mrpt::vision::CFeatureList::isPointFeature(size_t) const --> bool", pybind11::arg("i"));
		cl.def("getScale", (float (mrpt::vision::CFeatureList::*)(size_t) const) &mrpt::vision::CFeatureList::getScale, "C++: mrpt::vision::CFeatureList::getScale(size_t) const --> float", pybind11::arg("i"));
		cl.def("getTrackStatus", (enum mrpt::vision::TFeatureTrackStatus (mrpt::vision::CFeatureList::*)(size_t)) &mrpt::vision::CFeatureList::getTrackStatus, "C++: mrpt::vision::CFeatureList::getTrackStatus(size_t) --> enum mrpt::vision::TFeatureTrackStatus", pybind11::arg("i"));
		cl.def("setFeatureX", (void (mrpt::vision::CFeatureList::*)(size_t, float)) &mrpt::vision::CFeatureList::setFeatureX, "C++: mrpt::vision::CFeatureList::setFeatureX(size_t, float) --> void", pybind11::arg("i"), pybind11::arg("x"));
		cl.def("setFeatureXf", (void (mrpt::vision::CFeatureList::*)(size_t, float)) &mrpt::vision::CFeatureList::setFeatureXf, "C++: mrpt::vision::CFeatureList::setFeatureXf(size_t, float) --> void", pybind11::arg("i"), pybind11::arg("x"));
		cl.def("setFeatureY", (void (mrpt::vision::CFeatureList::*)(size_t, float)) &mrpt::vision::CFeatureList::setFeatureY, "C++: mrpt::vision::CFeatureList::setFeatureY(size_t, float) --> void", pybind11::arg("i"), pybind11::arg("y"));
		cl.def("setFeatureYf", (void (mrpt::vision::CFeatureList::*)(size_t, float)) &mrpt::vision::CFeatureList::setFeatureYf, "C++: mrpt::vision::CFeatureList::setFeatureYf(size_t, float) --> void", pybind11::arg("i"), pybind11::arg("y"));
		cl.def("setFeatureID", (void (mrpt::vision::CFeatureList::*)(size_t, unsigned long)) &mrpt::vision::CFeatureList::setFeatureID, "C++: mrpt::vision::CFeatureList::setFeatureID(size_t, unsigned long) --> void", pybind11::arg("i"), pybind11::arg("id"));
		cl.def("setFeatureResponse", (void (mrpt::vision::CFeatureList::*)(size_t, float)) &mrpt::vision::CFeatureList::setFeatureResponse, "C++: mrpt::vision::CFeatureList::setFeatureResponse(size_t, float) --> void", pybind11::arg("i"), pybind11::arg("r"));
		cl.def("setScale", (void (mrpt::vision::CFeatureList::*)(size_t, uint8_t)) &mrpt::vision::CFeatureList::setScale, "C++: mrpt::vision::CFeatureList::setScale(size_t, uint8_t) --> void", pybind11::arg("i"), pybind11::arg("s"));
		cl.def("setTrackStatus", (void (mrpt::vision::CFeatureList::*)(size_t, enum mrpt::vision::TFeatureTrackStatus)) &mrpt::vision::CFeatureList::setTrackStatus, "C++: mrpt::vision::CFeatureList::setTrackStatus(size_t, enum mrpt::vision::TFeatureTrackStatus) --> void", pybind11::arg("i"), pybind11::arg("s"));
		cl.def("mark_as_outdated", (void (mrpt::vision::CFeatureList::*)() const) &mrpt::vision::CFeatureList::mark_as_outdated, "C++: mrpt::vision::CFeatureList::mark_as_outdated() const --> void");
		cl.def("assign", (class mrpt::vision::CFeatureList & (mrpt::vision::CFeatureList::*)(const class mrpt::vision::CFeatureList &)) &mrpt::vision::CFeatureList::operator=, "C++: mrpt::vision::CFeatureList::operator=(const class mrpt::vision::CFeatureList &) --> class mrpt::vision::CFeatureList &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::CMatchedFeatureList file:mrpt/vision/CFeature.h line:445
		pybind11::class_<mrpt::vision::CMatchedFeatureList, std::shared_ptr<mrpt::vision::CMatchedFeatureList>> cl(M("mrpt::vision"), "CMatchedFeatureList", "**************************************************\n      Class CMATCHEDFEATURELIST\n***************************************************\n\n A list of features");
		cl.def( pybind11::init( [](){ return new mrpt::vision::CMatchedFeatureList(); } ) );
		cl.def( pybind11::init( [](mrpt::vision::CMatchedFeatureList const &o){ return new mrpt::vision::CMatchedFeatureList(o); } ) );
		cl.def("get_type", (enum mrpt::vision::TKeyPointMethod (mrpt::vision::CMatchedFeatureList::*)() const) &mrpt::vision::CMatchedFeatureList::get_type, "The type of the first feature in the list \n\nC++: mrpt::vision::CMatchedFeatureList::get_type() const --> enum mrpt::vision::TKeyPointMethod");
		cl.def("saveToTextFile", (void (mrpt::vision::CMatchedFeatureList::*)(const std::string &)) &mrpt::vision::CMatchedFeatureList::saveToTextFile, "Save list of matched features to a text file \n\nC++: mrpt::vision::CMatchedFeatureList::saveToTextFile(const std::string &) --> void", pybind11::arg("fileName"));
		cl.def("getBothFeatureLists", (void (mrpt::vision::CMatchedFeatureList::*)(class mrpt::vision::CFeatureList &, class mrpt::vision::CFeatureList &)) &mrpt::vision::CMatchedFeatureList::getBothFeatureLists, "Returns the matching features as two separate CFeatureLists \n\nC++: mrpt::vision::CMatchedFeatureList::getBothFeatureLists(class mrpt::vision::CFeatureList &, class mrpt::vision::CFeatureList &) --> void", pybind11::arg("list1"), pybind11::arg("list2"));
		cl.def("getMaxID", (void (mrpt::vision::CMatchedFeatureList::*)(const enum mrpt::vision::TListIdx &, unsigned long &, unsigned long &)) &mrpt::vision::CMatchedFeatureList::getMaxID, "Returns the maximum ID of the features in the list. If the max ID has\n   been already set up, this method just returns it.\n    Otherwise, this method finds, stores and returns it.\n\nC++: mrpt::vision::CMatchedFeatureList::getMaxID(const enum mrpt::vision::TListIdx &, unsigned long &, unsigned long &) --> void", pybind11::arg("idx"), pybind11::arg("firstListID"), pybind11::arg("secondListID"));
		cl.def("updateMaxID", (void (mrpt::vision::CMatchedFeatureList::*)(const enum mrpt::vision::TListIdx &)) &mrpt::vision::CMatchedFeatureList::updateMaxID, "Updates the value of the maximum ID of the features in the matched list,\n i.e. it explicitly searches for the max ID and updates the member\n variables. \n\nC++: mrpt::vision::CMatchedFeatureList::updateMaxID(const enum mrpt::vision::TListIdx &) --> void", pybind11::arg("idx"));
		cl.def("setLeftMaxID", (void (mrpt::vision::CMatchedFeatureList::*)(const unsigned long &)) &mrpt::vision::CMatchedFeatureList::setLeftMaxID, "Explicitly set the max IDs values to certain values \n\nC++: mrpt::vision::CMatchedFeatureList::setLeftMaxID(const unsigned long &) --> void", pybind11::arg("leftID"));
		cl.def("setRightMaxID", (void (mrpt::vision::CMatchedFeatureList::*)(const unsigned long &)) &mrpt::vision::CMatchedFeatureList::setRightMaxID, "C++: mrpt::vision::CMatchedFeatureList::setRightMaxID(const unsigned long &) --> void", pybind11::arg("rightID"));
		cl.def("setMaxIDs", (void (mrpt::vision::CMatchedFeatureList::*)(const unsigned long &, const unsigned long &)) &mrpt::vision::CMatchedFeatureList::setMaxIDs, "C++: mrpt::vision::CMatchedFeatureList::setMaxIDs(const unsigned long &, const unsigned long &) --> void", pybind11::arg("leftID"), pybind11::arg("rightID"));
		cl.def("assign", (class mrpt::vision::CMatchedFeatureList & (mrpt::vision::CMatchedFeatureList::*)(const class mrpt::vision::CMatchedFeatureList &)) &mrpt::vision::CMatchedFeatureList::operator=, "C++: mrpt::vision::CMatchedFeatureList::operator=(const class mrpt::vision::CMatchedFeatureList &) --> class mrpt::vision::CMatchedFeatureList &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
