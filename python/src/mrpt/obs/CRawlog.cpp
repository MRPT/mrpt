#include <iterator>
#include <memory>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <tuple>
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

// mrpt::obs::CRawlog file:mrpt/obs/CRawlog.h line:62
struct PyCallBack_mrpt_obs_CRawlog : public mrpt::obs::CRawlog {
	using mrpt::obs::CRawlog::CRawlog;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CRawlog *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CRawlog::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CRawlog *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CRawlog::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CRawlog *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CRawlog::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CRawlog *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRawlog::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CRawlog *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRawlog::serializeFrom(a0, a1);
	}
};

void bind_mrpt_obs_CRawlog(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CRawlog file:mrpt/obs/CRawlog.h line:62
		pybind11::class_<mrpt::obs::CRawlog, std::shared_ptr<mrpt::obs::CRawlog>, PyCallBack_mrpt_obs_CRawlog, mrpt::serialization::CSerializable> cl(M("mrpt::obs"), "CRawlog", "The main class for loading and processing robotics datasets, or \"rawlogs\".\n\n Please, refer to the [rawlog format specification](rawlog_format.html).\n\n In short, this class stores a sequence of objects, in one of two possible\nformats:\n  - Format #1: A sequence of actions and observations. There is a sequence\nof objects, where each one can be of one type:\n    - An action:	Implemented as a CActionCollection object, the\nactuation\nof the robot (i.e. odometry increment).\n    - Observations: Implemented as a CSensoryFrame, refering to a set of\nrobot observations from the same pose.\n  - Format #2: A sequence of actions and observations. There is a sequence\nof objects, where each one can be of one type:\n\n See also [RawLogViewer](app_RawLogViewer.html) for a GUI application for\n quick inspection and analysis of rawlogs.\n\n There is a field for dataset plain-text comments (human-friendly description,\n blocks of parameters, etc.) accessible through CRawlog::getCommentText() and\n CRawlog::setCommentText().\n\n This container provides a STL container-like interface (see CRawlog::begin,\n CRawlog::iterator, ...).\n\n \n There is a static helper method CRawlog::detectImagesDirectory() to\n       identify the directory where external images are stored.\n\n \n CSensoryFrame,\n     [Dataset file format](robotics_file_formats.html#datasets).\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CRawlog(); }, [](){ return new PyCallBack_mrpt_obs_CRawlog(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CRawlog const &o){ return new PyCallBack_mrpt_obs_CRawlog(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CRawlog const &o){ return new mrpt::obs::CRawlog(o); } ) );

		pybind11::enum_<mrpt::obs::CRawlog::TEntryType>(cl, "TEntryType", pybind11::arithmetic(), "The type of each entry in a rawlog.\n \n\n CRawlog::getType")
			.value("etSensoryFrame", mrpt::obs::CRawlog::etSensoryFrame)
			.value("etActionCollection", mrpt::obs::CRawlog::etActionCollection)
			.value("etObservation", mrpt::obs::CRawlog::etObservation)
			.value("etOther", mrpt::obs::CRawlog::etOther)
			.export_values();

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CRawlog::GetRuntimeClassIdStatic, "C++: mrpt::obs::CRawlog::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CRawlog::*)() const) &mrpt::obs::CRawlog::GetRuntimeClass, "C++: mrpt::obs::CRawlog::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CRawlog::*)() const) &mrpt::obs::CRawlog::clone, "C++: mrpt::obs::CRawlog::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CRawlog::CreateObject, "C++: mrpt::obs::CRawlog::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getCommentText", (void (mrpt::obs::CRawlog::*)(std::string &) const) &mrpt::obs::CRawlog::getCommentText, "Returns the block of comment text for the rawlog \n\nC++: mrpt::obs::CRawlog::getCommentText(std::string &) const --> void", pybind11::arg("t"));
		cl.def("getCommentText", (std::string (mrpt::obs::CRawlog::*)() const) &mrpt::obs::CRawlog::getCommentText, "Returns the block of comment text for the rawlog \n\nC++: mrpt::obs::CRawlog::getCommentText() const --> std::string");
		cl.def("setCommentText", (void (mrpt::obs::CRawlog::*)(const std::string &)) &mrpt::obs::CRawlog::setCommentText, "Changes the block of comment text for the rawlog \n\nC++: mrpt::obs::CRawlog::setCommentText(const std::string &) --> void", pybind11::arg("t"));
		cl.def("getCommentTextAsConfigFile", (void (mrpt::obs::CRawlog::*)(class mrpt::config::CConfigFileMemory &) const) &mrpt::obs::CRawlog::getCommentTextAsConfigFile, "Saves the block of comment text for the rawlog into the passed config\n file object. \n\nC++: mrpt::obs::CRawlog::getCommentTextAsConfigFile(class mrpt::config::CConfigFileMemory &) const --> void", pybind11::arg("memCfg"));
		cl.def("clear", (void (mrpt::obs::CRawlog::*)()) &mrpt::obs::CRawlog::clear, "Clear the sequence of actions/observations. Smart pointers to objects\n previously in the rawlog will remain being valid. \n\nC++: mrpt::obs::CRawlog::clear() --> void");
		cl.def("empty", (bool (mrpt::obs::CRawlog::*)() const) &mrpt::obs::CRawlog::empty, "Returns true if the rawlog is empty \n\nC++: mrpt::obs::CRawlog::empty() const --> bool");
		cl.def("insert", (void (mrpt::obs::CRawlog::*)(class mrpt::obs::CAction &)) &mrpt::obs::CRawlog::insert, "Add an action to the sequence: a collection of just one element is\n created.\n   The object is duplicated, so the original one can be freed if desired.\n\nC++: mrpt::obs::CRawlog::insert(class mrpt::obs::CAction &) --> void", pybind11::arg("action"));
		cl.def("insert", (void (mrpt::obs::CRawlog::*)(class mrpt::obs::CActionCollection &)) &mrpt::obs::CRawlog::insert, "Add a set of actions to the sequence; the object is duplicated, so the\n original one can be freed if desired.\n \n\n insert, insert\n\nC++: mrpt::obs::CRawlog::insert(class mrpt::obs::CActionCollection &) --> void", pybind11::arg("action"));
		cl.def("insert", (void (mrpt::obs::CRawlog::*)(class mrpt::obs::CSensoryFrame &)) &mrpt::obs::CRawlog::insert, "Add a set of observations to the sequence; the object is duplicated, so\n the original one can be free if desired.\n\nC++: mrpt::obs::CRawlog::insert(class mrpt::obs::CSensoryFrame &) --> void", pybind11::arg("observations"));
		cl.def("insert", (void (mrpt::obs::CRawlog::*)(const class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::obs::CRawlog::insert, "Generic add for a smart pointer to a CSerializable object:\n\nC++: mrpt::obs::CRawlog::insert(const class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> void", pybind11::arg("obj"));
		cl.def("loadFromRawLogFile", [](mrpt::obs::CRawlog &o, const std::string & a0) -> bool { return o.loadFromRawLogFile(a0); }, "", pybind11::arg("fileName"));
		cl.def("loadFromRawLogFile", (bool (mrpt::obs::CRawlog::*)(const std::string &, bool)) &mrpt::obs::CRawlog::loadFromRawLogFile, "Load the contents from a file containing one of these possibilities:\n  - A \"CRawlog\" object.\n  - Directly the sequence of objects (pairs\n `CSensoryFrame`/`CActionCollection` or `CObservation*` objects). In this\n case the method stops reading on EOF of an unrecogniced class name.\n  - Only if `non_obs_objects_are_legal` is true, any `CSerializable`\n object is allowed in the log file. Otherwise, the read stops on classes\n different from the ones listed in the item above.\n \n\n It returns false upon error reading or accessing the file.\n\nC++: mrpt::obs::CRawlog::loadFromRawLogFile(const std::string &, bool) --> bool", pybind11::arg("fileName"), pybind11::arg("non_obs_objects_are_legal"));
		cl.def("saveToRawLogFile", (bool (mrpt::obs::CRawlog::*)(const std::string &) const) &mrpt::obs::CRawlog::saveToRawLogFile, "Saves the contents to a rawlog-file, compatible with RawlogViewer (As\n the sequence of internal objects).\n  The file is saved with gz-commpressed if MRPT has gz-streams.\n \n\n It returns false if any error is found while writing/creating\n the target file.\n\nC++: mrpt::obs::CRawlog::saveToRawLogFile(const std::string &) const --> bool", pybind11::arg("fileName"));
		cl.def("size", (size_t (mrpt::obs::CRawlog::*)() const) &mrpt::obs::CRawlog::size, "Returns the number of actions / observations object in the sequence. \n\nC++: mrpt::obs::CRawlog::size() const --> size_t");
		cl.def("getType", (enum mrpt::obs::CRawlog::TEntryType (mrpt::obs::CRawlog::*)(size_t) const) &mrpt::obs::CRawlog::getType, "Returns the type of a given element.\n \n\n isAction, isObservation\n\nC++: mrpt::obs::CRawlog::getType(size_t) const --> enum mrpt::obs::CRawlog::TEntryType", pybind11::arg("index"));
		cl.def("remove", (void (mrpt::obs::CRawlog::*)(size_t)) &mrpt::obs::CRawlog::remove, "Delete the action or observation stored in the given index.\n \n\n std::exception If index is out of bounds\n\nC++: mrpt::obs::CRawlog::remove(size_t) --> void", pybind11::arg("index"));
		cl.def("remove", (void (mrpt::obs::CRawlog::*)(size_t, size_t)) &mrpt::obs::CRawlog::remove, "Delete the elements stored in the given range of indices (including both\n the first and last one).\n \n\n std::exception If any index is out of bounds\n\nC++: mrpt::obs::CRawlog::remove(size_t, size_t) --> void", pybind11::arg("first_index"), pybind11::arg("last_index"));
		cl.def("getAsAction", (class std::shared_ptr<class mrpt::obs::CActionCollection> (mrpt::obs::CRawlog::*)(size_t) const) &mrpt::obs::CRawlog::getAsAction, "Returns the i'th element in the sequence, as being actions, where\n index=0 is the first object.\n  If it is not a CActionCollection, it throws an exception. Do neighter\n modify nor delete the returned pointer.\n \n\n size, isAction, getAsObservations, getAsObservation\n \n\n std::exception If index is out of bounds\n\nC++: mrpt::obs::CRawlog::getAsAction(size_t) const --> class std::shared_ptr<class mrpt::obs::CActionCollection>", pybind11::arg("index"));
		cl.def("getAsObservations", (class std::shared_ptr<class mrpt::obs::CSensoryFrame> (mrpt::obs::CRawlog::*)(size_t) const) &mrpt::obs::CRawlog::getAsObservations, "Returns the i'th element in the sequence, as being an action, where\n index=0 is the first object.\n  If it is not an CSensoryFrame, it throws an exception.\n \n\n size, isAction, getAsAction, getAsObservation\n \n\n std::exception If index is out of bounds\n\nC++: mrpt::obs::CRawlog::getAsObservations(size_t) const --> class std::shared_ptr<class mrpt::obs::CSensoryFrame>", pybind11::arg("index"));
		cl.def("getAsGeneric", (class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::obs::CRawlog::*)(size_t) const) &mrpt::obs::CRawlog::getAsGeneric, "Returns the i'th element in the sequence, being its class whatever.\n \n\n size, isAction, getAsAction, getAsObservations\n \n\n std::exception If index is out of bounds\n\nC++: mrpt::obs::CRawlog::getAsGeneric(size_t) const --> class std::shared_ptr<class mrpt::serialization::CSerializable>", pybind11::arg("index"));
		cl.def("getAsObservation", (class std::shared_ptr<class mrpt::obs::CObservation> (mrpt::obs::CRawlog::*)(size_t) const) &mrpt::obs::CRawlog::getAsObservation, "Returns the i'th element in the sequence, as being an observation, where\n index=0 is the first object.\n  If it is not an CObservation, it throws an exception. Do neighter\n modify nor delete the returned pointer.\n  This is the proper method to obtain the objects stored in a \"only\n observations\"-rawlog file (named \"format #2\" above.\n \n\n size, isAction, getAsAction\n \n\n std::exception If index is out of bounds\n\nC++: mrpt::obs::CRawlog::getAsObservation(size_t) const --> class std::shared_ptr<class mrpt::obs::CObservation>", pybind11::arg("index"));
		cl.def("swap", (void (mrpt::obs::CRawlog::*)(class mrpt::obs::CRawlog &)) &mrpt::obs::CRawlog::swap, "Efficiently swap the contents of two existing objects.\n\nC++: mrpt::obs::CRawlog::swap(class mrpt::obs::CRawlog &) --> void", pybind11::arg("obj"));
		cl.def_static("readActionObservationPair", (bool (*)(class mrpt::serialization::CArchive &, class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, unsigned long &)) &mrpt::obs::CRawlog::readActionObservationPair, "Reads a consecutive pair action / observation from the rawlog opened at\n some input stream.\n Previous contents of action and observations are discarded, and\n upon exit they contain smart pointers to the new objects read from the\n rawlog file.\n The input/output variable \"rawlogEntry\" is just a counter of the last\n rawlog entry read, for logging or monitoring purposes.\n \n\n false if there was some error, true otherwise.\n \n\n getActionObservationPair, getActionObservationPairOrObservation\n\nC++: mrpt::obs::CRawlog::readActionObservationPair(class mrpt::serialization::CArchive &, class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, unsigned long &) --> bool", pybind11::arg("inStream"), pybind11::arg("action"), pybind11::arg("observations"), pybind11::arg("rawlogEntry"));
		cl.def_static("getActionObservationPairOrObservation", (bool (*)(class mrpt::serialization::CArchive &, class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, class std::shared_ptr<class mrpt::obs::CObservation> &, unsigned long &)) &mrpt::obs::CRawlog::getActionObservationPairOrObservation, "Reads a consecutive pair action/sensory_frame OR an observation,\n depending of the rawlog format, from the rawlog opened at some input\n stream.\n Previous contents of action and observations are discarded, and\n upon return they contain smart pointers to the new objects read from\n the rawlog file.\n\n Depending on the rawlog file format, at return either:\n  - action/observations contain objects, or\n  - observation contains an object.\n\n The input/output variable \"rawlogEntry\" is just a counter of the last\n rawlog entry read, for logging or monitoring purposes.\n \n\n false if there was some error, true otherwise.\n \n\n getActionObservationPair\n\nC++: mrpt::obs::CRawlog::getActionObservationPairOrObservation(class mrpt::serialization::CArchive &, class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, class std::shared_ptr<class mrpt::obs::CObservation> &, unsigned long &) --> bool", pybind11::arg("inStream"), pybind11::arg("action"), pybind11::arg("observations"), pybind11::arg("observation"), pybind11::arg("rawlogEntry"));
		cl.def_static("ReadFromArchive", (class std::tuple<bool, unsigned long, class std::shared_ptr<class mrpt::obs::CActionCollection>, class std::shared_ptr<class mrpt::obs::CSensoryFrame>, class std::shared_ptr<class mrpt::obs::CObservation> > (*)(class mrpt::serialization::CArchive &, const unsigned long)) &mrpt::obs::CRawlog::ReadFromArchive, "Alternative to getActionObservationPairOrObservation() returning the\n tuple [readOk, rawlogEntryIndex, action,sf, obs], with either (action,sf)\n or (obs) as empty smart pointers depending on the rawlog file format.\n readOk is false on EOF or any other error.\n\nC++: mrpt::obs::CRawlog::ReadFromArchive(class mrpt::serialization::CArchive &, const unsigned long) --> class std::tuple<bool, unsigned long, class std::shared_ptr<class mrpt::obs::CActionCollection>, class std::shared_ptr<class mrpt::obs::CSensoryFrame>, class std::shared_ptr<class mrpt::obs::CObservation> >", pybind11::arg("inStream"), pybind11::arg("rawlogEntryIndex"));
		cl.def("getActionObservationPair", (bool (mrpt::obs::CRawlog::*)(class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, unsigned long &) const) &mrpt::obs::CRawlog::getActionObservationPair, "Gets the next consecutive pair action / observation from the rawlog\n loaded into this object.\n Previous contents of action and observations are discarded, and\n upon return they contain smart pointers to the next objects read from\n the rawlog dataset.\n The input/output variable \"rawlogEntry\" is just a counter of the last\n rawlog entry read, for logging or monitoring purposes.\n \n\n false if there was some error, true otherwise.\n \n\n readActionObservationPair\n\nC++: mrpt::obs::CRawlog::getActionObservationPair(class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, unsigned long &) const --> bool", pybind11::arg("action"), pybind11::arg("observations"), pybind11::arg("rawlogEntry"));
		cl.def_static("detectImagesDirectory", (std::string (*)(const std::string &)) &mrpt::obs::CRawlog::detectImagesDirectory, "Tries to auto-detect the external-images directory of the given rawlog\nfile.\n  This searches for the existence of the directories:\n		- \"<rawlog_file_path>/<rawlog_filename>_Images\"\n		- \"<rawlog_file_path>/<rawlog_filename>_images\"\n		- \"<rawlog_file_path>/<rawlog_filename>_IMAGES\"\n		- \"<rawlog_file_path>/Images\"  (This one is returned if none of the\nchoices actually exists).\n\n  The results from this function should be written into\nmrpt::img::CImage::getImagesPathBase() to enable automatic\n  loading of extenrnally-stored images in rawlogs.\n\nC++: mrpt::obs::CRawlog::detectImagesDirectory(const std::string &) --> std::string", pybind11::arg("rawlogFilename"));
		cl.def("assign", (class mrpt::obs::CRawlog & (mrpt::obs::CRawlog::*)(const class mrpt::obs::CRawlog &)) &mrpt::obs::CRawlog::operator=, "C++: mrpt::obs::CRawlog::operator=(const class mrpt::obs::CRawlog &) --> class mrpt::obs::CRawlog &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CRawlog::iterator file:mrpt/obs/CRawlog.h line:232
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CRawlog::iterator, std::shared_ptr<mrpt::obs::CRawlog::iterator>> cl(enclosing_class, "iterator", "A normal iterator, plus the extra method \"getType\" to determine the\n type of each entry in the sequence. ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CRawlog::iterator(); } ) );
			cl.def("dereference", (class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::obs::CRawlog::iterator::*)()) &mrpt::obs::CRawlog::iterator::operator*, "C++: mrpt::obs::CRawlog::iterator::operator*() --> class std::shared_ptr<class mrpt::serialization::CSerializable>");
			cl.def("getType", (enum mrpt::obs::CRawlog::TEntryType (mrpt::obs::CRawlog::iterator::*)() const) &mrpt::obs::CRawlog::iterator::getType, "C++: mrpt::obs::CRawlog::iterator::getType() const --> enum mrpt::obs::CRawlog::TEntryType");
		}

		{ // mrpt::obs::CRawlog::const_iterator file:mrpt/obs/CRawlog.h line:288
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CRawlog::const_iterator, std::shared_ptr<mrpt::obs::CRawlog::const_iterator>> cl(enclosing_class, "const_iterator", "A normal iterator, plus the extra method \"getType\" to determine the type\n of each entry in the sequence. ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CRawlog::const_iterator(); } ) );
			cl.def("dereference", (const class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::obs::CRawlog::const_iterator::*)() const) &mrpt::obs::CRawlog::const_iterator::operator*, "C++: mrpt::obs::CRawlog::const_iterator::operator*() const --> const class std::shared_ptr<class mrpt::serialization::CSerializable>");
			cl.def("getType", (enum mrpt::obs::CRawlog::TEntryType (mrpt::obs::CRawlog::const_iterator::*)() const) &mrpt::obs::CRawlog::const_iterator::getType, "C++: mrpt::obs::CRawlog::const_iterator::getType() const --> enum mrpt::obs::CRawlog::TEntryType");
		}

	}
}
