#include <deque>
#include <iterator>
#include <memory>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <variant>

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

// mrpt::maps::CSimpleMap file:mrpt/maps/CSimpleMap.h line:61
struct PyCallBack_mrpt_maps_CSimpleMap : public mrpt::maps::CSimpleMap {
	using mrpt::maps::CSimpleMap::CSimpleMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimpleMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSimpleMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimpleMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSimpleMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimpleMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSimpleMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimpleMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimpleMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimpleMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimpleMap::serializeFrom(a0, a1);
	}
};

void bind_mrpt_maps_CSimpleMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CSimpleMap file:mrpt/maps/CSimpleMap.h line:61
		pybind11::class_<mrpt::maps::CSimpleMap, std::shared_ptr<mrpt::maps::CSimpleMap>, PyCallBack_mrpt_maps_CSimpleMap, mrpt::serialization::CSerializable> cl(M("mrpt::maps"), "CSimpleMap", "A view-based map: a set of poses and what the robot saw from those poses.\n\n A simplemap comprises a sequence of tuples, each containing:\n - The **keyframe SE(3) pose** of the robot, including (optionally) its\n   uncertainty, as instances of mrpt::poses::CPose3DPDF\n - The **raw observations** from that keyframe, in a mrpt::obs::CSensoryFrame\n - Optionally, the **twist** (linear and angular velocity) of the robot in the\n   local frame of reference, at that moment. It can be used to undistort data\n   from a rotatory lidar, for example.\n\n To generate an actual metric map (occupancy grid, point cloud, octomap, etc.)\n from a \"simple map\", the user must instantiate the desired metric map\n class(es) and invoke its virtual method\n mrpt::maps::CMetricMap::loadFromSimpleMap().\n\n Users can also use the new top-level [library\n mp2p_icp_filters](https://github.com/MOLAorg/mp2p_icp/) and its CLI\n application\n [sm2mm](https://github.com/MOLAorg/mp2p_icp/tree/master/apps/sm2mm)\n (simple-map to metric-map)\n to generate metric maps including pre-processing of raw data in a flexible\n way.\n\n To programatically change an existing simplemap, use the non-const get()\n method and modify the returned reference.\n\n Copy constructor and copy operator makes shallow copies of all data.\n A makeDeepCopy() method is also provided which duplicates all internal data,\n if really needed.\n\n \n Objects of this class are serialized into GZ-compressed files with\n       the extension `.simplemap`.\n       See [Robotics file formats](robotics_file_formats.html).\n\n \n mrpt::obs::CSensoryFrame, mrpt::poses::CPose3DPDF,\n     mrpt::maps::CMetricMap, https://github.com/MOLAorg/mp2p_icp/\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CSimpleMap(); }, [](){ return new PyCallBack_mrpt_maps_CSimpleMap(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CSimpleMap const &o){ return new PyCallBack_mrpt_maps_CSimpleMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CSimpleMap const &o){ return new mrpt::maps::CSimpleMap(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CSimpleMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CSimpleMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CSimpleMap::*)() const) &mrpt::maps::CSimpleMap::GetRuntimeClass, "C++: mrpt::maps::CSimpleMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CSimpleMap::*)() const) &mrpt::maps::CSimpleMap::clone, "C++: mrpt::maps::CSimpleMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CSimpleMap::CreateObject, "C++: mrpt::maps::CSimpleMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("makeDeepCopy", (class mrpt::maps::CSimpleMap (mrpt::maps::CSimpleMap::*)()) &mrpt::maps::CSimpleMap::makeDeepCopy, "makes a deep copy of all data  \n\nC++: mrpt::maps::CSimpleMap::makeDeepCopy() --> class mrpt::maps::CSimpleMap");
		cl.def("saveToFile", (bool (mrpt::maps::CSimpleMap::*)(const std::string &) const) &mrpt::maps::CSimpleMap::saveToFile, "Save this object to a .simplemap binary file (compressed with gzip)\n See [Robotics file formats](robotics_file_formats.html).\n \n\n loadFromFile()\n \n\n false on any error. \n\nC++: mrpt::maps::CSimpleMap::saveToFile(const std::string &) const --> bool", pybind11::arg("filName"));
		cl.def("loadFromFile", (bool (mrpt::maps::CSimpleMap::*)(const std::string &)) &mrpt::maps::CSimpleMap::loadFromFile, "Load the contents of this object from a .simplemap binary file (possibly\n compressed with gzip)\n See [Robotics file formats](robotics_file_formats.html).\n \n\n saveToFile()\n \n\n false on any error. \n\nC++: mrpt::maps::CSimpleMap::loadFromFile(const std::string &) --> bool", pybind11::arg("filName"));
		cl.def("size", (size_t (mrpt::maps::CSimpleMap::*)() const) &mrpt::maps::CSimpleMap::size, "Returns the number of keyframes in the map \n\nC++: mrpt::maps::CSimpleMap::size() const --> size_t");
		cl.def("empty", (bool (mrpt::maps::CSimpleMap::*)() const) &mrpt::maps::CSimpleMap::empty, "Returns size()!=0 \n\nC++: mrpt::maps::CSimpleMap::empty() const --> bool");
		cl.def("get", (struct mrpt::maps::CSimpleMap::Keyframe & (mrpt::maps::CSimpleMap::*)(size_t)) &mrpt::maps::CSimpleMap::get, "non-const accessor, returning a reference suitable for modification\n\nC++: mrpt::maps::CSimpleMap::get(size_t) --> struct mrpt::maps::CSimpleMap::Keyframe &", pybind11::return_value_policy::automatic, pybind11::arg("index"));
		cl.def("remove", (void (mrpt::maps::CSimpleMap::*)(size_t)) &mrpt::maps::CSimpleMap::remove, "Deletes the 0-based index i'th keyframe.\n \n\n std::exception On index out of bounds.\n \n\n insert, get, set\n\nC++: mrpt::maps::CSimpleMap::remove(size_t) --> void", pybind11::arg("index"));
		cl.def("insert", (void (mrpt::maps::CSimpleMap::*)(const struct mrpt::maps::CSimpleMap::Keyframe &)) &mrpt::maps::CSimpleMap::insert, "Adds a new keyframe (SE(3) pose) to the view-based map.\n  Both shared pointers are copied (shallow object copies).\n\nC++: mrpt::maps::CSimpleMap::insert(const struct mrpt::maps::CSimpleMap::Keyframe &) --> void", pybind11::arg("kf"));
		cl.def("clear", (void (mrpt::maps::CSimpleMap::*)()) &mrpt::maps::CSimpleMap::clear, "Remove all stored keyframes.  \n remove \n\nC++: mrpt::maps::CSimpleMap::clear() --> void");
		cl.def("changeCoordinatesOrigin", (void (mrpt::maps::CSimpleMap::*)(const class mrpt::poses::CPose3D &)) &mrpt::maps::CSimpleMap::changeCoordinatesOrigin, "Change the coordinate origin of all stored poses, that is, translates\n and rotates the map such that the old SE(3) origin (identity\n transformation) becomes the new provided one.\n\nC++: mrpt::maps::CSimpleMap::changeCoordinatesOrigin(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newOrigin"));
		cl.def("assign", (class mrpt::maps::CSimpleMap & (mrpt::maps::CSimpleMap::*)(const class mrpt::maps::CSimpleMap &)) &mrpt::maps::CSimpleMap::operator=, "C++: mrpt::maps::CSimpleMap::operator=(const class mrpt::maps::CSimpleMap &) --> class mrpt::maps::CSimpleMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CSimpleMap::Keyframe file:mrpt/maps/CSimpleMap.h line:71
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CSimpleMap::Keyframe, std::shared_ptr<mrpt::maps::CSimpleMap::Keyframe>> cl(enclosing_class, "Keyframe", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CSimpleMap::Keyframe(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CSimpleMap::Keyframe const &o){ return new mrpt::maps::CSimpleMap::Keyframe(o); } ) );
			cl.def_readwrite("pose", &mrpt::maps::CSimpleMap::Keyframe::pose);
			cl.def_readwrite("sf", &mrpt::maps::CSimpleMap::Keyframe::sf);
			cl.def_readwrite("localTwist", &mrpt::maps::CSimpleMap::Keyframe::localTwist);
			cl.def("assign", (struct mrpt::maps::CSimpleMap::Keyframe & (mrpt::maps::CSimpleMap::Keyframe::*)(const struct mrpt::maps::CSimpleMap::Keyframe &)) &mrpt::maps::CSimpleMap::Keyframe::operator=, "C++: mrpt::maps::CSimpleMap::Keyframe::operator=(const struct mrpt::maps::CSimpleMap::Keyframe &) --> struct mrpt::maps::CSimpleMap::Keyframe &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
