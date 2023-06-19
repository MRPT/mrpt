#include <iterator>
#include <memory>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Viewport.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
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

void bind_mrpt_core_safe_pointers(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::safe_ptr_basic file:mrpt/core/safe_pointers.h line:23
		pybind11::class_<mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>, std::shared_ptr<mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>>> cl(M("mrpt"), "safe_ptr_basic_mrpt_rtti_TRuntimeClassId_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>(); } ) );
		cl.def( pybind11::init( [](mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId> const &o){ return new mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>(o); } ) );
		cl.def( pybind11::init<const struct mrpt::rtti::TRuntimeClassId *>(), pybind11::arg("p") );

		cl.def("assign", (struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> & (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(struct mrpt::rtti::TRuntimeClassId *)) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=(struct mrpt::rtti::TRuntimeClassId *) --> struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("assign", (struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> & (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &)) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) --> struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const --> bool", pybind11::arg("o"));
		cl.def("get", (struct mrpt::rtti::TRuntimeClassId *& (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)()) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::get, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::get() --> struct mrpt::rtti::TRuntimeClassId *&", pybind11::return_value_policy::automatic);
		cl.def("arrow", (struct mrpt::rtti::TRuntimeClassId *& (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)()) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator->, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator->() --> struct mrpt::rtti::TRuntimeClassId *&", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::safe_ptr_basic file:mrpt/core/safe_pointers.h line:23
		pybind11::class_<mrpt::safe_ptr_basic<mrpt::opengl::Scene>, std::shared_ptr<mrpt::safe_ptr_basic<mrpt::opengl::Scene>>> cl(M("mrpt"), "safe_ptr_basic_mrpt_opengl_Scene_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::safe_ptr_basic<mrpt::opengl::Scene>(); } ) );
		cl.def( pybind11::init( [](mrpt::safe_ptr_basic<mrpt::opengl::Scene> const &o){ return new mrpt::safe_ptr_basic<mrpt::opengl::Scene>(o); } ) );
		cl.def( pybind11::init<const class mrpt::opengl::Scene *>(), pybind11::arg("p") );

		cl.def("assign", (struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> & (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(class mrpt::opengl::Scene *)) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=(class mrpt::opengl::Scene *) --> struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("assign", (struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> & (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &)) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) --> struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const class mrpt::opengl::Scene *) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==(const class mrpt::opengl::Scene *) const --> bool", pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const class mrpt::opengl::Scene *) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=(const class mrpt::opengl::Scene *) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const --> bool", pybind11::arg("o"));
		cl.def("get", (class mrpt::opengl::Scene *& (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)()) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::get, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::get() --> class mrpt::opengl::Scene *&", pybind11::return_value_policy::automatic);
		cl.def("arrow", (class mrpt::opengl::Scene *& (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)()) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator->, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator->() --> class mrpt::opengl::Scene *&", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::safe_ptr file:mrpt/core/safe_pointers.h line:71
		pybind11::class_<mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>, std::shared_ptr<mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>>, mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>> cl(M("mrpt"), "safe_ptr_mrpt_rtti_TRuntimeClassId_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>(); } ) );
		cl.def( pybind11::init( [](mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId> const &o){ return new mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>(o); } ) );
		cl.def( pybind11::init<const struct mrpt::rtti::TRuntimeClassId *>(), pybind11::arg("p") );

		cl.def("assign", (struct mrpt::safe_ptr<struct mrpt::rtti::TRuntimeClassId> & (mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr<struct mrpt::rtti::TRuntimeClassId> &)) &mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator=, "C++: mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator=(const struct mrpt::safe_ptr<struct mrpt::rtti::TRuntimeClassId> &) --> struct mrpt::safe_ptr<struct mrpt::rtti::TRuntimeClassId> &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (struct mrpt::safe_ptr<struct mrpt::rtti::TRuntimeClassId> & (mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::*)(struct mrpt::rtti::TRuntimeClassId *)) &mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator=, "C++: mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator=(struct mrpt::rtti::TRuntimeClassId *) --> struct mrpt::safe_ptr<struct mrpt::rtti::TRuntimeClassId> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("dereference", (struct mrpt::rtti::TRuntimeClassId & (mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::*)()) &mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator*, "C++: mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator*() --> struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("__getitem__", (struct mrpt::rtti::TRuntimeClassId & (mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::*)(size_t)) &mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator[], "C++: mrpt::safe_ptr<mrpt::rtti::TRuntimeClassId>::operator[](size_t) --> struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("assign", (struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> & (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(struct mrpt::rtti::TRuntimeClassId *)) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=(struct mrpt::rtti::TRuntimeClassId *) --> struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("assign", (struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> & (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &)) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator=(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) --> struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator==(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator!=(const struct mrpt::safe_ptr_basic<struct mrpt::rtti::TRuntimeClassId> &) const --> bool", pybind11::arg("o"));
		cl.def("get", (struct mrpt::rtti::TRuntimeClassId *& (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)()) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::get, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::get() --> struct mrpt::rtti::TRuntimeClassId *&", pybind11::return_value_policy::automatic);
		cl.def("arrow", (struct mrpt::rtti::TRuntimeClassId *& (mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::*)()) &mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator->, "C++: mrpt::safe_ptr_basic<mrpt::rtti::TRuntimeClassId>::operator->() --> struct mrpt::rtti::TRuntimeClassId *&", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::safe_ptr file:mrpt/core/safe_pointers.h line:71
		pybind11::class_<mrpt::safe_ptr<mrpt::opengl::Scene>, std::shared_ptr<mrpt::safe_ptr<mrpt::opengl::Scene>>, mrpt::safe_ptr_basic<mrpt::opengl::Scene>> cl(M("mrpt"), "safe_ptr_mrpt_opengl_Scene_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::safe_ptr<mrpt::opengl::Scene>(); } ) );
		cl.def( pybind11::init( [](mrpt::safe_ptr<mrpt::opengl::Scene> const &o){ return new mrpt::safe_ptr<mrpt::opengl::Scene>(o); } ) );
		cl.def( pybind11::init<const class mrpt::opengl::Scene *>(), pybind11::arg("p") );

		cl.def("assign", (struct mrpt::safe_ptr<class mrpt::opengl::Scene> & (mrpt::safe_ptr<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr<class mrpt::opengl::Scene> &)) &mrpt::safe_ptr<mrpt::opengl::Scene>::operator=, "C++: mrpt::safe_ptr<mrpt::opengl::Scene>::operator=(const struct mrpt::safe_ptr<class mrpt::opengl::Scene> &) --> struct mrpt::safe_ptr<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (struct mrpt::safe_ptr<class mrpt::opengl::Scene> & (mrpt::safe_ptr<mrpt::opengl::Scene>::*)(class mrpt::opengl::Scene *)) &mrpt::safe_ptr<mrpt::opengl::Scene>::operator=, "C++: mrpt::safe_ptr<mrpt::opengl::Scene>::operator=(class mrpt::opengl::Scene *) --> struct mrpt::safe_ptr<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("dereference", (class mrpt::opengl::Scene & (mrpt::safe_ptr<mrpt::opengl::Scene>::*)()) &mrpt::safe_ptr<mrpt::opengl::Scene>::operator*, "C++: mrpt::safe_ptr<mrpt::opengl::Scene>::operator*() --> class mrpt::opengl::Scene &", pybind11::return_value_policy::automatic);
		cl.def("__getitem__", (class mrpt::opengl::Scene & (mrpt::safe_ptr<mrpt::opengl::Scene>::*)(size_t)) &mrpt::safe_ptr<mrpt::opengl::Scene>::operator[], "C++: mrpt::safe_ptr<mrpt::opengl::Scene>::operator[](size_t) --> class mrpt::opengl::Scene &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("assign", (struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> & (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(class mrpt::opengl::Scene *)) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=(class mrpt::opengl::Scene *) --> struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("assign", (struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> & (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &)) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator=(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) --> struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const class mrpt::opengl::Scene *) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==(const class mrpt::opengl::Scene *) const --> bool", pybind11::arg("o"));
		cl.def("__eq__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator==(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const class mrpt::opengl::Scene *) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=(const class mrpt::opengl::Scene *) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator!=(const struct mrpt::safe_ptr_basic<class mrpt::opengl::Scene> &) const --> bool", pybind11::arg("o"));
		cl.def("get", (class mrpt::opengl::Scene *& (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)()) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::get, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::get() --> class mrpt::opengl::Scene *&", pybind11::return_value_policy::automatic);
		cl.def("arrow", (class mrpt::opengl::Scene *& (mrpt::safe_ptr_basic<mrpt::opengl::Scene>::*)()) &mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator->, "C++: mrpt::safe_ptr_basic<mrpt::opengl::Scene>::operator->() --> class mrpt::opengl::Scene *&", pybind11::return_value_policy::automatic);
	}
}
