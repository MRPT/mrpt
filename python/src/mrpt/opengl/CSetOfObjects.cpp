#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
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

// mrpt::opengl::CSetOfObjects file:mrpt/opengl/CSetOfObjects.h line:26
struct PyCallBack_mrpt_opengl_CSetOfObjects : public mrpt::opengl::CSetOfObjects {
	using mrpt::opengl::CSetOfObjects::CSetOfObjects;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSetOfObjects::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSetOfObjects::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSetOfObjects::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::renderUpdateBuffers();
	}
	bool isCompositeObject() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "isCompositeObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSetOfObjects::isCompositeObject();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::freeOpenGLResources();
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "initializeTextures");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::initializeTextures();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSetOfObjects::traceRay(a0, a1);
	}
	class mrpt::opengl::CRenderizable & setColor_u8(const struct mrpt::img::TColor & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "setColor_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CSetOfObjects::setColor_u8(a0);
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "setColorA_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CSetOfObjects::setColorA_u8(a0);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CSetOfObjects::internalBoundingBoxLocal();
	}
	bool cullElegible() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "cullElegible");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRenderizable::cullElegible();
	}
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizable::toYAMLMap(a0);
	}
	struct mrpt::math::TPoint3D_<float> getLocalRepresentativePoint() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "getLocalRepresentativePoint");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPoint3D_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPoint3D_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPoint3D_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPoint3D_<float>>(std::move(o));
		}
		return CRenderizable::getLocalRepresentativePoint();
	}
};

void bind_mrpt_opengl_CSetOfObjects(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CSetOfObjects file:mrpt/opengl/CSetOfObjects.h line:26
		pybind11::class_<mrpt::opengl::CSetOfObjects, std::shared_ptr<mrpt::opengl::CSetOfObjects>, PyCallBack_mrpt_opengl_CSetOfObjects, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CSetOfObjects", "A set of object, which are referenced to the coordinates framework\n established in this object.\n It can be established a hierarchy of \"CSetOfObjects\", where the coordinates\n framework of each one will be referenced to the parent's one.\n The list of child objects is accessed directly as in the class Scene\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSetOfObjects(); }, [](){ return new PyCallBack_mrpt_opengl_CSetOfObjects(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSetOfObjects const &o){ return new PyCallBack_mrpt_opengl_CSetOfObjects(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSetOfObjects const &o){ return new mrpt::opengl::CSetOfObjects(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::CSetOfObjects::Create, "C++: mrpt::opengl::CSetOfObjects::Create() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSetOfObjects::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSetOfObjects::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::GetRuntimeClass, "C++: mrpt::opengl::CSetOfObjects::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::clone, "C++: mrpt::opengl::CSetOfObjects::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSetOfObjects::CreateObject, "C++: mrpt::opengl::CSetOfObjects::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("insert", (void (mrpt::opengl::CSetOfObjects::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &)) &mrpt::opengl::CSetOfObjects::insert, "Insert a new object to the list.\n\nC++: mrpt::opengl::CSetOfObjects::insert(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) --> void", pybind11::arg("newObject"));
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::renderUpdateBuffers, "C++: mrpt::opengl::CSetOfObjects::renderUpdateBuffers() const --> void");
		cl.def("isCompositeObject", (bool (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::isCompositeObject, "C++: mrpt::opengl::CSetOfObjects::isCompositeObject() const --> bool");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::freeOpenGLResources, "C++: mrpt::opengl::CSetOfObjects::freeOpenGLResources() --> void");
		cl.def("initializeTextures", (void (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::initializeTextures, "C++: mrpt::opengl::CSetOfObjects::initializeTextures() const --> void");
		cl.def("clear", (void (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::clear, "Clear the list of objects in the scene, deleting objects' memory.\n\nC++: mrpt::opengl::CSetOfObjects::clear() --> void");
		cl.def("size", (size_t (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::size, "Returns number of objects.  \n\nC++: mrpt::opengl::CSetOfObjects::size() --> size_t");
		cl.def("empty", (bool (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::empty, "Returns true if there are no objects.  \n\nC++: mrpt::opengl::CSetOfObjects::empty() const --> bool");
		cl.def("getByName", (class std::shared_ptr<class mrpt::opengl::CRenderizable> (mrpt::opengl::CSetOfObjects::*)(const std::string &)) &mrpt::opengl::CSetOfObjects::getByName, "Returns the first object with a given name, or a nullptr pointer if not\n found.\n\nC++: mrpt::opengl::CSetOfObjects::getByName(const std::string &) --> class std::shared_ptr<class mrpt::opengl::CRenderizable>", pybind11::arg("str"));
		cl.def("removeObject", (void (mrpt::opengl::CSetOfObjects::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &)) &mrpt::opengl::CSetOfObjects::removeObject, "Removes the given object from the scene (it also deletes the object to\n free its memory).\n\nC++: mrpt::opengl::CSetOfObjects::removeObject(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) --> void", pybind11::arg("obj"));
		cl.def("dumpListOfObjects", (void (mrpt::opengl::CSetOfObjects::*)(class std::vector<std::string > &) const) &mrpt::opengl::CSetOfObjects::dumpListOfObjects, "Retrieves a list of all objects in text form\n \n\n Prefer asYAML() (since MRPT 2.1.3) \n\nC++: mrpt::opengl::CSetOfObjects::dumpListOfObjects(class std::vector<std::string > &) const --> void", pybind11::arg("lst"));
		cl.def("asYAML", (class mrpt::containers::yaml (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::asYAML, "Prints all objects in human-readable YAML form.\n \n\n (New in MRPT 2.1.3) \n\nC++: mrpt::opengl::CSetOfObjects::asYAML() const --> class mrpt::containers::yaml");
		cl.def("traceRay", (bool (mrpt::opengl::CSetOfObjects::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CSetOfObjects::traceRay, "C++: mrpt::opengl::CSetOfObjects::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("setColor_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CSetOfObjects::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::CSetOfObjects::setColor_u8, "C++: mrpt::opengl::CSetOfObjects::setColor_u8(const struct mrpt::img::TColor &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("c"));
		cl.def("setColorA_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CSetOfObjects::*)(const unsigned char)) &mrpt::opengl::CSetOfObjects::setColorA_u8, "C++: mrpt::opengl::CSetOfObjects::setColorA_u8(const unsigned char) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("a"));
		cl.def("contains", (bool (mrpt::opengl::CSetOfObjects::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) const) &mrpt::opengl::CSetOfObjects::contains, "C++: mrpt::opengl::CSetOfObjects::contains(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) const --> bool", pybind11::arg("obj"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::internalBoundingBoxLocal, "C++: mrpt::opengl::CSetOfObjects::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPosePDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPosePDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPosePDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPointPDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPointPDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPointPDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPose3DPDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPose3DPDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPose3DQuatPDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPose3DQuatPDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPose3DQuatPDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def("assign", (class mrpt::opengl::CSetOfObjects & (mrpt::opengl::CSetOfObjects::*)(const class mrpt::opengl::CSetOfObjects &)) &mrpt::opengl::CSetOfObjects::operator=, "C++: mrpt::opengl::CSetOfObjects::operator=(const class mrpt::opengl::CSetOfObjects &) --> class mrpt::opengl::CSetOfObjects &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
