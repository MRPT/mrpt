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
#include <mrpt/math/CPolygon.h>
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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
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

// mrpt::opengl::CPlanarLaserScan file:mrpt/opengl/CPlanarLaserScan.h line:55
struct PyCallBack_mrpt_opengl_CPlanarLaserScan : public mrpt::opengl::CPlanarLaserScan {
	using mrpt::opengl::CPlanarLaserScan::CPlanarLaserScan;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPlanarLaserScan::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPlanarLaserScan::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPlanarLaserScan::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::onUpdateBuffers_Triangles();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPlanarLaserScan::onUpdateBuffers_Points();
	}
	struct mrpt::math::TPoint3D_<float> getLocalRepresentativePoint() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "getLocalRepresentativePoint");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPoint3D_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPoint3D_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPoint3D_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPoint3D_<float>>(std::move(o));
		}
		return CPlanarLaserScan::getLocalRepresentativePoint();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CPlanarLaserScan::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "setColorA_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CRenderizable::setColorA_u8(a0);
	}
	class mrpt::opengl::CRenderizable & setColor_u8(const struct mrpt::img::TColor & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "setColor_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CRenderizable::setColor_u8(a0);
	}
	bool cullElegible() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "toYAMLMap");
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
	bool isCompositeObject() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "isCompositeObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRenderizable::isCompositeObject();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRenderizable::traceRay(a0, a1);
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPlanarLaserScan *>(this), "initializeTextures");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizable::initializeTextures();
	}
};

void bind_mrpt_opengl_CPlanarLaserScan(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CPlanarLaserScan file:mrpt/opengl/CPlanarLaserScan.h line:55
		pybind11::class_<mrpt::opengl::CPlanarLaserScan, std::shared_ptr<mrpt::opengl::CPlanarLaserScan>, PyCallBack_mrpt_opengl_CPlanarLaserScan, mrpt::opengl::CRenderizableShaderPoints, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CPlanarLaserScan", "This object renders a 2D laser scan by means of three elements: the points,\n the line along end-points and the 2D scanned surface.\n\n  By default, all those three elements are drawn, but you can individually\n switch them on/off with:\n    - CPlanarLaserScan::enablePoints()\n    - CPlanarLaserScan::enableLine()\n    - CPlanarLaserScan::enableSurface()\n\n  To change the final result, more methods allow further customization of the\n 3D object (color of each element, etc.).\n\n  The scan is passed or updated through CPlanarLaserScan::setScan()\n\n  \n  \n     mrpt::opengl::CPlanarLaserScan   \n\n\n preview_CPlanarLaserScan.png  \n  \n  \n\n  \n The laser points are projected at the sensor pose as given in the\n \"scan\" object, so this CPlanarLaserScan object should be placed at the exact\n pose of the robot coordinates origin.\n\n  \n mrpt::opengl::CPointCloud, opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CPlanarLaserScan(); }, [](){ return new PyCallBack_mrpt_opengl_CPlanarLaserScan(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CPlanarLaserScan const &o){ return new PyCallBack_mrpt_opengl_CPlanarLaserScan(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CPlanarLaserScan const &o){ return new mrpt::opengl::CPlanarLaserScan(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CPlanarLaserScan::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CPlanarLaserScan::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CPlanarLaserScan::*)() const) &mrpt::opengl::CPlanarLaserScan::GetRuntimeClass, "C++: mrpt::opengl::CPlanarLaserScan::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CPlanarLaserScan::*)() const) &mrpt::opengl::CPlanarLaserScan::clone, "C++: mrpt::opengl::CPlanarLaserScan::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CPlanarLaserScan::CreateObject, "C++: mrpt::opengl::CPlanarLaserScan::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CPlanarLaserScan::*)() const) &mrpt::opengl::CPlanarLaserScan::renderUpdateBuffers, "C++: mrpt::opengl::CPlanarLaserScan::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CPlanarLaserScan::*)()) &mrpt::opengl::CPlanarLaserScan::freeOpenGLResources, "C++: mrpt::opengl::CPlanarLaserScan::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CPlanarLaserScan::*)()) &mrpt::opengl::CPlanarLaserScan::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CPlanarLaserScan::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CPlanarLaserScan::*)()) &mrpt::opengl::CPlanarLaserScan::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CPlanarLaserScan::onUpdateBuffers_Triangles() --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CPlanarLaserScan::*)()) &mrpt::opengl::CPlanarLaserScan::onUpdateBuffers_Points, "C++: mrpt::opengl::CPlanarLaserScan::onUpdateBuffers_Points() --> void");
		cl.def("getLocalRepresentativePoint", (struct mrpt::math::TPoint3D_<float> (mrpt::opengl::CPlanarLaserScan::*)() const) &mrpt::opengl::CPlanarLaserScan::getLocalRepresentativePoint, "C++: mrpt::opengl::CPlanarLaserScan::getLocalRepresentativePoint() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("clear", (void (mrpt::opengl::CPlanarLaserScan::*)()) &mrpt::opengl::CPlanarLaserScan::clear, "Clear the scan \n\nC++: mrpt::opengl::CPlanarLaserScan::clear() --> void");
		cl.def("enablePoints", [](mrpt::opengl::CPlanarLaserScan &o) -> void { return o.enablePoints(); }, "");
		cl.def("enablePoints", (void (mrpt::opengl::CPlanarLaserScan::*)(bool)) &mrpt::opengl::CPlanarLaserScan::enablePoints, "Show or hides the scanned points \n sePointsWidth, setPointsColor\n\nC++: mrpt::opengl::CPlanarLaserScan::enablePoints(bool) --> void", pybind11::arg("enable"));
		cl.def("enableLine", [](mrpt::opengl::CPlanarLaserScan &o) -> void { return o.enableLine(); }, "");
		cl.def("enableLine", (void (mrpt::opengl::CPlanarLaserScan::*)(bool)) &mrpt::opengl::CPlanarLaserScan::enableLine, "Show or hides lines along all scanned points \n setLineWidth,\n setLineColor\n\nC++: mrpt::opengl::CPlanarLaserScan::enableLine(bool) --> void", pybind11::arg("enable"));
		cl.def("enableSurface", [](mrpt::opengl::CPlanarLaserScan &o) -> void { return o.enableSurface(); }, "");
		cl.def("enableSurface", (void (mrpt::opengl::CPlanarLaserScan::*)(bool)) &mrpt::opengl::CPlanarLaserScan::enableSurface, "Show or hides the scanned area as a 2D surface \n setSurfaceColor \n\nC++: mrpt::opengl::CPlanarLaserScan::enableSurface(bool) --> void", pybind11::arg("enable"));
		cl.def("setLineColor", [](mrpt::opengl::CPlanarLaserScan &o, float const & a0, float const & a1, float const & a2) -> void { return o.setLineColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setLineColor", (void (mrpt::opengl::CPlanarLaserScan::*)(float, float, float, float)) &mrpt::opengl::CPlanarLaserScan::setLineColor, "C++: mrpt::opengl::CPlanarLaserScan::setLineColor(float, float, float, float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("setPointsColor", [](mrpt::opengl::CPlanarLaserScan &o, float const & a0, float const & a1, float const & a2) -> void { return o.setPointsColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointsColor", (void (mrpt::opengl::CPlanarLaserScan::*)(float, float, float, float)) &mrpt::opengl::CPlanarLaserScan::setPointsColor, "C++: mrpt::opengl::CPlanarLaserScan::setPointsColor(float, float, float, float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("setSurfaceColor", [](mrpt::opengl::CPlanarLaserScan &o, float const & a0, float const & a1, float const & a2) -> void { return o.setSurfaceColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setSurfaceColor", (void (mrpt::opengl::CPlanarLaserScan::*)(float, float, float, float)) &mrpt::opengl::CPlanarLaserScan::setSurfaceColor, "C++: mrpt::opengl::CPlanarLaserScan::setSurfaceColor(float, float, float, float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("setScan", (void (mrpt::opengl::CPlanarLaserScan::*)(const class mrpt::obs::CObservation2DRangeScan &)) &mrpt::opengl::CPlanarLaserScan::setScan, "C++: mrpt::opengl::CPlanarLaserScan::setScan(const class mrpt::obs::CObservation2DRangeScan &) --> void", pybind11::arg("scan"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CPlanarLaserScan::*)() const) &mrpt::opengl::CPlanarLaserScan::internalBoundingBoxLocal, "C++: mrpt::opengl::CPlanarLaserScan::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CPlanarLaserScan & (mrpt::opengl::CPlanarLaserScan::*)(const class mrpt::opengl::CPlanarLaserScan &)) &mrpt::opengl::CPlanarLaserScan::operator=, "C++: mrpt::opengl::CPlanarLaserScan::operator=(const class mrpt::opengl::CPlanarLaserScan &) --> class mrpt::opengl::CPlanarLaserScan &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
