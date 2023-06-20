#include <any>
#include <functional>
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
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/Visualizable.h>
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
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
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

// mrpt::opengl::Visualizable file:mrpt/opengl/Visualizable.h line:21
struct PyCallBack_mrpt_opengl_Visualizable : public mrpt::opengl::Visualizable {
	using mrpt::opengl::Visualizable::Visualizable;

	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Visualizable *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"Visualizable::getVisualizationInto\"");
	}
};

void bind_mrpt_opengl_Visualizable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::Visualizable file:mrpt/opengl/Visualizable.h line:21
		pybind11::class_<mrpt::opengl::Visualizable, std::shared_ptr<mrpt::opengl::Visualizable>, PyCallBack_mrpt_opengl_Visualizable> cl(M("mrpt::opengl"), "Visualizable", "Interface for classes visualizable as an mrpt::opengl::CSetOfObjects.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_opengl_Visualizable(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_opengl_Visualizable const &>());
		cl.def("getVisualizationInto", (void (mrpt::opengl::Visualizable::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::opengl::Visualizable::getVisualizationInto, "Inserts 3D primitives representing this object into the provided\n container.\n Note that the former contents of `o` are not cleared.\n\n \n getVisualization()\n\nC++: mrpt::opengl::Visualizable::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("o"));
		cl.def("getVisualization", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (mrpt::opengl::Visualizable::*)() const) &mrpt::opengl::Visualizable::getVisualization, "Creates 3D primitives representing this objects.\n This is equivalent to getVisualizationInto() but creating, and returning\n by value, a new rpt::opengl::CSetOfObjects::Ptr shared pointer.\n \n\n getVisualizationInto()\n\nC++: mrpt::opengl::Visualizable::getVisualization() const --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");
		cl.def("assign", (class mrpt::opengl::Visualizable & (mrpt::opengl::Visualizable::*)(const class mrpt::opengl::Visualizable &)) &mrpt::opengl::Visualizable::operator=, "C++: mrpt::opengl::Visualizable::operator=(const class mrpt::opengl::Visualizable &) --> class mrpt::opengl::Visualizable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
