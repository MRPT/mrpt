#include <iterator>
#include <memory>
#include <mrpt/containers/yaml.h>
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
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

// mrpt::gui::CGlCanvasBaseHeadless file:mrpt/gui/CGlCanvasBase.h line:250
struct PyCallBack_mrpt_gui_CGlCanvasBaseHeadless : public mrpt::gui::CGlCanvasBaseHeadless {
	using mrpt::gui::CGlCanvasBaseHeadless::CGlCanvasBaseHeadless;

	void swapBuffers() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "swapBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBaseHeadless::swapBuffers();
	}
	void preRender() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "preRender");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBaseHeadless::preRender();
	}
	void postRender() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "postRender");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBaseHeadless::postRender();
	}
	void renderError(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "renderError");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBaseHeadless::renderError(a0);
	}
	void setCameraParams(const struct mrpt::gui::CGlCanvasBase::CamaraParams & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setCameraParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraParams(a0);
	}
	void setCameraPointing(float a0, float a1, float a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setCameraPointing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraPointing(a0, a1, a2);
	}
	void setZoomDistance(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setZoomDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setZoomDistance(a0);
	}
	void setAzimuthDegrees(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setAzimuthDegrees");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setAzimuthDegrees(a0);
	}
	void setElevationDegrees(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setElevationDegrees");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setElevationDegrees(a0);
	}
	void setCameraProjective(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setCameraProjective");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraProjective(a0);
	}
	void setCameraFOV(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "setCameraFOV");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::setCameraFOV(a0);
	}
	void OnUserManuallyMovesCamera(float a0, float a1, float a2, float a3, float a4, float a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "OnUserManuallyMovesCamera");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGlCanvasBase::OnUserManuallyMovesCamera(a0, a1, a2, a3, a4, a5);
	}
	double renderCanvas(int a0, int a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::gui::CGlCanvasBaseHeadless *>(this), "renderCanvas");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CGlCanvasBase::renderCanvas(a0, a1);
	}
};

void bind_mrpt_gui_CGlCanvasBase(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::gui::CGlCanvasBaseHeadless file:mrpt/gui/CGlCanvasBase.h line:250
		pybind11::class_<mrpt::gui::CGlCanvasBaseHeadless, std::shared_ptr<mrpt::gui::CGlCanvasBaseHeadless>, PyCallBack_mrpt_gui_CGlCanvasBaseHeadless, mrpt::gui::CGlCanvasBase> cl(M("mrpt::gui"), "CGlCanvasBaseHeadless", "A headless dummy implementation of CGlCanvasBase: can be used to keep track\n of user UI mouse events and update the camera parameters, with actual\n rendering being delegated to someone else. \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::gui::CGlCanvasBaseHeadless(); }, [](){ return new PyCallBack_mrpt_gui_CGlCanvasBaseHeadless(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_gui_CGlCanvasBaseHeadless const &o){ return new PyCallBack_mrpt_gui_CGlCanvasBaseHeadless(o); } ) );
		cl.def( pybind11::init( [](mrpt::gui::CGlCanvasBaseHeadless const &o){ return new mrpt::gui::CGlCanvasBaseHeadless(o); } ) );
		cl.def("assign", (class mrpt::gui::CGlCanvasBaseHeadless & (mrpt::gui::CGlCanvasBaseHeadless::*)(const class mrpt::gui::CGlCanvasBaseHeadless &)) &mrpt::gui::CGlCanvasBaseHeadless::operator=, "C++: mrpt::gui::CGlCanvasBaseHeadless::operator=(const class mrpt::gui::CGlCanvasBaseHeadless &) --> class mrpt::gui::CGlCanvasBaseHeadless &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
