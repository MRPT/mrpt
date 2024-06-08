#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
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

// mrpt::nav::CHolonomicFullEval file:mrpt/nav/holonomic/CHolonomicFullEval.h line:57
struct PyCallBack_mrpt_nav_CHolonomicFullEval : public mrpt::nav::CHolonomicFullEval {
	using mrpt::nav::CHolonomicFullEval::CHolonomicFullEval;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CHolonomicFullEval::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CHolonomicFullEval::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CHolonomicFullEval::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicFullEval::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicFullEval::serializeFrom(a0, a1);
	}
	void navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput & a0, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "navigate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicFullEval::navigate(a0, a1);
	}
	void initialize(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicFullEval::initialize(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicFullEval::saveConfigFile(a0);
	}
	double getTargetApproachSlowDownDistance() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "getTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHolonomicFullEval::getTargetApproachSlowDownDistance();
	}
	void setTargetApproachSlowDownDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval *>(this), "setTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicFullEval::setTargetApproachSlowDownDistance(a0);
	}
};

// mrpt::nav::CHolonomicFullEval::TOptions file:mrpt/nav/holonomic/CHolonomicFullEval.h line:72
struct PyCallBack_mrpt_nav_CHolonomicFullEval_TOptions : public mrpt::nav::CHolonomicFullEval::TOptions {
	using mrpt::nav::CHolonomicFullEval::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicFullEval::TOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::nav::CLogFileRecord_FullEval file:mrpt/nav/holonomic/CHolonomicFullEval.h line:159
struct PyCallBack_mrpt_nav_CLogFileRecord_FullEval : public mrpt::nav::CLogFileRecord_FullEval {
	using mrpt::nav::CLogFileRecord_FullEval::CLogFileRecord_FullEval;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_FullEval *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CLogFileRecord_FullEval::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_FullEval *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CLogFileRecord_FullEval::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_FullEval *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CLogFileRecord_FullEval::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_FullEval *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord_FullEval::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_FullEval *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord_FullEval::serializeFrom(a0, a1);
	}
	const class mrpt::math::CMatrixD * getDirectionScores() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_FullEval *>(this), "getDirectionScores");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::math::CMatrixD *>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::math::CMatrixD *> caster;
				return pybind11::detail::cast_ref<const class mrpt::math::CMatrixD *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::math::CMatrixD *>(std::move(o));
		}
		return CLogFileRecord_FullEval::getDirectionScores();
	}
};

// mrpt::nav::CHolonomicND file:mrpt/nav/holonomic/CHolonomicND.h line:53
struct PyCallBack_mrpt_nav_CHolonomicND : public mrpt::nav::CHolonomicND {
	using mrpt::nav::CHolonomicND::CHolonomicND;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CHolonomicND::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CHolonomicND::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CHolonomicND::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicND::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicND::serializeFrom(a0, a1);
	}
	void navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput & a0, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "navigate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicND::navigate(a0, a1);
	}
	void initialize(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicND::initialize(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicND::saveConfigFile(a0);
	}
	double getTargetApproachSlowDownDistance() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "getTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHolonomicND::getTargetApproachSlowDownDistance();
	}
	void setTargetApproachSlowDownDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND *>(this), "setTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicND::setTargetApproachSlowDownDistance(a0);
	}
};

// mrpt::nav::CHolonomicND::TOptions file:mrpt/nav/holonomic/CHolonomicND.h line:91
struct PyCallBack_mrpt_nav_CHolonomicND_TOptions : public mrpt::nav::CHolonomicND::TOptions {
	using mrpt::nav::CHolonomicND::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicND::TOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_holonomic_CHolonomicFullEval(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CHolonomicFullEval file:mrpt/nav/holonomic/CHolonomicFullEval.h line:57
		pybind11::class_<mrpt::nav::CHolonomicFullEval, std::shared_ptr<mrpt::nav::CHolonomicFullEval>, PyCallBack_mrpt_nav_CHolonomicFullEval, mrpt::nav::CAbstractHolonomicReactiveMethod> cl(M("mrpt::nav"), "CHolonomicFullEval", "Full evaluation of all possible directions within the discrete set of input\n directions.\n\n These are the optional parameters of the method which can be set by means of\n a configuration file passed to the constructor or to\n CHolonomicFullEval::initialize() or directly in \n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  \n CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicFullEval(); }, [](){ return new PyCallBack_mrpt_nav_CHolonomicFullEval(); } ), "doc");
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase *>(), pybind11::arg("INI_FILE") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CHolonomicFullEval const &o){ return new PyCallBack_mrpt_nav_CHolonomicFullEval(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CHolonomicFullEval const &o){ return new mrpt::nav::CHolonomicFullEval(o); } ) );
		cl.def_readwrite("options", &mrpt::nav::CHolonomicFullEval::options);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CHolonomicFullEval::GetRuntimeClassIdStatic, "C++: mrpt::nav::CHolonomicFullEval::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CHolonomicFullEval::*)() const) &mrpt::nav::CHolonomicFullEval::GetRuntimeClass, "C++: mrpt::nav::CHolonomicFullEval::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CHolonomicFullEval::*)() const) &mrpt::nav::CHolonomicFullEval::clone, "C++: mrpt::nav::CHolonomicFullEval::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CHolonomicFullEval::CreateObject, "C++: mrpt::nav::CHolonomicFullEval::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("navigate", (void (mrpt::nav::CHolonomicFullEval::*)(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &)) &mrpt::nav::CHolonomicFullEval::navigate, "C++: mrpt::nav::CHolonomicFullEval::navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &) --> void", pybind11::arg("ni"), pybind11::arg("no"));
		cl.def("initialize", (void (mrpt::nav::CHolonomicFullEval::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CHolonomicFullEval::initialize, "C++: mrpt::nav::CHolonomicFullEval::initialize(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("INI_FILE"));
		cl.def("saveConfigFile", (void (mrpt::nav::CHolonomicFullEval::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CHolonomicFullEval::saveConfigFile, "C++: mrpt::nav::CHolonomicFullEval::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("getTargetApproachSlowDownDistance", (double (mrpt::nav::CHolonomicFullEval::*)() const) &mrpt::nav::CHolonomicFullEval::getTargetApproachSlowDownDistance, "C++: mrpt::nav::CHolonomicFullEval::getTargetApproachSlowDownDistance() const --> double");
		cl.def("setTargetApproachSlowDownDistance", (void (mrpt::nav::CHolonomicFullEval::*)(const double)) &mrpt::nav::CHolonomicFullEval::setTargetApproachSlowDownDistance, "C++: mrpt::nav::CHolonomicFullEval::setTargetApproachSlowDownDistance(const double) --> void", pybind11::arg("dist"));
		cl.def("assign", (class mrpt::nav::CHolonomicFullEval & (mrpt::nav::CHolonomicFullEval::*)(const class mrpt::nav::CHolonomicFullEval &)) &mrpt::nav::CHolonomicFullEval::operator=, "C++: mrpt::nav::CHolonomicFullEval::operator=(const class mrpt::nav::CHolonomicFullEval &) --> class mrpt::nav::CHolonomicFullEval &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::CHolonomicFullEval::TOptions file:mrpt/nav/holonomic/CHolonomicFullEval.h line:72
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CHolonomicFullEval::TOptions, std::shared_ptr<mrpt::nav::CHolonomicFullEval::TOptions>, PyCallBack_mrpt_nav_CHolonomicFullEval_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "Algorithm options ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicFullEval::TOptions(); }, [](){ return new PyCallBack_mrpt_nav_CHolonomicFullEval_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CHolonomicFullEval_TOptions const &o){ return new PyCallBack_mrpt_nav_CHolonomicFullEval_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CHolonomicFullEval::TOptions const &o){ return new mrpt::nav::CHolonomicFullEval::TOptions(o); } ) );
			cl.def_readwrite("TOO_CLOSE_OBSTACLE", &mrpt::nav::CHolonomicFullEval::TOptions::TOO_CLOSE_OBSTACLE);
			cl.def_readwrite("TARGET_SLOW_APPROACHING_DISTANCE", &mrpt::nav::CHolonomicFullEval::TOptions::TARGET_SLOW_APPROACHING_DISTANCE);
			cl.def_readwrite("OBSTACLE_SLOW_DOWN_DISTANCE", &mrpt::nav::CHolonomicFullEval::TOptions::OBSTACLE_SLOW_DOWN_DISTANCE);
			cl.def_readwrite("HYSTERESIS_SECTOR_COUNT", &mrpt::nav::CHolonomicFullEval::TOptions::HYSTERESIS_SECTOR_COUNT);
			cl.def_readwrite("factorWeights", &mrpt::nav::CHolonomicFullEval::TOptions::factorWeights);
			cl.def_readwrite("factorNormalizeOrNot", &mrpt::nav::CHolonomicFullEval::TOptions::factorNormalizeOrNot);
			cl.def_readwrite("PHASE_FACTORS", &mrpt::nav::CHolonomicFullEval::TOptions::PHASE_FACTORS);
			cl.def_readwrite("PHASE_THRESHOLDS", &mrpt::nav::CHolonomicFullEval::TOptions::PHASE_THRESHOLDS);
			cl.def_readwrite("LOG_SCORE_MATRIX", &mrpt::nav::CHolonomicFullEval::TOptions::LOG_SCORE_MATRIX);
			cl.def_readwrite("clearance_threshold_ratio", &mrpt::nav::CHolonomicFullEval::TOptions::clearance_threshold_ratio);
			cl.def_readwrite("gap_width_ratio_threshold", &mrpt::nav::CHolonomicFullEval::TOptions::gap_width_ratio_threshold);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CHolonomicFullEval::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CHolonomicFullEval::TOptions::loadFromConfigFile, "C++: mrpt::nav::CHolonomicFullEval::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CHolonomicFullEval::TOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CHolonomicFullEval::TOptions::saveToConfigFile, "C++: mrpt::nav::CHolonomicFullEval::TOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::nav::CHolonomicFullEval::TOptions & (mrpt::nav::CHolonomicFullEval::TOptions::*)(const struct mrpt::nav::CHolonomicFullEval::TOptions &)) &mrpt::nav::CHolonomicFullEval::TOptions::operator=, "C++: mrpt::nav::CHolonomicFullEval::TOptions::operator=(const struct mrpt::nav::CHolonomicFullEval::TOptions &) --> struct mrpt::nav::CHolonomicFullEval::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::nav::CLogFileRecord_FullEval file:mrpt/nav/holonomic/CHolonomicFullEval.h line:159
		pybind11::class_<mrpt::nav::CLogFileRecord_FullEval, std::shared_ptr<mrpt::nav::CLogFileRecord_FullEval>, PyCallBack_mrpt_nav_CLogFileRecord_FullEval, mrpt::nav::CHolonomicLogFileRecord> cl(M("mrpt::nav"), "CLogFileRecord_FullEval", "A class for storing extra information about the execution of\n CHolonomicFullEval navigation.\n \n\n CHolonomicFullEval, CHolonomicLogFileRecord");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CLogFileRecord_FullEval(); }, [](){ return new PyCallBack_mrpt_nav_CLogFileRecord_FullEval(); } ) );
		cl.def_readwrite("selectedSector", &mrpt::nav::CLogFileRecord_FullEval::selectedSector);
		cl.def_readwrite("evaluation", &mrpt::nav::CLogFileRecord_FullEval::evaluation);
		cl.def_readwrite("dirs_scores", &mrpt::nav::CLogFileRecord_FullEval::dirs_scores);
		cl.def_readwrite("selectedTarget", &mrpt::nav::CLogFileRecord_FullEval::selectedTarget);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CLogFileRecord_FullEval::GetRuntimeClassIdStatic, "C++: mrpt::nav::CLogFileRecord_FullEval::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CLogFileRecord_FullEval::*)() const) &mrpt::nav::CLogFileRecord_FullEval::GetRuntimeClass, "C++: mrpt::nav::CLogFileRecord_FullEval::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CLogFileRecord_FullEval::*)() const) &mrpt::nav::CLogFileRecord_FullEval::clone, "C++: mrpt::nav::CLogFileRecord_FullEval::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CLogFileRecord_FullEval::CreateObject, "C++: mrpt::nav::CLogFileRecord_FullEval::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getDirectionScores", (const class mrpt::math::CMatrixD * (mrpt::nav::CLogFileRecord_FullEval::*)() const) &mrpt::nav::CLogFileRecord_FullEval::getDirectionScores, "C++: mrpt::nav::CLogFileRecord_FullEval::getDirectionScores() const --> const class mrpt::math::CMatrixD *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::nav::CLogFileRecord_FullEval & (mrpt::nav::CLogFileRecord_FullEval::*)(const class mrpt::nav::CLogFileRecord_FullEval &)) &mrpt::nav::CLogFileRecord_FullEval::operator=, "C++: mrpt::nav::CLogFileRecord_FullEval::operator=(const class mrpt::nav::CLogFileRecord_FullEval &) --> class mrpt::nav::CLogFileRecord_FullEval &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CHolonomicND file:mrpt/nav/holonomic/CHolonomicND.h line:53
		pybind11::class_<mrpt::nav::CHolonomicND, std::shared_ptr<mrpt::nav::CHolonomicND>, PyCallBack_mrpt_nav_CHolonomicND, mrpt::nav::CAbstractHolonomicReactiveMethod> cl(M("mrpt::nav"), "CHolonomicND", "An implementation of the holonomic reactive navigation method\n \"Nearness-Diagram\".\n   The algorithm \"Nearness-Diagram\" was proposed in:\n\n  Nearness diagram (ND) navigation: collision avoidance in troublesome\n scenarios, IEEE Transactions on\n   Robotics and Automation, Minguez, J. and Montano, L., vol. 20, no. 1, pp.\n 45-59, 2004.\n\n These are the optional parameters of the method which can be set by means of\n a configuration file passed to the constructor or to\n CHolonomicND::initialize() or directly in \n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  \n CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicND(); }, [](){ return new PyCallBack_mrpt_nav_CHolonomicND(); } ), "doc");
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase *>(), pybind11::arg("INI_FILE") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CHolonomicND const &o){ return new PyCallBack_mrpt_nav_CHolonomicND(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CHolonomicND const &o){ return new mrpt::nav::CHolonomicND(o); } ) );

		pybind11::enum_<mrpt::nav::CHolonomicND::TSituations>(cl, "TSituations", pybind11::arithmetic(), "The set of posible situations for each trajectory.\n (mrpt::typemeta::TEnumType works with this enum) ")
			.value("SITUATION_TARGET_DIRECTLY", mrpt::nav::CHolonomicND::SITUATION_TARGET_DIRECTLY)
			.value("SITUATION_SMALL_GAP", mrpt::nav::CHolonomicND::SITUATION_SMALL_GAP)
			.value("SITUATION_WIDE_GAP", mrpt::nav::CHolonomicND::SITUATION_WIDE_GAP)
			.value("SITUATION_NO_WAY_FOUND", mrpt::nav::CHolonomicND::SITUATION_NO_WAY_FOUND)
			.export_values();

		cl.def_readwrite("options", &mrpt::nav::CHolonomicND::options);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CHolonomicND::GetRuntimeClassIdStatic, "C++: mrpt::nav::CHolonomicND::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CHolonomicND::*)() const) &mrpt::nav::CHolonomicND::GetRuntimeClass, "C++: mrpt::nav::CHolonomicND::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CHolonomicND::*)() const) &mrpt::nav::CHolonomicND::clone, "C++: mrpt::nav::CHolonomicND::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CHolonomicND::CreateObject, "C++: mrpt::nav::CHolonomicND::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("navigate", (void (mrpt::nav::CHolonomicND::*)(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &)) &mrpt::nav::CHolonomicND::navigate, "C++: mrpt::nav::CHolonomicND::navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &) --> void", pybind11::arg("ni"), pybind11::arg("no"));
		cl.def("initialize", (void (mrpt::nav::CHolonomicND::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CHolonomicND::initialize, "Initialize the parameters of the navigator. \n\nC++: mrpt::nav::CHolonomicND::initialize(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("INI_FILE"));
		cl.def("saveConfigFile", (void (mrpt::nav::CHolonomicND::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CHolonomicND::saveConfigFile, "C++: mrpt::nav::CHolonomicND::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("getTargetApproachSlowDownDistance", (double (mrpt::nav::CHolonomicND::*)() const) &mrpt::nav::CHolonomicND::getTargetApproachSlowDownDistance, "C++: mrpt::nav::CHolonomicND::getTargetApproachSlowDownDistance() const --> double");
		cl.def("setTargetApproachSlowDownDistance", (void (mrpt::nav::CHolonomicND::*)(const double)) &mrpt::nav::CHolonomicND::setTargetApproachSlowDownDistance, "C++: mrpt::nav::CHolonomicND::setTargetApproachSlowDownDistance(const double) --> void", pybind11::arg("dist"));
		cl.def("assign", (class mrpt::nav::CHolonomicND & (mrpt::nav::CHolonomicND::*)(const class mrpt::nav::CHolonomicND &)) &mrpt::nav::CHolonomicND::operator=, "C++: mrpt::nav::CHolonomicND::operator=(const class mrpt::nav::CHolonomicND &) --> class mrpt::nav::CHolonomicND &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::CHolonomicND::TGap file:mrpt/nav/holonomic/CHolonomicND.h line:65
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CHolonomicND::TGap, std::shared_ptr<mrpt::nav::CHolonomicND::TGap>> cl(enclosing_class, "TGap", "The structure used to store a detected gap in obstacles. ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicND::TGap(); } ) );
			cl.def_readwrite("ini", &mrpt::nav::CHolonomicND::TGap::ini);
			cl.def_readwrite("end", &mrpt::nav::CHolonomicND::TGap::end);
			cl.def_readwrite("maxDistance", &mrpt::nav::CHolonomicND::TGap::maxDistance);
			cl.def_readwrite("minDistance", &mrpt::nav::CHolonomicND::TGap::minDistance);
			cl.def_readwrite("representative_sector", &mrpt::nav::CHolonomicND::TGap::representative_sector);
		}

		{ // mrpt::nav::CHolonomicND::TOptions file:mrpt/nav/holonomic/CHolonomicND.h line:91
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CHolonomicND::TOptions, std::shared_ptr<mrpt::nav::CHolonomicND::TOptions>, PyCallBack_mrpt_nav_CHolonomicND_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "Algorithm options ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicND::TOptions(); }, [](){ return new PyCallBack_mrpt_nav_CHolonomicND_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CHolonomicND_TOptions const &o){ return new PyCallBack_mrpt_nav_CHolonomicND_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CHolonomicND::TOptions const &o){ return new mrpt::nav::CHolonomicND::TOptions(o); } ) );
			cl.def_readwrite("TOO_CLOSE_OBSTACLE", &mrpt::nav::CHolonomicND::TOptions::TOO_CLOSE_OBSTACLE);
			cl.def_readwrite("WIDE_GAP_SIZE_PERCENT", &mrpt::nav::CHolonomicND::TOptions::WIDE_GAP_SIZE_PERCENT);
			cl.def_readwrite("RISK_EVALUATION_SECTORS_PERCENT", &mrpt::nav::CHolonomicND::TOptions::RISK_EVALUATION_SECTORS_PERCENT);
			cl.def_readwrite("RISK_EVALUATION_DISTANCE", &mrpt::nav::CHolonomicND::TOptions::RISK_EVALUATION_DISTANCE);
			cl.def_readwrite("MAX_SECTOR_DIST_FOR_D2_PERCENT", &mrpt::nav::CHolonomicND::TOptions::MAX_SECTOR_DIST_FOR_D2_PERCENT);
			cl.def_readwrite("TARGET_SLOW_APPROACHING_DISTANCE", &mrpt::nav::CHolonomicND::TOptions::TARGET_SLOW_APPROACHING_DISTANCE);
			cl.def_readwrite("factorWeights", &mrpt::nav::CHolonomicND::TOptions::factorWeights);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CHolonomicND::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CHolonomicND::TOptions::loadFromConfigFile, "C++: mrpt::nav::CHolonomicND::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CHolonomicND::TOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CHolonomicND::TOptions::saveToConfigFile, "C++: mrpt::nav::CHolonomicND::TOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::nav::CHolonomicND::TOptions & (mrpt::nav::CHolonomicND::TOptions::*)(const struct mrpt::nav::CHolonomicND::TOptions &)) &mrpt::nav::CHolonomicND::TOptions::operator=, "C++: mrpt::nav::CHolonomicND::TOptions::operator=(const struct mrpt::nav::CHolonomicND::TOptions &) --> struct mrpt::nav::CHolonomicND::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
