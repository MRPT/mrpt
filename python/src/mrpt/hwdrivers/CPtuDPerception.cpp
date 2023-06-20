#include <iterator>
#include <memory>
#include <mrpt/hwdrivers/CPtuDPerception.h>
#include <sstream> // __str__
#include <string>

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

// mrpt::hwdrivers::CPtuDPerception file:mrpt/hwdrivers/CPtuDPerception.h line:21
struct PyCallBack_mrpt_hwdrivers_CPtuDPerception : public mrpt::hwdrivers::CPtuDPerception {
	using mrpt::hwdrivers::CPtuDPerception::CPtuDPerception;

	bool rangeMeasure() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "rangeMeasure");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::rangeMeasure();
	}
	bool moveToAbsPos(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "moveToAbsPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::moveToAbsPos(a0, a1);
	}
	bool absPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "absPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::absPosQ(a0, a1);
	}
	bool moveToOffPos(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "moveToOffPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::moveToOffPos(a0, a1);
	}
	bool offPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "offPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::offPosQ(a0, a1);
	}
	bool maxPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "maxPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::maxPosQ(a0, a1);
	}
	bool minPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "minPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::minPosQ(a0, a1);
	}
	bool enableLimitsQ(bool & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "enableLimitsQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::enableLimitsQ(a0);
	}
	bool enableLimits(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "enableLimits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::enableLimits(a0);
	}
	bool inmediateExecution(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "inmediateExecution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::inmediateExecution(a0);
	}
	bool aWait() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "aWait");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::aWait();
	}
	bool haltAll() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "haltAll");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::haltAll();
	}
	bool halt(char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "halt");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::halt(a0);
	}
	bool speed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "speed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::speed(a0, a1);
	}
	bool speedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "speedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::speedQ(a0, a1);
	}
	bool aceleration(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "aceleration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::aceleration(a0, a1);
	}
	bool acelerationQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "acelerationQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::acelerationQ(a0, a1);
	}
	bool baseSpeed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "baseSpeed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::baseSpeed(a0, a1);
	}
	bool baseSpeedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "baseSpeedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::baseSpeedQ(a0, a1);
	}
	bool upperSpeed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "upperSpeed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::upperSpeed(a0, a1);
	}
	bool upperSpeedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "upperSpeedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::upperSpeedQ(a0, a1);
	}
	bool lowerSpeed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "lowerSpeed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::lowerSpeed(a0, a1);
	}
	bool lowerSpeedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "lowerSpeedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::lowerSpeedQ(a0, a1);
	}
	bool reset() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "reset");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::reset();
	}
	bool save() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "save");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::save();
	}
	bool restoreDefaults() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "restoreDefaults");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::restoreDefaults();
	}
	bool restoreFactoryDefaults() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "restoreFactoryDefaults");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::restoreFactoryDefaults();
	}
	bool version(char * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "version");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::version(a0);
	}
	void nversion(double & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "nversion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPtuDPerception::nversion(a0);
	}
	bool powerModeQ(bool a0, char & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "powerModeQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::powerModeQ(a0, a1);
	}
	bool powerMode(bool a0, char a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "powerMode");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::powerMode(a0, a1);
	}
	double status(double & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "status");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPtuDPerception::status(a0);
	}
	bool setLimits(char a0, double & a1, double & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "setLimits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::setLimits(a0, a1, a2);
	}
	bool changeMotionDir() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "changeMotionDir");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::changeMotionDir();
	}
	int checkErrors() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "checkErrors");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return CPtuDPerception::checkErrors();
	}
	void clearErrors() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "clearErrors");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPtuDPerception::clearErrors();
	}
	bool init(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "init");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::init(a0);
	}
	void close() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "close");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPtuDPerception::close();
	}
	double radError(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "radError");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPtuDPerception::radError(a0, a1);
	}
	long radToPos(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "radToPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<long>::value) {
				static pybind11::detail::override_caster_t<long> caster;
				return pybind11::detail::cast_ref<long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<long>(std::move(o));
		}
		return CPtuDPerception::radToPos(a0, a1);
	}
	double posToRad(char a0, long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "posToRad");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPtuDPerception::posToRad(a0, a1);
	}
	bool scan(char a0, int a1, float a2, float a3, double a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "scan");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::scan(a0, a1, a2, a3, a4);
	}
	bool verboseQ(bool & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "verboseQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::verboseQ(a0);
	}
	bool verbose(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "verbose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::verbose(a0);
	}
	bool echoModeQ(bool & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "echoModeQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::echoModeQ(a0);
	}
	bool echoMode(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "echoMode");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::echoMode(a0);
	}
	bool resolution() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPtuDPerception *>(this), "resolution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPtuDPerception::resolution();
	}
};

void bind_mrpt_hwdrivers_CPtuDPerception(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CPtuDPerception file:mrpt/hwdrivers/CPtuDPerception.h line:21
		pybind11::class_<mrpt::hwdrivers::CPtuDPerception, std::shared_ptr<mrpt::hwdrivers::CPtuDPerception>, PyCallBack_mrpt_hwdrivers_CPtuDPerception, mrpt::hwdrivers::CPtuBase> cl(M("mrpt::hwdrivers"), "CPtuDPerception", "This class implements initialization and communication methods to\n control a Pan and Tilt Unit model PTU-46-17.5, working in radians .\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CPtuDPerception(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CPtuDPerception(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_hwdrivers_CPtuDPerception const &o){ return new PyCallBack_mrpt_hwdrivers_CPtuDPerception(o); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::CPtuDPerception const &o){ return new mrpt::hwdrivers::CPtuDPerception(o); } ) );
		cl.def_readwrite("nError", &mrpt::hwdrivers::CPtuDPerception::nError);
		cl.def("rangeMeasure", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::rangeMeasure, "Search limit forward \n\nC++: mrpt::hwdrivers::CPtuDPerception::rangeMeasure() --> bool");
		cl.def("moveToAbsPos", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::moveToAbsPos, "Specification of positions in absolute terms \n\nC++: mrpt::hwdrivers::CPtuDPerception::moveToAbsPos(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("absPosQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::absPosQ, "Query position in absolute terms \n\nC++: mrpt::hwdrivers::CPtuDPerception::absPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("moveToOffPos", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::moveToOffPos, "Specify desired axis position as an offset from the current position. \n	This method recives the number of radians to move.\n	\n\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CPtuDPerception::moveToOffPos(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("offPosQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::offPosQ, "Query position in relative terms \n\nC++: mrpt::hwdrivers::CPtuDPerception::offPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("maxPosQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::maxPosQ, "Query max movement limit of a axis in absolute terms \n\nC++: mrpt::hwdrivers::CPtuDPerception::maxPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("minPosQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::minPosQ, "Query min movement limit of a axis in absolute terms \n\nC++: mrpt::hwdrivers::CPtuDPerception::minPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("enableLimitsQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool &)) &mrpt::hwdrivers::CPtuDPerception::enableLimitsQ, "Query if exist movement limits \n\nC++: mrpt::hwdrivers::CPtuDPerception::enableLimitsQ(bool &) --> bool", pybind11::arg("enable"));
		cl.def("enableLimits", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool)) &mrpt::hwdrivers::CPtuDPerception::enableLimits, "Enable/Disable movement limits \n\nC++: mrpt::hwdrivers::CPtuDPerception::enableLimits(bool) --> bool", pybind11::arg("set"));
		cl.def("inmediateExecution", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool)) &mrpt::hwdrivers::CPtuDPerception::inmediateExecution, "With I mode (default) instructs pan-tilt unit to immediately\n	execute positional commands. \n	In S mode instructs pan-tilt unit to execute positional commands\n	only when an Await Position Command Completion command is executed\n	or when put into Immediate Execution Mode. \n	\n\n\n\n\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CPtuDPerception::inmediateExecution(bool) --> bool", pybind11::arg("set"));
		cl.def("aWait", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::aWait, "Wait the finish of the last position command to\n	continue accept commands\n\nC++: mrpt::hwdrivers::CPtuDPerception::aWait() --> bool");
		cl.def("haltAll", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::haltAll, "Inmediately stop all \n\nC++: mrpt::hwdrivers::CPtuDPerception::haltAll() --> bool");
		cl.def("halt", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char)) &mrpt::hwdrivers::CPtuDPerception::halt, "Inmediately stop \n\nC++: mrpt::hwdrivers::CPtuDPerception::halt(char) --> bool", pybind11::arg("axis"));
		cl.def("speed", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::speed, "Specification of turn speed \n\nC++: mrpt::hwdrivers::CPtuDPerception::speed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("speedQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::speedQ, "Query turn speed \n\nC++: mrpt::hwdrivers::CPtuDPerception::speedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("aceleration", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::aceleration, "Specification (de/a)celeration in turn \n\nC++: mrpt::hwdrivers::CPtuDPerception::aceleration(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec2"));
		cl.def("acelerationQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::acelerationQ, "Query (de/a)celeration in turn \n\nC++: mrpt::hwdrivers::CPtuDPerception::acelerationQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec2"));
		cl.def("baseSpeed", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::baseSpeed, "Specification of velocity to which start and finish\n	the (de/a)celeration\n\nC++: mrpt::hwdrivers::CPtuDPerception::baseSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("baseSpeedQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::baseSpeedQ, "Query velocity to which start and finish\n	the (de/a)celeration\n\nC++: mrpt::hwdrivers::CPtuDPerception::baseSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("upperSpeed", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::upperSpeed, "Specification of velocity upper limit \n\nC++: mrpt::hwdrivers::CPtuDPerception::upperSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("upperSpeedQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::upperSpeedQ, "Query velocity upper limit \n\nC++: mrpt::hwdrivers::CPtuDPerception::upperSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("lowerSpeed", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::lowerSpeed, "Specification of velocity lower limit \n\nC++: mrpt::hwdrivers::CPtuDPerception::lowerSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("lowerSpeedQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &)) &mrpt::hwdrivers::CPtuDPerception::lowerSpeedQ, "Query velocity lower limit \n\nC++: mrpt::hwdrivers::CPtuDPerception::lowerSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("reset", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::reset, "Reset PTU to initial state \n\nC++: mrpt::hwdrivers::CPtuDPerception::reset() --> bool");
		cl.def("save", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::save, "Save or restart default values \n\nC++: mrpt::hwdrivers::CPtuDPerception::save() --> bool");
		cl.def("restoreDefaults", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::restoreDefaults, "Restore default values \n\nC++: mrpt::hwdrivers::CPtuDPerception::restoreDefaults() --> bool");
		cl.def("restoreFactoryDefaults", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::restoreFactoryDefaults, "Restore factory default values \n\nC++: mrpt::hwdrivers::CPtuDPerception::restoreFactoryDefaults() --> bool");
		cl.def("version", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char *)) &mrpt::hwdrivers::CPtuDPerception::version, "Version and CopyRights \n\nC++: mrpt::hwdrivers::CPtuDPerception::version(char *) --> bool", pybind11::arg("nVersion"));
		cl.def("nversion", (void (mrpt::hwdrivers::CPtuDPerception::*)(double &)) &mrpt::hwdrivers::CPtuDPerception::nversion, "Number of version \n\nC++: mrpt::hwdrivers::CPtuDPerception::nversion(double &) --> void", pybind11::arg("nVersion"));
		cl.def("powerModeQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool, char &)) &mrpt::hwdrivers::CPtuDPerception::powerModeQ, "Query power mode \n\nC++: mrpt::hwdrivers::CPtuDPerception::powerModeQ(bool, char &) --> bool", pybind11::arg("transit"), pybind11::arg("mode"));
		cl.def("powerMode", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool, char)) &mrpt::hwdrivers::CPtuDPerception::powerMode, "Specification of power mode \n\nC++: mrpt::hwdrivers::CPtuDPerception::powerMode(bool, char) --> bool", pybind11::arg("transit"), pybind11::arg("mode"));
		cl.def("status", (double (mrpt::hwdrivers::CPtuDPerception::*)(double &)) &mrpt::hwdrivers::CPtuDPerception::status, "Check if ptu is moving \n\nC++: mrpt::hwdrivers::CPtuDPerception::status(double &) --> double", pybind11::arg("rad"));
		cl.def("setLimits", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, double &, double &)) &mrpt::hwdrivers::CPtuDPerception::setLimits, "Set limits of movement \n\nC++: mrpt::hwdrivers::CPtuDPerception::setLimits(char, double &, double &) --> bool", pybind11::arg("axis"), pybind11::arg("l"), pybind11::arg("u"));
		cl.def("changeMotionDir", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::changeMotionDir, "C++: mrpt::hwdrivers::CPtuDPerception::changeMotionDir() --> bool");
		cl.def("checkErrors", (int (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::checkErrors, "Check errors, returns 0 if there are not errors or error code in\notherwise\n	Error codes:\n	\n\n\n\n\n\n\n\n\n\n\n\n\n\nC++: mrpt::hwdrivers::CPtuDPerception::checkErrors() --> int");
		cl.def("noError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::noError, "C++: mrpt::hwdrivers::CPtuDPerception::noError() --> bool");
		cl.def("comError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::comError, "C++: mrpt::hwdrivers::CPtuDPerception::comError() --> bool");
		cl.def("timeoutError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::timeoutError, "C++: mrpt::hwdrivers::CPtuDPerception::timeoutError() --> bool");
		cl.def("initError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::initError, "C++: mrpt::hwdrivers::CPtuDPerception::initError() --> bool");
		cl.def("panTiltHitError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::panTiltHitError, "C++: mrpt::hwdrivers::CPtuDPerception::panTiltHitError() --> bool");
		cl.def("panHitError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::panHitError, "C++: mrpt::hwdrivers::CPtuDPerception::panHitError() --> bool");
		cl.def("tiltHitError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::tiltHitError, "C++: mrpt::hwdrivers::CPtuDPerception::tiltHitError() --> bool");
		cl.def("maxLimitError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::maxLimitError, "C++: mrpt::hwdrivers::CPtuDPerception::maxLimitError() --> bool");
		cl.def("minLimitError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::minLimitError, "C++: mrpt::hwdrivers::CPtuDPerception::minLimitError() --> bool");
		cl.def("outOfRange", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::outOfRange, "C++: mrpt::hwdrivers::CPtuDPerception::outOfRange() --> bool");
		cl.def("illegalCommandError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::illegalCommandError, "C++: mrpt::hwdrivers::CPtuDPerception::illegalCommandError() --> bool");
		cl.def("unExpectedError", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::unExpectedError, "C++: mrpt::hwdrivers::CPtuDPerception::unExpectedError() --> bool");
		cl.def("clearErrors", (void (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::clearErrors, "Clear errors *\n\nC++: mrpt::hwdrivers::CPtuDPerception::clearErrors() --> void");
		cl.def("init", (bool (mrpt::hwdrivers::CPtuDPerception::*)(const std::string &)) &mrpt::hwdrivers::CPtuDPerception::init, "PTU and serial port initialization \n\nC++: mrpt::hwdrivers::CPtuDPerception::init(const std::string &) --> bool", pybind11::arg("port"));
		cl.def("close", (void (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::close, "Close Connection with serial port \n\nC++: mrpt::hwdrivers::CPtuDPerception::close() --> void");
		cl.def("radError", (double (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::radError, "To obtains the mistake for use discrete values when the movement\n	is expressed in radians. Parameters are the absolute position in\n	radians and the axis desired\n\nC++: mrpt::hwdrivers::CPtuDPerception::radError(char, double) --> double", pybind11::arg("axis"), pybind11::arg("nRadMoved"));
		cl.def("radToPos", (long (mrpt::hwdrivers::CPtuDPerception::*)(char, double)) &mrpt::hwdrivers::CPtuDPerception::radToPos, "To obtain the discrete value for a number of radians \n\nC++: mrpt::hwdrivers::CPtuDPerception::radToPos(char, double) --> long", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("posToRad", (double (mrpt::hwdrivers::CPtuDPerception::*)(char, long)) &mrpt::hwdrivers::CPtuDPerception::posToRad, "To obtain the number of radians for a discrete value \n\nC++: mrpt::hwdrivers::CPtuDPerception::posToRad(char, long) --> double", pybind11::arg("axis"), pybind11::arg("nPos"));
		cl.def("scan", (bool (mrpt::hwdrivers::CPtuDPerception::*)(char, int, float, float, double)) &mrpt::hwdrivers::CPtuDPerception::scan, "Performs a scan in the axis indicated and whit the precision desired. \n		\n\n {Pan or Till} \n		\n\n {Wait time betwen commands} \n		\n\n {initial position}\n		\n\n {final position}\n		\n\n {radians precision for the scan}\n\nC++: mrpt::hwdrivers::CPtuDPerception::scan(char, int, float, float, double) --> bool", pybind11::arg("axis"), pybind11::arg("wait"), pybind11::arg("initial"), pybind11::arg("final"), pybind11::arg("radPre"));
		cl.def("verboseQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool &)) &mrpt::hwdrivers::CPtuDPerception::verboseQ, "Query verbose mode \n\nC++: mrpt::hwdrivers::CPtuDPerception::verboseQ(bool &) --> bool", pybind11::arg("modo"));
		cl.def("verbose", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool)) &mrpt::hwdrivers::CPtuDPerception::verbose, "Set verbose. \n	\n	Example of response with FV (verbose) active:\n		FV *\n		PP * Current pan position is 0\n		Example of response with FT (terse) active:\n		FT *\n		PP * 0\n	\n\nC++: mrpt::hwdrivers::CPtuDPerception::verbose(bool) --> bool", pybind11::arg("set"));
		cl.def("echoModeQ", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool &)) &mrpt::hwdrivers::CPtuDPerception::echoModeQ, "Query echo mode \n\nC++: mrpt::hwdrivers::CPtuDPerception::echoModeQ(bool &) --> bool", pybind11::arg("mode"));
		cl.def("echoMode", (bool (mrpt::hwdrivers::CPtuDPerception::*)(bool)) &mrpt::hwdrivers::CPtuDPerception::echoMode, "Enable/Disable echo response with command. \n	\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CPtuDPerception::echoMode(bool) --> bool", pybind11::arg("mode"));
		cl.def("resolution", (bool (mrpt::hwdrivers::CPtuDPerception::*)()) &mrpt::hwdrivers::CPtuDPerception::resolution, "Query the pan and tilt resolution per position moved\n	and initialize local atributes\n\nC++: mrpt::hwdrivers::CPtuDPerception::resolution() --> bool");
		cl.def("assign", (class mrpt::hwdrivers::CPtuDPerception & (mrpt::hwdrivers::CPtuDPerception::*)(const class mrpt::hwdrivers::CPtuDPerception &)) &mrpt::hwdrivers::CPtuDPerception::operator=, "C++: mrpt::hwdrivers::CPtuDPerception::operator=(const class mrpt::hwdrivers::CPtuDPerception &) --> class mrpt::hwdrivers::CPtuDPerception &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
