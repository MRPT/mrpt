/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>

namespace mrpt::containers
{
class Parameters;
namespace internal
{
struct tag_as_proxy_t
{
};
struct tag_as_const_proxy_t
{
};
template <typename T>
const T& implAsGetter(const Parameters& p, const char* expectedType);
template <typename T>
const T& asGetter(const Parameters& p);
}  // namespace internal

/** Container for (possibly-nested) blocks of parameters of type `std::string`
 * or `double`.
 *
 *  \code
 * 	mrpt::containers::Parameters p;
 * 	p["N"] = 10;
 * 	auto& pid = p["PID"] = mrpt::containers::Parameters();
 * 	pid["Kp"] = 0.5;
 * 	p["PID"]["Ti"] = 2.0;
 * 	p["PID"]["N"].as<uint64_t>() = 1000;
 * 	p["PID"]["name"] = "foo";
 *
 * 	std::cout << p["PID"]["Kp"].as<double>() << "\n";
 *	std::cout << p["PID"]["Ti"].as<double>() << "\n";
 * 	std::cout << p["PID"]["N"].as<uint64_t>() << "\n";
 * 	std::cout << p["PID"]["name"].as<std::string>() << "\n";
 *  \endcode
 *
 * \ingroup mrpt_containers_grp
 * \note [new in MRPT 1.0.5]
 */
class Parameters
{
   public:
	using value_t =
		std::variant<std::monostate, double, uint64_t, std::string, Parameters>;
	using parameter_name_t = std::string;
	using data_t = std::map<parameter_name_t, value_t>;

	Parameters() = default;
	~Parameters() = default;

	/** Constructor from list of pairs of values. See examples in Parameters
	 * above. */
	Parameters(std::initializer_list<data_t::value_type> init) : data_(init) {}

	bool has(const std::string& s) const;
	bool empty() const;
	void clear();

	/** Returns the type of the given parameter, or an empty string if it does
	 * not exist. Possible return values are: "uninitialized", "double",
	 * "uint64_t", "std::string", "Parameters", "undefined" (should never
	 * happen).
	 */
	std::string typeOf(const std::string& name) const;

	/** Write access. */
	Parameters operator[](const char* s);
	inline Parameters operator[](const std::string& s)
	{
		return operator[](s.c_str());
	}

	/** Read access \throw std::runtime_error if parameter does not exist. */
	const Parameters operator[](const char* s) const;
	inline const Parameters operator[](const std::string& s) const
	{
		return operator[](s.c_str());
	}

   private:
	data_t data_;
	bool isProxy_ = false;
	bool isConstProxy_ = false;

	/** @name Internal proxy
	 * @{ */
   private:
	explicit Parameters(
		internal::tag_as_proxy_t, value_t& val, const char* name)
		: isProxy_(true), isConstProxy_(false), name_(name), valuenc_(&val)
	{
	}
	explicit Parameters(
		internal::tag_as_const_proxy_t, const value_t& val, const char* name)
		: isProxy_(true), isConstProxy_(true), name_(name), value_(&val)
	{
	}

   public:
	/** Returns a const ref to the existing value of the given type.
	 * \exception std::exception If the contained type does not match.
	 */
	template <typename T>
	const T& as() const
	{
		return internal::asGetter<T>(*this);
	}
	/** Returns a ref to the existing or new value of the given type.
	 */
	template <typename T>
	T& as()
	{
		if (isConstProxy_)
			throw std::logic_error("Trying to write into read-only proxy");
		if (!isProxy_)
			throw std::logic_error(
				"Trying to read from a Parameter block. Use "
				"`p[\"name\"].as<T>();` instead");

		if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
		if (!std::holds_alternative<T>(*valuenc_)) *valuenc_ = T();
		return std::get<T>(*valuenc_);
	}

	void operator=(const double v)
	{
		if (isConstProxy_)
			throw std::logic_error("Trying to write into read-only proxy");
		if (!isProxy_)
			throw std::logic_error(
				"Trying to write into a Parameter block. Use "
				"`p[\"name\"]=value;` instead");
		if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
		*valuenc_ = v;
	}
	void operator=(const std::string& v)
	{
		if (isConstProxy_)
			throw std::logic_error("Trying to write into read-only proxy");
		if (!isProxy_)
			throw std::logic_error(
				"Trying to write into a Parameter block. Use "
				"`p[\"name\"]=value;` instead");
		if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
		*valuenc_ = v;
	}
	Parameters& operator=(const Parameters& v)
	{
		if (isConstProxy_)
			throw std::logic_error("Trying to write into read-only proxy");

		if (isProxy_)
		{
			if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
			*valuenc_ = v;
			return std::get<Parameters>(*valuenc_);
		}
		else
		{
			data_ = v.data_;
			isProxy_ = false;
			isConstProxy_ = false;
			return *this;
		}
	}

	inline operator double() const { return as<double>(); }
	inline operator const std::string&() const { return as<std::string>(); }

   private:
	const char* name_ = nullptr;
	const value_t* value_ = nullptr;
	value_t* valuenc_ = nullptr;

	const Parameters* internalValueAsSelf() const;
	Parameters* internalValueAsSelf();

	const Parameters* internalMeOrValue() const;
	Parameters* internalMeOrValue();

	template <typename T>
	friend const T& internal::implAsGetter(
		const Parameters& p, const char* expectedType);

	/** @} */
};

}  // namespace mrpt::containers
