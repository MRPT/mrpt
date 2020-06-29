/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/core/demangle.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>

#include <any>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <variant>
#include <vector>

// forward declarations
// clang-format off
namespace YAML { class Node; }
namespace mrpt::containers { class Parameters;
namespace internal {
 enum tag_as_proxy_t {}; enum tag_as_const_proxy_t {};
 template <typename T> T implAsGetter(const Parameters& p);
}
// clang-format on

/** Powerful YAML-like container for possibly-nested blocks of parameters.
 *
 * The class Parameters acts as a "node" in a YAML structure, and can be of one
 *of these types:
 * - Leaf nodes ("scalar values"): Any type, stored as a C++17 std::any.
 * - Sequential container.
 * - Map ("dictionary"): pairs of `name: value`.
 *
 * Elements contained in either sequenties or dictionaries can be either plain
 *leaf types, or another container.
 *
 * This class was designed as a lightweight, while structured, way to pass
 *arbitrarialy-complex parameter blocks.
 *
 * See example in \ref containers_parameters_example/test.cpp
 * \snippet containers_parameters_example/test.cpp example-parameters
 *
 * \ingroup mrpt_containers_grp
 * \note [new in MRPT 2.0.5]
 */
class Parameters
{
   public:
	using scalar_t = std::any;
	using parameter_name_t = std::string;

	using sequence_t = std::vector<scalar_t>;
	using map_t = std::map<parameter_name_t, scalar_t>;

	using data_t = std::variant<std::monostate, sequence_t, map_t>;

	Parameters() = default;
	~Parameters() = default;

	/** Constructor for maps, from list of pairs of values. See examples in
	 * Parameters above. */
	Parameters(std::initializer_list<map_t::value_type> init) : data_(init) {}

	/** Constructor for sequences, from list of values. See examples in
	 * Parameters above. */
	Parameters(std::initializer_list<sequence_t::value_type> init) : data_(init)
	{
	}
	Parameters(const Parameters& v);

	static Parameters Sequence(
		std::initializer_list<sequence_t::value_type> init)
	{
		return Parameters(init);
	}
	static Parameters Sequence()
	{
		Parameters p;
		p.data_.emplace<sequence_t>();
		return p;
	}

	static Parameters Map(std::initializer_list<map_t::value_type> init)
	{
		return Parameters(init);
	}
	static Parameters Map()
	{
		Parameters p;
		p.data_.emplace<map_t>();
		return p;
	}

	/** Builds an object copying the structure and contents from an existing
	 * YAML Node. Requires mrpt built against yamlcpp. */
	static Parameters FromYAML(const YAML::Node& n);

	/** Parses the text as a YAML document, then converts it into a Parameters
	 * object */
	static Parameters FromYAMLText(const std::string& yamlTextBlock);

	/** For map nodes, checks if the given key name exists */
	bool has(const std::string& key) const;
	bool empty() const;
	void clear();

	bool isSequence() const;
	sequence_t& asSequence();
	const sequence_t& asSequence() const;

	bool isMap() const;
	map_t& asMap();
	const map_t& asMap() const;

	bool isScalar() const;

	void printAsYAML(std::ostream& o) const;
	void printAsYAML() const;  //!< prints to std::cout

	/** For map nodes, returns the type of the given child, or typeid(void) if
	 * empty.
	 * \exception std::exception If called on a non-scalar.
	 */
	const std::type_info& typeOfChild(const std::string& key) const;

	/** For scalar nodes, returns its type, or typeid(void) if an empty scalar.
	 * \exception std::exception If called on a non-scalar. */
	const std::type_info& type() const;

	/** Write access for maps */
	Parameters operator[](const char* key);
	/// \overload
	inline Parameters operator[](const std::string& key)
	{
		return operator[](key.c_str());
	}

	/** Read access  for maps
	 * \throw std::runtime_error if key does not exist. */
	const Parameters operator[](const char* key) const;
	/// \overload
	inline const Parameters operator[](const std::string& key) const
	{
		return operator[](key.c_str());
	}
	/** Read access for maps, with default value if key does not exist. */
	template <typename T>
	const T getOrDefault(const std::string& key, const T& defaultValue) const
	{
		const Parameters* p = internalMeOrValue();
		if (p->empty()) return defaultValue;
		if (!p->isMap())
			throw std::logic_error("getOrDefault() is only for map nodes.");

		const map_t& m = std::get<map_t>(p->data_);
		auto it = m.find(key);
		if (m.end() == it) return defaultValue;
		try
		{
			return std::any_cast<T>(it->second);
		}
		catch (const std::bad_any_cast& e)
		{
			throw std::logic_error(mrpt::format(
				"getOrDefault(): Trying to access key `%s` holding type `%s` "
				"as the wrong type: `%e`",
				key.c_str(), it->second.type().name(), e.what()));
		}
	}

	/** Write into an existing index of a sequence.
	 * \throw std::out_of_range if index is out of range. */
	Parameters operator()(int index);
	/** Read from an existing index of a sequence.
	 * \throw std::out_of_range if index is out of range. */
	const Parameters operator()(int index) const;

	/** Append a new value to a sequence.
	 * \throw std::exception If this is not a sequence */
	void push_back(const double v) { internalPushBack(v); }
	/// \overload
	void push_back(const std::string v) { internalPushBack(v); }
	/// \overload
	void push_back(const uint64_t v) { internalPushBack(v); }
	/// \overload
	void push_back(const bool v) { internalPushBack(bool(v)); }
	/// \overload
	void push_back(const Parameters& v) { internalPushBack(v); }

	/** Copies the structure and contents from an existing
	 * YAML Node. Requires mrpt built against yamlcpp. */
	void loadFromYAML(const YAML::Node& n);

   private:
	data_t data_;
	bool isProxy_ = false;
	bool isConstProxy_ = false;

	/** @name Internal proxy
	 * @{ */
   private:
	explicit Parameters(
		internal::tag_as_proxy_t, scalar_t& val, const char* name)
		: isProxy_(true), isConstProxy_(false), name_(name), valuenc_(&val)
	{
	}
	explicit Parameters(
		internal::tag_as_const_proxy_t, const scalar_t& val, const char* name)
		: isProxy_(true), isConstProxy_(true), name_(name), value_(&val)
	{
	}

   public:
	/** Returns a copy of the existing value of the given type, or tries to
	 * convert it between easily-compatible types (e.g. double<->int,
	 * string<->int).
	 * \exception std::exception If the contained type does not  match and there
	 * is no obvious conversion.
	 */
	template <typename T>
	T as() const
	{
		return internal::implAsGetter<T>(*this);
	}
	/** Returns a ref to the existing or new value of the given type. If types
	 * do not match, the old content will be discarded and a new variable
	 * created into this scalar node.
	 * \exception std::exception If accessing to a non-scalar node.
	 */
	template <typename T>
	T& asRef();

	/** const version of asRef(). Unlike `as<T>()`, this version will NOT try to
	 * convert between types if T does not match exactly the stored type, and
	 * will raise an exception instead. */
	template <typename T>
	const T& asRef() const;

	void operator=(const bool v);
	void operator=(const double v);
	void operator=(const uint64_t v);
	void operator=(const std::string& v);
	inline void operator=(const char* v) { operator=(std::string(v)); }
	inline void operator=(const std::string_view& v)
	{
		operator=(std::string(v));
	}
	Parameters& operator=(const Parameters& v);

	inline operator bool() const { return as<bool>(); }
	inline operator uint64_t() const { return as<uint64_t>(); }
	inline operator double() const { return as<double>(); }
	inline operator std::string() const { return as<std::string>(); }

   private:
	const char* name_ = nullptr;
	const scalar_t* value_ = nullptr;
	scalar_t* valuenc_ = nullptr;

	/** Returns the pointer to the referenced object, if a proxy, or nullptr
	 * otherwise */
	const Parameters* internalValueAsSelf() const;
	Parameters* internalValueAsSelf();

	/** Returns me or the pointer to the referenced object, if a proxy */
	const Parameters* internalMeOrValue() const;
	Parameters* internalMeOrValue();

	template <typename T>
	friend T internal::implAsGetter(const Parameters& p);

	// Return: true if the last printed char is a newline char
	static bool internalPrintAsYAML(
		const Parameters& p, std::ostream& o, int indent, bool first);

	template <typename T>
	void internalPushBack(const T& v);

	static bool internalPrintAsYAML(
		const std::monostate&, std::ostream& o, int indent, bool first);
	static bool internalPrintAsYAML(
		const sequence_t& v, std::ostream& o, int indent, bool first);
	static bool internalPrintAsYAML(
		const map_t& v, std::ostream& o, int indent, bool first);
	static bool internalPrintAsYAML(
		const scalar_t& v, std::ostream& o, int indent, bool first);

	/** Impl of operator=() */
	template <typename T>
	void implOpAssign(const T& v)
	{
		if (isConstProxy_)
			throw std::logic_error("Trying to write into read-only proxy");
		if (!isProxy_)
			throw std::logic_error(
				"Trying to write into a Parameter block. Use "
				"`p[\"name\"]=value;` instead");
		if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
		valuenc_->emplace<T>(v);
	}
	/** @} */
};

/** Prints a scalar, a part of a Parameters tree, or the entire structure,
 * in YAML-like format */
inline std::ostream& operator<<(std::ostream& o, const Parameters& p)
{
	p.printAsYAML(o);
	return o;
}

/** Macro to load a variable from a mrpt::containers::Parameters (initials MCP)
 * dictionary, throwing an std::invalid_argument exception  if the value is not
 * found (REQuired).
 *
 * Usage:
 * \code
 * mrpt::containers::Parameters p;
 * double K;
 *
 * MCP_LOAD_REQ(p, K);
 * \endcode
 */
#define MCP_LOAD_REQ(paramsVariable__, keyName__)                         \
	if (!paramsVariable__.has(#keyName__))                                \
		throw std::invalid_argument(mrpt::format(                         \
			"Required parameter `%s` not an existing key in dictionary.", \
			#keyName__));                                                 \
	keyName__ = paramsVariable__[#keyName__].as<decltype(keyName__)>()

/** Macro to load a variable from a mrpt::containers::Parameters (initials MCP)
 * dictionary, leaving it with its former value if not found (OPTional).
 *
 * Usage:
 * \code
 * mrpt::containers::Parameters p;
 * double K;
 *
 * MCP_LOAD_OPT(p, K);
 * \endcode
 */
#define MCP_LOAD_OPT(paramsVariable__, keyName__) \
	keyName__ = paramsVariable__.getOrDefault(#keyName__, keyName__)

/** Just like MCP_LOAD_REQ(), but converts the read number from degrees to
 * radians */
#define MCP_LOAD_REQ_DEG(paramsVariable__, keyName__) \
	MCP_LOAD_REQ(paramsVariable__, keyName__);        \
	keyName__ = mrpt::DEG2RAD(keyName__)

/** Just like MCP_LOAD_OPT(), but converts the read number from degrees to
 * radians */
#define MCP_LOAD_OPT_DEG(paramsVariable__, keyName__) \
	keyName__ = mrpt::RAD2DEG(keyName__);             \
	MCP_LOAD_OPT(paramsVariable__, keyName__);        \
	keyName__ = mrpt::DEG2RAD(keyName__)

/** Macro to store a variable into a mrpt::containers::Parameters (initials MCP)
 * dictionary, using as "key" the name of the variable.
 *
 * Usage:
 * \code
 * mrpt::containers::Parameters p;
 * double K = ...;
 *
 * MCP_SAVE(p, K);
 *
 * // If you want "K" to have degree units in the parameter block, radians when
 * // loaded in memory:
 * MCP_SAVE_DEG(p,K);
 * \endcode
 */
#define MCP_SAVE(paramsVariable__, keyName__) \
	paramsVariable__[#keyName__].asRef<decltype(keyName__)>() = keyName__;

#define MCP_SAVE_DEG(paramsVariable__, keyName__) \
	paramsVariable__[#keyName__].asRef<double>() = mrpt::RAD2DEG(keyName__);

}  // namespace mrpt::containers

namespace mrpt::containers
{
template <typename T>
T& Parameters::asRef()
{
	if (isConstProxy_)
		throw std::logic_error("Trying to write into read-only proxy");
	if (!isProxy_)
		throw std::logic_error(
			"Trying to read from a non-scalar. Use `p[\"name\"].as<T>();` "
			"instead");

	if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
	try
	{
		std::any_cast<T>(*valuenc_);
	}
	catch (const std::bad_any_cast&)
	{
		valuenc_->emplace<T>();
	}
	return *std::any_cast<T>(valuenc_);
}

template <typename T>
const T& Parameters::asRef() const
{
	if (!isProxy_)
		throw std::logic_error(
			"Trying to read from a non-scalar. Use `p[\"name\"].as<T>();` "
			"instead");
	const auto& expectedType = typeid(T);
	const auto& storedType = isConstProxy_ ? value_->type() : valuenc_->type();

	if (storedType != expectedType)
		THROW_EXCEPTION_FMT(
			"Trying to read parameter `%s` of type `%s` as if it was "
			"`%s` and no obvious conversion found.",
			name_, mrpt::demangle(storedType.name()).c_str(),
			mrpt::demangle(expectedType.name()).c_str());

	if (isConstProxy_)
		return *std::any_cast<const T>(value_);
	else
		return *std::any_cast<const T>(valuenc_);
}

template <typename T>
void Parameters::internalPushBack(const T& v)
{
	Parameters* p = internalMeOrValue();
	if (!p->isSequence())
		throw std::logic_error(
			"push_back() only available for sequence nodes.");
	sequence_t& seq = std::get<sequence_t>(p->data_);
	seq.emplace_back(v);
}

}  // namespace mrpt::containers

namespace mrpt::containers::internal
{
template <typename T>
T implAsGetter(const Parameters& p)
{
	ASSERT_(p.isProxy_);
	const auto& expectedType = typeid(T);
	const auto& storedType =
		p.isConstProxy_ ? p.value_->type() : p.valuenc_->type();

	if (storedType != expectedType)
	{
		if constexpr (std::is_convertible_v<double, T>)
			if (storedType == typeid(double))
				return static_cast<T>(p.as<double>());
		if constexpr (std::is_convertible_v<T, double>)
		{
			std::stringstream ss;
			p.printAsYAML(ss);
			T ret;
			ss >> ret;
			if (!ss.fail()) return ret;
		}
		if (storedType == typeid(std::string))
		{
			std::stringstream ss(p.as<std::string>());
			T ret;
			ss >> ret;
			if (!ss.fail()) return ret;
		}
		if constexpr (std::is_convertible_v<std::string, T>)
		{
			if (expectedType == typeid(std::string))
			{
				std::stringstream ss;
				p.printAsYAML(ss);
				return ss.str();
			}
		}
		if constexpr (std::is_same_v<T, bool>)
		{
			if (storedType == typeid(std::string))
			{
				const auto s = p.as<std::string>();
				return s == "true" || s == "True" || s == "T" || s == "TRUE";
			}
		}

		THROW_EXCEPTION_FMT(
			"Trying to read parameter `%s` of type `%s` as if it was "
			"`%s` and no obvious conversion found.",
			p.name_, mrpt::demangle(storedType.name()).c_str(),
			mrpt::demangle(expectedType.name()).c_str());
	}

	if (p.isConstProxy_)
		return *std::any_cast<const T>(p.value_);
	else
		return *std::any_cast<const T>(p.valuenc_);
}

}  // namespace mrpt::containers::internal
