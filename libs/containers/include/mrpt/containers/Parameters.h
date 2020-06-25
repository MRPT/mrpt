/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/format.h>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

// forward declarations
// clang-format off
namespace YAML { class Node; }
namespace mrpt::containers { class Parameters; 
namespace internal { 
 struct tag_as_proxy_t {}; struct tag_as_const_proxy_t {};
 template <typename T> const T& implAsGetter(const Parameters& p, const char* expectedType);
 template <typename T> const T& asGetter(const Parameters& p);
 const char* typeIdxToStr(const std::size_t idx);
}
// clang-format on

/** Powerful YAML-like container for possibly-nested blocks of parameters.
 *
 * The class Parameters acts as a "node" in a YAML structure, and can be of one
 *of these types:
 * - Leaf nodes ("values"): of type `std::string`, `double`, `uint64_t`.
 * - Sequential container.
 * - Map ("dictionary"): pairs of `name: value`.
 *
 * Elements contained in either sequenties or dictionaries can be either plain
 *leaf types, or another container.
 *
 * This class was designed as a lightweight but structured way to pass
 *arbitrarialy-complex parameter blocks.
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
 * \note [new in MRPT 2.0.5]
 */
class Parameters
{
   public:
	using value_t =
		std::variant<std::monostate, double, uint64_t, std::string, Parameters>;
	using parameter_name_t = std::string;

	using sequence_t = std::vector<value_t>;
	using map_t = std::map<parameter_name_t, value_t>;

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

	void printAsYAML(std::ostream& o) const;
	void printAsYAML() const;  //!< prints to std::cout

	/** For map nodes, returns the type of the given child, or an empty
	 * string if it does not exist. Possible return values are: "uninitialized",
	 * "double", "uint64_t", "std::string", "Parameters", "undefined" (should
	 * never happen).
	 */
	std::string typeOfChild(const std::string& key) const;

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
			return std::get<T>(it->second);
		}
		catch (const std::bad_variant_access&)
		{
			throw std::logic_error(mrpt::format(
				"getOrDefault(): Trying to access key `%s` holding variant "
				"type `%s` as the wrong type.",
				key.c_str(), internal::typeIdxToStr(it->second.index())));
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

	void operator=(const double v);
	void operator=(const std::string& v);
	Parameters& operator=(const Parameters& v);

	inline operator double() const { return as<double>(); }
	inline operator const std::string&() const { return as<std::string>(); }

   private:
	const char* name_ = nullptr;
	const value_t* value_ = nullptr;
	value_t* valuenc_ = nullptr;

	/** Returns the pointer to the referenced object, if a proxy, or nullptr
	 * otherwise */
	const Parameters* internalValueAsSelf() const;
	Parameters* internalValueAsSelf();

	/** Returns me or the pointer to the referenced object, if a proxy */
	const Parameters* internalMeOrValue() const;
	Parameters* internalMeOrValue();

	template <typename T>
	friend const T& internal::implAsGetter(
		const Parameters& p, const char* expectedType);

	static void internalPrintAsYAML(
		const Parameters& p, std::ostream& o, int indent, bool first);

	template <typename T>
	void internalPushBack(const T& v)
	{
		Parameters* p = internalMeOrValue();
		if (!p->isSequence())
			throw std::logic_error(
				"push_back() only available for sequence nodes.");
		sequence_t& seq = std::get<sequence_t>(p->data_);
		seq.emplace_back(v);
	}

	static void internalPrintAsYAML(
		const std::monostate&, std::ostream& o, int indent, bool first);
	static void internalPrintAsYAML(
		const double& v, std::ostream& o, int indent, bool first);
	static void internalPrintAsYAML(
		const std::string& v, std::ostream& o, int indent, bool first);
	static void internalPrintAsYAML(
		const uint64_t& v, std::ostream& o, int indent, bool first);
	static void internalPrintAsYAML(
		const sequence_t& v, std::ostream& o, int indent, bool first);
	static void internalPrintAsYAML(
		const map_t& v, std::ostream& o, int indent, bool first);
	static void internalPrintAsYAML(
		const value_t& v, std::ostream& o, int indent, bool first);

	/** @} */
};

}  // namespace mrpt::containers
