/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/ValueCommentPair.h>
#include <mrpt/core/bits_math.h>  // mrpt::RAD2DEG
#include <mrpt/core/demangle.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>

#include <any>
#include <array>
#include <cstdint>
#include <iosfwd>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <variant>
#include <vector>

// forward declarations
// clang-format off
// For auxiliary proxies:
namespace mrpt::containers { class yaml;
namespace internal {
 enum tag_as_proxy_t {}; enum tag_as_const_proxy_t {};
 template <typename T> T implAsGetter(const yaml& p);
 template <typename T> T implAnyAsGetter(const std::any& p);
}
// clang-format on

/** \defgroup mrpt_containers_yaml YAML/JSON C++ API
 * Header: `#include <mrpt/containers/yaml.h>`.
 * Library: \ref mrpt_containers_grp
 * \ingroup mrpt_containers_grp */

/** Powerful YAML-like container for possibly-nested blocks of parameters or
 *any arbitrary structured data contents, including documentation in the
 *form of comments attached to each node. Supports parsing from YAML or JSON
 *streams or text blocks.
 *
 * This class holds the root "node" in a YAML-like tree structure.
 * Each tree node can be of one of these types:
 * - Scalar values ("leaf nodes"): Can hold any type, stored as C++17 std::any.
 * - Sequence container.
 * - Map ("dictionary"): pairs of `name: value`.
 * - Null, empty nodes: yaml `~` or `null`.
 *
 * Sequences and dictionaries can hold, in turn, any of the four types above,
 * leading to arbitrarialy-complex nested structures.
 *
 * This class was designed as a lightweight, while structured, way to pass
 *arbitrarialy-complex parameter blocks but can be used to load and save
 *YAML files or as a database.
 *
 * `yaml` can be used to parse and emit YAML or JSON streams.
 * It does not support event-based parsing.
 * Parsing YAML 1.2 is supported thanks to
 *[libfyaml](https://github.com/pantoniou/libfyaml), which
 *[passes](http://matrix.yaml.io/) the full [YAML
 *testsuite](https://github.com/yaml/yaml-test-suite).
 *
 * See example in \ref containers_yaml_example/test.cpp
 * \snippet containers_yaml_example/test.cpp example-yaml
 * Output:
 *  \include containers_yaml_example/console.out
 *
 * \ingroup mrpt_containers_yaml
 * \note [New in MRPT 2.1.0]
 */
class yaml
{
   public:
	/** @name Types
	 * @{ */

	using scalar_t = std::any;
	using parameter_proxiedMapEntryName_t = std::string;

	struct node_t;

	using sequence_t = std::vector<node_t>;
	using map_t = std::map<parameter_proxiedMapEntryName_t, node_t>;

	struct node_t
	{
		/** Node data */
		std::variant<std::monostate, sequence_t, map_t, scalar_t> d;

		/** Optional comment block */
		std::array<
			std::optional<std::string>,
			static_cast<size_t>(CommentPosition::MAX)>
			comments;

		node_t() = default;
		~node_t() = default;

		template <
			typename T,  //
			typename = std::enable_if_t<!std::is_constructible_v<
				std::initializer_list<map_t::value_type>, T>>,  //
			typename = std::enable_if_t<!std::is_constructible_v<
				std::initializer_list<sequence_t::value_type>, T>>>
		node_t(const T& scalar)
		{
			d.emplace<scalar_t>().emplace<T>(scalar);
		}
		/** Specialization for literals */
		node_t(const char* str)
		{
			d.emplace<scalar_t>().emplace<const char*>(str);
		}

		node_t(std::initializer_list<map_t::value_type> init)
		{
			d.emplace<map_t>(init);
		}
		node_t(std::initializer_list<sequence_t::value_type> init)
		{
			d.emplace<sequence_t>(init);
		}

		bool isNullNode() const;
		bool isScalar() const;
		bool isSequence() const;
		bool isMap() const;

		/** Returns: "null", "sequence", "map", "scalar(<TYPE>)" */
		std::string typeName() const;

		/** Use: `for (auto &kv: n.asSequence()) {...}`
		 * \exception std::exception If called on a non-sequence node. */
		sequence_t& asSequence();
		const sequence_t& asSequence() const;

		/** Use: `for (auto &kv: n.asMap()) {...}`
		 * \exception std::exception If called on a non-map node. */
		map_t& asMap();
		const map_t& asMap() const;

		/** \exception std::exception If called on a non-scalar node. */
		scalar_t& asScalar();
		const scalar_t& asScalar() const;

		/** Returns 1 for null or scalar nodes, the number of children for
		 * sequence or map nodes. */
		size_t size() const;

		/** Returns a copy of the existing value of the given type, or tries to
		 * convert it between easily-compatible types (e.g. double<->int,
		 * string<->int).
		 * \exception std::exception If the contained type does not  match and
		 * there is no obvious conversion.
		 */
		template <typename T>
		T as() const
		{
			ASSERTMSG_(
				std::holds_alternative<scalar_t>(d),
				mrpt::format(
					"Trying to use as() on a node of type `%s`, but only "
					"available for `scalar` nodes.",
					typeName().c_str()));
			return internal::implAnyAsGetter<T>(std::get<scalar_t>(d));
		}
	};

	/** @} */

	/** @name Constructors and initializers
	 * @{ */

	yaml() = default;
	~yaml() = default;

	/** Constructor for maps, from list of pairs of values. See examples in
	 * yaml above. */
	yaml(std::initializer_list<map_t::value_type> init) : root_(init) {}

	/** Constructor for sequences, from list of values. See examples in
	 * yaml above. */
	yaml(std::initializer_list<sequence_t::value_type> init) : root_(init) {}
	yaml(const yaml& v);

	yaml(const node_t& s) : root_(s) {}

	static node_t Sequence(std::initializer_list<sequence_t::value_type> init)
	{
		return node_t(init);
	}
	static node_t Sequence()
	{
		node_t n;
		n.d.emplace<sequence_t>();
		return n;
	}

	static node_t Map(std::initializer_list<map_t::value_type> init)
	{
		return node_t(init);
	}
	static node_t Map()
	{
		node_t n;
		n.d.emplace<map_t>();
		return n;
	}

	/** Parses a text as YAML or JSON (autodetected) and returns a document.
	 * \exception std::exception Upon format errors
	 */
	static yaml FromText(const std::string& yamlTextBlock);

	/** Parses a text as YAML or JSON (autodetected) and stores the contents
	 * into this document.
	 *
	 * \exception std::exception Upon format errors
	 */
	void loadFromText(const std::string& yamlTextBlock);

	/** Parses the stream as YAML or JSON (autodetected) and returns a document.
	 * \exception std::exception Upon format errors
	 */
	static yaml FromStream(std::istream& i);

	/** Parses a text as YAML or JSON (autodetected) and stores the contents
	 * into this document.
	 *
	 * \exception std::exception Upon I/O or format errors
	 */
	void loadFromFile(const std::string& fileName);

	/** Parses the filename as YAML or JSON (autodetected) and returns a
	 * document.
	 * \exception std::exception Upon I/O or format errors.
	 */
	static yaml FromFile(const std::string& fileName);

	/** Parses the stream as YAML or JSON (autodetected) and stores the contents
	 * into this document.
	 *
	 * \exception std::exception Upon format errors
	 */
	void loadFromStream(std::istream& i);

	/** Builds an object copying the structure and contents from an existing
	 * YAMLCPP Node. Requires user to #include yamlcpp from your calling program
	 * (does NOT requires yamlcpp while compiling mrpt itself).
	 *
	 * \tparam YAML_NODE Must be `YAML::Node`. Made a template just to avoid
	 * build-time depedencies.
	 */
	template <typename YAML_NODE>
	inline static yaml FromYAMLCPP(const YAML_NODE& n);

	/** \overload (loads an existing YAMLCPP into this) */
	template <typename YAML_NODE>
	inline void loadFromYAMLCPP(const YAML_NODE& n);

	/** @} */

	/** @name Content and type checkers
	 * @{ */
	/** For map nodes, checks if the given key name exists */
	bool has(const std::string& key) const;

	/** For map or sequence nodes, checks if the container is empty. Also
	 * returns true for null(empty) nodes. */
	bool empty() const;

	/** Resets to empty (can be called on a root node or any other node to clear
	 * that subtree only). */
	void clear();

	bool isNullNode() const;
	bool isScalar() const;
	bool isSequence() const;
	bool isMap() const;

	/** For scalar nodes, returns its type, or typeid(void) if an empty node.
	 * \exception std::exception If called on a map or sequence. */
	const std::type_info& scalarType() const;

	/** @} */

	/** @name Range-for and conversion helpers
	 * @{ */

	/** Use: `for (auto &kv: n.asSequence()) {...}`
	 * \exception std::exception If called on a non-sequence node. */
	sequence_t& asSequence();
	const sequence_t& asSequence() const;

	/** Use: `for (auto &kv: n.asMap()) {...}`
	 * \exception std::exception If called on a non-map node. */
	map_t& asMap();
	const map_t& asMap() const;

	/** \exception std::exception If called on a non-scalar node. */
	scalar_t& asScalar();
	const scalar_t& asScalar() const;

	/** Returns 1 for null or scalar nodes, the number of children for sequence
	 * or map nodes. */
	size_t size() const;

	/** For a master yaml document, returns the root node; otherwise, the
	 * referenced node. */
	node_t& node() { return *dereferenceProxy(); }
	/** \overload */
	const node_t& node() const { return *dereferenceProxy(); }

	/** @} */

	/** @name Print and export
	 * @{ */
	void printAsYAML(std::ostream& o, bool debugInfo = false) const;
	void printAsYAML() const;  //!< prints to std::cout
	/** @} */

	/** @name Read/write to maps (dictionaries)
	 * @{ */

	/** Write access for maps */
	yaml operator[](const char* key);
	/// \overload
	inline yaml operator[](const std::string& key)
	{
		return operator[](key.c_str());
	}

	/** Read access  for maps
	 * \throw std::runtime_error if key does not exist. */
	const yaml operator[](const char* key) const;
	/// \overload
	inline const yaml operator[](const std::string& key) const
	{
		return operator[](key.c_str());
	}
	/** Scalar read access for maps, with default value if key does not exist.
	 */
	template <typename T>
	const T getOrDefault(const std::string& key, const T& defaultValue) const
	{
		MRPT_START
		const node_t* n = dereferenceProxy();
		if (n->isNullNode()) return defaultValue;
		if (!n->isMap())
			THROW_EXCEPTION_FMT(
				"getOrDefault() is only for map nodes, invoked on a node of "
				"type: '%s'",
				n->typeName().c_str());

		const map_t& m = std::get<map_t>(n->d);
		auto it = m.find(key);
		if (m.end() == it) return defaultValue;
		try
		{
			return yaml(internal::tag_as_const_proxy_t(), it->second, "")
				.as<T>();
		}
		catch (const std::bad_any_cast& e)
		{
			throw std::logic_error(mrpt::format(
				"getOrDefault(): Trying to access key `%s` holding type `%s` "
				"as the wrong type: `%s`",
				key.c_str(), n->typeName().c_str(), e.what()));
		}
		MRPT_END
	}
	/** @} */

	/** Write into an existing index of a sequence.
	 * \throw std::out_of_range if index is out of range. */
	yaml operator()(int index);
	/** Read from an existing index of a sequence.
	 * \throw std::out_of_range if index is out of range. */
	const yaml operator()(int index) const;

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
	void push_back(const yaml& v)
	{
		sequence_t& seq = asSequence();
		seq.emplace_back(v.root_);
	}

   private:
	node_t root_;
	bool isProxy_ = false;
	bool isConstProxy_ = false;

	// Proxy members:
	const char* proxiedMapEntryName_ = nullptr;
	const node_t* proxiedNode_ = nullptr;

	/** @name Internal proxy
	 * @{ */

	/** Returns the pointer to the referenced node data, if a proxy, or to the
	 * root node otherwise. Will never return nullptr. */
	const node_t* dereferenceProxy() const;
	node_t* dereferenceProxy();

	explicit yaml(internal::tag_as_proxy_t, node_t& val, const char* name)
		: isProxy_(true),
		  isConstProxy_(false),
		  proxiedMapEntryName_(name),
		  proxiedNode_(&val)
	{
	}
	explicit yaml(
		internal::tag_as_const_proxy_t, const node_t& val, const char* name)
		: isProxy_(true),
		  isConstProxy_(true),
		  proxiedMapEntryName_(name),
		  proxiedNode_(&val)
	{
	}
	/** @} */

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

	yaml& operator=(bool v);

	yaml& operator=(float v);
	yaml& operator=(double v);

	yaml& operator=(int8_t v);
	yaml& operator=(uint8_t v);
	yaml& operator=(int16_t v);
	yaml& operator=(uint16_t v);
	yaml& operator=(int32_t v);
	yaml& operator=(uint32_t v);
	yaml& operator=(int64_t v);
	yaml& operator=(uint64_t v);

	yaml& operator=(const std::string& v);
	inline yaml& operator=(const char* v) { return operator=(std::string(v)); }
	inline yaml& operator=(const std::string_view& v)
	{
		return operator=(std::string(v));
	}
	yaml& operator=(const yaml& v);

	/** vcp (value-comment) wrapper */
	template <typename T>
	yaml& operator=(const ValueCommentPair<T>& vc)
	{
		this->comment(vc.comment, vc.position);
		return operator=(vc.value);
	}

	inline operator bool() const { return as<bool>(); }

	inline operator double() const { return as<double>(); }
	inline operator float() const { return as<float>(); }

	inline operator int8_t() const { return as<int8_t>(); }
	inline operator uint8_t() const { return as<uint8_t>(); }
	inline operator int16_t() const { return as<int16_t>(); }
	inline operator uint16_t() const { return as<uint16_t>(); }
	inline operator int32_t() const { return as<int32_t>(); }
	inline operator uint32_t() const { return as<uint32_t>(); }
	inline operator int64_t() const { return as<int64_t>(); }
	inline operator uint64_t() const { return as<uint64_t>(); }

	inline operator std::string() const { return as<std::string>(); }

	/** Returns true if the proxied node has an associated comment block, at any
	 * location */
	bool hasComment() const;

	/** Returns true if the proxied node has an associated comment block at a
	 * particular position */
	bool hasComment(CommentPosition pos) const;

	/** Gets the comment associated to the proxied node. This version returns
	 * the first comment, of all possible (top, right,...).
	 *
	 * \exception std::exception If there is no comment attached.
	 * \sa hasComment()
	 */
	const std::string& comment() const;

	/** Gets the comment associated to the proxied node, at the particular
	 * position.
	 *
	 * \exception std::exception If there is no comment attached.
	 * \sa hasComment()
	 */
	const std::string& comment(CommentPosition pos) const;

	/** Sets the comment attached to a given proxied node.
	 * \exception std::exception If there is no comment attached.
	 * \sa hasComment()
	 */
	void comment(
		const std::string_view& c,
		CommentPosition position = CommentPosition::TOP);

   private:
	template <typename T>
	friend T internal::implAsGetter(const yaml& p);
	template <typename T>
	friend T internal::implAnyAsGetter(const scalar_t& p);

	// Return: true if the last printed char is a newline char
	static bool internalPrintNodeAsYAML(
		const node_t& p, std::ostream& o, int indent, bool first,
		bool debugInfo, bool needsSpace);

	template <typename T>
	void internalPushBack(const T& v);

	static bool internalPrintAsYAML(
		const std::monostate&, std::ostream& o, int indent, bool first,
		bool debugInfo, const std::optional<std::string>& rightComment,
		bool needsSpace);
	static bool internalPrintAsYAML(
		const sequence_t& v, std::ostream& o, int indent, bool first,
		bool debugInfo, const std::optional<std::string>& rightComment,
		bool needsSpace);
	static bool internalPrintAsYAML(
		const map_t& v, std::ostream& o, int indent, bool first, bool debugInfo,
		const std::optional<std::string>& rightComment, bool needsSpace);
	static bool internalPrintAsYAML(
		const scalar_t& v, std::ostream& o, int indent, bool first,
		bool debugInfo, const std::optional<std::string>& rightComment,
		bool needsSpace);

	/** Impl of operator=() */
	template <typename T>
	yaml& implOpAssign(const T& v)
	{
		ASSERTMSG_(
			isProxy_,
			"Trying to write into a non-leaf node, `p[\"name\"]=value;` "
			"instead");
		ASSERTMSG_(!isConstProxy_, "Trying to write into read-only proxy");
		ASSERT_(proxiedNode_ != nullptr);

		scalar_t& s = const_cast<node_t*>(proxiedNode_)->d.emplace<scalar_t>();
		s.emplace<T>(v);
		return *this;
	}
};

/** Prints a scalar, a part of a yaml tree, or the entire structure,
 * in YAML-like format */
std::ostream& operator<<(std::ostream& o, const yaml& p);

/** Macro to load a variable from a mrpt::containers::yaml (initials MCP)
 * dictionary, throwing an std::invalid_argument exception  if the value is not
 * found (REQuired).
 *
 * Usage:
 * \code
 * mrpt::containers::yaml p;
 * double K;
 *
 * MCP_LOAD_REQ(p, K);
 * \endcode
 */
#define MCP_LOAD_REQ(paramsVariable__, keyproxiedMapEntryName__)           \
	if (!paramsVariable__.has(#keyproxiedMapEntryName__))                  \
		throw std::invalid_argument(mrpt::format(                          \
			"Required parameter `%s` not an existing key in dictionary.",  \
			#keyproxiedMapEntryName__));                                   \
	keyproxiedMapEntryName__ = paramsVariable__[#keyproxiedMapEntryName__] \
								   .as<decltype(keyproxiedMapEntryName__)>()

/** Macro to load a variable from a mrpt::containers::yaml (initials MCP)
 * dictionary, leaving it with its former value if not found (OPTional).
 *
 * Usage:
 * \code
 * mrpt::containers::yaml p;
 * double K;
 *
 * MCP_LOAD_OPT(p, K);
 * \endcode
 */
#define MCP_LOAD_OPT(paramsVariable__, keyproxiedMapEntryName__) \
	keyproxiedMapEntryName__ = paramsVariable__.getOrDefault(    \
		#keyproxiedMapEntryName__, keyproxiedMapEntryName__)

/** Just like MCP_LOAD_REQ(), but converts the read number from degrees to
 * radians */
#define MCP_LOAD_REQ_DEG(paramsVariable__, keyproxiedMapEntryName__) \
	MCP_LOAD_REQ(paramsVariable__, keyproxiedMapEntryName__);        \
	keyproxiedMapEntryName__ = mrpt::DEG2RAD(keyproxiedMapEntryName__)

/** Just like MCP_LOAD_OPT(), but converts the read number from degrees to
 * radians */
#define MCP_LOAD_OPT_DEG(paramsVariable__, keyproxiedMapEntryName__)    \
	keyproxiedMapEntryName__ = mrpt::RAD2DEG(keyproxiedMapEntryName__); \
	MCP_LOAD_OPT(paramsVariable__, keyproxiedMapEntryName__);           \
	keyproxiedMapEntryName__ = mrpt::DEG2RAD(keyproxiedMapEntryName__)

/** Macro to store a variable into a mrpt::containers::yaml (initials MCP)
 * dictionary, using as "key" the name of the variable.
 *
 * Usage:
 * \code
 * mrpt::containers::yaml p;
 * double K = ...;
 *
 * MCP_SAVE(p, K);
 *
 * // If you want "K" to have degree units in the parameter block, radians when
 * // loaded in memory:
 * MCP_SAVE_DEG(p,K);
 * \endcode
 */
#define MCP_SAVE(paramsVariable__, keyproxiedMapEntryName__) \
	paramsVariable__[#keyproxiedMapEntryName__] = keyproxiedMapEntryName__;

#define MCP_SAVE_DEG(paramsVariable__, keyproxiedMapEntryName__) \
	paramsVariable__[#keyproxiedMapEntryName__] =                \
		mrpt::RAD2DEG(keyproxiedMapEntryName__);

}  // namespace mrpt::containers

namespace mrpt::containers
{
template <typename T>
T& yaml::asRef()
{
	ASSERTMSG_(
		isProxy_,
		"Trying to read from a non-scalar. Use `p[\"name\"].asRef<T>();` "
		"instead");
	ASSERTMSG_(!isConstProxy_, "Trying to write into read-only proxy");
	scalar_t& s = this->asScalar();

	try
	{
		std::any_cast<T>(s);
	}
	catch (const std::bad_any_cast&)
	{
		s.emplace<T>();
	}
	return *std::any_cast<T>(&s);
}

template <typename T>
const T& yaml::asRef() const
{
	ASSERTMSG_(
		isProxy_,
		"Trying to read from a non-scalar. Use `p[\"name\"].asRef<T>();` "
		"instead");
	const auto& expectedType = typeid(T);
	const scalar_t& s = this->asScalar();
	const auto& storedType = s.type();

	if (storedType != expectedType)
		THROW_EXCEPTION_FMT(
			"Trying to read parameter `%s` of type `%s` as if it was "
			"`%s` and no obvious conversion found.",
			proxiedMapEntryName_, mrpt::demangle(storedType.name()).c_str(),
			mrpt::demangle(expectedType.name()).c_str());

	return *std::any_cast<T>(&s);
}

template <typename T>
void yaml::internalPushBack(const T& v)
{
	ASSERT_(this->isSequence());
	sequence_t& seq = asSequence();
	seq.emplace_back().d.emplace<scalar_t>().emplace<T>(v);
}

template <typename YAML_NODE>
inline yaml yaml::FromYAMLCPP(const YAML_NODE& n)
{
	const auto invalidDbl = std::numeric_limits<double>::max();

	if (n.IsSequence())
	{
		yaml ret = yaml(Sequence());

		for (const auto& e : n)
		{
			if (e.IsNull())
			{
				sequence_t& seq =
					std::get<sequence_t>(ret.dereferenceProxy()->d);
				seq.push_back(node_t());
			}
			else if (e.IsScalar())
			{
				if (double v = e.template as<double>(invalidDbl);
					v != invalidDbl)
					ret.push_back(v);
				else
					ret.push_back(e.template as<std::string>());
			}
			else
			{
				// Recursive:
				ret.push_back(yaml::FromYAMLCPP(YAML_NODE(e)));
			}
		}
		return ret;
	}
	else if (n.IsMap())
	{
		yaml ret = yaml(yaml::Map());

		for (const auto& kv : n)
		{
			const auto& key = kv.first.template as<std::string>();
			const auto& val = kv.second;

			if (val.IsNull())
			{
				map_t& m = std::get<map_t>(ret.dereferenceProxy()->d);
				m[key];
			}
			else if (val.IsScalar())
			{
				if (double v = val.template as<double>(invalidDbl);
					v != invalidDbl)
					ret[key] = v;
				else
					ret[key] = val.template as<std::string>();
			}
			else
			{
				// Recursive:
				ret[key] = yaml::FromYAMLCPP(YAML_NODE(val));
			}
		}
		return ret;
	}
	else
	{
		THROW_EXCEPTION(
			"FromYAMLCPP only supports root YAML as sequence "
			"or map");
	}
}

template <typename YAML_NODE>
inline void yaml::loadFromYAMLCPP(const YAML_NODE& n)
{
	*this = yaml::FromYAMLCPP(n);
}

}  // namespace mrpt::containers

namespace mrpt::containers::internal
{
template <typename T>
T implAnyAsGetter(const mrpt::containers::yaml::scalar_t& s)
{
	const auto& expectedType = typeid(T);
	const auto& storedType = s.type();

	if (storedType == expectedType) return std::any_cast<const T&>(s);

	if constexpr (std::is_convertible_v<int, T>)
	{
		if (storedType == typeid(std::string))
		{
			const std::string str = implAnyAsGetter<std::string>(s);
			// Recognize hex or octal prefixes:
			try
			{
				std::size_t processed = 0;
				T ret = static_cast<T>(
					std::stol(str, &processed, 0 /*auto detect base*/));
				if (processed > 0) return ret;
			}
			catch (...)
			{ /*Invalid number*/
			}
		}
	}
	if constexpr (std::is_convertible_v<double, T>)
	{
		if (storedType == typeid(double))
			return static_cast<T>(implAnyAsGetter<double>(s));
	}
	if constexpr (std::is_convertible_v<T, double>)
	{
		std::stringstream ss;
		yaml::internalPrintAsYAML(s, ss, 0, true, false, {}, false);
		T ret;
		ss >> ret;
		if (!ss.fail()) return ret;
	}
	if constexpr (std::is_convertible_v<std::string, T>)
	{
		if (expectedType == typeid(std::string))
		{
			std::stringstream ss;
			yaml::internalPrintAsYAML(s, ss, 0, true, false, {}, false);
			return ss.str();
		}
	}
	if constexpr (std::is_same_v<T, bool>)
	{
		if (storedType == typeid(std::string))
		{
			const auto str = implAnyAsGetter<std::string>(s);
			return str == "true" || str == "True" || str == "T" ||
				   str == "TRUE";
		}
	}

	THROW_EXCEPTION_FMT(
		"Trying to access scalar of type `%s` as if it was "
		"`%s` and no obvious conversion found.",
		mrpt::demangle(storedType.name()).c_str(),
		mrpt::demangle(expectedType.name()).c_str());
}

template <typename T>
T implAsGetter(const yaml& p)
{
	MRPT_START
	ASSERTMSG_(
		p.isScalar(),
		mrpt::format(
			"Trying to read from a non-scalar. Actual node type: `%s`",
			p.node().typeName().c_str()));
	const yaml::scalar_t& s = p.asScalar();
	return implAnyAsGetter<T>(s);
	MRPT_END
}

}  // namespace mrpt::containers::internal
