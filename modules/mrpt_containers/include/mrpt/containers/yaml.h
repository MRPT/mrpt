/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/containers/ValueCommentPair.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/internal_yaml_fwrds.h>
#include <mrpt/core/bits_math.h>  // mrpt::RAD2DEG
#include <mrpt/core/demangle.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/typemeta/TEnumType.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <variant>
#include <vector>

/** \defgroup mrpt_containers_yaml YAML/JSON C++ API
 * Header: `#include <mrpt/containers/yaml.h>`.
 * Library: \ref mrpt_containers_grp
 * \ingroup mrpt_containers_grp */

namespace mrpt::containers
{
// Forward declarations for the proxy types:
class yaml_ref;
class yaml_cref;

/** Powerful YAML-like container for possibly-nested blocks of parameters or
 *any arbitrary structured data contents, including documentation in the
 *form of comments attached to each node. Supports parsing from YAML or JSON
 *streams, files, or text strings.
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
 * yaml can be used to parse YAML (v1.2) or JSON streams, and to emit YAML.
 * It does not support event-based parsing.
 * The parser uses Pantelis Antoniou's awesome
 *[libfyaml](https://github.com/pantoniou/libfyaml), which
 *[passes](http://matrix.yaml.io/) the full [YAML
 *testsuite](https://github.com/yaml/yaml-test-suite).
 *
 * Known limitations:
 * - *Parsing* comments is limited to right-hand comments for *sequence* or
 *   *map* entries.
 *
 * See examples below (\ref containers_yaml_example/main.cpp):
 * \snippet containers_yaml_example/main.cpp example-yaml
 * Output:
 *  \include containers_yaml_example/console.out
 *
 * Verbose debug information on YAML document parsing is emitted if the
 * environment variable `MRPT_YAML_PARSER_VERBOSE` is set to `1`.
 *
 * \ingroup mrpt_containers_yaml
 * \note [New in MRPT 2.1.0]
 * \note **Thread safety:** This class is not thread-safe. Concurrent reads on a
 *   `yaml` object that is not being mutated are safe; any concurrent access
 *   where at least one thread writes is undefined behavior. Synchronization is
 *   the caller's responsibility, same policy as `std::map` and `std::vector`.
 */
class yaml
{
 public:
  /** @name Types
   * @{ */

  struct node_t;

  /** Discriminated scalar union. Types are normalized on assignment:
   * - All signed integer widths → `int64_t`
   * - All unsigned integer widths → `uint64_t`
   * - `float` → `double`
   * - `const char*` → `std::string`
   * - Nested yaml documents → `std::shared_ptr<yaml>` (avoids recursive type)
   * - `std::monostate` represents a null scalar */
  using scalar_t = std::
      variant<std::monostate, bool, int64_t, uint64_t, double, std::string, std::shared_ptr<yaml>>;

  using sequence_t = std::vector<node_t>;
  /** Insertion-order map: vector of (key-node, value-node) pairs.
   * Keys are node_t scalars so they carry parse-time comments; lookup is
   * O(n) linear search — fast for typical config-file maps (< ~100 keys). */
  using map_t = std::vector<std::pair<node_t, node_t>>;

  /** Per-node comments (top, right, bottom). Stored behind a unique_ptr so
   * comment-free nodes (the common case) pay only 8 bytes. */
  struct NodeMeta
  {
    std::array<std::optional<std::string>, static_cast<size_t>(CommentPosition::MAX)> comments;
  };

  /** Legacy alias kept for backward source compatibility. */
  using comments_t =
      std::array<std::optional<std::string>, static_cast<size_t>(CommentPosition::MAX)>;

  struct mark_t
  {
    /// Position from the start of the input file
    std::size_t input_pos = 0;
    int line = 0;    //!< Line position (0-based index)
    int column = 0;  //!< Column  position (0-based index)
  };

  struct node_t
  {
    /** @name Data
     *  @{ */

    /** Node data */
    std::variant<std::monostate, sequence_t, map_t, scalar_t> d;

    /** Optional comment block. Null when no comments are attached (common case),
     * saving ~96 bytes per comment-free node. */
    std::unique_ptr<NodeMeta> meta;

    /** Positioning information about the placement of the element in the
     * original input file/stream, i.e. line and column number.
     * Kept inline (not in NodeMeta) so parse-error messages are always available.
     * \note (New in MRPT 2.5.0)
     */
    mark_t marks;

    /** Optional flag to print collections in short form (e.g. [A,B] for
     * sequences) \note (New in MRPT 2.1.8) */
    bool printInShortFormat = false;

    /** @} */

    // unique_ptr<NodeMeta> requires explicit copy operations:
    node_t() = default;
    node_t(node_t&&) = default;
    node_t& operator=(node_t&&) = default;
    node_t(const node_t& o) : d(o.d), marks(o.marks), printInShortFormat(o.printInShortFormat)
    {
      if (o.meta) meta = std::make_unique<NodeMeta>(*o.meta);
    }
    node_t& operator=(const node_t& o)
    {
      if (this != &o)
      {
        d = o.d;
        marks = o.marks;
        printInShortFormat = o.printInShortFormat;
        meta = o.meta ? std::make_unique<NodeMeta>(*o.meta) : nullptr;
      }
      return *this;
    }

    template <
        typename T,  //
        typename = std::enable_if_t<
            !std::is_constructible_v<std::initializer_list<map_t::value_type>, T>>,  //
        typename = std::enable_if_t<
            !std::is_constructible_v<std::initializer_list<sequence_t::value_type>, T>>>
    node_t(const T& scalar)
    {
      if constexpr (std::is_same_v<T, bool>)
        d = scalar_t(scalar);
      else if constexpr (std::is_integral_v<T> && std::is_signed_v<T>)
        d = scalar_t(static_cast<int64_t>(scalar));
      else if constexpr (std::is_integral_v<T> && std::is_unsigned_v<T>)
        d = scalar_t(static_cast<uint64_t>(scalar));
      else if constexpr (std::is_floating_point_v<T>)
        d = scalar_t(static_cast<double>(scalar));
      else
        d = scalar_t(scalar);  // std::string or shared_ptr<yaml>
    }
    /** Specialization for string literals */
    node_t(const char* str) { d = scalar_t(std::string(str)); }

    node_t(std::initializer_list<map_t::value_type> init) { d.emplace<map_t>(init); }
    node_t(std::initializer_list<sequence_t::value_type> init) { d.emplace<sequence_t>(init); }

    [[nodiscard]] bool isNullNode() const;
    [[nodiscard]] bool isScalar() const;
    [[nodiscard]] bool isSequence() const;
    [[nodiscard]] bool isMap() const;

    /** Returns: "null", "sequence", "map", "scalar(<TYPE>)" */
    [[nodiscard]] std::string typeName() const;

    /** Use: `for (auto &kv: n.asSequence()) {...}`
     * \exception std::exception If called on a non-sequence node. */
    [[nodiscard]] sequence_t& asSequence();
    [[nodiscard]] const sequence_t& asSequence() const;

    /** Use: `for (auto &kv: n.asMap()) {...}`
     * \exception std::exception If called on a non-map node. */
    [[nodiscard]] map_t& asMap();
    [[nodiscard]] const map_t& asMap() const;

    /** \exception std::exception If called on a non-scalar node. */
    [[nodiscard]] scalar_t& asScalar();
    [[nodiscard]] const scalar_t& asScalar() const;

    /** Returns 1 for null or scalar nodes, the number of children for
     * sequence or map nodes. */
    [[nodiscard]] size_t size() const;

    /** Returns a copy of the existing value of the given type, or tries
     * to convert it between easily-compatible types (e.g. double<->int,
     * string<->int).
     * \exception std::exception If the contained type does not  match
     * and there is no obvious conversion.
     */
    template <typename T>
    [[nodiscard]] T as() const;  // defined after implAnyAsGetter is declared

    [[nodiscard]] std::string_view internalAsStr() const
    {
      ASSERT_(isScalar());
      const auto& s = std::get<scalar_t>(d);
      if (const auto* str = std::get_if<std::string>(&s); str != nullptr) return {*str};
      THROW_EXCEPTION_FMT(
          "Used node_t as map key with a non-string scalar: '%s'", typeName().c_str());
    }

    [[nodiscard]] bool hasComment() const { return meta != nullptr; }
    [[nodiscard]] bool hasComment(CommentPosition pos) const
    {
      MRPT_START
      const auto posIndex = static_cast<unsigned int>(pos);
      ASSERT_LT_(posIndex, static_cast<unsigned int>(CommentPosition::MAX));
      return meta != nullptr && meta->comments[posIndex].has_value();
      MRPT_END
    }
    [[nodiscard]] const std::string& comment() const
    {
      MRPT_START
      if (meta)
      {
        for (const auto& c : meta->comments)
        {
          if (c.has_value()) return c.value();
        }
      }
      THROW_EXCEPTION("Trying to access comment but this node has none.");
      MRPT_END
    }
    [[nodiscard]] const std::string& comment(CommentPosition pos) const
    {
      MRPT_START
      const auto posIndex = static_cast<unsigned int>(pos);
      ASSERT_LT_(posIndex, static_cast<unsigned int>(CommentPosition::MAX));
      ASSERTMSG_(
          meta != nullptr && meta->comments[posIndex].has_value(),
          "Trying to access comment but this node has none.");
      return meta->comments[posIndex].value();
      MRPT_END
    }

    /** Ensure meta is allocated and return a mutable reference to the comment slot. */
    std::optional<std::string>& commentSlot(CommentPosition pos)
    {
      if (!meta) meta = std::make_unique<NodeMeta>();
      return meta->comments[static_cast<size_t>(pos)];
    }
  };

  /** @} */

  /** @name Constructors and initializers
   * @{ */

  yaml() = default;

  /** Constructor for maps, from list of pairs of values. See examples in
   * yaml above. */
  yaml(std::initializer_list<map_t::value_type> init) : root_(init) {}

  /** Constructor for sequences, from list of values. See examples in
   * yaml above. */
  yaml(std::initializer_list<sequence_t::value_type> init) : root_(init) {}
  yaml(const yaml& v);

  yaml(node_t s) : root_(std::move(s)) {}

  static node_t Sequence(std::initializer_list<sequence_t::value_type> init)
  {
    return node_t(init);  // NOLINT
  }
  static node_t Sequence()
  {
    node_t n;
    n.d.emplace<sequence_t>();
    return n;
  }

  static node_t Map(std::initializer_list<map_t::value_type> init)
  {
    return node_t(init);  // NOLINT
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

  /** Parses the stream as YAML or JSON (autodetected) and returns a
   * document. \exception std::exception Upon format errors
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

  /** Parses the stream as YAML or JSON (autodetected) and stores the
   * contents into this document.
   *
   * \exception std::exception Upon format errors
   */
  void loadFromStream(std::istream& i);

  /** Builds an object copying the structure and contents from an existing
   * YAMLCPP Node. Requires user to #include yamlcpp from your calling
   * program (does NOT requires yamlcpp while compiling mrpt itself).
   *
   * \tparam YAML_NODE Must be `YAML::Node`. Made a template just to avoid
   * build-time depedencies.
   */
  template <typename YAML_NODE>
  static yaml FromYAMLCPP(const YAML_NODE& n);

  /** \overload (loads an existing YAMLCPP into this) */
  template <typename YAML_NODE>
  void loadFromYAMLCPP(const YAML_NODE& n);

  /** Creates a yaml dictionary node from an Eigen or mrpt::math matrix.
   * Example (compatible with OpenCV & ROS YAML formats):
   * \code
   * rows: 2
   * cols: 3
   * data: [11, 12, 13, 21, 22, 23]
   * \endcode
   * \sa toMatrix()
   */
  template <typename MATRIX>
  static yaml FromMatrix(const MATRIX& m);

  /** Fills in a matrix from a yaml dictionary node.
   * The matrix can be either an Eigen or mrpt::math matrix.
   * Example yaml node (compatible with OpenCV & ROS YAML formats):
   * \code
   * rows: 2
   * cols: 3
   * data: [11, 12, 13, 21, 22, 23]
   * \endcode
   * \sa FromMatrix()
   */
  template <typename MATRIX>
  void toMatrix(MATRIX& m) const;

  /** Converts a sequence yaml node into a std::vector, trying to convert
   * all nodes to the same given `Scalar` type. \note (New in MRPT 2.3.3)
   */
  template <typename Scalar>
  std::vector<Scalar> toStdVector() const;

  /** @} */

  /** @name Content and type checkers
   * @{ */
  /** For map nodes, checks if the given key name exists.
   *  Returns false if the node is a `null` node.
   *  Throws if the node is not a map or null.
   */
  bool has(const std::string& key) const;

  /** For map or sequence nodes, checks if the container is empty. Also
   * returns true for null(empty) nodes. */
  bool empty() const;

  /** Resets to empty (can be called on a root node or any other node to
   * clear that subtree only). */
  void clear();

  [[nodiscard]] bool isNullNode() const;
  [[nodiscard]] bool isScalar() const;
  [[nodiscard]] bool isSequence() const;
  [[nodiscard]] bool isMap() const;

  /** For scalar nodes, returns its type, or typeid(void) if an empty
   * node. \exception std::exception If called on a map or sequence. */
  const std::type_info& scalarType() const;

  /** @} */

  /** @name Range-for and conversion helpers
   * @{ */

  /** Use: `for (auto &kv: n.asSequence()) {...}`
   * \exception std::exception If called on a non-sequence node. */
  sequence_t& asSequence();
  [[nodiscard]] const sequence_t& asSequence() const;

  /// Returns a copy of asSequence(), suitable for range-based loops
  [[nodiscard]] sequence_t asSequenceRange() const { return asSequence(); }

  /** Use: `for (auto &kv: n.asMap()) {...}`
   * \exception std::exception If called on a non-map node. */
  map_t& asMap();
  [[nodiscard]] const map_t& asMap() const;

  /// Returns a copy of asMap(), suitable for range-based loops
  [[nodiscard]] map_t asMapRange() const { return asMap(); }

  /** \exception std::exception If called on a non-scalar node. */
  scalar_t& asScalar();
  [[nodiscard]] const scalar_t& asScalar() const;

  /** Returns 1 for null or scalar nodes, the number of children for
   * sequence or map nodes. */
  [[nodiscard]] size_t size() const;

  /** For a master yaml document, returns the root node; otherwise, the
   * referenced node. */
  [[nodiscard]] node_t& node() { return *dereferenceProxy(); }
  /** \overload */
  [[nodiscard]] const node_t& node() const { return *dereferenceProxy(); }

  /** Maps only: returns a reference to the key node of a key-value pair.
   * \exception std::exception If called on a non-map node or key does not
   * exist.
   */
  [[nodiscard]] const node_t& keyNode(const std::string& keyName) const;
  [[nodiscard]] node_t& keyNode(const std::string& keyName);

  /** @} */

  /** @name Print and export
   * @{ */

  /** Prints the document in YAML format to the given stream. */
  void printAsYAML(std::ostream& o, const YamlEmitOptions& eo = {}) const;

  /// \overload (prints to std::cout)
  void printAsYAML() const;

  /** Prints a tree-like representation of all nodes in the document in a
   * custom format (nor YAML neither JSON). */
  void printDebugStructure(std::ostream& o) const;

  /** @} */

  /** @name Read/write to maps (dictionaries)
   * @{ */

  /** Write access for maps — returns a mutable reference to the child node */
  yaml_ref operator[](const std::string& key);
  /// \overload
  yaml_ref operator[](const char* key);

  /** Read access for maps — returns a const reference to the child node.
   * \throw std::runtime_error if key does not exist. */
  [[nodiscard]] yaml_cref operator[](const std::string& key) const;
  /// \overload
  [[nodiscard]] yaml_cref operator[](const char* key) const;

  /** Scalar read access for maps, with default value if key does not
   * exist.
   */
  template <typename T>
  [[nodiscard]] T getOrDefault(const std::string& key, const T& defaultValue) const
  {
    MRPT_START
    const node_t* n = dereferenceProxy();
    if (n->isNullNode())
    {
      return defaultValue;
    }
    if (!n->isMap())
    {
      THROW_EXCEPTION_FMT(
          "getOrDefault() is only for map nodes, invoked on a node of type: '%s'",
          n->typeName().c_str());
    }

    const auto& m = std::get<map_t>(n->d);
    auto it = std::find_if(
        m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
    if (m.end() == it)
    {
      return defaultValue;
    }
    try
    {
      return it->second.template as<T>();
    }
    catch (const std::exception& e)
    {
      throw std::logic_error(mrpt::format(
          "getOrDefault(): Trying to access key `%s` holding type `%s` as the wrong type: `%s`",
          key.c_str(), n->typeName().c_str(), e.what()));
    }
    MRPT_END
  }
  /** @} */

  /** Write into an existing index of a sequence.
   * \throw std::out_of_range if index is out of range. */
  yaml_ref operator()(int index);
  /** Read from an existing index of a sequence.
   * \throw std::out_of_range if index is out of range. */
  yaml_cref operator()(int index) const;

  /** Write into an existing index of a sequence (alias for operator()).
   * \throw std::out_of_range if index is out of range. */
  yaml_ref operator[](int index);
  /** Read from an existing index of a sequence (alias for operator() const).
   * \throw std::out_of_range if index is out of range. */
  yaml_cref operator[](int index) const;

  /** Maps only: removes the entry with the given key.
   * \return Number of entries removed (0 or 1). */
  size_t erase(const std::string& key);

  /** Sequences only: removes the element at the given index.
   * \return true if removed, false if index out of range. */
  bool erase(int index);

  /** Append a new value to a sequence.
   * \throw std::exception If this is not a sequence */
  void push_back(double v) { internalPushBack(v); }
  /// \overload
  void push_back(const std::string& v) { internalPushBack(v); }
  /// \overload
  void push_back(uint64_t v) { internalPushBack(v); }
  /// \overload
  void push_back(bool v) { internalPushBack<bool>(v); }
  /// \overload
  void push_back(const yaml& v)
  {
    sequence_t& seq = asSequence();
    seq.emplace_back(v.node());
  }

 private:
  node_t root_;

  /** Returns a pointer to the root node. Will never return nullptr. */
  [[nodiscard]] const node_t* dereferenceProxy() const { return &root_; }
  [[nodiscard]] node_t* dereferenceProxy() { return &root_; }

  // Legacy proxy constructor — used internally for YAMLCPP interop only:
  explicit yaml([[maybe_unused]] internal::tag_as_const_proxy_t, const node_t& val, std::string) :
      root_(val)
  {
  }

 public:
  /** @name Getters / setters
   * @{ */

  /** Returns a copy of the existing value of the given type, or tries to
   * convert it between easily-compatible types (e.g. double<->int,
   * string<->int).
   * \exception std::exception If the contained type does not  match and
   * there is no obvious conversion.
   */
  template <typename T>
  T as() const
  {
    return internal::implAsGetter<T>(*this);
  }

  /** Returns a ref to the existing or new value of the given type. If
   * types do not match, the old content will be discarded and a new
   * variable created into this scalar node. \exception std::exception If
   * accessing to a non-scalar node.
   */
  template <typename T>
  T& asRef();

  /** const version of asRef(). Unlike `as<T>()`, this version will NOT
   * try to convert between types if T does not match exactly the stored
   * type, and will raise an exception instead. */
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

  // Additional operator for "size_t", in systems/compilers where
  // size_t != all other types above
  // (e.g. OSX with clang, see
  // https://stackoverflow.com/a/11603907/1631514 )
  template <
      typename = std::enable_if<
          !std::is_same_v<std::size_t, uint64_t> && !std::is_same_v<std::size_t, int64_t> &&
          !std::is_same_v<std::size_t, uint32_t> && !std::is_same_v<std::size_t, int32_t>>  //
      >
  yaml& operator=(std::size_t v)
  {
    return operator=(static_cast<uint64_t>(v));
  }

  yaml& operator=(const std::string& v);
  yaml& operator=(const char* v) { return operator=(std::string(v)); }
  yaml& operator=(const std::string_view& v) { return operator=(std::string(v)); }
  yaml& operator=(const yaml& v);

  /** vcp (value-comment) wrapper */
  template <typename T>
  yaml& operator=(const ValueCommentPair<T>& vc)
  {
    this->comment(vc.comment, vc.position);
    operator=(vc.value);
    return *this;
  }

  /** vkcp (value-keyComment) wrapper */
  template <typename T>
  yaml& operator<<(const ValueKeyCommentPair<T>& vc)
  {
    // Init as map on first use:
    if (isNullNode())
    {
      node().d.emplace<map_t>();
    }
    ASSERTMSG_(
        isMap(),
        "<< operator with ValueKeyCommentPair requires a map "
        "(dictionary) "
        "on the left hand.");
    operator[](vc.keyname) = vc.value;
    // keyComment:
    const auto posIndex = static_cast<unsigned int>(vc.position);
    ASSERT_LT_(posIndex, static_cast<unsigned int>(CommentPosition::MAX));
    auto& n = keyNode(vc.keyname);
    n.commentSlot(vc.position).emplace(vc.comment);
    return *this;
  }

  operator bool() const { return as<bool>(); }

  operator double() const { return as<double>(); }
  operator float() const { return as<float>(); }

  operator int8_t() const { return as<int8_t>(); }
  operator uint8_t() const { return as<uint8_t>(); }
  operator int16_t() const { return as<int16_t>(); }
  operator uint16_t() const { return as<uint16_t>(); }
  operator int32_t() const { return as<int32_t>(); }
  operator uint32_t() const { return as<uint32_t>(); }
  operator int64_t() const { return as<int64_t>(); }
  operator uint64_t() const { return as<uint64_t>(); }

  operator std::string() const { return as<std::string>(); }
  /** @} */

  /** @name Leaf node comments API
   * @{ */

  /** Returns true if the proxied node has an associated comment block, at
   * any location */
  [[nodiscard]] bool hasComment() const;

  /** Returns true if the proxied node has an associated comment block at
   * a particular position */
  [[nodiscard]] bool hasComment(CommentPosition pos) const;

  /** Gets the comment associated to the proxied node. This version
   * returns the first comment, of all possible (top, right).
   *
   * \exception std::exception If there is no comment attached.
   * \sa hasComment()
   */
  [[nodiscard]] const std::string& comment() const;

  /** Gets the comment associated to the proxied node, at the particular
   * position. See code examples in mrpt::containers::yaml.
   *
   * \exception std::exception If there is no comment attached.
   * \sa hasComment()
   */
  [[nodiscard]] const std::string& comment(CommentPosition pos) const;

  /** Sets the comment attached to a given proxied node.
   * See code examples in mrpt::containers::yaml
   * \sa hasComment()
   */
  void comment(const std::string& c, CommentPosition position = CommentPosition::RIGHT);

  /** @} */

  /** @name Map key node comments API
   * @{ */

  /** Maps only: returns true if the given key node has an associated
   * comment block, at any location. \exception std::exception If called
   * on a non-map or key does not exist.
   */
  bool keyHasComment(const std::string& key) const;

  /** Maps only: Returns true if the given key has an associated comment
   * block at a particular position. \exception std::exception If called
   * on a non-map or key does not exist.
   */
  [[nodiscard]] bool keyHasComment(const std::string& key, CommentPosition pos) const;

  /** Maps only: Gets the comment associated to the given key. This
   * version returns the first comment, of all possible (top, right).
   *
   * \exception std::exception If called on a non-map or key does not
   * exist. \exception std::exception If there is no comment attached. \sa
   * hasComment()
   */
  [[nodiscard]] const std::string& keyComment(const std::string& key) const;

  /** Maps only: Gets the comment associated to the given key, at the
   * particular position. See code examples in mrpt::containers::yaml.
   *
   * \exception std::exception If called on a non-map or key does not
   * exist. \exception std::exception If there is no comment attached. \sa
   * hasComment()
   */
  [[nodiscard]] const std::string& keyComment(const std::string& key, CommentPosition pos) const;

  /** Maps only: Sets the comment attached to a given key.
   * See code examples in mrpt::containers::yaml
   *
   * \exception std::exception If called on a non-map or key does not
   * exist. \sa hasComment()
   */
  void keyComment(
      const std::string& key,
      const std::string& c,
      CommentPosition position = CommentPosition::TOP);

  /** @} */

 private:
  template <typename T>
  friend T internal::implAsGetter(const yaml& p);

  struct InternalPrintState
  {
    YamlEmitOptions eo;

    unsigned int indent = 0;
    bool needsNL = false;
    bool needsSpace = false;
    bool shortFormat = false;
  };

  // Return: true if the last printed char is a newline char
  static bool internalPrintNodeAsYAML(
      const node_t& p, std::ostream& o, const InternalPrintState& ps);

  static void internalPrintDebugStructure(const node_t& p, std::ostream& o, unsigned int indent);

  template <typename T>
  void internalPushBack(const T& v);

  static bool internalPrintAsYAML(
      const std::monostate&, std::ostream& o, const InternalPrintState& ps, const comments_t& cs);
  static bool internalPrintAsYAML(
      const sequence_t& v, std::ostream& o, const InternalPrintState& ps, const comments_t& cs);
  static bool internalPrintAsYAML(
      const map_t& v, std::ostream& o, const InternalPrintState& ps, const comments_t& cs);
  static bool internalPrintAsYAML(
      const scalar_t& v, std::ostream& o, const InternalPrintState& ps, const comments_t& cs);
  static bool internalPrintStringScalar(
      const std::string& s, std::ostream& o, const InternalPrintState& ps, const comments_t& cs);

  /** Impl of operator=() — normalizes integer/float widths before storage */
  template <typename T>
  yaml& implOpAssign(const T& v)
  {
    if constexpr (std::is_same_v<T, bool>)
      root_.d = scalar_t(v);
    else if constexpr (std::is_integral_v<T> && std::is_signed_v<T>)
      root_.d = scalar_t(static_cast<int64_t>(v));
    else if constexpr (std::is_integral_v<T> && std::is_unsigned_v<T>)
      root_.d = scalar_t(static_cast<uint64_t>(v));
    else if constexpr (std::is_floating_point_v<T>)
      root_.d = scalar_t(static_cast<double>(v));
    else
      root_.d = scalar_t(v);  // std::string, shared_ptr<yaml>
    return *this;
  }
};

/** Prints a scalar, a part of a yaml tree, or the entire structure,
 * in YAML-like format. This version does NOT emit neither the YAML header
 * nor the final end line.
 *
 * \sa yaml::PrintAsYAML
 */
std::ostream& operator<<(std::ostream& o, const yaml& p);

/** Non-owning mutable reference into a yaml tree node.
 * Returned by yaml::operator[]. Implicitly converts to yaml (deep copy) for
 * reading; assignments write through to the referenced node.
 *
 * \note Invalidated if the parent yaml container reallocates (e.g. a new
 *   map key is inserted). Treat it as a temporary handle, not a persistent
 *   reference.
 * \note [New in MRPT 3.x]
 */
class yaml_ref
{
 public:
  using node_t = yaml::node_t;
  using scalar_t = yaml::scalar_t;
  using sequence_t = yaml::sequence_t;
  using map_t = yaml::map_t;

  yaml_ref() = delete;
  explicit yaml_ref(node_t& n) : node_(&n) {}

  // ── Implicit conversion to yaml (deep copy) ─────────────────────────────
  operator yaml() const { return yaml(*node_); }

  // ── Node identity ────────────────────────────────────────────────────────
  [[nodiscard]] node_t& node() { return *node_; }
  [[nodiscard]] const node_t& node() const { return *node_; }

  // ── Type queries ─────────────────────────────────────────────────────────
  [[nodiscard]] bool isNullNode() const { return node_->isNullNode(); }
  [[nodiscard]] bool isScalar() const { return node_->isScalar(); }
  [[nodiscard]] bool isMap() const { return node_->isMap(); }
  [[nodiscard]] bool isSequence() const { return node_->isSequence(); }
  [[nodiscard]] std::string typeName() const { return node_->typeName(); }

  // ── Scalar read ──────────────────────────────────────────────────────────
  template <typename T>
  [[nodiscard]] T as() const
  {
    return node_->as<T>();
  }

  // ── Container queries ────────────────────────────────────────────────────
  [[nodiscard]] size_t size() const { return node_->size(); }
  [[nodiscard]] bool empty() const { return yaml(*node_).empty(); }
  [[nodiscard]] bool has(const std::string& key) const { return yaml(*node_).has(key); }
  void clear() { *node_ = node_t{}; }

  // ── Container access ─────────────────────────────────────────────────────
  [[nodiscard]] sequence_t& asSequence() { return node_->asSequence(); }
  [[nodiscard]] const sequence_t& asSequence() const { return node_->asSequence(); }
  [[nodiscard]] map_t& asMap() { return node_->asMap(); }
  [[nodiscard]] const map_t& asMap() const { return node_->asMap(); }
  [[nodiscard]] map_t asMapRange() const { return node_->asMap(); }
  [[nodiscard]] scalar_t& asScalar() { return node_->asScalar(); }
  [[nodiscard]] const scalar_t& asScalar() const { return node_->asScalar(); }

  // ── Key-node comment API (map nodes) ─────────────────────────────────────
  bool keyHasComment(const std::string& key) const
  {
    // direct lookup to avoid dangling refs through temporaries
    const map_t& m = node_->asMap();
    auto it = std::find_if(
        m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
    ASSERTMSG_(it != m.end(), mrpt::format("key '%s' not found", key.c_str()));
    return it->first.hasComment();
  }
  std::string keyComment(const std::string& key) const
  {
    const map_t& m = node_->asMap();
    auto it = std::find_if(
        m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
    ASSERTMSG_(it != m.end(), mrpt::format("key '%s' not found", key.c_str()));
    return it->first.comment();  // return by value
  }
  [[nodiscard]] const node_t& keyNode(const std::string& key) const
  {
    const map_t& m = node_->asMap();
    auto it = std::find_if(
        m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
    ASSERTMSG_(it != m.end(), mrpt::format("key '%s' not found", key.c_str()));
    return it->first;
  }
  node_t& keyNode(const std::string& key)
  {
    map_t& m = node_->asMap();
    auto it = std::find_if(
        m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
    ASSERTMSG_(it != m.end(), mrpt::format("key '%s' not found", key.c_str()));
    return const_cast<node_t&>(it->first);
  }

  // ── erase ─────────────────────────────────────────────────────────────────
  size_t erase(const std::string& key)
  {
    map_t& m = node_->asMap();
    const auto it = std::find_if(
        m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
    if (it == m.end()) return 0;
    m.erase(it);
    return 1;
  }
  bool erase(int index)
  {
    sequence_t& seq = node_->asSequence();
    if (index < 0 || index >= static_cast<int>(seq.size())) return false;
    seq.erase(seq.begin() + index);
    return true;
  }

  // ── Comments ─────────────────────────────────────────────────────────────
  [[nodiscard]] bool hasComment() const { return node_->hasComment(); }
  [[nodiscard]] bool hasComment(CommentPosition pos) const { return node_->hasComment(pos); }
  [[nodiscard]] const std::string& comment() const { return node_->comment(); }
  [[nodiscard]] const std::string& comment(CommentPosition pos) const
  {
    return node_->comment(pos);
  }
  void comment(const std::string& c, CommentPosition pos = CommentPosition::RIGHT)
  {
    node_->commentSlot(pos).emplace(c);
  }

  // ── Print ────────────────────────────────────────────────────────────────
  void printAsYAML(std::ostream& o, const YamlEmitOptions& eo = {}) const
  {
    yaml(*node_).printAsYAML(o, eo);
  }
  void printAsYAML() const { yaml(*node_).printAsYAML(); }

  // ── Subscript (returns yaml_ref/yaml_cref) ───────────────────────────────
  yaml_ref operator[](const std::string& key);
  yaml_ref operator[](const char* key);  // avoid char→int ambiguity
  [[nodiscard]] yaml_cref operator[](const std::string& key) const;
  [[nodiscard]] yaml_cref operator[](const char* key) const;
  yaml_ref operator[](int index);
  [[nodiscard]] yaml_cref operator[](int index) const;

  // ── Sequence operator() alias ─────────────────────────────────────────────
  yaml_ref operator()(int index) { return operator[](index); }
  [[nodiscard]] yaml_cref operator()(int index) const;  // defined after yaml_cref

  // ── scalarType, getOrDefault, asRef ──────────────────────────────────────
  [[nodiscard]] const std::type_info& scalarType() const
  {
    if (node_->isNullNode()) return typeid(void);
    return std::visit(
        [](const auto& v) -> const std::type_info& { return typeid(v); }, node_->asScalar());
  }
  template <typename T>
  [[nodiscard]] T getOrDefault(const std::string& key, const T& def) const
  {
    return yaml(*node_).template getOrDefault<T>(key, def);
  }
  template <typename T>
  T& asRef()
  {
    auto& s = node_->asScalar();
    if (!std::holds_alternative<T>(s)) s = T{};
    return std::get<T>(s);
  }

  // ── Assignment (write-through) ───────────────────────────────────────────
  yaml_ref& operator=(bool v)
  {
    node_->d = scalar_t(v);
    return *this;
  }
  yaml_ref& operator=(float v)
  {
    node_->d = scalar_t(static_cast<double>(v));
    return *this;
  }
  yaml_ref& operator=(double v)
  {
    node_->d = scalar_t(v);
    return *this;
  }
  yaml_ref& operator=(int8_t v)
  {
    node_->d = scalar_t(static_cast<int64_t>(v));
    return *this;
  }
  yaml_ref& operator=(uint8_t v)
  {
    node_->d = scalar_t(static_cast<uint64_t>(v));
    return *this;
  }
  yaml_ref& operator=(int16_t v)
  {
    node_->d = scalar_t(static_cast<int64_t>(v));
    return *this;
  }
  yaml_ref& operator=(uint16_t v)
  {
    node_->d = scalar_t(static_cast<uint64_t>(v));
    return *this;
  }
  yaml_ref& operator=(int32_t v)
  {
    node_->d = scalar_t(static_cast<int64_t>(v));
    return *this;
  }
  yaml_ref& operator=(uint32_t v)
  {
    node_->d = scalar_t(static_cast<uint64_t>(v));
    return *this;
  }
  yaml_ref& operator=(int64_t v)
  {
    node_->d = scalar_t(v);
    return *this;
  }
  yaml_ref& operator=(uint64_t v)
  {
    node_->d = scalar_t(v);
    return *this;
  }
  yaml_ref& operator=(const std::string& v)
  {
    node_->d = scalar_t(v);
    return *this;
  }
  yaml_ref& operator=(const char* v)
  {
    node_->d = scalar_t(std::string(v));
    return *this;
  }
  yaml_ref& operator=(const std::string_view& v)
  {
    node_->d = scalar_t(std::string(v));
    return *this;
  }
  yaml_ref& operator=(const yaml& v)
  {
    *node_ = v.node();
    return *this;
  }
  yaml_ref& operator=(const yaml_ref& v)
  {
    if (node_ != v.node_) *node_ = *v.node_;
    return *this;
  }
  template <
      typename = std::enable_if<
          !std::is_same_v<std::size_t, uint64_t> && !std::is_same_v<std::size_t, int64_t> &&
          !std::is_same_v<std::size_t, uint32_t> && !std::is_same_v<std::size_t, int32_t>>>
  yaml_ref& operator=(std::size_t v)
  {
    return operator=(static_cast<uint64_t>(v));
  }
  /** vcp (value-comment) wrapper */
  template <typename T>
  yaml_ref& operator=(const ValueCommentPair<T>& vc)
  {
    comment(vc.comment, vc.position);
    operator=(vc.value);
    return *this;
  }

  // ── Implicit scalar conversions ──────────────────────────────────────────
  operator bool() const { return as<bool>(); }
  operator double() const { return as<double>(); }
  operator float() const { return as<float>(); }
  operator int8_t() const { return as<int8_t>(); }
  operator uint8_t() const { return as<uint8_t>(); }
  operator int16_t() const { return as<int16_t>(); }
  operator uint16_t() const { return as<uint16_t>(); }
  operator int32_t() const { return as<int32_t>(); }
  operator uint32_t() const { return as<uint32_t>(); }
  operator int64_t() const { return as<int64_t>(); }
  operator uint64_t() const { return as<uint64_t>(); }
  operator std::string() const { return as<std::string>(); }

  // ── Sequence push_back ───────────────────────────────────────────────────
  void push_back(double v) { node_->asSequence().emplace_back().d = scalar_t(v); }
  void push_back(const std::string& v) { node_->asSequence().emplace_back().d = scalar_t(v); }
  void push_back(uint64_t v) { node_->asSequence().emplace_back().d = scalar_t(v); }
  void push_back(bool v) { node_->asSequence().emplace_back().d = scalar_t(v); }
  void push_back(const yaml& v) { node_->asSequence().emplace_back(v.node()); }

 private:
  node_t* node_;
};

/** Non-owning const reference into a yaml tree node.
 * Returned by `const yaml::operator[]`. Implicitly converts to yaml (deep copy).
 * \note [New in MRPT 3.x]
 */
class yaml_cref
{
 public:
  using node_t = yaml::node_t;
  using scalar_t = yaml::scalar_t;
  using sequence_t = yaml::sequence_t;
  using map_t = yaml::map_t;

  yaml_cref() = delete;
  explicit yaml_cref(const node_t& n) : node_(&n) {}
  yaml_cref(const yaml_ref& r) : node_(&r.node()) {}  // NOLINT(google-explicit-constructor)

  // ── Implicit conversion to yaml (deep copy) ─────────────────────────────
  operator yaml() const { return yaml(*node_); }

  // ── Node identity ────────────────────────────────────────────────────────
  [[nodiscard]] const node_t& node() const { return *node_; }

  // ── Type queries ─────────────────────────────────────────────────────────
  [[nodiscard]] bool isNullNode() const { return node_->isNullNode(); }
  [[nodiscard]] bool isScalar() const { return node_->isScalar(); }
  [[nodiscard]] bool isMap() const { return node_->isMap(); }
  [[nodiscard]] bool isSequence() const { return node_->isSequence(); }
  [[nodiscard]] std::string typeName() const { return node_->typeName(); }

  // ── Scalar read ──────────────────────────────────────────────────────────
  template <typename T>
  [[nodiscard]] T as() const
  {
    return node_->as<T>();
  }

  // ── Container queries ────────────────────────────────────────────────────
  [[nodiscard]] size_t size() const { return node_->size(); }
  [[nodiscard]] bool has(const std::string& key) const { return yaml(*node_).has(key); }

  // ── Matrix / vector conversion (delegates to yaml) ──────────────────────
  template <typename MATRIX>
  void toMatrix(MATRIX& m) const
  {
    yaml(*node_).toMatrix(m);
  }
  template <typename Scalar>
  [[nodiscard]] std::vector<Scalar> toStdVector() const
  {
    return yaml(*node_).toStdVector<Scalar>();
  }

  // ── Container access ─────────────────────────────────────────────────────
  [[nodiscard]] const sequence_t& asSequence() const { return node_->asSequence(); }
  [[nodiscard]] const map_t& asMap() const { return node_->asMap(); }
  [[nodiscard]] map_t asMapRange() const { return node_->asMap(); }
  [[nodiscard]] const scalar_t& asScalar() const { return node_->asScalar(); }

  // ── Sequence operator() alias ─────────────────────────────────────────────
  yaml_cref operator()(int index) const { return operator[](index); }

  // ── Comments ─────────────────────────────────────────────────────────────
  [[nodiscard]] bool hasComment() const { return node_->hasComment(); }
  [[nodiscard]] bool hasComment(CommentPosition pos) const { return node_->hasComment(pos); }
  [[nodiscard]] const std::string& comment() const { return node_->comment(); }
  [[nodiscard]] const std::string& comment(CommentPosition pos) const
  {
    return node_->comment(pos);
  }

  // ── Print ────────────────────────────────────────────────────────────────
  void printAsYAML(std::ostream& o, const YamlEmitOptions& eo = {}) const
  {
    yaml(*node_).printAsYAML(o, eo);
  }

  // ── Subscript (returns yaml_cref) ────────────────────────────────────────
  yaml_cref operator[](const std::string& key) const;
  yaml_cref operator[](const char* key) const;
  yaml_cref operator[](int index) const;

  // ── scalarType, asRef ────────────────────────────────────────────────────
  [[nodiscard]] const std::type_info& scalarType() const
  {
    if (node_->isNullNode()) return typeid(void);
    return std::visit(
        [](const auto& v) -> const std::type_info& { return typeid(v); }, node_->asScalar());
  }
  template <typename T>
  [[nodiscard]] const T& asRef() const
  {
    const auto& s = node_->asScalar();
    if (!std::holds_alternative<T>(s)) THROW_EXCEPTION("asRef<T>(): type mismatch");
    return std::get<T>(s);
  }

  // ── Implicit scalar conversions ──────────────────────────────────────────
  operator bool() const { return as<bool>(); }
  operator double() const { return as<double>(); }
  operator float() const { return as<float>(); }
  operator int8_t() const { return as<int8_t>(); }
  operator uint8_t() const { return as<uint8_t>(); }
  operator int16_t() const { return as<int16_t>(); }
  operator uint16_t() const { return as<uint16_t>(); }
  operator int32_t() const { return as<int32_t>(); }
  operator uint32_t() const { return as<uint32_t>(); }
  operator int64_t() const { return as<int64_t>(); }
  operator uint64_t() const { return as<uint64_t>(); }
  operator std::string() const { return as<std::string>(); }

 private:
  const node_t* node_;
};

std::ostream& operator<<(std::ostream& o, const yaml_ref& p);
std::ostream& operator<<(std::ostream& o, const yaml_cref& p);

// Deferred yaml_ref methods that return yaml_cref (yaml_cref must be complete):
inline yaml_cref yaml_ref::operator()(int index) const { return operator[](index); }

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
 *
 * Since MRPT 2.3.2, this also works for enums, converting to textual names of
 * values. Note that this requires enums to implement mrpt::typemeta::TEnumType.
 */
#define MCP_LOAD_REQ(Yaml__, Var__)                                                          \
  if (!Yaml__.has(#Var__))                                                                   \
    throw std::invalid_argument(                                                             \
        mrpt::format("Required parameter `%s` not an existing key in dictionary.", #Var__)); \
  if constexpr (std::is_enum_v<decltype(Var__)>)                                             \
    Var__ = mrpt::typemeta::TEnumType<std::remove_cv_t<decltype(Var__)>>::name2value(        \
        Yaml__[#Var__].as<std::string>());                                                   \
  else                                                                                       \
    Var__ = Yaml__[#Var__].as<decltype(Var__)>()

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
 *
 * Since MRPT 2.3.2, this also works for enums, converting to textual names of
 * values. Note that this requires enums to implement mrpt::typemeta::TEnumType.
 */
#define MCP_LOAD_OPT(Yaml__, Var__)                                                     \
  if constexpr (std::is_enum_v<decltype(Var__)>)                                        \
  {                                                                                     \
    if (!Yaml__.empty() && Yaml__.has(#Var__))                                          \
      Var__ = mrpt::typemeta::TEnumType<std::remove_cv_t<decltype(Var__)>>::name2value( \
          Yaml__[#Var__].as<std::string>());                                            \
  }                                                                                     \
  else if (!Yaml__.isNullNode() && !Yaml__.empty() && Yaml__.has(#Var__))               \
  Var__ = Yaml__[#Var__].as<decltype(Var__)>()

/** Just like MCP_LOAD_REQ(), but converts the read number from degrees to
 * radians */
#define MCP_LOAD_REQ_DEG(Yaml__, Var__) \
  MCP_LOAD_REQ(Yaml__, Var__);          \
  Var__ = mrpt::DEG2RAD(Var__)

/** Just like MCP_LOAD_OPT(), but converts the read number from degrees to
 * radians */
#define MCP_LOAD_OPT_DEG(Yaml__, Var__) \
  Var__ = mrpt::RAD2DEG(Var__);         \
  MCP_LOAD_OPT(Yaml__, Var__);          \
  Var__ = mrpt::DEG2RAD(Var__)

namespace internal
{
// We need to implement this as a template for the "if constexpr()" false branch
// not to be evaluated for enums, which would lead to build errors.
template <typename T, typename YAML_T>
void impl_mcp_save(YAML_T& y, const T& var, const char* varName)
{
  using enum_t = std::remove_cv_t<T>;

  if constexpr (std::is_enum_v<enum_t>)
  {
    y[varName] = mrpt::typemeta::TEnumType<enum_t>::value2name(var);
  }
  else
  {
    y[varName] = var;
  }
}
}  // namespace internal

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
 *
 * Since MRPT 2.3.2, this also works for enums, converting to textual names of
 * values. Note that this requires enums to implement mrpt::typemeta::TEnumType.
 */
#define MCP_SAVE(Yaml__, Var__) mrpt::containers::internal::impl_mcp_save(Yaml__, Var__, #Var__);

#define MCP_SAVE_DEG(Yaml__, Var__) Yaml__[#Var__] = mrpt::RAD2DEG(Var__);

}  // namespace mrpt::containers

namespace mrpt::containers
{
template <typename T>
T& yaml::asRef()
{
  scalar_t& s = this->asScalar();
  if (!std::holds_alternative<T>(s)) s = T{};
  return std::get<T>(s);
}

template <typename T>
const T& yaml::asRef() const
{
  const scalar_t& s = this->asScalar();
  if (!std::holds_alternative<T>(s))
  {
    std::stringstream ss;
    yaml::internalPrintAsYAML(s, ss, {}, {});
    THROW_EXCEPTION_FMT(
        "asRef<T>(): scalar holds value '%s' of a different type than requested.",
        ss.str().c_str());
  }
  return std::get<T>(s);
}

template <typename T>
void yaml::internalPushBack(const T& v)
{
  ASSERT_(this->isSequence());
  sequence_t& seq = asSequence();
  node_t& n = seq.emplace_back();
  if constexpr (std::is_same_v<T, bool>)
    n.d = scalar_t(v);
  else if constexpr (std::is_integral_v<T> && std::is_signed_v<T>)
    n.d = scalar_t(static_cast<int64_t>(v));
  else if constexpr (std::is_integral_v<T> && std::is_unsigned_v<T>)
    n.d = scalar_t(static_cast<uint64_t>(v));
  else if constexpr (std::is_floating_point_v<T>)
    n.d = scalar_t(static_cast<double>(v));
  else
    n.d = scalar_t(v);
}

template <typename YAML_NODE>
yaml yaml::FromYAMLCPP(const YAML_NODE& n)
{
  const auto invalidDbl = std::numeric_limits<double>::max();

  if (n.IsSequence())
  {
    yaml ret = yaml(Sequence());

    for (const auto& e : n)
    {
      if (e.IsNull())
      {
        sequence_t& seq = std::get<sequence_t>(ret.dereferenceProxy()->d);
        seq.push_back(node_t());
      }
      else if (e.IsScalar())
      {
        if (double v = e.template as<double>(invalidDbl); v != invalidDbl)
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
        m.emplace_back(node_t(key), node_t{});
      }
      else if (val.IsScalar())
      {
        if (double v = val.template as<double>(invalidDbl); v != invalidDbl)
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
void yaml::loadFromYAMLCPP(const YAML_NODE& n)
{
  *this = yaml::FromYAMLCPP(n);
}

template <typename MATRIX>
yaml yaml::FromMatrix(const MATRIX& m)
{
  yaml r = mrpt::containers::yaml::Map();
  r["rows"] = static_cast<int64_t>(m.rows());
  r["cols"] = static_cast<int64_t>(m.cols());
  r["data"] = mrpt::containers::yaml::Sequence();
  auto data = r["data"];
  data.node().printInShortFormat = true;
  for (int iRow = 0; iRow < m.rows(); iRow++)
    for (int iCol = 0; iCol < m.cols(); iCol++) data.push_back(m(iRow, iCol));
  return r;
}

template <typename MATRIX>
void yaml::toMatrix(MATRIX& m) const
{
  ASSERT_(isMap());
  ASSERT_(has("rows") && has("cols") && has("data"));
  const int nRows = (*this)["rows"].as<int>();
  const int nCols = (*this)["cols"].as<int>();
  ASSERT_((nRows > 0 && nCols > 0) || (nRows == 0 && nCols == 0));

  const auto& data = (*this)["data"];
  ASSERT_(data.isSequence());
  ASSERT_EQUAL_(static_cast<int>(data.size()), nRows * nCols);

  using entry_t = std::decay_t<decltype(m(0, 0))>;

  if (m.cols() <= 0 || m.rows() <= 0)
  {
    try
    {
      m.resize(nRows, nCols);
    }
    catch (const std::exception&)
    {
    }
  }
  ASSERT_EQUAL_(m.cols(), nCols);
  ASSERT_EQUAL_(m.rows(), nRows);

  for (int r = 0, idx = 0; r < nRows; r++)
    for (int c = 0; c < nCols; c++, idx++) m(r, c) = data[idx].as<entry_t>();
}

template <typename Scalar>
std::vector<Scalar> yaml::toStdVector() const
{
  ASSERT_(isSequence());
  const auto& seq = asSequence();

  std::vector<Scalar> ret;
  ret.reserve(seq.size());

  for (const auto& n : seq) ret.push_back(n.as<Scalar>());
  return ret;
}

}  // namespace mrpt::containers

// Forward-declare implAnyAsGetter so it can be referenced below:
namespace mrpt::containers::internal
{
template <typename T>
T implAnyAsGetter(const mrpt::containers::yaml::scalar_t& s);
}  // namespace mrpt::containers::internal

namespace mrpt::containers
{
// node_t::as<T>() is defined here (after implAnyAsGetter is declared above):
template <typename T>
T yaml::node_t::as() const
{
  ASSERTMSG_(
      std::holds_alternative<scalar_t>(d),
      mrpt::format(
          "Trying to use as() on a node of type `%s`, but only "
          "available for `scalar` nodes.",
          typeName().c_str()));
  return internal::implAnyAsGetter<T>(std::get<scalar_t>(d));
}
}  // namespace mrpt::containers

namespace mrpt::containers::internal
{
/** Helper: convert a scalar_t to its YAML text representation */
inline std::string scalarToString(const mrpt::containers::yaml::scalar_t& s)
{
  return std::visit(
      [](const auto& val) -> std::string
      {
        using V = std::decay_t<decltype(val)>;
        if constexpr (std::is_same_v<V, std::monostate>)
          return "~";
        else if constexpr (std::is_same_v<V, bool>)
          return val ? "true" : "false";
        else if constexpr (std::is_same_v<V, int64_t>)
          return std::to_string(val);
        else if constexpr (std::is_same_v<V, uint64_t>)
          return std::to_string(val);
        else if constexpr (std::is_same_v<V, double>)
        {
          std::stringstream ss;
          ss << mrpt::format("%.16g", val);
          return ss.str();
        }
        else if constexpr (std::is_same_v<V, std::string>)
          return val;
        else
          return "(yaml)";
      },
      s);
}

template <typename T>
T implAnyAsGetter(const mrpt::containers::yaml::scalar_t& s)
{
  using yaml = mrpt::containers::yaml;

  // Null scalar: can only be converted to bool (false) or checked explicitly
  if (std::holds_alternative<std::monostate>(s))
  {
    if constexpr (std::is_same_v<T, bool>) return false;
    THROW_EXCEPTION("Trying to read a null (empty) scalar value");
  }

  // 1) Exact match (for types that ARE variant alternatives)
  if constexpr (
      std::is_same_v<T, bool> || std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t> ||
      std::is_same_v<T, double> || std::is_same_v<T, std::string> ||
      std::is_same_v<T, std::shared_ptr<yaml>>)
  {
    if (const auto* p = std::get_if<T>(&s); p != nullptr) return *p;
  }

  // 2) bool target (not already exact-matched above, so source is numeric or string)
  if constexpr (std::is_same_v<T, bool>)
  {
    return std::visit(
        [](const auto& val) -> bool
        {
          using V = std::decay_t<decltype(val)>;
          if constexpr (std::is_arithmetic_v<V> && !std::is_same_v<V, bool>)
            return static_cast<bool>(val);
          else if constexpr (std::is_same_v<V, std::string>)
          {
            const auto& str = val;
            if (str == "y" || str == "Y" || str == "yes" || str == "Yes" || str == "YES" ||
                str == "true" || str == "True" || str == "TRUE" || str == "on" || str == "ON" ||
                str == "On")
              return true;
            if (str == "n" || str == "N" || str == "no" || str == "No" || str == "NO" ||
                str == "false" || str == "False" || str == "FALSE" || str == "off" ||
                str == "Off" || str == "OFF")
              return false;
            char* end = nullptr;
            const long long iv = std::strtoll(str.c_str(), &end, 0);
            if (end && end != str.c_str() && *end == '\0') return static_cast<bool>(iv != 0);
            THROW_EXCEPTION_FMT("Cannot convert string '%s' to bool", str.c_str());
          }
          THROW_EXCEPTION("Cannot convert this scalar type to bool");
        },
        s);
  }

  // 3) Float/double target
  if constexpr (std::is_same_v<T, double> || std::is_same_v<T, float>)
  {
    return std::visit(
        [](const auto& val) -> T
        {
          using V = std::decay_t<decltype(val)>;
          if constexpr (std::is_arithmetic_v<V> && !std::is_same_v<V, std::monostate>)
            return static_cast<T>(val);
          else if constexpr (std::is_same_v<V, std::string>)
          {
            char* end = nullptr;
            const double dv = std::strtod(val.c_str(), &end);
            if (end && end != val.c_str() && *end == '\0') return static_cast<T>(dv);
            THROW_EXCEPTION_FMT("Cannot convert string '%s' to float", val.c_str());
          }
          THROW_EXCEPTION("Cannot convert this scalar type to float");
        },
        s);
  }

  // 4) Integer targets (any integral type, signed or unsigned, any width)
  if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>)
  {
    return std::visit(
        [](const auto& val) -> T
        {
          using V = std::decay_t<decltype(val)>;
          if constexpr (std::is_arithmetic_v<V> && !std::is_same_v<V, std::monostate>)
          {
            return static_cast<T>(val);
          }
          else if constexpr (std::is_same_v<V, std::string>)
          {
            char* end = nullptr;
            errno = 0;
            const long long iv = std::strtoll(val.c_str(), &end, 0);
            if (end != nullptr && end != val.c_str() && errno != ERANGE)
            {
              const auto minVal = static_cast<long long>(std::numeric_limits<T>::min());
              auto maxVal = static_cast<long long>(std::numeric_limits<T>::max());
              if (maxVal < 0) maxVal = std::numeric_limits<long long>::max();
              if (iv < minVal || iv > maxVal)
                THROW_EXCEPTION_FMT(
                    "yaml: integer '%lld' out of range for type (valid [%lld,%lld])", iv, minVal,
                    maxVal);
              return static_cast<T>(iv);
            }
            THROW_EXCEPTION_FMT("Cannot convert string '%s' to integer", val.c_str());
          }
          THROW_EXCEPTION("Cannot convert this scalar type to integer");
        },
        s);
  }

  // 5) std::string target: serialize via the print path
  if constexpr (std::is_same_v<T, std::string>)
  {
    return scalarToString(s);
  }

  // No known conversion:
  THROW_EXCEPTION_FMT(
      "Trying to access scalar (value='%s') as type `%s` — no obvious conversion found.",
      scalarToString(s).c_str(), mrpt::demangle(typeid(T).name()).c_str());
}

template <typename T>
T implAsGetter(const yaml& p)
{
  MRPT_START
  ASSERTMSG_(
      p.isScalar(),
      mrpt::format(
          "Trying to read from a non-scalar. Actual node type: `%s`", p.node().typeName().c_str()));
  const yaml::scalar_t& s = p.asScalar();
  return implAnyAsGetter<T>(s);
  MRPT_END
}

}  // namespace mrpt::containers::internal
