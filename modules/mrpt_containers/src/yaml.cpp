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

#include <mrpt/containers/config.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>

#include <algorithm>
#include <cerrno>
#include <fstream>
#include <iostream>
#include <istream>
#include <utility>

#if MRPT_HAS_FYAML
#include <libfyaml.h>
#endif

using namespace mrpt::containers;

// ============ class: yaml::node_t =======
bool yaml::node_t::isNullNode() const
{
  return std::holds_alternative<std::monostate>(d) ||
         (std::holds_alternative<scalar_t>(d) &&
          std::holds_alternative<std::monostate>(std::get<scalar_t>(d)));
}
bool yaml::node_t::isScalar() const { return std::holds_alternative<scalar_t>(d); }
bool yaml::node_t::isSequence() const { return std::holds_alternative<sequence_t>(d); }
bool yaml::node_t::isMap() const { return std::holds_alternative<map_t>(d); }

std::string yaml::node_t::typeName() const
{
  MRPT_START
  if (isNullNode()) return "null";
  if (isSequence()) return "sequence";
  if (isMap()) return "map";
  ASSERT_(isScalar());
  const auto& s = std::get<scalar_t>(d);

  const std::string typeStr = std::visit(
      [](const auto& val) -> std::string
      {
        using V = std::decay_t<decltype(val)>;
        if constexpr (std::is_same_v<V, std::monostate>)
          return "null";
        else if constexpr (std::is_same_v<V, bool>)
          return "bool";
        else if constexpr (std::is_same_v<V, int64_t>)
          return "int64_t";
        else if constexpr (std::is_same_v<V, uint64_t>)
          return "uint64_t";
        else if constexpr (std::is_same_v<V, double>)
          return "double";
        else if constexpr (std::is_same_v<V, std::string>)
          return "std::string";
        else
          return "yaml";
      },
      s);

  return mrpt::format("scalar(%s)", typeStr.c_str());
  MRPT_END
}

yaml::sequence_t& yaml::node_t::asSequence()
{
  MRPT_START
  return const_cast<sequence_t&>(const_cast<const node_t*>(this)->asSequence());
  MRPT_END
}
const yaml::sequence_t& yaml::node_t::asSequence() const
{
  MRPT_START
  ASSERTMSG_(
      std::holds_alternative<sequence_t>(d),
      mrpt::format("Trying to access node of type `%s` as a sequence", typeName().c_str()));

  return std::get<sequence_t>(d);
  MRPT_END
}

yaml::map_t& yaml::node_t::asMap()
{
  MRPT_START
  return const_cast<map_t&>(const_cast<const node_t*>(this)->asMap());
  MRPT_END
}
const yaml::map_t& yaml::node_t::asMap() const
{
  MRPT_START
  ASSERTMSG_(
      std::holds_alternative<map_t>(d),
      mrpt::format("Trying to access node of type `%s` as a map", typeName().c_str()));
  return std::get<map_t>(d);
  MRPT_END
}

yaml::scalar_t& yaml::node_t::asScalar()
{
  MRPT_START
  return const_cast<scalar_t&>(const_cast<const node_t*>(this)->asScalar());
  MRPT_END
}
const yaml::scalar_t& yaml::node_t::asScalar() const
{
  MRPT_START
  ASSERTMSG_(
      std::holds_alternative<scalar_t>(d),
      mrpt::format("Trying to access node of type `%s` as a scalar", typeName().c_str()));

  return std::get<scalar_t>(d);
  MRPT_END
}

size_t yaml::node_t::size() const
{
  if (isScalar() || isNullNode())
  {
    return 1;
  }
  if (isMap())
  {
    return asMap().size();
  }
  return asSequence().size();
}

// ============ class: yaml =======
yaml::yaml(const yaml& v) { *this = v; }

bool yaml::isScalar() const { return dereferenceProxy()->isScalar(); }
bool yaml::isNullNode() const { return dereferenceProxy()->isNullNode(); }
bool yaml::isMap() const { return dereferenceProxy()->isMap(); }
bool yaml::isSequence() const { return dereferenceProxy()->isSequence(); }

yaml::sequence_t& yaml::asSequence()
{
  MRPT_START
  return const_cast<sequence_t&>(const_cast<const yaml*>(this)->asSequence());
  MRPT_END
}
const yaml::sequence_t& yaml::asSequence() const
{
  MRPT_START
  return dereferenceProxy()->asSequence();
  MRPT_END
}

yaml::map_t& yaml::asMap()
{
  MRPT_START
  return const_cast<map_t&>(const_cast<const yaml*>(this)->asMap());
  MRPT_END
}
const yaml::map_t& yaml::asMap() const
{
  MRPT_START
  return dereferenceProxy()->asMap();
  MRPT_END
}

yaml::scalar_t& yaml::asScalar()
{
  MRPT_START
  return const_cast<scalar_t&>(const_cast<const yaml*>(this)->asScalar());
  MRPT_END
}

const yaml::scalar_t& yaml::asScalar() const
{
  MRPT_START
  return dereferenceProxy()->asScalar();
  MRPT_END
}

size_t yaml::size() const { return dereferenceProxy()->size(); }

bool yaml::has(const std::string& key) const
{
  if (isNullNode()) return false;
  const map_t& m = this->asMap();
  return m.end() != std::find_if(
                        m.begin(), m.end(),
                        [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
}

const std::type_info& yaml::scalarType() const
{
  MRPT_START
  if (this->isNullNode()) return typeid(void);

  const scalar_t& s = this->asScalar();
  return std::visit([](const auto& val) -> const std::type_info& { return typeid(val); }, s);
  MRPT_END
}

// dereferenceProxy() is now inlined in yaml.h — returning &root_ directly.

bool yaml::empty() const
{
  auto n = dereferenceProxy();
  if (std::holds_alternative<map_t>(n->d))
  {
    return std::get<map_t>(n->d).empty();
  }

  if (std::holds_alternative<sequence_t>(n->d))
  {
    return std::get<sequence_t>(n->d).empty();
  }

  if (std::holds_alternative<std::monostate>(n->d))
  {
    return true;
  }

  if (std::holds_alternative<scalar_t>(n->d))
    THROW_EXCEPTION(
        "empty() called on a scalar node: only available for maps or "
        "sequences.");

  THROW_EXCEPTION("Should never reach here");
}

void yaml::clear()
{
  auto n = dereferenceProxy();
  *n = node_t();
}

yaml& yaml::operator=(bool v) { return implOpAssign(v); }

yaml& yaml::operator=(double v) { return implOpAssign(v); }
yaml& yaml::operator=(float v) { return implOpAssign(v); }

yaml& yaml::operator=(int8_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint8_t v) { return implOpAssign(v); }

yaml& yaml::operator=(int16_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint16_t v) { return implOpAssign(v); }

yaml& yaml::operator=(int32_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint32_t v) { return implOpAssign(v); }

yaml& yaml::operator=(int64_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint64_t v) { return implOpAssign(v); }

yaml& yaml::operator=(const std::string& v) { return implOpAssign(v); }
yaml& yaml::operator=(const yaml& v)
{
  MRPT_START
  root_ = v.root_;
  return *this;
  MRPT_END
}

yaml_ref yaml::operator[](const std::string& s)
{
  node_t* n = dereferenceProxy();
  if (n->isNullNode()) n->d.emplace<map_t>();
  if (!n->isMap()) THROW_EXCEPTION("write operator[] not applicable to non-map nodes.");

  map_t& m = std::get<map_t>(n->d);
  auto it = std::find_if(
      m.begin(), m.end(), [&s](const auto& kv) { return kv.first.internalAsStr() == s; });
  if (it == m.end())
  {
    m.emplace_back(node_t(s), node_t{});
    return yaml_ref(m.back().second);
  }
  return yaml_ref(it->second);
}

yaml_cref yaml::operator[](const std::string& key) const
{
  const node_t* n = dereferenceProxy();
  if (n->isNullNode()) THROW_EXCEPTION("read operator[] not applicable to null nodes.");
  if (!n->isMap()) THROW_EXCEPTION("read operator[] only available for map nodes.");

  const map_t& m = std::get<map_t>(n->d);
  auto it = std::find_if(
      m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
  if (m.end() == it) THROW_EXCEPTION_FMT("Access non-existing map key `%s`", key.c_str());

  return yaml_cref(it->second);
}

yaml_ref yaml::operator()(int index)
{
  node_t* n = dereferenceProxy();
  ASSERTMSG_(!n->isNullNode(), "write operator() not applicable to empty nodes or sequences.");
  ASSERTMSG_(n->isSequence(), "write operator() only available for sequence nodes.");

  sequence_t& seq = std::get<sequence_t>(n->d);
  if (index < 0 || index >= static_cast<int>(seq.size()))
    THROW_TYPED_EXCEPTION("yaml::operator() out of range", std::out_of_range);

  return yaml_ref(seq.at(static_cast<std::size_t>(index)));
}
yaml_cref yaml::operator()(int index) const
{
  const node_t* n = dereferenceProxy();
  ASSERTMSG_(!n->isNullNode(), "read operator() not applicable to empty nodes or sequences.");
  ASSERTMSG_(n->isSequence(), "read operator() only available for sequence nodes.");

  const sequence_t& seq = std::get<sequence_t>(n->d);
  if (index < 0 || index >= static_cast<int>(seq.size()))
    THROW_TYPED_EXCEPTION("yaml::operator() out of range", std::out_of_range);

  return yaml_cref(seq.at(static_cast<std::size_t>(index)));
}

yaml_ref yaml::operator[](int index) { return operator()(index); }
yaml_cref yaml::operator[](int index) const { return operator()(index); }

yaml_ref yaml::operator[](const char* key)
{
  ASSERT_(key != nullptr);
  return operator[](std::string(key));
}
yaml_cref yaml::operator[](const char* key) const
{
  ASSERT_(key != nullptr);
  return operator[](std::string(key));
}

size_t yaml::erase(const std::string& key)
{
  map_t& m = asMap();
  const auto it = std::find_if(
      m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
  if (it == m.end()) return 0;
  m.erase(it);
  return 1;
}

bool yaml::erase(int index)
{
  sequence_t& seq = asSequence();
  if (index < 0 || index >= static_cast<int>(seq.size())) return false;
  seq.erase(seq.begin() + index);
  return true;
}

void yaml::printAsYAML() const { printAsYAML(std::cout); }

void yaml::printAsYAML(std::ostream& o, const YamlEmitOptions& eo) const
{
  const node_t* n = dereferenceProxy();

  if (eo.emitHeader)
  {
    o << "%YAML 1.2\n---\n";
  }

  InternalPrintState ps;
  ps.eo = eo;

  bool lastLineNL = yaml::internalPrintNodeAsYAML(*n, o, ps);
  if (!lastLineNL && eo.endWithNewLine)
  {
    o << "\n";
  }
}

void yaml::printDebugStructure(std::ostream& o) const
{
  const node_t* n = dereferenceProxy();
  unsigned int indent = 0;
  internalPrintDebugStructure(*n, o, indent);
}

bool yaml::internalPrintNodeAsYAML(const node_t& p, std::ostream& o, const InternalPrintState& ps)
{
  // Build a comments_t view (all-empty if no meta):
  static const comments_t emptyComments{};
  const comments_t& cs = p.meta ? p.meta->comments : emptyComments;

  // Handle "top" comments, which is common to all types:
  if (const auto& tc = cs[static_cast<size_t>(CommentPosition::TOP)]; tc.has_value())
  {
    const std::string sInd(ps.indent, ' ');
    const std::string& comment = tc.value();

    // Split line by line:
    for (size_t i = 0; i < comment.size();)
    {
      if (i > 0)
      {
        o << sInd;
      }

      size_t nextLN = comment.find('\n', i);
      if (nextLN == std::string::npos)
      {
        nextLN = comment.size();
      }

      const size_t lineLen = nextLN - i;
      std::string_view line(comment.data() + i, lineLen);
      o << "# " << line << "\n";

      i += lineLen + 1;
    }
    // Indent of next line:
    if (!comment.empty())
    {
      o << sInd;
    }
  }

  // Dispatch:
  if (p.isScalar()) return internalPrintAsYAML(std::get<scalar_t>(p.d), o, ps, cs);

  if (p.isNullNode()) return internalPrintAsYAML(std::monostate(), o, ps, cs);

  auto ps2 = ps;

  if (p.isMap())
  {
    if (ps.needsNL)
    {
      o << "\n";
      ps2.indent += 2;
      ps2.needsNL = false;
    }
    return internalPrintAsYAML(std::get<map_t>(p.d), o, ps2, cs);
  }

  if (p.isSequence())
  {
    if (ps.needsNL)
    {
      if (p.printInShortFormat)
      {
        o << " ";
      }
      else
      {
        o << "\n";
        if (ps.eo.indentSequences)
        {
          ps2.indent += 2;
        }
      }
      ps2.needsNL = false;
    }

    ps2.shortFormat = p.printInShortFormat;
    return internalPrintAsYAML(std::get<sequence_t>(p.d), o, ps2, cs);
  }

  THROW_EXCEPTION("Should never reach here");
}

namespace
{
std::string shortenComment(const std::optional<std::string>& c)
{
  if (!c.has_value())
  {
    return {"[none]"};
  }

  const size_t maxLen = 14;
  if (c.value().size() > maxLen - 3)
  {
    return {c.value().substr(0, maxLen) + "..."};
  }
  return {c.value()};
}
}  // namespace

void yaml::internalPrintDebugStructure(const node_t& p, std::ostream& o, unsigned int indent)
{
  const std::string sInd(indent, ' ');
  const auto* m = p.meta.get();

  o << sInd << "- " << p.typeName() << ". Comments:";
  if (m == nullptr || (!m->comments.at(0).has_value() && !m->comments.at(1).has_value()))
  {
    o << " [none]. ";
  }
  else
  {
    o << " top='" << shortenComment(m->comments.at(0)) << "' right:'"
      << shortenComment(m->comments.at(1)) << "'. ";
  }

  o << " (line,col):(" << p.marks.line << "," << p.marks.column << ") ";

  // Dispatch:
  if (p.isScalar())
  {
    o << "Value: '";
    internalPrintAsYAML(std::get<scalar_t>(p.d), o, {}, {});
    o << "'\n";
    return;
  }
  if (p.isNullNode())
  {
    o << "null.\n";
    return;
  }

  o << "\n";

  indent += 2;

  if (p.isMap())
  {
    const std::string sInd2(indent, ' ');
    const auto& m = std::get<map_t>(p.d);
    for (const auto& kv : m)
    {
      o << sInd2 << "- Key:\n";
      internalPrintDebugStructure(kv.first, o, indent + 2);
      o << sInd2 << "- Value:\n";
      internalPrintDebugStructure(kv.second, o, indent + 2);
    }
    return;
  }

  if (p.isSequence())
  {
    const auto& s = std::get<sequence_t>(p.d);
    for (const auto& element : s)
    {
      internalPrintDebugStructure(element, o, indent);
    }
    return;
  }

  THROW_EXCEPTION("Should never reach here");
}

namespace
{
void internalPrintRightComment(std::ostream& o, const std::string& c)
{
  if (c.find('\n') == std::string::npos)
  {
    o << "  # " << c << "\n";
    return;
  }
  // We don't handle multiline right comment at present:
  auto s = c;
  s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
  o << "  # " << s << "\n";
}
}  // namespace

bool yaml::internalPrintAsYAML(
    const std::monostate&, std::ostream& o, const InternalPrintState& ps, const comments_t& cs)
{
  if (ps.needsSpace)
  {
    o << " ";
  }

  o << "~";

  if (const auto& rc = cs[static_cast<size_t>(CommentPosition::RIGHT)]; rc.has_value())
  {
    internalPrintRightComment(o, rc.value());
    return true;  // \n already emitted.
  }

  return false;  // \n not emitted.
}
bool yaml::internalPrintAsYAML(
    const yaml::sequence_t& v,
    std::ostream& o,
    const InternalPrintState& ps,
    [[maybe_unused]] const comments_t& cs)
{
  // Only print sequence in short format if all entries are scalars:
  bool doUseShortFormat = ps.shortFormat;
  if (doUseShortFormat)
  {
    for (const auto& e : v)
      if (!e.isScalar() && !e.isNullNode())
      {
        std::cerr << "\nSkip short format for: " << e.typeName() << "\n";
        doUseShortFormat = false;
        break;
      }
  }

  if (doUseShortFormat)
  {
    o << "[";
    bool first = true;
    for (const auto& e : v)
    {
      if (!first) o << ", ";

      auto ps2 = ps;
      ps2.needsNL = false;
      ps2.needsSpace = false;
      internalPrintNodeAsYAML(e, o, ps2);
      first = false;
    }
    o << "]\n";
  }
  else
  {
    const std::string sInd(ps.indent, ' ');
    for (const auto& e : v)
    {
      o << sInd << "-";
      auto ps2 = ps;
      ps2.needsNL = true;
      ps2.needsSpace = true;
      if (!internalPrintNodeAsYAML(e, o, ps2)) o << "\n";
    }
  }
  return true;
}
bool yaml::internalPrintAsYAML(
    const yaml::map_t& m,
    std::ostream& o,
    const InternalPrintState& ps,
    [[maybe_unused]] const comments_t& cs)
{
  const std::string sInd(ps.indent, ' ');
  for (const auto& kv : m)
  {
    o << sInd;
    auto ps2 = ps;
    ps2.needsSpace = false;
    internalPrintNodeAsYAML(kv.first, o, ps2);
    o << ":";
    const node_t& v = kv.second;

    ps2 = ps;
    ps2.needsNL = true;
    ps2.needsSpace = true;
    bool r = internalPrintNodeAsYAML(v, o, ps2);

    if (!r)
    {
      o << "\n";
    }
  }
  return true;
}

// Return true if multiline
bool yaml::internalPrintStringScalar(
    const std::string& sIn, std::ostream& o, const InternalPrintState& ps, const comments_t& cs)
{
  const std::string sInd(ps.indent + 2, ' ');

  // representation of empty *string*
  const thread_local std::string emptyStr = "''";
  const std::string& s = sIn.empty() ? emptyStr : sIn;

  const bool hasFinalNL = !s.empty() && s.back() == '\n';

  // Split line by line:
  for (size_t i = 0; i < s.size();)
  {
    size_t nextLN = s.find('\n', i);
    if (nextLN == std::string::npos)
    {
      if (i == 0)
      {
        // single line:
        o << s;
        if (const auto& rc = cs[static_cast<size_t>(CommentPosition::RIGHT)]; rc.has_value())
        {
          internalPrintRightComment(o, rc.value());
          return true;  // \n already emitted
        }
        return false;  // \n not emitted
      }

      nextLN = s.size();
    }

    if (i == 0)
    {
      o << "|";
      if (!hasFinalNL)
      {
        o << "-";
      }

      if (const auto& rc = cs[static_cast<size_t>(CommentPosition::RIGHT)]; rc.has_value())
      {
        internalPrintRightComment(o, rc.value());
      }
      else
      {
        o << "\n";
      }
    }
    // Indent + string line:
    o << sInd;

    const size_t lineLen = nextLN - i;
    std::string_view line(s.data() + i, lineLen);
    o << line << "\n";

    i += lineLen + 1;
  }
  return true;
}

bool yaml::internalPrintAsYAML(
    const yaml::scalar_t& v, std::ostream& o, const InternalPrintState& ps, const comments_t& cs)
{
  // Null scalar:
  if (std::holds_alternative<std::monostate>(v))
    return internalPrintAsYAML(std::monostate{}, o, ps, cs);

  // Nested yaml document:
  if (const auto* yp = std::get_if<std::shared_ptr<yaml>>(&v); yp != nullptr)
    return internalPrintNodeAsYAML(*(*yp)->dereferenceProxy(), o, ps);

  if (ps.needsSpace) o << " ";

  bool stringHandledNL = false;
  std::visit(
      [&](const auto& val)
      {
        using V = std::decay_t<decltype(val)>;
        if constexpr (std::is_same_v<V, std::monostate> || std::is_same_v<V, std::shared_ptr<yaml>>)
        {
          // handled above
        }
        else if constexpr (std::is_same_v<V, bool>)
        {
          o << (val ? "true" : "false");
        }
        else if constexpr (std::is_same_v<V, int64_t> || std::is_same_v<V, uint64_t>)
        {
          o << val;
        }
        else if constexpr (std::is_same_v<V, double>)
        {
          const std::string s = mrpt::format("%.16g", val);
          o << s;
          if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
              s.find('E') == std::string::npos && s.find('n') == std::string::npos &&
              s.find('i') == std::string::npos && s.find('I') == std::string::npos)
            o << ".0";
        }
        else if constexpr (std::is_same_v<V, std::string>)
        {
          stringHandledNL = internalPrintStringScalar(val, o, ps, cs);
        }
      },
      v);

  if (stringHandledNL) return true;

  if (const auto& rc = cs[static_cast<size_t>(CommentPosition::RIGHT)]; rc.has_value())
  {
    internalPrintRightComment(o, rc.value());
    return true;
  }
  return false;
}

yaml yaml::FromText(const std::string& yamlTextBlock)
{
  MRPT_START
  yaml doc;
  doc.loadFromText(yamlTextBlock);
  return doc;
  MRPT_END
}

namespace
{
yaml::scalar_t textToScalar(const std::string& s)
{
  // tag:yaml.org,2002:null
  if (s == "~" || s == "null" || s == "Null" || s == "NULL") return {};

  // tag:yaml.org,2002:bool (YAML 1.1 aliases accepted)
  if (s == "true" || s == "True" || s == "TRUE" || s == "yes" || s == "Yes" || s == "YES" ||
      s == "on" || s == "On" || s == "ON")
    return yaml::scalar_t(true);
  if (s == "false" || s == "False" || s == "FALSE" || s == "no" || s == "No" || s == "NO" ||
      s == "off" || s == "Off" || s == "OFF")
    return yaml::scalar_t(false);

  // tag:yaml.org,2002:int — try signed first, then unsigned for large values
  // Reject leading zeros (would imply octal in YAML 1.1; YAML 1.2 forbids them)
  if (!s.empty() && (std::isdigit(static_cast<unsigned char>(s[0])) || s[0] == '-' || s[0] == '+'))
  {
    const bool hasHexPrefix = (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ||
                              (s.size() > 3 && (s[0] == '+' || s[0] == '-') && s[1] == '0' &&
                               (s[2] == 'x' || s[2] == 'X'));
    const std::size_t signLen = (s[0] == '+' || s[0] == '-') ? 1u : 0u;
    const bool leadingZeroOctal = !hasHexPrefix && s.size() > signLen + 1 && s[signLen] == '0';
    if (!leadingZeroOctal)
    {
      char* end = nullptr;
      errno = 0;
      const long long iv = std::strtoll(s.c_str(), &end, 0);
      if (end != nullptr && end != s.c_str() && *end == '\0' && errno != ERANGE)
        return yaml::scalar_t(static_cast<int64_t>(iv));

      // Large positive — try unsigned
      errno = 0;
      const unsigned long long uv = std::strtoull(s.c_str(), &end, 0);
      if (end != nullptr && end != s.c_str() && *end == '\0' && errno != ERANGE)
        return yaml::scalar_t(static_cast<uint64_t>(uv));
    }
  }

  // tag:yaml.org,2002:float
  if (!s.empty())
  {
    char* end = nullptr;
    errno = 0;
    const double dv = std::strtod(s.c_str(), &end);
    if (end != nullptr && end != s.c_str() && *end == '\0' && errno != ERANGE)
      return yaml::scalar_t(dv);
  }

  return yaml::scalar_t(s);
}
}  // namespace

#if MRPT_HAS_FYAML
namespace
{
std::optional<yaml::node_t> recursiveParse(struct fy_parser* p);
}
static bool MRPT_YAML_PARSER_VERBOSE = mrpt::get_env<bool>("MRPT_YAML_PARSER_VERBOSE", false);

#define PARSER_DBG_OUT(STR_)                                                      \
  do                                                                              \
  {                                                                               \
    if (MRPT_YAML_PARSER_VERBOSE)                                                 \
      std::cout << ">> " << STR_ << "\n"; /* NOLINT(bugprone-macro-parentheses)*/ \
    else                                                                          \
    {                                                                             \
    }                                                                             \
  } while (0)

namespace
{

std::optional<std::string> extractComment(struct fy_token* t, enum fy_comment_placement cp)
{
#if MRPT_LIBFYAML_VERSION >= 0x000904  // 0.9.4
  const char* strRet = fy_token_get_comment(t, cp);
#else
  std::array<char, 2048> str = {};
  const char* strRet = fy_token_get_comment(t, str.data(), str.size(), cp);
#endif

  if (strRet == nullptr || strRet[0] == '\0')
  {
    return {};
  }

  // str is already a 0-terminated string:
  auto c = std::string(strRet);

  PARSER_DBG_OUT(
      "token: " << reinterpret_cast<void*>(t) << " comment [" << static_cast<int>(cp) << "]: '" << c
                << "'");

  return c;
}

void parseTokenCommentsAndMarks(struct fy_token* tk, yaml::node_t& n)
{
  if (tk == nullptr) return;

  if (auto cT = extractComment(tk, fycp_top); cT)
    n.commentSlot(CommentPosition::TOP).emplace(std::move(cT.value()));

  if (auto cR = extractComment(tk, fycp_right); cR)
    n.commentSlot(CommentPosition::RIGHT).emplace(std::move(cR.value()));

  if (auto cB = extractComment(tk, fycp_bottom); cB)
    n.commentSlot(CommentPosition::BOTTOM).emplace(std::move(cB.value()));

  if (const struct fy_mark* mrk = fy_token_start_mark(tk); mrk)
  {
    n.marks.column = mrk->column;
    n.marks.line = mrk->line;
    n.marks.input_pos = mrk->input_pos;
  }
}

std::optional<yaml::node_t> recursiveParse(struct fy_parser* p)
{
  MRPT_START

  struct fy_event* event = fy_parser_parse(p);
  if (!event)
  {
    return {};
  }

  // process event:
  switch (event->type)
  {
    case FYET_NONE:
    {
      PARSER_DBG_OUT("Event: None");
      fy_parser_event_free(p, event);  // free event
      return recursiveParse(p);        // Keep going
    }
    break;
    case FYET_STREAM_START:
    {
      PARSER_DBG_OUT("Event: Stream start");
      fy_parser_event_free(p, event);  // free event
      return recursiveParse(p);        // Keep going
    }
    break;
    case FYET_STREAM_END:
    {
      PARSER_DBG_OUT("Event: Stream end");
      fy_parser_event_free(p, event);  // free event
      return recursiveParse(p);        // Keep going
    }
    break;
    case FYET_DOCUMENT_START:
    {
      PARSER_DBG_OUT("Event: Doc start");
      fy_parser_event_free(p, event);  // free event
      return recursiveParse(p);        // Keep going
    }
    break;
    case FYET_DOCUMENT_END:
    {
      PARSER_DBG_OUT("Event: Doc end");
      fy_parser_event_free(p, event);  // free event
      return recursiveParse(p);        // Keep going
    }
    break;
    case FYET_MAPPING_START:
    {
      PARSER_DBG_OUT("Event: MAP START");
      fy_parser_event_free(p, event);  // free event

      yaml::node_t n;
      yaml::map_t& m = n.d.emplace<yaml::map_t>();

      parseTokenCommentsAndMarks(event->scalar.value, n);

      for (;;)
      {
        // Next event is map key:
        auto nKey = recursiveParse(p);
        // end of map reached?
        if (!nKey.has_value())
        {
          break;
        }

        ASSERT_(nKey->isScalar());

        // Insert key (carrying parse-time comments) and a placeholder value:
        m.emplace_back(std::move(*nKey), yaml::node_t{});
        yaml::node_t& val = m.back().second;

        // and next event is mapped content:
        auto nVal = recursiveParse(p);
        ASSERT_(nVal.has_value());

        val = std::move(nVal.value());
      }

      return n;
    }
    break;
    case FYET_MAPPING_END:
    {
      PARSER_DBG_OUT("Event: MAP END");
      fy_parser_event_free(p, event);  // free event
      return {};
    }
    break;
    case FYET_SEQUENCE_START:
    {
      PARSER_DBG_OUT("Event: SEQ START");
      fy_parser_event_free(p, event);  // free event
      yaml::node_t n;
      yaml::sequence_t& s = n.d.emplace<yaml::sequence_t>();

      parseTokenCommentsAndMarks(event->scalar.value, n);

      for (;;)
      {
        auto entry = recursiveParse(p);
        if (!entry.has_value())
        {
          break;  // end of sequence reached?
        }
        s.push_back(std::move(entry.value()));
      }

      return n;
    }
    break;
    case FYET_SEQUENCE_END:
    {
      PARSER_DBG_OUT("Event: SEQ END");
      fy_parser_event_free(p, event);  // free event
      return {};
    }
    break;
    case FYET_SCALAR:
    {
      size_t strValueLen = 0;
      const char* strValue = fy_token_get_text(event->scalar.value, &strValueLen);
      const std::string sValue(strValue, strValueLen);

      PARSER_DBG_OUT(
          "token: " << reinterpret_cast<void*>(event->scalar.value)
                    << " Scalar: implicit=" << (event->scalar.tag_implicit ? "1" : "0")
                    << " tag: " << static_cast<void*>(event->scalar.tag)
                    << " anchor: " << static_cast<void*>(event->scalar.anchor)
                    << fy_token_get_text0(event->scalar.anchor) << " value: " << sValue);

      if (event->scalar.value)
      {
        yaml::node_t n;
        n.d.emplace<yaml::scalar_t>(textToScalar(sValue));

        parseTokenCommentsAndMarks(event->scalar.value, n);

        fy_parser_event_free(p, event);  // free event
        return n;
      }

      THROW_EXCEPTION(
          "Unexpected empty scalar?! Re-run with environment "
          "variable MRPT_YAML_PARSER_VERBOSE=1 to get more details.");
    }
    break;
    case FYET_ALIAS:
      fy_parser_event_free(p, event);  // free event
      return recursiveParse(p);        // Keep going
  };

  THROW_EXCEPTION_FMT("Unexpected parser event type %i", event->type);

#undef PARSER_DBG_OUT
  MRPT_END
}
}  // namespace
#endif

void yaml::loadFromText(const std::string& yamlTextBlock)
{
  MRPT_START
#if MRPT_HAS_FYAML

  // Reset:
  *this = yaml();

  struct fy_parse_cfg cfg
  {
  };
  cfg.search_path = "";
  cfg.diag = nullptr;
  cfg.flags = FYPCF_PARSE_COMMENTS;

  struct fy_parser* parser = fy_parser_create(&cfg);
  ASSERT_(parser);

  if (fy_parser_set_string(parser, yamlTextBlock.data(), yamlTextBlock.size()))
    THROW_EXCEPTION("Error in fy_parser_set_string()");

  auto optNode = recursiveParse(parser);
  if (optNode.has_value()) root_ = std::move(optNode.value());

  fy_parser_destroy(parser);

#else
  THROW_EXCEPTION("MRPT was built without libfyaml");
#endif

  MRPT_END
}

yaml yaml::FromStream(std::istream& i)
{
  MRPT_START
  yaml doc;
  doc.loadFromStream(i);
  return doc;
  MRPT_END
}

// Replicated from mrpt::io to avoid dependency to that module:
namespace
{
std::string local_file_get_contents(const std::string& fileName)
{
  // Credits: https://stackoverflow.com/a/2602258/1631514
  // Note: Add "binary" to make sure the "tellg" file size matches the
  // actual number of read bytes afterwards:
  std::ifstream t(fileName, std::ios::binary);
  if (!t.is_open())
    THROW_EXCEPTION_FMT("file_get_contents(): Error opening for read file `%s`", fileName.c_str());

  t.seekg(0, std::ios::end);
  const auto size = t.tellg();
  std::string buffer(static_cast<size_t>(size), ' ');
  t.seekg(0);
  t.read(&buffer[0], size);
  return buffer;
}
}  // namespace

void yaml::loadFromFile(const std::string& fileName)
{
  MRPT_START
  clear();
  this->loadFromText(local_file_get_contents(fileName));
  MRPT_END
}

yaml yaml::FromFile(const std::string& fileName)
{
  MRPT_START
  yaml doc;
  doc.loadFromFile(fileName);
  return doc;
  MRPT_END
}

void yaml::loadFromStream(std::istream& i)
{
  MRPT_START
  std::string str;

  i.seekg(0, std::ios::end);
  str.reserve(static_cast<std::size_t>(i.tellg()));
  i.seekg(0, std::ios::beg);
  str.assign((std::istreambuf_iterator<char>(i)), std::istreambuf_iterator<char>());

  this->loadFromText(str);
  MRPT_END
}

// ============ class: yaml_ref / yaml_cref =======

namespace
{
yaml_ref mapFindOrCreate(yaml::node_t& node, const std::string& key)
{
  if (node.isNullNode()) node.d.emplace<yaml::map_t>();
  ASSERTMSG_(node.isMap(), "operator[] requires a map node");
  yaml::map_t& m = std::get<yaml::map_t>(node.d);
  auto it = std::find_if(
      m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
  if (it == m.end())
  {
    m.emplace_back(yaml::node_t(key), yaml::node_t{});
    return yaml_ref(m.back().second);
  }
  return yaml_ref(it->second);
}

yaml_cref mapFind(const yaml::node_t& node, const std::string& key)
{
  ASSERTMSG_(!node.isNullNode(), "read operator[] not applicable to null nodes.");
  ASSERTMSG_(node.isMap(), "read operator[] only available for map nodes.");
  const yaml::map_t& m = std::get<yaml::map_t>(node.d);
  auto it = std::find_if(
      m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
  if (it == m.end()) THROW_EXCEPTION_FMT("Access non-existing map key `%s`", key.c_str());
  return yaml_cref(it->second);
}

yaml_ref seqAt(yaml::node_t& node, int index)
{
  ASSERTMSG_(node.isSequence(), "write operator[](int) only available for sequence nodes.");
  yaml::sequence_t& seq = std::get<yaml::sequence_t>(node.d);
  if (index < 0 || index >= static_cast<int>(seq.size()))
    THROW_TYPED_EXCEPTION("yaml_ref::operator[](int) out of range", std::out_of_range);
  return yaml_ref(seq.at(static_cast<std::size_t>(index)));
}

yaml_cref seqAtConst(const yaml::node_t& node, int index)
{
  ASSERTMSG_(node.isSequence(), "read operator[](int) only available for sequence nodes.");
  const yaml::sequence_t& seq = std::get<yaml::sequence_t>(node.d);
  if (index < 0 || index >= static_cast<int>(seq.size()))
    THROW_TYPED_EXCEPTION("yaml_cref::operator[](int) out of range", std::out_of_range);
  return yaml_cref(seq.at(static_cast<std::size_t>(index)));
}
}  // namespace

yaml_ref yaml_ref::operator[](const std::string& key) { return mapFindOrCreate(*node_, key); }
yaml_ref yaml_ref::operator[](const char* key)
{
  ASSERT_(key != nullptr);
  return mapFindOrCreate(*node_, std::string(key));
}
yaml_cref yaml_ref::operator[](const std::string& key) const { return mapFind(*node_, key); }
yaml_cref yaml_ref::operator[](const char* key) const
{
  ASSERT_(key != nullptr);
  return mapFind(*node_, std::string(key));
}
yaml_ref yaml_ref::operator[](int index) { return seqAt(*node_, index); }
yaml_cref yaml_ref::operator[](int index) const { return seqAtConst(*node_, index); }

yaml_cref yaml_cref::operator[](const std::string& key) const { return mapFind(*node_, key); }
yaml_cref yaml_cref::operator[](const char* key) const
{
  ASSERT_(key != nullptr);
  return mapFind(*node_, std::string(key));
}
yaml_cref yaml_cref::operator[](int index) const { return seqAtConst(*node_, index); }

std::ostream& mrpt::containers::operator<<(std::ostream& o, const yaml_ref& p)
{
  return o << static_cast<yaml>(p);
}
std::ostream& mrpt::containers::operator<<(std::ostream& o, const yaml_cref& p)
{
  return o << static_cast<yaml>(p);
}

std::ostream& mrpt::containers::operator<<(std::ostream& o, const yaml& p)
{
  YamlEmitOptions eo;
  eo.emitHeader = false;
  eo.endWithNewLine = false;

  p.printAsYAML(o, eo);
  return o;
}

// --- leaf node comments API ---
bool yaml::hasComment() const { return dereferenceProxy()->hasComment(); }

bool yaml::hasComment(CommentPosition pos) const { return dereferenceProxy()->hasComment(pos); }

const std::string& yaml::comment() const { return dereferenceProxy()->comment(); }

const std::string& yaml::comment(CommentPosition pos) const
{
  return dereferenceProxy()->comment(pos);
}

void yaml::comment(const std::string& c, CommentPosition position)
{
  node_t* n = dereferenceProxy();
  n->commentSlot(position).emplace(c);
}

// --- key node comments API ---
const yaml::node_t& findKeyNode(const yaml::node_t* me, const std::string& key)
{
  const auto& m = me->asMap();
  const auto itK = std::find_if(
      m.begin(), m.end(), [&key](const auto& kv) { return kv.first.internalAsStr() == key; });
  ASSERTMSG_(
      itK != m.end(),
      mrpt::format("key '%.*s' not present in map", static_cast<int>(key.size()), key.data()));
  return itK->first;
}

bool yaml::keyHasComment(const std::string& key) const
{
  MRPT_START
  return findKeyNode(dereferenceProxy(), key).hasComment();
  MRPT_END
}

bool yaml::keyHasComment(const std::string& key, CommentPosition pos) const
{
  MRPT_START
  return findKeyNode(dereferenceProxy(), key).hasComment(pos);
  MRPT_END
}

const std::string& yaml::keyComment(const std::string& key) const
{
  MRPT_START
  return findKeyNode(dereferenceProxy(), key).comment();
  MRPT_END
}

const std::string& yaml::keyComment(const std::string& key, CommentPosition pos) const
{
  MRPT_START
  return findKeyNode(dereferenceProxy(), key).comment(pos);
  MRPT_END
}

void yaml::keyComment(const std::string& key, const std::string& c, CommentPosition position)
{
  auto& n = const_cast<node_t&>(findKeyNode(dereferenceProxy(), key));
  n.commentSlot(position).emplace(c);
}

const yaml::node_t& yaml::keyNode(const std::string& keyName) const
{
  MRPT_START
  const yaml::node_t& n = findKeyNode(dereferenceProxy(), keyName);
  return n;
  MRPT_END
}
yaml::node_t& yaml::keyNode(const std::string& keyName)
{
  const yaml::node_t& n = findKeyNode(dereferenceProxy(), keyName);
  return const_cast<yaml::node_t&>(n);
}
