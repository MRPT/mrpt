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

#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/serialization/CSchemeArchive.h>

#include <algorithm>
#include <istream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace mrpt::serialization;

namespace
{
using mrpt::containers::yaml;
using PathStep = std::variant<std::string, size_t>;

/** Schema archive backed by a mrpt::containers::yaml document.
 * Child archives keep the path from the shared root instead of a reference to
 * their node: yaml containers are vectors, so adding siblings may move nodes.
 */
class CSchemeArchiveYAML : public CSchemeArchiveBase_impl
{
 public:
  CSchemeArchiveYAML() : m_root(std::make_shared<yaml>()) {}
  CSchemeArchiveYAML(std::shared_ptr<yaml> root, std::vector<PathStep> path) :
      m_root(std::move(root)), m_path(std::move(path))
  {
  }

  CSchemeArchiveBase& operator=(const int32_t val) override { return set(val); }
  CSchemeArchiveBase& operator=(const uint32_t val) override { return set(val); }
  CSchemeArchiveBase& operator=(const int64_t val) override { return set(val); }
  CSchemeArchiveBase& operator=(const uint64_t val) override { return set(val); }
  CSchemeArchiveBase& operator=(const float val) override { return set(val); }
  CSchemeArchiveBase& operator=(const double val) override { return set(val); }
  CSchemeArchiveBase& operator=(const std::string val) override { return set(val); }
  CSchemeArchiveBase& operator=(const bool val) override { return set(val); }

  explicit operator int32_t() const override { return get<int32_t>(); }
  explicit operator uint32_t() const override { return get<uint32_t>(); }
  explicit operator int64_t() const override { return get<int64_t>(); }
  explicit operator uint64_t() const override { return get<uint64_t>(); }
  explicit operator float() const override { return get<float>(); }
  explicit operator double() const override { return get<double>(); }
  explicit operator bool() const override { return get<bool>(); }
  explicit operator std::string() const override { return get<std::string>(); }

  CSchemeArchiveBase& operator=(const mrpt::serialization::CSerializable& obj) override
  {
    ReadObject(*m_parent, obj);
    return *m_parent;
  }
  void readTo(mrpt::serialization::CSerializable& obj) override { WriteObject(*m_parent, obj); }

  CSchemeArchiveBase operator[](size_t idx) override { return child(PathStep(idx)); }
  CSchemeArchiveBase operator[](std::string key) override
  {
    return child(PathStep(std::move(key)));
  }

  std::ostream& writeToStream(std::ostream& out) const override
  {
    if (m_path.empty())
    {
      m_root->printAsJSON(out);
      return out;
    }
    yaml sub;
    if (const auto* n = find(); n != nullptr)
    {
      sub.node() = *n;
    }
    sub.printAsJSON(out);
    return out;
  }

  std::istream& readFromStream(std::istream& in) override
  {
    auto doc = yaml::FromStream(in);
    resolve() = std::move(doc.node());
    return in;
  }

 private:
  std::shared_ptr<yaml> m_root;
  std::vector<PathStep> m_path;

  template <typename T>
  CSchemeArchiveBase& set(const T& val)
  {
    resolve() = yaml::node_t(val);
    return *m_parent;
  }

  /** Missing or null entries read as a default-constructed value. */
  template <typename T>
  T get() const
  {
    const auto* n = find();
    if (n == nullptr || n->isNullNode())
    {
      return T{};
    }
    return n->as<T>();
  }

  CSchemeArchiveBase child(PathStep step)
  {
    auto path = m_path;
    path.push_back(std::move(step));
    return CSchemeArchiveBase(std::make_unique<CSchemeArchiveYAML>(m_root, std::move(path)));
  }

  static bool keyMatches(const yaml::node_t& keyNode, const std::string& key)
  {
    return keyNode.isScalar() && keyNode.as<std::string>() == key;
  }

  /** Returns the node at m_path, or nullptr if it does not exist. */
  [[nodiscard]] const yaml::node_t* find() const
  {
    const yaml::node_t* n = &m_root->node();
    for (const auto& step : m_path)
    {
      if (const auto* key = std::get_if<std::string>(&step); key != nullptr)
      {
        if (!n->isMap())
        {
          return nullptr;
        }
        const auto& m = n->asMap();
        const auto it = std::find_if(
            m.begin(), m.end(), [&](const auto& kv) { return keyMatches(kv.first, *key); });
        if (it == m.end())
        {
          return nullptr;
        }
        n = &it->second;
      }
      else
      {
        const size_t idx = std::get<size_t>(step);
        if (!n->isSequence() || idx >= n->asSequence().size())
        {
          return nullptr;
        }
        n = &n->asSequence()[idx];
      }
    }
    return n;
  }

  /** Returns the node at m_path, creating it (and its parents) if needed. */
  yaml::node_t& resolve()
  {
    yaml::node_t* n = &m_root->node();
    for (const auto& step : m_path)
    {
      if (const auto* key = std::get_if<std::string>(&step); key != nullptr)
      {
        if (n->isNullNode())
        {
          n->d.emplace<yaml::map_t>();
        }
        auto& m = n->asMap();
        auto it = std::find_if(
            m.begin(), m.end(), [&](const auto& kv) { return keyMatches(kv.first, *key); });
        if (it == m.end())
        {
          m.emplace_back(yaml::node_t(*key), yaml::node_t());
          n = &m.back().second;
        }
        else
        {
          n = &it->second;
        }
      }
      else
      {
        const size_t idx = std::get<size_t>(step);
        if (n->isNullNode())
        {
          n->d.emplace<yaml::sequence_t>();
        }
        auto& seq = n->asSequence();
        if (idx >= seq.max_size())
        {
          THROW_EXCEPTION_FMT("Sequence index out of range: %zu", idx);
        }
        if (idx >= seq.size())
        {
          seq.resize(idx + 1);
        }
        n = &seq[idx];
      }
    }
    return *n;
  }
};
}  // namespace

CSchemeArchiveBase mrpt::serialization::archiveJSON()
{
  return CSchemeArchiveBase(std::make_unique<CSchemeArchiveYAML>());
}
