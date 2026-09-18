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

#include <mrpt/containers/deepcopy_poly_ptr.h>

#include <cstddef>
#include <iterator>
#include <memory>
#include <type_traits>

namespace mrpt::containers
{
namespace internal
{
/** @name Access to the smart pointer stored in a container element
 * @{ */
template <typename T>
const std::shared_ptr<T>& element_as_shared_ptr(const std::shared_ptr<T>& p)
{
  return p;
}
template <typename T>
const T& element_as_shared_ptr(const deepcopy_poly_ptr<T>& p)
{
  return p.get_ptr();
}
/** @} */
}  // namespace internal

/** Adapts a const_iterator of a container of smart pointers so that
 * dereferencing it yields a `shared_ptr<const T>` instead of a
 * `shared_ptr<T>`.
 *
 * Containers of `X::Ptr` would otherwise let a `const` container hand out
 * mutable pointees, defeating const-correctness. Note that, as with any proxy
 * iterator, `operator*` returns by value, so use `(*it)->method()` and not
 * `it->method()`.
 *
 * \ingroup mrpt_containers_grp
 */
template <typename UNDERLYING_IT>
class deep_const_iterator
{
 private:
  using underlying_ptr_t =
      std::decay_t<decltype(internal::element_as_shared_ptr(*std::declval<UNDERLYING_IT>()))>;

 public:
  using element_type = typename underlying_ptr_t::element_type;
  using value_type = std::shared_ptr<const element_type>;
  using reference = value_type;
  using pointer = void;
  using difference_type = std::ptrdiff_t;
  using iterator_category = std::bidirectional_iterator_tag;

  deep_const_iterator() = default;
  deep_const_iterator(const UNDERLYING_IT& it) : m_it(it) {}

  [[nodiscard]] value_type operator*() const { return internal::element_as_shared_ptr(*m_it); }

  deep_const_iterator& operator++()
  {
    ++m_it;
    return *this;
  }
  deep_const_iterator operator++(int)
  {
    auto aux = *this;
    ++m_it;
    return aux;
  }
  deep_const_iterator& operator--()
  {
    --m_it;
    return *this;
  }
  deep_const_iterator operator--(int)
  {
    auto aux = *this;
    --m_it;
    return aux;
  }

  [[nodiscard]] bool operator==(const deep_const_iterator& o) const { return m_it == o.m_it; }
  [[nodiscard]] bool operator!=(const deep_const_iterator& o) const { return m_it != o.m_it; }

  /** Returns the wrapped underlying iterator */
  [[nodiscard]] const UNDERLYING_IT& underlying() const { return m_it; }

 private:
  UNDERLYING_IT m_it;
};

}  // namespace mrpt::containers
