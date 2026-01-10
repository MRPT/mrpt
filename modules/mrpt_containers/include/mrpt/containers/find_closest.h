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

#include <cmath>  // std::abs
#include <cstdlib>
#include <limits>
#include <optional>

namespace mrpt::containers
{
/** \addtogroup mrpt_containers_find_closest Find closest utility functions
 *  \ingroup mrpt_containers_grp
 * @{ */

/** For an associate container `Container` mapping real number keys to `T`
 * values, searchs for the closest key within a given tolerance, that is,
 * the key closest to `x` within the interval [x-tolerace, x+tolerance].
 * An empty `std::optional` is returned if none is found.
 *
 * Computational cost: `O(log(N)+M)` with `N` the total size of the container,
 * `M` the worst-case number of items in any interval of width `2*tolerance`.
 *
 * \sa find_closest()
 * \note (New in MRPT 2.5.0)
 */
template <typename Container>
std::optional<std::pair<typename Container::key_type, typename Container::mapped_type>>
find_closest_with_tolerance(
    const Container& data,
    const typename Container::key_type x,
    const typename Container::key_type tolerance)
{
  const auto t_min = x - tolerance;
  const auto t_max = x + tolerance;

  auto it_lo = data.lower_bound(t_min);
  auto it_hi = data.upper_bound(t_max);

  auto min_distance = std::numeric_limits<typename Container::key_type>::max();
  std::optional<std::pair<typename Container::key_type, typename Container::mapped_type>> best;

  for (auto it = it_lo; it != it_hi; ++it)
  {
    if (it == data.end())
    {
      continue;
    }
    const auto dist = std::abs(it->first - x);
    if (dist < min_distance)
    {
      min_distance = dist;
      best = {it->first, it->second};
    }
  }

  return best;
}

/** For an associate container `Container` mapping real number keys to `T`
 * values, searchs for the closest key within a given tolerance, that is,
 * the key closest to `x` within the interval [x-tolerace, x+tolerance].
 *
 * An empty `std::optional` is returned if none is found, i.e. if the container
 * was empty.
 *
 * Computational cost: `O(log(N))` with `N` the total size of the container.
 *
 * \sa find_closest_with_tolerance()
 * \note (New in MRPT 2.5.0)
 */
template <typename Container>
std::optional<std::pair<typename Container::key_type, typename Container::mapped_type>>
find_closest(const Container& data, const typename Container::key_type x)
{
  typename Container::const_iterator its[2] = {data.lower_bound(x), data.upper_bound(x)};

  if (!data.empty() && its[0] != data.begin())
  {
    --its[0];
  }

  auto min_distance = std::numeric_limits<typename Container::key_type>::max();
  std::optional<std::pair<typename Container::key_type, typename Container::mapped_type>> best;

  for (const auto& it : its)
  {
    if (it == data.end())
    {
      continue;
    }
    const auto dist = std::abs(it->first - x);
    if (dist < min_distance)
    {
      min_distance = dist;
      best = {it->first, it->second};
    }
  }

  return best;
}
/**  @} */

}  // namespace mrpt::containers
