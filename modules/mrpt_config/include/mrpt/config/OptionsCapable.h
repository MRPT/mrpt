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

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>

#include <map>
#include <string>

namespace mrpt::config
{
/** Mixin interface for classes that expose one or more `CLoadableOptions`
 * structures (e.g. "insertionOptions", "likelihoodOptions", "renderOptions")
 * in a generic, name-indexed way, so that callers never need to know the
 * concrete class nor the exact set/names of options structures it defines --
 * works uniformly for all present and future classes implementing this
 * interface.
 *
 * "Creation options" (those that configure a class' internal structure, e.g.
 * a voxel/grid size) are handled separately via trySetCreationOptions(),
 * since changing them may be incompatible with already-built internal state.
 *
 * \ingroup mrpt_config_grp
 */
class OptionsCapable
{
 public:
  OptionsCapable() = default;
  OptionsCapable(const OptionsCapable&) = default;
  OptionsCapable& operator=(const OptionsCapable&) = default;
  OptionsCapable(OptionsCapable&&) = default;
  OptionsCapable& operator=(OptionsCapable&&) = default;
  virtual ~OptionsCapable() = default;

  /** Maps an options-group name (e.g. "insertionOptions") to a pointer to
   * the corresponding, live `CLoadableOptions` member. Pointers remain valid
   * as long as `this` is alive, and point directly to the actual option
   * members (no copies), so writes through them (e.g. via
   * `loadFromConfigFile()`) take effect immediately.
   *
   * Implementations should list every `CLoadableOptions` member they define,
   * INCLUDING "creationOptions" if present (so it can be discovered/exported
   * generically); however, callers must use trySetCreationOptions(), not a
   * direct write through this pointer, to safely *modify* creation options.
   */
  [[nodiscard]] virtual std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() = 0;

  /** Attempts to apply new "creation options" (i.e. those that configure an
   * internal structure, such as a voxel/grid size), read from the given
   * `section` of `cfg`.
   *
   * Many creation-time parameters are just runtime thresholds with no effect
   * on already-built internal structures, so applying them in place is
   * often possible even after the object holds data. Others (e.g. a voxel
   * size) cannot be changed without discarding existing contents.
   *
   * \return true if the new options were applied; false if doing so would
   * require destroying the object's current contents (in which case it is
   * left unmodified) -- or if this class does not define any
   * "creationOptions" group at all (the default implementation).
   */
  [[nodiscard]] virtual bool trySetCreationOptions(
      [[maybe_unused]] const mrpt::config::CConfigFileBase& cfg,
      [[maybe_unused]] const std::string& section)
  {
    return false;
  }
};

}  // namespace mrpt::config
