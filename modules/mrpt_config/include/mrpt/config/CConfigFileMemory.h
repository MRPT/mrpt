/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/pimpl.h>

#include <string>
#include <vector>

namespace mrpt::config
{
/** This class implements a config file-like interface over a memory-stored
 * string list.
 *
 * Use base class `CConfigFileBase`'s methods
 * `read_{int,float,double,string,...}()` and `write()` to actually read and
 * write values.
 *
 * It can also parse a YAML text block and expose its fields (up to the first
 * level of hierarchy, as allowed by INI-like files). This can be used to port
 * MRPT classes relying on INI files to using YAML files transparently.
 * This feature required building MRPT with yaml-cpp, and is provided by
 * CConfigFileMemory::setContentFromYAML().
 *
 * See: \ref config_file_format
 *
 * \ingroup mrpt_config_grp
 * \note YAML support was introduced in MRPT 1.9.9
 */
class CConfigFileMemory : public CConfigFileBase
{
 public:
  /** Empty constructor. Upon construction, call any of the "setContent"
   * method */
  CConfigFileMemory();

  CConfigFileMemory(const CConfigFileMemory&) = default;
  CConfigFileMemory& operator=(const CConfigFileMemory&) = default;
  CConfigFileMemory(CConfigFileMemory&&) = default;
  CConfigFileMemory& operator=(CConfigFileMemory&&) = default;

  /** Constructor and initialize from a list of strings */
  CConfigFileMemory(const std::vector<std::string>& stringList);
  /** Constructor and initialize from string with the whole "config file" */
  CConfigFileMemory(const std::string& str);
  /** dtor */
  ~CConfigFileMemory() override;

  /** Changes the contents of the virtual "config file" */
  void setContent(const std::vector<std::string>& stringList);
  /** Changes the contents of the virtual "config file" */
  void setContent(const std::string& str);
  /** Return the current contents of the virtual "config file" */
  std::string getContent() const;

  /** Empties the virtual "config file" */
  void clear() override;

  /** Returns a list with all the section names */
  void getAllSections(std::vector<std::string>& sections) const override;
  /** Returns a list with all the keys into a section */
  void getAllKeys(const std::string& section, std::vector<std::string>& keys) const override;

 private:
  /** The IniFile object */
  struct Impl;
  mrpt::pimpl<Impl> m_impl;

 protected:
  /** A virtual method to write a generic string */
  void writeString(
      const std::string& section, const std::string& name, const std::string& str) override;
  /** A virtual method to read a generic string */
  std::string readString(
      const std::string& section,
      const std::string& name,
      const std::string& defaultStr,
      bool failIfNotFound = false) const override;

};  // End of class def.

}  // namespace mrpt::config
