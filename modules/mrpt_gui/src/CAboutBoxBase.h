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
#include <string>

class CAboutBoxBase
{
 public:
  CAboutBoxBase(
      const std::string& appName, const std::string& additionalInfo, const bool showStandardInfo);
  virtual ~CAboutBoxBase();

 protected:
  std::string MRPTBuildVersion() const;
  std::string tutorial() const;
  std::string license() const;
  std::string information(const std::string& guiLibName, const std::string& guiLibVersion) const;

  const std::string m_appName;
  const std::string m_additionalInfo;
  const bool m_showStandardInfo;
};
