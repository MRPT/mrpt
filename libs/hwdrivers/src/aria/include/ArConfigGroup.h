/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARCONFIGGROUP_H
#define ARCONFIGGROUP_H

#include "ArConfig.h"

/// Container for holding a group of ArConfigs
class ArConfigGroup
{
public:
  /// Constructor
  AREXPORT ArConfigGroup(const char *baseDirectory = NULL);
  /// Destructor
  AREXPORT ~ArConfigGroup(void);
  /// Adds a config to the group
  AREXPORT void addConfig(ArConfig *config);
  /// Removes a config from the group
  AREXPORT void remConfig(ArConfig *config);
  /// Parses the given file (starting from the base directory)
  AREXPORT bool parseFile(const char *fileName, bool continueOnError = false);
  /// Reloads the last file parsed
  AREXPORT bool reloadFile(bool continueOnError = true);
  /// Writes a file out (overwrites any existing file)
  AREXPORT bool writeFile(const char *fileName);
  /// Sets the base directory on all configs this contains
  AREXPORT void setBaseDirectory(const char *baseDirectory);
  /// Gets the baes directory of this group (not the configs it contains)
  AREXPORT const char *getBaseDirectory(void) const;
protected:
  std::string myBaseDirectory;
  std::string myLastFile;
  std::list<ArConfig *> myConfigs;
};

#endif // ARCONFIGGROUP
