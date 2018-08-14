/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>

namespace mrpt::system
{
/** @defgroup vector_loadsave Load and save vectors to files
 * Header: `#include <mrpt/system/vector_loadsave.h>`.
 * Library: \ref mrpt_system_grp
 * \ingroup mrpt_system_grp
 * @{ */

/** A useful function for debugging, which saves a std::vector into a text file
 * (compat. with MATLAB)
 * \return Returns false on any error, true on everything OK.
 */
bool vectorToTextFile(
	const std::vector<float>& vec, const std::string& fileName,
	bool append = false, bool byRows = false);
//! \overload
bool vectorToTextFile(
	const std::vector<double>& vec, const std::string& fileName,
	bool append = false, bool byRows = false);
//! \overload
bool vectorToTextFile(
	const std::vector<int>& vec, const std::string& fileName,
	bool append = false, bool byRows = false);
//! \overload
bool vectorToTextFile(
	const std::vector<size_t>& vec, const std::string& fileName,
	bool append = false, bool byRows = false);
//! \overload
template <class EIGEN_MATRIX>
bool vectorToTextFile(const EIGEN_MATRIX& vec, const std::string& fileName)
{
	try
	{
		vec.saveToTextFile(fileName);
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/** Load a std::vector from a text file (compat. with MATLAB)
 * \return Returns false on any error, true on everything OK.
 * \sa loadBinaryFile
 */
bool vectorFromTextFile(
	std::vector<double>& vec, const std::string& fileName,
	const bool byRows = false);

/** @} */

}  // namespace mrpt::system
