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

namespace mrpt::io
{
/** @defgroup vector_loadsave Load and save vectors to files (in #include
 * <mrpt/io/vector_loadsave.h>)
 * \ingroup mrpt_io_grp
 * @{ */

/** Saves a vector directly as a binary dump to a file:
 * \return Returns false on any error, true on everything OK.
 * \sa loadBinaryFile
 */
bool vectorToBinaryFile(
	const std::vector<uint8_t>& vec, const std::string& fileName);

/** Loads a entire file as a vector of bytes.
 * \return Returns false on any error, true on everything OK.
 * \sa vectorToBinaryFile
 */
bool loadBinaryFile(
	std::vector<uint8_t>& out_data, const std::string& fileName);

/** Loads a text file as a vector of string lines.
 * \return Returns false on any error, true on everything OK.
 */
bool loadTextFile(std::vector<std::string>& o, const std::string& fileName);
/** @} */

}  // namespace mrpt::io
