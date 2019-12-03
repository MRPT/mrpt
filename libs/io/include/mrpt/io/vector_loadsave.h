/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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
 * \sa file_get_contents()
 */
bool loadTextFile(std::vector<std::string>& o, const std::string& fileName);

/** Loads an entire text file and return its contents as a single std::string.
 * \exception std::runtime_error On any read error.
 * \sa loadBinaryFile(), loadTextFile()
 * \note Relying on C++17 RVO to return a string without worring on
 * return-by-value of big objects.
 */
std::string file_get_contents(const std::string& fileName);

/** @} */

}  // namespace mrpt::io
