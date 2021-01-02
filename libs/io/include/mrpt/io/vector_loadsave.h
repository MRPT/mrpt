/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>

namespace mrpt::io
{
// clang-format off
/** @defgroup vector_loadsave Load and save vectors to files (in #include <mrpt/io/vector_loadsave.h>)
 * \ingroup mrpt_io_grp
 * @{ */
// clang-format on

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

/** A useful function for debugging, which saves a numeric std::vector as a
 * plain-text file compatible with MATLAB.
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

/** Load a numeric std::vector<double> from a text file (compat. with MATLAB)
 * \return Returns false on any error, true on everything OK.
 * \sa loadBinaryFile
 */
bool vectorNumericFromTextFile(
	std::vector<double>& vec, const std::string& fileName,
	const bool byRows = false);

/** @} */

}  // namespace mrpt::io
