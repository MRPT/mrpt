/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <fstream>
#include <iosfwd>
#include <sstream>
#include <string>
#include <vector>

namespace mrpt::io
{
/** @defgroup csv_load Load matrix from CSV file (in #include <mrpt/io/csv.h>)
 * \ingroup mrpt_io_grp
 * @{ */

/** Loads a matrix from a CSV text file.
 * Empty lines or those with a trailing `#` are ignored.
 * Requires including the `<Eigen/Dense>` header in the user translation unit.
 *
 * \tparam MATRIX Can be any Eigen matrix or mrpt::math matrices.
 *
 * \note Based on: https://stackoverflow.com/a/39146048/1631514
 */
template <typename MATRIX>
void load_csv(const std::string& path, MATRIX& M)
{
	std::ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<typename MATRIX::Scalar> values;
	uint rows = 0;
	while (std::getline(indata, line))
	{
		if (!line.empty() && line[0] == '#') continue;
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ','))
		{
			std::stringstream cs(cell);
			typename MATRIX::Scalar val;
			cs >> val;
			values.push_back(val);
		}
		++rows;
	}
	// Convert from RowMajor if needed!
	M = Eigen::Map<const Eigen::Matrix<
		typename MATRIX::Scalar, MATRIX::RowsAtCompileTime,
		MATRIX::ColsAtCompileTime, Eigen::RowMajor>>(
		values.data(), rows, values.size() / rows);
}

/** @} */

}  // namespace mrpt::io
