/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/vector_loadsave.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>

using namespace mrpt;
using namespace mrpt::io;
using namespace std;

bool mrpt::io::loadBinaryFile(
	std::vector<uint8_t>& out_data, const std::string& fileName)
{
	try
	{
		CFileInputStream fi(fileName);
		size_t N = fi.getTotalBytesCount();

		out_data.resize(N);
		if (N)
		{
			size_t NN = fi.Read(&out_data[0], N);
			return NN == N;
		}
		else
			return true;
	}
	catch (...)
	{
		return false;
	}
}

bool mrpt::io::vectorToBinaryFile(
	const std::vector<uint8_t>& vec, const std::string& fileName)
{
	try
	{
		mrpt::io::CFileOutputStream of(fileName);
		if (!vec.empty()) of.Write(&vec[0], sizeof(vec[0]) * vec.size());
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool mrpt::io::loadTextFile(
	std::vector<std::string>& o, const std::string& fileName)
{
	o.clear();
	std::ifstream f(fileName);
	if (!f.is_open()) return false;
	std::string s;
	while (std::getline(f, s)) o.emplace_back(std::move(s));
	return true;
}
