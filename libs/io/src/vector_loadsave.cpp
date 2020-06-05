/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::io;
using namespace std;
namespace os = mrpt::system::os;

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

std::string mrpt::io::file_get_contents(const std::string& fileName)
{
	// Credits: https://stackoverflow.com/a/2602258/1631514
	// Note: Add "binary" to make sure the "tellg" file size matches the actual
	// number of read bytes afterwards:
	std::ifstream t(fileName, ios::binary);
	if (!t.is_open())
		THROW_EXCEPTION_FMT(
			"file_get_contents(): Error opening for read file `%s`",
			fileName.c_str());

	t.seekg(0, std::ios::end);
	std::size_t size = t.tellg();
	std::string buffer(size, ' ');
	t.seekg(0);
	t.read(&buffer[0], size);
	return buffer;
}

bool mrpt::io::vectorToTextFile(
	const vector<float>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (float it : vec) os::fprintf(f, byRows ? "%e " : "%e\n", it);

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::io::vectorToTextFile(
	const vector<double>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (double it : vec) os::fprintf(f, byRows ? "%e " : "%e\n", it);

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::io::vectorToTextFile(
	const vector<int>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (int it : vec) os::fprintf(f, byRows ? "%i " : "%i\n", it);

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::io::vectorToTextFile(
	const vector<size_t>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (unsigned long it : vec)
		os::fprintf(f, byRows ? "%u " : "%u\n", static_cast<unsigned int>(it));

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::io::vectorNumericFromTextFile(
	std::vector<double>& vec, const std::string& fileName, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), "r");
	if (!f) return false;

	double number = 0;

	while (!feof(f))
	{
		size_t readed = fscanf(f, byRows ? "%lf" : "%lf\n", &number);
		if ((!byRows) || (readed == 1)) vec.push_back(number);
	}

	return true;
}
