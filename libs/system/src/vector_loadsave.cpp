/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/system/vector_loadsave.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

bool mrpt::system::vectorToTextFile(
	const vector<float>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (float it : vec) os::fprintf(f, byRows ? "%e " : "%e\n", it);

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::system::vectorToTextFile(
	const vector<double>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (double it : vec) os::fprintf(f, byRows ? "%e " : "%e\n", it);

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::system::vectorToTextFile(
	const vector<int>& vec, const string& fileName, bool append, bool byRows)
{
	FILE* f = os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (int it : vec) os::fprintf(f, byRows ? "%i " : "%i\n", it);

	if (byRows) os::fprintf(f, "\n");

	os::fclose(f);
	return true;  // All ok.
}

bool mrpt::system::vectorToTextFile(
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

bool mrpt::system::vectorFromTextFile(
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
