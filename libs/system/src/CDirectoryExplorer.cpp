/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#ifdef _WIN32
#ifdef _MSC_VER
#include <sys/utime.h>
#endif
#include <windows.h>

#include <direct.h>
#include <io.h>
#else
#include <dirent.h>
#include <sys/types.h>
#include <unistd.h>
#include <utime.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <queue>

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
		explore
 ---------------------------------------------------------------*/
void CDirectoryExplorer::explore(
	const string& path, const unsigned long in_mask, TFileInfoList& outList)
{
	MRPT_START

	unsigned long mask = in_mask;

	outList.clear();

	// The path terminated in "/" or "\\"
	string searchPath(path);
	if (searchPath.size())
		if (searchPath[searchPath.size() - 1] != '\\' &&
			searchPath[searchPath.size() - 1] != '/')
		{
#ifdef _WIN32
			searchPath += '\\';
#else
			searchPath.push_back('/');
#endif
		}

		// cout << "searchPath:"<<searchPath<<endl;

#ifdef _WIN32
	// ====================
	// WINDOWS VERSION
	// ====================
	WIN32_FIND_DATAA f;
	TFileInfo newEntry;

	string searchPath_mask = searchPath + string("*.*");

	HANDLE h = FindFirstFileA(searchPath_mask.c_str(), &f);
	if (h == INVALID_HANDLE_VALUE)
		THROW_EXCEPTION("Error starting exploration! (does path exist?)");

	// Include the FILE_ATTRIB_ARCHIVE flag for files:
	if (mask & FILE_ATTRIB_ARCHIVE) mask |= FILE_ATTRIBUTE_NORMAL;
	do
	{
		if ((mask & f.dwFileAttributes) != 0)  // Passes the user masks:
		{
			// File name:
			newEntry.name = string(f.cFileName);

			// Complete file path:
			newEntry.wholePath = searchPath;
			newEntry.wholePath += newEntry.name;

			// File size:
			newEntry.fileSize = ((uint64_t)f.nFileSizeLow) +
								(((uint64_t)f.nFileSizeHigh) << 32);

			// File times:
			struct stat statDat;
			if (stat(newEntry.wholePath.c_str(), &statDat))
			{
				FindClose(h);
				THROW_EXCEPTION_FMT(
					"Cannot get stat for file: '%s'",
					newEntry.wholePath.c_str());
			}

			newEntry.modTime = statDat.st_mtime;
			newEntry.accessTime = statDat.st_atime;

			// Flags:
			newEntry.isDir = 0 != (statDat.st_mode & _S_IFDIR);
			newEntry.isSymLink =
				false;  // (We donnot look for this in Windows, by now...)

			// Save:
			outList.push_back(newEntry);
		}
	} while (FindNextFileA(h, &f));

	FindClose(h);  // Ignore possible errors..

// Done
#else
	// ====================
	// LINUX VERSION
	// ====================
	TFileInfo newEntry;
	struct dirent* ent;

	DIR* dir = opendir(searchPath.c_str());
	if (!dir) THROW_EXCEPTION("Error starting exploration! (does path exist?)");

	while ((ent = readdir(dir)) != nullptr)
	{
		if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
		{
			// File name:
			newEntry.name = string(ent->d_name);

			// Complete file path:
			newEntry.wholePath = searchPath;
			newEntry.wholePath += newEntry.name;

			// File times:
			struct stat statDat
			{
			}, lstatDat{};
			if (stat(newEntry.wholePath.c_str(), &statDat))
			{
				closedir(dir);
				THROW_EXCEPTION_FMT(
					"Cannot get stat for file: '%s'",
					newEntry.wholePath.c_str());
			}

			newEntry.modTime = statDat.st_mtime;
			newEntry.accessTime = statDat.st_atime;

			// Flags:
			newEntry.isDir = S_ISDIR(statDat.st_mode);

			if (((mask & FILE_ATTRIB_ARCHIVE) != 0 && !newEntry.isDir) ||
				((mask & FILE_ATTRIB_DIRECTORY) != 0 && newEntry.isDir))
			{
				// File size:
				newEntry.fileSize = (intmax_t)statDat.st_size;

				// Is it a symbolic link?? Need to call "lstat":
				if (!lstat(newEntry.wholePath.c_str(), &lstatDat))
				{
					newEntry.isSymLink = S_ISLNK(lstatDat.st_mode);
				}
				else
					newEntry.isSymLink = false;

				// Save:
				outList.push_back(newEntry);
			}
		}
	}

	closedir(dir);

// Done
#endif

	MRPT_END
}

// Auxiliary function to order by name, ascending
bool cmpFileEntriesName_Asc(
	const CDirectoryExplorer::TFileInfo& a,
	const CDirectoryExplorer::TFileInfo& b)
{
	return a.wholePath < b.wholePath;
}
bool cmpFileEntriesName_Desc(
	const CDirectoryExplorer::TFileInfo& a,
	const CDirectoryExplorer::TFileInfo& b)
{
	return a.wholePath > b.wholePath;
}

/*---------------------------------------------------------------
		sortByName
 ---------------------------------------------------------------*/
void CDirectoryExplorer::sortByName(
	TFileInfoList& lstFiles, bool ascendingOrder)
{
	std::sort(
		lstFiles.begin(), lstFiles.end(),
		ascendingOrder ? cmpFileEntriesName_Asc : cmpFileEntriesName_Desc);
}

/*---------------------------------------------------------------
		filterByExtension
 ---------------------------------------------------------------*/
void CDirectoryExplorer::filterByExtension(
	TFileInfoList& lstFiles, const std::string& extension)
{
	int i, n = (int)lstFiles.size();
	for (i = n - 1; i >= 0; i--)
	{
		if (0 !=
			os::_strcmpi(
				mrpt::system::extractFileExtension(lstFiles[i].name).c_str(),
				extension.c_str()))
		{
			// Does not match:
			lstFiles.erase(lstFiles.begin() + i);
		}
	}
}
