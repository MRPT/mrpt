/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/exceptions.h>  // for MRPT_END, MRPT_START, e
#include <mrpt/core/format.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>	 // for sprintf

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#if STD_FS_IS_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <conio.h>
#include <direct.h>
#include <io.h>
#include <process.h>
#include <sys/utime.h>
#include <tlhelp32.h>
#else
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <utime.h>

#include <cerrno>
#include <ctime>
#endif

#include <sys/stat.h>
#include <sys/types.h>

#if !defined(_MSC_VER)
#define _access access
#define _rmdir rmdir
#ifndef _stat  // It seems MinGW already defines this.
#define _stat stat
#endif
#endif

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
					ExtractFileName
	Extracts just the name of a filename from a
	  complete path plus name plus extension.
  ---------------------------------------------------------------*/
string mrpt::system::extractFileName(const string& filePath)
{
	int i, dotPos = int(filePath.size());
	if (filePath.size() < 2) return string("");

	for (i = (int)filePath.size() - 1;
		 i >= 0 && !(filePath[i] == '\\' || filePath[i] == '/'); i--)
		if (dotPos == int(filePath.size()) && filePath[i] == '.') dotPos = i;
	return filePath.substr(i + 1, dotPos - i - 1);
}

/*---------------------------------------------------------------
					ExtractFileDirectory
	Extracts just the directory of a filename from a
	  complete path plus name plus extension.
  ---------------------------------------------------------------*/
string mrpt::system::extractFileDirectory(const string& filePath)
{
	if (filePath.size() < 2) return filePath;

	// Search the first "/" or "\" from the right:
	int i;
	for (i = (int)filePath.size() - 1; i > 0; i--)
		if (filePath[i] == '\\' || filePath[i] == '/') break;

	if (!i) return string("./");
	else
		return filePath.substr(0, i + 1);
}

/*---------------------------------------------------------------
					ExtractFileName
	Extracts just the name of a filename from a
	  complete path plus name plus extension.
  ---------------------------------------------------------------*/
string mrpt::system::extractFileExtension(
	const string& filePath, bool ignore_gz)
{
	if (filePath.size() < 2) return string("");

	size_t i_end = filePath.size() - 1;

	int i = (int)(i_end);
	while (i > 0)
	{
		if (filePath[i] == '.')
		{
			string the_ext = filePath.substr(i + 1, i_end - i);
			if (!ignore_gz || the_ext != "gz") return the_ext;
			else
			{
				i_end = --i;
			}
		}
		else
			i--;
	}
	// No extension:
	return string("");
}

/*---------------------------------------------------------------
					FileExists
  ---------------------------------------------------------------*/
bool mrpt::system::fileExists(const string& path)
{
	return 0 ==
		_access(path.c_str(), 0x00);  // 0x00 = Check for existence only!
}

/*---------------------------------------------------------------
					directoryExists
  ---------------------------------------------------------------*/
bool mrpt::system::directoryExists(const std::string& _path)
{
	std::string path = _path;

	// Remove the trailing "/" or "\\":
	if (!path.empty() && (*path.rbegin() == '/' || *path.rbegin() == '\\'))
		path = path.substr(0, path.size() - 1);

	// Verify it's a directory:
	struct _stat buf
	{
	};
	if (0 != _stat(path.c_str(), &buf)) return false;

#ifdef _WIN32
	return 0 != (buf.st_mode & _S_IFDIR);
#else
	return S_ISDIR(buf.st_mode);
#endif
}

/*---------------------------------------------------------------
						createDirectory
 ---------------------------------------------------------------*/
bool mrpt::system::createDirectory(const string& dirName)
{
#ifdef _WIN32
	bool rc = 0 != CreateDirectoryA(dirName.c_str(), nullptr);
	return (rc || GetLastError() == ERROR_ALREADY_EXISTS);
#else
	int ret = mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (ret && errno != EEXIST)	 // We ignore this error...
	{
		string str = format("[createDirectory %s]", dirName.c_str());
		perror(str.c_str());
		return false;
	}
	else
		return true;  // OK
#endif
}

/*---------------------------------------------------------------
						deleteFile
 ---------------------------------------------------------------*/
bool mrpt::system::deleteFile(const string& fileName)
{
	return 0 == remove(fileName.c_str());
}

/*---------------------------------------------------------------
					mrpt::system::deleteFiles
---------------------------------------------------------------*/
void mrpt::system::deleteFiles(const string& s)
{
	MRPT_START
	size_t len = s.size() + 20;
	std::vector<char> aux(len);
#ifdef _WIN32
	os::sprintf(&aux[0], len, "del %s", &s[0]);
	for (char* c = &aux[0]; *c; c++)
		if (*c == '/') *c = '\\';
	os::strcat(&aux[0], len, " /Q");
#else
	os::sprintf(&aux[0], len, "rm %s", &s[0]);
#endif

	int res = ::system(&aux[0]);
	if (res)
	{
		fprintf(
			stderr,
			"[mrpt::system::deleteFiles] Warning: error invoking: `%s`\n",
			&aux[0]);
	}
	MRPT_END
}

/*---------------------------------------------------------------
					deleteFilesInDirectory
---------------------------------------------------------------*/
bool mrpt::system::deleteFilesInDirectory(
	const string& path, bool deleteDirectoryAsWell)
{
	if (!directoryExists(path)) return false;

	CDirectoryExplorer::TFileInfoList lstFiles;
	CDirectoryExplorer::explore(
		path, FILE_ATTRIB_ARCHIVE | FILE_ATTRIB_DIRECTORY, lstFiles);

	for (auto& lstFile : lstFiles)
	{
		if (lstFile.isDir)
		{
			if (lstFile.name != "." && lstFile.name != "..")
			{
				if (!mrpt::system::deleteFilesInDirectory(
						lstFile.wholePath, true))
					return false;
			}
		}
		else
		{
			if (!mrpt::system::deleteFile(lstFile.wholePath)) return false;
		}
	}

	// Finally, delete the directory itselt.
	if (deleteDirectoryAsWell) return 0 == _rmdir(path.c_str());
	else
		return true;
}

std::string mrpt::system::getcwd()
{
	MRPT_START

#ifdef _WIN32
	char auxBuf[MAX_PATH] = "";
	if (!::GetCurrentDirectoryA(sizeof(auxBuf) - 1, auxBuf))
		THROW_EXCEPTION("Error getting current working directory!");
	return std::string(auxBuf);

#else
	size_t size = 100;
	for (;;)
	{
		std::string cwd;
		cwd.resize(size);
		if (::getcwd(&cwd[0], size) == &cwd[0]) { return cwd; }
		if (errno != ERANGE)
			THROW_EXCEPTION("Error getting current working directory!");
		size *= 2;
	}
#endif

	MRPT_END
}

/*---------------------------------------------------------------
					getTempFileName
  ---------------------------------------------------------------*/
std::string mrpt::system::getTempFileName()
{
#ifdef _WIN32
	FILETIME tt;
	GetSystemTimeAsFileTime(&tt);
	const UINT uniq = static_cast<UINT>(tt.dwLowDateTime);
	char TMP_PATH[MAX_PATH];
	char tmpPath[MAX_PATH];
	GetTempPathA(MAX_PATH, tmpPath);
	GetTempFileNameA(tmpPath, "mrpt", uniq, TMP_PATH);
	return std::string(TMP_PATH);
#else
	char tmp[] = "/tmp/mrpt_tempXXXXXX";
	int fd;
	fd = mkstemp(tmp);
	ASSERT_(fd >= 0);
	close(fd);
	return std::string(tmp);
#endif
}

/** Renames a file - If the target path is different and the filesystem allows
 * it, it will be moved to the new location.
 * \return false on any error. In that case, if a pointer to a receiver string
 * is passed in error_msg, a description of the error is saved there.
 */
bool mrpt::system::renameFile(
	const string& oldFileName, const string& newFileName,
	std::string* error_msg)
{
	bool ret_err = 0 == rename(oldFileName.c_str(), newFileName.c_str());

	if (error_msg)
	{
		if (ret_err) *error_msg = strerror(errno);
		else
			*error_msg = "";
	}

	return ret_err;
}

/*---------------------------------------------------------------
			fileNameStripInvalidChars
---------------------------------------------------------------*/
std::string mrpt::system::fileNameStripInvalidChars(
	const std::string& filename, const char replacement_to_invalid_chars)
{
	const char forbid[] = {'<', '>', ':', '"', '/', '\\', '|', '?', '*'};
	const unsigned int nForbid = sizeof(forbid) / sizeof(forbid[0]);

	string ret(filename);
	for (char& c : ret)
	{
		bool invalid = (c < 32);

		for (unsigned int i = 0; !invalid && i < nForbid; i++)
			if (c == forbid[i]) invalid = true;

		if (invalid) c = replacement_to_invalid_chars;
	}
	return ret;
}

/*---------------------------------------------------------------
			getFileSize
---------------------------------------------------------------*/
uint64_t mrpt::system::getFileSize(const std::string& fileName)
{
#if defined(_MSC_VER)
	// Visual Studio:
	struct __stat64 filStat;
	if (_stat64(fileName.c_str(), &filStat)) return uint64_t(-1);
	else
		return uint64_t(filStat.st_size);
#else
	// The rest of the world:
	struct stat filStat
	{
	};
	if (stat(fileName.c_str(), &filStat)) return uint64_t(-1);
	else
		return uint64_t(filStat.st_size);
#endif
}

/** Replace the filename extension by another one.  */
std::string mrpt::system::fileNameChangeExtension(
	const std::string& filePath, const std::string& newExtension)
{
	if (filePath.size() < 2) return filePath;

	const size_t i_end = filePath.size() - 1;

	for (int i = int(i_end); i > 0; i--)
		if (filePath[i] == '.') return filePath.substr(0, i + 1) + newExtension;

	// No extension found: add it:
	return filePath + string(".") + newExtension;
}

bool mrpt::system::copyFile(
	const std::string& sourceFile, const std::string& targetFile,
	std::string* outErrStr)
{
	std::error_code ec;
	bool ok = fs::copy_file(
		fs::path(sourceFile), fs::path(targetFile),
		fs::copy_options::overwrite_existing, ec);

	if (ok) return true;

	if (outErrStr) *outErrStr = ec.message();

	return false;
}

std::string mrpt::system::filePathSeparatorsToNative(
	const std::string& filePath)
{
	return fs::path(filePath).native();
}

time_t mrpt::system::getFileModificationTime(const std::string& filename)
{
	struct stat fS
	{
	};
	if (0 != stat(filename.c_str(), &fS)) return 0;
	else
		return fS.st_mtime;
}

#include <mrpt/version.h>
// Read docs in .h
std::string mrpt::system::getShareMRPTDir()
{
	using std::string;
	using std::vector;

	static vector<string> sPaths;
	static string sDetectedPath;
	static bool is_first = true;
	if (is_first)
	{
		is_first = false;

		// Source dir:
		sPaths.push_back(
			string(MRPT_CMAKE_SOURCE_DIR) + string("/share/mrpt/"));
		// Install dir:
		sPaths.push_back(
			string(MRPT_CMAKE_INSTALL_PREFIX) + string("/share/mrpt/"));

		// Program path & ".." & "../..":
		char buf[2048];
		bool sBufOk = false;
#ifdef _WIN32
		sBufOk = (0 != GetModuleFileNameA(NULL, buf, sizeof(buf)));
#endif
#ifdef MRPT_OS_LINUX
		ssize_t nRead = readlink("/proc/self/exe", buf, sizeof(buf));
		if (nRead >= 0) buf[nRead] = '\0';
		sBufOk = (-1 != nRead);
#endif

		if (sBufOk)
		{
			string sBuf = string(buf);
			std::replace(sBuf.begin(), sBuf.end(), '\\', '/');
			sBuf = extractFileDirectory(sBuf);
			sPaths.push_back(sBuf + string("share/mrpt/"));
			sPaths.push_back(sBuf + string("../share/mrpt/"));
			sPaths.push_back(sBuf + string("../../share/mrpt/"));
		}

		for (const auto& e : sPaths)
			if (directoryExists(e))
			{
				sDetectedPath = e;
				break;
			}
	}
	return sDetectedPath;
}

std::string mrpt::system::toAbsolutePath(const std::string& path)
{
	//
	x;
}

std::string mrpt::system::pathJoin(const std::vector<std::string>& paths)
{
	//
	x;
}
