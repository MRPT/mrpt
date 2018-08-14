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

namespace mrpt::system
{
/** @defgroup filesystem Directories, files, and file names
 * Header: `#include <mrpt/system/filesystem.h>`.
 * Library: \ref mrpt_system_grp
 * \ingroup mrpt_system_grp
 * @{ */

#define ASSERT_FILE_EXISTS_(FIL)       \
	ASSERTMSG_(                        \
		mrpt::system::fileExists(FIL), \
		std::string("Assert file existence failed: ") + ::std::string(FIL))

#define ASSERT_DIRECTORY_EXISTS_(DIR)                        \
	ASSERTMSG_(                                              \
		mrpt::system::directoryExists(DIR),                  \
		std::string("Assert directory existence failed: ") + \
			::std::string(DIR))

/** Returns the name of a proposed temporary file name */
std::string getTempFileName();

/** Returns the current working directory  */
std::string getcwd();

/** Attempts to find the directory `[PREFIX/]share/mrpt/` and returns its
 * absolute path, or empty string if not found.
 * Example return paths: Linux after installing = `/usr/share/mrpt/`;
 * manually-built system = `[CMAKE_SOURCE_DIR]/share/mrpt/`, etc.  */
std::string getShareMRPTDir();

/** Creates a directory
 * \return Returns false on any error, true on directory created or already
 * existed.
 */
bool createDirectory(const std::string& dirName);

/** Deletes a single file. For multiple files see deleteFiles
 * \return Returns false on any error, true on everything OK.
 * \sa deleteFiles
 */
bool deleteFile(const std::string& fileName);

/** Delete one or more files, especified by the (optional) path and the file
 * name (including '?' or '*') - Use forward slash ('/') for directories for
 * compatibility between Windows and Linux, since they will be internally
 * traslated into backward slashes ('\') if MRPT is compiled under Windows.
 * \sa deleteFile
 */
void deleteFiles(const std::string& s);

/** Renames a file - If the target path is different and the filesystem allows
 * it, it will be moved to the new location.
 * \return false on any error. In that case, if a pointer to a receiver string
 * is passed in error_msg, a description of the error is saved there.
 */
bool renameFile(
	const std::string& oldFileName, const std::string& newFileName,
	std::string* error_msg = nullptr);

/** Delete all the files in a given directory (nothing done if directory does
 * not exists, or path is a file).
 * \sa deleteFile
 * \return true on success
 */
bool deleteFilesInDirectory(
	const std::string& s, bool deleteDirectoryAsWell = false);

/** Extract just the name (without extension) of a filename from a complete path
 * plus name plus extension.
 *  This function works for either "/" or "\" directory separators.
 * \sa extractFileExtension,extractFileDirectory
 */
std::string extractFileName(const std::string& filePath);

/** Extract the extension of a filename.
 *  For example, for "dummy.cpp", it will return "cpp".
 *  If "ignore_gz" is true, the second extension will be returned if the file
 * name
 *   ends in ".gz", for example, for "foo.map.gz", this will return "map".
 * \sa extractFileName,extractFileDirectory
 */
std::string extractFileExtension(
	const std::string& filePath, bool ignore_gz = false);

/** Extract the whole path (the directory) of a filename from a complete path
 * plus name plus extension.
 *  This function works for either "/" or "\" directory separators.
 * \sa extractFileName,extractFileExtension
 */
std::string extractFileDirectory(const std::string& filePath);

/** Test if a given file (or directory) exists.
 * \sa directoryExists
 */
bool fileExists(const std::string& fileName);

/** Test if a given directory exists (it fails if the given path refers to an
 * existing file).
 * \sa fileExists
 */
bool directoryExists(const std::string& fileName);

/** Replace invalid filename chars by underscores ('_') or any other user-given
 * char.
 *  Invalid chars are: '<','>',':','"','/','\\','|','?','*'
 */
std::string fileNameStripInvalidChars(
	const std::string& filename, const char replacement_to_invalid_chars = '_');

/** Replace the filename extension by another one.
 *  Example:
 *   \code
 *     fileNameChangeExtension("cool.txt","bar") // -> "cool.bar"
 *   \endcode
 */
std::string fileNameChangeExtension(
	const std::string& filename, const std::string& newExtension);

/** Return the size of the given file, or size_t(-1) if some error is found
 * accessing that file.  */
uint64_t getFileSize(const std::string& fileName);

/** Return the time of the file last modification, or "0" if the file doesn't
 * exist.  */
time_t getFileModificationTime(const std::string& filename);

/** Windows: replace all '/'->'\' , in Linux/MacOS: replace all '\'->'/' */
std::string filePathSeparatorsToNative(const std::string& filePath);

/** Copies file \a sourceFile to \a targetFile. If the target file exists, it
 * will be overwritten.
 *  If the target file cannot be overwritten, the function first tries to
 * change its permissions/attributes and retries opening it for write.
 *
 * \note Only for Windows: After a successful copy, if \a copyAttribs is true,
 * the attributes of the source file are also copied. Note that not all
 *   attributes can be copied:
 * http://msdn2.microsoft.com/en-us/library/aa365535.aspx
 *
 * \return true on success, false on any error, whose description can be
 * optionally get in outErrStr
 */
bool copyFile(
	const std::string& sourceFile, const std::string& targetFile,
	std::string* outErrStr = nullptr, bool copyAttribs = true);

/** @} */
}  // namespace mrpt::system
