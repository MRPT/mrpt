/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_FILESYSTEM_H
#define  MRPT_FILESYSTEM_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace system
	{
		/** @defgroup filesystem Directories, files, and file names
		  * \ingroup mrpt_base_grp
		  * @{ */

		/** Returns the name of a proposed temporary file name */
		std::string BASE_IMPEXP getTempFileName();

		/** Returns the current working directory.
		  */
		std::string BASE_IMPEXP getcwd();

		/** Creates a directory
		  * \return Returns false on any error, true on everything OK.
		  * \todo Seems to return false on Linux even after creating the directory OK.
		  */
		bool  BASE_IMPEXP createDirectory( const std::string &dirName );

		/** Deletes a single file. For multiple files see deleteFiles
		  * \return Returns false on any error, true on everything OK.
		  * \sa deleteFiles
		  */
		bool BASE_IMPEXP deleteFile( const std::string &fileName );

		/** Delete one or more files, especified by the (optional) path and the file name (including '?' or '*') - Use forward slash ('/') for directories for compatibility between Windows and Linux, since they will be internally traslated into backward slashes ('\') if MRPT is compiled under Windows.
		  * \sa deleteFile
		  */
		void BASE_IMPEXP deleteFiles(const std::string &s);

		/** Renames a file - If the target path is different and the filesystem allows it, it will be moved to the new location.
		  * \return false on any error. In that case, if a pointer to a receiver string is passed in error_msg, a description of the error is saved there.
		  */
		bool BASE_IMPEXP renameFile( const std::string &oldFileName, const std::string &newFileName,  std::string *error_msg=NULL );

		/** Delete all the files in a given directory (nothing done if directory does not exists, or path is a file).
		  * \sa deleteFile
		  * \return true on success
		  */
		bool BASE_IMPEXP deleteFilesInDirectory(const std::string &s, bool deleteDirectoryAsWell = false );

		/** Extract just the name (without extension) of a filename from a complete path plus name plus extension.
		  *  This function works for either "/" or "\" directory separators.
		  * \sa extractFileExtension,extractFileDirectory
		  */
		std::string  BASE_IMPEXP extractFileName(const std::string& filePath);

		/** Extract the extension of a filename.
		  *  For example, for "dummy.cpp", it will return "cpp".
		  *  If "ignore_gz" is true, the second extension will be returned if the file name
		  *   ends in ".gz", for example, for "foo.map.gz", this will return "map".
		  * \sa extractFileName,extractFileDirectory
		  */
		std::string  BASE_IMPEXP extractFileExtension(const std::string &filePath, bool ignore_gz = false );

		/** Extract the whole path (the directory) of a filename from a complete path plus name plus extension.
		  *  This function works for either "/" or "\" directory separators.
		  * \sa extractFileName,extractFileExtension
		  */
		std::string  BASE_IMPEXP extractFileDirectory(const std::string &filePath);

		/** Test if a given file (or directory) exists.
		  * \sa directoryExists
		  */
		bool BASE_IMPEXP fileExists(const std::string& fileName);

		/** Test if a given directory exists (it fails if the given path refers to an existing file).
		  * \sa fileExists
		  */
		bool BASE_IMPEXP directoryExists(const std::string& fileName);

		/** Replace invalid filename chars by underscores ('_').
		  *  Invalid chars are identified by those not being alphanumeric or: ".-#%$&()+[]{}"
		  */
		std::string BASE_IMPEXP fileNameStripInvalidChars( const std::string &filename);

		/** Replace the filename extension by another one.
		  *  Example:
		  *   \code
		  *     fileNameChangeExtension("cool.txt","bar") // -> "cool.bar"
		  *   \endcode
		  */
		std::string BASE_IMPEXP fileNameChangeExtension( const std::string &filename, const std::string &newExtension );

		/** Return the size of the given file, or size_t(-1) if some error is found accessing that file.
		  */
		uint64_t BASE_IMPEXP getFileSize(const std::string &fileName);

		/** Windows: replace all '/'->'\' , in Linux/MacOS: replace all '\'->'/' */
		std::string BASE_IMPEXP filePathSeparatorsToNative(const std::string & filePath);

		/** Copies file \a sourceFile to \a targetFile. If the target file exists, it will be overwritten.
		  *  If the target file cannot be overwritten, the function first tries to change its permissions/attributes and retries opening it for write.
		  *
		  * \note Only for Windows: After a successful copy, if \a copyAttribs is true, the attributes of the source file are also copied. Note that not all
		  *   attributes can be copied: http://msdn2.microsoft.com/en-us/library/aa365535.aspx
		  *
		  * \return true on success, false on any error, whose description can be optionally get in outErrStr
		  */
		bool BASE_IMPEXP copyFile(
			const std::string &sourceFile,
			const std::string &targetFile,
			std::string *outErrStr = NULL,
			bool copyAttribs = true );


		/** @} */


	} // End of namespace

} // End of namespace

#endif
