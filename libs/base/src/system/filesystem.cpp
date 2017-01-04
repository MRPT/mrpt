/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/CDirectoryExplorer.h>

#include <cstring>
#include <stdio.h>

#ifdef MRPT_OS_WINDOWS
    #include <conio.h>
	#include <windows.h>
	#include <process.h>
	#include <tlhelp32.h>
	#include <sys/utime.h>
	#include <io.h>
	#include <direct.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include <sys/time.h>
    #include <time.h>
	#include <unistd.h>
	#include <utime.h>
	#include <errno.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>

#if !defined(_MSC_VER)
#	define _access access
#	define _rmdir rmdir
#	ifndef _stat   // It seems MinGW already defines this.
#		define _stat stat
#	endif
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
					ExtractFileName
	Extracts just the name of a filename from a
      complete path plus name plus extension.
  ---------------------------------------------------------------*/
string  mrpt::system::extractFileName(const string& filePath)
{
	int	 i,dotPos = int(filePath.size());
	if (filePath.size()<2) return string("");

	for (i=(int)filePath.size()-1;i>=0 && !(filePath[i]=='\\' || filePath[i]=='/');i--)
		if (dotPos==int(filePath.size()) && filePath[i]=='.')
			dotPos = i;
	return filePath.substr(i+1,dotPos-i-1);
}

/*---------------------------------------------------------------
					ExtractFileDirectory
	Extracts just the directory of a filename from a
      complete path plus name plus extension.
  ---------------------------------------------------------------*/
string  mrpt::system::extractFileDirectory(const string& filePath)
{
	if (filePath.size()<2) return filePath;

	// Search the first "/" or "\" from the right:
	int i;
	for (i=(int)filePath.size()-1;i>0;i--)
		if (filePath[i]=='\\' || filePath[i]=='/')
			break;

	if (!i) return string("");
	else	return filePath.substr(0,i+1);
}

/*---------------------------------------------------------------
					ExtractFileName
	Extracts just the name of a filename from a
      complete path plus name plus extension.
  ---------------------------------------------------------------*/
string  mrpt::system::extractFileExtension(const string& filePath, bool ignore_gz)
{
	if (filePath.size()<2) return string("");

	size_t i_end = filePath.size()-1;

	int	i= (int)(i_end);
	while (i>0)
	{
		if (filePath[i]=='.')
		{
			string the_ext = filePath.substr(i+1,i_end-i);
			if (!ignore_gz || the_ext!="gz")
				return the_ext;
			else
			{
				i_end = --i;
			}
		}
		else	i--;
	}
	// No extension:
	return string("");
}

/*---------------------------------------------------------------
					FileExists
  ---------------------------------------------------------------*/
bool  mrpt::system::fileExists(const string& path)
{
	return 0 == _access(path.c_str(), 0x00 ); // 0x00 = Check for existence only!
}

/*---------------------------------------------------------------
					directoryExists
  ---------------------------------------------------------------*/
bool  mrpt::system::directoryExists(const std::string& _path)
{
	std::string path = _path;

	// Remove the trailing "/" or "\\":
	if (!path.empty() && (*path.rbegin()=='/' || *path.rbegin()=='\\'))
		path = path.substr(0,path.size()-1);

	// Verify it's a directory:
	struct _stat buf;
	if (0!=_stat(path.c_str(),&buf)) return false;

#ifdef MRPT_OS_WINDOWS
	return 0!=(buf.st_mode &_S_IFDIR);
#else
	return S_ISDIR(buf.st_mode);
#endif
}

/*---------------------------------------------------------------
						createDirectory
 ---------------------------------------------------------------*/
bool  mrpt::system::createDirectory( const string &dirName )
{
#ifdef MRPT_OS_WINDOWS
	bool rc = 0!=CreateDirectoryA( dirName.c_str(), NULL);
	return (rc || GetLastError()==ERROR_ALREADY_EXISTS);
#else
    int ret = mkdir( dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
    if (ret && errno!=EEXIST) // We ignore this error...
    {
		string str = format("[createDirectory %s]",dirName.c_str());
		perror(str.c_str());
		return false;
    }
    else
		return true;	// OK
#endif
}

/*---------------------------------------------------------------
						deleteFile
 ---------------------------------------------------------------*/
bool  mrpt::system::deleteFile( const string &fileName )
{
	return 0==remove( fileName.c_str() );
}

/*---------------------------------------------------------------
					mrpt::system::deleteFiles
---------------------------------------------------------------*/
void mrpt::system::deleteFiles(const string &s)
{
	MRPT_START
	size_t	len = s.size()+20;
	char	*aux=new char[len];
#ifdef MRPT_OS_WINDOWS
	os::sprintf(aux,len,"del %s",&s[0]);
	for (char *c=aux;*c;c++)
		if (*c=='/') *c = '\\';
	os::strcat(aux,len," /Q");
#else
	os::sprintf(aux,len,"rm %s",&s[0]);
#endif

	int res = ::system(aux);
	if (res)
	{
		cerr << "[mrpt::system::deleteFiles] Warning: error invoking: " << aux << endl;
	}
	delete[] aux;
	MRPT_END
}

/*---------------------------------------------------------------
					deleteFilesInDirectory
---------------------------------------------------------------*/
bool mrpt::system::deleteFilesInDirectory(const string &path, bool deleteDirectoryAsWell)
{
	if (!directoryExists(path)) return false;

	CDirectoryExplorer::TFileInfoList	lstFiles;
	CDirectoryExplorer::explore(path,FILE_ATTRIB_ARCHIVE | FILE_ATTRIB_DIRECTORY,lstFiles);

	for (CDirectoryExplorer::TFileInfoList::iterator i=lstFiles.begin();i!=lstFiles.end();++i)
	{
		if (i->isDir)
		{
			if (i->name!="." && i->name!="..")
			{
				if (!mrpt::system::deleteFilesInDirectory(i->wholePath, true))
					return false;
			}
		}
		else
		{
			if (!mrpt::system::deleteFile(i->wholePath))
				return false;
		}
	}

	// Finally, delete the directory itselt.
	if (deleteDirectoryAsWell)
			return 0==_rmdir(path.c_str());
	else	return true;
}

std::string mrpt::system::getcwd()
{
	MRPT_START

#ifdef MRPT_OS_WINDOWS
	char	auxBuf[MAX_PATH] = "";
	if(!::GetCurrentDirectoryA(sizeof(auxBuf) - 1, auxBuf))
		THROW_EXCEPTION("Error getting current working directory!");
	return std::string(auxBuf);

#else
	size_t size = 100;
	for (;;)
	{
		char *buffer = (char *) malloc(size);
		if (::getcwd(buffer, size) == buffer)
		{
			std::string s(buffer);
			free(buffer);
			return s;
		}
		free (buffer);
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
#ifdef MRPT_OS_WINDOWS
	FILETIME		tt;
	GetSystemTimeAsFileTime(&tt);
	const UINT uniq = static_cast<UINT>(tt.dwLowDateTime);
	char	TMP_PATH[ MAX_PATH ];
	char	tmpPath[ MAX_PATH ];
	GetTempPathA(MAX_PATH,tmpPath);
	GetTempFileNameA(tmpPath,"mrpt", uniq, TMP_PATH);
	return std::string( TMP_PATH );
#else
	char tmp[] = "/tmp/mrpt_tempXXXXXX";
	int fd;
	fd = mkstemp(tmp);
	ASSERT_(fd>=0);
	close(fd);
	return std::string( tmp );
#endif
}

/** Renames a file - If the target path is different and the filesystem allows it, it will be moved to the new location.
  * \return false on any error. In that case, if a pointer to a receiver string is passed in error_msg, a description of the error is saved there.
  */
bool mrpt::system::renameFile( const string &oldFileName, const string &newFileName,  std::string *error_msg )
{
	bool ret_err = 0==rename( oldFileName.c_str(), newFileName.c_str() );

	if (error_msg)
	{
		if (ret_err)
			*error_msg= strerror(errno);
		else
			*error_msg="";
	}

	return ret_err;
}

/*---------------------------------------------------------------
			fileNameStripInvalidChars
---------------------------------------------------------------*/
std::string mrpt::system::fileNameStripInvalidChars(const std::string &filename, const char replacement_to_invalid_chars)
{
	const char forbid[] = {'<','>',':','"','/','\\','|','?','*'};
	const unsigned int nForbid = sizeof(forbid) / sizeof(forbid[0]);

	string	ret(filename	);
	for (string::iterator c=ret.begin();c!=ret.end();++c)
	{
		bool invalid = (*c < 32);

		for (unsigned int i = 0; !invalid && i < nForbid; i++)
			if (*c == forbid[i])
				invalid = true;

		if (invalid) *c = '_';
	}
	return ret;
}


/*---------------------------------------------------------------
			getFileSize
---------------------------------------------------------------*/
uint64_t mrpt::system::getFileSize(const std::string &fileName)
{
#if defined(_MSC_VER)
	// Visual Studio:
	struct __stat64 filStat;
	if ( _stat64( fileName.c_str(), &filStat ) )
			return uint64_t(-1);
	else	return uint64_t(filStat.st_size);
#else
	// The rest of the world:
	struct stat filStat;
	if ( stat( fileName.c_str(), &filStat ) )
			return uint64_t(-1);
	else	return uint64_t(filStat.st_size);
#endif
}

/** Replace the filename extension by another one.  */
std::string mrpt::system::fileNameChangeExtension( const std::string &filePath, const std::string &newExtension )
{
	if (filePath.size()<2) return filePath;

	const size_t i_end = filePath.size()-1;

	for (int i=int(i_end);i>0;i--)
		if (filePath[i]=='.')
			 return filePath.substr(0,i+1) + newExtension;

	// No extension found: add it:
	return filePath + string(".") + newExtension;
}


/*---------------------------------------------------------------
			copyFile
---------------------------------------------------------------*/
bool mrpt::system::copyFile(
	const std::string &sourceFile,
	const std::string &targetFile,
	std::string *outErrStr,
	bool copyAttribs)
{
	const std::string org = mrpt::system::filePathSeparatorsToNative(sourceFile);
	const std::string trg = mrpt::system::filePathSeparatorsToNative(targetFile);

	const bool fil_exs = fileExists(org);
	const bool dir_exs = directoryExists(org);

	// Is source a directory?
	if ( !fil_exs )
	{
		if (outErrStr) *outErrStr = string("Source does not exist or permision denied!: ")+org;
		return false;
	}
	if (dir_exs)
	{
		if (outErrStr) *outErrStr = string("Is source a directory?: ")+org;
		return false;
	}

	// Check if source file exists and we have access to open it:
	FILE *f_src=fopen(org.c_str(),"rb");
	if (!f_src)
	{
		if (outErrStr) *outErrStr = string("Source file exists but cannot open it... is file being used?:  ")+org;
		return false;
	}

	// Assure that "target" is not an existing directory:
	if ( directoryExists(trg) )
	{
		if (outErrStr) *outErrStr = string("Target cannot be a directory: ")+trg;
		fclose(f_src);
		return false;
	}

	// Try to open the file for writting:
	FILE *f_trg=fopen(trg.c_str(),"wb");
	if (!f_trg)
	{
		if (!fileExists(trg))
		{
			// It does not exist and does not allow us to create it:
			if (outErrStr) *outErrStr = string("Cannot create target file: ")+trg;
			fclose(f_src);
			return false;
		}
		else
		{
			// It exists, but cannot overwrite it:

#ifdef MRPT_OS_WINDOWS
			// Try changing the permisions of the target file:
			DWORD dwProp = GetFileAttributesA( trg.c_str() );
			if (dwProp==INVALID_FILE_ATTRIBUTES)
			{
				if (outErrStr) *outErrStr = string("Cannot get file attributes for target file, trying to remove a possible read-only attribute after first attempt of copy failed, for: ")+trg;
				fclose(f_src);
				return false;
			}

			dwProp &= ~FILE_ATTRIBUTE_HIDDEN;
			dwProp &= ~FILE_ATTRIBUTE_READONLY;
			dwProp &= ~FILE_ATTRIBUTE_SYSTEM;

			if (!SetFileAttributesA( trg.c_str(), dwProp ))
			{
				if (outErrStr) *outErrStr = string("Cannot get file attributes for target file, trying to remove a possible read-only attribute after first attempt of copy failed, for: ")+trg;
				fclose(f_src);
				return false;
			}

			// Try again:
			f_trg=fopen(trg.c_str(),"wb");
			if (!f_trg)
			{
				if (outErrStr) *outErrStr = string("Cannot overwrite target file, even after changing file attributes! : ")+trg;
				fclose(f_src);
				return false;
			}
#else
			// Try changing the permisions of the target file:
			if (chmod( trg.c_str(), S_IRWXU | S_IRGRP | S_IROTH ) )
			{
				if (outErrStr) *outErrStr = string("Cannot set file permissions for target file, trying to remove a possible read-only attribute after first attempt of copy failed, for: ")+trg;
				fclose(f_src);
				return false;
			}

			// Try again:
			f_trg=fopen(trg.c_str(),"wb");
			if (!f_trg)
			{
				if (outErrStr) *outErrStr = string("Cannot overwrite target file, even after changing file permissions! : ")+trg;
				fclose(f_src);
				return false;
			}
#endif
		}
	}

	// Ok, here we have both files open: Perform the copy:
	char	buf[66000];
	size_t	nBytes=0;
	while (0!= (nBytes=fread(buf,1,64*1024,f_src)) )
	{
		if (nBytes!=fwrite(buf,1,nBytes,f_trg))
		{
			if (outErrStr) *outErrStr = string("Error writing the contents of the target file (disk full?): ")+trg;
			fclose(f_src);
			fclose(f_trg);
			return false;
		}
	}

	// Close file handles:
	fclose(f_src);
	fclose(f_trg);

	// In Windows only, copy the file attributes:
	if (copyAttribs)
	{
#ifdef MRPT_OS_WINDOWS
		DWORD dwPropSrc = GetFileAttributesA( org.c_str() );
		if (dwPropSrc==INVALID_FILE_ATTRIBUTES)
		{
			if (outErrStr) *outErrStr = string("Cannot get the file attributes for source file:  ")+org;
			return false;
		}
		DWORD dwPropTrg = GetFileAttributesA( trg.c_str() );
		if (dwPropTrg==INVALID_FILE_ATTRIBUTES)
		{
			if (outErrStr) *outErrStr = string("Cannot get the file attributes for target file:  ")+trg;
			return false;
		}

		// Leave only those attributes which can be legally copied:
		// (Refer to: http://msdn2.microsoft.com/en-us/library/aa365535.aspx )
		dwPropSrc &= FILE_ATTRIBUTE_ARCHIVE | FILE_ATTRIBUTE_HIDDEN | FILE_ATTRIBUTE_NORMAL | \
					 FILE_ATTRIBUTE_NOT_CONTENT_INDEXED | FILE_ATTRIBUTE_OFFLINE | \
					 FILE_ATTRIBUTE_READONLY | FILE_ATTRIBUTE_SYSTEM | FILE_ATTRIBUTE_TEMPORARY;

		// Copy them to the target attributes:
		dwPropTrg &= ~(FILE_ATTRIBUTE_ARCHIVE | FILE_ATTRIBUTE_HIDDEN | FILE_ATTRIBUTE_NORMAL | \
					   FILE_ATTRIBUTE_NOT_CONTENT_INDEXED | FILE_ATTRIBUTE_OFFLINE | \
					   FILE_ATTRIBUTE_READONLY | FILE_ATTRIBUTE_SYSTEM | FILE_ATTRIBUTE_TEMPORARY);
		dwPropTrg |= dwPropSrc;

		// Set attributes of target file:
		if (!SetFileAttributesA( trg.c_str(), dwPropTrg ))
		{
			if (outErrStr) *outErrStr = string("Cannot set file attributes for target file: ")+trg;
			return false;
		}
#else
		// Linux: No file attributes to copy.
#endif
	} // end // if (copyAttribs)

	return true;
}

// ---------------------------------------------------------------
//    filePathSeparatorsToNative
// ---------------------------------------------------------------
std::string mrpt::system::filePathSeparatorsToNative(const std::string & filePath)
{
	std::string ret = filePath;
	const size_t N = ret.size();
	for (size_t i=0;i<N;i++)
	{
#ifdef MRPT_OS_WINDOWS
		if (ret[i]=='/') ret[i]='\\';
#else
		if (ret[i]=='\\') ret[i]='/';
#endif
	}
	return ret;
}

time_t mrpt::system::getFileModificationTime(const std::string &filename)
{
	struct stat fS;
	if (0!=stat( filename.c_str(), &fS)) return 0;
	else return fS.st_mtime;
}

