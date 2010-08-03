/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/CDirectoryExplorer.h>

#ifdef MRPT_OS_WINDOWS
    #include <conio.h>
	#include <windows.h>
	#include <process.h>
	#include <tlhelp32.h>
	#include <sys/utime.h>
	#include <io.h>
	#include <direct.h>
#else
    #include <pthread.h>
    #include <termios.h>
    #include <unistd.h>
    #include <sys/select.h>
    #include <sys/time.h>
    #include <time.h>
	#include <unistd.h>
	#include <utime.h>
	#include <errno.h>
	#include <signal.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>

#if !defined(_MSC_VER)
	#define _access access
	#define _rmdir rmdir
	#define _stat stat
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
	return 0!=CreateDirectoryA( dirName.c_str(), NULL );
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
	MRPT_START;
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
	MRPT_END;
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
	static int	uniq = 0;
	char	TMP_PATH[ MAX_PATH ];
	char	tmpPath[ MAX_PATH ];
	GetTempPathA(MAX_PATH,tmpPath);
	GetTempFileNameA(tmpPath,"mrpt", ++uniq, TMP_PATH);
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
std::string mrpt::system::fileNameStripInvalidChars( const std::string &filename)
{
	string	ret(filename);
	for (string::iterator c=ret.begin();c!=ret.end();++c)
	{
		if (!isalnum(*c) &&
			*c !='.' &&
			*c !='-' &&
			*c !='#' &&
			*c !='%' &&
			*c !='$' &&
			*c !='&' &&
			*c !='+' &&
			*c !='(' && *c !=')' &&
			*c !='[' && *c !=']' &&
			*c !='{' && *c !='{' )
				*c = '_';
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
