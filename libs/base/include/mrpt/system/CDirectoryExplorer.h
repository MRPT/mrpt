/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  CDirectoryExplorer_H
#define  CDirectoryExplorer_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace system
	{
		#define FILE_ATTRIB_ARCHIVE		0x0020
		#define FILE_ATTRIB_DIRECTORY	0x0010

		/** This class allows the enumeration of the files/directories that exist into a given path.
		  *  The only existing method is "explore" and returns the list of found files & directories.
		  *  Refer to the example in /samples/UTILS/directoryExplorer
		  *
		  *  \sa CFileSystemWatcher
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CDirectoryExplorer
		{
		public:
			/** This represents the information about each file.
			  * \sa
			  */
			struct BASE_IMPEXP TFileInfo
			{
				/** The file name (without the whole path).
  				  */
				std::string     name;

				/** The whole file path.
  				  */
				std::string	wholePath;

				/** Access and modification times.
  				  */
				time_t          accessTime,modTime;

				bool            isDir, isSymLink;

				/** The size of the file in bytes.
				  */
				uint64_t	        fileSize;
			};

			/** The list type used in "explore".
			  * \sa explore
 			  */
			typedef std::deque<TFileInfo> TFileInfoList;

		public:
			/** The path of the directory to examine must be passed to this constructor, among the
			  *  According to the following parameters, the object will collect the list of files, which
			  *   can be modified later through other methods in this class.
			  * \param path The path to examine (IT MUST BE A DIRECTORY), e.g "d:\temp\", or "/usr/include/"
			  * \param mask One or the OR'ed combination of the values "FILE_ATTRIB_ARCHIVE" and "FILE_ATTRIB_DIRECTORY", depending on what file types do you want in the list (These values are platform-independent).
			  * \param outList The list of found files/directories is stored here.
			  * \sa sortByName
			  */
			static void explore(
				const std::string 	&path,
				const unsigned long 	mask,
				TFileInfoList  		&outList );

			/** Sort the file entries by name, in ascending or descending order
			  */
			static void sortByName( TFileInfoList &lstFiles, bool ascendingOrder=true );

			/** Remove from the list of files those whose extension does not coincide (without case) with the given one.
			  *  Example:  filterByExtension(lst,"txt");
			  */
			static void filterByExtension( TFileInfoList &lstFiles, const std::string &extension  );

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
