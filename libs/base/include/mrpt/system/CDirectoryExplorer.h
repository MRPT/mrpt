/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CDirectoryExplorer_H
#define  CDirectoryExplorer_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>
#include <deque>

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
