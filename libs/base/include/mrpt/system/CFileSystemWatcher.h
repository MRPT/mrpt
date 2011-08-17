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
#ifndef  CFileSystemWatcher_H
#define  CFileSystemWatcher_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CThreadSafeQueue.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace system
	{
		/** This class subscribes to notifications of file system changes, thus it can be used to efficiently stay informed about changes in a directory tree.
		  *  - Windows: Requires Windows 2000 or newer.
		  *  - Linux: Requires kernel 2.6.13 or newer.
		  *  Using this class in an old Linux or other unsoported system (Unix,etc...) has no effect, i.e. no notification will be ever received.
		  *  \sa CDirectoryExplorer
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CFileSystemWatcher
		{
		public:
			/** Each of the changes detected by utils::CFileSystemWatcher
			  */
			struct BASE_IMPEXP TFileSystemChange
			{
				TFileSystemChange() : 
					path(), isDir(false),
					eventModified(false), eventCloseWrite(false),
					eventDeleted(false), eventMovedTo(false),
					eventMovedFrom(false), eventCreated(false),
					eventAccessed(false) {}

				std::string path; 				//!< Complete path of the file/directory that has changed.
				bool 		isDir;				//!< Whether the event happened to a file or a directory.
				bool 		eventModified;
				bool 		eventCloseWrite;
				bool 		eventDeleted;
				bool 		eventMovedTo;
				bool 		eventMovedFrom;
				bool 		eventCreated;
				bool		eventAccessed;
			};

			typedef std::deque<TFileSystemChange>	TFileSystemChangeList;

			/** Creates the subscription to a specified path.
			  * \param path The file or directory to watch.
			  */
			CFileSystemWatcher(	const std::string &path );

			/** Destructor
			  */
			virtual ~CFileSystemWatcher();

			/** Call this method sometimes to get the list of changes in the watched directory.
			  *  \sa processChange
			  */
			void getChanges( TFileSystemChangeList &out_list );

		private:
			std::string		m_watchedDirectory; //!< Ended in "/"
	#ifdef MRPT_OS_WINDOWS
			void	*m_hNotif;
			mrpt::system::TThreadHandle	m_watchThread;
			void thread_win32_watch(); //!< Watch thread; only needed in win32
			mrpt::utils::CThreadSafeQueue<TFileSystemChange>  m_queue_events_win32;

	#endif

	#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
			int 	m_fd;  //!< The fd returned by inotify_init.
			int 	m_wd;  //!< The fd of the watch.
	#endif

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
