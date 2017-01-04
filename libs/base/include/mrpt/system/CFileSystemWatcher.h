/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CFileSystemWatcher_H
#define  CFileSystemWatcher_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CThreadSafeQueue.h>

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
