/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
