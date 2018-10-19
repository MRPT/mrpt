/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/system/os.h>
#include <thread>
#include <mutex>
#include <queue>

namespace mrpt::system
{
/** This class subscribes to notifications of file system changes, thus it can
 * be used to efficiently stay informed about changes in a directory tree.
 *  - Windows: Requires Windows 2000 or newer.
 *  - Linux: Requires kernel 2.6.13 or newer.
 *  Using this class in an old Linux or other unsoported system (Unix,etc...)
 * has no effect, i.e. no notification will be ever received.
 *  \sa CDirectoryExplorer
 * \ingroup mrpt_system_grp
 */
class CFileSystemWatcher
{
   public:
	/** Each of the changes detected by utils::CFileSystemWatcher
	 */
	struct TFileSystemChange
	{
		TFileSystemChange() = default;
		/** Complete path of the file/directory that has changed. */
		std::string path{};
		/** Whether the event happened to a file or a directory. */
		bool isDir{false};
		bool eventModified{false};
		bool eventCloseWrite{false};
		bool eventDeleted{false};
		bool eventMovedTo{false};
		bool eventMovedFrom{false};
		bool eventCreated{false};
		bool eventAccessed{false};
	};

	using TFileSystemChangeList = std::deque<TFileSystemChange>;

	/** Creates the subscription to a specified path.
	 * \param path The file or directory to watch.
	 */
	CFileSystemWatcher(const std::string& path);

	/** Destructor
	 */
	virtual ~CFileSystemWatcher();

	/** Call this method sometimes to get the list of changes in the watched
	 * directory.
	 *  \sa processChange
	 */
	void getChanges(TFileSystemChangeList& out_list);

   private:
	/** Ended in "/" */
	std::string m_watchedDirectory;
#ifdef _WIN32
	void* m_hNotif{nullptr};
	std::thread m_watchThread;
	/** Watch thread; only needed in win32 */
	void thread_win32_watch();
	std::queue<TFileSystemChange*> m_queue_events_win32_msgs;
	mutable std::mutex m_queue_events_win32_cs;
#endif

#if defined(MRPT_OS_LINUX) || defined(__APPLE__)
	/** The fd returned by inotify_init. */
	int m_fd;
	/** The fd of the watch. */
	int m_wd;
#endif

};  // End of class def.

}  // namespace mrpt::system
