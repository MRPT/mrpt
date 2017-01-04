/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers



#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS
#	ifndef _WIN32_WINNT
#		define _WIN32_WINNT 0x0400
#	endif
	#include <windows.h>
#else
	#if MRPT_HAS_INOTIFY
		#include <sys/inotify.h>
	#endif

//	#include <time.h>
	#include <unistd.h>
	#include <errno.h>
	#include <stdio.h>
#endif

#include <cstring>

#include <mrpt/system/CFileSystemWatcher.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>


using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CFileSystemWatcher::CFileSystemWatcher( const std::string &path ) :
	m_watchedDirectory(path)
{
	MRPT_START
	ASSERT_(!path.empty())

	if (m_watchedDirectory[m_watchedDirectory.size()-1] != '/' && m_watchedDirectory[m_watchedDirectory.size()-1] != '\\' )
		m_watchedDirectory.push_back('/');

#ifdef MRPT_OS_WINDOWS
	// Windows version:
	HANDLE hDir = CreateFileA(
		path.c_str(),
		FILE_LIST_DIRECTORY,
		FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		NULL, //security attributes
		OPEN_EXISTING,
		FILE_FLAG_BACKUP_SEMANTICS,
		NULL);
	if( hDir == INVALID_HANDLE_VALUE )
	{
		m_hNotif=NULL;
        THROW_EXCEPTION("FindFirstChangeNotificationA returned error!");
	}

	m_hNotif = static_cast<void*>(hDir);

	m_watchThread = mrpt::system::createThreadFromObjectMethod(this, &CFileSystemWatcher::thread_win32_watch);

#else
#	if MRPT_HAS_INOTIFY
	// Linux version:
	m_wd = -1;

	m_fd = inotify_init();
	if (m_fd < 0)
        THROW_EXCEPTION("inotify_init returned error!");

	// Create watcher:
	m_wd = inotify_add_watch(
		m_fd,
		path.c_str(),
		IN_CLOSE_WRITE | IN_DELETE | IN_MOVED_TO | IN_MOVED_FROM | IN_CREATE | IN_ACCESS
		);

	if (m_wd < 0)
        THROW_EXCEPTION("inotify_add_watch returned error!");
#	endif
#endif
	MRPT_END
}


/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CFileSystemWatcher::~CFileSystemWatcher( )
{
#ifdef MRPT_OS_WINDOWS
	// Windows version:
	if (m_hNotif)
	{
		// Kill thread:
		if (!m_watchThread.isClear())
			mrpt::system::terminateThread(m_watchThread);
		CloseHandle(HANDLE(m_hNotif));
		m_hNotif=NULL;
	}
#else
#	if	MRPT_HAS_INOTIFY
	// Linux version:
	if (m_fd >= 0)
	{
		close(m_fd);
		m_fd = -1;
		if (m_wd>=0)
			inotify_rm_watch(m_fd, m_wd);
	}
#	endif
#endif
}

/*---------------------------------------------------------------
					getChanges
 ---------------------------------------------------------------*/
void CFileSystemWatcher::getChanges( TFileSystemChangeList &out_list )
{
	out_list.clear();

#ifdef MRPT_OS_WINDOWS
	// Windows version:
	ASSERTMSG_(m_hNotif!=NULL,"CFileSystemWatcher was not initialized correctly.")

	// Work is done in thread_win32_watch().
	// Just check for incoming mail:
	while (!m_queue_events_win32.empty())
	{
		TFileSystemChange *obj = m_queue_events_win32.get();
		if (obj)
		{
			out_list.push_back(*obj);
			delete obj;
		}
	}


#else
#	if	MRPT_HAS_INOTIFY
	if (m_fd<0) return;	// Not open?

	// Linux version:
	// Refer to:
	//  http://www.linuxjournal.com/article/8478
	//  http://inotify.aiken.cz/?section=common&page=home&lang=en
	struct timeval time;
	fd_set rfds;
	int ret;

	// timeout
	time.tv_sec = 0;
	time.tv_usec = 100;

	// zero-out the fd_set
	FD_ZERO (&rfds);

	// Add inotify fd
	FD_SET (m_fd, &rfds);

	ret = select (m_fd + 1, &rfds, NULL, NULL, &time);
	if (ret < 0)
	{
		perror ("[CFileSystemWatcher::getChanges] select");
		return;
	}

	else if (!ret)
	{
		// timed out!
	}
	else if (FD_ISSET (m_fd, &rfds))
	{
		// inotify events are available! : Read them!

		/* size of the event structure, not counting name */
		#define EVENT_SIZE  (sizeof (struct inotify_event))

		/* reasonable guess as to size of 1024 events */
		#define BUF_LEN        (1024 * (EVENT_SIZE + 16))

		char buf[BUF_LEN];
		int len, i = 0;

		len = read (m_fd, buf, BUF_LEN);
		if (len < 0)
		{
			if (errno == EINTR)
			{
				/* need to reissue system call */
			}
			else
				perror ("[CFileSystemWatcher::getChanges] read");
		}
		else if (!len)
		{
				/* BUF_LEN too small? */
		}

		while (i < len)
		{
			struct inotify_event event_val;
			::memcpy(&event_val, &buf[i], sizeof(event_val)); // Was: event = (struct inotify_event *) ;
			struct inotify_event *event = &event_val;

			i += EVENT_SIZE + event->len;

//			printf ("wd=%d mask=%u cookie=%u len=%u\n",event->wd, event->mask,event->cookie, event->len);

			string eventName;
			if (event->len) eventName = event->name;

			// Add event to output list:
			// ---------------------------------------------
			if ( 0==(event->mask & IN_UNMOUNT) &&
			     0==(event->mask & IN_Q_OVERFLOW) &&
			     0==(event->mask & IN_IGNORED) )
			{
				TFileSystemChange	newEntry;

				newEntry.path 				= m_watchedDirectory + eventName;
				newEntry.isDir 				= event->mask & IN_ISDIR;
				newEntry.eventModified		= event->mask & IN_MODIFY;
				newEntry.eventCloseWrite  	= event->mask & IN_CLOSE_WRITE;
				newEntry.eventDeleted  		= event->mask & IN_DELETE;
				newEntry.eventMovedTo  		= event->mask & IN_MOVED_TO;
				newEntry.eventMovedFrom  	= event->mask & IN_MOVED_FROM;
				newEntry.eventCreated		= event->mask & IN_CREATE;
				newEntry.eventAccessed		= event->mask & IN_ACCESS;

				out_list.push_back( newEntry );
			}
		}

	}
#	endif
#endif

}


#ifdef MRPT_OS_WINDOWS

void CFileSystemWatcher::thread_win32_watch()
{

	uint8_t buf[8*1024];
	DWORD dwRead=0;

	while(ReadDirectoryChangesW(
		HANDLE(m_hNotif),
		buf,
		sizeof(buf),
		false, // No subtree
		FILE_NOTIFY_CHANGE_FILE_NAME |
		FILE_NOTIFY_CHANGE_DIR_NAME |
		FILE_NOTIFY_CHANGE_ATTRIBUTES |
		FILE_NOTIFY_CHANGE_SIZE |
		FILE_NOTIFY_CHANGE_LAST_WRITE |
		FILE_NOTIFY_CHANGE_LAST_ACCESS |
		FILE_NOTIFY_CHANGE_CREATION,
		&dwRead,
		NULL,
		NULL))
	{
		// Interpret read data as FILE_NOTIFY_INFORMATION:
		// There might be several notifications in the same data block:
		size_t idx=0;
		for(;;)
		{
			// Yep... this is MS's idea of a beautiful and easy way to return data:
			FILE_NOTIFY_INFORMATION *fni = reinterpret_cast<FILE_NOTIFY_INFORMATION*>(&buf[idx]);

			// Extract the name (stored as a WCHAR*) into a UTF-8 std::string:
			ASSERTMSG_(fni->FileNameLength<10000,"Name length >10K... this is probably an error")

			int reqLen = WideCharToMultiByte(CP_UTF8,0,fni->FileName,fni->FileNameLength >> 1,NULL,0,NULL, NULL);
			std::vector<char> tmpBuf(reqLen);
			int actLen = WideCharToMultiByte(CP_UTF8,0,fni->FileName,fni->FileNameLength >> 1,&tmpBuf[0],tmpBuf.size(),NULL, NULL);
			ASSERTMSG_(actLen>0,"Error converting filename from WCHAR* to UTF8")

			const std::string filName(&tmpBuf[0],actLen);

			TFileSystemChange	newEntry;
			newEntry.path	= m_watchedDirectory + filName;
			newEntry.isDir 	= mrpt::system::directoryExists(newEntry.path);

			// Fill out the new entry:
			switch (fni->Action)
			{
				case FILE_ACTION_ADDED:
				{
					newEntry.eventCreated = true;
					m_queue_events_win32.push(new TFileSystemChange(newEntry));
				} break;
				case FILE_ACTION_REMOVED:
				{
					newEntry.eventDeleted = true;
					m_queue_events_win32.push(new TFileSystemChange(newEntry));
				} break;
				case FILE_ACTION_MODIFIED:
				{
					newEntry.eventModified = true;
					m_queue_events_win32.push(new TFileSystemChange(newEntry));
				} break;
				case FILE_ACTION_RENAMED_OLD_NAME:
				{
					newEntry.eventMovedFrom = true;
					m_queue_events_win32.push(new TFileSystemChange(newEntry));
				} break;
				case FILE_ACTION_RENAMED_NEW_NAME:
				{
					newEntry.eventMovedTo = true;
					m_queue_events_win32.push(new TFileSystemChange(newEntry));
				} break;
			}

			// Next entry?
			if (fni->NextEntryOffset>0)
				idx+=fni->NextEntryOffset;
			else break; // done
		}
	}

	printf("Done!");
}
#endif
