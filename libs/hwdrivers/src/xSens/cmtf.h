#ifndef _CMT_FILE_DEF_H_2007_01_24
#define _CMT_FILE_DEF_H_2007_01_24

#include <stdio.h>
#include <mrpt/utils/mrpt_stdint.h>

#ifndef _WIN32
#   include <sys/types.h>
#endif

#ifdef _CMT_STATIC_LIB
namespace xsens {
#endif

#ifdef _WIN32
	typedef  __int64	CmtFilePos;
#else
	typedef  off_t		CmtFilePos;
#endif

#ifdef _CMT_STATIC_LIB
}	// end of xsens namespace
#endif

// setFilePos defines
#ifdef _WIN32
#	define CMT_FILEPOS_BEGIN				FILE_BEGIN
#	define CMT_FILEPOS_CURRENT				FILE_CURRENT
#	define CMT_FILEPOS_END					FILE_END
#else
#	define CMT_FILEPOS_BEGIN				SEEK_SET
#	define CMT_FILEPOS_CURRENT				SEEK_CUR
#	define CMT_FILEPOS_END					SEEK_END
#endif

#endif
