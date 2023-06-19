#pragma once

#include <mrpt/serialization/CArchive.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>

namespace mrpt
{
namespace serialization
{

// Manual instantiations: pybind11 / binder do not generate this automatically:
inline CArchiveStreamBase<mrpt::io::CFileGZInputStream> archiveFrom(mrpt::io::CFileGZInputStream& s)
{
	return CArchiveStreamBase<mrpt::io::CFileGZInputStream>(s);
}

inline CArchiveStreamBase<mrpt::io::CFileGZOutputStream> archiveFrom(mrpt::io::CFileGZOutputStream& s)
{
	return CArchiveStreamBase<mrpt::io::CFileGZOutputStream>(s);
}

inline CArchiveStreamBase<mrpt::io::CFileInputStream> archiveFrom(mrpt::io::CFileInputStream& s)
{
	return CArchiveStreamBase<mrpt::io::CFileInputStream>(s);
}

inline CArchiveStreamBase<mrpt::io::CFileOutputStream> archiveFrom(mrpt::io::CFileOutputStream& s)
{
	return CArchiveStreamBase<mrpt::io::CFileOutputStream>(s);
}


}
}