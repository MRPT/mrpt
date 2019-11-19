#include <cvd/timeddiskbuffer.h>

CVD::Exceptions::TimedDiskBuffer::IncompatibleListLengths::IncompatibleListLengths()
{
	what = "TimedDiskBuffer received file and timestamp lists of different lengths.";
}
