
#include <cvd/videofilebuffer.h>

#include <sstream>
using namespace std;
namespace CVD
{

using namespace Exceptions::VideoFileBuffer;
	
//
// EXCEPTIONS
//

Exceptions::VideoFileBuffer::FileOpen::FileOpen(const std::string& name, const string& error)
{
	what = "RawVideoFileBuffer: Error opening file \"" + name + "\": " + error;
}

Exceptions::VideoFileBuffer::BadFrameAlloc::BadFrameAlloc()
{
	what = "RawVideoFileBuffer: Unable to allocate video frame.";
}

Exceptions::VideoFileBuffer::BadDecode::BadDecode(double t, const string& s)
{
	ostringstream os;
	os << "RawVideoFileBuffer: Error decoding video frame at time " << t;
	
	if(s == "")
		os 	<< ".";
	else
		os << ": " << s;
	what = os.str();
}

Exceptions::VideoFileBuffer::EndOfFile::EndOfFile()
{
	what =  "RawVideoFileBuffer: Tried to read off the end of the file.";
}

Exceptions::VideoFileBuffer::BadSeek::BadSeek(double t, const string& s)
{
	ostringstream os;
	os << "RawVideoFileBuffer: Seek to time " << t << "s failed";

	
	if(s == "")
		os 	<< ".";
	else
		os << ": " << s;

	what = os.str();
}

}
