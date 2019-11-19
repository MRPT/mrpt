#include "cvd/vision.h"


namespace CVD
{
	void halfSample(const BasicImage<byte>& in, BasicImage<byte>& out)
	{   
		halfSample<byte>(in, out);
	}
}
