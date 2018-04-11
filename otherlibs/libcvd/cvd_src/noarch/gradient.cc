#include "cvd/vision.h"

using namespace std;

namespace CVD {

    void gradient(const BasicImage<byte>& im, BasicImage<short[2]>& out)
    {
		if( im.size() != out.size())
			throw Exceptions::Vision::IncompatibleImageSizes("gradient");
		gradient<byte,short[2]>(im,out);
    }
};
