#include "cvd/vision.h"
namespace CVD{
	void twoThirdsSample(const BasicImage<byte>& in, BasicImage<byte>& out)
	{
		twoThirdsSample<byte>(in, out);
	}
}
