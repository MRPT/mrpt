#include <cvd/fast_corner.h>
#include "cvd_src/fast/prototypes.h"

namespace CVD
{
	void fast_corner_detect_7(const BasicImage<byte>& i, std::vector<ImageRef>& corners, int b)
	{
		fast_corner_detect_plain_7(i, corners, b);
	}
}
