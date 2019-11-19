#include <cvd/fast_corner.h>
#include <cvd/nonmax_suppression.h>
#include "cvd_src/fast/prototypes.h"

using namespace CVD;
using namespace std;

namespace CVD
{

void fast_corner_detect_9_nonmax(const BasicImage<byte>& I, std::vector<ImageRef>& corners, int barrier)
{
	vector<ImageRef> c;
	vector<int> s;
	fast_corner_detect_9(I, c, barrier);
	fast_corner_score_9(I, c, barrier, s);
	nonmax_suppression(c, s, corners);
}

}
