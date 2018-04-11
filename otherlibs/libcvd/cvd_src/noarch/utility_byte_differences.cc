#include "cvd/utility.h"

namespace CVD {

    void differences(const byte* a, const byte* b, short* diff, unsigned int count) {
		differences<byte, short>(a, b, diff, count);
    }
    
    void differences(const short* a, const short* b, short* diff, unsigned int count) {
		differences<short, short>(a, b, diff, count);
    }
}
