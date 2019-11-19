//-*- c++ -*-
#ifndef CVD_INC_CONNECTED_COMPONENTS_H
#define CVD_INC_CONNECTED_COMPONENTS_H

#include <cvd/image_ref.h>
#include <vector>

namespace CVD {

	///Find the connected components of the input, using 4-way floodfill.
	///This is implemented as the graph based algorithm. There is no restriction
	///on the input except that positions can not be INT_MIN or INT_MAX.
	///
	///The pixels in the resulting segments are not sorted.
	///@param v List of pixel positions
	///@param r List of segments.
	///@ingroup gVision
	void connected_components(const std::vector<ImageRef>& v, std::vector<std::vector<ImageRef> >& r);
}

#endif
