// -*- c++ -*-
#ifndef __DVFRAME_H
#define __DVFRAME_H

#include <cvd/videoframe.h>
#include <cvd/byte.h>

namespace CVD {

/// A frame from a Firewire (IEEE 1394) camera via DVBuffer2.
/// Frames are 8-bit greyscale, using the byte type.
/// @ingroup gVideoFrame	
class DVFrame : public VideoFrame<byte> {
public:
	/// (Used internally) Construct a DVFrame.
	/// @param s The image size
	/// @param t The time of this frame
	/// @param buff The buffer number
	/// @param dptr The image data
  DVFrame(ImageRef s, timeval t, int buff, byte* dptr):
    VideoFrame<byte>(t.tv_sec + 0.000001*t.tv_usec, dptr, s)
  {
    //my_size = s;
    my_buffer = buff;
    //my_data = dptr;
  }

  ~DVFrame(){my_data=0;}

  int my_buffer; ///< The buffer number
};

}

#endif
