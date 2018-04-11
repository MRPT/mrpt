#ifndef __BREZENHAM_H
#define __BREZENHAM_H

#include <TooN/TooN.h>
#include <cvd/image_ref.h>

namespace CVD {


/// Class to implement the Bresenham line-drawing algorithm.
/// This object does not draw directly into an image, it simply outputs the set
/// of image co-ordinates that should be visited to draw a line in a certain
/// direction. Pixels are generated in a 4-connected sense (i.e. there are 
/// no diagonal steps--each step is either horizontal or vertical)
/// details of the algorithm. See also Brezenham8.
/// @ingroup gGraphics
class Brezenham {
 public:
 /// Construct a line-drawing object
 /// @param dir The (x,y) direction of the line
  Brezenham(TooN::Vector<2> dir);

  /// Returns the next image co-ordinate along the line
  ImageRef step();

 private:
  double residual;
  double val1;
  double val2;
  ImageRef step1;
  ImageRef step2;
};


/// Class to implement the Bresenham line-drawing algorithm. 
/// This object does not draw directly into an image, it simply outputs the set
/// of image co-ordinates that should be visited to draw a line in a certain
/// direction. Pixels are generated in a 8-connected sense (i.e. diagonal steps
/// are possible). See also Brezenham.
/// @ingroup gGraphics
class Brezenham8 {
 public:
  /// Construct a line-drawing object
  /// @param dir The (x,y) direction of the line
  Brezenham8(TooN::Vector<2> dir);

  /// Returns the next image co-ordinate along the line
  ImageRef step();

  /// Which compass position most orthogonal to the line's direction?
  /// At each pixel given by 
  /// step() you are guaranteed to be able to walk in the direction given
  /// by sideways and not walk on the line.
  ImageRef sideways() {return my_sideways;}

 private:
  double residual;
  double val1;
  double val2;
  ImageRef step1;
  ImageRef step2;
  ImageRef my_sideways;
};

}

#endif
