#ifndef CVD_INCLUDE_EDGE_HPP
#define CVD_INCLUDE_EDGE_HPP

#include <cvd/image.h>
#include <cvd/convolution.h>
#include <TooN/TooN.h>
#include <cmath>
#include <queue>

namespace CVD {

 	
  template <class T>
  void canny_gradient(const BasicImage <T> &im, BasicImage <TooN::Vector<2> > &grad, const double sigma = 1.0)
  {
  	  Image<T> out(im.size());
	  convolveGaussian(im, out, sigma);

	  for(int y=1; y < im.size().y-1; y++) {
		  for(int x=1; x < im.size().x-1; x++) {
			  const double xgrad = ((out[y-1][x-1] + 2 * out[y][x-1] + out[y+1][x-1]) - (out[y-1][x+1] + 2 * out[y][x+1] + out[y+1][x+1]));
			  const double ygrad = ((out[y-1][x-1] + 2 * out[y-1][x] + out[y-1][x+1]) - (out[y+1][x-1] + 2 * out[y+1][x] + out[y+1][x+1]));
			  grad[y][x][0] = xgrad;
			  grad[y][x][1] = ygrad;
		  }
	  }
  }

  
  template <class T>
  void canny(const BasicImage <T> &im, BasicImage <T> &out, const double sigma = 1.0, const double lower_threshold = 0.1, const double upper_threshold = 0.2) {
    //GRANTA_ASSERT(im.size() == out.size());
    const int STRONG_EDGE = 4;
    const int WEAK_EDGE = 5;
    const int REJECT_EDGE = 6;
    const double pi = std::atan(1.0)*4;
    Image <double> mags(out.size());
    Image <byte> bins(out.size());
    convolveGaussian(im, out, sigma);
    /*
      Compute the gradients, the gradient directions, and bin each directions into 4 discrete bins:

      * horizontal (id=0): angles in [-22.5, 22.5) (midangle 0) and [157.5, 202.5) (midangle 180)
      * diagonal (id=1): angles in [22.5, 67.5) (midangle 45)
      * vertical (id=2): angles in [67.5, 112.5) (midangle 90)
      * anti-diagonal (id=3): angles in [112.5, 157.5) (midangle 135)

      For each pixel location, the bin id of its pixel's discretized direction is stored in bins[y][x].
      Additionally, the magnitude is stored in mags[y][x] for the non-maximal suppression step.
     */
    for(int y=1; y < im.size().y-1; y++) {
      for(int x=1; x < im.size().x-1; x++) {
        const double ygrad = ((out[y-1][x-1] + 2 * out[y-1][x] + out[y-1][x+1]) -
                              (out[y+1][x-1] + 2 * out[y+1][x] + out[y+1][x+1]));
        const double xgrad = ((out[y-1][x-1] + 2 * out[y][x-1] + out[y+1][x-1]) -
                              (out[y-1][x+1] + 2 * out[y][x+1] + out[y+1][x+1]));
        const double mag = std::sqrt(xgrad * xgrad + ygrad * ygrad);
        mags[y][x] = mag;
        if (mag >= lower_threshold) {
          double dir = std::atan(ygrad/xgrad);
          if (dir < 0) {
            dir += pi;
          }
          dir = (180.0 * dir) / pi;
          int dirbin = static_cast<int>((dir + 22.5) / 45.0) % 4;
          bins[y][x] = dirbin;
        }
        else {
          bins[y][x] = REJECT_EDGE;
        }
      }
    }
    //cout << "Total time gradient: " << timer.get_time() << endl;

    /*
      Perform non-maximal suppression, rejecting any pixel with a gradient magnitude less than its
      neighbors (along the gradient direction). 
     */
    std::queue<ImageRef> edge_locations;

    /* Perform non-maximal suppression along the gradient direction and also perform
       hysterisis thresholding. */
    for(int y=2; y < im.size().y-2; y++) {
      for(int x=2; x < im.size().x-2; x++) {
        out[y][x] = 0.0;
        const double mag = mags[y][x];

        /* Skip rejected pixels. */
        if (bins[y][x] == REJECT_EDGE) {
          continue;
        }
        bool edge = false;       
        /* If a pixel is an edge then a pixel has a higher gradient
           magnitude than its neighbors along the gradient direction
           (not an if and only if). Reject those pixels that are not
           local maxima along the gradient direction. */
        switch (bins[y][x]) {
        case 0: /* horizontal gradient direction */
          if (mag > mags[y][x-1] && mag > mags[y][x+1]) {
            edge = true;
          }
          break;
        case 1: /* diagonal gradient direction */
          if (mag > mags[y-1][x-1] && mag > mags[y+1][x+1]) {
            edge = true;
          }
          break;
        case 2: /* vertical gradient direction */
          if (mag > mags[y-1][x] && mag > mags[y+1][x]) {
            edge = true;
          }
          break;
        case 3: /* anti-diagonal gradient direction */
          if (mag > mags[y+1][x-1] && mag > mags[y-1][x+1]) {
            edge = true;
          }
          break;
        }
        /*
          If the magnitude is greater than the upper threshold, push it onto the queue so it
          can be traced later.
         */
        if (edge && mag > upper_threshold) {
          bins[y][x] = STRONG_EDGE; // strong edge
          edge_locations.push(ImageRef(x, y));
        }
        /*
          Otherwise, if it's above the lower threshold, mark it as weak.
         */
        else if (edge && mag > lower_threshold) {
          bins[y][x] = WEAK_EDGE; // weak edge
        }
        /*
          Since we've already rejected weak edges, we shouldn't get here.
         */
        else {
          bins[y][x] = REJECT_EDGE; // reject edge
        }
      }
    }
    //cout << "total time hysteris: " << timer.get_time() << endl;

    /* While there are still unvisited accepted (strong) edge locations... */
    while (edge_locations.size() > 0) {
      /* Pop the next strong edge. */
      ImageRef loc(edge_locations.front());
      edge_locations.pop();

      /* Record its magnitude. */
      out[loc] = mags[loc];

      /* Mark any weak edges connected to it as strong and push them onto the stack.
         These edges will never be pushed twice since only weak edges can be pushed
         onto the stack at this point.*/
      if (bins[loc.y][loc.x] == STRONG_EDGE) {        
        if (bins[loc.y-1][loc.x-1] == WEAK_EDGE) {
          bins[loc.y-1][loc.x-1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x-1,loc.y-1));
        }
        if (bins[loc.y-1][loc.x] == WEAK_EDGE) {
          bins[loc.y-1][loc.x] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x,loc.y-1));
        }
        if (bins[loc.y-1][loc.x+1] == WEAK_EDGE) {
          bins[loc.y-1][loc.x+1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x+1,loc.y-1));
        }
        if (bins[loc.y][loc.x-1] == WEAK_EDGE) {
          bins[loc.y][loc.x-1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x-1,loc.y));
        }
        if (bins[loc.y][loc.x] == WEAK_EDGE) {
          bins[loc.y][loc.x] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x,loc.y));
        }
        if (bins[loc.y][loc.x+1] == WEAK_EDGE) {
          bins[loc.y][loc.x+1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x+1,loc.y));
        }
        if (bins[loc.y+1][loc.x-1] == WEAK_EDGE) {
          bins[loc.y+1][loc.x-1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x-1,loc.y+1));
        }
        if (bins[loc.y+1][loc.x] == WEAK_EDGE) {
          bins[loc.y+1][loc.x] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x,loc.y+1));
        }
        if (bins[loc.y+1][loc.x+1] == WEAK_EDGE) {
          bins[loc.y+1][loc.x+1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x+1,loc.y+1));
        }
      }
    }
    //cout << "total time tracing: " << timer.get_time() << endl;
  }


  template <class T>
  void canny2(const BasicImage <T> &im, BasicImage <T> &out, double sigma = 1.0, double lower_threshold = 0.1, double upper_threshold = 0.2) {
    //GRANTA_ASSERT(im.size() == out.size());
    const int STRONG_EDGE = 4;
    const int WEAK_EDGE = 5;
    const int REJECT_EDGE = 6;
    Image <double> mags(out.size());
    Image <byte> bins(out.size());
    convolveGaussian(im, out, sigma);
    lower_threshold = lower_threshold * lower_threshold;
    upper_threshold = upper_threshold * upper_threshold;
    /*
      Compute the gradients, the gradient directions, and bin each directions into 4 discrete bins:

      * horizontal (id=0): angles in [-22.5, 22.5) (midangle 0) and [157.5, 202.5) (midangle 180)
      * diagonal (id=1): angles in [22.5, 67.5) (midangle 45)
      * vertical (id=2): angles in [67.5, 112.5) (midangle 90)
      * anti-diagonal (id=3): angles in [112.5, 157.5) (midangle 135)

      For each pixel location, the bin id of its pixel's discretized direction is stored in bins[y][x].
      Additionally, the magnitude is stored in mags[y][x] for the non-maximal suppression step.
     */
    for(int y=1; y < im.size().y-1; y++) {
      for(int x=1; x < im.size().x-1; x++) {
        const double ygrad = ((out[y-1][x-1] + 2 * out[y-1][x] + out[y-1][x+1]) -
                              (out[y+1][x-1] + 2 * out[y+1][x] + out[y+1][x+1]));
        const double xgrad = ((out[y-1][x-1] + 2 * out[y][x-1] + out[y+1][x-1]) -
                              (out[y-1][x+1] + 2 * out[y][x+1] + out[y+1][x+1]));
        const double axgrad = std::abs(xgrad);
        const double aygrad = std::abs(ygrad);
        //const double mag = std::sqrt(xgrad * xgrad + ygrad * ygrad);
        const double mag = xgrad * xgrad + ygrad * ygrad;
        mags[y][x] = mag;
        if (mag >= lower_threshold) {
          const bool xgrad_pos = xgrad >= 0;
          const bool ygrad_pos = ygrad >= 0;
          const bool xgrad_neg = xgrad <= 0;
          const bool ygrad_neg = ygrad <= 0;
          const bool axgrad_sup = axgrad >= aygrad;
          const bool aygrad_sup = axgrad <= aygrad;
          if ((xgrad_pos && ygrad_pos && axgrad_sup)
              || (xgrad_neg && ygrad_neg && axgrad_sup)) {
            bins[y][x] = 0;
          }
          else if ((xgrad_pos && ygrad_pos && aygrad_sup)
                   || (xgrad_neg && ygrad_neg && aygrad_sup)) {
            bins[y][x] = 1;
          }
          else if ((xgrad_neg && ygrad_pos && aygrad_sup)
                   || (xgrad_pos && ygrad_neg && aygrad_sup)) {
            bins[y][x] = 2;
          }
          else if ((xgrad_neg && ygrad_pos && axgrad_sup)
                   || (xgrad_pos && ygrad_neg && axgrad_sup)) {
            bins[y][x] = 3;
          }
        }
        else {
          bins[y][x] = REJECT_EDGE;
        }
      }
    }
    //cout << "Total time gradient: " << timer.get_time() << endl;

    /*
      Perform non-maximal suppression, rejecting any pixel with a gradient magnitude less than its
      neighbors (along the gradient direction). 
     */
    std::queue<ImageRef> edge_locations;


    for(int y=2; y < im.size().y-2; y++) {
      out[y][0] = 0;
      out[y][1] = 0;
      out[y][im.size().x-2] = 0;
      out[y][im.size().x-1] = 0;
    }

    for(int x=1; x < im.size().x-1; x++) {
      out[0][x] = 0;
      out[1][x] = 0;
      out[im.size().y-2][x] = 0;
      out[im.size().y-1][x] = 0;
    }
    
    /* Perform non-maximal suppression along the gradient direction and also perform
       hysterisis thresholding. */
    for(int y=2; y < im.size().y-2; y++) {
      for(int x=2; x < im.size().x-2; x++) {
        out[y][x] = 0.0;
        const double mag = mags[y][x];
        /* Skip rejected pixels. */
        if (bins[y][x] == REJECT_EDGE) {
          continue;
        }
        bool edge = false;       
        /* If a pixel is an edge then a pixel has a higher gradient
           magnitude than its neighbors along the gradient direction
           (not an if and only if). Reject those pixels that are not
           local maxima along the gradient direction. */
        switch (bins[y][x]) {
        case 0: /* horizontal gradient direction */
          if (mag > mags[y][x-1] && mag > mags[y][x+1]) {
            edge = true;
          }
          break;
        case 1: /* diagonal gradient direction */
          if (mag > mags[y-1][x-1] && mag > mags[y+1][x+1]) {
            edge = true;
          }
          break;
        case 2: /* vertical gradient direction */
          if (mag > mags[y-1][x] && mag > mags[y+1][x]) {
            edge = true;
          }
          break;
        case 3: /* anti-diagonal gradient direction */
          if (mag > mags[y+1][x-1] && mag > mags[y-1][x+1]) {
            edge = true;
          }
          break;
        }
        /*
          If the magnitude is greater than the upper threshold, push it onto the queue so it
          can be traced later.
         */
        if (edge && mag > upper_threshold) {
          bins[y][x] = STRONG_EDGE; // strong edge
          edge_locations.push(ImageRef(x, y));
        }
        /*
          Otherwise, if it's above the lower threshold, mark it as weak.
         */
        else if (edge && mag > lower_threshold) {
          bins[y][x] = WEAK_EDGE; // weak edge
        }
        /*
          Since we've already rejected weak edges, we shouldn't get here.
         */
        else {
          bins[y][x] = REJECT_EDGE; // reject edge
        }
      }
    }
    //cout << "total time hysteris: " << timer.get_time() << endl;

    /* While there are still unvisited accepted (strong) edge locations... */
    while (edge_locations.size() > 0) {
      /* Pop the next strong edge. */
      ImageRef loc(edge_locations.front());
      edge_locations.pop();

      /* Record its magnitude. */
      out[loc] = mags[loc];

      /* Mark any weak edges connected to it as strong and push them onto the stack.
         These edges will never be pushed twice since only weak edges can be pushed
         onto the stack at this point.*/
      if (bins[loc.y][loc.x] == STRONG_EDGE) {        
        if (bins[loc.y-1][loc.x-1] == WEAK_EDGE) {
          bins[loc.y-1][loc.x-1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x-1,loc.y-1));
        }
        if (bins[loc.y-1][loc.x] == WEAK_EDGE) {
          bins[loc.y-1][loc.x] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x,loc.y-1));
        }
        if (bins[loc.y-1][loc.x+1] == WEAK_EDGE) {
          bins[loc.y-1][loc.x+1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x+1,loc.y-1));
        }
        if (bins[loc.y][loc.x-1] == WEAK_EDGE) {
          bins[loc.y][loc.x-1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x-1,loc.y));
        }
        if (bins[loc.y][loc.x] == WEAK_EDGE) {
          bins[loc.y][loc.x] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x,loc.y));
        }
        if (bins[loc.y][loc.x+1] == WEAK_EDGE) {
          bins[loc.y][loc.x+1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x+1,loc.y));
        }
        if (bins[loc.y+1][loc.x-1] == WEAK_EDGE) {
          bins[loc.y+1][loc.x-1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x-1,loc.y+1));
        }
        if (bins[loc.y+1][loc.x] == WEAK_EDGE) {
          bins[loc.y+1][loc.x] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x,loc.y+1));
        }
        if (bins[loc.y+1][loc.x+1] == WEAK_EDGE) {
          bins[loc.y+1][loc.x+1] = STRONG_EDGE;
          edge_locations.push(ImageRef(loc.x+1,loc.y+1));
        }
      }
    }
    //cout << "total time tracing: " << timer.get_time() << endl;
  }
  
  /*
    Takes in an arbitray edge map (in) and a mask. For each pixel,
    if the mask is above a threshold (threshold), it computes the
    gradient direction for that pixel, storing the result in grad_dir_map.
   */
  template <class T>
  void compute_gradient_direction_at(const BasicImage <T> &in, const BasicImage <T> &mask, const double threshold,
                                     BasicImage <T> &grad_dir_map) {
    zero_border(grad_dir_map);
    for(int y=1; y < in.size().y-1; y++) {
      for(int x=1; x < in.size().x-1; x++) {
        if (mask[y][x] > threshold) {
          const double ygrad = ((in[y-1][x-1] + 2 * in[y-1][x] + in[y-1][x+1]) -
                                (in[y+1][x-1] + 2 * in[y+1][x] + in[y+1][x+1]));
          const double xgrad = ((in[y-1][x-1] + 2 * in[y][x-1] + in[y+1][x-1]) -
                                (in[y-1][x+1] + 2 * in[y][x+1] + in[y+1][x+1]));
          double dir = std::atan2(xgrad, ygrad);
          grad_dir_map[y][x] = dir;
        }
        else {
          grad_dir_map[y][x] = 0.0;
        }
      }
    }
  }
}
#endif
