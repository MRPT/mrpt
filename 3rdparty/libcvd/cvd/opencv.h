#ifndef LIBCVD_OPENCV_H
#define LIBCVD_OPENCV_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/rgba.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace CVD {
   namespace Internal{
		
		template<class C> struct opencv_traits
		{
			static const int depth = cv::DataDepth<C>::value;
			static const int channels=1;
		};

		template<class C> struct opencv_traits<Rgb<C> >: public opencv_traits<C>
		{
			static const int channels=3;
		};

		template<class C> struct opencv_traits<Rgba<C> >: public opencv_traits<C>
		{
			static const int channels=4;
		};

		template<class C> struct opencv_type
		{	
			static const int type = CV_MAKETYPE(opencv_traits<C>::depth, opencv_traits<C>::channels);
		};
   };


  template <class T>
  cv::Mat toMat(const CVD::BasicImage<T> &img) {
    return cv::Mat(img.size().y, img.size().x, Internal::opencv_type<T>::type, (void*)img.data(), img.row_stride() * sizeof(T));
  }

  template <class T>
  void equalizeHist(const CVD::BasicImage<T> &in, CVD::BasicImage<T> &out) {
    equalizeHist(toMat(in), toMat(out));
  }
}
#endif
