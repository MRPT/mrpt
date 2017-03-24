/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef SIFT_H
#define SIFT_H

//#include "cxcore.h"
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#ifdef __cplusplus
 extern "C" {
#endif

/******************************** Structures *********************************/

/** holds feature data relevant to detection */
struct detection_data
{
	int r;
	int c;
	int octv;
	int intvl;
	double subintvl;
	double scl_octv;
};

struct feature;


/******************************* Defs and macros *****************************/

/** default number of sampled intervals per octave */
#define SIFT_INTVLS 3

/** default sigma for initial gaussian smoothing */
#define SIFT_SIGMA 1.6

/** default threshold on keypoint contrast |D(x)| */
//#define SIFT_CONTR_THR 0.04

/** default threshold on keypoint ratio of principle curvatures */
//#define SIFT_CURV_THR 10

/** double image size before pyramid construction? */
#define SIFT_IMG_DBL 1

/** default width of descriptor histogram array */
#define SIFT_DESCR_WIDTH 4

/** default number of bins per histogram in descriptor array */
#define SIFT_DESCR_HIST_BINS 8

/* assumed gaussian blur for input image */
#define SIFT_INIT_SIGMA 0.5

/* width of border in which to ignore keypoints */
#define SIFT_IMG_BORDER 5

/* maximum steps of keypoint interpolation before failure */
#define SIFT_MAX_INTERP_STEPS 5

/* default number of bins in histogram for orientation assignment */
#define SIFT_ORI_HIST_BINS 36

/* determines gaussian sigma for orientation assignment */
#define SIFT_ORI_SIG_FCTR 1.5

/* determines the radius of the region used in orientation assignment */
#define SIFT_ORI_RADIUS 3.0 * SIFT_ORI_SIG_FCTR

/* number of passes of orientation histogram smoothing */
#define SIFT_ORI_SMOOTH_PASSES 2

/* orientation magnitude relative to max that results in new feature */
#define SIFT_ORI_PEAK_RATIO 0.8

/* determines the size of a single descriptor orientation histogram */
#define SIFT_DESCR_SCL_FCTR 3.0

/* threshold on magnitude of elements of descriptor vector */
#define SIFT_DESCR_MAG_THR 0.2

/* factor used to convert floating-point descriptor to unsigned char */
#define SIFT_INT_DESCR_FCTR 512.0

/* returns a feature's detection data */
#define feat_detection_data(f) ( (struct detection_data*)(f->feature_data) )


/*************************** Function Prototypes *****************************/

/**
Finds SIFT features in an image using default parameter values.  All
detected features are stored in the array pointed to by \a feat.

@param img the image in which to detect features
@param feat a pointer to an array in which to store detected features

@return Returns the number of features stored in \a feat or -1 on failure
@see _sift_features()
*/
extern int sift_features( IplImage* img, struct feature** feat );



/**
Finda SIFT features in an image using user-specified parameter values.  All
detected features are stored in the array pointed to by \a feat.

@param img the image in which to detect features
@param feat a pointer to an array in which to store detected features
@param intvls the number of intervals sampled per octave of scale space
@param sigma the amount of Gaussian smoothing applied to each image level
	before building the scale space representation for an octave
@param contr_thr a threshold on the value of the scale space function
	\f$\left|D(\hat{x})\right|\f$, where \f$\hat{x}\f$ is a vector specifying
	feature location and scale, used to reject unstable features;  assumes
pixel values in the range [0, 1]
@param curv_thr threshold on a feature's ratio of principle curvatures
	used to reject features that are too edge-like
@param img_dbl should be 1 if image doubling prior to scale space
	construction is desired or 0 if not
@param descr_width the width, \f$n\f$, of the \f$n \times n\f$ array of
	orientation histograms used to compute a feature's descriptor
@param descr_hist_bins the number of orientations in each of the
	histograms in the array used to compute a feature's descriptor

@return Returns the number of keypoints stored in \a feat or -1 on failure
@see sift_features()
*/
extern int _sift_features( IplImage* img, struct feature** feat, int intvls,
						  double sigma, double contr_thr, int curv_thr,
						  int img_dbl, int descr_width, int descr_hist_bins );


#ifdef __cplusplus
  }
#endif

#endif
