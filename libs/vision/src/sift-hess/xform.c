/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xform.h"
#include "imgfeatures.h"
#include "utils.h"

//#include "cxcore.h"
//#include <highgui.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <gsl/gsl_sf.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <time.h>

/************************* Local Function Prototypes *************************/

static __inline struct feature* get_match( struct feature*, int );
int get_matched_features( struct feature*, int, int, struct feature*** );
int calc_min_inliers( int, int, double, double );
struct feature** draw_ransac_sample( struct feature**, int, int, gsl_rng* );
void extract_corresp_pts( struct feature**, int, int, CvPoint2D64f**,
						 CvPoint2D64f** );
int find_consensus( struct feature**, int, int, CvMat*, ransac_err_fn,
				   double, struct feature*** );
static __inline void release_mem( CvPoint2D64f*, CvPoint2D64f*,
struct feature** );

/********************** Functions prototyped in xform.h **********************/


/*
Calculates a best-fit image transform from image feature correspondences
using RANSAC.

For more information refer to:

Fischler, M. A. and Bolles, R. C.  Random sample consensus: a paradigm for
model fitting with applications to image analysis and automated cartography.
<EM>Communications of the ACM, 24</EM>, 6 (1981), pp. 381--395.

@param features an array of features; only features with a non-NULL match
	of type mtype are used in homography computation
@param n number of features in feat
@param mtype determines which of each feature's match fields to use
	for model computation; should be one of FEATURE_FWD_MATCH,
	FEATURE_BCK_MATCH, or FEATURE_MDL_MATCH; if this is FEATURE_MDL_MATCH,
	correspondences are assumed to be between a feature's img_pt field
	and its match's mdl_pt field, otherwise correspondences are assumed to
	be between the the feature's img_pt field and its match's img_pt field
@param xform_fn pointer to the function used to compute the desired
	transformation from feature correspondences
@param m minimum number of correspondences necessary to instantiate the
	model computed by xform_fn
@param p_badxform desired probability that the final transformation
	returned by RANSAC is corrupted by outliers (i.e. the probability that
	no samples of all inliers were drawn)
@param err_fn pointer to the function used to compute a measure of error
	between putative correspondences and a computed model
@param err_tol correspondences within this distance of a computed model are
	considered as inliers
@param inliers if not NULL, output as an array of pointers to the final
	set of inliers
@param n_in if not NULL and \a inliers is not NULL, output as the final
	number of inliers

@return Returns a transformation matrix computed using RANSAC or NULL
	on error or if an acceptable transform could not be computed.
*/
CvMat* ransac_xform( struct feature* features, int n, int mtype,
					ransac_xform_fn xform_fn, int m, double p_badxform,
					ransac_err_fn err_fn, double err_tol,
struct feature*** inliers, int* n_in )
{
	struct feature** matched, ** sample, ** consensus, ** consensus_max = NULL;
	struct ransac_data* rdata;
	CvPoint2D64f* pts, * mpts;
	CvMat* M = NULL;
	gsl_rng* rng;
	double p, in_frac = RANSAC_INLIER_FRAC_EST;
	int i, nm, in, in_min, in_max = 0, k = 0;

	nm = get_matched_features( features, n, mtype, &matched );
	if( nm < m )
	{
		fprintf( stderr, "Warning: not enough matches to compute xform, %s" \
			" line %d\n", __FILE__, __LINE__ );
		goto end;
	}

	/* initialize random number generator */
	rng = gsl_rng_alloc( gsl_rng_mt19937 );
	gsl_rng_set( rng, time(NULL) );

	in_min = calc_min_inliers( nm, m, RANSAC_PROB_BAD_SUPP, p_badxform );
	p = pow( 1.0 - pow( in_frac, m ), k );
	i = 0;
	while( p > p_badxform )
	{
		sample = draw_ransac_sample( matched, nm, m, rng );
		extract_corresp_pts( sample, m, mtype, &pts, &mpts );
		M = xform_fn( pts, mpts, m );
		if( ! M )
			goto iteration_end;
		in = find_consensus( matched, nm, mtype, M, err_fn, err_tol, &consensus);
		if( in > in_max )
		{
			if( consensus_max )
				free( consensus_max );
			consensus_max = consensus;
			in_max = in;
			in_frac = (double)in_max / nm;
		}
		else
			free( consensus );
		cvReleaseMat( &M );

iteration_end:
		release_mem( pts, mpts, sample );
		p = pow( 1.0 - pow( in_frac, m ), ++k );
	}

	/* calculate final transform based on best consensus set */
	if( in_max >= in_min )
	{
		extract_corresp_pts( consensus_max, in_max, mtype, &pts, &mpts );
		M = xform_fn( pts, mpts, in_max );
		in = find_consensus( matched, nm, mtype, M, err_fn, err_tol, &consensus);
		cvReleaseMat( &M );
		release_mem( pts, mpts, consensus_max );
		extract_corresp_pts( consensus, in, mtype, &pts, &mpts );
		M = xform_fn( pts, mpts, in );
		if( inliers )
		{
			*inliers = consensus;
			consensus = NULL;
		}
		if( n_in )
			*n_in = in;
		release_mem( pts, mpts, consensus );
	}
	else if( consensus_max )
	{
		if( inliers )
			*inliers = NULL;
		if( n_in )
			*n_in = 0;
		free( consensus_max );
	}

	gsl_rng_free( rng );
end:
	for( i = 0; i < nm; i++ )
	{
		rdata = feat_ransac_data( matched[i] );
		matched[i]->feature_data = rdata->orig_feat_data;
		free( rdata );
	}
	free( matched );
	return M;
}



/*
Calculates a least-squares planar homography from point correspondeces.

@param pts array of points
@param mpts array of corresponding points; each pts[i], i=0..n-1, corresponds
	to mpts[i]
@param n number of points in both pts and mpts; must be at least 4

@return Returns the 3 x 3 least-squares planar homography matrix that
	transforms points in pts to their corresponding points in mpts or NULL if
	fewer than 4 correspondences were provided
*/
CvMat* lsq_homog( CvPoint2D64f* pts, CvPoint2D64f* mpts, int n )
{
	CvMat* H, * A, * B, X;
	double x[9];
	int i;

	if( n < 4 )
	{
		fprintf( stderr, "Warning: too few points in lsq_homog(), %s line %d\n",
			__FILE__, __LINE__ );
		return NULL;
	}

	/* set up matrices so we can unstack homography into X; AX = B */
	A = cvCreateMat( 2*n, 8, CV_64FC1 );
	B = cvCreateMat( 2*n, 1, CV_64FC1 );
	X = cvMat( 8, 1, CV_64FC1, x );
	H = cvCreateMat(3, 3, CV_64FC1);
	cvZero( A );
	for( i = 0; i < n; i++ )
	{
		cvmSet( A, i, 0, pts[i].x );
		cvmSet( A, i+n, 3, pts[i].x );
		cvmSet( A, i, 1, pts[i].y );
		cvmSet( A, i+n, 4, pts[i].y );
		cvmSet( A, i, 2, 1.0 );
		cvmSet( A, i+n, 5, 1.0 );
		cvmSet( A, i, 6, -pts[i].x * mpts[i].x );
		cvmSet( A, i, 7, -pts[i].y * mpts[i].x );
		cvmSet( A, i+n, 6, -pts[i].x * mpts[i].y );
		cvmSet( A, i+n, 7, -pts[i].y * mpts[i].y );
		cvmSet( B, i, 0, mpts[i].x );
		cvmSet( B, i+n, 0, mpts[i].y );
	}
	cvSolve( A, B, &X, CV_SVD );
	x[8] = 1.0;
	X = cvMat( 3, 3, CV_64FC1, x );
	cvConvert( &X, H );

	cvReleaseMat( &A );
	cvReleaseMat( &B );
	return H;
}



/*
Calculates the transfer error between a point and its correspondence for
a given homography, i.e. for a point x, it's correspondence x', and
homography H, computes d(x', Hx)^2.

@param pt a point
@param mpt pt's correspondence
@param H a homography matrix

@return Returns the transfer error between pt and mpt given H
*/
double homog_xfer_err( CvPoint2D64f pt, CvPoint2D64f mpt, CvMat* H )
{
	CvPoint2D64f xpt = persp_xform_pt( pt, H );

	return sqrt( dist_sq_2D( xpt, mpt ) );
}



/*
Performs a perspective transformation on a single point.  That is, for a
point (x, y) and a 3 x 3 matrix T this function returns the point
(u, v), where

[x' y' w']^T = T * [x y 1]^T,

and

(u, v) = (x'/w', y'/w').

Note that affine transforms are a subset of perspective transforms.

@param pt a 2D point
@param T a perspective transformation matrix

@return Returns the point (u, v) as above.
*/
CvPoint2D64f persp_xform_pt( CvPoint2D64f pt, CvMat* T )
{
	CvMat XY, UV;
	double xy[3] = { pt.x, pt.y, 1.0 }, uv[3] = { 0 };
	CvPoint2D64f rslt;

	cvInitMatHeader( &XY, 3, 1, CV_64FC1, xy, CV_AUTOSTEP );
	cvInitMatHeader( &UV, 3, 1, CV_64FC1, uv, CV_AUTOSTEP );
	cvMatMul( T, &XY, &UV );
	rslt = cvPoint2D64f( uv[0] / uv[2], uv[1] / uv[2] );

	return rslt;
}


/************************ Local funciton definitions *************************/

/*
Returns a feature's match according to a specified match type

@param feat feature
@param mtype match type, one of FEATURE_FWD_MATCH, FEATURE_BCK_MATCH, or
FEATURE_MDL_MATCH

@return Returns feat's match corresponding to mtype or NULL for bad mtype
*/
static __inline struct feature* get_match( struct feature* feat, int mtype )
{
	if( mtype == FEATURE_MDL_MATCH )
		return feat->mdl_match;
	if( mtype == FEATURE_BCK_MATCH )
		return feat->bck_match;
	if( mtype == FEATURE_FWD_MATCH )
		return feat->fwd_match;
	return NULL;
}



/*
Finds all features with a match of a specified type and stores pointers
to them in an array.  Additionally initializes each matched feature's
feature_data field with a ransac_data structure.

@param features array of features
@param n number of features in features
@param mtype match type, one of FEATURE_{FWD,BCK,MDL}_MATCH
@param matched output as an array of pointers to features with a match of
the specified type

@return Returns the number of features output in matched.
*/
int get_matched_features( struct feature* features, int n, int mtype,
struct feature*** matched )
{
	struct feature** _matched;
	struct ransac_data* rdata;
	int i, m = 0;

	_matched = calloc( n, sizeof( struct feature* ) );
	for( i = 0; i < n; i++ )
		if( get_match( features + i, mtype ) )
		{
			rdata = malloc( sizeof( struct ransac_data ) );
			memset( rdata, 0, sizeof( struct ransac_data ) );
			rdata->orig_feat_data = features[i].feature_data;
			_matched[m] = features + i;
			_matched[m]->feature_data = rdata;
			m++;
		}
		*matched = _matched;
		return m;
}



/*
Calculates the minimum number of inliers as a function of the number of
putative correspondences.  Based on equation (7) in

Chum, O. and Matas, J.  Matching with PROSAC -- Progressive Sample Consensus.
In <EM>Conference on Computer Vision and Pattern Recognition (CVPR)</EM>,
(2005), pp. 220--226.

@param n number of putative correspondences
@param m min number of correspondences to compute the model in question
@param p_badsupp prob. that a bad model is supported by a correspondence
@param p_badxform desired prob. that the final transformation returned is bad

@return Returns the minimum number of inliers required to guarantee, based
	on p_badsupp, that the probability that the final transformation returned
	by RANSAC is less than p_badxform
*/
int calc_min_inliers( int n, int m, double p_badsupp, double p_badxform )
{
	double pi, sum;
	int i, j;

	for( j = m+1; j <= n; j++ )
	{
		sum = 0;
		for( i = j; i <= n; i++ )
		{
			pi = ( i - m ) * log( p_badsupp ) + ( n - i + m ) * log( 1.0 - p_badsupp ) +
				gsl_sf_lnchoose( n - m, i - m );
			sum += exp( pi );
		}
		if( sum < p_badxform )
			break;
	}
	return j;
}



/*
Draws a RANSAC sample from a set of features.

@param features array of pointers to features from which to sample
@param n number of features in features
@param m size of the sample
@param rng random number generator used to sample

@return Returns an array of pointers to the sampled features; the sampled
	field of each sampled feature's ransac_data is set to 1
*/
struct feature** draw_ransac_sample( struct feature** features, int n,
									int m, gsl_rng* rng )
{
	struct feature** sample, * feat;
	struct ransac_data* rdata;
	int i, x;

	for( i = 0; i < n; i++ )
	{
		rdata = feat_ransac_data( features[i] );
		rdata->sampled = 0;
	}

	sample = calloc( m, sizeof( struct feature* ) );
	for( i = 0; i < m; i++ )
	{
		do
		{
			x = gsl_rng_uniform_int( rng, n );
			feat = features[x];
			rdata = feat_ransac_data( feat );
		}
		while( rdata->sampled );
		sample[i] = feat;
		rdata->sampled = 1;
	}

	return sample;
}



/*
Extrancs raw point correspondence locations from a set of features

@param features array of features from which to extract points and match
	points; each of these is assumed to have a match of type mtype
@param n number of features
@param mtype match type; if FEATURE_MDL_MATCH correspondences are assumed
	to be between each feature's img_pt field and it's match's mdl_pt field,
	otherwise, correspondences are assumed to be between img_pt and img_pt
@param pts output as an array of raw point locations from features
@param mpts output as an array of raw point locations from features' matches
*/
void extract_corresp_pts( struct feature** features, int n, int mtype,
						 CvPoint2D64f** pts, CvPoint2D64f** mpts )
{
	struct feature* match;
	CvPoint2D64f* _pts, * _mpts;
	int i;

	_pts = calloc( n, sizeof( CvPoint2D64f ) );
	_mpts = calloc( n, sizeof( CvPoint2D64f ) );

	if( mtype == FEATURE_MDL_MATCH )
		for( i = 0; i < n; i++ )
		{
			match = get_match( features[i], mtype );
			if( ! match )
				fatal_error( "feature does not have match of type %d, %s line %d",
							mtype, __FILE__, __LINE__ );
			_pts[i] = features[i]->img_pt;
			_mpts[i] = match->mdl_pt;
		}

	else
		for( i = 0; i < n; i++ )
		{
			match = get_match( features[i], mtype );
			if( ! match )
				fatal_error( "feature does not have match of type %d, %s line %d",
							mtype, __FILE__, __LINE__ );
			_pts[i] = features[i]->img_pt;
			_mpts[i] = match->img_pt;
		}

		*pts = _pts;
		*mpts = _mpts;
}



/*
For a given model and error function, finds a consensus from a set of
feature correspondences.

@param features set of pointers to features; every feature is assumed to
	have a match of type mtype
@param n number of features in features
@param mtype determines the match field of each feature against which to
	measure error; if this is FEATURE_MDL_MATCH, correspondences are assumed
	to be between the feature's img_pt field and the match's mdl_pt field;
	otherwise matches are assumed to be between img_pt and img_pt
@param M model for which a consensus set is being found
@param err_fn error function used to measure distance from M
@param err_tol correspondences within this distance of M are added to the
	consensus set
@param consensus output as an array of pointers to features in the
	consensus set

@return Returns the number of points in the consensus set
*/
int find_consensus( struct feature** features, int n, int mtype,
				   CvMat* M, ransac_err_fn err_fn, double err_tol,
				   struct feature*** consensus )
{
	struct feature** _consensus;
	struct feature* match;
	CvPoint2D64f pt, mpt;
	double err;
	int i, in = 0;

	_consensus = calloc( n, sizeof( struct feature* ) );

	if( mtype == FEATURE_MDL_MATCH )
		for( i = 0; i < n; i++ )
		{
			match = get_match( features[i], mtype );
			if( ! match )
				fatal_error( "feature does not have match of type %d, %s line %d",
							mtype, __FILE__, __LINE__ );
			pt = features[i]->img_pt;
			mpt = match->mdl_pt;
			err = err_fn( pt, mpt, M );
			if( err <= err_tol )
				_consensus[in++] = features[i];
		}

	else
		for( i = 0; i < n; i++ )
		{
			match = get_match( features[i], mtype );
			if( ! match )
				fatal_error( "feature does not have match of type %d, %s line %d",
							mtype, __FILE__, __LINE__ );
			pt = features[i]->img_pt;
			mpt = match->img_pt;
			err = err_fn( pt, mpt, M );
			if( err <= err_tol )
				_consensus[in++] = features[i];
		}
	*consensus = _consensus;
	return in;
}



/*
Releases memory and reduces code size above

@param pts1 an array of points
@param pts2 an array of points
@param features an array of pointers to features; can be NULL
*/
static __inline void release_mem( CvPoint2D64f* pts1, CvPoint2D64f* pts2,
struct feature** features )
{
	free( pts1 );
	free( pts2 );
	if( features )
		free( features );
}
