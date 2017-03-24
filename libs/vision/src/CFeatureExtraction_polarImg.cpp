/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;


/****************************************************************************************
                                   Linear-Polar Transform
  J.L. Blanco, Apr 2009
****************************************************************************************/
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM < 0x111
void my_cvLinearPolar( const CvArr* srcarr, CvArr* dstarr,
            CvPoint2D32f center, double maxRadius, int flags )
{
    CvMat* mapx = 0;
    CvMat* mapy = 0;
    float* buf = 0;

    CV_FUNCNAME( "cvLinPolar" );

    __BEGIN__;

    CvMat srcstub, *src = (CvMat*)srcarr;
    CvMat dststub, *dst = (CvMat*)dstarr;
    CvSize ssize, dsize;

    CV_CALL( src = cvGetMat( srcarr, &srcstub,0,0 ));
    CV_CALL( dst = cvGetMat( dstarr, &dststub,0,0 ));

    if( !CV_ARE_TYPES_EQ( src, dst ))
        CV_ERROR( CV_StsUnmatchedFormats, "" );

	ssize.width = src->cols;
    ssize.height = src->rows;
    dsize.width = dst->cols;
    dsize.height = dst->rows;

    CV_CALL( mapx = cvCreateMat( dsize.height, dsize.width, CV_32F ));
    CV_CALL( mapy = cvCreateMat( dsize.height, dsize.width, CV_32F ));

    if( !(flags & CV_WARP_INVERSE_MAP) )
    {
        int phi, rho;

        for( phi = 0; phi < dsize.height; phi++ )
        {
            double cp = cos(phi*2*CV_PI/(dsize.height-1));
            double sp = sin(phi*2*CV_PI/(dsize.height-1));
            float* mx = (float*)(mapx->data.ptr + phi*mapx->step);
            float* my = (float*)(mapy->data.ptr + phi*mapy->step);

            for( rho = 0; rho < dsize.width; rho++ )
            {
                double r = maxRadius*(rho+1)/double(dsize.width-1);
                double x = r*cp + center.x;
                double y = r*sp + center.y;

                mx[rho] = (float)x;
                my[rho] = (float)y;
            }
        }
    }
    else
    {
        int x, y;
        CvMat bufx, bufy, bufp, bufa;
        const double ascale = (ssize.height-1)/(2*CV_PI);
        const double pscale = (ssize.width-1)/maxRadius;

        CV_CALL( buf = (float*)cvAlloc( 4*dsize.width*sizeof(buf[0]) ));

        bufx = cvMat( 1, dsize.width, CV_32F, buf );
        bufy = cvMat( 1, dsize.width, CV_32F, buf + dsize.width );
        bufp = cvMat( 1, dsize.width, CV_32F, buf + dsize.width*2 );
        bufa = cvMat( 1, dsize.width, CV_32F, buf + dsize.width*3 );

        for( x = 0; x < dsize.width; x++ )
            bufx.data.fl[x] = (float)x - center.x;

        for( y = 0; y < dsize.height; y++ )
        {
            float* mx = (float*)(mapx->data.ptr + y*mapx->step);
            float* my = (float*)(mapy->data.ptr + y*mapy->step);

            for( x = 0; x < dsize.width; x++ )
                bufy.data.fl[x] = (float)y - center.y;

            cvCartToPolar( &bufx, &bufy, &bufp, &bufa, 0 );

            for( x = 0; x < dsize.width; x++ )
                bufp.data.fl[x] += 1.f;

            for( x = 0; x < dsize.width; x++ )
            {
                double rho = bufp.data.fl[x]*pscale;
                double phi = bufa.data.fl[x]*ascale;
                mx[x] = (float)rho;
                my[x] = (float)phi;
            }
        }
    }

    cvRemap( src, dst, mapx, mapy, flags, cvScalarAll(0) );

    __END__;

    cvFree( &buf );
    cvReleaseMat( &mapx );
    cvReleaseMat( &mapy );
}
#endif


/************************************************************************************************
								computePolarImageDescriptors
************************************************************************************************/
void  CFeatureExtraction::internal_computePolarImageDescriptors(
	const mrpt::utils::CImage	&in_img,
	CFeatureList		&in_features) const
{
	MRPT_START
#if MRPT_HAS_OPENCV

	ASSERT_(options.PolarImagesOptions.radius>1)
	ASSERT_(options.PolarImagesOptions.bins_angle>1)
	ASSERT_(options.PolarImagesOptions.bins_distance>1)

	const unsigned int radius = options.PolarImagesOptions.radius;
	const unsigned int patch_w = options.PolarImagesOptions.bins_distance;
	const unsigned int patch_h = options.PolarImagesOptions.bins_angle;

	CImage	linpolar_frame( patch_w, patch_h, in_img.getChannelCount() );

	// Compute intensity-domain spin images
	for (CFeatureList::iterator it=in_features.begin();it!=in_features.end();++it)
	{
		// Overwrite scale with the descriptor scale:
		(*it)->scale = radius;

		// Use OpenCV to convert:
#if MRPT_OPENCV_VERSION_NUM < 0x111
		my_cvLinearPolar(	// Use version embedded above in this file
#else
		cvLinearPolar(		// Use version sent to OpenCV
#endif
			in_img.getAs<IplImage>(),
			linpolar_frame.getAs<IplImage>(),
			cvPoint2D32f( (*it)->x,(*it)->y ),
			radius,
			CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );

		// Get the image as a matrix and save as patch:
		linpolar_frame.getAsMatrix( (*it)->descriptors.PolarImg );

	} // end for it

#else
		THROW_EXCEPTION("This method needs MRPT compiled with OpenCV support")
#endif
	MRPT_END
}

