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

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


/************************************************************************************************
*								computeSpinImageDescriptors											*
************************************************************************************************/
void  CFeatureExtraction::internal_computeSpinImageDescriptors(
	const CImage	&in_img,
	CFeatureList		&in_features) const
{
	MRPT_START

	ASSERT_(options.SpinImagesOptions.radius>1)

	// This is a C++ implementation of the descriptor "Intensity-domain spin images" as described
	//  in "A sparse texture representation using affine-invariant regions", S Lazebnik, C Schmid, J Ponce,
	//  2003 IEEE Computer Society Conference on Computer Vision.

	const unsigned int HIST_N_INT = options.SpinImagesOptions.hist_size_intensity;
	const unsigned int HIST_N_DIS = options.SpinImagesOptions.hist_size_distance;
	const unsigned int R = options.SpinImagesOptions.radius;
	const int img_w = static_cast<int>( in_img.getWidth() );
	const int img_h = static_cast<int>( in_img.getHeight() );
	const bool img_color = in_img.isColor();

	// constant for passing intensity [0,255] to histogram index [0,HIST_N_INT-1]:
	const float k_int2idx = (HIST_N_INT-1) / 255.0f;
	const float k_idx2int = 1.0f/k_int2idx;

	// constant for passing distances in pixels [0,R] to histogram index [0,HIST_N_DIS-1]:
	const float k_dis2idx = (HIST_N_DIS-1) / static_cast<float>(R);
	const float k_idx2dis = 1.0f/k_dis2idx;

	// The Gaussian kernel used below is approximated up to a given distance
	//  in pixels, given by 2 times the appropriate std. deviations:
	const int STD_TIMES = 2;
	const int  kernel_size_dist = static_cast<int>( ceil(k_dis2idx * STD_TIMES * options.SpinImagesOptions.std_dist) );

	const float _2var_int  = -1.0f/(2*square( options.SpinImagesOptions.std_intensity ));
	const float _2var_dist = -1.0f/(2*square( options.SpinImagesOptions.std_dist ));


	// Create the 2D histogram:
	CMatrixDouble  hist2d(HIST_N_INT,HIST_N_DIS);

	// Compute intensity-domain spin images
	for (CFeatureList::iterator it=in_features.begin();it!=in_features.end();++it)
	{
		// Overwrite scale with the descriptor scale:
		(*it)->scale = options.SpinImagesOptions.radius;

		// Reset histogram to zeros:
		hist2d.zeros();

		// Define the ROI around the interest point which counts for the histogram:
		int px0 = round( (*it)->x - R );
		int px1 = round( (*it)->x + R );
		int py0 = round( (*it)->y - R );
		int py1 = round( (*it)->y + R );

		// Clip at img borders:
		px0=max(0,px0);
		px1=min(img_w-1,px1);
		py0=max(0,py0);
		py1=min(img_h-1,py1);

		uint8_t pix_val;
		uint8_t *aux_pix_ptr;

		for (int px=px0;px<=px1;px++)
		{
			for (int py=py0;py<=py1;py++)
			{
				// get the pixel color [0,255]
				if (!img_color)
					pix_val = *in_img.get_unsafe(px,py,0);
				else
				{
					aux_pix_ptr = in_img.get_unsafe(px,py,0);
					pix_val = (aux_pix_ptr[0] + aux_pix_ptr[1] + aux_pix_ptr[2]) / 3;
				}

				const float pix_dist = hypot( (*it)->x - px, (*it)->y - py );
				const int center_bin_dist = k_dis2idx * pix_dist;

				// A factor to correct the histogram due to the existence of more pixels at larger radius:
				//  Obtained as Area = PI ( R1^2 - R2^2 ) for R1,R2 being pix_dist +- Delta/2
				//const double density_circular_area = 1.0/ (M_2PI * (center_bin_dist+0.5) * square(k_idx2dis) );

#if 0
				// "normal" histogram
				const int bin_int = k_int2idx * pix_val;

				if (center_bin_dist<static_cast<int>(HIST_N_DIS))  // this accounts for the "square" or "circle" shapes of the area to account for
				{
					hist2d(bin_int,center_bin_dist) +=1; // * density_circular_area;
				}
#else
				// Apply a "soft-histogram", so each pixel counts into several bins,
				//  weighted by a exponential function to model a Gaussian kernel
				const int bin_int_low = max(0, static_cast<int>(ceil(k_int2idx * ( pix_val - STD_TIMES * options.SpinImagesOptions.std_intensity ))) );
				const int bin_int_hi  = min(static_cast<int>(HIST_N_INT-1), static_cast<int>(ceil(k_int2idx * ( pix_val + STD_TIMES * options.SpinImagesOptions.std_intensity ))));

				//cout << "d: " << pix_dist << "v: " << (int)pix_val << "\t";

				if (center_bin_dist<static_cast<int>(HIST_N_DIS))  // this accounts for the "square" or "circle" shapes of the area to account for
				{
					const int bin_dist_low = max(0, center_bin_dist-kernel_size_dist);
					const int bin_dist_hi  = min(static_cast<int>(HIST_N_DIS-1), center_bin_dist+kernel_size_dist);

					int bin_dist, bin_int;
					float pix_dist_cur_dist = pix_dist - bin_dist_low*k_idx2dis;

					for (bin_dist = bin_dist_low;bin_dist<=bin_dist_hi;bin_dist++, pix_dist_cur_dist-=k_idx2dis)
					{
						float pix_val_cur_val = pix_val-(bin_int_low*k_idx2int);

						for (bin_int=bin_int_low;bin_int<=bin_int_hi;bin_int++, pix_val_cur_val-=k_idx2int)
						{
							// Gaussian kernel:
							double v = _2var_dist * square(pix_dist_cur_dist) + _2var_int  * square(pix_val_cur_val);
//								_2var_dist * square(pix_dist - bin_dist*k_idx2dis ) +
//								_2var_int  * square(pix_val-(bin_int*k_idx2int));

							hist2d.get_unsafe(bin_int,bin_dist) += exp(v); // * density_circular_area;
						}
					}
					//hist2d(bin_int,bin_dist) *= ;
				} // in range
#endif

			} // end py
		} // end px

		// Normalize:
		hist2d.normalize(0,1); // [0,1]

#if 0
		{	// Debug
			static int n=0;
			CMatrixDouble AA(hist2d);
			AA.normalize(0,1);
			CImage  aux_img( AA );
			aux_img.saveToFile( format("spin_feat_%04i.png",n) );
			CImage  aux_img2 = in_img;
			aux_img2.drawCircle((*it)->x,(*it)->y,20,TColor(255,0,0));
			aux_img2.saveToFile( format("spin_feat_%04i_map.png",n) );
			n++;
		}
#endif

		// Save the histogram as a vector:
		unsigned idx=0;
		std::vector<float> &ptr_trg = (*it)->descriptors.SpinImg;
		ptr_trg.resize( HIST_N_INT * HIST_N_DIS );

		for (unsigned i=0;i<HIST_N_DIS;i++)
			for (unsigned j=0;j<HIST_N_INT;j++)
				ptr_trg[idx++] = hist2d.get_unsafe(j,i);

		(*it)->descriptors.SpinImg_range_rows = HIST_N_DIS;

	} // end for each feature

	MRPT_END
}

