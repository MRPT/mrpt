/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservation3DRangeScan_project3D_impl_H
#define CObservation3DRangeScan_project3D_impl_H

#include <mrpt/utils/round.h> // round()

namespace mrpt {
namespace obs {
namespace detail {
	// Auxiliary functions which implement SSE-optimized proyection of 3D point cloud:
	template <class POINTMAP> void do_project_3d_pointcloud(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca, std::vector<uint16_t> &idxs_x, std::vector<uint16_t> &idxs_y,const mrpt::obs::TRangeImageFilterParams &filterParams);
	template <class POINTMAP> void do_project_3d_pointcloud_SSE2(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca, std::vector<uint16_t> &idxs_x, std::vector<uint16_t> &idxs_y,const mrpt::obs::TRangeImageFilterParams &filterParams);

	template <class POINTMAP>
	void project3DPointsFromDepthImageInto(
			mrpt::obs::CObservation3DRangeScan    & src_obs,
			POINTMAP                   & dest_pointcloud,
			const mrpt::obs::T3DPointsProjectionParams & projectParams,
			const mrpt::obs::TRangeImageFilterParams &filterParams)
	{
		using namespace mrpt::math;

		if (!src_obs.hasRangeImage) return;

		mrpt::utils::PointCloudAdapter<POINTMAP> pca(dest_pointcloud);

		// ------------------------------------------------------------
		// Stage 1/3: Create 3D point cloud local coordinates
		// ------------------------------------------------------------
		const int W = src_obs.rangeImage.cols();
		const int H = src_obs.rangeImage.rows();
		ASSERT_(W!=0 && H!=0);
		const size_t WH = W*H;

		src_obs.resizePoints3DVectors(WH); // This is to make sure points3D_idxs_{x,y} have the expected sizes.
		pca.resize(WH); // Reserve memory for 3D points. It will be later resized again to the actual number of valid points

		if (src_obs.range_is_depth)
		{
			// range_is_depth = true

			// Use cached tables?
			if (projectParams.PROJ3D_USE_LUT)
			{
				// Use LUT:
				if (src_obs.m_3dproj_lut.prev_camParams!=src_obs.cameraParams || WH!=size_t(src_obs.m_3dproj_lut.Kys.size()))
				{
					src_obs.m_3dproj_lut.prev_camParams = src_obs.cameraParams;
					src_obs.m_3dproj_lut.Kys.resize(WH);
					src_obs.m_3dproj_lut.Kzs.resize(WH);

					const float r_cx = src_obs.cameraParams.cx();
					const float r_cy = src_obs.cameraParams.cy();
					const float r_fx_inv = 1.0f/src_obs.cameraParams.fx();
					const float r_fy_inv = 1.0f/src_obs.cameraParams.fy();

					float *kys = &src_obs.m_3dproj_lut.Kys[0];
					float *kzs = &src_obs.m_3dproj_lut.Kzs[0];
					for (int r=0;r<H;r++)
						for (int c=0;c<W;c++)
						{
							*kys++ = (r_cx - c) * r_fx_inv;
							*kzs++ = (r_cy - r) * r_fy_inv;
						}
				} // end update LUT.

				ASSERT_EQUAL_(WH,size_t(src_obs.m_3dproj_lut.Kys.size()))
				ASSERT_EQUAL_(WH,size_t(src_obs.m_3dproj_lut.Kzs.size()))
				float *kys = &src_obs.m_3dproj_lut.Kys[0];
				float *kzs = &src_obs.m_3dproj_lut.Kzs[0];

				if (filterParams.rangeMask_min) { // sanity check:
					ASSERT_EQUAL_(filterParams.rangeMask_min->cols(), src_obs.rangeImage.cols());
					ASSERT_EQUAL_(filterParams.rangeMask_min->rows(), src_obs.rangeImage.rows());
				}
				if (filterParams.rangeMask_max) { // sanity check:
					ASSERT_EQUAL_(filterParams.rangeMask_max->cols(), src_obs.rangeImage.cols());
					ASSERT_EQUAL_(filterParams.rangeMask_max->rows(), src_obs.rangeImage.rows());
				}
	#if MRPT_HAS_SSE2
				if ((W & 0x07)==0 && projectParams.USE_SSE2)
					 do_project_3d_pointcloud_SSE2(H,W,kys,kzs,src_obs.rangeImage,pca, src_obs.points3D_idxs_x, src_obs.points3D_idxs_y,filterParams );
				else do_project_3d_pointcloud(H,W,kys,kzs,src_obs.rangeImage,pca, src_obs.points3D_idxs_x, src_obs.points3D_idxs_y,filterParams );  // if image width is not 8*N, use standard method
	#else
				do_project_3d_pointcloud(H,W,kys,kzs,src_obs.rangeImage,pca,src_obs.points3D_idxs_x, src_obs.points3D_idxs_y,filterParams);
	#endif
			}
			else
			{
				// Without LUT:
				const float r_cx =  src_obs.cameraParams.cx();
				const float r_cy = src_obs.cameraParams.cy();
				const float r_fx_inv = 1.0f/src_obs.cameraParams.fx();
				const float r_fy_inv = 1.0f/src_obs.cameraParams.fy();
				TRangeImageFilter rif(filterParams);
				size_t idx=0;
				for (int r=0;r<H;r++)
					for (int c=0;c<W;c++)
					{
						const float D = src_obs.rangeImage.coeff(r,c);
						if (rif.do_range_filter(r,c,D))
						{
							const float Kz = (r_cy - r) * r_fy_inv;
							const float Ky = (r_cx - c) * r_fx_inv;
							pca.setPointXYZ(idx,
								D,        // x
								Ky * D,   // y
								Kz * D    // z
								);
							src_obs.points3D_idxs_x[idx]=c;
							src_obs.points3D_idxs_y[idx]=r;
							++idx;
						}
					}
				pca.resize(idx); // Actual number of valid pts
			}
		}
		else
		{
			/* range_is_depth = false :
			  *   Ky = (r_cx - c)/r_fx
			  *   Kz = (r_cy - r)/r_fy
			  *
			  *   x(i) = rangeImage(r,c) / sqrt( 1 + Ky^2 + Kz^2 )
			  *   y(i) = Ky * x(i)
			  *   z(i) = Kz * x(i)
			  */
			const float r_cx = src_obs.cameraParams.cx();
			const float r_cy = src_obs.cameraParams.cy();
			const float r_fx_inv = 1.0f/src_obs.cameraParams.fx();
			const float r_fy_inv = 1.0f/src_obs.cameraParams.fy();
			TRangeImageFilter rif(filterParams);
			size_t idx=0;
			for (int r=0;r<H;r++)
				for (int c=0;c<W;c++)
				{
					const float D = src_obs.rangeImage.coeff(r,c);
					if (rif.do_range_filter(r,c,D))
					{
						const float Ky = (r_cx - c) * r_fx_inv;
						const float Kz = (r_cy - r) * r_fy_inv;
						pca.setPointXYZ(idx,
							D / std::sqrt(1+Ky*Ky+Kz*Kz), // x
							Ky * D,   // y
							Kz * D    // z
							);
						src_obs.points3D_idxs_x[idx]=c;
						src_obs.points3D_idxs_y[idx]=r;
						++idx;
					}
				}
			pca.resize(idx); // Actual number of valid pts
		}

		// -------------------------------------------------------------
		// Stage 2/3: Project local points into RGB image to get colors
		// -------------------------------------------------------------
		if (src_obs.hasIntensityImage)
		{
			const int imgW = src_obs.intensityImage.getWidth();
			const int imgH = src_obs.intensityImage.getHeight();
			const bool hasColorIntensityImg = src_obs.intensityImage.isColor();

			const float cx = src_obs.cameraParamsIntensity.cx();
			const float cy = src_obs.cameraParamsIntensity.cy();
			const float fx = src_obs.cameraParamsIntensity.fx();
			const float fy = src_obs.cameraParamsIntensity.fy();

			// Unless we are in a special case (both depth & RGB images coincide)...
			const bool isDirectCorresp = src_obs.doDepthAndIntensityCamerasCoincide();

			// ...precompute the inverse of the pose transformation out of the loop,
			//  store as a 4x4 homogeneous matrix to exploit SSE optimizations below:
			mrpt::math::CMatrixFixedNumeric<float,4,4> T_inv;
			if (!isDirectCorresp)
			{
			 mrpt::math::CMatrixFixedNumeric<double,3,3> R_inv;
			 mrpt::math::CMatrixFixedNumeric<double,3,1> t_inv;
				mrpt::math::homogeneousMatrixInverse(
					src_obs.relativePoseIntensityWRTDepth.getRotationMatrix(),src_obs.relativePoseIntensityWRTDepth.m_coords,
					R_inv,t_inv);

				T_inv(3,3)=1;
				T_inv.block<3,3>(0,0)=R_inv.cast<float>();
				T_inv.block<3,1>(0,3)=t_inv.cast<float>();
			}

			Eigen::Matrix<float,4,1>  pt_wrt_color, pt_wrt_depth;
			pt_wrt_depth[3]=1;

			mrpt::utils::TColor pCol;

			// For each local point:
			const size_t nPts = pca.size();
			for (size_t i=0;i<nPts;i++)
			{
				int img_idx_x, img_idx_y;  // projected pixel coordinates, in the RGB image plane
				bool pointWithinImage = false;
				if (isDirectCorresp)
				{
					pointWithinImage=true;
					img_idx_x = src_obs.points3D_idxs_x[i];
					img_idx_y = src_obs.points3D_idxs_y[i];
				}
				else
				{
					// Project point, which is now in "pca" in local coordinates wrt the depth camera, into the intensity camera:
					pca.getPointXYZ(i,pt_wrt_depth[0],pt_wrt_depth[1],pt_wrt_depth[2]);
					pt_wrt_color.noalias() = T_inv*pt_wrt_depth;

					// Project to image plane:
					if (pt_wrt_color[2]) {
						img_idx_x = mrpt::utils::round( cx + fx * pt_wrt_color[0]/pt_wrt_color[2] );
						img_idx_y = mrpt::utils::round( cy + fy * pt_wrt_color[1]/pt_wrt_color[2] );
						pointWithinImage=
							img_idx_x>=0 && img_idx_x<imgW &&
							img_idx_y>=0 && img_idx_y<imgH;
					}
				}

				if (pointWithinImage)
				{
					if (hasColorIntensityImg)  {
						const uint8_t *c= src_obs.intensityImage.get_unsafe(img_idx_x, img_idx_y, 0);
						pCol.R = c[2];
						pCol.G = c[1];
						pCol.B = c[0];
					}
					else{
						uint8_t c= *src_obs.intensityImage.get_unsafe(img_idx_x, img_idx_y, 0);
						pCol.R = pCol.G = pCol.B = c;
					}
				}
				else
				{
					pCol.R = pCol.G = pCol.B = 255;
				}
				// Set color:
				pca.setPointRGBu8(i,pCol.R,pCol.G,pCol.B);
			} // end for each point
		} // end if src_obs has intensity image

		// ...

		// ------------------------------------------------------------
		// Stage 3/3: Apply 6D transformations
		// ------------------------------------------------------------
		if (projectParams.takeIntoAccountSensorPoseOnRobot || projectParams.robotPoseInTheWorld)
		{
			mrpt::poses::CPose3D  transf_to_apply; // Either ROBOTPOSE or ROBOTPOSE(+)SENSORPOSE or SENSORPOSE
			if (projectParams.takeIntoAccountSensorPoseOnRobot)
				transf_to_apply = src_obs.sensorPose;
			if (projectParams.robotPoseInTheWorld)
				transf_to_apply.composeFrom(*projectParams.robotPoseInTheWorld, mrpt::poses::CPose3D(transf_to_apply));

			const mrpt::math::CMatrixFixedNumeric<float,4,4> HM = transf_to_apply.getHomogeneousMatrixVal().cast<float>();
			Eigen::Matrix<float,4,1>  pt, pt_transf;
			pt[3]=1;

			const size_t nPts = pca.size();
			for (size_t i=0;i<nPts;i++)
			{
				pca.getPointXYZ(i,pt[0],pt[1],pt[2]);
				pt_transf.noalias() = HM*pt;
				pca.setPointXYZ(i,pt_transf[0],pt_transf[1],pt_transf[2]);
			}
		}
	} // end of project3DPointsFromDepthImageInto

	// Auxiliary functions which implement proyection of 3D point clouds:
	template <class POINTMAP>
	inline void do_project_3d_pointcloud(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca, std::vector<uint16_t> &idxs_x, std::vector<uint16_t> &idxs_y,const mrpt::obs::TRangeImageFilterParams &fp)
	{
		TRangeImageFilter rif(fp);
		// Preconditions: minRangeMask() has the right size
		size_t idx=0;
		for (int r=0;r<H;r++)
			for (int c=0;c<W;c++)
			{
				const float D = rangeImage.coeff(r,c);
				if (!rif.do_range_filter(r,c,D))
					continue;

				pca.setPointXYZ(idx, D /*x*/, *kys++ * D /*y*/, *kzs++ * D /*z*/);
				idxs_x[idx]=c;
				idxs_y[idx]=r;
				++idx;
			}
		pca.resize(idx);
	}

	// Auxiliary functions which implement proyection of 3D point clouds:
	template <class POINTMAP>
	inline void do_project_3d_pointcloud_SSE2(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca, std::vector<uint16_t> &idxs_x, std::vector<uint16_t> &idxs_y,const mrpt::obs::TRangeImageFilterParams &filterParams)
	{
	#if MRPT_HAS_SSE2
			// Preconditions: minRangeMask() has the right size
			// Use optimized version:
			const int W_4 = W >> 2;  // /=4 , since we process 4 values at a time.
			size_t idx=0;
			MRPT_ALIGN16 float xs[4],ys[4],zs[4];
			const __m128 D_zeros = _mm_set_ps(.0f,.0f,.0f,.0f);
			const __m128 xormask = (filterParams.rangeCheckBetween) ?
				_mm_cmpneq_ps(D_zeros,D_zeros) :	// want points BETWEEN min and max to be valid
				_mm_cmpeq_ps(D_zeros,D_zeros);		// want points OUTSIDE of min and max to be valid
			for (int r=0;r<H;r++)
			{
				const float *D_ptr = &rangeImage.coeffRef(r,0);  // Matrices are 16-aligned
				const float *Dgt_ptr = !filterParams.rangeMask_min ? NULL : &filterParams.rangeMask_min->coeffRef(r,0);
				const float *Dlt_ptr = !filterParams.rangeMask_max ? NULL : &filterParams.rangeMask_max->coeffRef(r,0);

				for (int c=0;c<W_4;c++)
				{
					const __m128 D = _mm_load_ps(D_ptr);
					const __m128 nz_mask = _mm_cmpgt_ps(D, D_zeros);
					__m128 valid_range_mask;
					if (!filterParams.rangeMask_min && !filterParams.rangeMask_max)
					{	// No filter: just skip D=0 points
						valid_range_mask = nz_mask;
					}
					else
					{
						if (!filterParams.rangeMask_min || !filterParams.rangeMask_max)
						{	// Only one filter
							if (filterParams.rangeMask_min)
							{
								const __m128 Dmin = _mm_load_ps(Dgt_ptr);
								 valid_range_mask = _mm_and_ps(_mm_cmpgt_ps(D, Dmin ), _mm_cmpgt_ps(Dmin, D_zeros));
							}
							else
							{
								const __m128 Dmax = _mm_load_ps(Dlt_ptr);
								valid_range_mask = _mm_and_ps(_mm_cmplt_ps(D, Dmax ), _mm_cmpgt_ps(Dmax, D_zeros));
							}
							valid_range_mask = _mm_and_ps(valid_range_mask, nz_mask ); // Filter out D=0 points
						}
						else
						{
							// We have both: D>Dmin and D<Dmax conditions, with XOR to optionally invert the selection:
							const __m128 Dmin = _mm_load_ps(Dgt_ptr);
							const __m128 Dmax = _mm_load_ps(Dlt_ptr);

							const __m128 gt_mask = _mm_cmpgt_ps(D, Dmin );
							const __m128 lt_mask = _mm_and_ps( _mm_cmplt_ps(D, Dmax), nz_mask ); // skip points at zero
							valid_range_mask = _mm_and_ps(gt_mask, lt_mask ); // (D>Dmin && D<Dmax)
							valid_range_mask = _mm_xor_ps(valid_range_mask, xormask );
							// Add the case of D_min & D_max = 0 (no filtering)
							valid_range_mask = _mm_or_ps(valid_range_mask, _mm_and_ps(_mm_cmpeq_ps(Dmin,D_zeros),_mm_cmpeq_ps(Dmax,D_zeros)) );
							// Finally, ensure no invalid ranges get thru:
							valid_range_mask = _mm_and_ps(valid_range_mask, nz_mask );
						}
					}
					const int valid_range_maski = _mm_movemask_epi8(_mm_castps_si128(valid_range_mask)); // 0x{f|0}{f|0}{f|0}{f|0}
					if (valid_range_maski!=0)  // Any of the 4 values is valid?
					{
						const __m128 KY = _mm_load_ps(kys);
						const __m128 KZ = _mm_load_ps(kzs);

						_mm_storeu_ps(xs , D);
						_mm_storeu_ps(ys , _mm_mul_ps(KY,D));
						_mm_storeu_ps(zs , _mm_mul_ps(KZ,D));

						for (int q=0;q<4;q++)
							if ((valid_range_maski & (1<<(q*4))) !=0) {
								pca.setPointXYZ(idx,xs[q],ys[q],zs[q]);
								idxs_x[idx]=(c<<2)+q;
								idxs_y[idx]=r;
								++idx;
							}
					}
					D_ptr+=4;
					if (Dgt_ptr) Dgt_ptr+=4;
					if (Dlt_ptr) Dlt_ptr+=4;
					kys+=4;
					kzs+=4;
				}
			}
			pca.resize(idx);
	#endif
	}

} // End of namespace
} // End of namespace
} // End of namespace
#endif
