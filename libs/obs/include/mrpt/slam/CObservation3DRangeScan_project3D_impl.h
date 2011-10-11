/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CObservation3DRangeScan_project3D_impl_H
#define CObservation3DRangeScan_project3D_impl_H

namespace mrpt {
namespace slam {
namespace detail {
	// Auxiliary functions which implement SSE-optimized proyection of 3D point cloud:
	template <class POINTMAP> void do_project_3d_pointcloud(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca);
	template <class POINTMAP> void do_project_3d_pointcloud_SSE2(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca);

	template <class POINTMAP>
	void project3DPointsFromDepthImageInto(
			CObservation3DRangeScan    & src_obs,
			POINTMAP                   & dest_pointcloud,
			const bool                   takeIntoAccountSensorPoseOnRobot,
			const mrpt::poses::CPose3D * robotPoseInTheWorld,
			const bool                   PROJ3D_USE_LUT)
	{
		if (!src_obs.hasRangeImage) return;

		mrpt::utils::PointCloudAdapter<POINTMAP> pca(dest_pointcloud);

		// ------------------------------------------------------------
		// Stage 1/3: Create 3D point cloud local coordinates
		// ------------------------------------------------------------
		const int W = src_obs.rangeImage.cols();
		const int H = src_obs.rangeImage.rows();
		const size_t WH = W*H;

		// Reserve memory for 3D points:
		pca.resize(WH);

		if (src_obs.range_is_depth)
		{
			// range_is_depth = true

			// Use cached tables?
			if (PROJ3D_USE_LUT)
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

	#if MRPT_HAS_SSE2
				if ((W & 0x07)==0)
						do_project_3d_pointcloud_SSE2(H,W,kys,kzs,src_obs.rangeImage,pca);
				else	do_project_3d_pointcloud(H,W,kys,kzs,src_obs.rangeImage,pca);  // if image width is not 8*N, use standard method
	#else
				do_project_3d_pointcloud(H,W,kys,kzs,src_obs.rangeImage,pca);
	#endif
			}
			else
			{
				// Without LUT:
				const float r_cx =  src_obs.cameraParams.cx();
				const float r_cy = src_obs.cameraParams.cy();
				const float r_fx_inv = 1.0f/src_obs.cameraParams.fx();
				const float r_fy_inv = 1.0f/src_obs.cameraParams.fy();
				size_t idx=0;
				for (int r=0;r<H;r++)
					for (int c=0;c<W;c++)
					{
						const float Kz = (r_cy - r) * r_fy_inv;
						const float Ky = (r_cx - c) * r_fx_inv;
						const float D = src_obs.rangeImage.coeff(r,c);
						pca.setPointXYZ(idx++,
							D,        // x
							Ky * D,   // y
							Kz * D    // z
							);
					}
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
			size_t idx=0;
			for (int r=0;r<H;r++)
				for (int c=0;c<W;c++)
				{
					const float Ky = (r_cx - c) * r_fx_inv;
					const float Kz = (r_cy - r) * r_fy_inv;
					const float D = src_obs.rangeImage.coeff(r,c);
					pca.setPointXYZ(idx++,
						D / std::sqrt(1+Ky*Ky+Kz*Kz), // x
						Ky * D,   // y
						Kz * D    // z
						);
				}
		}

		// -------------------------------------------------------------
		// Stage 2/3: Project local points into RGB image to get colors
		// -------------------------------------------------------------

		// ...

		// ------------------------------------------------------------
		// Stage 3/3: Apply 6D transformations
		// ------------------------------------------------------------
		if (takeIntoAccountSensorPoseOnRobot || robotPoseInTheWorld)
		{
			mrpt::poses::CPose3D  transf_to_apply; // Either ROBOTPOSE or ROBOTPOSE(+)SENSORPOSE or SENSORPOSE
			if (takeIntoAccountSensorPoseOnRobot)
				transf_to_apply = src_obs.sensorPose;
			if (robotPoseInTheWorld)
				transf_to_apply.composeFrom(*robotPoseInTheWorld, CPose3D(transf_to_apply));

			// ...
		}

	} // end of project3DPointsFromDepthImageInto

	// Auxiliary functions which implement proyection of 3D point clouds:
	template <class POINTMAP>
	void do_project_3d_pointcloud(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca)
	{
		size_t idx=0;
		for (int r=0;r<H;r++)
			for (int c=0;c<W;c++)
			{
				const float D = rangeImage.coeff(r,c);
				pca.setPointXYZ(idx++,
					D,          // x
					*kys++ * D, // y
					*kzs++ * D  // z
					);
			}
	}

	// Auxiliary functions which implement proyection of 3D point clouds:
	template <class POINTMAP>
	void do_project_3d_pointcloud_SSE2(const int H,const int W,const float *kys,const float *kzs,const mrpt::math::CMatrix &rangeImage, mrpt::utils::PointCloudAdapter<POINTMAP> &pca)
	{
	#if MRPT_HAS_SSE2
			// Use optimized version:
			const int W_4 = W >> 2;  // /=4 , since we process 4 values at a time.
			size_t idx=0;
			EIGEN_ALIGN16 float xs[4],ys[4],zs[4];
			for (int r=0;r<H;r++)
			{
				const float *D_ptr = &rangeImage.coeffRef(r,0);  // Matrices are 16-aligned

				for (int c=0;c<W_4;c++)
				{
					const __m128 D = _mm_load_ps(D_ptr);

					const __m128 KY = _mm_load_ps(kys);
					const __m128 KZ = _mm_load_ps(kzs);

					_mm_storeu_ps(xs , D);
					_mm_storeu_ps(ys , _mm_mul_ps(KY,D));
					_mm_storeu_ps(zs , _mm_mul_ps(KZ,D));

					D_ptr+=4;
					kys+=4;
					kzs+=4;
					pca.setPointXYZ(idx++,xs[0],ys[0],zs[0]);
					pca.setPointXYZ(idx++,xs[1],ys[1],zs[1]);
					pca.setPointXYZ(idx++,xs[2],ys[2],zs[2]);
					pca.setPointXYZ(idx++,xs[3],ys[3],zs[3]);
				}
			}
	#endif
	}


} // End of namespace
} // End of namespace
} // End of namespace

#endif
