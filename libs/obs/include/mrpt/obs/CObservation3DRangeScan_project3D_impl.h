/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/cpu.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/obs/T3DPointsProjectionParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/opengl/pointcloud_adapters.h>

#include <Eigen/Dense>	// block<>()

namespace mrpt::obs::detail
{
// Auxiliary functions which implement SSE-optimized proyection of 3D point
// cloud:
template <class POINTMAP>
void do_project_3d_pointcloud(
	const int H, const int W, const float* kxs, const float* kys,
	const float* kzs, mrpt::math::CMatrix_u16& rangeImage,
	const float rangeUnits, mrpt::opengl::PointCloudAdapter<POINTMAP>& pca,
	std::vector<uint16_t>& idxs_x, std::vector<uint16_t>& idxs_y,
	const mrpt::obs::TRangeImageFilterParams& fp, bool MAKE_ORGANIZED,
	const int DECIM);
template <class POINTMAP>
void do_project_3d_pointcloud_SSE2(
	const int H, const int W, const float* kxs, const float* kys,
	const float* kzs, mrpt::math::CMatrix_u16& rangeImage,
	const float rangeUnits, mrpt::opengl::PointCloudAdapter<POINTMAP>& pca,
	std::vector<uint16_t>& idxs_x, std::vector<uint16_t>& idxs_y,
	const mrpt::obs::TRangeImageFilterParams& fp, bool MAKE_ORGANIZED);

template <typename POINTMAP>
inline void range2XYZ_LUT(
	mrpt::opengl::PointCloudAdapter<POINTMAP>& pca,
	mrpt::obs::CObservation3DRangeScan& src_obs,
	const mrpt::obs::T3DPointsProjectionParams& pp,
	const mrpt::obs::TRangeImageFilterParams& fp, const int H, const int W,
	const int DECIM, const bool use_rotated_LUT)
{
	const size_t WH = W * H;
	const auto& lut = src_obs.get_unproj_lut();

	// Select between coordinates wrt the robot/vehicle, or local wrt sensor:
	const auto& Kxs = use_rotated_LUT ? lut.Kxs_rot : lut.Kxs;
	const auto& Kys = use_rotated_LUT ? lut.Kys_rot : lut.Kys;
	const auto& Kzs = use_rotated_LUT ? lut.Kzs_rot : lut.Kzs;

	ASSERT_EQUAL_(WH, size_t(Kxs.size()));
	ASSERT_EQUAL_(WH, size_t(Kys.size()));
	ASSERT_EQUAL_(WH, size_t(Kzs.size()));
	const float* kxs = &Kxs[0];
	const float* kys = &Kys[0];
	const float* kzs = &Kzs[0];

	if (fp.rangeMask_min)
	{  // sanity check:
		ASSERT_EQUAL_(fp.rangeMask_min->cols(), src_obs.rangeImage.cols());
		ASSERT_EQUAL_(fp.rangeMask_min->rows(), src_obs.rangeImage.rows());
	}
	if (fp.rangeMask_max)
	{  // sanity check:
		ASSERT_EQUAL_(fp.rangeMask_max->cols(), src_obs.rangeImage.cols());
		ASSERT_EQUAL_(fp.rangeMask_max->rows(), src_obs.rangeImage.rows());
	}

	mrpt::math::CMatrix_u16* ri = pp.layer.empty()
		? &src_obs.rangeImage
		: &src_obs.rangeImageOtherLayers.at(pp.layer);

#if MRPT_HAS_SSE2
	// if image width is not 8*N, use standard method
	if ((W & 0x07) == 0 && pp.USE_SSE2 && DECIM == 1 &&
		mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
		do_project_3d_pointcloud_SSE2(
			H, W, kxs, kys, kzs, *ri, src_obs.rangeUnits, pca,
			src_obs.points3D_idxs_x, src_obs.points3D_idxs_y, fp,
			pp.MAKE_ORGANIZED);
	else
#endif
		do_project_3d_pointcloud(
			H, W, kxs, kys, kzs, *ri, src_obs.rangeUnits, pca,
			src_obs.points3D_idxs_x, src_obs.points3D_idxs_y, fp,
			pp.MAKE_ORGANIZED, DECIM);

	// Do final traslation, if we were generating points in the vehicle frame:
	if (use_rotated_LUT)
	{
		const size_t nPts = pca.size();
		const auto trans = mrpt::math::TPoint3Df(
			src_obs.sensorPose.x(), src_obs.sensorPose.y(),
			src_obs.sensorPose.z());
		for (size_t i = 0; i < nPts; i++)
		{
			mrpt::math::TPoint3Df pt;
			pca.getPointXYZ(i, pt.x, pt.y, pt.z);
			pt += trans;
			pca.setPointXYZ(i, pt.x, pt.y, pt.z);
		}
	}
}

template <class POINTMAP>
void unprojectInto(
	mrpt::obs::CObservation3DRangeScan& src_obs, POINTMAP& dest_pointcloud,
	const mrpt::obs::T3DPointsProjectionParams& pp,
	const mrpt::obs::TRangeImageFilterParams& fp)
{
	using namespace mrpt::math;

	mrpt::opengl::PointCloudAdapter<POINTMAP> pca(dest_pointcloud);

	if (!src_obs.hasRangeImage)
	{
		pca.resize(0);
		return;
	}
	if (!pp.layer.empty())
	{
		ASSERT_EQUAL_(src_obs.rangeImageOtherLayers.count(pp.layer), 1);
		const auto& ri = src_obs.rangeImageOtherLayers.at(pp.layer);

		ASSERT_EQUAL_(ri.cols(), src_obs.rangeImage.cols());
		ASSERT_EQUAL_(ri.rows(), src_obs.rangeImage.rows());
	}

	// Decide whether we could directly use the precomputed LUT of pixel
	// directions including the sensor pose:
	const bool use_rotated_LUT =
		// The user DO want the transformed points
		pp.takeIntoAccountSensorPoseOnRobot &&
		// We don't have colors (local coordinates needed then)
		(!pca.HAS_RGB || !src_obs.hasIntensityImage) &&
		// and we are not to use a global pose in the world/map
		!pp.robotPoseInTheWorld;

	// ------------------------------------------------------------
	// Stage 1/3: Create 3D point cloud local coordinates
	// ------------------------------------------------------------
	const int W = src_obs.rangeImage.cols();
	const int H = src_obs.rangeImage.rows();
	ASSERT_(W != 0 && H != 0);
	const size_t WH = W * H;

	const auto DECIM = pp.decimation;
	if (pp.decimation == 1)
	{
		// No decimation: one point per range image pixel

		// This is to make sure points3D_idxs_{x,y} have the expected sizes
		src_obs.resizePoints3DVectors(WH);
		// Reserve memory for 3D points. It will be later resized again to the
		// actual number of valid points
		pca.resize(WH);
		if (pp.MAKE_ORGANIZED) pca.setDimensions(H, W);
	}
	else
	{
		// Decimate range image:
		ASSERTMSG_(
			(W % DECIM) == 0 && (H % DECIM == 0),
			"Width/Height are not an exact multiple of decimation");
		const int Wd = W / DECIM;
		const int Hd = H / DECIM;
		ASSERT_(Wd != 0 && Hd != 0);
		const size_t WHd = Wd * Hd;

		src_obs.resizePoints3DVectors(WHd);
		pca.resize(WHd);
		if (pp.MAKE_ORGANIZED) pca.setDimensions(Hd, Wd);
	}
	range2XYZ_LUT<POINTMAP>(pca, src_obs, pp, fp, H, W, DECIM, use_rotated_LUT);

	// -------------------------------------------------------------
	// Stage 2/3: Project local points into RGB image to get colors
	// -------------------------------------------------------------
	if constexpr (pca.HAS_RGB)
	{
		if (src_obs.hasIntensityImage)
		{
			const int imgW = src_obs.intensityImage.getWidth();
			const int imgH = src_obs.intensityImage.getHeight();
			const bool hasColorIntensityImg = src_obs.intensityImage.isColor();

			const float cx = src_obs.cameraParamsIntensity.cx();
			const float cy = src_obs.cameraParamsIntensity.cy();
			const float fx = src_obs.cameraParamsIntensity.fx();
			const float fy = src_obs.cameraParamsIntensity.fy();

			// Unless we are in a special case (both depth & RGB images
			// coincide)...
			const bool isDirectCorresp =
				src_obs.doDepthAndIntensityCamerasCoincide();

			// ...precompute the inverse of the pose transformation out of
			// the
			// loop,
			//  store as a 4x4 homogeneous matrix to exploit SSE
			//  optimizations
			//  below:
			mrpt::math::CMatrixFixed<float, 4, 4> T_inv;
			if (!isDirectCorresp)
			{
				mrpt::math::CMatrixFixed<double, 3, 3> R_inv;
				mrpt::math::CMatrixFixed<double, 3, 1> t_inv;
				mrpt::math::homogeneousMatrixInverse(
					src_obs.relativePoseIntensityWRTDepth.getRotationMatrix(),
					src_obs.relativePoseIntensityWRTDepth.m_coords, R_inv,
					t_inv);

				T_inv(3, 3) = 1;
				T_inv.insertMatrix(0, 0, R_inv.cast_float());
				T_inv.insertMatrix(0, 3, t_inv.cast_float());
			}

			CVectorFixedFloat<4> pt_wrt_color, pt_wrt_depth;
			pt_wrt_depth[3] = 1;
			mrpt::img::TColor pCol;

			// For each local point:
			const size_t nPts = pca.size();
			const auto& iimg = src_obs.intensityImage;
			const uint8_t* img_data = iimg.ptrLine<uint8_t>(0);
			const auto img_stride = iimg.getRowStride();
			for (size_t i = 0; i < nPts; i++)
			{
				int img_idx_x,
					img_idx_y;	// projected pixel coordinates, in the
				// RGB image plane
				bool pointWithinImage = false;
				if (isDirectCorresp)
				{
					pointWithinImage = true;
					img_idx_x = src_obs.points3D_idxs_x[i];
					img_idx_y = src_obs.points3D_idxs_y[i];
				}
				else
				{
					// Project point, which is now in "pca" in local
					// coordinates
					// wrt the depth camera, into the intensity camera:
					pca.getPointXYZ(
						i, pt_wrt_depth[0], pt_wrt_depth[1], pt_wrt_depth[2]);
					pt_wrt_color = T_inv * pt_wrt_depth;

					// Project to image plane:
					if (pt_wrt_color[2])
					{
						img_idx_x = mrpt::round(
							cx + fx * pt_wrt_color[0] / pt_wrt_color[2]);
						img_idx_y = mrpt::round(
							cy + fy * pt_wrt_color[1] / pt_wrt_color[2]);
						pointWithinImage = img_idx_x >= 0 && img_idx_x < imgW &&
							img_idx_y >= 0 && img_idx_y < imgH;
					}
				}

				if (pointWithinImage)
				{
					if (hasColorIntensityImg)
					{
						const auto px_idx =
							img_stride * img_idx_y + 3 * img_idx_x;
						pCol.R = img_data[px_idx + 2];
						pCol.G = img_data[px_idx + 1];
						pCol.B = img_data[px_idx + 0];
					}
					else
					{
						const auto px_idx = img_stride * img_idx_y + img_idx_x;
						pCol.R = pCol.G = pCol.B = img_data[px_idx];
					}
				}
				else
				{
					pCol.R = pCol.G = pCol.B = 255;
				}
				// Set color:
				pca.setPointRGBu8(i, pCol.R, pCol.G, pCol.B);
			}  // end for each point
		}  // end if src_obs has intensity image
	}
	// ...

	// ------------------------------------------------------------
	// Stage 3/3: Apply 6D transformations
	// ------------------------------------------------------------
	if (!use_rotated_LUT &&
		(pp.takeIntoAccountSensorPoseOnRobot || pp.robotPoseInTheWorld))
	{
		mrpt::poses::CPose3D transf_to_apply;  // Either ROBOTPOSE or
		// ROBOTPOSE(+)SENSORPOSE or
		// SENSORPOSE
		if (pp.takeIntoAccountSensorPoseOnRobot)
			transf_to_apply = src_obs.sensorPose;
		if (pp.robotPoseInTheWorld)
			transf_to_apply.composeFrom(
				*pp.robotPoseInTheWorld, mrpt::poses::CPose3D(transf_to_apply));

		const auto HM =
			transf_to_apply
				.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>()
				.cast_float();
		mrpt::math::CVectorFixedFloat<4> pt, pt_transf;
		pt[3] = 1;

		const size_t nPts = pca.size();
		for (size_t i = 0; i < nPts; i++)
		{
			pca.getPointXYZ(i, pt[0], pt[1], pt[2]);
			pt_transf = HM * pt;
			pca.setPointXYZ(i, pt_transf[0], pt_transf[1], pt_transf[2]);
		}
	}
}  // end of unprojectInto

// Auxiliary functions which implement (un)projection of 3D point clouds:
template <class POINTMAP>
inline void do_project_3d_pointcloud(
	const int H, const int W, const float* kxs, const float* kys,
	const float* kzs, mrpt::math::CMatrix_u16& rangeImage,
	const float rangeUnits, mrpt::opengl::PointCloudAdapter<POINTMAP>& pca,
	std::vector<uint16_t>& idxs_x, std::vector<uint16_t>& idxs_y,
	const mrpt::obs::TRangeImageFilterParams& fp, bool MAKE_ORGANIZED,
	const int DECIM)
{
	TRangeImageFilter rif(fp);
	// Preconditions: minRangeMask() has the right size
	size_t idx = 0;
	if (DECIM == 1)
	{
		for (int r = 0; r < H; r++)
			for (int c = 0; c < W; c++)
			{
				const float D = rangeImage.coeff(r, c) * rangeUnits;
				// LUT projection coefs:
				const auto kx = *kxs++, ky = *kys++, kz = *kzs++;
				if (!rif.do_range_filter(r, c, D))
				{
					if (MAKE_ORGANIZED) pca.setInvalidPoint(idx++);
					if (fp.mark_invalid_ranges) rangeImage.coeffRef(r, c) = 0;
					continue;
				}
				pca.setPointXYZ(idx, kx * D, ky * D /*y*/, kz * D /*z*/);
				idxs_x[idx] = c;
				idxs_y[idx] = r;
				++idx;
			}
	}
	else
	{
		const int Hd = H / DECIM, Wd = W / DECIM;

		for (int rd = 0; rd < Hd; rd++)
			for (int cd = 0; cd < Wd; cd++)
			{
				bool valid_pt = false;
				float min_d = std::numeric_limits<float>::max();
				for (int rb = 0; rb < DECIM; rb++)
					for (int cb = 0; cb < DECIM; cb++)
					{
						const auto r = rd * DECIM + rb, c = cd * DECIM + cb;
						const float D = rangeImage.coeff(r, c) * rangeUnits;
						if (rif.do_range_filter(r, c, D))
						{
							valid_pt = true;
							if (D < min_d) min_d = D;
						}
						else
						{
							if (fp.mark_invalid_ranges)
								rangeImage.coeffRef(r, c) = 0;
						}
					}
				if (!valid_pt)
				{
					if (MAKE_ORGANIZED) pca.setInvalidPoint(idx++);
					continue;
				}
				const auto eq_r = rd * DECIM + DECIM / 2,
						   eq_c = cd * DECIM + DECIM / 2;
				const auto eq_idx = eq_c + eq_r * W;
				const auto kx = kxs[eq_idx], ky = kys[eq_idx], kz = kzs[eq_idx];
				pca.setPointXYZ(
					idx, kx * min_d /*x*/, ky * min_d /*y*/, kz * min_d /*z*/);
				idxs_x[idx] = eq_c;
				idxs_y[idx] = eq_r;
				++idx;
			}
	}
	pca.resize(idx);
	// Make sure indices are also resized down to the actual number of points,
	// even if they are not part of the object PCA refers to:
	idxs_x.resize(idx);
	idxs_y.resize(idx);
}

// Auxiliary functions which implement (un)projection of 3D point clouds:
template <class POINTMAP>
inline void do_project_3d_pointcloud_SSE2(
	const int H, const int W, const float* kxs, const float* kys,
	const float* kzs, mrpt::math::CMatrix_u16& rangeImage,
	const float rangeUnits, mrpt::opengl::PointCloudAdapter<POINTMAP>& pca,
	std::vector<uint16_t>& idxs_x, std::vector<uint16_t>& idxs_y,
	const mrpt::obs::TRangeImageFilterParams& fp, bool MAKE_ORGANIZED)
{
#if MRPT_HAS_SSE2
	// Preconditions: minRangeMask() has the right size
	// Use optimized version:
	const int W_4 = W >> 2;	 // /=4 , since we process 4 values at a time.
	size_t idx = 0;
	alignas(MRPT_MAX_STATIC_ALIGN_BYTES) float xs[4], ys[4], zs[4];
	const __m128 D_zeros = _mm_set_ps(.0f, .0f, .0f, .0f);
	const __m128 xormask = (fp.rangeCheckBetween)
		? _mm_cmpneq_ps(D_zeros, D_zeros)
		:  // want points BETWEEN min and max to be valid
		_mm_cmpeq_ps(
			D_zeros,
			D_zeros);  // want points OUTSIDE of min and max to be valid
	for (int r = 0; r < H; r++)
	{
		const uint16_t* Du16_ptr = &rangeImage(r, 0);
		const float* Dgt_ptr =
			!fp.rangeMask_min ? nullptr : &(*fp.rangeMask_min)(r, 0);
		const float* Dlt_ptr =
			!fp.rangeMask_max ? nullptr : &(*fp.rangeMask_max)(r, 0);

		for (int c = 0; c < W_4; c++)
		{
			// Let the compiler optimize this as MMX instructions (tested in
			// godbolt):
			alignas(16) float tmp[4];
			for (int b = 0; b < 4; b++)
				tmp[b] = Du16_ptr[b] * rangeUnits;

			const __m128 D = _mm_load_ps(&tmp[0]);
			const __m128 nz_mask = _mm_cmpgt_ps(D, D_zeros);
			__m128 valid_range_mask;
			if (!fp.rangeMask_min && !fp.rangeMask_max)
			{  // No filter: just skip D=0 points
				valid_range_mask = nz_mask;
			}
			else
			{
				if (!fp.rangeMask_min || !fp.rangeMask_max)
				{  // Only one filter
					if (fp.rangeMask_min)
					{
						const __m128 Dmin = _mm_load_ps(Dgt_ptr);
						valid_range_mask = _mm_or_ps(
							_mm_cmpgt_ps(D, Dmin), _mm_cmpeq_ps(Dmin, D_zeros));
					}
					else
					{
						const __m128 Dmax = _mm_load_ps(Dlt_ptr);
						valid_range_mask = _mm_or_ps(
							_mm_cmplt_ps(D, Dmax), _mm_cmpeq_ps(Dmax, D_zeros));
					}
					valid_range_mask = _mm_and_ps(
						valid_range_mask, nz_mask);	 // Filter out D=0 points
				}
				else
				{
					// We have both: D>Dmin and D<Dmax conditions, with XOR to
					// optionally invert the selection:
					const __m128 Dmin = _mm_load_ps(Dgt_ptr);
					const __m128 Dmax = _mm_load_ps(Dlt_ptr);

					const __m128 gt_mask = _mm_or_ps(
						_mm_cmpgt_ps(D, Dmin), _mm_cmpeq_ps(Dmin, D_zeros));
					const __m128 lt_mask = _mm_or_ps(
						_mm_cmplt_ps(D, Dmax), _mm_cmpeq_ps(Dmax, D_zeros));
					// (D>Dmin && D<Dmax) & skip points at zero
					valid_range_mask =
						_mm_and_ps(nz_mask, _mm_and_ps(gt_mask, lt_mask));
					valid_range_mask = _mm_xor_ps(valid_range_mask, xormask);
					// Add the case of D_min & D_max = 0 (no filtering)
					valid_range_mask = _mm_or_ps(
						valid_range_mask,
						_mm_and_ps(
							_mm_cmpeq_ps(Dmin, D_zeros),
							_mm_cmpeq_ps(Dmax, D_zeros)));
					// Finally, ensure no invalid ranges get thru:
					valid_range_mask = _mm_and_ps(valid_range_mask, nz_mask);
				}
			}
			const int valid_range_maski = _mm_movemask_epi8(
				_mm_castps_si128(valid_range_mask));  // 0x{f|0}{f|0}{f|0}{f|0}
			if (valid_range_maski != 0)	 // Any of the 4 values is valid?
			{
				const __m128 KX = _mm_load_ps(kxs);
				const __m128 KY = _mm_load_ps(kys);
				const __m128 KZ = _mm_load_ps(kzs);

				_mm_storeu_ps(xs, _mm_mul_ps(KX, D));
				_mm_storeu_ps(ys, _mm_mul_ps(KY, D));
				_mm_storeu_ps(zs, _mm_mul_ps(KZ, D));

				for (int q = 0; q < 4; q++)
				{
					const int actual_c = (c << 2) + q;
					if ((valid_range_maski & (1 << (q * 4))) != 0)
					{
						pca.setPointXYZ(idx, xs[q], ys[q], zs[q]);
						idxs_x[idx] = actual_c;
						idxs_y[idx] = r;
						++idx;
					}
					else
					{
						if (MAKE_ORGANIZED)
						{
							pca.setInvalidPoint(idx);
							++idx;
						}
						if (fp.mark_invalid_ranges)
							rangeImage.coeffRef(r, actual_c) = 0;
					}
				}
			}
			else if (MAKE_ORGANIZED)
			{
				for (int q = 0; q < 4; q++)
				{
					pca.setInvalidPoint(idx);
					++idx;
					const int actual_c = (c << 2) + q;
					if (fp.mark_invalid_ranges)
						rangeImage.coeffRef(r, actual_c) = 0;
				}
			}
			Du16_ptr += 4;
			if (Dgt_ptr) Dgt_ptr += 4;
			if (Dlt_ptr) Dlt_ptr += 4;
			kxs += 4;
			kys += 4;
			kzs += 4;
		}
	}
	pca.resize(idx);
	// Make sure indices are also resized down to the actual number of points,
	// even if they are not part of the object PCA refers to:
	idxs_x.resize(idx);
	idxs_y.resize(idx);
#endif
}
}  // namespace mrpt::obs::detail
