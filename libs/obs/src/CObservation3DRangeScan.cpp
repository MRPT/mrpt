/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/3rdparty/do_opencv_includes.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/bits_mem.h>  // vector_strong_clear
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/math/CLevenbergMarquardt.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/ops_containers.h>  // norm(), etc.
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <cstring>
#include <limits>
#include <mutex>
#include <unordered_map>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::img;
using mrpt::config::CConfigFileMemory;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation3DRangeScan, CObservation, mrpt::obs)

// Static LUT:

// Index info: camera parameters + range_is_depth
struct LUT_info
{
	mrpt::img::TCamera calib;
	mrpt::poses::CPose3D sensorPose;
	bool range_is_depth;
};

inline bool operator==(const LUT_info& a, const LUT_info& o)
{
	return a.calib == o.calib && a.sensorPose == o.sensorPose &&
		   a.range_is_depth == o.range_is_depth;
}

namespace std
{
template <>
struct hash<LUT_info>
{
	size_t operator()(const LUT_info& k) const
	{
		size_t res = 17;
		res = res * 31 + hash<mrpt::img::TCamera>()(k.calib);
		res = res * 31 + hash<bool>()(k.range_is_depth);
		for (unsigned int i = 0; i < 6; i++)
			res = res * 31 + hash<double>()(k.sensorPose[i]);
		return res;
	}
};
}  // namespace std

static std::unordered_map<LUT_info, CObservation3DRangeScan::unproject_LUT_t>
	LUTs;
static std::mutex LUTs_mtx;

const CObservation3DRangeScan::unproject_LUT_t&
	CObservation3DRangeScan::get_unproj_lut() const
{
#if MRPT_HAS_OPENCV
	// Access to, or create upon first usage:
	LUT_info linfo;
	linfo.calib = this->cameraParams;
	linfo.sensorPose = this->sensorPose;
	linfo.range_is_depth = this->range_is_depth;

	LUTs_mtx.lock();
	// Protect against infinite memory growth: imagine sensorPose gets changed
	// everytime for a sweeping sensor, etc.
	// Unlikely, but "just in case" (TM)
	if (LUTs.size() > 100) LUTs.clear();

	const unproject_LUT_t& ret = LUTs[linfo];

	LUTs_mtx.unlock();

	ASSERT_EQUAL_(rangeImage.cols(), static_cast<int>(cameraParams.ncols));
	ASSERT_EQUAL_(rangeImage.rows(), static_cast<int>(cameraParams.nrows));

	// already existed and was filled?
	unsigned int H = cameraParams.nrows, W = cameraParams.ncols;
	const size_t WH = W * H;
	if (ret.Kxs.size() == WH) return ret;

	// fill LUT upon first use:
	auto& lut = const_cast<unproject_LUT_t&>(ret);

	lut.Kxs.resize(WH);
	lut.Kys.resize(WH);
	lut.Kzs.resize(WH);
	lut.Kxs_rot.resize(WH);
	lut.Kys_rot.resize(WH);
	lut.Kzs_rot.resize(WH);

	// Undistort all points:
	cv::Mat pts(1, WH, CV_32FC2), undistort_pts(1, WH, CV_32FC2);

	const auto& intrMat = cameraParams.intrinsicParams;
	const auto& dist = cameraParams.dist;
	cv::Mat cv_distortion(
		1, dist.size(), CV_64F, const_cast<double*>(&dist[0]));
	cv::Mat cv_intrinsics(3, 3, CV_64F);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cv_intrinsics.at<double>(i, j) = intrMat(i, j);

	for (unsigned int r = 0; r < H; r++)
		for (unsigned int c = 0; c < W; c++)
		{
			auto& p = pts.at<cv::Vec2f>(r * W + c);
			p[0] = c;
			p[1] = r;
		}

	cv::undistortPoints(pts, undistort_pts, cv_intrinsics, cv_distortion);

	// Note: undistort_pts now holds point coordinates with (-1,-1)=top left,
	// (1,1)=bottom-right

	ASSERT_EQUAL_(undistort_pts.size().area(), static_cast<int>(WH));
	undistort_pts.reshape(WH);

	float* kxs = &lut.Kxs[0];
	float* kys = &lut.Kys[0];
	float* kzs = &lut.Kzs[0];
	float* kxs_rot = &lut.Kxs_rot[0];
	float* kys_rot = &lut.Kys_rot[0];
	float* kzs_rot = &lut.Kzs_rot[0];

	for (size_t idx = 0; idx < WH; idx++)
	{
		const auto& p = undistort_pts.at<cv::Vec2f>(idx);
		const float c = p[0], r = p[1];

		// XYZ -> (-Y,-Z, X)
		auto v = mrpt::math::TPoint3Df(1.0f, -c, -r);

		// Range instead of depth? Use a unit vector:
		if (!this->range_is_depth) v *= 1.0f / v.norm();

		// compute also the rotated version:
		auto v_rot = sensorPose.rotateVector(v);

		*kxs++ = v.x;
		*kys++ = v.y;
		*kzs++ = v.z;

		*kxs_rot++ = v_rot.x;
		*kys_rot++ = v_rot.y;
		*kzs_rot++ = v_rot.z;
	}

	return ret;
#else
	THROW_EXCEPTION("This method requires MRPT built against OpenCV");
#endif
}

static bool EXTERNALS_AS_TEXT_value = false;
void CObservation3DRangeScan::EXTERNALS_AS_TEXT(bool value)
{
	EXTERNALS_AS_TEXT_value = value;
}
bool CObservation3DRangeScan::EXTERNALS_AS_TEXT()
{
	return EXTERNALS_AS_TEXT_value;
}

// Whether to use a memory pool for 3D points:
#define COBS3DRANGE_USE_MEMPOOL

// Do performance time logging?
//#define  PROJ3D_PERFLOG

// Data types for memory pooling CObservation3DRangeScan:
#ifdef COBS3DRANGE_USE_MEMPOOL

#include <mrpt/system/CGenericMemoryPool.h>

// Memory pool for XYZ points ----------------
struct CObservation3DRangeScan_Points_MemPoolParams
{
	/** Width*Height, that is, the number of 3D points */
	size_t WH{0};
	inline bool isSuitable(
		const CObservation3DRangeScan_Points_MemPoolParams& req) const
	{
		return WH >= req.WH;
	}
};
struct CObservation3DRangeScan_Points_MemPoolData
{
	std::vector<float> pts_x, pts_y, pts_z;
	/** for each point, the corresponding (x,y) pixel coordinates */
	std::vector<uint16_t> idxs_x, idxs_y;
};
using TMyPointsMemPool = mrpt::system::CGenericMemoryPool<
	CObservation3DRangeScan_Points_MemPoolParams,
	CObservation3DRangeScan_Points_MemPoolData>;

// Memory pool for the rangeImage matrix ----------------
struct CObservation3DRangeScan_Ranges_MemPoolParams
{
	/** Size of matrix */
	int H{0}, W{0};
	inline bool isSuitable(
		const CObservation3DRangeScan_Ranges_MemPoolParams& req) const
	{
		return H == req.H && W == req.W;
	}
};
struct CObservation3DRangeScan_Ranges_MemPoolData
{
	mrpt::math::CMatrix_u16 rangeImage;
};
using TMyRangesMemPool = mrpt::system::CGenericMemoryPool<
	CObservation3DRangeScan_Ranges_MemPoolParams,
	CObservation3DRangeScan_Ranges_MemPoolData>;

static void mempool_donate_xyz_buffers(CObservation3DRangeScan& obs)
{
	if (obs.points3D_x.empty()) return;
	// Before dying, donate my memory to the pool for the joy of future
	// class-brothers...
	TMyPointsMemPool* pool = TMyPointsMemPool::getInstance();
	if (!pool) return;

	CObservation3DRangeScan_Points_MemPoolParams mem_params;
	mem_params.WH = obs.points3D_x.capacity();
	if (obs.points3D_y.capacity() != mem_params.WH)
		obs.points3D_y.resize(mem_params.WH);
	if (obs.points3D_z.capacity() != mem_params.WH)
		obs.points3D_z.resize(mem_params.WH);
	if (obs.points3D_idxs_x.capacity() != mem_params.WH)
		obs.points3D_idxs_x.resize(mem_params.WH);
	if (obs.points3D_idxs_y.capacity() != mem_params.WH)
		obs.points3D_idxs_y.resize(mem_params.WH);

	auto* mem_block = new CObservation3DRangeScan_Points_MemPoolData();
	obs.points3D_x.swap(mem_block->pts_x);
	obs.points3D_y.swap(mem_block->pts_y);
	obs.points3D_z.swap(mem_block->pts_z);
	obs.points3D_idxs_x.swap(mem_block->idxs_x);
	obs.points3D_idxs_y.swap(mem_block->idxs_y);

	pool->dump_to_pool(mem_params, mem_block);
}
void mempool_donate_range_matrix(CObservation3DRangeScan& obs)
{
	if (obs.rangeImage.cols() == 0 || obs.rangeImage.rows() == 0) return;

	// Before dying, donate my memory to the pool for the joy of future
	// class-brothers...
	TMyRangesMemPool* pool = TMyRangesMemPool::getInstance();
	if (!pool) return;

	CObservation3DRangeScan_Ranges_MemPoolParams mem_params;
	mem_params.H = obs.rangeImage.rows();
	mem_params.W = obs.rangeImage.cols();

	auto* mem_block = new CObservation3DRangeScan_Ranges_MemPoolData();
	obs.rangeImage.swap(mem_block->rangeImage);

	pool->dump_to_pool(mem_params, mem_block);
}
#endif

CObservation3DRangeScan::~CObservation3DRangeScan()
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	mempool_donate_xyz_buffers(*this);
	mempool_donate_range_matrix(*this);
#endif
}

uint8_t CObservation3DRangeScan::serializeGetVersion() const { return 10; }
void CObservation3DRangeScan::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	// The data
	out << maxRange << sensorPose;
	out << hasPoints3D;
	if (hasPoints3D)
	{
		ASSERT_(
			points3D_x.size() == points3D_y.size() &&
			points3D_x.size() == points3D_z.size() &&
			points3D_idxs_x.size() == points3D_x.size() &&
			points3D_idxs_y.size() == points3D_x.size());
		uint32_t N = points3D_x.size();
		out << N;
		if (N)
		{
			out.WriteBufferFixEndianness(&points3D_x[0], N);
			out.WriteBufferFixEndianness(&points3D_y[0], N);
			out.WriteBufferFixEndianness(&points3D_z[0], N);
			out.WriteBufferFixEndianness(&points3D_idxs_x[0], N);  // New in v8
			out.WriteBufferFixEndianness(&points3D_idxs_y[0], N);  // New in v8
		}
	}

	out << hasRangeImage;
	out << rangeUnits;  // new in v9
	if (hasRangeImage)
	{
		out.WriteAs<uint32_t>(rangeImage.rows());
		out.WriteAs<uint32_t>(rangeImage.cols());
		if (rangeImage.size() != 0)
			out.WriteBufferFixEndianness<uint16_t>(
				rangeImage.data(), rangeImage.size());
	}
	out << hasIntensityImage;
	if (hasIntensityImage) out << intensityImage;
	out << hasConfidenceImage;
	if (hasConfidenceImage) out << confidenceImage;
	out << cameraParams;  // New in v2
	out << cameraParamsIntensity;  // New in v4
	out << relativePoseIntensityWRTDepth;  // New in v4

	out << stdError;
	out << timestamp;
	out << sensorLabel;

	// New in v3:
	out << m_points3D_external_stored << m_points3D_external_file;
	out << m_rangeImage_external_stored << m_rangeImage_external_file;

	// New in v5:
	out << range_is_depth;

	// New in v6:
	out << static_cast<int8_t>(intensityImageChannel);

	// New in v7:
	out << hasPixelLabels();
	if (hasPixelLabels())
	{
		pixelLabels->writeToStream(out);
	}

	// v10:
	out.WriteAs<uint8_t>(rangeImageOtherLayers.size());
	for (const auto& kv : rangeImageOtherLayers)
	{
		out << kv.first;
		const auto& ri = kv.second;
		out.WriteAs<uint32_t>(ri.cols());
		out.WriteAs<uint32_t>(ri.rows());
		if (!ri.empty())
			out.WriteBufferFixEndianness<uint16_t>(ri.data(), ri.size());
	}
}

void CObservation3DRangeScan::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		{
			uint32_t N;

			in >> maxRange >> sensorPose;

			if (version > 0)
				in >> hasPoints3D;
			else
				hasPoints3D = true;

			if (hasPoints3D)
			{
				in >> N;
				resizePoints3DVectors(N);

				if (N)
				{
					in.ReadBufferFixEndianness(&points3D_x[0], N);
					in.ReadBufferFixEndianness(&points3D_y[0], N);
					in.ReadBufferFixEndianness(&points3D_z[0], N);

					if (version == 0)
					{
						vector<char> validRange(N);  // for v0.
						in.ReadBuffer(
							&validRange[0], sizeof(validRange[0]) * N);
					}
					if (version >= 8)
					{
						in.ReadBufferFixEndianness(&points3D_idxs_x[0], N);
						in.ReadBufferFixEndianness(&points3D_idxs_y[0], N);
					}
				}
			}
			else
			{
				this->resizePoints3DVectors(0);
			}

			if (version >= 1)
			{
				in >> hasRangeImage;
				if (version >= 9)
					in >> rangeUnits;
				else
					rangeUnits = 1e-3f;  // default units

				if (hasRangeImage)
				{
					if (version < 9)
					{
						// Convert from old format:
						mrpt::math::CMatrixF ri;
						in >> ri;
						const uint32_t rows = ri.rows(), cols = ri.cols();
						ASSERT_(rows > 0 && cols > 0);

						// Call "rangeImage_setSize()" to exploit the mempool:
						rangeImage_setSize(rows, cols);

						for (uint32_t r = 0; r < rows; r++)
							for (uint32_t c = 0; c < cols; c++)
								rangeImage(r, c) = static_cast<uint16_t>(
									mrpt::round(ri(r, c) / rangeUnits));
					}
					else
					{
						const uint32_t rows = in.ReadAs<uint32_t>();
						const uint32_t cols = in.ReadAs<uint32_t>();

						// Call "rangeImage_setSize()" to exploit the mempool:
						rangeImage_setSize(rows, cols);

						// new v9:
						if (rangeImage.size() != 0)
							in.ReadBufferFixEndianness<uint16_t>(
								rangeImage.data(), rangeImage.size());
					}
				}

				in >> hasIntensityImage;
				if (hasIntensityImage) in >> intensityImage;

				in >> hasConfidenceImage;
				if (hasConfidenceImage) in >> confidenceImage;

				if (version >= 2)
				{
					in >> cameraParams;

					if (version >= 4)
					{
						in >> cameraParamsIntensity >>
							relativePoseIntensityWRTDepth;
					}
					else
					{
						cameraParamsIntensity = cameraParams;
						relativePoseIntensityWRTDepth = CPose3D();
					}
				}
			}

			in >> stdError;
			in >> timestamp;
			in >> sensorLabel;

			if (version >= 3)
			{
				// New in v3:
				in >> m_points3D_external_stored >> m_points3D_external_file;
				in >> m_rangeImage_external_stored >>
					m_rangeImage_external_file;
			}
			else
			{
				m_points3D_external_stored = false;
				m_rangeImage_external_stored = false;
			}

			if (version >= 5)
			{
				in >> range_is_depth;
			}
			else
			{
				range_is_depth = true;
			}

			if (version >= 6)
			{
				int8_t i;
				in >> i;
				intensityImageChannel = static_cast<TIntensityChannelID>(i);
			}
			else
			{
				intensityImageChannel = CH_VISIBLE;
			}

			pixelLabels.reset();  // Remove existing data first (_unique() is to
			// leave alive any user copies of the shared
			// pointer).
			if (version >= 7)
			{
				bool do_have_labels;
				in >> do_have_labels;

				if (do_have_labels)
					pixelLabels.reset(
						TPixelLabelInfoBase::readAndBuildFromStream(in));
			}

			rangeImageOtherLayers.clear();
			if (version >= 10)
			{
				const auto numLayers = in.ReadAs<uint8_t>();
				for (size_t i = 0; i < numLayers; i++)
				{
					std::string name;
					in >> name;
					auto& ri = rangeImageOtherLayers[name];

					const uint32_t rows = in.ReadAs<uint32_t>();
					const uint32_t cols = in.ReadAs<uint32_t>();
					ri.resize(rows, cols);
					if (ri.size() != 0)
						in.ReadBufferFixEndianness<uint16_t>(
							ri.data(), ri.size());
				}
			}

			// auto-fix wrong camera resolution in parameters:
			if (hasRangeImage &&
				(static_cast<int>(cameraParams.ncols) != rangeImage.cols() ||
				 static_cast<int>(cameraParams.nrows) != rangeImage.rows()))
			{
				std::cerr << "[CObservation3DRangeScan] Warning: autofixing "
							 "incorrect camera resolution in TCamera:"
						  << cameraParams.ncols << "x" << cameraParams.nrows
						  << " => " << rangeImage.cols() << "x"
						  << rangeImage.rows() << "\n";
				cameraParams.ncols = rangeImage.cols();
				cameraParams.nrows = rangeImage.rows();
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CObservation3DRangeScan::swap(CObservation3DRangeScan& o)
{
	CObservation::swap(o);

	std::swap(hasPoints3D, o.hasPoints3D);
	points3D_x.swap(o.points3D_x);
	points3D_y.swap(o.points3D_y);
	points3D_z.swap(o.points3D_z);
	points3D_idxs_x.swap(o.points3D_idxs_x);
	points3D_idxs_y.swap(o.points3D_idxs_y);
	std::swap(m_points3D_external_stored, o.m_points3D_external_stored);
	std::swap(m_points3D_external_file, o.m_points3D_external_file);

	std::swap(hasRangeImage, o.hasRangeImage);
	rangeImage.swap(o.rangeImage);
	std::swap(m_rangeImage_external_stored, o.m_rangeImage_external_stored);
	std::swap(m_rangeImage_external_file, o.m_rangeImage_external_file);

	std::swap(hasIntensityImage, o.hasIntensityImage);
	std::swap(intensityImageChannel, o.intensityImageChannel);
	intensityImage.swap(o.intensityImage);

	std::swap(hasConfidenceImage, o.hasConfidenceImage);
	confidenceImage.swap(o.confidenceImage);

	std::swap(pixelLabels, o.pixelLabels);

	std::swap(relativePoseIntensityWRTDepth, o.relativePoseIntensityWRTDepth);

	std::swap(cameraParams, o.cameraParams);
	std::swap(cameraParamsIntensity, o.cameraParamsIntensity);

	std::swap(maxRange, o.maxRange);
	std::swap(sensorPose, o.sensorPose);
	std::swap(stdError, o.stdError);

	rangeImageOtherLayers.swap(o.rangeImageOtherLayers);
}

void CObservation3DRangeScan::load() const
{
	if (hasPoints3D && m_points3D_external_stored)
	{
		const string fil = points3D_getExternalStorageFileAbsolutePath();
		if (mrpt::system::strCmpI(
				"txt", mrpt::system::extractFileExtension(fil, true)))
		{
			CMatrixFloat M;
			M.loadFromTextFile(fil);
			ASSERT_EQUAL_(M.rows(), 3);
			const auto N = M.cols();

			auto& xs = const_cast<std::vector<float>&>(points3D_x);
			auto& ys = const_cast<std::vector<float>&>(points3D_y);
			auto& zs = const_cast<std::vector<float>&>(points3D_z);
			xs.resize(N);
			ys.resize(N);
			zs.resize(N);
			std::memcpy(&xs[0], &M(0, 0), sizeof(float) * N);
			std::memcpy(&ys[0], &M(1, 0), sizeof(float) * N);
			std::memcpy(&zs[0], &M(2, 0), sizeof(float) * N);
		}
		else
		{
			mrpt::io::CFileGZInputStream fi(fil);
			auto f = mrpt::serialization::archiveFrom(fi);
			f >> const_cast<std::vector<float>&>(points3D_x) >>
				const_cast<std::vector<float>&>(points3D_y) >>
				const_cast<std::vector<float>&>(points3D_z);
		}
	}

	if (hasRangeImage && m_rangeImage_external_stored)
	{
		for (size_t idx = 0; idx < 1 + rangeImageOtherLayers.size(); idx++)
		{
			std::string layerName;
			mrpt::math::CMatrix_u16* ri = nullptr;
			if (idx == 0)
				ri = const_cast<mrpt::math::CMatrix_u16*>(&rangeImage);
			else
			{
				auto it = rangeImageOtherLayers.begin();
				std::advance(it, idx - 1);
				layerName = it->first;
				ri = const_cast<mrpt::math::CMatrix_u16*>(&it->second);
			}
			const string fil =
				rangeImage_getExternalStorageFileAbsolutePath(layerName);
			if (mrpt::system::strCmpI(
					"txt", mrpt::system::extractFileExtension(fil, true)))
			{
				ri->loadFromTextFile(fil);
			}
			else
			{
				auto& me = const_cast<CObservation3DRangeScan&>(*this);

				mrpt::io::CFileGZInputStream fi(fil);
				auto f = mrpt::serialization::archiveFrom(fi);
				const uint32_t rows = f.ReadAs<uint32_t>();
				const uint32_t cols = f.ReadAs<uint32_t>();
				me.rangeImage_setSize(rows, cols);
				if (ri->size() != 0)
					f.ReadBufferFixEndianness<uint16_t>(ri->data(), ri->size());
			}

		}  // end for each layer
	}
}

void CObservation3DRangeScan::unload()
{
	if (hasPoints3D && m_points3D_external_stored)
	{
		mrpt::vector_strong_clear(points3D_x);
		mrpt::vector_strong_clear(points3D_y);
		mrpt::vector_strong_clear(points3D_z);
	}

	if (hasRangeImage && m_rangeImage_external_stored) rangeImage.setSize(0, 0);

	intensityImage.unload();
	confidenceImage.unload();
}

std::string CObservation3DRangeScan::rangeImage_getExternalStorageFile(
	const std::string& rangeImageLayer) const
{
	std::string filName = m_rangeImage_external_file;
	if (!rangeImageLayer.empty())
	{
		const auto curExt = mrpt::system::extractFileExtension(filName);
		mrpt::system::fileNameChangeExtension(
			filName, std::string("layer_") + rangeImageLayer + curExt);
	}
	return filName;
}

void CObservation3DRangeScan::rangeImage_getExternalStorageFileAbsolutePath(
	std::string& out_path, const std::string& rangeImageLayer) const
{
	std::string filName = rangeImage_getExternalStorageFile(rangeImageLayer);

	ASSERT_(filName.size() > 2);
	if (filName[0] == '/' || (filName[1] == ':' && filName[2] == '\\'))
	{
		out_path = filName;
	}
	else
	{
		out_path = CImage::getImagesPathBase();
		size_t N = CImage::getImagesPathBase().size() - 1;
		if (CImage::getImagesPathBase()[N] != '/' &&
			CImage::getImagesPathBase()[N] != '\\')
			out_path += "/";
		out_path += filName;
	}
}
void CObservation3DRangeScan::points3D_getExternalStorageFileAbsolutePath(
	std::string& out_path) const
{
	ASSERT_(m_points3D_external_file.size() > 2);
	if (m_points3D_external_file[0] == '/' ||
		(m_points3D_external_file[1] == ':' &&
		 m_points3D_external_file[2] == '\\'))
	{
		out_path = m_points3D_external_file;
	}
	else
	{
		out_path = CImage::getImagesPathBase();
		size_t N = CImage::getImagesPathBase().size() - 1;
		if (CImage::getImagesPathBase()[N] != '/' &&
			CImage::getImagesPathBase()[N] != '\\')
			out_path += "/";
		out_path += m_points3D_external_file;
	}
}

void CObservation3DRangeScan::points3D_convertToExternalStorage(
	const std::string& fileName_, const std::string& use_this_base_dir)
{
	ASSERT_(!points3D_isExternallyStored());
	ASSERT_(
		points3D_x.size() == points3D_y.size() &&
		points3D_x.size() == points3D_z.size());

	if (EXTERNALS_AS_TEXT_value)
		m_points3D_external_file =
			mrpt::system::fileNameChangeExtension(fileName_, "txt");
	else
		m_points3D_external_file =
			mrpt::system::fileNameChangeExtension(fileName_, "bin");

	// Use "use_this_base_dir" in "*_getExternalStorageFileAbsolutePath()"
	// instead of CImage::getImagesPathBase()
	const string savedDir = CImage::getImagesPathBase();
	CImage::setImagesPathBase(use_this_base_dir);
	const string real_absolute_path =
		points3D_getExternalStorageFileAbsolutePath();
	CImage::setImagesPathBase(savedDir);

	if (EXTERNALS_AS_TEXT_value)
	{
		const size_t nPts = points3D_x.size();

		CMatrixFloat M(3, nPts);
		M.setRow(0, points3D_x);
		M.setRow(1, points3D_y);
		M.setRow(2, points3D_z);

		M.saveToTextFile(real_absolute_path, MATRIX_FORMAT_FIXED);
	}
	else
	{
		mrpt::io::CFileGZOutputStream fo(real_absolute_path);
		auto f = mrpt::serialization::archiveFrom(fo);
		f << points3D_x << points3D_y << points3D_z;
	}

	m_points3D_external_stored = true;

	// Really dealloc memory, clear() is not enough:
	vector_strong_clear(points3D_x);
	vector_strong_clear(points3D_y);
	vector_strong_clear(points3D_z);
	vector_strong_clear(points3D_idxs_x);
	vector_strong_clear(points3D_idxs_y);
}
void CObservation3DRangeScan::rangeImage_convertToExternalStorage(
	const std::string& fileName_, const std::string& use_this_base_dir)
{
	ASSERT_(!rangeImage_isExternallyStored());
	if (EXTERNALS_AS_TEXT_value)
		m_rangeImage_external_file =
			mrpt::system::fileNameChangeExtension(fileName_, "txt");
	else
		m_rangeImage_external_file =
			mrpt::system::fileNameChangeExtension(fileName_, "bin");

	// Use "use_this_base_dir" in "*_getExternalStorageFileAbsolutePath()"
	// instead of CImage::getImagesPathBase()
	const string savedDir = CImage::getImagesPathBase();
	CImage::setImagesPathBase(use_this_base_dir);

	for (size_t idx = 0; idx < 1 + rangeImageOtherLayers.size(); idx++)
	{
		std::string layerName;
		mrpt::math::CMatrix_u16* ri = nullptr;
		if (idx == 0)
			ri = &rangeImage;
		else
		{
			auto it = rangeImageOtherLayers.begin();
			std::advance(it, idx - 1);
			layerName = it->first;
			ri = &it->second;
		}
		const string real_absolute_path =
			rangeImage_getExternalStorageFileAbsolutePath(layerName);

		if (EXTERNALS_AS_TEXT_value)
		{
			ri->saveToTextFile(real_absolute_path, MATRIX_FORMAT_FIXED);
		}
		else
		{
			mrpt::io::CFileGZOutputStream fo(real_absolute_path);
			auto f = mrpt::serialization::archiveFrom(fo);

			f.WriteAs<uint32_t>(ri->rows());
			f.WriteAs<uint32_t>(ri->cols());
			if (ri->size() != 0)
				f.WriteBufferFixEndianness<uint16_t>(ri->data(), ri->size());
		}
	}

	m_rangeImage_external_stored = true;
	rangeImage.setSize(0, 0);

	CImage::setImagesPathBase(savedDir);
}

// Auxiliary function for "recoverCameraCalibrationParameters"
#define CALIB_DECIMAT 15

namespace mrpt::obs::detail
{
struct TLevMarData
{
	const CObservation3DRangeScan& obs;
	const double z_offset;
	TLevMarData(const CObservation3DRangeScan& obs_, const double z_offset_)
		: obs(obs_), z_offset(z_offset_)
	{
	}
};

static void cam2vec(const TCamera& camPar, CVectorDouble& x)
{
	if (x.size() < 4 + 4) x.resize(4 + 4);

	x[0] = camPar.fx();
	x[1] = camPar.fy();
	x[2] = camPar.cx();
	x[3] = camPar.cy();

	for (size_t i = 0; i < 4; i++) x[4 + i] = camPar.dist[i];
}
static void vec2cam(const CVectorDouble& x, TCamera& camPar)
{
	camPar.intrinsicParams(0, 0) = x[0];  // fx
	camPar.intrinsicParams(1, 1) = x[1];  // fy
	camPar.intrinsicParams(0, 2) = x[2];  // cx
	camPar.intrinsicParams(1, 2) = x[3];  // cy

	for (size_t i = 0; i < 4; i++) camPar.dist[i] = x[4 + i];
}
static void cost_func(
	const CVectorDouble& par, const TLevMarData& d, CVectorDouble& err)
{
	const CObservation3DRangeScan& obs = d.obs;

	TCamera params;
	vec2cam(par, params);

	const size_t nC = obs.rangeImage.cols();
	const size_t nR = obs.rangeImage.rows();

	err = CVectorDouble();  // .resize( nC*nR/square(CALIB_DECIMAT) );

	for (size_t r = 0; r < nR; r += CALIB_DECIMAT)
	{
		for (size_t c = 0; c < nC; c += CALIB_DECIMAT)
		{
			const size_t idx = nC * r + c;

			TPoint3D p(
				obs.points3D_x[idx] + d.z_offset, obs.points3D_y[idx],
				obs.points3D_z[idx]);
			TPoint3D P(-p.y, -p.z, p.x);
			TPixelCoordf pixel;
			{  // mrpt-obs shouldn't depend on mrpt-vision just for this!
				// pinhole::projectPoint_with_distortion(p_wrt_cam,cam,pixel);

				// Pinhole model:
				const double x = P.x / P.z;
				const double y = P.y / P.z;

				// Radial distortion:
				const double r2 = square(x) + square(y);
				const double r4 = square(r2);

				pixel.x =
					params.cx() +
					params.fx() *
						(x * (1 + params.dist[0] * r2 + params.dist[1] * r4 +
							  2 * params.dist[2] * x * y +
							  params.dist[3] * (r2 + 2 * square(x))));
				pixel.y =
					params.cy() +
					params.fy() *
						(y * (1 + params.dist[0] * r2 + params.dist[1] * r4 +
							  2 * params.dist[3] * x * y +
							  params.dist[2] * (r2 + 2 * square(y))));
			}

			// In theory, it should be (r,c):
			err.push_back(c - pixel.x);
			err.push_back(r - pixel.y);
		}
	}
}  // end error_func
}  // namespace mrpt::obs::detail

/** A Levenberg-Marquart-based optimizer to recover the calibration parameters
 * of a 3D camera given a range (depth) image and the corresponding 3D point
 * cloud.
 * \param camera_offset The offset (in meters) in the +X direction of the point
 * cloud. It's 1cm for SwissRanger SR4000.
 * \return The final average reprojection error per pixel (typ <0.05 px)
 */
double CObservation3DRangeScan::recoverCameraCalibrationParameters(
	const CObservation3DRangeScan& obs, mrpt::img::TCamera& out_camParams,
	const double camera_offset)
{
	MRPT_START

	ASSERT_(obs.hasRangeImage && obs.hasPoints3D);
	ASSERT_(
		obs.points3D_x.size() == obs.points3D_y.size() &&
		obs.points3D_x.size() == obs.points3D_z.size());

	using TMyLevMar =
		CLevenbergMarquardtTempl<CVectorDouble, detail::TLevMarData>;
	TMyLevMar::TResultInfo info;

	const size_t nR = obs.rangeImage.rows();
	const size_t nC = obs.rangeImage.cols();

	TCamera camInit;
	camInit.ncols = nC;
	camInit.nrows = nR;
	camInit.intrinsicParams(0, 0) = 250;
	camInit.intrinsicParams(1, 1) = 250;
	camInit.intrinsicParams(0, 2) = nC >> 1;
	camInit.intrinsicParams(1, 2) = nR >> 1;

	CVectorDouble initial_x;
	detail::cam2vec(camInit, initial_x);

	initial_x.resize(8);
	CVectorDouble increments_x(initial_x.size());
	increments_x.fill(1e-4);

	CVectorDouble optimal_x;

	TMyLevMar lm;
	lm.execute(
		optimal_x, initial_x, &mrpt::obs::detail::cost_func, increments_x,
		detail::TLevMarData(obs, camera_offset), info,
		mrpt::system::LVL_INFO, /* verbose */
		1000, /* max iter */
		1e-3, 1e-9, 1e-9, false);

	const double avr_px_err =
		sqrt(info.final_sqr_err / double(nC * nR) / square(CALIB_DECIMAT));

	out_camParams.ncols = nC;
	out_camParams.nrows = nR;
	out_camParams.focalLengthMeters = camera_offset;
	detail::vec2cam(optimal_x, out_camParams);

	return avr_px_err;

	MRPT_END
}

void CObservation3DRangeScan::getZoneAsObs(
	CObservation3DRangeScan& obs, const unsigned int& r1,
	const unsigned int& r2, const unsigned int& c1, const unsigned int& c2)
{
	unsigned int cols = cameraParams.ncols;
	unsigned int rows = cameraParams.nrows;

	ASSERT_((r1 < r2) && (c1 < c2));
	ASSERT_((r2 < rows) && (c2 < cols));
	// Maybe we needed to copy more base obs atributes

	// Copy zone of range image
	obs.hasRangeImage = hasRangeImage;
	if (hasRangeImage)
		obs.rangeImage = rangeImage.asEigen().block(r2 - r1, c2 - c1, r1, c1);

	// Copy zone of intensity image
	obs.hasIntensityImage = hasIntensityImage;
	obs.intensityImageChannel = intensityImageChannel;
	if (hasIntensityImage)
		intensityImage.extract_patch(
			obs.intensityImage, c1, r1, c2 - c1, r2 - r1);

	// Copy zone of confidence image
	obs.hasConfidenceImage = hasConfidenceImage;
	if (hasConfidenceImage)
		confidenceImage.extract_patch(
			obs.confidenceImage, c1, r1, c2 - c1, r2 - r1);

	// Zone labels: It's too complex, just document that pixel labels are NOT
	// extracted.

	// Copy zone of scanned points
	obs.hasPoints3D = hasPoints3D;
	if (hasPoints3D)
	{
		// Erase a possible previous content
		if (obs.points3D_x.size() > 0)
		{
			obs.points3D_x.clear();
			obs.points3D_y.clear();
			obs.points3D_z.clear();
		}

		for (unsigned int i = r1; i < r2; i++)
			for (unsigned int j = c1; j < c2; j++)
			{
				obs.points3D_x.push_back(points3D_x.at(cols * i + j));
				obs.points3D_y.push_back(points3D_y.at(cols * i + j));
				obs.points3D_z.push_back(points3D_z.at(cols * i + j));
			}
	}

	obs.maxRange = maxRange;
	obs.sensorPose = sensorPose;
	obs.stdError = stdError;

	obs.cameraParams = cameraParams;
}

/** Use this method instead of resizing all three \a points3D_x, \a points3D_y &
 * \a points3D_z to allow the usage of the internal memory pool. */
void CObservation3DRangeScan::resizePoints3DVectors(const size_t WH)
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	// If WH=0 this is a clear:
	if (!WH)
	{
		vector_strong_clear(points3D_x);
		vector_strong_clear(points3D_y);
		vector_strong_clear(points3D_z);
		vector_strong_clear(points3D_idxs_x);
		vector_strong_clear(points3D_idxs_y);
		return;
	}

	if (WH <= points3D_x.size())  // reduce size, don't realloc
	{
		points3D_x.resize(WH);
		points3D_y.resize(WH);
		points3D_z.resize(WH);
		points3D_idxs_x.resize(WH);
		points3D_idxs_y.resize(WH);
		return;
	}

	// Request memory for the X,Y,Z buffers from the memory pool:
	TMyPointsMemPool* pool = TMyPointsMemPool::getInstance();
	if (pool)
	{
		CObservation3DRangeScan_Points_MemPoolParams mem_params;
		mem_params.WH = WH;

		CObservation3DRangeScan_Points_MemPoolData* mem_block =
			pool->request_memory(mem_params);

		if (mem_block)
		{  // Take the memory via swaps:
			points3D_x.swap(mem_block->pts_x);
			points3D_y.swap(mem_block->pts_y);
			points3D_z.swap(mem_block->pts_z);
			points3D_idxs_x.swap(mem_block->idxs_x);
			points3D_idxs_y.swap(mem_block->idxs_y);
			delete mem_block;
		}
	}
#endif

	// Either if there was no pool memory or we got it, make sure the size of
	// vectors is OK:
	points3D_x.resize(WH);
	points3D_y.resize(WH);
	points3D_z.resize(WH);
	points3D_idxs_x.resize(WH);
	points3D_idxs_y.resize(WH);
}

size_t CObservation3DRangeScan::getScanSize() const
{
	// x,y,z vectors have the same size.
	return points3D_x.size();
}

// Similar to calling "rangeImage.setSize(H,W)" but this method provides memory
// pooling to speed-up the memory allocation.
void CObservation3DRangeScan::rangeImage_setSize(const int H, const int W)
{
	bool ri_done = rangeImage.cols() == W && rangeImage.rows() == H;

#ifdef COBS3DRANGE_USE_MEMPOOL
	// Request memory from the memory pool:
	TMyRangesMemPool* pool = TMyRangesMemPool::getInstance();
	if (pool && !ri_done)
	{
		CObservation3DRangeScan_Ranges_MemPoolParams mem_params;
		mem_params.H = H;
		mem_params.W = W;

		CObservation3DRangeScan_Ranges_MemPoolData* mem_block =
			pool->request_memory(mem_params);

		if (mem_block)
		{  // Take the memory via swaps:
			rangeImage.swap(mem_block->rangeImage);
			delete mem_block;
			ri_done = true;
		}
	}
// otherwise, continue with the normal method:
#endif
	// Fall-back to normal method:
	if (!ri_done) rangeImage.setSize(H, W);

	// and in all cases, do the resize for the other layers:
	for (auto& layer : rangeImageOtherLayers) layer.second.setSize(H, W);
}

// Return true if \a relativePoseIntensityWRTDepth equals the pure rotation
// (0,0,0,-90deg,0,-90deg) (with a small comparison epsilon)
bool CObservation3DRangeScan::doDepthAndIntensityCamerasCoincide() const
{
	static const double EPSILON = 1e-7;
	static mrpt::poses::CPose3D ref_pose(0, 0, 0, -90.0_deg, 0, -90.0_deg);

	return (relativePoseIntensityWRTDepth.m_coords.array().abs() < EPSILON)
			   .all() &&
		   ((ref_pose.getRotationMatrix() -
			 relativePoseIntensityWRTDepth.getRotationMatrix())
				.array()
				.abs() < EPSILON)
			   .all();
}

void CObservation3DRangeScan::convertTo2DScan(
	mrpt::obs::CObservation2DRangeScan& out_scan2d,
	const T3DPointsTo2DScanParams& sp, const TRangeImageFilterParams& fp)
{
	out_scan2d.sensorLabel = sensorLabel;
	out_scan2d.timestamp = this->timestamp;

	if (!this->hasRangeImage)
	{  // Nothing to do!
		out_scan2d.resizeScan(0);
		return;
	}

	const size_t nCols = this->rangeImage.cols();
	const size_t nRows = this->rangeImage.rows();
	if (fp.rangeMask_min)
	{  // sanity check:
		ASSERT_EQUAL_(fp.rangeMask_min->cols(), rangeImage.cols());
		ASSERT_EQUAL_(fp.rangeMask_min->rows(), rangeImage.rows());
	}
	if (fp.rangeMask_max)
	{  // sanity check:
		ASSERT_EQUAL_(fp.rangeMask_max->cols(), rangeImage.cols());
		ASSERT_EQUAL_(fp.rangeMask_max->rows(), rangeImage.rows());
	}

	// Compute the real horizontal FOV from the range camera intrinsic calib
	// data:
	// Note: this assumes the range image has been "undistorted", which is true
	// for data
	//        from OpenNI, and will be in the future for libfreenect in MRPT,
	//        but it's
	//        not implemented yet (as of Mar 2012), so this is an approximation
	//        in that case.
	const double cx = this->cameraParams.cx();
	const double cy = this->cameraParams.cy();
	const double fx = this->cameraParams.fx();
	const double fy = this->cameraParams.fy();

	// (Imagine the camera seen from above to understand this geometry)
	const double real_FOV_left = atan2(cx, fx);
	const double real_FOV_right = atan2(nCols - 1 - cx, fx);

	// FOV of the equivalent "fake" "laser scanner":
	const float FOV_equiv = 2. * std::max(real_FOV_left, real_FOV_right);

	// Now, we should create more "fake laser" points than columns in the image,
	//  since laser scans are assumed to sample space at evenly-spaced angles,
	//  while in images it is like ~tan(angle).
	ASSERT_ABOVE_(
		sp.oversampling_ratio, (sp.use_origin_sensor_pose ? 0.0 : 1.0));
	const auto nLaserRays = static_cast<size_t>(nCols * sp.oversampling_ratio);

	// Prepare 2D scan data fields:
	out_scan2d.aperture = FOV_equiv;
	out_scan2d.maxRange = this->maxRange;
	out_scan2d.resizeScan(nLaserRays);

	out_scan2d.resizeScanAndAssign(
		nLaserRays, 2.0 * this->maxRange,
		false);  // default: all ranges=invalid
	if (sp.use_origin_sensor_pose)
		out_scan2d.sensorPose = mrpt::poses::CPose3D();
	else
		out_scan2d.sensorPose = this->sensorPose;

	// The vertical FOVs given by the user can be translated into limits of the
	// tangents (tan>0 means above, i.e. z>0):
	const float tan_min = -tan(std::abs(sp.angle_inf));
	const float tan_max = tan(std::abs(sp.angle_sup));

	// Precompute the tangents of the vertical angles of each "ray"
	// for every row in the range image:
	std::vector<float> vert_ang_tan(nRows);
	for (size_t r = 0; r < nRows; r++) vert_ang_tan[r] = d2f((cy - r) / fy);

	if (!sp.use_origin_sensor_pose)
	{
		// Algorithm 1: each column in the range image corresponds to a known
		// orientation in the 2D scan:
		// -------------------
		out_scan2d.rightToLeft = false;

		// Angle "counter" for the fake laser scan direction, and the increment:
		double ang = -FOV_equiv * 0.5;
		const double A_ang = FOV_equiv / (nLaserRays - 1);

		TRangeImageFilter rif(fp);

		// Go thru columns, and keep the minimum distance (along the +X axis,
		// not 3D distance!)
		// for each direction (i.e. for each column) which also lies within the
		// vertical FOV passed
		// by the user.
		for (size_t i = 0; i < nLaserRays; i++, ang += A_ang)
		{
			// Equivalent column in the range image for the "i'th" ray:
			const double tan_ang = tan(ang);
			// make sure we don't go out of range (just in case):
			const size_t c = std::min(
				static_cast<size_t>(std::max(0.0, cx + fx * tan_ang)),
				nCols - 1);

			bool any_valid = false;
			float closest_range = out_scan2d.maxRange;

			for (size_t r = 0; r < nRows; r++)
			{
				const float D = rangeImage.coeff(r, c) * rangeUnits;
				if (!rif.do_range_filter(r, c, D)) continue;

				// All filters passed:
				const float this_point_tan = vert_ang_tan[r] * D;
				if (this_point_tan > tan_min && this_point_tan < tan_max)
				{
					any_valid = true;
					mrpt::keep_min(closest_range, D);
				}
			}

			if (any_valid)
			{
				out_scan2d.setScanRangeValidity(i, true);
				// Compute the distance in 2D from the "depth" in closest_range:
				out_scan2d.setScanRange(
					i, closest_range * std::sqrt(1.0 + tan_ang * tan_ang));
			}
		}  // end for columns
	}
	else
	{
		// Algorithm 2: project to 3D and reproject (for a different sensorPose
		// at the origin)
		// ------------------------------------------------------------------------
		out_scan2d.rightToLeft = true;

		T3DPointsProjectionParams projParams;
		projParams.takeIntoAccountSensorPoseOnRobot = true;

		mrpt::opengl::CPointCloud::Ptr pc = mrpt::opengl::CPointCloud::Create();
		this->unprojectInto(*pc, projParams, fp);

		const std::vector<float>&xs = pc->getArrayX(), &ys = pc->getArrayY(),
			  &zs = pc->getArrayZ();
		const size_t N = xs.size();

		const double A_ang = FOV_equiv / (nLaserRays - 1);
		const double ang0 = -FOV_equiv * 0.5;

		for (size_t i = 0; i < N; i++)
		{
			if (zs[i] < sp.z_min || zs[i] > sp.z_max) continue;

			const double phi_wrt_origin = atan2(ys[i], xs[i]);

			int i_range = (phi_wrt_origin - ang0) / A_ang;
			if (i_range < 0 || i_range >= int(nLaserRays)) continue;

			const float r_wrt_origin = ::hypotf(xs[i], ys[i]);
			if (out_scan2d.getScanRange(i_range) > r_wrt_origin)
				out_scan2d.setScanRange(i_range, r_wrt_origin);
			out_scan2d.setScanRangeValidity(i_range, true);
		}
	}
}

void CObservation3DRangeScan::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);

	this->load();  // Make sure the 3D point cloud, etc... are all loaded in
	// memory.

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot "
		 "base:\n";
	o << sensorPose.getHomogeneousMatrixVal<CMatrixDouble44>() << sensorPose
	  << "\n";

	o << "maxRange = " << maxRange << " [meters]"
	  << "\n";
	o << "rangeUnits = " << rangeUnits << " [meters]"
	  << "\n";

	o << "Has 3D point cloud? ";
	if (hasPoints3D)
	{
		o << "YES: " << points3D_x.size() << " points";
		if (points3D_isExternallyStored())
			o << ". External file: " << points3D_getExternalStorageFile()
			  << "\n";
		else
			o << " (embedded)."
			  << "\n";
	}
	else
		o << "NO"
		  << "\n";

	o << "Range is depth: " << (range_is_depth ? "YES" : "NO") << "\n";
	o << "Has raw range data? " << (hasRangeImage ? "YES" : "NO");
	if (hasRangeImage)
	{
		if (rangeImage_isExternallyStored())
			o << ". External file: " << rangeImage_getExternalStorageFile("");
		else
			o << " (embedded).";
	}
	o << "\n";

	for (auto& layer : rangeImageOtherLayers)
	{
		o << "Additional rangeImage layer: '" << layer.first << "'";
		if (rangeImage_isExternallyStored())
			o << ". External file: " << rangeImage_getExternalStorageFile("");
		else
			o << " (embedded).";
		o << "\n";
	}

	o << "\n"
	  << "Has intensity data? " << (hasIntensityImage ? "YES" : "NO");
	if (hasIntensityImage)
	{
		if (intensityImage.isExternallyStored())
			o << ". External file: " << intensityImage.getExternalStorageFile()
			  << "\n";
		else
			o << " (embedded).\n";
		// Channel?
		o << "Source channel: "
		  << mrpt::typemeta::TEnumType<
				 CObservation3DRangeScan::TIntensityChannelID>::
				 value2name(intensityImageChannel)
		  << "\n";
	}

	o << "\n"
	  << "Has confidence data? " << (hasConfidenceImage ? "YES" : "NO");
	if (hasConfidenceImage)
	{
		if (confidenceImage.isExternallyStored())
			o << ". External file: " << confidenceImage.getExternalStorageFile()
			  << "\n";
		else
			o << " (embedded)."
			  << "\n";
	}

	o << "\n"
	  << "Has pixel labels? " << (hasPixelLabels() ? "YES" : "NO");
	if (hasPixelLabels())
	{
		o << " Human readable labels:"
		  << "\n";
		for (auto it = pixelLabels->pixelLabelNames.begin();
			 it != pixelLabels->pixelLabelNames.end(); ++it)
			o << " label[" << it->first << "]: '" << it->second << "'"
			  << "\n";
	}

	o << "\n"
	  << "\n";
	o << "Depth camera calibration parameters:"
	  << "\n";
	{
		CConfigFileMemory cfg;
		cameraParams.saveToConfigFile("DEPTH_CAM_PARAMS", cfg);
		o << cfg.getContent() << "\n";
	}
	o << "\n"
	  << "Intensity camera calibration parameters:"
	  << "\n";
	{
		CConfigFileMemory cfg;
		cameraParamsIntensity.saveToConfigFile("INTENSITY_CAM_PARAMS", cfg);
		o << cfg.getContent() << "\n";
	}
	o << "\n"
	  << "\n"
	  << "Pose of the intensity cam. wrt the depth cam:\n"
	  << relativePoseIntensityWRTDepth << "\n"
	  << relativePoseIntensityWRTDepth
			 .getHomogeneousMatrixVal<CMatrixDouble44>()
	  << "\n";
}

T3DPointsTo2DScanParams::T3DPointsTo2DScanParams()
	: angle_sup(mrpt::DEG2RAD(5)),
	  angle_inf(mrpt::DEG2RAD(5)),
	  z_min(-std::numeric_limits<double>::max()),
	  z_max(std::numeric_limits<double>::max())
{
}

void CObservation3DRangeScan::undistort()
{
#if MRPT_HAS_OPENCV

	// DEPTH image:
	{
		// OpenCV wrapper (copy-less) for rangeImage:

		const cv::Mat distortion(
			1, cameraParams.dist.size(), CV_64F, &cameraParams.dist[0]);
		const cv::Mat intrinsics(
			3, 3, CV_64F, &cameraParams.intrinsicParams(0, 0));

		const auto imgSize = cv::Size(rangeImage.rows(), rangeImage.cols());

		double alpha = 0;  // all depth pixels are visible in the output
		const cv::Mat newIntrinsics = cv::getOptimalNewCameraMatrix(
			intrinsics, distortion, imgSize, alpha);

		cv::Mat outRangeImg(rangeImage.rows(), rangeImage.cols(), CV_16UC1);

		// Undistort:
		const cv::Mat R_eye = cv::Mat::eye(3, 3, CV_32FC1);

		cv::Mat m1, m2;

		cv::initUndistortRectifyMap(
			intrinsics, distortion, R_eye, newIntrinsics, imgSize, CV_32FC1, m1,
			m2);

		for (size_t idx = 0; idx < 1 + rangeImageOtherLayers.size(); idx++)
		{
			mrpt::math::CMatrix_u16* ri = nullptr;
			if (idx == 0)
				ri = &rangeImage;
			else
			{
				auto it = rangeImageOtherLayers.begin();
				std::advance(it, idx - 1);
				ri = &it->second;
			}
			cv::Mat rangeImg(ri->rows(), ri->cols(), CV_16UC1, ri->data());

			// Remap:
			cv::remap(rangeImg, outRangeImg, m1, m2, cv::INTER_NEAREST);
			// Overwrite:
			outRangeImg.copyTo(rangeImg);
		}

		cameraParams.dist.fill(0);
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				cameraParams.intrinsicParams(r, c) =
					newIntrinsics.at<double>(r, c);
	}

	// RGB image:
	if (hasIntensityImage)
	{
		mrpt::img::CImage newIntImg;
		intensityImage.undistort(newIntImg, cameraParamsIntensity);

		intensityImage = std::move(newIntImg);
		cameraParamsIntensity.dist.fill(0);
	}

#else
	THROW_EXCEPTION("This method requires OpenCV");
#endif
}

mrpt::img::CImage CObservation3DRangeScan::rangeImageAsImage(
	const mrpt::math::CMatrix_u16& ri, float val_min, float val_max,
	float rangeUnits, const std::optional<mrpt::img::TColormap> color)
{
#if MRPT_HAS_OPENCV
	if (val_max < 1e-4f) val_max = ri.maxCoeff() * rangeUnits;

	ASSERT_ABOVE_(val_max, val_min);

	const float range_inv = rangeUnits / (val_max - val_min);

	ASSERT_ABOVE_(ri.cols(), 0);
	ASSERT_ABOVE_(ri.rows(), 0);

	mrpt::img::CImage img;
	const int cols = ri.cols(), rows = ri.rows();

	const auto col = color.value_or(mrpt::img::TColormap::cmGRAYSCALE);

	const bool is_gray = (col == mrpt::img::TColormap::cmGRAYSCALE);

	img.resize(cols, rows, is_gray ? mrpt::img::CH_GRAY : mrpt::img::CH_RGB);

	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			// Normalized value in the range [0,1]:
			const float val_01 = (ri.coeff(r, c) - val_min) * range_inv;
			if (is_gray)
			{
				img.setPixel(c, r, static_cast<uint8_t>(val_01 * 255));
			}
			else
			{
				float R, G, B;
				mrpt::img::colormap(col, val_01, R, G, B);

				img.setPixel(
					c, r,
					mrpt::img::TColor(
						static_cast<uint8_t>(R * 255),
						static_cast<uint8_t>(G * 255),
						static_cast<uint8_t>(B * 255)));
			}
		}
	}

	return img;
#else
	THROW_EXCEPTION("This method requires OpenCV");
#endif
}

mrpt::img::CImage CObservation3DRangeScan::rangeImage_getAsImage(
	const std::optional<mrpt::img::TColormap> color,
	const std::optional<float> normMinRange,
	const std::optional<float> normMaxRange,
	const std::optional<std::string> additionalLayerName) const
{
	ASSERT_(this->hasRangeImage);
	const mrpt::math::CMatrix_u16* ri =
		(!additionalLayerName || additionalLayerName->empty())
			? &rangeImage
			: &rangeImageOtherLayers.at(*additionalLayerName);

	const float val_min = normMinRange.value_or(.0f);
	const float val_max = normMaxRange.value_or(this->maxRange);
	ASSERT_ABOVE_(val_max, val_min);

	return rangeImageAsImage(*ri, val_min, val_max, rangeUnits, color);
}
