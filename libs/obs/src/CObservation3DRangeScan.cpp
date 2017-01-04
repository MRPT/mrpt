/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/opengl/CPointCloud.h>

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CLevenbergMarquardt.h>
#include <mrpt/math/ops_containers.h> // norm(), etc.
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <limits>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation3DRangeScan, CObservation,mrpt::obs)

// Static LUT:
CObservation3DRangeScan::TCached3DProjTables CObservation3DRangeScan::m_3dproj_lut;

bool CObservation3DRangeScan::EXTERNALS_AS_TEXT = false;


// Whether to use a memory pool for 3D points:
#define COBS3DRANGE_USE_MEMPOOL

// Do performance time logging?
//#define  PROJ3D_PERFLOG


// Data types for memory pooling CObservation3DRangeScan:
#ifdef COBS3DRANGE_USE_MEMPOOL

#	include <mrpt/system/CGenericMemoryPool.h>

	// Memory pool for XYZ points ----------------
	struct CObservation3DRangeScan_Points_MemPoolParams
	{
		size_t WH; //!< Width*Height, that is, the number of 3D points
		inline bool isSuitable(const CObservation3DRangeScan_Points_MemPoolParams &req) const {
			return WH>=req.WH;
		}
	};
	struct CObservation3DRangeScan_Points_MemPoolData
	{
		std::vector<float> pts_x,pts_y,pts_z;
		std::vector<uint16_t> idxs_x, idxs_y; //!< for each point, the corresponding (x,y) pixel coordinates
	};
	typedef mrpt::system::CGenericMemoryPool<CObservation3DRangeScan_Points_MemPoolParams,CObservation3DRangeScan_Points_MemPoolData> TMyPointsMemPool;

	// Memory pool for the rangeImage matrix ----------------
	struct CObservation3DRangeScan_Ranges_MemPoolParams
	{
		int H,W; //!< Size of matrix
		inline bool isSuitable(const CObservation3DRangeScan_Ranges_MemPoolParams &req) const {
			return H==req.H && W==req.W;
		}
	};
	struct CObservation3DRangeScan_Ranges_MemPoolData
	{
		mrpt::math::CMatrix rangeImage;
	};
	typedef mrpt::system::CGenericMemoryPool<CObservation3DRangeScan_Ranges_MemPoolParams,CObservation3DRangeScan_Ranges_MemPoolData> TMyRangesMemPool;

	void mempool_donate_xyz_buffers(CObservation3DRangeScan &obs)
	{
		if (!obs.points3D_x.empty())
		{
			// Before dying, donate my memory to the pool for the joy of future class-brothers...
			TMyPointsMemPool *pool = TMyPointsMemPool::getInstance();
			if (pool)
			{
				CObservation3DRangeScan_Points_MemPoolParams mem_params;
				mem_params.WH = obs.points3D_x.capacity();
				if (obs.points3D_y.capacity()!=mem_params.WH) obs.points3D_y.resize(mem_params.WH);
				if (obs.points3D_z.capacity()!=mem_params.WH) obs.points3D_z.resize(mem_params.WH);
				if (obs.points3D_idxs_x.capacity()!=mem_params.WH) obs.points3D_idxs_x.resize(mem_params.WH);
				if (obs.points3D_idxs_y.capacity()!=mem_params.WH) obs.points3D_idxs_y.resize(mem_params.WH);

				CObservation3DRangeScan_Points_MemPoolData *mem_block = new CObservation3DRangeScan_Points_MemPoolData();
				obs.points3D_x.swap( mem_block->pts_x );
				obs.points3D_y.swap( mem_block->pts_y );
				obs.points3D_z.swap( mem_block->pts_z );
				obs.points3D_idxs_x.swap( mem_block->idxs_x );
				obs.points3D_idxs_y.swap( mem_block->idxs_y );

				pool->dump_to_pool(mem_params, mem_block);
			}
		}
	}
	void mempool_donate_range_matrix(CObservation3DRangeScan &obs)
	{
		if (obs.rangeImage.cols()>1 && obs.rangeImage.rows()>1)
		{
			// Before dying, donate my memory to the pool for the joy of future class-brothers...
			TMyRangesMemPool *pool = TMyRangesMemPool::getInstance();
			if (pool)
			{
				CObservation3DRangeScan_Ranges_MemPoolParams mem_params;
				mem_params.H = obs.rangeImage.rows();
				mem_params.W = obs.rangeImage.cols();

				CObservation3DRangeScan_Ranges_MemPoolData *mem_block = new CObservation3DRangeScan_Ranges_MemPoolData();
				obs.rangeImage.swap( mem_block->rangeImage );

				pool->dump_to_pool(mem_params, mem_block);
			}
		}
	}

#endif

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservation3DRangeScan::CObservation3DRangeScan( ) :
	m_points3D_external_stored(false),
	m_rangeImage_external_stored(false),
	hasPoints3D(false),
	hasRangeImage(false),
	range_is_depth(true),
	hasIntensityImage(false),
	intensityImageChannel(CH_VISIBLE),
	hasConfidenceImage(false),
	pixelLabels(), // Start without label info
	cameraParams(),
	cameraParamsIntensity(),
	relativePoseIntensityWRTDepth(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90)),
	maxRange( 5.0f ),
	sensorPose(),
	stdError( 0.01f )
{
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/

CObservation3DRangeScan::~CObservation3DRangeScan()
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	mempool_donate_xyz_buffers(*this);
	mempool_donate_range_matrix(*this);
#endif
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 8;
	else
	{
		// The data
		out << maxRange << sensorPose;

		out << hasPoints3D;
		if (hasPoints3D)
		{
			ASSERT_(points3D_x.size()==points3D_y.size() && points3D_x.size()==points3D_z.size() && points3D_idxs_x.size()==points3D_x.size() && points3D_idxs_y.size()==points3D_x.size())
			uint32_t N = points3D_x.size();
			out << N;
			if (N)
			{
				out.WriteBufferFixEndianness( &points3D_x[0], N );
				out.WriteBufferFixEndianness( &points3D_y[0], N );
				out.WriteBufferFixEndianness( &points3D_z[0], N );
				out.WriteBufferFixEndianness( &points3D_idxs_x[0], N );  // New in v8
				out.WriteBufferFixEndianness( &points3D_idxs_y[0], N );  // New in v8
			}
		}

		out << hasRangeImage; if (hasRangeImage) out << rangeImage;
		out << hasIntensityImage; if (hasIntensityImage)  out << intensityImage;
		out << hasConfidenceImage; if (hasConfidenceImage) out << confidenceImage;

		out << cameraParams; // New in v2
		out << cameraParamsIntensity; // New in v4
		out << relativePoseIntensityWRTDepth; // New in v4

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
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
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
		{
			uint32_t		N;

			in >> maxRange >> sensorPose;

			if (version>0)
				in >> hasPoints3D;
			else hasPoints3D = true;

			if (hasPoints3D)
			{
				in >> N;
				resizePoints3DVectors(N);

				if (N)
				{
					in.ReadBufferFixEndianness( &points3D_x[0], N);
					in.ReadBufferFixEndianness( &points3D_y[0], N);
					in.ReadBufferFixEndianness( &points3D_z[0], N);

					if (version==0) 
					{
						vector<char> validRange(N);  // for v0.
						in.ReadBuffer( &validRange[0], sizeof(validRange[0])*N );
					}
					if (version>=8) {
						in.ReadBufferFixEndianness( &points3D_idxs_x[0], N);
						in.ReadBufferFixEndianness( &points3D_idxs_y[0], N);
					}
				}
			}
			else
			{
				this->resizePoints3DVectors(0);
			}

			if (version>=1)
			{
				in >> hasRangeImage;
				if (hasRangeImage)
				{
#ifdef COBS3DRANGE_USE_MEMPOOL
					// We should call "rangeImage_setSize()" to exploit the mempool:
					this->rangeImage_setSize(480,640);
#endif
					in >> rangeImage;
				}

				in >> hasIntensityImage;
				if (hasIntensityImage)
					in >>intensityImage;

				in >> hasConfidenceImage;
				if (hasConfidenceImage)
					in >> confidenceImage;

				if (version>=2)
				{
					in >> cameraParams;

					if (version>=4)
					{
						in >> cameraParamsIntensity
						   >> relativePoseIntensityWRTDepth;
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

			if (version>=3)
			{
				// New in v3:
				in >> m_points3D_external_stored >> m_points3D_external_file;
				in >> m_rangeImage_external_stored >> m_rangeImage_external_file;
			}
			else
			{
				m_points3D_external_stored = false;
				m_rangeImage_external_stored = false;
			}

			if (version>=5)
			{
				in >> range_is_depth;
			}
			else
			{
				range_is_depth = true;
			}

			if (version>=6)
			{
				int8_t i;
				in >> i;
				intensityImageChannel = static_cast<TIntensityChannelID>(i);
			}
			else
			{
				intensityImageChannel = CH_VISIBLE;
			}

			pixelLabels.clear_unique(); // Remove existing data first (_unique() is to leave alive any user copies of the shared pointer).
			if (version>=7)
			{

				bool do_have_labels;
				in >> do_have_labels;

				if (do_have_labels)
					pixelLabels = TPixelLabelInfoPtr( TPixelLabelInfoBase::readAndBuildFromStream(in) );
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

void CObservation3DRangeScan::swap(CObservation3DRangeScan &o)
{
	CObservation::swap(o);

	std::swap(hasPoints3D,o.hasPoints3D);
	points3D_x.swap(o.points3D_x);
	points3D_y.swap(o.points3D_y);
	points3D_z.swap(o.points3D_z);
	points3D_idxs_x.swap(o.points3D_idxs_x);
	points3D_idxs_y.swap(o.points3D_idxs_y);
	std::swap(m_points3D_external_stored,o.m_points3D_external_stored);
	std::swap(m_points3D_external_file,o.m_points3D_external_file);

	std::swap(hasRangeImage,o.hasRangeImage);
	rangeImage.swap(o.rangeImage);
	std::swap(m_rangeImage_external_stored, o.m_rangeImage_external_stored);
	std::swap(m_rangeImage_external_file, o.m_rangeImage_external_file);

	std::swap(hasIntensityImage,o.hasIntensityImage);
	std::swap(intensityImageChannel,o.intensityImageChannel);
	intensityImage.swap(o.intensityImage);

	std::swap(hasConfidenceImage,o.hasConfidenceImage);
	confidenceImage.swap(o.confidenceImage);

	std::swap(pixelLabels,o.pixelLabels);

	std::swap(relativePoseIntensityWRTDepth, o.relativePoseIntensityWRTDepth);

	std::swap(cameraParams,o.cameraParams);
	std::swap(cameraParamsIntensity, o.cameraParamsIntensity);

	std::swap(maxRange, o.maxRange);
	std::swap(sensorPose, o.sensorPose);
	std::swap(stdError, o.stdError);

}

void CObservation3DRangeScan::load() const
{
	if (hasPoints3D && m_points3D_external_stored)
	{
		const string fil = points3D_getExternalStorageFileAbsolutePath();
		if (mrpt::system::strCmpI("txt",mrpt::system::extractFileExtension(fil,true)))
		{
			CMatrixFloat M;
			M.loadFromTextFile(fil);
			ASSERT_EQUAL_(M.rows(),3);

			M.extractRow(0,const_cast<std::vector<float>&>(points3D_x));
			M.extractRow(1,const_cast<std::vector<float>&>(points3D_y));
			M.extractRow(2,const_cast<std::vector<float>&>(points3D_z));
		}
		else
		{
			mrpt::utils::CFileGZInputStream f(fil);
			f >> const_cast<std::vector<float>&>(points3D_x) >> const_cast<std::vector<float>&>(points3D_y) >> const_cast<std::vector<float>&>(points3D_z);
		}
	}

	if (hasRangeImage && m_rangeImage_external_stored)
	{
		const string fil = rangeImage_getExternalStorageFileAbsolutePath();
		if (mrpt::system::strCmpI("txt",mrpt::system::extractFileExtension(fil,true)))
		{
			const_cast<CMatrix&>(rangeImage).loadFromTextFile(fil);
		}
		else
		{
			mrpt::utils::CFileGZInputStream f(fil);
			f >> const_cast<CMatrix&>(rangeImage);
		}
	}
}

void CObservation3DRangeScan::unload()
{
	if (hasPoints3D && m_points3D_external_stored)
	{
		vector_strong_clear( points3D_x );
		vector_strong_clear( points3D_y );
		vector_strong_clear( points3D_z );
	}

	if (hasRangeImage && m_rangeImage_external_stored)
		rangeImage.setSize(0,0);

	intensityImage.unload();
	confidenceImage.unload();
}

void CObservation3DRangeScan::rangeImage_getExternalStorageFileAbsolutePath(std::string &out_path) const
{
	ASSERT_(m_rangeImage_external_file.size()>2);
	if (m_rangeImage_external_file[0]=='/' || ( m_rangeImage_external_file[1]==':' && m_rangeImage_external_file[2]=='\\' ) )
	{
		out_path= m_rangeImage_external_file;
	}
	else
	{
		out_path = CImage::IMAGES_PATH_BASE;
		size_t N=CImage::IMAGES_PATH_BASE.size()-1;
		if (CImage::IMAGES_PATH_BASE[N]!='/' && CImage::IMAGES_PATH_BASE[N]!='\\' )
			out_path+= "/";
		out_path+= m_rangeImage_external_file;
	}
}
void CObservation3DRangeScan::points3D_getExternalStorageFileAbsolutePath(std::string &out_path) const
{
	ASSERT_(m_points3D_external_file.size()>2);
	if (m_points3D_external_file[0]=='/' || ( m_points3D_external_file[1]==':' && m_points3D_external_file[2]=='\\' ) )
	{
		out_path= m_points3D_external_file;
	}
	else
	{
		out_path = CImage::IMAGES_PATH_BASE;
		size_t N=CImage::IMAGES_PATH_BASE.size()-1;
		if (CImage::IMAGES_PATH_BASE[N]!='/' && CImage::IMAGES_PATH_BASE[N]!='\\' )
			out_path+= "/";
		out_path+= m_points3D_external_file;
	}
}

void CObservation3DRangeScan::points3D_convertToExternalStorage( const std::string &fileName_, const std::string &use_this_base_dir )
{
	ASSERT_(!points3D_isExternallyStored())
	ASSERT_(points3D_x.size()==points3D_y.size() && points3D_x.size()==points3D_z.size())

	if (EXTERNALS_AS_TEXT)
	     m_points3D_external_file = mrpt::system::fileNameChangeExtension(fileName_,"txt");
	else m_points3D_external_file = mrpt::system::fileNameChangeExtension(fileName_,"bin");

	// Use "use_this_base_dir" in "*_getExternalStorageFileAbsolutePath()" instead of CImage::IMAGES_PATH_BASE
	const string savedDir = CImage::IMAGES_PATH_BASE;
	CImage::IMAGES_PATH_BASE = use_this_base_dir;
	const string real_absolute_file_path = points3D_getExternalStorageFileAbsolutePath();
	CImage::IMAGES_PATH_BASE = savedDir;

	if (EXTERNALS_AS_TEXT)
	{
		const size_t nPts = points3D_x.size();

		CMatrixFloat M(3,nPts);
		M.insertRow(0,points3D_x);
		M.insertRow(1,points3D_y);
		M.insertRow(2,points3D_z);

		M.saveToTextFile(
			real_absolute_file_path,
			MATRIX_FORMAT_FIXED );
	}
	else
	{
		mrpt::utils::CFileGZOutputStream f(real_absolute_file_path);
		f  << points3D_x << points3D_y << points3D_z;
	}

	m_points3D_external_stored = true;

	// Really dealloc memory, clear() is not enough:
	vector_strong_clear(points3D_x);
	vector_strong_clear(points3D_y);
	vector_strong_clear(points3D_z);
	vector_strong_clear(points3D_idxs_x);
	vector_strong_clear(points3D_idxs_y);
}
void CObservation3DRangeScan::rangeImage_convertToExternalStorage( const std::string &fileName_, const std::string &use_this_base_dir )
{
	ASSERT_(!rangeImage_isExternallyStored())
	if (EXTERNALS_AS_TEXT)
	     m_rangeImage_external_file = mrpt::system::fileNameChangeExtension(fileName_,"txt");
	else m_rangeImage_external_file = mrpt::system::fileNameChangeExtension(fileName_,"bin");

	// Use "use_this_base_dir" in "*_getExternalStorageFileAbsolutePath()" instead of CImage::IMAGES_PATH_BASE
	const string savedDir = CImage::IMAGES_PATH_BASE;
	CImage::IMAGES_PATH_BASE = use_this_base_dir;
	const string real_absolute_file_path = rangeImage_getExternalStorageFileAbsolutePath();
	CImage::IMAGES_PATH_BASE = savedDir;

	if (EXTERNALS_AS_TEXT)
	{
		rangeImage.saveToTextFile(
			real_absolute_file_path,
			MATRIX_FORMAT_FIXED );
	}
	else 
	{
		mrpt::utils::CFileGZOutputStream f(real_absolute_file_path);
		f  << rangeImage;
	}

	m_rangeImage_external_stored = true;
	rangeImage.setSize(0,0);
}

// ==============  Auxiliary function for "recoverCameraCalibrationParameters"  =========================

#define CALIB_DECIMAT  15

namespace mrpt
{
	namespace obs
	{
		namespace detail
		{
			struct TLevMarData
			{
				const CObservation3DRangeScan &obs;
				const double z_offset;
				TLevMarData(const CObservation3DRangeScan &obs_, const double z_offset_ ) :
					obs(obs_),z_offset(z_offset_) {}
			};

			void cam2vec(const TCamera &camPar,CVectorDouble &x)
			{
				if (x.size()<4+4) x.resize(4+4);

				x[0] = camPar.fx();
				x[1] = camPar.fy();
				x[2] = camPar.cx();
				x[3] = camPar.cy();

				for (size_t i=0;i<4;i++)
					x[4+i] = camPar.dist[i];
			}
			void vec2cam(const CVectorDouble &x, TCamera &camPar)
			{
				camPar.intrinsicParams(0,0) = x[0]; // fx
				camPar.intrinsicParams(1,1) = x[1]; // fy
				camPar.intrinsicParams(0,2) = x[2]; // cx
				camPar.intrinsicParams(1,2) = x[3]; // cy

				for (size_t i=0;i<4;i++)
					camPar.dist[i] = x[4+i];
			}
			void cost_func(
				const CVectorDouble &par,
				const TLevMarData &d,
				CVectorDouble &err)
			{
				const CObservation3DRangeScan &obs = d.obs;

				TCamera params;
				vec2cam(par,params);

				const size_t nC = obs.rangeImage.getColCount();
				const size_t nR = obs.rangeImage.getRowCount();


				err = CVectorDouble(); // .resize( nC*nR/square(CALIB_DECIMAT) );

				for (size_t r=0;r<nR;r+=CALIB_DECIMAT)
				{
					for (size_t c=0;c<nC;c+=CALIB_DECIMAT)
					{
						const size_t idx = nC*r + c;

						TPoint3D		p( obs.points3D_x[idx]+d.z_offset, obs.points3D_y[idx], obs.points3D_z[idx] );
						TPoint3D		P(-p.y, -p.z , p.x );
						TPixelCoordf	pixel;
						{	// mrpt-obs shouldn't depend on mrpt-vision just for this!
							//pinhole::projectPoint_with_distortion(p_wrt_cam,cam,pixel);

							// Pinhole model:
							const double x = P.x/P.z;
							const double y = P.y/P.z;

							// Radial distortion:
							const double r2 = square(x)+square(y);
							const double r4 = square(r2);

							pixel.x = params.cx() + params.fx() *(  x*(1+params.dist[0]*r2+params.dist[1]*r4+ 2*params.dist[2]*x*y+params.dist[3]*(r2+2*square(x))  )  );
							pixel.y = params.cy() + params.fy() *(  y*(1+params.dist[0]*r2+params.dist[1]*r4+ 2*params.dist[3]*x*y+params.dist[2]*(r2+2*square(y)))  );
						}


						// In theory, it should be (r,c):
						err.push_back( c-pixel.x );
						err.push_back( r-pixel.y );
					}
				}
			} // end error_func
		}
	}
}



/** A Levenberg-Marquart-based optimizer to recover the calibration parameters of a 3D camera given a range (depth) image and the corresponding 3D point cloud.
  * \param camera_offset The offset (in meters) in the +X direction of the point cloud. It's 1cm for SwissRanger SR4000.
  * \return The final average reprojection error per pixel (typ <0.05 px)
  */
double CObservation3DRangeScan::recoverCameraCalibrationParameters(
	const CObservation3DRangeScan	&obs,
	mrpt::utils::TCamera			&out_camParams,
	const double camera_offset)
{
	MRPT_START

	ASSERT_(obs.hasRangeImage && obs.hasPoints3D)
	ASSERT_(obs.points3D_x.size() == obs.points3D_y.size() && obs.points3D_x.size() == obs.points3D_z.size())

	typedef CLevenbergMarquardtTempl<CVectorDouble, detail::TLevMarData > TMyLevMar;
	TMyLevMar::TResultInfo  info;

	const size_t nR = obs.rangeImage.getRowCount();
	const size_t nC = obs.rangeImage.getColCount();

	TCamera camInit;
	camInit.ncols = nC;
	camInit.nrows = nR;
	camInit.intrinsicParams(0,0) = 250;
	camInit.intrinsicParams(1,1) = 250;
	camInit.intrinsicParams(0,2) = nC >> 1;
	camInit.intrinsicParams(1,2) = nR >> 1;

	CVectorDouble initial_x;
	detail::cam2vec(camInit,initial_x);

	initial_x.resize(8);
	CVectorDouble increments_x(initial_x.size()); 
	increments_x.assign(1e-4);

	CVectorDouble optimal_x;

	TMyLevMar lm;
	lm.execute(
		optimal_x,
		initial_x,
		&mrpt::obs::detail::cost_func,
		increments_x,
		detail::TLevMarData(obs,camera_offset),
		info,
		mrpt::utils::LVL_INFO, /* verbose */
		1000 , /* max iter */
		1e-3,
		1e-9,
		1e-9,
		false
		);

	const double avr_px_err = sqrt(info.final_sqr_err/(nC*nR/square(CALIB_DECIMAT)));

	out_camParams.ncols = nC;
	out_camParams.nrows = nR;
	out_camParams.focalLengthMeters = camera_offset;
	detail::vec2cam(optimal_x,out_camParams);

	return avr_px_err;

	MRPT_END
}

void CObservation3DRangeScan::getZoneAsObs(
	CObservation3DRangeScan &obs,
	const unsigned int &r1, const unsigned int &r2,
	const unsigned int &c1, const unsigned int &c2 )
{
	unsigned int cols = cameraParams.ncols;
	unsigned int rows = cameraParams.nrows;

	ASSERT_( (r1<r2) && (c1<c2) )
	ASSERT_( (r2<rows) && (c2<cols) )

	// Maybe we needed to copy more base obs atributes

	// Copy zone of range image
	obs.hasRangeImage = hasRangeImage;
	if ( hasRangeImage )
		rangeImage.extractSubmatrix( r1, r2, c1, c2, obs.rangeImage );

	// Copy zone of intensity image
	obs.hasIntensityImage = hasIntensityImage;
	obs.intensityImageChannel = intensityImageChannel;
	if ( hasIntensityImage )
		intensityImage.extract_patch( obs.intensityImage, c1, r1, c2-c1, r2-r1 );

	// Copy zone of confidence image
	obs.hasConfidenceImage = hasConfidenceImage;
	if ( hasConfidenceImage )
		confidenceImage.extract_patch( obs.confidenceImage, c1, r1, c2-c1, r2-r1 );

	// Zone labels: It's too complex, just document that pixel labels are NOT extracted.

	// Copy zone of scanned points
	obs.hasPoints3D = hasPoints3D;
	if ( hasPoints3D )
	{
		// Erase a possible previous content
		if ( obs.points3D_x.size() > 0 )
		{
			obs.points3D_x.clear();
			obs.points3D_y.clear();
			obs.points3D_z.clear();
		}

		for ( unsigned int i = r1; i < r2; i++ )
			for ( unsigned int j = c1; j < c2; j++ )
			{
				obs.points3D_x.push_back( points3D_x.at( cols*i + j ) );
				obs.points3D_y.push_back( points3D_y.at( cols*i + j ) );
				obs.points3D_z.push_back( points3D_z.at( cols*i + j ) );
			}
	}

	obs.maxRange	= maxRange;
	obs.sensorPose	= sensorPose;
	obs.stdError	= stdError;

	obs.cameraParams = cameraParams;
}


/** Use this method instead of resizing all three \a points3D_x, \a points3D_y & \a points3D_z to allow the usage of the internal memory pool. */
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

	if (WH<=points3D_x.size()) // reduce size, don't realloc
	{
		points3D_x.resize( WH );
		points3D_y.resize( WH );
		points3D_z.resize( WH );
		points3D_idxs_x.resize( WH );
		points3D_idxs_y.resize( WH );
		return;
	}

	// Request memory for the X,Y,Z buffers from the memory pool:
	TMyPointsMemPool *pool = TMyPointsMemPool::getInstance();
	if (pool)
	{
		CObservation3DRangeScan_Points_MemPoolParams mem_params;
		mem_params.WH = WH;

		CObservation3DRangeScan_Points_MemPoolData *mem_block = pool->request_memory(mem_params);

		if (mem_block)
		{	// Take the memory via swaps:
			points3D_x.swap( mem_block->pts_x );
			points3D_y.swap( mem_block->pts_y );
			points3D_z.swap( mem_block->pts_z );
			points3D_idxs_x.swap( mem_block->idxs_x );
			points3D_idxs_y.swap( mem_block->idxs_y );
			delete mem_block;
		}
	}
#endif

	// Either if there was no pool memory or we got it, make sure the size of vectors is OK:
	points3D_x.resize( WH );
	points3D_y.resize( WH );
	points3D_z.resize( WH );
	points3D_idxs_x.resize( WH );
	points3D_idxs_y.resize( WH );
}

// Similar to calling "rangeImage.setSize(H,W)" but this method provides memory pooling to speed-up the memory allocation.
void CObservation3DRangeScan::rangeImage_setSize(const int H, const int W)
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	// Request memory from the memory pool:
	TMyRangesMemPool *pool = TMyRangesMemPool::getInstance();
	if (pool)
	{
		CObservation3DRangeScan_Ranges_MemPoolParams mem_params;
		mem_params.H = H;
		mem_params.W = W;

		CObservation3DRangeScan_Ranges_MemPoolData *mem_block = pool->request_memory(mem_params);

		if (mem_block)
		{	// Take the memory via swaps:
			rangeImage.swap(mem_block->rangeImage);
			delete mem_block;
			return;
		}
	}
	// otherwise, continue with the normal method:
#endif
	// Fall-back to normal method:
	rangeImage.setSize(H,W);
}

// Return true if \a relativePoseIntensityWRTDepth equals the pure rotation (0,0,0,-90deg,0,-90deg) (with a small comparison epsilon)
bool CObservation3DRangeScan::doDepthAndIntensityCamerasCoincide() const
{
	static const double EPSILON=1e-7;
	static mrpt::poses::CPose3D ref_pose(0,0,0,DEG2RAD(-90),0,DEG2RAD(-90));

	return
		(relativePoseIntensityWRTDepth.m_coords.array().abs() < EPSILON ).all() &&
		((ref_pose.getRotationMatrix() - relativePoseIntensityWRTDepth.getRotationMatrix()).array().abs() < EPSILON).all();
}


// Convert into equivalent 2D "fake" laser scan. See .h for doc
void CObservation3DRangeScan::convertTo2DScan(
	CObservation2DRangeScan &out_scan2d,
	const std::string &sensorLabel,
	const double angle_sup,
	const double angle_inf,
	const double oversampling_ratio,
	const mrpt::math::CMatrix * rangeMask_min
	)
{
	T3DPointsTo2DScanParams sp;
	sp.sensorLabel = sensorLabel;
	sp.angle_sup = angle_sup;
	sp.angle_inf = angle_inf;
	sp.oversampling_ratio = oversampling_ratio;
	TRangeImageFilterParams fp;
	fp.rangeMask_min = rangeMask_min;
	convertTo2DScan(out_scan2d, sp,fp);
}

void CObservation3DRangeScan::convertTo2DScan(mrpt::obs::CObservation2DRangeScan & out_scan2d, const T3DPointsTo2DScanParams &sp, const TRangeImageFilterParams &fp )
{
	out_scan2d.sensorLabel = sensorLabel;
	out_scan2d.timestamp = this->timestamp;

	if (!this->hasRangeImage)
	{	// Nothing to do!
		out_scan2d.resizeScan(0);
		return;
	}

	const size_t nCols = this->rangeImage.cols();
	const size_t nRows = this->rangeImage.rows();
	if (fp.rangeMask_min) { // sanity check:
		ASSERT_EQUAL_(fp.rangeMask_min->cols(), rangeImage.cols());
		ASSERT_EQUAL_(fp.rangeMask_min->rows(), rangeImage.rows());
	}
	if (fp.rangeMask_max) { // sanity check:
		ASSERT_EQUAL_(fp.rangeMask_max->cols(), rangeImage.cols());
		ASSERT_EQUAL_(fp.rangeMask_max->rows(), rangeImage.rows());
	}

	// Compute the real horizontal FOV from the range camera intrinsic calib data:
	// Note: this assumes the range image has been "undistorted", which is true for data
	//        from OpenNI, and will be in the future for libfreenect in MRPT, but it's
	//        not implemented yet (as of Mar 2012), so this is an approximation in that case.
	const double cx = this->cameraParams.cx();
	const double cy = this->cameraParams.cy();
	const double fx = this->cameraParams.fx();
	const double fy = this->cameraParams.fy();

	// (Imagine the camera seen from above to understand this geometry)
	const double  real_FOV_left  = atan2(cx, fx);
	const double  real_FOV_right = atan2(nCols-1-cx, fx);

	// FOV of the equivalent "fake" "laser scanner":
	const float  FOV_equiv = 2. * std::max(real_FOV_left,real_FOV_right);

	// Now, we should create more "fake laser" points than columns in the image,
	//  since laser scans are assumed to sample space at evenly-spaced angles,
	//  while in images it is like ~tan(angle).
	ASSERT_ABOVE_(sp.oversampling_ratio, (sp.use_origin_sensor_pose ? 0.0 : 1.0) );
	const size_t nLaserRays = static_cast<size_t>( nCols * sp.oversampling_ratio );

	// Prepare 2D scan data fields:
	out_scan2d.aperture = FOV_equiv;
	out_scan2d.maxRange = this->maxRange;
	out_scan2d.resizeScan(nLaserRays);
		
	out_scan2d.resizeScanAndAssign(nLaserRays, 2.0 * this->maxRange, false ); // default: all ranges=invalid
	if (sp.use_origin_sensor_pose)
	     out_scan2d.sensorPose = mrpt::poses::CPose3D();
	else out_scan2d.sensorPose = this->sensorPose;

	// The vertical FOVs given by the user can be translated into limits of the tangents (tan>0 means above, i.e. z>0):
	const float tan_min = -tan( std::abs(sp.angle_inf) );
	const float tan_max =  tan( std::abs(sp.angle_sup) );

	// Precompute the tangents of the vertical angles of each "ray"
	// for every row in the range image:
	std::vector<float> vert_ang_tan(nRows);
	for (size_t r=0;r<nRows;r++)
		vert_ang_tan[r] = static_cast<float>( (cy-r)/fy );

	if (!sp.use_origin_sensor_pose)
	{
		// Algorithm 1: each column in the range image corresponds to a known orientation in the 2D scan:
		// -------------------
		out_scan2d.rightToLeft = false;

		// Angle "counter" for the fake laser scan direction, and the increment:
		double ang  = -FOV_equiv*0.5;
		const double A_ang = FOV_equiv/(nLaserRays-1);

		TRangeImageFilter rif(fp);

		// Go thru columns, and keep the minimum distance (along the +X axis, not 3D distance!)
		// for each direction (i.e. for each column) which also lies within the vertical FOV passed
		// by the user.
		for (size_t i=0;i<nLaserRays;i++, ang+=A_ang )
		{
			// Equivalent column in the range image for the "i'th" ray:
			const double tan_ang = tan(ang);
			// make sure we don't go out of range (just in case):
			const size_t c = std::min(static_cast<size_t>(std::max(0.0,cx + fx*tan_ang)),nCols-1);

			bool any_valid = false;
			float closest_range = out_scan2d.maxRange;

			for (size_t r=0;r<nRows;r++)
			{
				const float D = this->rangeImage.coeff(r,c);
				if (!rif.do_range_filter(r,c,D))
					continue;

				// All filters passed:
				const float this_point_tan = vert_ang_tan[r] * D;
				if (this_point_tan>tan_min && this_point_tan<tan_max)
				{
					any_valid = true;
					mrpt::utils::keep_min(closest_range, D);
				}
			}

			if (any_valid)
			{
				out_scan2d.setScanRangeValidity(i, true);
				// Compute the distance in 2D from the "depth" in closest_range:
				out_scan2d.setScanRange(i, closest_range*std::sqrt(1.0+tan_ang*tan_ang) );
			}
		} // end for columns
	}
	else
	{
		// Algorithm 2: project to 3D and reproject (for a different sensorPose at the origin)
		// ------------------------------------------------------------------------
		out_scan2d.rightToLeft = true;

		T3DPointsProjectionParams projParams;
		projParams.takeIntoAccountSensorPoseOnRobot = true;
		
		mrpt::opengl::CPointCloudPtr pc = mrpt::opengl::CPointCloud::Create();
		this->project3DPointsFromDepthImageInto(*pc, projParams, fp);

		const std::vector<float> & xs = pc->getArrayX(), &ys = pc->getArrayY(), &zs = pc->getArrayZ();
		const size_t N = xs.size();

		const double A_ang = FOV_equiv/(nLaserRays-1);
		const double ang0  = -FOV_equiv*0.5;

		for (size_t i=0;i<N;i++)
		{
			if (zs[i]<sp.z_min || zs[i]>sp.z_max)
				continue;

			const double phi_wrt_origin = atan2(ys[i], xs[i]);

			int i_range = (phi_wrt_origin-ang0)/A_ang;
			if (i_range<0 || i_range>=int(N))
				continue;

			const float  r_wrt_origin = ::hypotf(xs[i],ys[i]);
			if (out_scan2d.scan[i_range]> r_wrt_origin) out_scan2d.setScanRange(i_range, r_wrt_origin);
			out_scan2d.setScanRangeValidity(i_range, true);
		}

	}
}
	
void CObservation3DRangeScan::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	this->load(); // Make sure the 3D point cloud, etc... are all loaded in memory.

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << sensorPose.getHomogeneousMatrixVal() << sensorPose << endl;

	o << "maxRange = " << maxRange << " m" << endl;

	o << "Has 3D point cloud? ";
	if (hasPoints3D)
	{
		o << "YES: " << points3D_x.size() << " points";
		if (points3D_isExternallyStored())
			o << ". External file: " << points3D_getExternalStorageFile() << endl;
		else o << " (embedded)." << endl;
	}
	else	o << "NO" << endl;

	o << "Has raw range data? " << (hasRangeImage ? "YES": "NO");
	if (hasRangeImage)
	{
		if (rangeImage_isExternallyStored())
				o << ". External file: " << rangeImage_getExternalStorageFile() << endl;
		else o << " (embedded)." << endl;
	}

	o << endl << "Has intensity data? " << (hasIntensityImage ? "YES": "NO");
	if (hasIntensityImage)
	{
		if (intensityImage.isExternallyStored())
			o << ". External file: " << intensityImage.getExternalStorageFile() << endl;
		else o << " (embedded).\n";
		// Channel?
		o << "Source channel: " << mrpt::utils::TEnumType<CObservation3DRangeScan::TIntensityChannelID>::value2name(intensityImageChannel) << endl;
	}

	o << endl << "Has confidence data? " << (hasConfidenceImage ? "YES": "NO");
	if (hasConfidenceImage)
	{
		if (confidenceImage.isExternallyStored())
			o << ". External file: " << confidenceImage.getExternalStorageFile() << endl;
		else o << " (embedded)." << endl;
    }

	o << endl << "Has pixel labels? " << (hasPixelLabels()? "YES": "NO");
	if (hasPixelLabels())
	{
        o << " Human readable labels:" << endl;
		for (TPixelLabelInfoBase::TMapLabelID2Name::const_iterator it=pixelLabels->pixelLabelNames.begin();it!=pixelLabels->pixelLabelNames.end();++it)
			o << " label[" << it->first << "]: '" << it->second << "'" << endl;
	}

	o << endl << endl;
	o << "Depth camera calibration parameters:" << endl;
	{
		CConfigFileMemory cfg;
		cameraParams.saveToConfigFile("DEPTH_CAM_PARAMS",cfg);
		o << cfg.getContent() << endl;
	}
	o << endl << "Intensity camera calibration parameters:" << endl;
	{
		CConfigFileMemory cfg;
		cameraParamsIntensity.saveToConfigFile("INTENSITY_CAM_PARAMS",cfg);
		o << cfg.getContent() << endl;
	}
	o << endl << endl << "Pose of the intensity cam. wrt the depth cam:\n"
		<< relativePoseIntensityWRTDepth << endl
		<< relativePoseIntensityWRTDepth.getHomogeneousMatrixVal() << endl;

}

void CObservation3DRangeScan::TPixelLabelInfoBase::writeToStream(mrpt::utils::CStream &out) const
{
	const uint8_t version = 1; // for possible future changes.
	out << version; 

	// 1st: Save number MAX_NUM_DIFFERENT_LABELS so we can reconstruct the object in the class factory later on.
	out << BITFIELD_BYTES;

	// 2nd: data-specific serialization:
	this->internal_writeToStream(out);
}


// Deserialization and class factory. All in one, ladies and gentlemen
CObservation3DRangeScan::TPixelLabelInfoBase* CObservation3DRangeScan::TPixelLabelInfoBase::readAndBuildFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;

	switch (version)
	{
	case 1: 
		{
			// 1st: Read NUM BYTES
			uint8_t  bitfield_bytes;
			in >> bitfield_bytes;

			// Hand-made class factory. May be a good solution if there will be not too many different classes:
			CObservation3DRangeScan::TPixelLabelInfoBase *new_obj = NULL;
			switch (bitfield_bytes)
			{
			case 1: new_obj = new CObservation3DRangeScan::TPixelLabelInfo<1>(); break;
			case 2: new_obj = new CObservation3DRangeScan::TPixelLabelInfo<2>(); break;
			case 3:
			case 4: new_obj = new CObservation3DRangeScan::TPixelLabelInfo<4>(); break;
			case 5:
			case 6:
			case 7:
			case 8: new_obj = new CObservation3DRangeScan::TPixelLabelInfo<8>(); break;
			default: 
				throw std::runtime_error("Unknown type of pixelLabel inner class while deserializing!");
			};
			// 2nd: data-specific serialization:
			new_obj->internal_readFromStream(in);

			return new_obj;
		}
		break;

	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
		break;
	};

}

T3DPointsTo2DScanParams::T3DPointsTo2DScanParams() : 
	angle_sup(mrpt::utils::DEG2RAD(5)), angle_inf(mrpt::utils::DEG2RAD(5)),
	z_min(-std::numeric_limits<double>::max() ),z_max(std::numeric_limits<double>::max()),
	oversampling_ratio(1.2),use_origin_sensor_pose(false)
{
}
