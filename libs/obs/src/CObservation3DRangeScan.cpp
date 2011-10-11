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

#include <mrpt/obs.h>   // Precompiled headers

#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>

#include <mrpt/math/CLevenbergMarquardt.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTimeLogger.h>

using namespace std;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation3DRangeScan, CObservation,mrpt::slam)

// Static LUT:
CObservation3DRangeScan::TCached3DProjTables CObservation3DRangeScan::m_3dproj_lut;


// Whether external files for 3D points & range are text or binary.
//#define EXTERNALS_AS_TEXT

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
	};
	typedef CGenericMemoryPool<CObservation3DRangeScan_Points_MemPoolParams,CObservation3DRangeScan_Points_MemPoolData> TMyPointsMemPool;

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
		mrpt::utils::CMatrix rangeImage;
	};
	typedef CGenericMemoryPool<CObservation3DRangeScan_Ranges_MemPoolParams,CObservation3DRangeScan_Ranges_MemPoolData> TMyRangesMemPool;

	void mempool_donate_xyz_buffers(CObservation3DRangeScan &obs)
	{
		if (!obs.points3D_x.empty())
		{
			// Before dying, donate my memory to the pool for the joy of future class-brothers...
			TMyPointsMemPool &pool = TMyPointsMemPool::getInstance();

			CObservation3DRangeScan_Points_MemPoolParams mem_params;
			mem_params.WH = obs.points3D_x.size();

			CObservation3DRangeScan_Points_MemPoolData *mem_block = new CObservation3DRangeScan_Points_MemPoolData();
			obs.points3D_x.swap( mem_block->pts_x );
			obs.points3D_y.swap( mem_block->pts_y );
			obs.points3D_z.swap( mem_block->pts_z );

			pool.dump_to_pool(mem_params, mem_block);
		}
	}
	void mempool_donate_range_matrix(CObservation3DRangeScan &obs)
	{
		if (obs.rangeImage.cols()>1 && obs.rangeImage.rows()>1)
		{
			// Before dying, donate my memory to the pool for the joy of future class-brothers...
			TMyRangesMemPool &pool = TMyRangesMemPool::getInstance();

			CObservation3DRangeScan_Ranges_MemPoolParams mem_params;
			mem_params.H = obs.rangeImage.rows();
			mem_params.W = obs.rangeImage.cols();

			CObservation3DRangeScan_Ranges_MemPoolData *mem_block = new CObservation3DRangeScan_Ranges_MemPoolData();
			obs.rangeImage.swap( mem_block->rangeImage );

			pool.dump_to_pool(mem_params, mem_block);
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
void  CObservation3DRangeScan::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 6;
	else
	{
		// The data
		out << maxRange << sensorPose;

		out << hasPoints3D;
		if (hasPoints3D)
		{
			uint32_t N = points3D_x.size();
			out << N;
			if (N)
			{
				out.WriteBufferFixEndianness( &points3D_x[0], N );
				out.WriteBufferFixEndianness( &points3D_y[0], N );
				out.WriteBufferFixEndianness( &points3D_z[0], N );
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
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::readFromStream(CStream &in, int version)
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
#ifdef EXTERNALS_AS_TEXT
		CMatrixFloat M;
		M.loadFromTextFile(fil);

		M.extractRow(0,const_cast<std::vector<float>&>(points3D_x));
		M.extractRow(1,const_cast<std::vector<float>&>(points3D_y));
		M.extractRow(2,const_cast<std::vector<float>&>(points3D_z));
#else
		mrpt::utils::CFileGZInputStream f(fil);
		f >> const_cast<std::vector<float>&>(points3D_x) >> const_cast<std::vector<float>&>(points3D_y) >> const_cast<std::vector<float>&>(points3D_z);
#endif
	}

	if (hasRangeImage && m_rangeImage_external_stored)
	{
		const string fil = rangeImage_getExternalStorageFileAbsolutePath();
#ifdef EXTERNALS_AS_TEXT
		const_cast<CMatrix&>(rangeImage).loadFromTextFile(fil);
#else
		mrpt::utils::CFileGZInputStream f(fil);
		f >> const_cast<CMatrix&>(rangeImage);
#endif
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

void CObservation3DRangeScan::points3D_convertToExternalStorage( const std::string &fileName, const std::string &use_this_base_dir )
{
	ASSERT_(!points3D_isExternallyStored())
	m_points3D_external_file = fileName;

	// Use "use_this_base_dir" in "*_getExternalStorageFileAbsolutePath()" instead of CImage::IMAGES_PATH_BASE
	const string savedDir = CImage::IMAGES_PATH_BASE;
	CImage::IMAGES_PATH_BASE = use_this_base_dir;
	const string real_absolute_file_path = points3D_getExternalStorageFileAbsolutePath();
	CImage::IMAGES_PATH_BASE = savedDir;

	ASSERT_(points3D_x.size()==points3D_y.size() && points3D_x.size()==points3D_z.size())

#ifdef EXTERNALS_AS_TEXT
	const size_t nPts = points3D_x.size();

	CMatrixFloat M(3,nPts);
	M.insertRow(0,points3D_x);
	M.insertRow(1,points3D_y);
	M.insertRow(2,points3D_z);

	M.saveToTextFile(
		real_absolute_file_path,
		MATRIX_FORMAT_FIXED );
#else
	mrpt::utils::CFileGZOutputStream f(real_absolute_file_path);
	f  << points3D_x << points3D_y << points3D_z;
#endif

	m_points3D_external_stored = true;

	// Really dealloc memory, clear() is not enough:
	vector_strong_clear(points3D_x);
	vector_strong_clear(points3D_y);
	vector_strong_clear(points3D_z);
}
void CObservation3DRangeScan::rangeImage_convertToExternalStorage( const std::string &fileName, const std::string &use_this_base_dir )
{
	ASSERT_(!rangeImage_isExternallyStored())
	m_rangeImage_external_file = fileName;

	// Use "use_this_base_dir" in "*_getExternalStorageFileAbsolutePath()" instead of CImage::IMAGES_PATH_BASE
	const string savedDir = CImage::IMAGES_PATH_BASE;
	CImage::IMAGES_PATH_BASE = use_this_base_dir;
	const string real_absolute_file_path = rangeImage_getExternalStorageFileAbsolutePath();
	CImage::IMAGES_PATH_BASE = savedDir;

#ifdef EXTERNALS_AS_TEXT
	rangeImage.saveToTextFile(
		real_absolute_file_path,
		MATRIX_FORMAT_FIXED );
#else
	mrpt::utils::CFileGZOutputStream f(real_absolute_file_path);
	f  << rangeImage;
#endif

	m_rangeImage_external_stored = true;
	rangeImage.setSize(0,0);
}

// ==============  Auxiliary function for "recoverCameraCalibrationParameters"  =========================

#define CALIB_DECIMAT  15

namespace mrpt
{
	namespace slam
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

			void cam2vec(const TCamera &camPar,vector_double &x)
			{
				if (x.size()<4+4) x.resize(4+4);

				x[0] = camPar.fx();
				x[1] = camPar.fy();
				x[2] = camPar.cx();
				x[3] = camPar.cy();

				for (size_t i=0;i<4;i++)
					x[4+i] = camPar.dist[i];
			}
			void vec2cam(const vector_double &x, TCamera &camPar)
			{
				camPar.intrinsicParams(0,0) = x[0]; // fx
				camPar.intrinsicParams(1,1) = x[1]; // fy
				camPar.intrinsicParams(0,2) = x[2]; // cx
				camPar.intrinsicParams(1,2) = x[3]; // cy

				for (size_t i=0;i<4;i++)
					camPar.dist[i] = x[4+i];
			}
			void cost_func(
				const vector_double &par,
				const TLevMarData &d,
				vector_double &err)
			{
				const CObservation3DRangeScan &obs = d.obs;

				TCamera params;
				vec2cam(par,params);

				const size_t nC = obs.rangeImage.getColCount();
				const size_t nR = obs.rangeImage.getRowCount();


				err = vector_double(); // .resize( nC*nR/square(CALIB_DECIMAT) );

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

	typedef CLevenbergMarquardtTempl<vector_double, detail::TLevMarData > TMyLevMar;
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

	vector_double initial_x;
	detail::cam2vec(camInit,initial_x);

	initial_x.resize(8);
	vector_double increments_x(initial_x.size(), 1e-4);

	vector_double optimal_x;

	TMyLevMar::execute(
		optimal_x,
		initial_x,
		&mrpt::slam::detail::cost_func,
		increments_x,
		detail::TLevMarData(obs,camera_offset),
		info,
		false, /* verbose */
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
		return;
	}


	// Request memory for the X,Y,Z buffers from the memory pool:
	TMyPointsMemPool &pool = TMyPointsMemPool::getInstance();

	CObservation3DRangeScan_Points_MemPoolParams mem_params;
	mem_params.WH = WH;

	CObservation3DRangeScan_Points_MemPoolData *mem_block = pool.request_memory(mem_params);

	if (mem_block)
	{	// Take the memory via swaps:
		points3D_x.swap( mem_block->pts_x );
		points3D_y.swap( mem_block->pts_y );
		points3D_z.swap( mem_block->pts_z );
		delete mem_block;
	}
#endif

	// Either if there was no pool memory or we got it, make sure the size of vectors is OK:
	points3D_x.resize( WH );
	points3D_y.resize( WH );
	points3D_z.resize( WH );
}


// Similar to calling "rangeImage.setSize(H,W)" but this method provides memory pooling to speed-up the memory allocation.
void CObservation3DRangeScan::rangeImage_setSize(const int H, const int W)
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	// Request memory from the memory pool:
	TMyRangesMemPool &pool = TMyRangesMemPool::getInstance();

	CObservation3DRangeScan_Ranges_MemPoolParams mem_params;
	mem_params.H = H;
	mem_params.W = W;

	CObservation3DRangeScan_Ranges_MemPoolData *mem_block = pool.request_memory(mem_params);

	if (mem_block)
	{	// Take the memory via swaps:
		rangeImage.swap(mem_block->rangeImage);
		delete mem_block;
		return;
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
		(relativePoseIntensityWRTDepth.m_coords.array() < EPSILON ).all() &&
		((ref_pose.m_ROT - relativePoseIntensityWRTDepth.m_ROT).array().abs() < EPSILON).all();
}
