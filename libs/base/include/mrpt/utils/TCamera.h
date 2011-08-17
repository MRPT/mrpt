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
#ifndef  TCamera_H
#define  TCamera_H

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose3DQuat.h>

namespace mrpt
{
	namespace utils
	{
		using namespace mrpt::math;
		using namespace mrpt::poses;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( TCamera, mrpt::utils::CSerializable )

		/** Structure to hold the parameters of a pinhole camera model.
		  *  The parameters obtained for one camera resolution can be used for any other resolution by means of the method TCamera::scaleToResolution()
		  *
		  * \sa mrpt::vision::CCamModel, the application <a href="http://www.mrpt.org/Application:camera-calib-gui" >camera-calib-gui</a> for calibrating a camera
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP TCamera : public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( TCamera )

		public:
			TCamera() : ncols(640), nrows(480), focalLengthMeters(0)
			{
				intrinsicParams.set_unsafe(0,0,507.808);
				intrinsicParams.set_unsafe(1,1,507.808);
				intrinsicParams.set_unsafe(0,2,356.2368);
				intrinsicParams.set_unsafe(1,2,252.9216);
				intrinsicParams.set_unsafe(2,2,1);
				for (size_t i=0;i<dist.SizeAtCompileTime ;i++)
					dist[i] = 0;
			}

			/** @name Camera parameters
			    @{ */

			uint32_t			ncols,nrows;        //!< Camera resolution
			CMatrixDouble33 	intrinsicParams;    //!< Matrix of intrinsic parameters (containing the focal length and principal point coordinates)
			CArrayDouble<5> 	dist;               //!< [k1 k2 t1 t2 t3] -> k_i: parameters of radial distortion, t_i: parameters of tangential distortion (default=0)
			double  			focalLengthMeters;  //!< The focal length of the camera, in meters (can be used among 'intrinsicParams' to determine the pixel size).

			/** @} */

			/** Rescale all the parameters for a new camera resolution (it raises an exception if the aspect ratio is modified, which is not permitted).
			  */
			void scaleToResolution(unsigned int new_ncols, unsigned int new_nrows);

			/**  Save as a config block:
			  *  \code
			  *  [SECTION]
			  *  resolution = [NCOLS NROWS]
			  *  cx         = CX
			  *  cy         = CY
			  *  fx         = FX
			  *  fy         = FY
			  *  dist       = [K1 K2 T1 T2 K3]
			  *  focal_length = FOCAL_LENGTH
			  *  \endcode
			  */
			void saveToConfigFile( const std::string &section, mrpt::utils::CConfigFileBase &cfg ) const;

			/**  Load all the params from a config source, in the format used in saveToConfigFile(), that is:
			  *
			  *  \code
			  *  [SECTION]
			  *  resolution = [NCOLS NROWS]
			  *  cx         = CX
			  *  cy         = CY
			  *  fx         = FX
			  *  fy         = FY
			  *  dist       = [K1 K2 T1 T2 K3]
			  *  focal_length = FOCAL_LENGTH  [optional field]
			  *  \endcode
			  *  \exception std::exception on missing fields
			  */
			void loadFromConfigFile(const std::string &section, const mrpt::utils::CConfigFileBase &cfg );

			/** Dumps all the parameters as a multi-line string, with the same format than \a saveToConfigFile.  \sa saveToConfigFile */
			std::string dumpAsText()
			{
				mrpt::utils::CConfigFileMemory cfg;
				saveToConfigFile("",cfg);
				return cfg.getContent();
			}


			/** Set the matrix of intrinsic params of the camera from the individual values of focal length and principal point coordinates (in pixels)
			  */
			inline void setIntrinsicParamsFromValues ( double fx, double fy, double cx, double cy )
			{
				intrinsicParams.set_unsafe( 0, 0, fx );
				intrinsicParams.set_unsafe( 1, 1, fy );
				intrinsicParams.set_unsafe( 0, 2, cx );
				intrinsicParams.set_unsafe( 1, 2, cy );
			}

			/** Get the vector of distortion params of the camera  */
			inline void getDistortionParamsVector ( CMatrixDouble15 &distParVector ) const
			{
				for (size_t i=0;i<5;i++)
					distParVector.set_unsafe(0,i, dist[i]);
			}

			/** Get a vector with the distortion params of the camera  */
			inline std::vector<double> getDistortionParamsAsVector () const {
				std::vector<double>  v(5);
				for (size_t i=0;i<5;i++)
					v[i] = dist[i];
				return v;
			}

			/** Set the whole vector of distortion params of the camera */
			void setDistortionParamsVector( const CMatrixDouble15 &distParVector )
			{
				for (size_t i=0;i<5;i++)
					dist[i] = distParVector.get_unsafe(0,i);
			}

			/** Set the whole vector of distortion params of the camera from a 4 or 5-vector */
			template <class VECTORLIKE>
			void setDistortionParamsVector( const VECTORLIKE &distParVector )
			{
				ASSERT_(distParVector.size()==4 || distParVector.size()==5)
				dist[4] = 0; // Default value
				for (typename VECTORLIKE::Index i=0;i<distParVector.size();i++)
					dist[i] = distParVector[i];
			}

			/** Set the vector of distortion params of the camera from the individual values of the distortion coefficients
			  */
			inline void setDistortionParamsFromValues( double k1, double k2, double p1, double p2, double k3 = 0 )
			{
				dist[0] = k1;
				dist[1] = k2;
				dist[2] = p1;
				dist[3] = p2;
				dist[4] = k3;
			}

			/** Get the value of the principal point x-coordinate (in pixels). */
			inline double cx() const { return intrinsicParams(0,2); }
			/** Get the value of the principal point y-coordinate  (in pixels). */
			inline double cy() const { return intrinsicParams(1,2); }
			/** Get the value of the focal length x-value (in pixels). */
			inline double fx() const { return intrinsicParams(0,0); }
			/** Get the value of the focal length y-value (in pixels). */
			inline double fy() const { return intrinsicParams(1,1); }

			/** Set the value of the principal point x-coordinate (in pixels). */
			inline void cx(double val) { intrinsicParams(0,2)=val; }
			/** Set the value of the principal point y-coordinate  (in pixels). */
			inline void cy(double val) { intrinsicParams(1,2)=val; }
			/** Set the value of the focal length x-value (in pixels). */
			inline void fx(double val) { intrinsicParams(0,0)=val; }
			/** Set the value of the focal length y-value (in pixels). */
			inline void fy(double val) { intrinsicParams(1,1)=val; }

			/** Get the value of the k1 distortion parameter.  */
			inline double k1() const { return dist[0]; }
			/** Get the value of the k2 distortion parameter.  */
			inline double k2() const { return dist[1]; }
			/** Get the value of the p1 distortion parameter.  */
			inline double p1() const { return dist[2]; }
			/** Get the value of the p2 distortion parameter.  */
			inline double p2() const { return dist[3]; }
			/** Get the value of the k3 distortion parameter.  */
			inline double k3() const { return dist[4]; }

			/** Get the value of the k1 distortion parameter.  */
			inline void k1(double val) { dist[0]=val; }
			/** Get the value of the k2 distortion parameter.  */
			inline void k2(double val) { dist[1]=val; }
			/** Get the value of the p1 distortion parameter.  */
			inline void p1(double val) { dist[2]=val; }
			/** Get the value of the p2 distortion parameter.  */
			inline void p2(double val) { dist[3]=val; }
			/** Get the value of the k3 distortion parameter.  */
			inline void k3(double val) { dist[4]=val; }
		}; // end class TCamera


		bool BASE_IMPEXP operator ==(const mrpt::utils::TCamera& a, const mrpt::utils::TCamera& b);
		bool BASE_IMPEXP operator !=(const mrpt::utils::TCamera& a, const mrpt::utils::TCamera& b);


		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( TStereoCamera, mrpt::utils::CSerializable )

		enum TStereoCameraModel
		{
		    Bumblebee = 0,
		    Custom,
		    Uncalibrated
		};

		/** Structure to hold the parameters of a pinhole stereo camera model.
		  *  The parameters obtained for one camera resolution can be used for any other resolution by means of the method TCamera::scaleToResolution()
		  *
		  * \sa mrpt::vision::CCamModel, the application <a href="http://www.mrpt.org/Application:camera-calib-gui" >camera-calib-gui</a> for calibrating a camera
		 */
        class BASE_IMPEXP TStereoCamera : public mrpt::utils::CSerializable
		{
            DEFINE_SERIALIZABLE( TStereoCamera )

        public:

		    TStereoCameraModel  model;
		    TCamera             leftCamera, rightCamera;
		    CPose3DQuat         rightCameraPose;

            // Default constructor:
            // Bumblebee with 640x480 images
		    TStereoCamera() : model( Bumblebee )
		    {
		        leftCamera.ncols = rightCamera.ncols = 640;
		        leftCamera.nrows = rightCamera.nrows = 480;

                leftCamera.setIntrinsicParamsFromValues(
                    0.81945957*leftCamera.ncols, 1.09261276*leftCamera.nrows,
                    0.499950781*leftCamera.ncols, 0.506134245*leftCamera.nrows );
                leftCamera.setDistortionParamsFromValues( -3.627383e-001, 2.099672e-001, 0, 0, -8.575903e-002 );

                rightCamera.setIntrinsicParamsFromValues(
                            0.822166309*leftCamera.ncols, 1.096221745*leftCamera.nrows,
                            0.507065918*leftCamera.ncols, 0.524686589*leftCamera.nrows );
                rightCamera.setDistortionParamsFromValues( -3.782850e-001, 2.539438e-001, 0, 0, -1.279638e-001 );

                leftCamera.focalLengthMeters = rightCamera.focalLengthMeters = 0.0038;      // 3.8 mm

                // Camera pose
                CMatrixDouble44 A;
                A.set_unsafe(0,0,9.999777e-001);    A.set_unsafe(0,1,-6.262494e-003);   A.set_unsafe(0,2,2.340592e-003);    A.set_unsafe(0,3,1.227338e-001);
                A.set_unsafe(1,0,6.261120e-003);    A.set_unsafe(1,1,9.999802e-001);    A.set_unsafe(1,2,5.939072e-004);    A.set_unsafe(1,3,-3.671682e-004);
                A.set_unsafe(2,0,-2.344265e-003);   A.set_unsafe(2,1,-5.792392e-004);   A.set_unsafe(2,2,9.999971e-001);    A.set_unsafe(2,3,-1.499571e-004);
                A.set_unsafe(3,0,0);                A.set_unsafe(3,1,0);                A.set_unsafe(3,2,0);                A.set_unsafe(3,3,0);
                rightCameraPose = CPose3DQuat( A );
		    }

		    /**  Save as a config block:
			  *  \code
			  *  [SECTION]
			  *  resolution = [NCOLS NROWS]
			  *  cx         = CX
			  *  cy         = CY
			  *  fx         = FX
			  *  fy         = FY
			  *  dist       = [K1 K2 T1 T2 K3]
			  *  focal_length = FOCAL_LENGTH
			  *  \endcode
			  */
			void saveToConfigFile( const std::string &section, mrpt::utils::CConfigFileBase &cfg ) const;

			/**  Load all the params from a config source, in the format used in saveToConfigFile(), that is:
			  *
			  *  \code
			  *  [SECTION]
			  *  resolution = [NCOLS NROWS]
			  *  cx         = CX
			  *  cy         = CY
			  *  fx         = FX
			  *  fy         = FY
			  *  dist       = [K1 K2 T1 T2 K3]
			  *  focal_length = FOCAL_LENGTH  [optional field]
			  *  \endcode
			  *  \exception std::exception on missing fields
			  */
			void loadFromConfigFile(const std::string &section, const mrpt::utils::CConfigFileBase &cfg );

		}; // end class TStereoCamera

	} // End of namespace
} // end of namespace
#endif
