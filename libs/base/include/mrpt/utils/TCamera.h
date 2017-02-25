/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  TCamera_H
#define  TCamera_H

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose3DQuat.h>

namespace mrpt
{
	namespace utils
	{
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

            // This must be added for declaration of MEX-related functions
            DECLARE_MEX_CONVERSION

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
			mrpt::math::CMatrixDouble33 	intrinsicParams;    //!< Matrix of intrinsic parameters (containing the focal length and principal point coordinates)
			mrpt::math::CArrayDouble<5> 	dist;               //!< [k1 k2 t1 t2 k3] -> k_i: parameters of radial distortion, t_i: parameters of tangential distortion (default=0)
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
			/** overload This signature is consistent with the rest of MRPT APIs */
			inline void loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &section) { loadFromConfigFile(section,cfg); }

			/** Dumps all the parameters as a multi-line string, with the same format than \a saveToConfigFile.  \sa saveToConfigFile */
			std::string dumpAsText() const;


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
			inline void getDistortionParamsVector ( mrpt::math::CMatrixDouble15 &distParVector ) const
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
			void setDistortionParamsVector( const mrpt::math::CMatrixDouble15 &distParVector )
			{
				for (size_t i=0;i<5;i++)
					dist[i] = distParVector.get_unsafe(0,i);
			}

			/** Set the whole vector of distortion params of the camera from a 4 or 5-vector */
			template <class VECTORLIKE>
			void setDistortionParamsVector( const VECTORLIKE &distParVector )
			{
				size_t N = static_cast<size_t>(distParVector.size());
				ASSERT_(N==4 || N==5)
				dist[4] = 0; // Default value
				for (size_t i=0;i<N;i++) dist[i] = distParVector[i];
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
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( TCamera, mrpt::utils::CSerializable )


		bool BASE_IMPEXP operator ==(const mrpt::utils::TCamera& a, const mrpt::utils::TCamera& b);
		bool BASE_IMPEXP operator !=(const mrpt::utils::TCamera& a, const mrpt::utils::TCamera& b);

	} // End of namespace
} // end of namespace

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM( mrpt::utils::TCamera ) // Not working at the beginning?
#endif
