/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  TStereoCamera_H
#define  TStereoCamera_H

#include <mrpt/utils/TCamera.h>

namespace mrpt
{
	namespace utils
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( TStereoCamera, mrpt::utils::CSerializable )

		/** Structure to hold the parameters of a pinhole stereo camera model.
		  *  The parameters obtained for one camera resolution can be used for any other resolution by means of the method TStereoCamera::scaleToResolution()
		  *
		  * \sa mrpt::vision, the application stereo-calib-gui for calibrating a stereo camera
		 */
        class BASE_IMPEXP TStereoCamera : public mrpt::utils::CSerializable
		{
            DEFINE_SERIALIZABLE( TStereoCamera )
        public:
		    TCamera     leftCamera, rightCamera;  //!< Intrinsic and distortion parameters of the left and right cameras
		    mrpt::poses::CPose3DQuat rightCameraPose;  //!< Pose of the right camera with respect to the coordinate origin of the left camera

            // Default constructor:
		    TStereoCamera();

		    /**  Save all params to a plain text config file in this format:
			  *  \code
			  *  [<SECTION>_LEFT]
			  *  resolution = [NCOLS NROWS]
			  *  cx         = CX
			  *  cy         = CY
			  *  fx         = FX
			  *  fy         = FY
			  *  dist       = [K1 K2 T1 T2 K3]
			  *
			  *  [<SECTION>_RIGHT]
			  *  resolution = [NCOLS NROWS]
			  *  cx         = CX
			  *  cy         = CY
			  *  fx         = FX
			  *  fy         = FY
			  *  dist       = [K1 K2 T1 T2 K3]
			  *
			  *  [<SECTION>_LEFT2RIGHT_POSE]
			  *  pose_quaternion = [x y z qr qx qy qz]
			  *
			  *  \endcode
			  *   Notice that 3 different sections are read, of which "section" is only the prefix.
			  */
			void saveToConfigFile( const std::string &section, mrpt::utils::CConfigFileBase &cfg ) const;

			/**  Load all the params from a config source, in the same format that used in saveToConfigFile().
			  *   Notice that 3 different sections are read, of which "section" is only the prefix.
			  *  \exception std::exception on missing fields
			  */
			void loadFromConfigFile(const std::string &section, const mrpt::utils::CConfigFileBase &cfg );
			/** overload This signature is consistent with the rest of MRPT APIs */
			inline void loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &section) { loadFromConfigFile(section,cfg); }

			/** Dumps all the parameters as a multi-line string, with the same format than \a saveToConfigFile.  \sa saveToConfigFile */
			std::string dumpAsText() const;

			/** Rescale all the parameters for a new camera resolution (it raises an exception if the aspect ratio is modified, which is not permitted).
			  */
			void scaleToResolution(unsigned int new_ncols, unsigned int new_nrows)
			{
				leftCamera.scaleToResolution(new_ncols,new_nrows);
				rightCamera.scaleToResolution(new_ncols,new_nrows);
			}

		}; // end class TStereoCamera
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( TStereoCamera, mrpt::utils::CSerializable )

	} // End of namespace
} // end of namespace
#endif
