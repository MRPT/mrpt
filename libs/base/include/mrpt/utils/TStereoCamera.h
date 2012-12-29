/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  TStereoCamera_H
#define  TStereoCamera_H

#include <mrpt/utils/TCamera.h>

namespace mrpt
{
	namespace utils
	{
		using namespace mrpt::math;
		using namespace mrpt::poses;

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
		    CPose3DQuat rightCameraPose;  //!< Pose of the right camera with respect to the coordinate origin of the left camera

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

	} // End of namespace
} // end of namespace
#endif
