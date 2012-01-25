/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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
		    TCamera             leftCamera, rightCamera;
		    CPose3DQuat         rightCameraPose;

            // Default constructor:
            // Bumblebee with 640x480 images
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
			std::string dumpAsText()
			{
				mrpt::utils::CConfigFileMemory cfg;
				saveToConfigFile("",cfg);
				return cfg.getContent();
			}

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
