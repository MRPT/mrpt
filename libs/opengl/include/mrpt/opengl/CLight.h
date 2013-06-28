/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef opengl_CLight_H
#define opengl_CLight_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/TTypeName.h>
#include <mrpt/opengl/link_pragmas.h>

namespace mrpt
{
	namespace utils { class CStream; }

	namespace opengl
	{
		/** Each of the possible lights of a 3D scene \sa COpenGLViewport 
		  * *IMPORTANT NOTE*: It's the user responsibility to define unique light IDs for each light. 
		  *  The OpenGL standard only assures that valid IDs are 0,1,..7
		  * Refer to standard OpenGL literature and tutorials for the meaning of each field.
		  */
		struct OPENGL_IMPEXP CLight
		{
			CLight(); //!< Default constructor, sets default values

			void setPosition(float x,float y,float z,float w);
			void setDirection(float dx,float dy,float dz);

			uint8_t light_ID; //!< OpenGL ID (typical range: 0-7)

			float	color_ambient[4];
			float	color_diffuse[4];
			float	color_specular[4];

			float	position[4];  //!< [x,y,z,w]: w=0 means directional light, w=1 means a light at a real 3D position.
			float	direction[3]; //!< [x,y,z]
			float	constant_attenuation;
			float	linear_attenuation;
			float	quadratic_attenuation;
			float	spot_exponent;
			float	spot_cutoff;

			void writeToStream(mrpt::utils::CStream &out) const;
			void readFromStream(mrpt::utils::CStream &in);

			void  sendToOpenGL() const; //!< Define the light in the current OpenGL rendering context (users normally don't need to call this explicitly, it's done from within a \sa COpenGLViewport)
		};

		OPENGL_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::opengl::CLight &o);
		OPENGL_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::opengl::CLight &o);


	} // end namespace

	namespace utils 
	{
		template<> struct TTypeName <mrpt::opengl::CLight> { \
			static std::string get() { return std::string("mrpt::opengl::CLight"); } };
	}

} // End of namespace


#endif
