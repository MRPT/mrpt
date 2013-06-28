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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/opengl/CLight.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// Ctor:
CLight::CLight() :
	light_ID (0),
	constant_attenuation(1.f),
	linear_attenuation(0.f),
	quadratic_attenuation(0.f),
	spot_exponent(0.f),
	spot_cutoff(180.f)
{
	color_ambient[0] = 0.05f; 
	color_ambient[1] = 0.05f; 
	color_ambient[2] = 0.05f; 
	color_ambient[3] = 1.f; 

	color_diffuse[0] = 0.7f;
	color_diffuse[1] = 0.7f;
	color_diffuse[2] = 0.7f;
	color_diffuse[3] = 1.f;

	color_specular[0] = 0.05f;
	color_specular[1] = 0.05f;
	color_specular[2] = 0.05f;
	color_specular[3] = 1.f;

	setPosition(0,0,1,0);
	setDirection(0,0,-1);
}


void CLight::writeToStream(mrpt::utils::CStream &out) const
{
	const uint8_t version = 0;
	out << version;
	
	out << light_ID
		<< color_ambient[0] << color_ambient[1] << color_ambient[2] << color_ambient[3] 
		<< color_diffuse[0] << color_diffuse[1] << color_diffuse[2] << color_diffuse[3] 
		<< color_specular[0] << color_specular[1] << color_specular[2] << color_specular[3] 
		<< position[0] << position[1] << position[2] << position[3] 
		<< direction[0] << direction[1] << direction[2]
		<< constant_attenuation << linear_attenuation << quadratic_attenuation << spot_exponent << spot_cutoff;
}

void  CLight::readFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;
	
	switch(version)
	{
	case 0:
		in >> light_ID
			>> color_ambient[0] >> color_ambient[1] >> color_ambient[2] >> color_ambient[3] 
			>> color_diffuse[0] >> color_diffuse[1] >> color_diffuse[2] >> color_diffuse[3] 
			>> color_specular[0] >> color_specular[1] >> color_specular[2] >> color_specular[3] 
			>> position[0] >> position[1] >> position[2] >> position[3] 
			>> direction[0] >> direction[1] >> direction[2]
			>> constant_attenuation >> linear_attenuation >> quadratic_attenuation >> spot_exponent >> spot_cutoff;
			break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void  CLight::sendToOpenGL() const
{
#if MRPT_HAS_OPENGL_GLUT
	const GLenum id = (GLenum)((int)GL_LIGHT0 + light_ID);

	glEnable( id );

	glLightfv( id, GL_AMBIENT,               color_ambient );
	glLightfv( id, GL_DIFFUSE,               color_diffuse );
	glLightfv( id, GL_SPECULAR,              color_specular );
	glLightfv( id, GL_POSITION,              position );
	glLightfv( id, GL_SPOT_DIRECTION,        direction );
	glLightf ( id, GL_SPOT_EXPONENT,         spot_exponent );
	glLightf ( id, GL_SPOT_CUTOFF,           spot_cutoff );
	glLightf ( id, GL_CONSTANT_ATTENUATION,  constant_attenuation );
	glLightf ( id, GL_LINEAR_ATTENUATION,    linear_attenuation );
	glLightf ( id, GL_QUADRATIC_ATTENUATION, quadratic_attenuation );
#endif
}

void CLight::setPosition(float x,float y,float z,float w)
{
	position[0] = x;
	position[1] = y;
	position[2] = z;
	position[3] = w;
}
void CLight::setDirection(float dx,float dy,float dz)
{
	direction[0] = dx;
	direction[1] = dy;
	direction[2] = dz;
}



namespace mrpt
{
	namespace opengl
	{
		mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::opengl::CLight &o)
		{
			o.readFromStream(in);
			return in;
		}
		mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::opengl::CLight &o)
		{
			o.writeToStream(out);
			return out;
		}
	}
}
