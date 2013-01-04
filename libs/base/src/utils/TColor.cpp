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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;

// Static colors:
TColor TColor::red		= TColor(255,0,0);
TColor TColor::green	= TColor(0,255,0);
TColor TColor::blue		= TColor(0,0,255);
TColor TColor::black    = TColor(0,0,0);
TColor TColor::white    = TColor(255,255,255);
TColor TColor::gray     = TColor(127,127,127);


// Text streaming:
std::ostream & mrpt::utils::operator << (std::ostream& o, const TColor & c)
{
	char buf[200];
	mrpt::system::os::sprintf(buf,sizeof(buf),"RGBA=[%u,%u,%u,%u]",static_cast<unsigned int>(c.R),static_cast<unsigned int>(c.G),static_cast<unsigned int>(c.B),static_cast<unsigned int>(c.A) );
	o << buf;
	return o;
}

// Binary streaming:
CStream & mrpt::utils::operator << (CStream& o, const TColor & c) 
{
	o << c.R<<c.G<<c.B<<c.A;
	return o;
}

CStream & mrpt::utils::operator >> (CStream& i, TColor & c)
{
	i >> c.R>>c.G>>c.B>>c.A;
	return i;
}


// Text streaming:
std::ostream & mrpt::utils::operator << (std::ostream& o, const TColorf & c)
{
	char buf[200];
	mrpt::system::os::sprintf(buf,sizeof(buf),"RGBAf=[%f,%f,%f,%f]",c.R,c.G,c.B,c.A);
	o << buf;
	return o;
}

// Binary streaming:
CStream & mrpt::utils::operator << (CStream& o, const TColorf & c) 
{
	o << c.R<<c.G<<c.B<<c.A;
	return o;
}

CStream & mrpt::utils::operator >> (CStream& i, TColorf & c)
{
	i >> c.R>>c.G>>c.B>>c.A;
	return i;
}
