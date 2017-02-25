/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

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
CStream & mrpt::utils::operator << (mrpt::utils::CStream& o, const TColor & c) 
{
	o << c.R<<c.G<<c.B<<c.A;
	return o;
}

CStream & mrpt::utils::operator >> (mrpt::utils::CStream& i, TColor & c)
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
CStream & mrpt::utils::operator << (mrpt::utils::CStream& o, const TColorf & c) 
{
	o << c.R<<c.G<<c.B<<c.A;
	return o;
}

CStream & mrpt::utils::operator >> (mrpt::utils::CStream& i, TColorf & c)
{
	i >> c.R>>c.G>>c.B>>c.A;
	return i;
}
