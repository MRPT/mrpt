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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/color_maps.h>
#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;


/*-------------------------------------------------------------
					hsv2rgb
-------------------------------------------------------------*/
void  mrpt::utils::hsv2rgb(
	float	h,
	float	s,
	float	v,
	float	&r,
	float	&g,
	float	&b)
{
	// See: http://en.wikipedia.org/wiki/HSV_color_space
	h = max(0.0f, min(1.0f,h));
	s = max(0.0f, min(1.0f,s));
	v = max(0.0f, min(1.0f,v));

	int		Hi = ((int)floor(h *6)) % 6;
	float	f  = (h*6) - Hi;
	float	p  = v*(1-s);
	float	q  = v*(1-f*s);
	float	t  = v*(1-(1-f)*s);

	switch (Hi)
	{
	case 0:	r=v; g=t;b=p; break;
	case 1:	r=q; g=v;b=p; break;
	case 2:	r=p; g=v;b=t; break;
	case 3:	r=p; g=q;b=v; break;
	case 4:	r=t; g=p;b=v; break;
	case 5:	r=v; g=p;b=q; break;
	}
}

/*-------------------------------------------------------------
					rgb2hsv
-------------------------------------------------------------*/
void  mrpt::utils::rgb2hsv(
	float	r,
	float	g,
	float	b,
	float	&h,
	float	&s,
	float	&v )
{
	// See: http://en.wikipedia.org/wiki/HSV_color_space
	r = max(0.0f, min(1.0f,r));
	g = max(0.0f, min(1.0f,g));
	b = max(0.0f, min(1.0f,b));

	float	Max = max3(r,g,b);
	float	Min = min3(r,g,b);

	if (Max==Min)
	{
		h = 0;
	}
	else
	{
		if (Max==r)
		{
			if (g>=b)
					h = (g-b)/(6*(Max-Min));
			else	h = 1-(g-b)/(6*(Max-Min));
		}
		else
		if (Max==g)
				h = 1/3.0f + (b-r)/(6*(Max-Min));
		else	h = 2/3.0f + (r-g)/(6*(Max-Min));
	}

	if (Max == 0)
			s = 0;
	else	s = 1 - Min/Max;

	v = Max;
}


/*-------------------------------------------------------------
					colormap
-------------------------------------------------------------*/
void mrpt::utils::colormap(
	const TColormap &color_map,
	const float	color_index,
	float	&r,
	float	&g,
	float	&b)
{
	MRPT_START
	switch (color_map)
	{
		case cmJET:
			jet2rgb(color_index,r,g,b);
			break;
		case cmGRAYSCALE:
			r = g = b = color_index;
			break;
		default:
			THROW_EXCEPTION("Invalid color_map");
	};
	MRPT_END
}

/*-------------------------------------------------------------
					jet2rgb
-------------------------------------------------------------*/
void  mrpt::utils::jet2rgb(
	const float	color_index,
	float	&r,
	float	&g,
	float	&b)
{
	static bool	jet_table_done = false;
	static vector_float	jet_r,jet_g,jet_b;


	// Initialize tables
	if (!jet_table_done)
	{
		jet_table_done = true;

		// Refer to source code of "jet" in MATLAB:
		float JET_R[] = { 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.062500,0.125000,0.187500,0.250000,0.312500,0.375000,0.437500,0.500000,0.562500,0.625000,0.687500,0.750000,0.812500,0.875000,0.937500,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,0.937500,0.875000,0.812500,0.750000,0.687500,0.625000,0.562500,0.500000 };
		float JET_G[] = { 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.062500,0.125000,0.187500,0.250000,0.312500,0.375000,0.437500,0.500000,0.562500,0.625000,0.687500,0.750000,0.812500,0.875000,0.937500,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,0.937500,0.875000,0.812500,0.750000,0.687500,0.625000,0.562500,0.500000,0.437500,0.375000,0.312500,0.250000,0.187500,0.125000,0.062500,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000 };
		float JET_B[] = { 0.562500,0.625000,0.687500,0.750000,0.812500,0.875000,0.937500,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,0.937500,0.875000,0.812500,0.750000,0.687500,0.625000,0.562500,0.500000,0.437500,0.375000,0.312500,0.250000,0.187500,0.125000,0.062500,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000 };
		const size_t N = sizeof(JET_B)/sizeof(JET_B[0]);

		jet_r.resize(N);
		jet_g.resize(N);
		jet_b.resize(N);
		for (size_t i=0;i<N;i++)
		{
			jet_r[i] = JET_R[i];
			jet_g[i] = JET_G[i];
			jet_b[i] = JET_B[i];
		}
	}

	// Return interpolate value:
	r = math::interpolate(color_index, jet_r, 0.0f,1.0f);
	g = math::interpolate(color_index, jet_g, 0.0f,1.0f);
	b = math::interpolate(color_index, jet_b, 0.0f,1.0f);
}
