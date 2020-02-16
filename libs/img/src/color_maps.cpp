/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

#include <mrpt/img/color_maps.h>
#include <Eigen/Dense>
#include <mrpt/math/interp_fit.hpp>

using namespace mrpt;
using namespace mrpt::img;
using namespace std;

/*-------------------------------------------------------------
					hsv2rgb
-------------------------------------------------------------*/
void mrpt::img::hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
{
	// See: http://en.wikipedia.org/wiki/HSV_color_space
	h = max(0.0f, min(1.0f, h));
	s = max(0.0f, min(1.0f, s));
	v = max(0.0f, min(1.0f, v));

	int Hi = ((int)floor(h * 6)) % 6;
	float f = (h * 6) - Hi;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);

	switch (Hi)
	{
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
			g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
		case 5:
			r = v;
			g = p;
			b = q;
			break;
	}
}

/*-------------------------------------------------------------
					rgb2hsv
-------------------------------------------------------------*/
void mrpt::img::rgb2hsv(float r, float g, float b, float& h, float& s, float& v)
{
	// See: http://en.wikipedia.org/wiki/HSV_color_space
	r = max(0.0f, min(1.0f, r));
	g = max(0.0f, min(1.0f, g));
	b = max(0.0f, min(1.0f, b));

	float Max = max3(r, g, b);
	float Min = min3(r, g, b);

	if (Max == Min)
	{
		h = 0;
	}
	else
	{
		if (Max == r)
		{
			if (g >= b)
				h = (g - b) / (6 * (Max - Min));
			else
				h = 1 - (g - b) / (6 * (Max - Min));
		}
		else if (Max == g)
			h = 1 / 3.0f + (b - r) / (6 * (Max - Min));
		else
			h = 2 / 3.0f + (r - g) / (6 * (Max - Min));
	}

	if (Max == 0)
		s = 0;
	else
		s = 1 - Min / Max;

	v = Max;
}

/*-------------------------------------------------------------
					colormap
-------------------------------------------------------------*/
void mrpt::img::colormap(
	const TColormap& color_map, const float col_indx_in, float& r, float& g,
	float& b)
{
	MRPT_START

	const float color_index = std::min(1.0f, std::max(.0f, col_indx_in));

	switch (color_map)
	{
		case cmJET:
			jet2rgb(color_index, r, g, b);
			break;
		case cmGRAYSCALE:
			r = g = b = color_index;
			break;
		case cmHOT:
			hot2rgb(color_index, r, g, b);
			break;
		default:
			THROW_EXCEPTION("Invalid color_map");
	};
	MRPT_END
}

/*-------------------------------------------------------------
					jet2rgb
-------------------------------------------------------------*/
void mrpt::img::jet2rgb(const float color_index, float& r, float& g, float& b)
{
	static bool jet_table_done = false;
	static Eigen::VectorXf jet_r, jet_g, jet_b;

	// Initialize tables
	if (!jet_table_done)
	{
		jet_table_done = true;

		// Refer to source code of "jet" in MATLAB:
		float JET_R[] = {0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0625f, 0.125f, 0.1875f, 0.250f,  0.3125f, 0.375f,
						 0.4375f, 0.5f,   0.5625f, 0.625f,  0.6875f, 0.750f,
						 0.8125f, 0.875f, 0.9375f, 1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   0.9375f, 0.875f,  0.8125f, 0.750f,
						 0.6875f, 0.625f, 0.5625f, 0.500000};
		float JET_G[] = {0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0625f, 0.125f,  0.1875f, 0.250f,
						 0.3125f, 0.375f, 0.4375f, 0.5f,	0.5625f, 0.625f,
						 0.6875f, 0.750f, 0.8125f, 0.875f,  0.9375f, 1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	0.9375f, 0.875f,
						 0.8125f, 0.750f, 0.6875f, 0.625f,  0.5625f, 0.5f,
						 0.4375f, 0.375f, 0.3125f, 0.250f,  0.1875f, 0.125f,
						 0.0625f, 0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.000000};
		float JET_B[] = {0.5625f, 0.625f, 0.6875f, 0.750f,  0.8125f, 0.875f,
						 0.9375f, 1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 1.0f,	1.0f,   1.0f,	1.0f,	1.0f,	1.0f,
						 0.9375f, 0.875f, 0.8125f, 0.750f,  0.6875f, 0.625f,
						 0.5625f, 0.5f,   0.4375f, 0.375f,  0.3125f, 0.250f,
						 0.1875f, 0.125f, 0.0625f, 0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.0f,	0.0f,	0.0f,
						 0.0f,	0.0f,   0.0f,	0.000000};
		const int N = sizeof(JET_B) / sizeof(JET_B[0]);

		jet_r.resize(N);
		jet_g.resize(N);
		jet_b.resize(N);
		for (int i = 0; i < N; i++)
		{
			jet_r[i] = JET_R[i];
			jet_g[i] = JET_G[i];
			jet_b[i] = JET_B[i];
		}
	}

	// Return interpolate value:
	r = math::interpolate(color_index, jet_r, 0.0f, 1.0f);
	g = math::interpolate(color_index, jet_g, 0.0f, 1.0f);
	b = math::interpolate(color_index, jet_b, 0.0f, 1.0f);
}

void mrpt::img::hot2rgb(const float color_index, float& r, float& g, float& b)
{
	static bool table_done = false;
	static Eigen::VectorXf hot_r, hot_g, hot_b;

	// Initialize tables
	if (!table_done)
	{
		table_done = true;

		// Refer to source code of "hot" in MATLAB:
		float HOT_R[] = {0.041667f, 0.0833f, 0.125f, 0.166667f, 0.2083f, 0.250f,
						 0.291667f, 0.3333f, 0.375f, 0.416667f, 0.4583f, 0.5f,
						 0.541667f, 0.5833f, 0.625f, 0.666667f, 0.7083f, 0.750f,
						 0.791667f, 0.8333f, 0.875f, 0.916667f, 0.9583f, 1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f};
		float HOT_G[] = {0.0f,		0.0f,	0.0f,   0.0f,		0.0f,	0.0f,
						 0.0f,		0.0f,	0.0f,   0.0f,		0.0f,	0.0f,
						 0.0f,		0.0f,	0.0f,   0.0f,		0.0f,	0.0f,
						 0.0f,		0.0f,	0.0f,   0.0f,		0.0f,	0.0f,
						 0.041667f, 0.0833f, 0.125f, 0.166667f, 0.2083f, 0.250f,
						 0.291667f, 0.3333f, 0.375f, 0.416667f, 0.4583f, 0.5f,
						 0.541667f, 0.5833f, 0.625f, 0.666667f, 0.7083f, 0.750f,
						 0.791667f, 0.8333f, 0.875f, 0.916667f, 0.9583f, 1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f,		1.0f,	1.0f,
						 1.0f,		1.0f,	1.0f,   1.0f};
		float HOT_B[] = {
			0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,
			0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,
			0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,
			0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,
			0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,
			0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,   0.0f,	0.0f,
			0.0625f, 0.125f, 0.1875f, 0.250f, 0.3125f, 0.375f, 0.4375f, 0.5f,
			0.5625f, 0.625f, 0.6875f, 0.750f, 0.8125f, 0.875f, 0.9375f, 1.0f};
		const int N = sizeof(HOT_B) / sizeof(HOT_B[0]);

		hot_r.resize(N);
		hot_g.resize(N);
		hot_b.resize(N);
		for (int i = 0; i < N; i++)
		{
			hot_r[i] = HOT_R[i];
			hot_g[i] = HOT_G[i];
			hot_b[i] = HOT_B[i];
		}
	}

	// Return interpolate value:
	r = math::interpolate(color_index, hot_r, 0.0f, 1.0f);
	g = math::interpolate(color_index, hot_g, 0.0f, 1.0f);
	b = math::interpolate(color_index, hot_b, 0.0f, 1.0f);
}
