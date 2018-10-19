/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/fourier.h>
#include <algorithm>
#include <mrpt/core/bits_math.h>
#include <cmath>

using namespace mrpt;
using namespace std;
using namespace mrpt::math;

// Next we declare some auxiliary functions:
namespace mrpt::math
{
// Replaces data[1..2*nn] by its discrete Fourier transform, if isign is input
// as 1; or replaces
// data[1..2*nn] by nn times its inverse discrete Fourier transform, if isign is
// input as -1.
// data is a complex array of length nn or, equivalently, a real array of length
// 2*nn. nn MUST
// be an integer power of 2 (this is not checked for!).
static void four1(float data[], unsigned long nn, int isign)
{
	unsigned long n, mmax, m, j, i;
	double wtemp, wr, wpr, wpi, wi,
		theta;  // Double precision for the trigonometric recurrences.
	float tempr, tempi;

	n = nn << 1;
	j = 1;

	for (i = 1; i < n;
		 i += 2)  // This is the bit-reversal section of the routine.
	{
		if (j > i)
		{
			std::swap(data[j], data[i]);  // Exchange the two complex numbers.
			std::swap(data[j + 1], data[i + 1]);
		}
		m = nn;
		while (m >= 2 && j > m)
		{
			j -= m;
			m >>= 1;
		}
		j += m;
	}
	// Here begins the Danielson-Lanczos section of the routine.
	mmax = 2;
	while (n > mmax)  // Outer loop executed log2 nn times.
	{
		unsigned long istep = mmax << 1;
		theta = isign * (6.28318530717959 /
						 mmax);  // Initialize the trigonometric recurrence.
		wtemp = sin(0.5 * theta);
		wpr = -2.0 * wtemp * wtemp;
		wpi = sin(theta);
		wr = 1.0;
		wi = 0.0;
		for (m = 1; m < mmax; m += 2)  // Here are the two nested inner loops.
		{
			for (i = m; i <= n; i += istep)
			{
				j = i + mmax;  // This is the Danielson-Lanczos formula:
				tempr = (float)(wr * data[j] - wi * data[j + 1]);
				tempi = (float)(wr * data[j + 1] + wi * data[j]);
				data[j] = data[i] - tempr;
				data[j + 1] = data[i + 1] - tempi;
				data[i] += tempr;
				data[i + 1] += tempi;
			}
			wr = (wtemp = wr) * wpr - wi * wpi +
				 wr;  // Trigonometric recurrence.
			wi = wi * wpr + wtemp * wpi + wi;
		}
		mmax = istep;
	}
}

// Calculates the Fourier transform of a set of n real-valued data points.
// Replaces this data (which
// is stored in array data[1..n]) by the positive frequency half of its complex
// Fourier transform.
// The real-valued first and last components of the complex transform are
// returned as elements
// data[1] and data[2], respectively. n must be a power of 2. This routine also
// calculates the
// inverse transform of a complex data array if it is the transform of real
// data. (Result in this case
// must be multiplied by 2/n.)
static void realft(float data[], unsigned long n)
{
	unsigned long i, i1, i2, i3, i4, np3;
	float c1 = 0.5, c2, h1r, h1i, h2r, h2i;
	double wr, wi, wpr, wpi, wtemp,
		theta;  // Double precision for the trigonometric recurrences.
	theta = 3.141592653589793 / (double)(n >> 1);  // Initialize the recurrence.

	c2 = -0.5;
	four1(data, n >> 1, 1);  // The forward transform is here.

	wtemp = sin(0.5 * theta);
	wpr = -2.0 * wtemp * wtemp;
	wpi = sin(theta);
	wr = 1.0 + wpr;
	wi = wpi;
	np3 = n + 3;
	for (i = 2; i <= (n >> 2); i++)  // Case i=1 done separately below.
	{
		i4 = 1 + (i3 = np3 - (i2 = 1 + (i1 = i + i - 1)));
		h1r = c1 * (data[i1] + data[i3]);  // The two separate transforms are
		// separated out of data.
		h1i = c1 * (data[i2] - data[i4]);
		h2r = -c2 * (data[i2] + data[i4]);
		h2i = c2 * (data[i1] - data[i3]);
		data[i1] =
			(float)(h1r + wr * h2r - wi * h2i);  // Here they are recombined to
		// form the true transform of
		// the original real data.
		data[i2] = (float)(h1i + wr * h2i + wi * h2r);
		data[i3] = (float)(h1r - wr * h2r + wi * h2i);
		data[i4] = (float)(-h1i + wr * h2i + wi * h2r);
		wr = (wtemp = wr) * wpr - wi * wpi + wr;  // The recurrence.
		wi = wi * wpr + wtemp * wpi + wi;
	}

	data[1] = (h1r = data[1]) + data[2];
	// Squeeze the first and last data together to get them all within the
	// original array.
	data[2] = h1r - data[2];
}

/**
	Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
	You may use, copy, modify this code for any purpose and
	without fee. You may distribute this ORIGINAL package.
  */
using FFT_TYPE = float;

static void makewt(int nw, int* ip, FFT_TYPE* w);
static void bitrv2(int n, int* ip, FFT_TYPE* a);
static void cftbsub(int n, FFT_TYPE* a, FFT_TYPE* w);
static void cftfsub(int n, FFT_TYPE* a, FFT_TYPE* w);
static void rftfsub(int n, FFT_TYPE* a, int nc, FFT_TYPE* c);
static void rftbsub(int n, FFT_TYPE* a, int nc, FFT_TYPE* c);

static void cftbsub(int n, FFT_TYPE* a, FFT_TYPE* w)
{
	int j, j1, j2, j3, k, k1, ks, l, m;
	FFT_TYPE wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
	FFT_TYPE x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

	l = 2;
	while ((l << 1) < n)
	{
		m = l << 2;
		for (j = 0; j <= l - 2; j += 2)
		{
			j1 = j + l;
			j2 = j1 + l;
			j3 = j2 + l;
			x0r = a[j] + a[j1];
			x0i = a[j + 1] + a[j1 + 1];
			x1r = a[j] - a[j1];
			x1i = a[j + 1] - a[j1 + 1];
			x2r = a[j2] + a[j3];
			x2i = a[j2 + 1] + a[j3 + 1];
			x3r = a[j2] - a[j3];
			x3i = a[j2 + 1] - a[j3 + 1];
			a[j] = x0r + x2r;
			a[j + 1] = x0i + x2i;
			a[j2] = x0r - x2r;
			a[j2 + 1] = x0i - x2i;
			a[j1] = x1r - x3i;
			a[j1 + 1] = x1i + x3r;
			a[j3] = x1r + x3i;
			a[j3 + 1] = x1i - x3r;
		}
		if (m < n)
		{
			wk1r = w[2];
			for (j = m; j <= l + m - 2; j += 2)
			{
				j1 = j + l;
				j2 = j1 + l;
				j3 = j2 + l;
				x0r = a[j] + a[j1];
				x0i = a[j + 1] + a[j1 + 1];
				x1r = a[j] - a[j1];
				x1i = a[j + 1] - a[j1 + 1];
				x2r = a[j2] + a[j3];
				x2i = a[j2 + 1] + a[j3 + 1];
				x3r = a[j2] - a[j3];
				x3i = a[j2 + 1] - a[j3 + 1];
				a[j] = x0r + x2r;
				a[j + 1] = x0i + x2i;
				a[j2] = x2i - x0i;
				a[j2 + 1] = x0r - x2r;
				x0r = x1r - x3i;
				x0i = x1i + x3r;
				a[j1] = wk1r * (x0r - x0i);
				a[j1 + 1] = wk1r * (x0r + x0i);
				x0r = x3i + x1r;
				x0i = x3r - x1i;
				a[j3] = wk1r * (x0i - x0r);
				a[j3 + 1] = wk1r * (x0i + x0r);
			}
			k1 = 1;
			ks = -1;
			for (k = (m << 1); k <= n - m; k += m)
			{
				k1++;
				ks = -ks;
				wk1r = w[k1 << 1];
				wk1i = w[(k1 << 1) + 1];
				wk2r = ks * w[k1];
				wk2i = w[k1 + ks];
				wk3r = wk1r - 2 * wk2i * wk1i;
				wk3i = 2 * wk2i * wk1r - wk1i;
				for (j = k; j <= l + k - 2; j += 2)
				{
					j1 = j + l;
					j2 = j1 + l;
					j3 = j2 + l;
					x0r = a[j] + a[j1];
					x0i = a[j + 1] + a[j1 + 1];
					x1r = a[j] - a[j1];
					x1i = a[j + 1] - a[j1 + 1];
					x2r = a[j2] + a[j3];
					x2i = a[j2 + 1] + a[j3 + 1];
					x3r = a[j2] - a[j3];
					x3i = a[j2 + 1] - a[j3 + 1];
					a[j] = x0r + x2r;
					a[j + 1] = x0i + x2i;
					x0r -= x2r;
					x0i -= x2i;
					a[j2] = wk2r * x0r - wk2i * x0i;
					a[j2 + 1] = wk2r * x0i + wk2i * x0r;
					x0r = x1r - x3i;
					x0i = x1i + x3r;
					a[j1] = wk1r * x0r - wk1i * x0i;
					a[j1 + 1] = wk1r * x0i + wk1i * x0r;
					x0r = x1r + x3i;
					x0i = x1i - x3r;
					a[j3] = wk3r * x0r - wk3i * x0i;
					a[j3 + 1] = wk3r * x0i + wk3i * x0r;
				}
			}
		}
		l = m;
	}
	if (l < n)
	{
		for (j = 0; j <= l - 2; j += 2)
		{
			j1 = j + l;
			x0r = a[j] - a[j1];
			x0i = a[j + 1] - a[j1 + 1];
			a[j] += a[j1];
			a[j + 1] += a[j1 + 1];
			a[j1] = x0r;
			a[j1 + 1] = x0i;
		}
	}
}

static void cftfsub(int n, FFT_TYPE* a, FFT_TYPE* w)
{
	int j, j1, j2, j3, k, k1, ks, l, m;
	FFT_TYPE wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
	FFT_TYPE x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

	l = 2;
	while ((l << 1) < n)
	{
		m = l << 2;
		for (j = 0; j <= l - 2; j += 2)
		{
			j1 = j + l;
			j2 = j1 + l;
			j3 = j2 + l;
			x0r = a[j] + a[j1];
			x0i = a[j + 1] + a[j1 + 1];
			x1r = a[j] - a[j1];
			x1i = a[j + 1] - a[j1 + 1];
			x2r = a[j2] + a[j3];
			x2i = a[j2 + 1] + a[j3 + 1];
			x3r = a[j2] - a[j3];
			x3i = a[j2 + 1] - a[j3 + 1];
			a[j] = x0r + x2r;
			a[j + 1] = x0i + x2i;
			a[j2] = x0r - x2r;
			a[j2 + 1] = x0i - x2i;
			a[j1] = x1r + x3i;
			a[j1 + 1] = x1i - x3r;
			a[j3] = x1r - x3i;
			a[j3 + 1] = x1i + x3r;
		}
		if (m < n)
		{
			wk1r = w[2];
			for (j = m; j <= l + m - 2; j += 2)
			{
				j1 = j + l;
				j2 = j1 + l;
				j3 = j2 + l;
				x0r = a[j] + a[j1];
				x0i = a[j + 1] + a[j1 + 1];
				x1r = a[j] - a[j1];
				x1i = a[j + 1] - a[j1 + 1];
				x2r = a[j2] + a[j3];
				x2i = a[j2 + 1] + a[j3 + 1];
				x3r = a[j2] - a[j3];
				x3i = a[j2 + 1] - a[j3 + 1];
				a[j] = x0r + x2r;
				a[j + 1] = x0i + x2i;
				a[j2] = x0i - x2i;
				a[j2 + 1] = x2r - x0r;
				x0r = x1r + x3i;
				x0i = x1i - x3r;
				a[j1] = wk1r * (x0i + x0r);
				a[j1 + 1] = wk1r * (x0i - x0r);
				x0r = x3i - x1r;
				x0i = x3r + x1i;
				a[j3] = wk1r * (x0r + x0i);
				a[j3 + 1] = wk1r * (x0r - x0i);
			}
			k1 = 1;
			ks = -1;
			for (k = (m << 1); k <= n - m; k += m)
			{
				k1++;
				ks = -ks;
				wk1r = w[k1 << 1];
				wk1i = w[(k1 << 1) + 1];
				wk2r = ks * w[k1];
				wk2i = w[k1 + ks];
				wk3r = wk1r - 2 * wk2i * wk1i;
				wk3i = 2 * wk2i * wk1r - wk1i;
				for (j = k; j <= l + k - 2; j += 2)
				{
					j1 = j + l;
					j2 = j1 + l;
					j3 = j2 + l;
					x0r = a[j] + a[j1];
					x0i = a[j + 1] + a[j1 + 1];
					x1r = a[j] - a[j1];
					x1i = a[j + 1] - a[j1 + 1];
					x2r = a[j2] + a[j3];
					x2i = a[j2 + 1] + a[j3 + 1];
					x3r = a[j2] - a[j3];
					x3i = a[j2 + 1] - a[j3 + 1];
					a[j] = x0r + x2r;
					a[j + 1] = x0i + x2i;
					x0r -= x2r;
					x0i -= x2i;
					a[j2] = wk2r * x0r + wk2i * x0i;
					a[j2 + 1] = wk2r * x0i - wk2i * x0r;
					x0r = x1r + x3i;
					x0i = x1i - x3r;
					a[j1] = wk1r * x0r + wk1i * x0i;
					a[j1 + 1] = wk1r * x0i - wk1i * x0r;
					x0r = x1r - x3i;
					x0i = x1i + x3r;
					a[j3] = wk3r * x0r + wk3i * x0i;
					a[j3 + 1] = wk3r * x0i - wk3i * x0r;
				}
			}
		}
		l = m;
	}
	if (l < n)
	{
		for (j = 0; j <= l - 2; j += 2)
		{
			j1 = j + l;
			x0r = a[j] - a[j1];
			x0i = a[j + 1] - a[j1 + 1];
			a[j] += a[j1];
			a[j + 1] += a[j1 + 1];
			a[j1] = x0r;
			a[j1 + 1] = x0i;
		}
	}
}

static void makewt(int nw, int* ip, FFT_TYPE* w)
{
	void bitrv2(int n, int* ip, FFT_TYPE* a);
	int nwh, j;
	FFT_TYPE delta, x, y;

	ip[0] = nw;
	ip[1] = 1;
	if (nw > 2)
	{
		nwh = nw >> 1;
		delta = atan(1.0f) / nwh;
		w[0] = 1;
		w[1] = 0;
		w[nwh] = cos(delta * nwh);
		w[nwh + 1] = w[nwh];
		for (j = 2; j <= nwh - 2; j += 2)
		{
			x = cos(delta * j);
			y = sin(delta * j);
			w[j] = x;
			w[j + 1] = y;
			w[nw - j] = y;
			w[nw - j + 1] = x;
		}
		bitrv2(nw, ip + 2, w);
	}
}

/**
	Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
	You may use, copy, modify this code for any purpose and
	without fee. You may distribute this ORIGINAL package.
  */
static void makect(int nc, int* ip, FFT_TYPE* c)
{
	int nch, j;
	FFT_TYPE delta;

	ip[1] = nc;
	if (nc > 1)
	{
		nch = nc >> 1;
		delta = atan(1.0f) / nch;
		c[0] = 0.5f;
		c[nch] = 0.5f * cos(delta * nch);
		for (j = 1; j <= nch - 1; j++)
		{
			c[j] = 0.5f * cos(delta * j);
			c[nc - j] = 0.5f * sin(delta * j);
		}
	}
}

/**
	Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
	You may use, copy, modify this code for any purpose and
	without fee. You may distribute this ORIGINAL package.
  */
static void bitrv2(int n, int* ip, FFT_TYPE* a)
{
	int j, j1, k, k1, l, m, m2;
	FFT_TYPE xr, xi;

	ip[0] = 0;
	l = n;
	m = 1;
	while ((m << 2) < l)
	{
		l >>= 1;
		for (j = 0; j <= m - 1; j++)
		{
			ip[m + j] = ip[j] + l;
		}
		m <<= 1;
	}
	if ((m << 2) > l)
	{
		for (k = 1; k <= m - 1; k++)
		{
			for (j = 0; j <= k - 1; j++)
			{
				j1 = (j << 1) + ip[k];
				k1 = (k << 1) + ip[j];
				xr = a[j1];
				xi = a[j1 + 1];
				a[j1] = a[k1];
				a[j1 + 1] = a[k1 + 1];
				a[k1] = xr;
				a[k1 + 1] = xi;
			}
		}
	}
	else
	{
		m2 = m << 1;
		for (k = 1; k <= m - 1; k++)
		{
			for (j = 0; j <= k - 1; j++)
			{
				j1 = (j << 1) + ip[k];
				k1 = (k << 1) + ip[j];
				xr = a[j1];
				xi = a[j1 + 1];
				a[j1] = a[k1];
				a[j1 + 1] = a[k1 + 1];
				a[k1] = xr;
				a[k1 + 1] = xi;
				j1 += m2;
				k1 += m2;
				xr = a[j1];
				xi = a[j1 + 1];
				a[j1] = a[k1];
				a[j1 + 1] = a[k1 + 1];
				a[k1] = xr;
				a[k1 + 1] = xi;
			}
		}
	}
}

/**
	Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
	You may use, copy, modify this code for any purpose and
	without fee. You may distribute this ORIGINAL package.
  */
static void cdft(int n, int isgn, FFT_TYPE* a, int* ip, FFT_TYPE* w)
{
	if (n > (ip[0] << 2))
	{
		makewt(n >> 2, ip, w);
	}
	if (n > 4)
	{
		bitrv2(n, ip + 2, a);
	}
	if (isgn < 0)
	{
		cftfsub(n, a, w);
	}
	else
	{
		cftbsub(n, a, w);
	}
}

static void rftfsub(int n, FFT_TYPE* a, int nc, FFT_TYPE* c)
{
	int j, k, kk, ks;
	FFT_TYPE wkr, wki, xr, xi, yr, yi;

	ks = (nc << 2) / n;
	kk = 0;
	for (k = (n >> 1) - 2; k >= 2; k -= 2)
	{
		j = n - k;
		kk += ks;
		wkr = 0.5f - c[kk];
		wki = c[nc - kk];
		xr = a[k] - a[j];
		xi = a[k + 1] + a[j + 1];
		yr = wkr * xr + wki * xi;
		yi = wkr * xi - wki * xr;
		a[k] -= yr;
		a[k + 1] -= yi;
		a[j] += yr;
		a[j + 1] -= yi;
	}
}

static void rdft(int n, int isgn, FFT_TYPE* a, int* ip, FFT_TYPE* w)
{
	int nw, nc;
	FFT_TYPE xi;

	nw = ip[0];
	if (n > (nw << 2))
	{
		nw = n >> 2;
		makewt(nw, ip, w);
	}
	nc = ip[1];
	if (n > (nc << 2))
	{
		nc = n >> 2;
		makect(nc, ip, w + nw);
	}
	if (isgn < 0)
	{
		a[1] = 0.5f * (a[0] - a[1]);
		a[0] -= a[1];
		if (n > 4)
		{
			rftfsub(n, a, nc, w + nw);
			bitrv2(n, ip + 2, a);
		}
		cftfsub(n, a, w);
	}
	else
	{
		if (n > 4)
		{
			bitrv2(n, ip + 2, a);
		}
		cftbsub(n, a, w);
		if (n > 4)
		{
			rftbsub(n, a, nc, w + nw);
		}
		xi = a[0] - a[1];
		a[0] += a[1];
		a[1] = xi;
	}
}

static void rftbsub(int n, FFT_TYPE* a, int nc, FFT_TYPE* c)
{
	int j, k, kk, ks;
	FFT_TYPE wkr, wki, xr, xi, yr, yi;

	ks = (nc << 2) / n;
	kk = 0;
	for (k = (n >> 1) - 2; k >= 2; k -= 2)
	{
		j = n - k;
		kk += ks;
		wkr = 0.5f - c[kk];
		wki = c[nc - kk];
		xr = a[k] - a[j];
		xi = a[k + 1] + a[j + 1];
		yr = wkr * xr - wki * xi;
		yi = wkr * xi + wki * xr;
		a[k] -= yr;
		a[k + 1] -= yi;
		a[j] += yr;
		a[j + 1] -= yi;
	}
}

/**
	Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
	You may use, copy, modify this code for any purpose and
	without fee. You may distribute this ORIGINAL package.

-------- Real DFT / Inverse of Real DFT --------
	[definition]
		<case1> RDFT
			R[k1][k2] = sum_j1=0^n1-1 sum_j2=0^n2-1 a[j1][j2] *
							cos(2*pi*j1*k1/n1 + 2*pi*j2*k2/n2),
							0<=k1<n1, 0<=k2<n2
			I[k1][k2] = sum_j1=0^n1-1 sum_j2=0^n2-1 a[j1][j2] *
							sin(2*pi*j1*k1/n1 + 2*pi*j2*k2/n2),
							0<=k1<n1, 0<=k2<n2
		<case2> IRDFT (excluding scale)
			a[k1][k2] = (1/2) * sum_j1=0^n1-1 sum_j2=0^n2-1
							(R[j1][j2] *
							cos(2*pi*j1*k1/n1 + 2*pi*j2*k2/n2) +
							I[j1][j2] *
							sin(2*pi*j1*k1/n1 + 2*pi*j2*k2/n2)),
							0<=k1<n1, 0<=k2<n2
		(notes: R[n1-k1][n2-k2] = R[k1][k2],
				I[n1-k1][n2-k2] = -I[k1][k2],
				R[n1-k1][0] = R[k1][0],
				I[n1-k1][0] = -I[k1][0],
				R[0][n2-k2] = R[0][k2],
				I[0][n2-k2] = -I[0][k2],
				0<k1<n1, 0<k2<n2)
	[usage]
		<case1>
			ip[0] = 0; // first time only
			rdft2d(n1, n2, 1, a, t, ip, w);
		<case2>
			ip[0] = 0; // first time only
			rdft2d(n1, n2, -1, a, t, ip, w);
	[parameters]
		n1     :data length (int)
				n1 >= 2, n1 = power of 2
		n2     :data length (int)
				n2 >= 2, n2 = power of 2
		a[0...n1-1][0...n2-1]
			   :input/output data (FFT_TYPE **)
				<case1>
					output data
						a[k1][2*k2] = R[k1][k2] = R[n1-k1][n2-k2],
						a[k1][2*k2+1] = I[k1][k2] = -I[n1-k1][n2-k2],
							0<k1<n1, 0<k2<n2/2,
						a[0][2*k2] = R[0][k2] = R[0][n2-k2],
						a[0][2*k2+1] = I[0][k2] = -I[0][n2-k2],
							0<k2<n2/2,
						a[k1][0] = R[k1][0] = R[n1-k1][0],
						a[k1][1] = I[k1][0] = -I[n1-k1][0],
						a[n1-k1][1] = R[k1][n2/2] = R[n1-k1][n2/2],
						a[n1-k1][0] = -I[k1][n2/2] = I[n1-k1][n2/2],
							0<k1<n1/2,
						a[0][0] = R[0][0],
						a[0][1] = R[0][n2/2],
						a[n1/2][0] = R[n1/2][0],
						a[n1/2][1] = R[n1/2][n2/2]
				<case2>
					input data
						a[j1][2*j2] = R[j1][j2] = R[n1-j1][n2-j2],
						a[j1][2*j2+1] = I[j1][j2] = -I[n1-j1][n2-j2],
							0<j1<n1, 0<j2<n2/2,
						a[0][2*j2] = R[0][j2] = R[0][n2-j2],
						a[0][2*j2+1] = I[0][j2] = -I[0][n2-j2],
							0<j2<n2/2,
						a[j1][0] = R[j1][0] = R[n1-j1][0],
						a[j1][1] = I[j1][0] = -I[n1-j1][0],
						a[n1-j1][1] = R[j1][n2/2] = R[n1-j1][n2/2],
						a[n1-j1][0] = -I[j1][n2/2] = I[n1-j1][n2/2],
							0<j1<n1/2,
						a[0][0] = R[0][0],
						a[0][1] = R[0][n2/2],
						a[n1/2][0] = R[n1/2][0],
						a[n1/2][1] = R[n1/2][n2/2]
		t[0...2*n1-1]
			   :work area (FFT_TYPE *)
		ip[0...*]
			   :work area for bit reversal (int *)
				length of ip >= 2+sqrt(n)  ; if n % 4 == 0
								2+sqrt(n/2); otherwise
				(n = max(n1, n2/2))
				ip[0],ip[1] are pointers of the cos/sin table.
		w[0...*]
			   :cos/sin table (FFT_TYPE *)
				length of w >= max(n1/2, n2/4) + n2/4
				w[],ip[] are initialized if ip[0] == 0.
	[remark]
		Inverse of
			rdft2d(n1, n2, 1, a, t, ip, w);
		is
			rdft2d(n1, n2, -1, a, t, ip, w);
			for (j1 = 0; j1 <= n1 - 1; j1++) {
				for (j2 = 0; j2 <= n2 - 1; j2++) {
					a[j1][j2] *= 2.0 / (n1 * n2);
				}
			}
  */
static void rdft2d(
	int n1, int n2, int isgn, FFT_TYPE** a, FFT_TYPE* t, int* ip, FFT_TYPE* w)
{
	int n, nw, nc, n1h, i, j, i2;
	FFT_TYPE xi;

	n = n1 << 1;
	if (n < n2)
	{
		n = n2;
	}
	nw = ip[0];
	if (n > (nw << 2))
	{
		nw = n >> 2;
		makewt(nw, ip, w);
	}
	nc = ip[1];
	if (n2 > (nc << 2))
	{
		nc = n2 >> 2;
		makect(nc, ip, w + nw);
	}
	n1h = n1 >> 1;
	if (isgn < 0)
	{
		for (i = 1; i <= n1h - 1; i++)
		{
			j = n1 - i;
			xi = a[i][0] - a[j][0];
			a[i][0] += a[j][0];
			a[j][0] = xi;
			xi = a[j][1] - a[i][1];
			a[i][1] += a[j][1];
			a[j][1] = xi;
		}
		for (j = 0; j <= n2 - 2; j += 2)
		{
			for (i = 0; i <= n1 - 1; i++)
			{
				i2 = i << 1;
				t[i2] = a[i][j];
				t[i2 + 1] = a[i][j + 1];
			}
			cdft(n1 << 1, isgn, t, ip, w);
			for (i = 0; i <= n1 - 1; i++)
			{
				i2 = i << 1;
				a[i][j] = t[i2];
				a[i][j + 1] = t[i2 + 1];
			}
		}
		for (i = 0; i <= n1 - 1; i++)
		{
			rdft(n2, isgn, a[i], ip, w);
		}
	}
	else
	{
		for (i = 0; i <= n1 - 1; i++)
		{
			rdft(n2, isgn, a[i], ip, w);
		}
		for (j = 0; j <= n2 - 2; j += 2)
		{
			for (i = 0; i <= n1 - 1; i++)
			{
				i2 = i << 1;
				t[i2] = a[i][j];
				t[i2 + 1] = a[i][j + 1];
			}
			cdft(n1 << 1, isgn, t, ip, w);
			for (i = 0; i <= n1 - 1; i++)
			{
				i2 = i << 1;
				a[i][j] = t[i2];
				a[i][j + 1] = t[i2 + 1];
			}
		}
		for (i = 1; i <= n1h - 1; i++)
		{
			j = n1 - i;
			a[j][0] = 0.5f * (a[i][0] - a[j][0]);
			a[i][0] -= a[j][0];
			a[j][1] = 0.5f * (a[i][1] + a[j][1]);
			a[i][1] -= a[j][1];
		}
	}
}

/**
	Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
	You may use, copy, modify this code for any purpose and
	without fee. You may distribute this ORIGINAL package.

-------- Complex DFT (Discrete Fourier Transform) --------
	[definition]
		<case1>
			X[k1][k2] = sum_j1=0^n1-1 sum_j2=0^n2-1 x[j1][j2] *
							exp(2*pi*i*j1*k1/n1) *
							exp(2*pi*i*j2*k2/n2), 0<=k1<n1, 0<=k2<n2
		<case2>
			X[k1][k2] = sum_j1=0^n1-1 sum_j2=0^n2-1 x[j1][j2] *
							exp(-2*pi*i*j1*k1/n1) *
							exp(-2*pi*i*j2*k2/n2), 0<=k1<n1, 0<=k2<n2
		(notes: sum_j=0^n-1 is a summation from j=0 to n-1)
	[usage]
		<case1>
			ip[0] = 0; // first time only
			cdft2d(n1, 2*n2, 1, a, t, ip, w);
		<case2>
			ip[0] = 0; // first time only
			cdft2d(n1, 2*n2, -1, a, t, ip, w);
	[parameters]
		n1     :data length (int)
				n1 >= 1, n1 = power of 2
		2*n2   :data length (int)
				n2 >= 1, n2 = power of 2
		a[0...n1-1][0...2*n2-1]
			   :input/output data (double **)
				input data
					a[j1][2*j2] = Re(x[j1][j2]),
					a[j1][2*j2+1] = Im(x[j1][j2]),
					0<=j1<n1, 0<=j2<n2
				output data
					a[k1][2*k2] = Re(X[k1][k2]),
					a[k1][2*k2+1] = Im(X[k1][k2]),
					0<=k1<n1, 0<=k2<n2
		t[0...2*n1-1]
			   :work area (double *)
		ip[0...*]
			   :work area for bit reversal (int *)
				length of ip >= 2+sqrt(n)  ; if n % 4 == 0
								2+sqrt(n/2); otherwise
				(n = max(n1, n2))
				ip[0],ip[1] are pointers of the cos/sin table.
		w[0...*]
			   :cos/sin table (double *)
				length of w >= max(n1/2, n2/2)
				w[],ip[] are initialized if ip[0] == 0.
	[remark]
		Inverse of
			cdft2d(n1, 2*n2, -1, a, t, ip, w);
		is
			cdft2d(n1, 2*n2, 1, a, t, ip, w);
			for (j1 = 0; j1 <= n1 - 1; j1++) {
				for (j2 = 0; j2 <= 2 * n2 - 1; j2++) {
					a[j1][j2] *= 1.0 / (n1 * n2);
				}
			}

*/
static void cdft2d(
	int n1, int n2, int isgn, FFT_TYPE** a, FFT_TYPE* t, int* ip, FFT_TYPE* w)
{
	void makewt(int nw, int* ip, FFT_TYPE* w);
	void cdft(int n, int isgn, FFT_TYPE* a, int* ip, FFT_TYPE* w);
	int n, i, j, i2;

	n = n1 << 1;
	if (n < n2)
	{
		n = n2;
	}
	if (n > (ip[0] << 2))
	{
		makewt(n >> 2, ip, w);
	}
	for (i = 0; i <= n1 - 1; i++)
	{
		cdft(n2, isgn, a[i], ip, w);
	}
	for (j = 0; j <= n2 - 2; j += 2)
	{
		for (i = 0; i <= n1 - 1; i++)
		{
			i2 = i << 1;
			t[i2] = a[i][j];
			t[i2 + 1] = a[i][j + 1];
		}
		cdft(n1 << 1, isgn, t, ip, w);
		for (i = 0; i <= n1 - 1; i++)
		{
			i2 = i << 1;
			a[i][j] = t[i2];
			a[i][j + 1] = t[i2 + 1];
		}
	}
}

}  // namespace mrpt::math

void mrpt::math::fft_real(
	CVectorFloat& in_realData, CVectorFloat& out_FFT_Re,
	CVectorFloat& out_FFT_Im, CVectorFloat& out_FFT_Mag)
{
	MRPT_START

	auto n = (unsigned long)in_realData.size();

	// TODO: Test data lenght is 2^N...

	CVectorFloat auxVect(n + 1);

	memcpy(&auxVect[1], &in_realData[0], n * sizeof(auxVect[0]));

	realft(&auxVect[0], n);

	unsigned int n_2 = 1 + (n / 2);

	out_FFT_Re.resize(n_2);
	out_FFT_Im.resize(n_2);
	out_FFT_Mag.resize(n_2);

	for (unsigned int i = 0; i < n_2; i++)
	{
		if (i == (n_2 - 1))
			out_FFT_Re[i] = auxVect[2];
		else
			out_FFT_Re[i] = auxVect[1 + i * 2];

		if (i == 0 || i == (n_2 - 1))
			out_FFT_Im[i] = 0;
		else
			out_FFT_Im[i] = auxVect[1 + i * 2 + 1];

		out_FFT_Mag[i] =
			std::sqrt(square(out_FFT_Re[i]) + square(out_FFT_Im[i]));
	}

	MRPT_END
}

void math::dft2_real(
	const CMatrixFloat& in_data, CMatrixFloat& out_real, CMatrixFloat& out_imag)
{
	MRPT_START

	size_t i, j;
	using float_ptr = FFT_TYPE*;

	// The dimensions:
	size_t dim1 = in_data.rows();
	size_t dim2 = in_data.cols();

	// Transform to format compatible with C routines:
	// ------------------------------------------------------------
	FFT_TYPE** a;
	FFT_TYPE* t;
	int* ip;
	FFT_TYPE* w;

	// Reserve memory and copy data:
	// --------------------------------------
	a = new float_ptr[dim1];
	for (i = 0; i < dim1; i++)
	{
		a[i] = new FFT_TYPE[dim2];
		for (j = 0; j < dim2; j++) a[i][j] = in_data.get_unsafe(i, j);
	}

	t = new FFT_TYPE[2 * dim1 + 20];
	ip = new int[(int)ceil(20 + 2 + sqrt((FFT_TYPE)max(dim1, dim2 / 2)))];
	ip[0] = 0;
	w = new FFT_TYPE[max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20];

	// Do the job!
	// --------------------------------------
	rdft2d((int)dim1, (int)dim2, 1, a, t, ip, w);

	// Transform back to MRPT matrix format:
	// --------------------------------------
	out_real.setSize(dim1, dim2);
	out_imag.setSize(dim1, dim2);

	// a[k1][2*k2] = R[k1][k2] = R[n1-k1][n2-k2],
	// a[k1][2*k2+1] = I[k1][k2] = -I[n1-k1][n2-k2],
	//       0<k1<n1, 0<k2<n2/2,
	for (i = 1; i < dim1; i++)
		for (j = 1; j < dim2 / 2; j++)
		{
			out_real.set_unsafe(i, j, (float)a[i][j * 2]);
			out_real.set_unsafe(dim1 - i, dim2 - j, (float)a[i][j * 2]);
			out_imag.set_unsafe(i, j, (float)-a[i][j * 2 + 1]);
			out_imag.set_unsafe(dim1 - i, dim2 - j, (float)a[i][j * 2 + 1]);
		}
	// a[0][2*k2] = R[0][k2] = R[0][n2-k2],
	// a[0][2*k2+1] = I[0][k2] = -I[0][n2-k2],
	//     0<k2<n2/2,
	for (j = 1; j < dim2 / 2; j++)
	{
		out_real.set_unsafe(0, j, (float)a[0][j * 2]);
		out_real.set_unsafe(0, dim2 - j, (float)a[0][j * 2]);
		out_imag.set_unsafe(0, j, (float)-a[0][j * 2 + 1]);
		out_imag.set_unsafe(0, dim2 - j, (float)a[0][j * 2 + 1]);
	}

	// a[k1][0] = R[k1][0] = R[n1-k1][0],
	// a[k1][1] = I[k1][0] = -I[n1-k1][0],
	// a[n1-k1][1] = R[k1][n2/2] = R[n1-k1][n2/2],
	// a[n1-k1][0] = -I[k1][n2/2] = I[n1-k1][n2/2],
	//    0<k1<n1/2,
	for (i = 1; i < dim1 / 2; i++)
	{
		out_real.set_unsafe(i, 0, (float)a[i][0]);
		out_real.set_unsafe(dim1 - i, 0, (float)a[i][0]);
		out_imag.set_unsafe(i, 0, (float)-a[i][1]);
		out_imag.set_unsafe(dim1 - i, 0, (float)a[i][1]);
		out_real.set_unsafe(i, dim2 / 2, (float)a[dim1 - i][1]);
		out_real.set_unsafe(dim1 - i, dim2 / 2, (float)a[dim1 - i][1]);
		out_imag.set_unsafe(i, dim2 / 2, (float)a[dim1 - i][0]);
		out_imag.set_unsafe(dim1 - i, dim2 / 2, (float)-a[dim1 - i][0]);
	}

	// a[0][0] = R[0][0],
	// a[0][1] = R[0][n2/2],
	// a[n1/2][0] = R[n1/2][0],
	// a[n1/2][1] = R[n1/2][n2/2]
	out_real.set_unsafe(0, 0, (float)a[0][0]);
	out_real.set_unsafe(0, dim2 / 2, (float)a[0][1]);
	out_real.set_unsafe(dim1 / 2, 0, (float)a[dim1 / 2][0]);
	out_real.set_unsafe(dim1 / 2, dim2 / 2, (float)a[dim1 / 2][1]);

	// Free temporary memory:
	for (i = 0; i < dim1; i++) delete[] a[i];
	delete[] a;
	delete[] t;
	delete[] ip;
	delete[] w;

	MRPT_END
}

void math::idft2_real(
	const CMatrixFloat& in_real, const CMatrixFloat& in_imag,
	CMatrixFloat& out_data)
{
	MRPT_START

	size_t i, j;
	using float_ptr = FFT_TYPE*;

	ASSERT_(in_real.rows() == in_imag.rows());
	ASSERT_(in_real.cols() == in_imag.cols());

	// The dimensions:
	size_t dim1 = in_real.rows();
	size_t dim2 = in_real.cols();

	if (mrpt::round2up(dim1) != dim1 || mrpt::round2up(dim2) != dim2)
		THROW_EXCEPTION("Matrix sizes are not a power of two!");

	// Transform to format compatible with C routines:
	// ------------------------------------------------------------
	FFT_TYPE** a;
	FFT_TYPE* t;
	int* ip;
	FFT_TYPE* w;

	// Reserve memory and copy data:
	// --------------------------------------
	a = new float_ptr[dim1];
	for (i = 0; i < dim1; i++) a[i] = new FFT_TYPE[dim2];

	// a[j1][2*j2] = R[j1][j2] = R[n1-j1][n2-j2],
	// a[j1][2*j2+1] = I[j1][j2] = -I[n1-j1][n2-j2],
	//    0<j1<n1, 0<j2<n2/2,
	for (i = 1; i < dim1; i++)
		for (j = 1; j < dim2 / 2; j++)
		{
			a[i][2 * j] = in_real.get_unsafe(i, j);
			a[i][2 * j + 1] = -in_imag.get_unsafe(i, j);
		}

	// a[0][2*j2] = R[0][j2] = R[0][n2-j2],
	// a[0][2*j2+1] = I[0][j2] = -I[0][n2-j2],
	//    0<j2<n2/2,
	for (j = 1; j < dim2 / 2; j++)
	{
		a[0][2 * j] = in_real.get_unsafe(0, j);
		a[0][2 * j + 1] = -in_imag.get_unsafe(0, j);
	}

	// a[j1][0] = R[j1][0] = R[n1-j1][0],
	// a[j1][1] = I[j1][0] = -I[n1-j1][0],
	// a[n1-j1][1] = R[j1][n2/2] = R[n1-j1][n2/2],
	// a[n1-j1][0] = -I[j1][n2/2] = I[n1-j1][n2/2],
	//    0<j1<n1/2,
	for (i = 1; i < dim1 / 2; i++)
	{
		a[i][0] = in_real.get_unsafe(i, 0);
		a[i][1] = -in_imag.get_unsafe(i, 0);
		a[dim1 - i][1] = in_real.get_unsafe(i, dim2 / 2);
		a[dim1 - i][0] = in_imag.get_unsafe(i, dim2 / 2);
	}

	// a[0][0] = R[0][0],
	// a[0][1] = R[0][n2/2],
	// a[n1/2][0] = R[n1/2][0],
	// a[n1/2][1] = R[n1/2][n2/2]
	a[0][0] = in_real.get_unsafe(0, 0);
	a[0][1] = in_real.get_unsafe(0, dim2 / 2);
	a[dim1 / 2][0] = in_real.get_unsafe(dim1 / 2, 0);
	a[dim1 / 2][1] = in_real.get_unsafe(dim1 / 2, dim2 / 2);

	t = new FFT_TYPE[2 * dim1 + 20];
	ip = new int[(int)ceil(20 + 2 + sqrt((FFT_TYPE)max(dim1, dim2 / 2)))];
	ip[0] = 0;
	w = new FFT_TYPE[max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20];

	// Do the job!
	// --------------------------------------
	rdft2d((int)dim1, (int)dim2, -1, a, t, ip, w);

	// Transform back to MRPT matrix format:
	// --------------------------------------
	out_data.setSize(dim1, dim2);

	FFT_TYPE scale = 2.0f / (dim1 * dim2);

	for (i = 0; i < dim1; i++)
		for (j = 0; j < dim2; j++)
			out_data.set_unsafe(i, j, (float)(a[i][j] * scale));

	// Free temporary memory:
	for (i = 0; i < dim1; i++) delete[] a[i];
	delete[] a;
	delete[] t;
	delete[] ip;
	delete[] w;

	MRPT_END
}

/*---------------------------------------------------------------
						myGeneralDFT

	sign ->  -1: DFT, 1:IDFT
 ---------------------------------------------------------------*/
static void myGeneralDFT(
	int sign, const CMatrixFloat& in_real, const CMatrixFloat& in_imag,
	CMatrixFloat& out_real, CMatrixFloat& out_imag)
{
	ASSERT_(in_real.rows() == in_imag.rows());
	ASSERT_(in_real.cols() == in_imag.cols());

	// The dimensions:
	size_t dim1 = in_real.rows();
	size_t dim2 = in_real.cols();

	size_t k1, k2, n1, n2;
	float w_r, w_i;
	auto ang1 = (float)(sign * M_2PI / dim1);
	auto ang2 = (float)(sign * M_2PI / dim2);
	float phase;
	float R, I;
	float scale = sign == 1 ? (1.0f / (dim1 * dim2)) : 1;

	out_real.setSize(dim1, dim2);
	out_imag.setSize(dim1, dim2);

	for (k1 = 0; k1 < dim1; k1++)
	{
		for (k2 = 0; k2 < dim2; k2++)
		{
			R = I = 0;  // Accum:

			for (n1 = 0; n1 < dim1; n1++)
			{
				phase = ang1 * n1 * k1;
				for (n2 = 0; n2 < dim2; n2++)
				{
					w_r = cos(phase);
					w_i = sin(phase);

					R += w_r * in_real.get_unsafe(n1, n2) -
						 w_i * in_imag.get_unsafe(n1, n2);
					I += w_i * in_real.get_unsafe(n1, n2) +
						 w_r * in_imag.get_unsafe(n1, n2);

					phase += ang2 * k2;
				}  // end for k2
			}  // end for k1

			// Save result:
			out_real.set_unsafe(k1, k2, R * scale);
			out_imag.set_unsafe(k1, k2, I * scale);

		}  // end for k2
	}  // end for k1
}

/*---------------------------------------------------------------
						dft2_complex
 ---------------------------------------------------------------*/
void math::dft2_complex(
	const CMatrixFloat& in_real, const CMatrixFloat& in_imag,
	CMatrixFloat& out_real, CMatrixFloat& out_imag)
{
	MRPT_START

	ASSERT_(in_real.rows() == in_imag.rows());
	ASSERT_(in_real.cols() == in_imag.cols());

	// The dimensions:
	size_t dim1 = in_real.rows();
	size_t dim2 = in_real.cols();
	size_t i, j;

	bool dim1IsPower2 = (mrpt::round2up(dim1) == dim1);
	bool dim2IsPower2 = (mrpt::round2up(dim2) == dim2);

	// FFT or DFT??
	if (dim1IsPower2 && dim2IsPower2)
	{
		// ----------------------------------------
		//			Optimized FFT:
		// ----------------------------------------
		using float_ptr = FFT_TYPE*;

		// Transform to format compatible with C routines:
		// ------------------------------------------------------------
		static FFT_TYPE** a = nullptr;
		static FFT_TYPE* t = nullptr;
		static int* ip = nullptr;
		static FFT_TYPE* w = nullptr;

		// Reserve memory
		// --------------------------------------
		static int alreadyInitSize1 = -1, alreadyInitSize2 = -1;

		if (alreadyInitSize1 != (int)dim1 || alreadyInitSize2 != (int)dim2)
		{
			// Create/realloc buffers:
			if (a)
			{
				for (i = 0; i < dim1; i++) delete[] a[i];
				delete[] a;
			}
			if (ip) delete[] ip;
			if (t) delete[] t;
			if (w) delete[] w;

			alreadyInitSize1 = (int)dim1;
			alreadyInitSize2 = (int)dim2;

			a = new float_ptr[dim1];
			for (i = 0; i < dim1; i++) a[i] = new FFT_TYPE[2 * dim2];

			t = new FFT_TYPE[2 * dim1 + 20];
			ip = new int[(int)ceil(
				20 + 2 + sqrt((FFT_TYPE)max(dim1, dim2 / 2)))];
			ip[0] = 0;
			w = new FFT_TYPE[max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20];
		}

		// and copy data:
		// --------------------------------------
		for (i = 0; i < dim1; i++)
			for (j = 0; j < dim2; j++)
			{
				a[i][2 * j + 0] = in_real.get_unsafe(i, j);
				a[i][2 * j + 1] = in_imag.get_unsafe(i, j);
			}

		// Do the job!
		// --------------------------------------
		cdft2d((int)dim1, (int)(2 * dim2), 1, a, t, ip, w);

		// Transform back to MRPT matrix format:
		// --------------------------------------
		out_real.setSize(dim1, dim2);
		out_imag.setSize(dim1, dim2);

		// a[k1][2*k2] = Re(X[k1][k2]),
		// a[k1][2*k2+1] = Im(X[k1][k2]),
		// 0<=k1<n1, 0<=k2<n2
		for (i = 0; i < dim1; i++)
			for (j = 0; j < dim2; j++)
			{
				out_real.set_unsafe(i, j, (float)a[i][j * 2 + 0]);
				out_imag.set_unsafe(i, j, (float)a[i][j * 2 + 1]);
			}

	}  // end FFT
	else
	{
		// ----------------------------------------
		//			General DFT:
		// ----------------------------------------
		printf("Using general DFT...\n");
		myGeneralDFT(-1, in_real, in_imag, out_real, out_imag);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						idft2_complex
 ---------------------------------------------------------------*/
void math::idft2_complex(
	const CMatrixFloat& in_real, const CMatrixFloat& in_imag,
	CMatrixFloat& out_real, CMatrixFloat& out_imag)
{
	MRPT_START

	ASSERT_(in_real.rows() == in_imag.rows());
	ASSERT_(in_real.cols() == in_imag.cols());

	// The dimensions:
	size_t dim1 = in_real.rows();
	size_t dim2 = in_real.cols();
	size_t i, j;

	bool dim1IsPower2 = (mrpt::round2up(dim1) == dim1);
	bool dim2IsPower2 = (mrpt::round2up(dim2) == dim2);

	// FFT or DFT??
	if (dim1IsPower2 && dim2IsPower2)
	{
		using float_ptr = FFT_TYPE*;
		// ----------------------------------------
		//			Optimized FFT:
		// ----------------------------------------

		// Transform to format compatible with C routines:
		// ------------------------------------------------------------
		static FFT_TYPE** a = nullptr;
		static FFT_TYPE* t = nullptr;
		static int* ip = nullptr;
		static FFT_TYPE* w = nullptr;

		// Reserve memory
		// --------------------------------------

		// and copy data:
		// --------------------------------------
		static int alreadyInitSize1 = -1, alreadyInitSize2 = -1;

		if (alreadyInitSize1 != (int)dim1 || alreadyInitSize2 != (int)dim2)
		{
			// Create/realloc buffers:
			if (a)
			{
				for (i = 0; i < dim1; i++) delete[] a[i];
				delete[] a;
			}
			if (ip) delete[] ip;
			if (t) delete[] t;
			if (w) delete[] w;

			alreadyInitSize1 = (int)dim1;
			alreadyInitSize2 = (int)dim2;

			a = new float_ptr[dim1];
			for (i = 0; i < dim1; i++) a[i] = new FFT_TYPE[2 * dim2];
			t = new FFT_TYPE[2 * dim1 + 20];
			ip = new int[(int)ceil(
				20 + 2 + sqrt((FFT_TYPE)max(dim1, dim2 / 2)))];
			ip[0] = 0;
			w = new FFT_TYPE[max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20];
		}

		// and copy data:
		// --------------------------------------
		for (i = 0; i < dim1; i++)
			for (j = 0; j < dim2; j++)
			{
				a[i][2 * j + 0] = in_real.get_unsafe(i, j);
				a[i][2 * j + 1] = in_imag.get_unsafe(i, j);
			}

		// Do the job!
		// --------------------------------------
		cdft2d((int)dim1, (int)(2 * dim2), -1, a, t, ip, w);

		// Transform back to MRPT matrix format:
		// --------------------------------------
		out_real.setSize(dim1, dim2);
		out_imag.setSize(dim1, dim2);

		FFT_TYPE scale = 1.0f / (dim1 * dim2);

		// a[k1][2*k2] = Re(X[k1][k2]),
		// a[k1][2*k2+1] = Im(X[k1][k2]),
		// 0<=k1<n1, 0<=k2<n2
		for (i = 0; i < dim1; i++)
			for (j = 0; j < dim2; j++)
			{
				//				out_real.set_unsafe(i,j,(float)(a[i][j*2+0]*scale));
				//				out_imag.set_unsafe(i,j,(float)(a[i][j*2+1]*scale));
				out_real.set_unsafe(i, j, (float)(a[i][j * 2 + 0]));
				out_imag.set_unsafe(i, j, (float)(a[i][j * 2 + 1]));
			}

		out_real *= scale;
		out_imag *= scale;

		// The element (0,0) is purely real!
		out_imag.set_unsafe(0, 0, 0);

	}  // end FFT
	else
	{
		// ----------------------------------------
		//			General DFT:
		// ----------------------------------------
		printf("Using general DFT...\n");
		myGeneralDFT(1, in_real, in_imag, out_real, out_imag);
	}

	MRPT_END
}

void mrpt::math::cross_correlation_FFT(
	const CMatrixFloat& A, const CMatrixFloat& B, CMatrixFloat& out_corr)
{
	MRPT_START

	ASSERT_(A.cols() == B.cols() && A.rows() == B.rows());
	if (mrpt::round2up(A.rows()) != A.rows() ||
		mrpt::round2up(A.cols()) != A.cols())
		THROW_EXCEPTION("Size of input matrices must be powers of two.");

	// Find smallest valid size:
	size_t x, y;
	const size_t lx = A.cols();
	const size_t ly = A.rows();

	const CMatrixFloat& i1 = A;
	const CMatrixFloat& i2 = B;

	// FFT:
	CMatrixFloat I1_R, I1_I, I2_R, I2_I, ZEROS(ly, lx);
	math::dft2_complex(i1, ZEROS, I1_R, I1_I);
	math::dft2_complex(i2, ZEROS, I2_R, I2_I);

	// Compute the COMPLEX division of I2 by I1:
	for (y = 0; y < ly; y++)
		for (x = 0; x < lx; x++)
		{
			float r1 = I1_R.get_unsafe(y, x);
			float r2 = I2_R.get_unsafe(y, x);

			float ii1 = I1_I.get_unsafe(y, x);
			float ii2 = I2_I.get_unsafe(y, x);

			float den = square(r1) + square(ii1);
			I2_R.set_unsafe(y, x, (r1 * r2 + ii1 * ii2) / den);
			I2_I.set_unsafe(y, x, (ii2 * r1 - r2 * ii1) / den);
		}

	// IFFT:
	CMatrixFloat res_R, res_I;
	math::idft2_complex(I2_R, I2_I, res_R, res_I);

	out_corr.setSize(ly, lx);
	for (y = 0; y < ly; y++)
		for (x = 0; x < lx; x++)
			out_corr(y, x) = sqrt(square(res_R(y, x)) + square(res_I(y, x)));

	MRPT_END
}
