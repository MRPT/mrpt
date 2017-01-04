/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/utils.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/interp_fit.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/fourier.h>

#include <mrpt/system/string_utils.h>
#include <mrpt/utils/CFileStream.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/math/ops_matrices.h>

#include <float.h>
#include <algorithm>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Next there are declared some auxiliary functions:
namespace mrpt
{
	namespace math
	{
		//Replaces data[1..2*nn] by its discrete Fourier transform, if isign is input as 1; or replaces
		//data[1..2*nn] by nn times its inverse discrete Fourier transform, if isign is input as -1.
		//data is a complex array of length nn or, equivalently, a real array of length 2*nn. nn MUST
		//be an integer power of 2 (this is not checked for!).
		void four1(float data[], unsigned long nn, int isign)
		{
			unsigned long	n,mmax,m,j,i;
			double			wtemp,wr,wpr,wpi,wi,theta; // Double precision for the trigonometric recurrences.
			float			tempr,tempi;

			n=nn << 1;
			j=1;

			for (i=1;i<n;i+=2)	// This is the bit-reversal section of the routine.
			{
				if (j > i)
				{
					swap(data[j],data[i]);	// Exchange the two complex numbers.
					swap(data[j+1],data[i+1]);
				}
				m=nn;
				while (m >= 2 && j > m)
				{
					j -= m;
					m >>= 1;
				}
				j += m;
			}
			// Here begins the Danielson-Lanczos section of the routine.
			mmax=2;
			while (n > mmax) // Outer loop executed log2 nn times.
			{
				unsigned long istep=mmax << 1;
				theta=isign*(6.28318530717959/mmax);	// Initialize the trigonometric recurrence.
				wtemp=sin(0.5*theta);
				wpr = -2.0*wtemp*wtemp;
				wpi=sin(theta);
				wr=1.0;
				wi=0.0;
				for (m=1;m<mmax;m+=2) // Here are the two nested inner loops.
				{
					for (i=m;i<=n;i+=istep)
					{
						j=i+mmax; // This is the Danielson-Lanczos formula:
						tempr=(float) (wr*data[j]-wi*data[j+1]);
						tempi=(float) (wr*data[j+1]+wi*data[j]);
						data[j]=data[i]-tempr;
						data[j+1]=data[i+1]-tempi;
						data[i] += tempr;
						data[i+1] += tempi;
					}
					wr=(wtemp=wr)*wpr-wi*wpi+wr; // Trigonometric recurrence.
					wi=wi*wpr+wtemp*wpi+wi;
				}
			mmax=istep;
			}
		}


		//Calculates the Fourier transform of a set of n real-valued data points. Replaces this data (which
		//is stored in array data[1..n]) by the positive frequency half of its complex Fourier transform.
		//The real-valued first and last components of the complex transform are returned as elements
		//data[1] and data[2], respectively. n must be a power of 2. This routine also calculates the
		//inverse transform of a complex data array if it is the transform of real data. (Result in this case
		//must be multiplied by 2/n.)
		void realft(float data[], unsigned long n)
		{
			unsigned long		i,i1,i2,i3,i4,np3;
			float				c1=0.5,c2,h1r,h1i,h2r,h2i;
			double				wr,wi,wpr,wpi,wtemp,theta;	// Double precision for the trigonometric recurrences.
			theta=3.141592653589793/(double) (n>>1);		// Initialize the recurrence.

			c2 = -0.5;
			four1(data,n>>1,1);		// The forward transform is here.

			wtemp=sin(0.5*theta);
			wpr = -2.0*wtemp*wtemp;
			wpi=sin(theta);
			wr=1.0+wpr;
			wi=wpi;
			np3=n+3;
			for (i=2;i<=(n>>2);i++) // Case i=1 done separately below.
			{
				i4=1+(i3=np3-(i2=1+(i1=i+i-1)));
				h1r=c1*(data[i1]+data[i3]);		// The two separate transforms are separated out of data.
				h1i=c1*(data[i2]-data[i4]);
				h2r = -c2*(data[i2]+data[i4]);
				h2i=c2*(data[i1]-data[i3]);
				data[i1]=(float)(h1r+wr*h2r-wi*h2i);		// Here they are recombined to form the true transform of the original real data.
				data[i2]=(float)(h1i+wr*h2i+wi*h2r);
				data[i3]=(float)(h1r-wr*h2r+wi*h2i);
				data[i4]=(float)( -h1i+wr*h2i+wi*h2r);
				wr=(wtemp=wr)*wpr-wi*wpi+wr; // The recurrence.
				wi=wi*wpr+wtemp*wpi+wi;
			}

			data[1] = (h1r=data[1])+data[2];
			// Squeeze the first and last data together to get them all within the original array.
			data[2] = h1r-data[2];
		}

		/**
			Copyright(C) 1997 Takuya OOURA (email: ooura@mmm.t.u-tokyo.ac.jp).
			You may use, copy, modify this code for any purpose and
			without fee. You may distribute this ORIGINAL package.
		  */
		typedef	float	FFT_TYPE;

		void makewt(int nw, int *ip, FFT_TYPE *w);
		void bitrv2(int n, int *ip, FFT_TYPE *a);
		void cftbsub(int n, FFT_TYPE *a, FFT_TYPE *w);
		void cftfsub(int n, FFT_TYPE *a, FFT_TYPE *w);
		void dctsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c);
		void dstsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c);
		void dctsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c);
		void rftfsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c);
		void rftbsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c);


		void cftbsub(int n, FFT_TYPE *a, FFT_TYPE *w)
		{
			int j, j1, j2, j3, k, k1, ks, l, m;
			FFT_TYPE wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
			FFT_TYPE x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

			l = 2;
			while ((l << 1) < n) {
				m = l << 2;
				for (j = 0; j <= l - 2; j += 2) {
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
				if (m < n) {
					wk1r = w[2];
					for (j = m; j <= l + m - 2; j += 2) {
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
					for (k = (m << 1); k <= n - m; k += m) {
						k1++;
						ks = -ks;
						wk1r = w[k1 << 1];
						wk1i = w[(k1 << 1) + 1];
						wk2r = ks * w[k1];
						wk2i = w[k1 + ks];
						wk3r = wk1r - 2 * wk2i * wk1i;
						wk3i = 2 * wk2i * wk1r - wk1i;
						for (j = k; j <= l + k - 2; j += 2) {
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
			if (l < n) {
				for (j = 0; j <= l - 2; j += 2) {
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

		void cftfsub(int n, FFT_TYPE *a, FFT_TYPE *w)
		{
			int j, j1, j2, j3, k, k1, ks, l, m;
			FFT_TYPE wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
			FFT_TYPE x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

			l = 2;
			while ((l << 1) < n) {
				m = l << 2;
				for (j = 0; j <= l - 2; j += 2) {
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
				if (m < n) {
					wk1r = w[2];
					for (j = m; j <= l + m - 2; j += 2) {
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
					for (k = (m << 1); k <= n - m; k += m) {
						k1++;
						ks = -ks;
						wk1r = w[k1 << 1];
						wk1i = w[(k1 << 1) + 1];
						wk2r = ks * w[k1];
						wk2i = w[k1 + ks];
						wk3r = wk1r - 2 * wk2i * wk1i;
						wk3i = 2 * wk2i * wk1r - wk1i;
						for (j = k; j <= l + k - 2; j += 2) {
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
			if (l < n) {
				for (j = 0; j <= l - 2; j += 2) {
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



		void makewt(int nw, int *ip, FFT_TYPE *w)
		{
			void bitrv2(int n, int *ip, FFT_TYPE *a);
			int nwh, j;
			FFT_TYPE delta, x, y;

			ip[0] = nw;
			ip[1] = 1;
			if (nw > 2) {
				nwh = nw >> 1;
				delta = atan(1.0f) / nwh;
				w[0] = 1;
				w[1] = 0;
				w[nwh] = cos(delta * nwh);
				w[nwh + 1] = w[nwh];
				for (j = 2; j <= nwh - 2; j += 2) {
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
		void makect(int nc, int *ip, FFT_TYPE *c)
		{
			int nch, j;
			FFT_TYPE delta;

			ip[1] = nc;
			if (nc > 1) {
				nch = nc >> 1;
				delta = atan(1.0f) / nch;
				c[0] = 0.5f;
				c[nch] = 0.5f * cos(delta * nch);
				for (j = 1; j <= nch - 1; j++) {
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
		void bitrv2(int n, int *ip, FFT_TYPE *a)
		{
			int j, j1, k, k1, l, m, m2;
			FFT_TYPE xr, xi;

			ip[0] = 0;
			l = n;
			m = 1;
			while ((m << 2) < l) {
				l >>= 1;
				for (j = 0; j <= m - 1; j++) {
					ip[m + j] = ip[j] + l;
				}
				m <<= 1;
			}
			if ((m << 2) > l) {
				for (k = 1; k <= m - 1; k++) {
					for (j = 0; j <= k - 1; j++) {
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
			} else {
				m2 = m << 1;
				for (k = 1; k <= m - 1; k++) {
					for (j = 0; j <= k - 1; j++) {
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
		void cdft(int n, int isgn, FFT_TYPE *a, int *ip, FFT_TYPE *w)
		{
			if (n > (ip[0] << 2)) {
				makewt(n >> 2, ip, w);
			}
			if (n > 4) {
				bitrv2(n, ip + 2, a);
			}
			if (isgn < 0) {
				cftfsub(n, a, w);
			} else {
				cftbsub(n, a, w);
			}
		}

		void rftfsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c)
		{
			int j, k, kk, ks;
			FFT_TYPE wkr, wki, xr, xi, yr, yi;

			ks = (nc << 2) / n;
			kk = 0;
			for (k = (n >> 1) - 2; k >= 2; k -= 2) {
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

		void dctsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c)
		{
			int j, k, kk, ks, m;
			FFT_TYPE wkr, wki, xr;

			ks = nc / n;
			kk = ks;
			m = n >> 1;
			for (k = 1; k <= m - 1; k++) {
				j = n - k;
				wkr = c[kk] - c[nc - kk];
				wki = c[kk] + c[nc - kk];
				kk += ks;
				xr = wki * a[k] - wkr * a[j];
				a[k] = wkr * a[k] + wki * a[j];
				a[j] = xr;
			}
			a[m] *= 2 * c[kk];
		}

		void dstsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c)
		{
			int j, k, kk, ks, m;
			FFT_TYPE wkr, wki, xr;

			ks = nc / n;
			kk = ks;
			m = n >> 1;
			for (k = 1; k <= m - 1; k++) {
				j = n - k;
				wkr = c[kk] - c[nc - kk];
				wki = c[kk] + c[nc - kk];
				kk += ks;
				xr = wki * a[j] - wkr * a[k];
				a[j] = wkr * a[j] + wki * a[k];
				a[k] = xr;
			}
			a[m] *= 2 * c[kk];
		}


		void rdft(int n, int isgn, FFT_TYPE *a, int *ip, FFT_TYPE *w)
		{
			int nw, nc;
			FFT_TYPE xi;

			nw = ip[0];
			if (n > (nw << 2)) {
				nw = n >> 2;
				makewt(nw, ip, w);
			}
			nc = ip[1];
			if (n > (nc << 2)) {
				nc = n >> 2;
				makect(nc, ip, w + nw);
			}
			if (isgn < 0) {
				a[1] = 0.5f * (a[0] - a[1]);
				a[0] -= a[1];
				if (n > 4) {
					rftfsub(n, a, nc, w + nw);
					bitrv2(n, ip + 2, a);
				}
				cftfsub(n, a, w);
			} else {
				if (n > 4) {
					bitrv2(n, ip + 2, a);
				}
				cftbsub(n, a, w);
				if (n > 4) {
					rftbsub(n, a, nc, w + nw);
				}
				xi = a[0] - a[1];
				a[0] += a[1];
				a[1] = xi;
			}
		}

		void rftbsub(int n, FFT_TYPE *a, int nc, FFT_TYPE *c)
		{
			int j, k, kk, ks;
			FFT_TYPE wkr, wki, xr, xi, yr, yi;

			ks = (nc << 2) / n;
			kk = 0;
			for (k = (n >> 1) - 2; k >= 2; k -= 2) {
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
		void rdft2d(int n1, int n2, int isgn, FFT_TYPE **a, FFT_TYPE *t, int *ip, FFT_TYPE *w)
		{
			int n, nw, nc, n1h, i, j, i2;
			FFT_TYPE xi;

			n = n1 << 1;
			if (n < n2) {
				n = n2;
			}
			nw = ip[0];
			if (n > (nw << 2)) {
				nw = n >> 2;
				makewt(nw, ip, w);
			}
			nc = ip[1];
			if (n2 > (nc << 2)) {
				nc = n2 >> 2;
				makect(nc, ip, w + nw);
			}
			n1h = n1 >> 1;
			if (isgn < 0) {
				for (i = 1; i <= n1h - 1; i++) {
					j = n1 - i;
					xi = a[i][0] - a[j][0];
					a[i][0] += a[j][0];
					a[j][0] = xi;
					xi = a[j][1] - a[i][1];
					a[i][1] += a[j][1];
					a[j][1] = xi;
				}
				for (j = 0; j <= n2 - 2; j += 2) {
					for (i = 0; i <= n1 - 1; i++) {
						i2 = i << 1;
						t[i2] = a[i][j];
						t[i2 + 1] = a[i][j + 1];
					}
					cdft(n1 << 1, isgn, t, ip, w);
					for (i = 0; i <= n1 - 1; i++) {
						i2 = i << 1;
						a[i][j] = t[i2];
						a[i][j + 1] = t[i2 + 1];
					}
				}
				for (i = 0; i <= n1 - 1; i++) {
					rdft(n2, isgn, a[i], ip, w);
				}
			} else {
				for (i = 0; i <= n1 - 1; i++) {
					rdft(n2, isgn, a[i], ip, w);
				}
				for (j = 0; j <= n2 - 2; j += 2) {
					for (i = 0; i <= n1 - 1; i++) {
						i2 = i << 1;
						t[i2] = a[i][j];
						t[i2 + 1] = a[i][j + 1];
					}
					cdft(n1 << 1, isgn, t, ip, w);
					for (i = 0; i <= n1 - 1; i++) {
						i2 = i << 1;
						a[i][j] = t[i2];
						a[i][j + 1] = t[i2 + 1];
					}
				}
				for (i = 1; i <= n1h - 1; i++) {
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
		void cdft2d(int n1, int n2, int isgn, FFT_TYPE **a, FFT_TYPE *t,
			int *ip, FFT_TYPE *w)
		{
			void makewt(int nw, int *ip, FFT_TYPE *w);
			void cdft(int n, int isgn, FFT_TYPE *a, int *ip, FFT_TYPE *w);
			int n, i, j, i2;

			n = n1 << 1;
			if (n < n2) {
				n = n2;
			}
			if (n > (ip[0] << 2)) {
				makewt(n >> 2, ip, w);
			}
			for (i = 0; i <= n1 - 1; i++) {
				cdft(n2, isgn, a[i], ip, w);
			}
			for (j = 0; j <= n2 - 2; j += 2) {
				for (i = 0; i <= n1 - 1; i++) {
					i2 = i << 1;
					t[i2] = a[i][j];
					t[i2 + 1] = a[i][j + 1];
				}
				cdft(n1 << 1, isgn, t, ip, w);
				for (i = 0; i <= n1 - 1; i++) {
					i2 = i << 1;
					a[i][j] = t[i2];
					a[i][j + 1] = t[i2 + 1];
				}
			}
		}


	} // end namespace
} // end namespace

/*---------------------------------------------------------------
						normalPDF
 ---------------------------------------------------------------*/
double  math::normalPDF(double x, double mu, double std)
{
	return  ::exp( -0.5 * square((x-mu)/std) ) / (std*2.506628274631000502415765284811) ;
}


/*---------------------------------------------------------------
						erfc
 ---------------------------------------------------------------*/
double  math::erfc(const double x)
{
#if defined(HAVE_ERF)
	return ::erfc(x);
#else
// copied from TMath for those platforms which do not have a
// C99 compliant compiler

// Compute the complementary error function erfc(x).
// Erfc(x) = (2/sqrt(pi)) Integral(exp(-t^2))dt between x and infinity
//
//--- Nve 14-nov-1998 UU-SAP Utrecht

// The parameters of the Chebyshev fit
const double a1 = -1.26551223,   a2 = 1.00002368,
a3 =  0.37409196,   a4 = 0.09678418,
a5 = -0.18628806,   a6 = 0.27886807,
a7 = -1.13520398,   a8 = 1.48851587,
a9 = -0.82215223,  a10 = 0.17087277;

double v = 1.0; // The return value
double z = ::fabs(x);

if (z <= 0) return v; // erfc(0)=1

double t = 1.0/(1.0+0.5*z);

v = t* ::exp((-z*z) +a1+t*(a2+t*(a3+t*(a4+t*(a5+t*(a6+t*(a7+t*(a8+t*(a9+t*a10)))))))));

if (x < 0) v = 2.0-v; // erfc(-x)=2-erfc(x)

return v;
#endif
}

/*---------------------------------------------------------------
						erf
 ---------------------------------------------------------------*/
double  math::erf(double x)
{
#if defined(HAVE_ERF)
	return ::erf(x);
#else
	return (1.0 - math::erfc(x));
#endif
}

/*---------------------------------------------------------------
						chi2inv
 ---------------------------------------------------------------*/
double  math::chi2inv(double P, unsigned int dim)
{
	ASSERT_(P>=0 && P<1)
	if (P==0)
	     return 0;
	else return dim * pow( 1.0 - 2.0/(9*dim) + sqrt(2.0/(9*dim))*normalQuantile(P), 3 );
}

/*---------------------------------------------------------------
						factorial64
 ---------------------------------------------------------------*/
uint64_t  math::factorial64(unsigned int n)
{
	uint64_t		ret = 1;

	for (unsigned int i=2;i<=n;i++)
		ret *= i;

	return ret;
}

/*---------------------------------------------------------------
						factorial
 ---------------------------------------------------------------*/
double  math::factorial(unsigned int n)
{
	double	retLog=0;

	for (unsigned int i=2;i<=n;i++)
		retLog += ::log( (double)n );

	return ::exp(retLog);
}

/*---------------------------------------------------------------
						fft_real
 ---------------------------------------------------------------*/
void  math::fft_real(	CVectorFloat	&in_realData,
						CVectorFloat	&out_FFT_Re,
						CVectorFloat	&out_FFT_Im,
						CVectorFloat	&out_FFT_Mag )
{
	MRPT_START

	unsigned long	n = (unsigned long)in_realData.size();

	// TODO: Test data lenght is 2^N...

	CVectorFloat		auxVect( n+1 );

	memcpy( &auxVect[1], &in_realData[0], n * sizeof(auxVect[0]));

	realft( &auxVect[0],  n );

	unsigned int				n_2 = 1 + (n / 2);

	out_FFT_Re.resize(n_2);
	out_FFT_Im.resize(n_2);
	out_FFT_Mag.resize(n_2);

	for (unsigned int i=0;i<n_2;i++)
	{
		if (i==(n_2-1))
				out_FFT_Re[i] = auxVect[2];
		else	out_FFT_Re[i] = auxVect[1+i*2];

		if (i==0 || i==(n_2-1))
				out_FFT_Im[i] = 0;
		else	out_FFT_Im[i] = auxVect[1+i*2+1];

		out_FFT_Mag[i] = sqrt( square(out_FFT_Re[i])+square(out_FFT_Im[i]) );
	}

	MRPT_END
}

/*---------------------------------------------------------------
						normalQuantile
 ---------------------------------------------------------------*/
double  math::normalQuantile(double p)
{
	double q, t, u;

	static const double a[6] =
		{
		-3.969683028665376e+01,  2.209460984245205e+02,
		-2.759285104469687e+02,  1.383577518672690e+02,
		-3.066479806614716e+01,  2.506628277459239e+00
		};
	static const double b[5] =
		{
		-5.447609879822406e+01,  1.615858368580409e+02,
		-1.556989798598866e+02,  6.680131188771972e+01,
		-1.328068155288572e+01
		};
	static const double c[6] =
		{
		-7.784894002430293e-03, -3.223964580411365e-01,
		-2.400758277161838e+00, -2.549732539343734e+00,
		4.374664141464968e+00,  2.938163982698783e+00
		};
	static const double d[4] =
		{
		7.784695709041462e-03,  3.224671290700398e-01,
		2.445134137142996e+00,  3.754408661907416e+00
		};

	ASSERT_(!isNaN(p))
	ASSERT_(p < 1.0 && p > 0.0)

	q = min(p,1-p);

	if (q > 0.02425)
	{
		/* Rational approximation for central region. */
		u = q-0.5;
		t = u*u;
		u = u*(((((a[0]*t+a[1])*t+a[2])*t+a[3])*t+a[4])*t+a[5])
		/(((((b[0]*t+b[1])*t+b[2])*t+b[3])*t+b[4])*t+1);
	}
	else
	{
		/* Rational approximation for tail region. */
		t = sqrt(-2*::log(q));
		u = (((((c[0]*t+c[1])*t+c[2])*t+c[3])*t+c[4])*t+c[5])
		/((((d[0]*t+d[1])*t+d[2])*t+d[3])*t+1);
	}

	/* The relative error of the approximation has absolute value less
	than 1.15e-9.  One iteration of Halley's rational method (third
	order) gives full machine precision... */
	t = normalCDF(u)-q;    /* error */
	t = t* 2.506628274631000502415765284811 * ::exp(u*u/2);   /* f(u)/df(u) */
	u = u-t/(1+u*t/2);     /* Halley's method */

	return (p > 0.5 ? -u : u);
}

/*---------------------------------------------------------------
						normalCDF
 ---------------------------------------------------------------*/
double  math::normalCDF(double u)
{
	static const double a[5] =
		{
		1.161110663653770e-002,3.951404679838207e-001,2.846603853776254e+001,
		1.887426188426510e+002,3.209377589138469e+003
		};
	static const double b[5] =
		{
		1.767766952966369e-001,8.344316438579620e+000,1.725514762600375e+002,
		1.813893686502485e+003,8.044716608901563e+003
		};
	static const double c[9] =
		{
		2.15311535474403846e-8,5.64188496988670089e-1,8.88314979438837594e00,
		6.61191906371416295e01,2.98635138197400131e02,8.81952221241769090e02,
		1.71204761263407058e03,2.05107837782607147e03,1.23033935479799725E03
		};
	static const double d[9] =
		{
		1.00000000000000000e00,1.57449261107098347e01,1.17693950891312499e02,
		5.37181101862009858e02,1.62138957456669019e03,3.29079923573345963e03,
		4.36261909014324716e03,3.43936767414372164e03,1.23033935480374942e03
		};
	static const double p[6] =
		{
		1.63153871373020978e-2,3.05326634961232344e-1,3.60344899949804439e-1,
		1.25781726111229246e-1,1.60837851487422766e-2,6.58749161529837803e-4
		};
	static const double q[6] =
		{
		1.00000000000000000e00,2.56852019228982242e00,1.87295284992346047e00,
		5.27905102951428412e-1,6.05183413124413191e-2,2.33520497626869185e-3
		};
	double y, z;

	ASSERT_( !isNaN(u) );
	ASSERT_(  isFinite(u) );

	y = fabs(u);
	if (y <= 0.46875* 1.4142135623730950488016887242097 )
	{
		/* evaluate erf() for |u| <= sqrt(2)*0.46875 */
		z = y*y;
		y = u*((((a[0]*z+a[1])*z+a[2])*z+a[3])*z+a[4])
			/((((b[0]*z+b[1])*z+b[2])*z+b[3])*z+b[4]);
		return 0.5+y;
	}

	z = ::exp(-y*y/2)/2;
	if (y <= 4.0)
	{
		/* evaluate erfc() for sqrt(2)*0.46875 <= |u| <= sqrt(2)*4.0 */
		y = y/ 1.4142135623730950488016887242097 ;
		y =
		((((((((c[0]*y+c[1])*y+c[2])*y+c[3])*y+c[4])*y+c[5])*y+c[6])*y+c[7])*y+c[8])


		/((((((((d[0]*y+d[1])*y+d[2])*y+d[3])*y+d[4])*y+d[5])*y+d[6])*y+d[7])*y+d[8]);

		y = z*y;
	}
	else
	{
		/* evaluate erfc() for |u| > sqrt(2)*4.0 */
		z = z* 1.4142135623730950488016887242097 /y;
		y = 2/(y*y);
			y = y*(((((p[0]*y+p[1])*y+p[2])*y+p[3])*y+p[4])*y+p[5])
		/(((((q[0]*y+q[1])*y+q[2])*y+q[3])*y+q[4])*y+q[5]);
			y = z*( 0.564189583547756286948 - y);
	}
	return (u < 0.0 ? y : 1-y);
}


/*---------------------------------------------------------------
						dft2_real
 ---------------------------------------------------------------*/
void  math::dft2_real(
	const CMatrixFloat &in_data,
	CMatrixFloat		&out_real,
	CMatrixFloat		&out_imag )
{
	MRPT_START

	size_t				i,j;
	typedef FFT_TYPE*	float_ptr ;

	// The dimensions:
	size_t	dim1 = in_data.getRowCount();
	size_t	dim2 = in_data.getColCount();

	// Transform to format compatible with C routines:
	// ------------------------------------------------------------
	FFT_TYPE	**a;
	FFT_TYPE	*t;
	int			*ip;
	FFT_TYPE	*w;

	// Reserve memory and copy data:
	// --------------------------------------
	a = new float_ptr[dim1];
	for (i=0;i<dim1;i++)
	{
		a[i] = new FFT_TYPE[dim2];
		for (j=0;j<dim2;j++)
			a[i][j] = in_data.get_unsafe(i,j);
	}

	t  = new FFT_TYPE[2*dim1+20];
	ip = new int[(int)ceil(20+2+sqrt((FFT_TYPE)max(dim1,dim2/2)))];
	ip[0] = 0;
	w  = new FFT_TYPE[max(dim1/2,dim2/4)+dim2/4+20];

	// Do the job!
	// --------------------------------------
	rdft2d((int)dim1,(int)dim2,1,a,t,ip,w);

	// Transform back to MRPT matrix format:
	// --------------------------------------
	out_real.setSize(dim1,dim2);
	out_imag.setSize(dim1,dim2);


	// a[k1][2*k2] = R[k1][k2] = R[n1-k1][n2-k2],
	// a[k1][2*k2+1] = I[k1][k2] = -I[n1-k1][n2-k2],
	//       0<k1<n1, 0<k2<n2/2,
	for (i=1;i<dim1;i++)
		for (j=1;j<dim2/2;j++)
		{
			out_real.set_unsafe(i,j,(float)a[i][j*2]); out_real.set_unsafe(dim1-i,dim2-j,(float)a[i][j*2]);
			out_imag.set_unsafe(i,j,(float)-a[i][j*2+1]); out_imag.set_unsafe(dim1-i,dim2-j,(float)a[i][j*2+1]);
		}
	// a[0][2*k2] = R[0][k2] = R[0][n2-k2],
	// a[0][2*k2+1] = I[0][k2] = -I[0][n2-k2],
	//     0<k2<n2/2,
	for (j=1;j<dim2/2;j++)
	{
		out_real.set_unsafe(0,j,(float)a[0][j*2]);   out_real.set_unsafe(0,dim2-j,(float)a[0][j*2]);
		out_imag.set_unsafe(0,j,(float)-a[0][j*2+1]); out_imag.set_unsafe(0,dim2-j,(float)a[0][j*2+1]);
	}

	// a[k1][0] = R[k1][0] = R[n1-k1][0],
	// a[k1][1] = I[k1][0] = -I[n1-k1][0],
    // a[n1-k1][1] = R[k1][n2/2] = R[n1-k1][n2/2],
    // a[n1-k1][0] = -I[k1][n2/2] = I[n1-k1][n2/2],
	//    0<k1<n1/2,
	for (i=1;i<dim1/2;i++)
	{
		out_real.set_unsafe(i,0,(float)a[i][0]); out_real.set_unsafe(dim1-i,0,(float)a[i][0]);
		out_imag.set_unsafe(i,0,(float)-a[i][1]); out_imag.set_unsafe(dim1-i,0,(float)a[i][1]);
		out_real.set_unsafe(i,dim2/2,(float)a[dim1-i][1]);  out_real.set_unsafe(dim1-i,dim2/2,(float)a[dim1-i][1]);
		out_imag.set_unsafe(i,dim2/2,(float)a[dim1-i][0]); out_imag.set_unsafe(dim1-i,dim2/2,(float)-a[dim1-i][0]);
	}

	// a[0][0] = R[0][0],
    // a[0][1] = R[0][n2/2],
    // a[n1/2][0] = R[n1/2][0],
    // a[n1/2][1] = R[n1/2][n2/2]
	out_real.set_unsafe(0,0,(float)a[0][0]);
	out_real.set_unsafe(0,dim2/2,(float)a[0][1]);
	out_real.set_unsafe(dim1/2,0,(float)a[dim1/2][0]);
	out_real.set_unsafe(dim1/2,dim2/2,(float)a[dim1/2][1]);


	// Free temporary memory:
	for (i=0;i<dim1;i++) delete[] a[i];
	delete[] a;
	delete[] t;
	delete[] ip;
	delete[] w;

	MRPT_END
}

/*---------------------------------------------------------------
						idft2_real
 ---------------------------------------------------------------*/
void  math::idft2_real(
	const CMatrixFloat	&in_real,
	const CMatrixFloat	&in_imag,
	CMatrixFloat		&out_data )
{
	MRPT_START

	size_t				i,j;
	typedef FFT_TYPE*		float_ptr ;

	ASSERT_(in_real.getColCount()==in_imag.getColCount());
	ASSERT_(in_real.getRowCount()==in_imag.getRowCount());

	// The dimensions:
	size_t	dim1 = in_real.getRowCount();
	size_t	dim2 = in_real.getColCount();

	if (math::round2up(dim1)!=dim1 || math::round2up(dim2)!=dim2)
		THROW_EXCEPTION("Matrix sizes are not a power of two!")

	// Transform to format compatible with C routines:
	// ------------------------------------------------------------
	FFT_TYPE	**a;
	FFT_TYPE	*t;
	int		*ip;
	FFT_TYPE	*w;

	// Reserve memory and copy data:
	// --------------------------------------
	a = new float_ptr[dim1];
	for (i=0;i<dim1;i++)	a[i] = new FFT_TYPE[dim2];

	//a[j1][2*j2] = R[j1][j2] = R[n1-j1][n2-j2],
    //a[j1][2*j2+1] = I[j1][j2] = -I[n1-j1][n2-j2],
    //    0<j1<n1, 0<j2<n2/2,
	for (i=1;i<dim1;i++)
		for (j=1;j<dim2/2;j++)
		{
			a[i][2*j  ] = in_real.get_unsafe(i,j);
			a[i][2*j+1] = -in_imag.get_unsafe(i,j);
		}

	//a[0][2*j2] = R[0][j2] = R[0][n2-j2],
    //a[0][2*j2+1] = I[0][j2] = -I[0][n2-j2],
    //    0<j2<n2/2,
	for (j=1;j<dim2/2;j++)
	{
		a[0][2*j  ] = in_real.get_unsafe(0,j);
		a[0][2*j+1] = -in_imag.get_unsafe(0,j);
	}

    //a[j1][0] = R[j1][0] = R[n1-j1][0],
    //a[j1][1] = I[j1][0] = -I[n1-j1][0],
    //a[n1-j1][1] = R[j1][n2/2] = R[n1-j1][n2/2],
    //a[n1-j1][0] = -I[j1][n2/2] = I[n1-j1][n2/2],
    //    0<j1<n1/2,
	for (i=1;i<dim1/2;i++)
	{
		a[i][0] = in_real.get_unsafe(i,0);
		a[i][1] = -in_imag.get_unsafe(i,0);
		a[dim1-i][1] = in_real.get_unsafe(i,dim2/2);
		a[dim1-i][0] = in_imag.get_unsafe(i,dim2/2);
	}

    //a[0][0] = R[0][0],
    //a[0][1] = R[0][n2/2],
    //a[n1/2][0] = R[n1/2][0],
    //a[n1/2][1] = R[n1/2][n2/2]
	a[0][0] = in_real.get_unsafe(0,0);
	a[0][1] = in_real.get_unsafe(0,dim2/2);
	a[dim1/2][0] = in_real.get_unsafe(dim1/2,0);
	a[dim1/2][1] = in_real.get_unsafe(dim1/2,dim2/2);

	t  = new FFT_TYPE[2*dim1+20];
	ip = new int[(int)ceil(20+2+sqrt((FFT_TYPE)max(dim1,dim2/2)))];
	ip[0] = 0;
	w  = new FFT_TYPE[max(dim1/2,dim2/4)+dim2/4+20];

	// Do the job!
	// --------------------------------------
	rdft2d((int)dim1,(int)dim2,-1,a,t,ip,w);

	// Transform back to MRPT matrix format:
	// --------------------------------------
	out_data.setSize(dim1,dim2);

	FFT_TYPE	scale = 2.0f / (dim1*dim2);

	for (i=0;i<dim1;i++)
		for (j=0;j<dim2;j++)
			out_data.set_unsafe(i,j,(float)(a[i][j]*scale));

	// Free temporary memory:
	for (i=0;i<dim1;i++) delete[] a[i];
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
void 	myGeneralDFT(
	int									sign,
	const CMatrixFloat &in_real,
	const CMatrixFloat &in_imag,
	CMatrixFloat		&out_real,
	CMatrixFloat		&out_imag )
{
	ASSERT_(in_real.getRowCount() == in_imag.getRowCount() )
	ASSERT_(in_real.getColCount() == in_imag.getColCount() )

	// The dimensions:
	size_t	dim1 = in_real.getRowCount();
	size_t	dim2 = in_real.getColCount();

	size_t	k1,k2,n1,n2;
	float	w_r,w_i;
	float	ang1 = (float)( sign*M_2PI/dim1 );
	float	ang2 = (float)( sign*M_2PI/dim2 );
	float	phase;
	float	R,I;
	float	scale = sign==1 ? (1.0f/(dim1*dim2)):1;

	out_real.setSize(dim1,dim2);
	out_imag.setSize(dim1,dim2);

	for (k1=0;k1<dim1;k1++)
	{
		for (k2=0;k2<dim2;k2++)
		{
			R=I=0;	// Accum:

			for (n1=0;n1<dim1;n1++)
			{
				phase = ang1*n1*k1;
				for (n2=0;n2<dim2;n2++)
				{
					w_r = cos(phase);
					w_i = sin(phase);

					R+= w_r*in_real.get_unsafe(n1,n2) - w_i*in_imag.get_unsafe(n1,n2);
					I+= w_i*in_real.get_unsafe(n1,n2) + w_r*in_imag.get_unsafe(n1,n2);

					phase+= ang2*k2;
				} // end for k2
			} // end for k1

			// Save result:
			out_real.set_unsafe(k1,k2,R*scale);
			out_imag.set_unsafe(k1,k2,I*scale);

		} // end for k2
	} // end for k1
}

/*---------------------------------------------------------------
						dft2_complex
 ---------------------------------------------------------------*/
void  math::dft2_complex(
			const CMatrixFloat &in_real,
			const CMatrixFloat &in_imag,
			CMatrixFloat		&out_real,
			CMatrixFloat		&out_imag)
{
	MRPT_START

	ASSERT_(in_real.getRowCount() == in_imag.getRowCount() )
	ASSERT_(in_real.getColCount() == in_imag.getColCount() )

	// The dimensions:
	size_t	dim1 = in_real.getRowCount();
	size_t	dim2 = in_real.getColCount();
	size_t	i,j;

	bool	dim1IsPower2 = (math::round2up(dim1) == dim1);
	bool	dim2IsPower2 = (math::round2up(dim2) == dim2);

	// FFT or DFT??
	if (dim1IsPower2 && dim2IsPower2)
	{
		// ----------------------------------------
		//			Optimized FFT:
		// ----------------------------------------
		typedef FFT_TYPE*		float_ptr ;

		// Transform to format compatible with C routines:
		// ------------------------------------------------------------
		static FFT_TYPE	**a=NULL;
		static FFT_TYPE	*t= NULL;
		static int	*ip=NULL;
		static FFT_TYPE	*w=NULL;

		// Reserve memory
		// --------------------------------------
		static int	alreadyInitSize1 = -1, alreadyInitSize2 = -1;

		if (alreadyInitSize1!=(int)dim1 || alreadyInitSize2!=(int)dim2)
		{
			// Create/realloc buffers:
			if (a)
			{
				for (i=0;i<dim1;i++) delete[] a[i];
				delete[] a;
			}
			if (ip)	delete[] ip;
			if (t)  delete[] t;
			if (w)  delete[] w;

			alreadyInitSize1 = (int)dim1;
			alreadyInitSize2 = (int)dim2;

			a = new float_ptr[dim1];
			for (i=0;i<dim1;i++) a[i] = new FFT_TYPE[2*dim2];

			t  = new FFT_TYPE[2*dim1+20];
			ip = new int[(int)ceil(20+2+sqrt((FFT_TYPE)max(dim1,dim2/2)))];
			ip[0] = 0;
			w  = new FFT_TYPE[max(dim1/2,dim2/4)+dim2/4+20];
		}

		// and copy data:
		// --------------------------------------
		for (i=0;i<dim1;i++)
			for (j=0;j<dim2;j++)
			{
				a[i][2*j+0] = in_real.get_unsafe(i,j);
				a[i][2*j+1] = in_imag.get_unsafe(i,j);
			}


		// Do the job!
		// --------------------------------------
		cdft2d((int)dim1,(int)(2*dim2),1,a,t,ip,w);

		// Transform back to MRPT matrix format:
		// --------------------------------------
		out_real.setSize(dim1,dim2);
		out_imag.setSize(dim1,dim2);

		// a[k1][2*k2] = Re(X[k1][k2]),
		// a[k1][2*k2+1] = Im(X[k1][k2]),
		// 0<=k1<n1, 0<=k2<n2
		for (i=0;i<dim1;i++)
			for (j=0;j<dim2;j++)
			{
				out_real.set_unsafe(i,j,(float)a[i][j*2+0]);
				out_imag.set_unsafe(i,j,(float)a[i][j*2+1]);
			}

	} // end FFT
	else
	{
		// ----------------------------------------
		//			General DFT:
		// ----------------------------------------
		printf("Using general DFT...\n");
		myGeneralDFT(-1,in_real, in_imag, out_real,out_imag );

	}


	MRPT_END
}

/*---------------------------------------------------------------
						idft2_complex
 ---------------------------------------------------------------*/
void  math::idft2_complex(
			const CMatrixFloat &in_real,
			const CMatrixFloat &in_imag,
			CMatrixFloat		&out_real,
			CMatrixFloat		&out_imag )
{
	MRPT_START

	ASSERT_(in_real.getRowCount() == in_imag.getRowCount() )
	ASSERT_(in_real.getColCount() == in_imag.getColCount() )

	// The dimensions:
	size_t	dim1 = in_real.getRowCount();
	size_t	dim2 = in_real.getColCount();
	size_t	i,j;

	bool	dim1IsPower2 = (math::round2up(dim1) == dim1);
	bool	dim2IsPower2 = (math::round2up(dim2) == dim2);

	// FFT or DFT??
	if (dim1IsPower2 && dim2IsPower2)
	{
		typedef FFT_TYPE*		float_ptr ;
		// ----------------------------------------
		//			Optimized FFT:
		// ----------------------------------------

		// Transform to format compatible with C routines:
		// ------------------------------------------------------------
		static FFT_TYPE	**a=NULL;
		static FFT_TYPE			*t=NULL;
		static int				*ip=NULL;
		static FFT_TYPE		*w=NULL;

		// Reserve memory
		// --------------------------------------

		// and copy data:
		// --------------------------------------
		static int	alreadyInitSize1 = -1, alreadyInitSize2 = -1;

		if (alreadyInitSize1!=(int)dim1 || alreadyInitSize2!=(int)dim2)
		{
			// Create/realloc buffers:
			if (a)
			{
				for (i=0;i<dim1;i++) delete[] a[i];
				delete[] a;
			}
			if (ip)	delete[] ip;
			if (t)  delete[] t;
			if (w)  delete[] w;

			alreadyInitSize1 = (int)dim1;
			alreadyInitSize2 = (int)dim2;

			a = new float_ptr[dim1];
			for (i=0;i<dim1;i++) a[i] = new FFT_TYPE[2*dim2];
			t  = new FFT_TYPE[2*dim1+20];
			ip = new int[(int)ceil(20+2+sqrt((FFT_TYPE)max(dim1,dim2/2)))];
			ip[0] = 0;
			w  = new FFT_TYPE[max(dim1/2,dim2/4)+dim2/4+20];
		}

		// and copy data:
		// --------------------------------------
		for (i=0;i<dim1;i++)
			for (j=0;j<dim2;j++)
			{
				a[i][2*j+0] = in_real.get_unsafe(i,j);
				a[i][2*j+1] = in_imag.get_unsafe(i,j);
			}

		// Do the job!
		// --------------------------------------
		cdft2d((int)dim1,(int)(2*dim2),-1,a,t,ip,w);

		// Transform back to MRPT matrix format:
		// --------------------------------------
		out_real.setSize(dim1,dim2);
		out_imag.setSize(dim1,dim2);

		FFT_TYPE	scale = 1.0f/(dim1*dim2);

		// a[k1][2*k2] = Re(X[k1][k2]),
		// a[k1][2*k2+1] = Im(X[k1][k2]),
		// 0<=k1<n1, 0<=k2<n2
		for (i=0;i<dim1;i++)
			for (j=0;j<dim2;j++)
			{
//				out_real.set_unsafe(i,j,(float)(a[i][j*2+0]*scale));
//				out_imag.set_unsafe(i,j,(float)(a[i][j*2+1]*scale));
				out_real.set_unsafe(i,j,(float)(a[i][j*2+0]));
				out_imag.set_unsafe(i,j,(float)(a[i][j*2+1]));
			}

		out_real *= scale;
		out_imag *= scale;

		// The element (0,0) is purely real!
		out_imag.set_unsafe(0,0,0);

	} // end FFT
	else
	{
		// ----------------------------------------
		//			General DFT:
		// ----------------------------------------
		printf("Using general DFT...\n");
		myGeneralDFT(1,in_real, in_imag, out_real,out_imag );
	}

	MRPT_END
}


/*---------------------------------------------------------------
						isNan
 ---------------------------------------------------------------*/
bool mrpt::math::isNaN(float v) MRPT_NO_THROWS
{
#if defined(__BORLANDC__) || defined(_MSC_VER)
		return 0!=_isnan((double)v);
#elif defined(__GNUC__)
		return isnan(v);
#else
        return false;
#endif
}

/** Returns true if value is Not-a-number (NAN)
  */
bool mrpt::math::isNaN(double v) MRPT_NO_THROWS
{
#if defined(__BORLANDC__) || defined(_MSC_VER)
		return 0!=_isnan(v);
#elif defined(__GNUC__)
		return std::isnan(v);
#else
        return false;
#endif
}

/** Returns true if value is finite
  */
bool mrpt::math::isFinite(float v) MRPT_NO_THROWS
{
#if defined(__BORLANDC__) || defined(_MSC_VER)
		return 0!=_finite(v);
#elif defined(__GNUC__)
		return std::isfinite(v);
#else
        return false;
#endif
}


/** Returns true if value is finite
  */
bool mrpt::math::isFinite(double v) MRPT_NO_THROWS
{
#if defined(__BORLANDC__) || defined(_MSC_VER)
		return 0!=_finite(v);
#elif defined(__GNUC__)
		return isfinite(v);
#else
        return false;
#endif
}


#ifdef HAVE_LONG_DOUBLE
	/*---------------------------------------------------------------
							isnan
	---------------------------------------------------------------*/
	bool  mrpt::math::isNaN(long double f) MRPT_NO_THROWS
	{
#if MRPT_CHECK_VISUALC_VERSION(14) || defined(__GNUC__)
		return std::isnan(f);
#else
		return 0!=_isnan(f);
#endif
	}

	/*---------------------------------------------------------------
							isFinite
	---------------------------------------------------------------*/
	bool  mrpt::math::isFinite(long double f) MRPT_NO_THROWS
	{
#if MRPT_CHECK_VISUALC_VERSION(14) || defined(__GNUC__)
		return std::isfinite(f);
#else
		return 0!=_finite(f);
#endif
	}
#endif


// Loads a vector from a text file:
bool math::loadVector( CFileStream &f, ::std::vector<int> &d)
{
    MRPT_START

    std::string str;
    if (!f.readLine(str)) return false;

    const char *s = str.c_str();

	char *nextTok, *context;
	const char *delim=" \t";

    d.clear();
	nextTok = mrpt::system::strtok( (char*)s,delim,&context);
	while (nextTok != NULL)
	{
		d.push_back( atoi(nextTok) );
		nextTok = mrpt::system::strtok (NULL,delim,&context);
	};

    return true;
    MRPT_END
}

bool math::loadVector( CFileStream &f, ::std::vector<double> &d)
{
    MRPT_START

    std::string str;
    if (!f.readLine(str)) return false;

    const char *s = str.c_str();

	char *nextTok, *context;
	const char *delim=" \t";

    d.clear();
	nextTok = mrpt::system::strtok( (char*)s,delim,&context);
	while (nextTok != NULL)
	{
		d.push_back( atof(nextTok) );
		nextTok = mrpt::system::strtok (NULL,delim,&context);
	};

    return true;
    MRPT_END
}

// See declaration for the documentation
double math::averageLogLikelihood(
	const CVectorDouble &logWeights,
	const CVectorDouble &logLikelihoods )
{
	MRPT_START

	// Explained in:
	// http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability
	ASSERT_( logWeights.size()==logLikelihoods.size() );

	if ( !logWeights.size() )
		THROW_EXCEPTION("ERROR: logWeights vector is empty!")

	CVectorDouble::const_iterator itLW,itLL;
	double lw_max = math::maximum(logWeights);
	double ll_max = math::maximum(logLikelihoods);
	double SUM1=0, SUM2=0, tmpVal;

	for (itLW=logWeights.begin(),itLL=logLikelihoods.begin();itLW!=logWeights.end();itLW++,itLL++)
	{
		tmpVal = *itLW - lw_max;
		SUM1+=std::exp( tmpVal );
		SUM2+=std::exp( tmpVal + *itLL - ll_max );
	}

	double res = -std::log(SUM1) + std::log(SUM2) + ll_max;
	MRPT_CHECK_NORMAL_NUMBER(res);
	return res;
	MRPT_END
}

// Unweighted version:
double math::averageLogLikelihood( const CVectorDouble &logLikelihoods )
{
	MRPT_START

	// Explained in:
	// http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability
	size_t N = logLikelihoods.size();
	if ( !N )
		THROW_EXCEPTION("ERROR: logLikelihoods vector is empty!")

	double ll_max = math::maximum(logLikelihoods);
	double SUM1=0;

	for (size_t i=0;i<N;i++)
        SUM1+=exp( logLikelihoods[i] - ll_max );

    double res = log(SUM1) - log (static_cast<double>(N)) + ll_max;

	MRPT_CHECK_NORMAL_NUMBER(res);
	return res;
	MRPT_END
}

// Wrapped angles average:
double math::averageWrap2Pi(const CVectorDouble &angles )
{
	if (angles.empty()) return 0;

	int 	W_phi_R=0,W_phi_L=0;
	double	phi_R=0,phi_L=0;

	// First: XY
	// -----------------------------------
	for (CVectorDouble::Index i=0;i<angles.size();i++)
	{
		double phi = angles[i];
		if (abs(phi)>1.5707963267948966192313216916398)
		{
			// LEFT HALF: 0,2pi
			if (phi<0) phi = (M_2PI + phi);

			phi_L += phi;
			W_phi_L ++;
		}
		else
		{
			// RIGHT HALF: -pi,pi
			phi_R += phi;
			W_phi_R ++;
		}
	}

	// Next: PHI
	// -----------------------------------
	// The mean value from each side:
	if (W_phi_L)	phi_L /= static_cast<double>(W_phi_L);  // [0,2pi]
	if (W_phi_R)	phi_R /= static_cast<double>(W_phi_R);  // [-pi,pi]

	// Left side to [-pi,pi] again:
	if (phi_L>M_PI) phi_L -= M_2PI;

	// The total mean:
	return ((phi_L * W_phi_L + phi_R * W_phi_R )/(W_phi_L+W_phi_R));
}

// Spline
double math::spline(const double t, const CVectorDouble &x, const CVectorDouble &y, bool wrap2pi)
{
	// Check input data
	ASSERT_( x.size() == 4 && y.size() == 4 );
	ASSERT_( x[0] <= x[1] && x[1] <= x[2] && x[2] <= x[3] );
	ASSERT_( t > x[0] && t < x[3] );
	//ASSERT_( t >= x[1] && t <= x[2] );

	std::vector<double> h;
	h.resize(3);
	for(unsigned int i = 0; i < 3; i++)
		h[i] = x[i+1]-x[i];

	double k	= 1/(4*h[0]*h[1]+4*h[0]*h[2]+3*h[1]*h[1]+4*h[1]*h[2]);
	double a11	= 2*(h[1]+h[2])*k;
	double a12	= -h[1]*k;
	double a22	= 2*(h[0]+h[1])*k;

	double y0,y1,y2,y3;

	if (!wrap2pi)
	{
		y0 = y[0];
		y1 = y[1];
		y2 = y[2];
		y3 = y[3];
	}
	else
	{
		// Assure the function is linear without jumps in the interval:
		y0 = mrpt::math::wrapToPi( y[0] );
		y1 = mrpt::math::wrapToPi( y[1] );
		y2 = mrpt::math::wrapToPi( y[2] );
		y3 = mrpt::math::wrapToPi( y[3] );

		double Ay;

		Ay = y1-y0;
		if (Ay>M_PI)
			y1-=M_2PI;
		else if (Ay<-M_PI)
			y1+=M_2PI;

		Ay = y2-y1;
		if (Ay>M_PI)
			y2-=M_2PI;
		else if (Ay<-M_PI)
			y2+=M_2PI;

		Ay = y3-y2;
		if (Ay>M_PI)
			y3-=M_2PI;
		else if (Ay<-M_PI)
			y3+=M_2PI;
	}

	double b1	= (y2-y1)/h[1]-(y1-y0)/h[0];
	double b2	= (y3-y2)/h[2]-(y2-y1)/h[1];

	double z0	= 0;
	double z1	= 6*(a11*b1 + a12*b2);
	double z2	= 6*(a12*b1 + a22*b2);
	double z3	= 0;

	double res = 0;   // PACO: WARNING: May be used un-initialized!!!!!!!
	if( t < x[1] )
		res = (z1*pow((t-x[0]),3)+z0*pow((x[1]-t),3))/(6*h[0]) + (y1/h[0]-h[0]/6*z1)*(t-x[0]) + (y0/h[0]-h[0]/6*z0)*(x[1]-t);
	else
	{
		if( t < x[2] )
			res = (z2*pow((t-x[1]),3)+z1*pow((x[2]-t),3))/(6*h[1]) + (y2/h[1]-h[1]/6*z2)*(t-x[1]) + (y1/h[1]-h[1]/6*z1)*(x[2]-t);
		else
			if( t < x[3] )
				res = (z3*pow((t-x[2]),3)+z2*pow((x[3]-t),3))/(6*h[2]) + (y3/h[2]-h[2]/6*z3)*(t-x[2]) + (y2/h[2]-h[2]/6*z2)*(x[3]-t);
	}
	return wrap2pi ? mrpt::math::wrapToPi(res) : res;

	/** /
	A   = [2*(h(1)+h(2)) h(2);h(2) 2*(h(2)+h(3))];

	b1  = (pt(3,2) - pt(2,2))/h(2) - (pt(2,2) - pt(1,2))/h(1);
	b2  = (pt(4,2) - pt(3,2))/h(3) - (pt(3,2) - pt(2,2))/h(2);
	B   = [b1;b2];

	z = 6*inv(A)*B;
	z = [0;z;0];

	out = (z(inte+1)*(t-pt(inte,1))^3+z(inte)*(pt(inte+1,1)-t)^3)/(6*h(inte)) + (pt(inte+1,2)/h(inte)-h(inte)/6*z(inte+1))*(t-pt(inte,1)) + (pt(inte,2)/h(inte)-h(inte)/6*z(inte))*(pt(inte+1,1)-t);
	/ **/
}

string math::MATLAB_plotCovariance2D(
	const CMatrixFloat  &cov,
	const CVectorFloat  &mean,
    const float         &stdCount,
	const string        &style,
	const size_t		&nEllipsePoints )
{
	MRPT_START
	CMatrixD        cov2(cov);
	CVectorDouble   mean2(2);
	mean2[0]=mean[0];
	mean2[1]=mean[1];

    return MATLAB_plotCovariance2D( cov2,mean2,stdCount, style,nEllipsePoints );

    MRPT_END
}

string math::MATLAB_plotCovariance2D(
	const CMatrixDouble  &cov,
	const CVectorDouble  &mean,
    const float         &stdCount,
	const string        &style,
	const size_t		&nEllipsePoints )
{
	MRPT_START

	ASSERT_(cov.getColCount()==cov.getRowCount() && cov.getColCount()==2 );
	ASSERT_(cov(0,1)==cov(1,0) );
    ASSERT_(!((cov(0,0)==0) ^ (cov(1,1)==0)) );  // Both or none 0
	ASSERT_(mean.size()==2);

	std::vector<float>				X,Y,COS,SIN;
	std::vector<float>::iterator	x,y,Cos,Sin;
	double							ang;
	CMatrixD						eigVal,eigVec,M;
	string							str;

	X.resize(nEllipsePoints);
	Y.resize(nEllipsePoints);
	COS.resize(nEllipsePoints);
	SIN.resize(nEllipsePoints);

	// Fill the angles:
	for (Cos=COS.begin(),Sin=SIN.begin(),ang=0;Cos!=COS.end();++Cos,++Sin, ang+= (M_2PI/(nEllipsePoints-1)) )
	{
		*Cos = (float)cos(ang);
		*Sin = (float)sin(ang);
	}

	cov.eigenVectors(eigVec,eigVal);
	eigVal = eigVal.array().sqrt().matrix();
	M = eigVal * eigVec.adjoint();

	// Compute the points of the ellipsoid:
	// ----------------------------------------------
	for (x=X.begin(), y=Y.begin(), Cos=COS.begin(),Sin=SIN.begin(); x!=X.end(); ++x,++y,++Cos,++Sin)
	{
		*x = (float)( mean[0] + stdCount * (*Cos * M(0,0) + *Sin * M(1,0)) );
		*y = (float)( mean[1] + stdCount * (*Cos * M(0,1) + *Sin * M(1,1)) );
	}

	// Save the code to plot the ellipsoid:
	// ----------------------------------------------
	str += string("plot([ ");
	for (x=X.begin();x!=X.end();++x)
	{
		str += format("%.4f",*x);
		if (x!=(X.end()-1))	str += format(",");
	}
	str += string("],[ ");
	for (y=Y.begin();y!=Y.end();++y)
	{
		str += format("%.4f",*y);
		if (y!=(Y.end()-1))	str += format(",");
	}

	str += format("],'%s');\n",style.c_str());

	return str;
	MRPT_END_WITH_CLEAN_UP(cerr << "The matrix that led to error was: " << endl << cov << endl; )
}

/*---------------------------------------------------------------
						cross_correlation_FFT
 ---------------------------------------------------------------*/
void  mrpt::math::cross_correlation_FFT(
	const CMatrixFloat	&A,
	const CMatrixFloat	&B,
	CMatrixFloat		&out_corr )
{
	MRPT_START

	ASSERT_( size(A,1)==size(B,1) && size(A,2)==size(B,2) );
	if ( math::round2up( size(A,1) ) != size(A,1)  || math::round2up( size(A,2) ) != size(A,2)  )
		THROW_EXCEPTION("Size of input matrices must be powers of two.");

	// Find smallest valid size:
	size_t		x,y;
	const size_t	lx = size(A,2);
	const size_t	ly = size(A,1);

	const CMatrixFloat  &i1 = A;
	const CMatrixFloat  &i2 = B;

	// FFT:
	CMatrixFloat		I1_R,I1_I,I2_R,I2_I,ZEROS(ly,lx);
	math::dft2_complex(i1,ZEROS,I1_R,I1_I);
	math::dft2_complex(i2,ZEROS,I2_R,I2_I);

	// Compute the COMPLEX division of I2 by I1:
	for (y = 0;y<ly;y++)
		for (x = 0;x<lx;x++)
		{
			float	r1 = I1_R.get_unsafe(y,x);
			float	r2 = I2_R.get_unsafe(y,x);

			float	ii1 = I1_I.get_unsafe(y,x);
			float	ii2 = I2_I.get_unsafe(y,x);

			float	den = square(r1)+square(ii1);
			I2_R.set_unsafe(y,x, (r1*r2+ii1*ii2)/den);
			I2_I.set_unsafe(y,x, (ii2*r1-r2*ii1)/den);
		}

	// IFFT:
	CMatrixFloat	res_R,res_I;
	math::idft2_complex(I2_R,I2_I,res_R,res_I);

	out_corr.setSize(ly,lx);
	for (y = 0;y<ly;y++)
		for (x = 0;x<lx;x++)
			out_corr(y,x) = sqrt( square(res_R(y,x)) + square(res_I(y,x)) );

	MRPT_END
}


double mrpt::math::interpolate2points(const double x, const double x0, const double y0, const double x1, const double y1, bool wrap2pi )
{
	MRPT_START
	if (x0==x1) THROW_EXCEPTION_CUSTOM_MSG1("ERROR: Both x0 and x1 are equal (=%f)",x0);

	const double Ax = x1-x0;
	const double Ay = y1-y0;

	double r = y0+Ay*(x-x0)/Ax;
	if (!wrap2pi)
	     return r;
	else return mrpt::math::wrapToPi(r);

	MRPT_END
}

/*---------------------------------------------------------------
                    median filter of a vector
 ---------------------------------------------------------------*/
//template<typename VECTOR>
void mrpt::math::medianFilter( const std::vector<double> &inV, std::vector<double> &outV, const int &_winSize, const int &numberOfSigmas )
{
	MRPT_UNUSED_PARAM(numberOfSigmas);
    ASSERT_( (int)inV.size() >= _winSize );
    ASSERT_( _winSize >= 2 );                    // The minimum window size is 3 elements
    size_t winSize = _winSize;

    if( !(winSize%2) )                            // We use an odd number of elements for the window size
        winSize++;

    size_t sz = inV.size();
    outV.resize( sz );

    std::vector<double> aux(winSize);
    size_t mpoint = winSize/2;
    for( size_t k = 0; k < sz; ++k )
    {
        aux.clear();

        size_t idx_to_start    = std::max( size_t(0), k-mpoint );                                   // Dealing with the boundaries
        size_t n_elements      = std::min(std::min( winSize, sz+mpoint-k), k+mpoint+1 );

        aux.resize(n_elements);
        for(size_t m = idx_to_start, n = 0; m < idx_to_start+n_elements; ++m, ++n )
            aux[n] = inV[m];

        std::sort( aux.begin(), aux.end() );

        size_t  auxSz       = aux.size();
        size_t auxMPoint   = auxSz/2;
        outV[k] = (auxSz%2) ? (aux[auxMPoint]) : (0.5*(aux[auxMPoint-1]+aux[auxMPoint]));     // If the window is even, take the mean value of the middle points
    } // end-for
} // end medianFilter


double mrpt::math::chi2CDF(unsigned int degreesOfFreedom, double arg)
{
	return noncentralChi2CDF(degreesOfFreedom, 0.0, arg);
}

template <class T>
void noncentralChi2OneIteration(T arg, T & lans, T & dans, T & pans, unsigned int & j)
{
	double tol = -50.0;
	if(lans < tol)
	{
		lans = lans + std::log(arg / j);
		dans = std::exp(lans);
	}
	else
	{
		dans = dans * arg / j;
	}
	pans = pans - dans;
	j += 2;
}

std::pair<double, double> mrpt::math::noncentralChi2PDF_CDF(unsigned int degreesOfFreedom, double noncentrality, double arg, double eps)
{
	ASSERTMSG_(noncentrality >= 0.0 && arg >= 0.0 && eps > 0.0,"noncentralChi2PDF_CDF(): parameters must be positive.")

	if (arg == 0.0 && degreesOfFreedom > 0)
		return std::make_pair(0.0, 0.0);

	// Determine initial values
	double b1 = 0.5 * noncentrality,
		   ao = std::exp(-b1),
		   eps2 = eps / ao,
		   lnrtpi2 = 0.22579135264473,
		   probability, density, lans, dans, pans, sum, am, hold;
	unsigned int maxit = 500,
		i, m;
	if(degreesOfFreedom % 2)
	{
		i = 1;
		lans = -0.5 * (arg + std::log(arg)) - lnrtpi2;
		dans = std::exp(lans);
		pans = mrpt::math::erf(std::sqrt(arg/2.0));
	}
	else
	{
		i = 2;
		lans = -0.5 * arg;
		dans = std::exp(lans);
		pans = 1.0 - dans;
	}

	// Evaluate first term
	if(degreesOfFreedom == 0)
	{
		m = 1;
		degreesOfFreedom = 2;
		am = b1;
		sum = 1.0 / ao - 1.0 - am;
		density = am * dans;
		probability = 1.0 + am * pans;
	}
	else
	{
		m = 0;
		degreesOfFreedom = degreesOfFreedom - 1;
		am = 1.0;
		sum = 1.0 / ao - 1.0;
		while(i < degreesOfFreedom)
			noncentralChi2OneIteration(arg, lans, dans, pans, i);
		degreesOfFreedom = degreesOfFreedom + 1;
		density = dans;
		probability = pans;
	}
	// Evaluate successive terms of the expansion
	for(++m; m<maxit; ++m)
	{
		am = b1 * am / m;
		noncentralChi2OneIteration(arg, lans, dans, pans, degreesOfFreedom);
		sum = sum - am;
		density = density + am * dans;
		hold = am * pans;
		probability = probability + hold;
		if((pans * sum < eps2) && (hold < eps2))
			break; // converged
	}
	if(m == maxit)
		THROW_EXCEPTION("noncentralChi2PDF_CDF(): no convergence.");
	return std::make_pair(0.5 * ao * density, std::min(1.0, std::max(0.0, ao * probability)));
}

double mrpt::math::chi2PDF(unsigned int degreesOfFreedom, double arg, double accuracy)
{
	return mrpt::math::noncentralChi2PDF_CDF(degreesOfFreedom, 0.0, arg, accuracy).first;
}

double mrpt::math::noncentralChi2CDF(unsigned int degreesOfFreedom, double noncentrality, double arg)
{
	const double a = degreesOfFreedom + noncentrality;
	const double b = (a + noncentrality) / square(a);
	const double t = (std::pow((double)arg / a, 1.0/3.0) - (1.0 - 2.0 / 9.0 * b)) / std::sqrt(2.0 / 9.0 * b);
	return 0.5*(1.0 + mrpt::math::erf(t/std::sqrt(2.0)));
}
