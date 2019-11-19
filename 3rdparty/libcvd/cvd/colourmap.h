#ifndef CVD_INC_COLOURMAPS_H
#define CVD_INC_COLOURMAPS_H

#include <cvd/rgb.h>
#include <cvd/rgba.h>
#include <cvd/internal/convert_pixel_types.h>
#include <cvd/internal/rgb_components.h>

#include <algorithm>

namespace CVD
{
	
	namespace Internal
	{
		Rgb<float> grey(float d)
		{
			using std::max; 
			using std::min;
			d = max(0.f, min(d, 1.0f));
			return Rgb<float>(d, d, d);
		}
			
		Rgb<float> hot(float d)
		{
			using std::max; 
			using std::min;
			d = max(0.f, min(d, 1.0f));

			if(d < 1./3.)
				return Rgb<float>(d*3, 0, 0);
			else if(d < 2./3.)
				return Rgb<float>(1, (d-1./3.)*3, 0);
			else
				return Rgb<float>(1, 1, (d-2./3.)*3);
		}
		
		Rgb<float> jet(float d)
		{
			using std::max; 
			using std::min;
			d = max(0.f, min(d, 1.0f));
			double r=0,g=0,b=0;

			if(d < 1./4.) // Red to yello
			{
				r=1;
				g=d/(1./4.);
				b=0;
			}
			else if(d < 2./4.) //Yello to green
			{
				g=1;
				r=1-(d-1./4.)/(1./4.);
				b=0;
			}	
			else if(d < 3./4.) // Green to cyan
			{
				r=0;
				g=1;
				b=(d-2./4.)/(1./4.);
			}
			else //cyan to blue
			{
				b=1;
				g = 1- (d-3./4.)/(1./4.);
				r=0;
			}

			return Rgb<float>(r, g, b);
		}

		Rgb<float> gkr(float d)
		{
			using std::max; 
			using std::min;
			
			if(d < 1./2.)
				return Rgb<float>(1-d*2, 0, 0);
			else
				return Rgb<float>(0, (d-1./2.)*2, 0);


		}


		template<class C, class D>
		Rgb<C> conv(const D& func, float d)
		{
			Rgb<float> col = func(d);
			Rgb<C> r;
			Pixel::ConvertPixels<Rgb<float>, Rgb<C> >::convert(&col, &r, 1);
			return r;
		}
	};


	template<class C> struct Colourmap;

	///Handy class for generating a colourscale.
	///Expected range is \f$[0, 1)\f$, and clamping is performed.
	///Currently only RGB scales are provided. TODO: RGBA scales.
	///@ingroup gGraphics
	///Examples in examples/colourmaps.cc
	///@include colourmaps.cc
	template<class C> struct Colourmap<Rgb<C> >
	{
		
		///Glow/Hot colourscale (red-yellow-white)
		///@param d Value in \f$[0, 1)\f$ to map
		static Rgb<C> hot(double d) { return Internal::conv<C>(Internal::hot, d);} 

		///Jet colourscale (red-yellow-green-cyan-blue)
		///@param d Value in \f$[0, 1)\f$ to map
		static Rgb<C> jet(double d) { return Internal::conv<C>(Internal::jet, d);} 

		///Green-black-red colourscale
		///@param d Value in \f$[0, 1)\f$ to map
		static Rgb<C> gkr(double d) { return Internal::conv<C>(Internal::gkr, d);} 
		///Gray colourscale
		///@param d Value in \f$[0, 1)\f$ to map
		static Rgb<C> grey(double d) { return Internal::conv<C>(Internal::grey, d);} 
	};
	

	///@example colourmaps.cc
	///Example of how to use the CVD::Colourmap class for visualisation.


};


#endif
