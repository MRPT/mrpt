// Pulled out of gl_helpers to make documentation neater
// Paul Smith 20/4/05

#ifndef CVD_GLES1_TYPES_H
#define CVD_GLES1_TYPES_H

#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/rgb8.h>
#include <cvd/rgba.h>
#include <cvd/la.h>

namespace CVD
{
	
	namespace gl
	{
		template<class C> struct data;

		//Scalar types

		template<> struct data<unsigned char>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_UNSIGNED_BYTE;
		};

		template<> struct data<signed char>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_BYTE;
		};
		
		template<> struct data<unsigned  short>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_UNSIGNED_SHORT;
		};

		template<> struct data<short>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_SHORT;
		};

		template<> struct data<float>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_FLOAT;
		};

		//Rgb<*> types
	
		template<> struct data<Rgb<unsigned char> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_UNSIGNED_BYTE;
		};

		template<> struct data<Rgb<signed char> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_BYTE;
		};
		
		template<> struct data<Rgb<unsigned  short> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_UNSIGNED_SHORT;
		};

		template<> struct data<Rgb<short> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_SHORT;
		};

		template<> struct data<Rgb<float> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_FLOAT;
		};

		//Rgba<*> types

		template<> struct data<Rgba<unsigned char> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_UNSIGNED_BYTE;
		};

		template<> struct data<Rgba<signed char> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_BYTE;
		};
		
		template<> struct data<Rgba<unsigned  short> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_UNSIGNED_SHORT;
		};

		template<> struct data<Rgba<short> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_SHORT;
		};

		template<> struct data<Rgba<float> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_FLOAT;
		};

		//La<*> types
		template<> struct data<La<unsigned char> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_UNSIGNED_BYTE;
		};

		template<> struct data<La<signed char> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_BYTE;
		};
		
		template<> struct data<La<unsigned  short> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_UNSIGNED_SHORT;
		};

		template<> struct data<La<short> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_SHORT;
		};
		
		template<> struct data<La<float> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_FLOAT;
		};

		//Rgb8 type

		template<> struct data<Rgb8>
		{
			static const int format=GL_RGBA;
			static const int type  =GL_UNSIGNED_BYTE;
		};
		
	};

};

#endif
