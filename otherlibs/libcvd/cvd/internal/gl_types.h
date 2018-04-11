// Pulled out of gl_helpers to make documentation neater
// Paul Smith 20/4/05

#ifndef CVD_GL_TYPES_H
#define CVD_GL_TYPES_H

#include <cvd/internal/gles1_types.h>

namespace CVD
{
	
	namespace gl
	{
		template<class C> struct data;

		//Scalar types
		template<> struct data<unsigned int>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_UNSIGNED_INT;
		};

		template<> struct data<int>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_INT;
		};

		template<> struct data<double>
		{
			static const int format=GL_LUMINANCE;
			static const int type  =GL_DOUBLE;
		};

		//Rgb<*> types
	
		template<> struct data<Rgb<unsigned int> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_UNSIGNED_INT;
		};

		template<> struct data<Rgb<int> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_INT;
		};

		template<> struct data<Rgb<double> >
		{
			static const int format=GL_RGB;
			static const int type  =GL_DOUBLE;
		};

		//Rgba<*> types

		template<> struct data<Rgba<unsigned int> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_UNSIGNED_INT;
		};

		template<> struct data<Rgba<int> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_INT;
		};

		template<> struct data<Rgba<double> >
		{
			static const int format=GL_RGBA;
			static const int type  =GL_DOUBLE;
		};

		//La<*> types

		template<> struct data<La<unsigned int> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_UNSIGNED_INT;
		};

		template<> struct data<La<int> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_INT;
		};

		template<> struct data<La<double> >
		{
			static const int format=GL_LUMINANCE_ALPHA;
			static const int type  =GL_DOUBLE;
		};

	};

};

#endif
