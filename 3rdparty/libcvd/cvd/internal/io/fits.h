#ifndef CVD_INCLUDE_INTERNAL_IO_FITS_H
#define CVD_INCLUDE_INTERNAL_IO_FITS_H

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cvd/image.h>
#include <cvd/internal/load_and_save.h>

namespace CVD
{
namespace FITS
{

	using CVD::Internal::TypeList;
	using CVD::Internal::Head;

	//Probably not the best way to deal with FITS. Data is unlikely to be RGBA, for
	//instance. Maybe it would be best to implement loading of tr1::array and Vector.

	////////////////////////////////////////////////////////////////////////////////
	//
	// FITS reading
	//

	class ReadPimpl;
	class reader
	{
		public:
			reader(std::istream&);
			~reader();

			ImageRef size();
			bool top_row_first();

			void get_raw_pixel_line(unsigned char*);
			void get_raw_pixel_line(signed short*);
			void get_raw_pixel_line(unsigned short*);
			void get_raw_pixel_line(signed int*);
			void get_raw_pixel_line(float*);
			void get_raw_pixel_line(double*);

			void get_raw_pixel_line(Rgb<unsigned char>*);
			void get_raw_pixel_line(Rgb<signed short>*);
			void get_raw_pixel_line(Rgb<unsigned short>*);
			void get_raw_pixel_line(Rgb<signed int>*);
			void get_raw_pixel_line(Rgb<float>*);
			void get_raw_pixel_line(Rgb<double>*);

			void get_raw_pixel_line(Rgba<unsigned char>*);
			void get_raw_pixel_line(Rgba<signed short>*);
			void get_raw_pixel_line(Rgba<unsigned short>*);
			void get_raw_pixel_line(Rgba<signed int>*);
			void get_raw_pixel_line(Rgba<float>*);
			void get_raw_pixel_line(Rgba<double>*);

			std::string datatype();
			std::string name();


			typedef TypeList<byte, 
					TypeList<signed short,
					TypeList<unsigned short,
					TypeList<signed int,
					TypeList<float,
					TypeList<double,
					TypeList<Rgb<byte>, 
					TypeList<Rgb<signed short>, 
					TypeList<Rgb<unsigned short>, 
					TypeList<Rgb<signed int>, 
					TypeList<Rgb<float>, 
					TypeList<Rgb<double>, 
					TypeList<Rgba<byte>, 
					TypeList<Rgba<signed short>, 
					TypeList<Rgba<unsigned short>, 
					TypeList<Rgba<signed int>, 
					TypeList<Rgba<float>, 
					TypeList<Rgba<double>, 
					                      Head> > > > > > > > > > > > > > > > > > Types;
		
		private:
			std::unique_ptr<ReadPimpl> t; 
	};

	
	////////////////////////////////////////////////////////////////////////////////
	//
	// FITS writing, copied and modified from tiff.h
	//

	template<typename C>     struct IntMapper                   { typedef int type;};
	template<>               struct IntMapper<bool>             { typedef unsigned char type; };
	template<>               struct IntMapper<char>             { typedef short type; };
	template<>               struct IntMapper<unsigned char>    { typedef unsigned char type; };
	template<>               struct IntMapper<short>            { typedef short type; };
	template<>               struct IntMapper<unsigned short>   { typedef unsigned short type; };
	template<>               struct IntMapper<int>              { typedef int type; };

	//Mapping for integral types
	template<class ComponentIn, int is_integral> struct ComponentMapper_
	{
		typedef typename IntMapper<ComponentIn>::type type;
	};

	//Mapping for non integral types
	template<class ComponentIn> struct ComponentMapper_<ComponentIn, 0> { typedef double type; };
	template<> struct ComponentMapper_<float, 0> { typedef float type; };
	
	template<class ComponentIn> struct ComponentMapper
	{
		typedef typename ComponentMapper_<ComponentIn, Pixel::traits<ComponentIn>::integral>::type type;
	};
	
	//Mapping for Rgbish types
	template<class ComponentIn> struct ComponentMapper<Rgb<ComponentIn> >
	{
		typedef Rgb<typename ComponentMapper_<ComponentIn, Pixel::traits<ComponentIn>::integral>::type> type;
	};

	template<class ComponentIn> struct ComponentMapper<Rgba<ComponentIn> >
	{
		typedef Rgba<typename ComponentMapper_<ComponentIn, Pixel::traits<ComponentIn>::integral>::type> type;
	};

	template<> struct ComponentMapper<Rgb8>
	{
		typedef Rgb<byte> type;
	};



	
	class WritePimpl;

	class writer
	{
		public:
			writer(std::ostream&, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >& p);
			~writer();

			void write_raw_pixel_line(const unsigned char*);
			void write_raw_pixel_line(const short*);
			void write_raw_pixel_line(const unsigned short*);
			void write_raw_pixel_line(const int*);
			void write_raw_pixel_line(const float*);
			void write_raw_pixel_line(const double*);

			void write_raw_pixel_line(const Rgb<unsigned char>*);
			void write_raw_pixel_line(const Rgb<short>*);
			void write_raw_pixel_line(const Rgb<unsigned short>*);
			void write_raw_pixel_line(const Rgb<int>*);
			void write_raw_pixel_line(const Rgb<float>*);
			void write_raw_pixel_line(const Rgb<double>*);

			void write_raw_pixel_line(const Rgba<unsigned char>*);
			void write_raw_pixel_line(const Rgba<short>*);
			void write_raw_pixel_line(const Rgba<unsigned short>*);
			void write_raw_pixel_line(const Rgba<int>*);
			void write_raw_pixel_line(const Rgba<float>*);
			void write_raw_pixel_line(const Rgba<double>*);

			template<class Incoming> struct Outgoing
			{		
				typedef typename ComponentMapper<Incoming>::type type;
			};		

			static const int top_row_first=1;
		private:
			WritePimpl* t; 
	};
	
}
}
#endif
