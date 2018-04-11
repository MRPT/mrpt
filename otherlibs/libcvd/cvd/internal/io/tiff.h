#ifndef PNM_TIFF
#define PNM_TIFF

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cvd/image.h>
#include <cvd/internal/load_and_save.h>

namespace CVD
{
namespace TIFF
{

	using CVD::Internal::TypeList;
	using CVD::Internal::Head;


	////////////////////////////////////////////////////////////////////////////////
	//
	// TIFF reading
	//

	class TIFFPimpl;
	class tiff_reader
	{
		public:
			tiff_reader(std::istream&);
			~tiff_reader();

			ImageRef size();
			bool top_row_first();

			void get_raw_pixel_line(bool*);
			void get_raw_pixel_line(unsigned char*);
			void get_raw_pixel_line(unsigned short*);
			void get_raw_pixel_line(float*);
			void get_raw_pixel_line(double*);

			void get_raw_pixel_line(Rgb<unsigned char>*);
			void get_raw_pixel_line(Rgb<unsigned short>*);
			void get_raw_pixel_line(Rgb<float>*);
			void get_raw_pixel_line(Rgb<double>*);

			void get_raw_pixel_line(Rgba<unsigned char>*);
			void get_raw_pixel_line(Rgba<unsigned short>*);
			void get_raw_pixel_line(Rgba<float>*);
			void get_raw_pixel_line(Rgba<double>*);

			std::string datatype();
			std::string name();


			typedef TypeList<bool, 
					TypeList<byte, 
					TypeList<unsigned short,
					TypeList<float,
					TypeList<double,
					TypeList<Rgb<byte>, 
					TypeList<Rgb<unsigned short>, 
					TypeList<Rgb<float>, 
					TypeList<Rgb<double>, 
					TypeList<Rgba<byte>, 
					TypeList<Rgba<unsigned short>, 
					TypeList<Rgba<float>, 
					TypeList<Rgba<double>, 
					                      Head> > > > > > > > > > > > > Types;
		
		private:
			std::unique_ptr<TIFFPimpl> t; 
	};


	////////////////////////////////////////////////////////////////////////////////
	//
	// TIFF writing
	//

	//This is a really nasty way of doing pattern matching for ranges of numbers.
	//The range is encoded un unary notation. The range is on some integer, x.
	//g1 is set if x > 1. g8 is set if x > 8 and so on.
	//This allows us to choose a type with a reasonable number of bits.
	template<int g1, int g8> struct IntMapper    { typedef unsigned short type;};
	template<>               struct IntMapper<1, 0> { typedef unsigned char type; };
	template<>               struct IntMapper<0, 0> { typedef bool type; };


	//Mapping for integral types
	template<class ComponentIn, int is_integral> struct ComponentMapper_
	{
		typedef typename IntMapper<
									(Pixel::traits<ComponentIn>::bits_used > 1),
									(Pixel::traits<ComponentIn>::bits_used > 8)
									>::type type;
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


	
	class TIFFWritePimpl;

	class tiff_writer
	{
		public:
			tiff_writer(std::ostream&, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >& p);
			~tiff_writer();

			void write_raw_pixel_line(const bool*);
			void write_raw_pixel_line(const unsigned char*);
			void write_raw_pixel_line(const unsigned short*);
			void write_raw_pixel_line(const float*);
			void write_raw_pixel_line(const double*);

			void write_raw_pixel_line(const Rgb<unsigned char>*);
			void write_raw_pixel_line(const Rgb<unsigned short>*);
			void write_raw_pixel_line(const Rgb<float>*);
			void write_raw_pixel_line(const Rgb<double>*);

			void write_raw_pixel_line(const Rgba<unsigned char>*);
			void write_raw_pixel_line(const Rgba<unsigned short>*);
			void write_raw_pixel_line(const Rgba<float>*);
			void write_raw_pixel_line(const Rgba<double>*);

			template<class Incoming> struct Outgoing
			{		
				typedef typename ComponentMapper<Incoming>::type type;
			};		

			static const int top_row_first=1;
		private:
			std::unique_ptr<TIFFWritePimpl> t; 
	};
	

}
}
#endif
