#ifndef PNM_SAVE_POSTSCRIPT_H
#define PNM_SAVE_POSTSCRIPT_H

#include <iostream>
#include <string>
#include <memory>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/internal/convert_pixel_types.h>
#include <cvd/internal/io/parameter.h>

namespace CVD
{
namespace PS
{

	class WritePimpl;


	class writer
	{
		public:
			writer(std::ostream&, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >& p);
			~writer();

			void write_raw_pixel_line(const byte*);
			void write_raw_pixel_line(const Rgb<byte>*);

			template<class Incoming> struct Outgoing
			{		
				typedef byte type;
			};		

			static const int top_row_first=0;
		protected:
			std::unique_ptr<WritePimpl> t; 
	};

	template<class C> struct writer::Outgoing<Rgb<C> > 
	{
		typedef Rgb<byte> type;
	};


	template<class C> struct writer::Outgoing<Rgba<C> > 
	{
		typedef Rgb<byte> type;
	};

	template<> struct writer::Outgoing<Rgb8> 
	{
		typedef Rgb<byte> type;
	};

		
	class eps_writer
	{
		public:
			eps_writer(std::ostream&, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >& p);
			~eps_writer();

			void write_raw_pixel_line(const byte*);
			void write_raw_pixel_line(const Rgb<byte>*);

			template<class Incoming> struct Outgoing
			{		
				typedef typename writer::Outgoing<Incoming>::type type;
			};		

			static const int top_row_first=1;
		protected:
			std::unique_ptr<WritePimpl> t; 
	};

}
}
#endif

