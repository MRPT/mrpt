#ifndef CVD_INCLUDE_INTERNAL_IO_TEXT_H
#define CVD_INCLUDE_INTERNAL_IO_TEXT_H

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cvd/image.h>
#include <cvd/internal/load_and_save.h>

namespace CVD
{
namespace TEXT
{

	using CVD::Internal::TypeList;
	using CVD::Internal::Head;

	class ReadPimpl;
	class reader
	{
		public:
			reader(std::istream&);
			~reader();

			ImageRef size();
			bool top_row_first();

			void get_raw_pixel_line(double*);

			std::string datatype();
			std::string name();


			typedef TypeList<double,  Head> Types;
		
		private:
			std::unique_ptr<ReadPimpl> t; 
	};


	////////////////////////////////////////////////////////////////////////////////
	//
	// TEXT writing
	//
	class WritePimpl;

	class writer
	{
		public:
			writer(std::ostream&, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >& p);
			~writer();

			void write_raw_pixel_line(const double*);
			void write_raw_pixel_line(const float*);

			template<class Incoming> struct Outgoing
			{		
				typedef double type;
			};		

			static const int top_row_first=1;
		private:
			std::unique_ptr<WritePimpl> t; 
	};
	
}
}
#endif
