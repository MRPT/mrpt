#ifndef PNM_NAME_CVD_RGB_TYPES_H
#define PNM_NAME_CVD_RGB_TYPES_H

#include <string>
#include <cvd/internal/name_builtin_types.h>

namespace CVD
{
namespace PNM
{

	template<> struct  type_name<CVD::Rgb8>{static const std::string name(){return "CVD::Rgb8";}};
	template<class O> struct type_name<CVD::Rgb<O> >
	{
		static const std::string name(){return "CVD::Rgb<" + type_name<O>::name() + ">";}
	};
	template<class O> struct type_name<CVD::Rgba<O> >
	{
		static const std::string name(){return "CVD::Rgba<" + type_name<O>::name() + ">";}
	};
	template<class O> struct type_name<CVD::Bgrx<O> >
	{
		static const std::string name(){return "CVD::Bgrx<" + type_name<O>::name() + ">";}
	};
}
}

#endif

