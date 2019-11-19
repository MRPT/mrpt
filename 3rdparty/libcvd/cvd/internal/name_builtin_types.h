#ifndef PNM_NAME_BUILTIN_TYPES_H
#define PNM_NAME_BUILTIN_TYPES_H

#include <string>

namespace CVD
{
namespace PNM
{
	template<class T> struct type_name{ static const std::string name(){ return"unknown"; }};
	template<> struct  type_name<bool>{static const std::string name(){ return "bool";}};
	template<> struct  type_name<char>{static const std::string name(){ return "char";}};
	template<> struct  type_name<signed char>{static const std::string name(){ return "signed char";}};
	template<> struct  type_name<unsigned char>{static const std::string name(){ return "unsigned char";}};
	template<> struct  type_name<short>{static const std::string name(){ return "short";}};
	template<> struct  type_name<unsigned short>{static const std::string name(){ return "unsigned short";}};
	template<> struct  type_name<int>{static const std::string name(){ return "int";}};
	template<> struct  type_name<unsigned int>{static const std::string name(){ return "unsigned int";}};
	template<> struct  type_name<long>{static const std::string name(){ return "long";}};
	template<> struct  type_name<unsigned long>{static const std::string name(){ return "unsigned long";}};
	template<> struct  type_name<long long>{static const std::string name(){ return "long long";}};
	template<> struct  type_name<unsigned long long>{static const std::string name(){ return "unsigned long long";}};
	template<> struct  type_name<float>{static const std::string name(){ return "float";}};
	template<> struct  type_name<double>{static const std::string name(){ return "double";}};
	template<> struct  type_name<long double>{static const std::string name(){ return "long double";}};
}
}


#endif
