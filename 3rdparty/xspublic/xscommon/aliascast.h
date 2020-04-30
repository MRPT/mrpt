
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef ALIASCAST_H
#define ALIASCAST_H

#ifdef __cplusplus
#ifndef XSENS_MATH_FIRMWARE

#include <type_traits>

namespace Internal
{
	/*! \class AliasCaster
		Alias caster can be used to reinterpret a memory locatation as a different type without breaking strict aliasing rules.
		For example casting a pointer to a struct to a pointer to a uint16_t.
		AliasCaster only supports casting pointers.
		Example:
		MyStruct myStruct;
		uint16_t* myUint16ptr = AliasCaster<uint16_t*, MyStruct>(&myStruct);
		uint16_t firstUint = *myUint16ptr;
		The helper function alias_cast (see below) is provided for easier usage
		\note Strictly speaking, according to the standard, assigning to alias.from and the reading alias.to is undefined behaviour
		\note Putting only pointers in the union an invalid solution according to the link below
		http://cellperformance.beyond3d.com/articles/2006/06/understanding-strict-aliasing.html
		http://blog.regehr.org/archives/959

		The chosen solution is to cast the source pointer to a pointer to a union of the 'base' types of the source and destination pointers
		and then return the adress of the destination field of the union.
	*/
	template <typename U, typename V>
	class AliasCaster {
	public:
		typedef U to_type; //!< This is a pointer to type
		typedef V from_type; //!< This is a pointer from type
		typedef typename std::remove_pointer<to_type>::type base_to_type; //!< This is a base pointer to type
		typedef typename std::remove_pointer<from_type>::type base_from_type; //!< This is a base pointer from type

		static_assert(std::is_pointer<to_type>::value && std::is_pointer<from_type>::value, "alias_caster requires pointer types as arguments");
		static_assert(!std::is_const<base_from_type>::value || (std::is_const<base_from_type>::value && std::is_const<base_to_type>::value), "alias_caster requires a const destination type here");

		static to_type cast(from_type from) {return &(reinterpret_cast<AliasType*>(from)->to);} //!< This is a pointer type cast
	private:
		// The union to cast the from_type to
		typedef union  {
			base_from_type from;
			base_to_type to;
		} AliasUnion;

		// Depending on the const-ness of the from_type, we need to cast to a const union or not
		typedef typename std::conditional<
			std::is_const<base_from_type>::value,
			typename std::add_const<AliasUnion>::type,
			AliasUnion
		>::type AliasType;
	};
} // namespace Internal

/*! Casts one pointer to another without breaking strict aliasing rules.
	\see Internal::AliasCaster
	Example:
	MyStruct myStruct;
	uint16_t* myUint16ptr = alias_cast<uint16_t*>(&myStruct);
	uint16_t firstUint = *myUint16ptr;
*/
template <typename U, typename V>
U alias_cast(V fromPtr) {
	return Internal::AliasCaster<U, V>::cast(fromPtr);
}

#endif // XSENS_MATH_FIRMWARE

#endif // __cplusplus

#endif // ALIASCAST_H