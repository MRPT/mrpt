/*                       
	This file is part of the CVD Library.

	Copyright (C) 2005 The Authors

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc.,
	51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <cvd/colourspace.h>
#include <cvd/config.h>

namespace CVD
{
namespace ColourSpace
{


void yuv411_to_rgb_y(const unsigned char* in, int size, unsigned char* out, unsigned char *lum_out)
{
	//Time: 5.0ms memcpy on number of luma pixels only takes 1.5 ms

	const unsigned char* in_end = in + size * 6 / 4;

	size /=8;
	
	__asm__ __volatile__(
		//Put some zeros in mm7
		"mov		$0, %%eax		\n\t"
		"movd		%%eax, %%mm7	\n\t"
	
		//Load out and in
		"mov		%0, %%edi		\n\t"
		"mov		%1, %%esi		\n\t"
		"push		%%ebx			\n\t" //save ebx. For some reason, the compiler won't do this
		"push		%%ebp			\n\t" //save the stack base
		"push		%[end]			\n\t"
		"mov		%[luma], %%ebp	\n\t"

".Lfoo:								\n\t"

		"prefetchnta	128(%%esi)	\n\t"

		/*Convert yuv411 to rgb8	
		format is u1 y1 y2 v1 y3 y4
		u, v are int8 - 128
		y	   are uint8

		For YUV in general:
		r = y + 1.140 * v			= y + rv
		g = y - 0.394*u - 0.581*v	= y + guv
		b = y + 2.028 * u			= y + bu

		Arithmetic is performed using fixed point with a left shift of 7 (ie *128)
		u1, v1 are not interpolated between y pixels, ie u1,v1 are used for y1,y2,y3,y4
		b is truncated slightly, ie b= y + 2.0 * u

		The resultant 8 bit r,g,b values need to be saturated, since overflow occurs
		and this leads to very ugly results.
		*/
		
		////////////////////////////////////////////////////////////
		//
		//	Load and convert the chrominance values to mmxable things
		//

		//Create the register mm0 = * bu guv rv
		//Let ecx, edx contain (u-128), (v-128)

		//Note the following 4 instructions are in a different order in the second 
		//block, since it's faster to have them different (!)

		"movzbl 3(%%esi), %%edx		\n\t"	//dx = v   move-(with-zero-extend)-byte-(in to)-long 
		"sub	$128, %%edx			\n\t"
		"movzbl	(%%esi), %%ecx		\n\t"	//cx = u
		"sub	$128, %%ecx			\n\t"	
		
		"mov	%%edx, %%eax		\n\t"	//Calculate and insert rv = v * 147
		"imul	$147, %%eax			\n\t"
		"pinsrw	$0, %%eax, %%mm0	\n\t"

		"mov 	%%ecx, %%ebx		\n\t"	//Calculate and insert bu = v * 256
		"imul	$256, %%ebx			\n\t"
		"pinsrw $2, %%ebx, %%mm0	\n\t"

		"imul	$-38, %%ecx			\n\t"	//Calculate and insert guv = -38u + -74v  GKMOD
		"imul	$-74, %%edx			\n\t"

		//There's a gap here while we wait for the imuls to finish. Maybe?



		"add 	%%ecx, %%edx		\n\t"
		"pinsrw $1, %%edx, %%mm0   	\n\t" 	//now, mm0 = 0 bu guv rv

		////////////////////////////////////////////////////////////
		//
		// Load and convert luminance
		//
		
		//Put y4y3y2y1 in to eax
		"mov	2(%%esi), %%eax		\n\t" //Load y4y3?? in to eax
		"movw	1(%%esi), %%ax		\n\t" //load y2y1 in to ax

		//Unpack and shift left by 7, so that mm3 contains the fixed point version of y4--y1
		"movd	%%eax, %%mm3		\n\t"
		"punpcklbw %%mm7, %%mm3		\n\t"
		"psllw 	$7, %%mm3			\n\t"

		"pshufw $0x40, %%mm3, %%mm1	\n\t" //Create y2 y1 y1 y1 register mm1: 0x40 = 01 00 00 00
		"pshufw $0xa5, %%mm3, %%mm2	\n\t" //Create y3 y3 y2 y2 register mm2: 0xa5 = 10 10 01 01
		"pshufw $0xfe, %%mm3, %%mm3	\n\t" //Create y4 y4 y4 y3 register mm3: 0xfe = 11 11 11 10
	

		#ifdef CVD_HAVE_SSE2
			"movnti	%%eax, (%%ebp)		\n\t"
		#else 
			"mov	%%eax, (%%ebp)		\n\t"
		#endif
		
		/////////////////////////////////////////////////////////////
		//
		// Perform fixed point conversion to RGB
		//

		//Perform mm1 += mm0, mm2 += mm0, etc
		//permute mm0 each time in to the right shape
		"pshufw $0x24, %%mm0, %%mm0 \n\t"	//mm0 = r b g r    0x24 = 00 10 01 00
		"paddw	%%mm0, %%mm1		\n\t"
		
		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = g r b g    0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm2		\n\t"

		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = b g r b	   0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm3		\n\t"

		//Perform right shift of 7 (mm5)  on mm1, 2, 3, 4
		//!!!!! Apparently, there is only one shifter. Might be good to reorganise a bit
		"psrlw	$7, %%mm1		\n\t"
		"psrlw	$7, %%mm2		\n\t"
		"psrlw	$7, %%mm3		\n\t"

		//Pack mm1,2 in to mm1  with unsigned saturation
		"packuswb	%%mm2, %%mm1	\n\t"
		
		//At this point we only have 3 pixels, which won't leave the writing
		//position in a 64 bit aligned state, so do another 3.
	
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
		//Perform the conversion on the second set of 3 pixels. 


		////////////////////////////////////////////////////////////
		//
		//	Load and convert the chrominance values to mmxable things
		//

		//Create the register mm0 = * bu guv rv
		//Let ecx, edx contain (u-128), (v-128)

		"movzbl 9(%%esi), %%edx		\n\t"	//dx = v
		"movzbl	6(%%esi), %%ecx		\n\t"	//cx = u   move-(with-zero-extend)-byte-(in to)-long
		"sub	$128, %%edx			\n\t"
		"sub	$128, %%ecx			\n\t"	
		
		"mov	%%edx, %%eax		\n\t"	//Calculate and insert rv = v * 147
		"imul	$147, %%eax			\n\t"
		"pinsrw	$0, %%eax, %%mm0	\n\t"

		"mov 	%%ecx, %%ebx		\n\t"	//Calculate and insert bu = v * 256
		"imul	$256, %%ebx			\n\t"
		"pinsrw $2, %%ebx, %%mm0	\n\t"

		"imul	$-38, %%ecx			\n\t"	//Calculate and insert guv = -38u + -74v
		"imul	$-74, %%edx			\n\t"
		"add 	%%ecx, %%edx		\n\t"
		"pinsrw $1, %%edx, %%mm0   	\n\t" 	//now, mm0 = 0 bu guv rv


		////////////////////////////////////////////////////////////
		//
		// Load and convert luminance
		//
		
		//Put y4y3y2y1 in to eax
		"mov	8(%%esi), %%eax		\n\t" //Load y4y3?? in to eax
		"movw	7(%%esi), %%ax		\n\t" //load y2y1 in to ax
		
		//Unpack and shift left by 7, so that mm5 contains the fixed point version of y4--y1
		"movd	%%eax, %%mm6		\n\t"
		"punpcklbw %%mm7, %%mm6		\n\t"
		"psllw 	$7, %%mm6			\n\t"
		

		#ifdef CVD_HAVE_SSE2
			"movnti %%eax, 4(%%ebp)		\n\t"
		#else 
			"mov %%eax, 4(%%ebp)		\n\t"
		#endif
	//	"pushl	%%eax				\n\t"
	//	"movq	-8(%%esp), %%mm2	\n\t"
	//	"movntq %%mm2, (%%ebp)		\n\t"
	//	"popl  %%eax				\n\t"
	//	"popl  %%ebx				\n\t"
		

		"pshufw $0x40, %%mm6, %%mm4	\n\t" //Create y2 y1 y1 y1 register mm1: 0x40 = 01 00 00 00
		"pshufw $0xa5, %%mm6, %%mm5	\n\t" //Create y3 y3 y2 y2 register mm2: 0xa5 = 10 10 01 01
		"pshufw $0xfe, %%mm6, %%mm6	\n\t" //Create y4 y4 y4 y3 register mm3: 0xfe = 11 11 11 10


		/////////////////////////////////////////////////////////////
		//
		// Perform fixed point conversion to RGB
		//

		//Perform mm1 += mm0, mm2 += mm0, etc
		//permute mm0 each time in to the right shape
		"pshufw $0x24, %%mm0, %%mm0 \n\t"	//mm0 = r b g r    0x24 = 00 10 01 00
		"paddw	%%mm0, %%mm4		\n\t"
		
		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = g r b g    0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm5		\n\t"

		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = b g r b	   0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm6		\n\t"

		//Perform right shift of 7 (mm5)  on mm1, 2, 3, 4
		"psrlw	$7, %%mm4			\n\t"
		"psrlw	$7, %%mm5			\n\t"
		"psrlw	$7, %%mm6			\n\t"

		//Pack mm1,2 in to mm1 and mm3,4 in to mm3 with unsigned saturation
		"packuswb	%%mm4, %%mm3	\n\t"
		"packuswb	%%mm6, %%mm5	\n\t"
		
		//movntq uses write-combining. Fast.
		"movntq		%%mm1, (%%edi)	\n\t"
		"movntq		%%mm3, 8(%%edi)	\n\t"
		"movntq		%%mm5, 16(%%edi)\n\t"


////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		

		//Increment counters
		"add		$12, %%esi		\n\t"
		"add		$24, %%edi		\n\t"
		"add		$8, %%ebp		\n\t"
		"cmp		%%esi, 	(%%esp)	\n\t"	//in_end is on the top of the stack
		"jne 		.Lfoo	   		\n\t"
		
		"popl		%%eax			\n\t"   //discard the top
		"popl		%%ebp			\n\t"   //restore ebp
		"popl		%%ebx			\n\t"	//restore ebx
		"emms						\n\t"	//End mmx
	//   0          1        2   	
	:
	: "m" (out), "m" (in), "g" (size), [luma] "m" (lum_out), [end] "m" (in_end)
	: "eax", "ecx", "edx",  "edi", "esi", "mm0", "mm1", "mm2", "mm3", "mm4", "mm5", "mm6", "mm7"
	);
}
void yuv411_to_rgb(const unsigned  char* in, int size, unsigned char* out)
{
	//Time: 5.0ms memcpy on number of luma pixels only takes 1.5 ms
	size /=8;
	__asm__ __volatile__(
		//Put some zeros in mm7
		"mov		$0, %%eax		\n\t"
		"movd		%%eax, %%mm7	\n\t"
	
		//Load out and in
		"mov		%0, %%edi		\n\t"
		"mov		%1, %%esi		\n\t"
		"push		%%ebx			\n\t" //save ebx. For some reason, the compiler won't do this
		"push		%%ebp			\n\t" //save the stack base
		"mov		%2, %%ebp		\n\t"

".Lyuv411dec:						\n\t"
		
		"prefetchnta	64(%%esi)	\n\t"
		/*Convert yuv411 to rgb8	
		format is u1 y1 y2 v1 y3 y4
		u, v are int8 - 128
		y	   are uint8

		For YUV in general:
		r = y + 1.140 * v			= y + rv
		g = y - 0.394*u - 0.581*v	= y + guv
		b = y + 2.028 * u			= y + bu

		Arithmetic is performed using fixed point with a left shift of 7 (ie *128)
		u1, v1 are not interpolated between y pixels, ie u1,v1 are used for y1,y2,y3,y4
		b is truncated slightly, ie b= y + 2.0 * u

		The resultant 8 bit r,g,b values need to be saturated, since overflow occurs
		and this leads to very ugly results.
		*/
		
		////////////////////////////////////////////////////////////
		//
		//	Load and convert the chrominance values to mmxable things
		//

		//Create the register mm0 = * bu guv rv
		//Let ecx, edx contain (u-128), (v-128)

		//Note the following 4 instructions are in a different order in the second 
		//block, since it's faster to have them different (!)

		"movzbl 3(%%esi), %%edx		\n\t"	//dx = v   move-(with-zero-extend)-byte-(in to)-long 
		"sub	$128, %%edx			\n\t"
		"movzbl	(%%esi), %%ecx		\n\t"	//cx = u
		"sub	$128, %%ecx			\n\t"	
		
		"mov	%%edx, %%eax		\n\t"	//Calculate and insert rv = v * 147
		"imul	$147, %%eax			\n\t"
		"pinsrw	$0, %%eax, %%mm0	\n\t"

		"mov 	%%ecx, %%ebx		\n\t"	//Calculate and insert bu = v * 256
		"imul	$256, %%ebx			\n\t"
		"pinsrw $2, %%ebx, %%mm0	\n\t"

		"imul	$-38, %%ecx			\n\t"	//Calculate and insert guv = -38u + -74v
		"imul	$-74, %%edx			\n\t"

		//There's a gap here while we wait for the imuls to finish. Maybe?



		"add 	%%ecx, %%edx		\n\t"
		"pinsrw $1, %%edx, %%mm0   	\n\t" 	//now, mm0 = 0 bu guv rv

		////////////////////////////////////////////////////////////
		//
		// Load and convert luminance
		//
		
		//Put y4y3y2y1 in to eax
		"mov	2(%%esi), %%eax		\n\t" //Load y4y3?? in to eax
		"movw	1(%%esi), %%ax		\n\t" //load y2y1 in to ax
		
		//Unpack and shift left by 7, so that mm3 contains the fixed point version of y4--y1
		"movd	%%eax, %%mm3		\n\t"
		"punpcklbw %%mm7, %%mm3		\n\t"
		"psllw 	$7, %%mm3			\n\t"

		"pshufw $0x40, %%mm3, %%mm1	\n\t" //Create y2 y1 y1 y1 register mm1: 0x40 = 01 00 00 00
		"pshufw $0xa5, %%mm3, %%mm2	\n\t" //Create y3 y3 y2 y2 register mm2: 0xa5 = 10 10 01 01
		"pshufw $0xfe, %%mm3, %%mm3	\n\t" //Create y4 y4 y4 y3 register mm3: 0xfe = 11 11 11 10


		/////////////////////////////////////////////////////////////
		//
		// Perform fixed point conversion to RGB
		//

		//Perform mm1 += mm0, mm2 += mm0, etc
		//permute mm0 each time in to the right shape
		"pshufw $0x24, %%mm0, %%mm0 \n\t"	//mm0 = r b g r    0x24 = 00 10 01 00
		"paddw	%%mm0, %%mm1		\n\t"
		
		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = g r b g    0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm2		\n\t"

		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = b g r b	   0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm3		\n\t"

		//Perform right shift of 7 (mm5)  on mm1, 2, 3, 4
		//!!!!! Apparently, there is only one shifter. Might be good to reorganise a bit
		"psrlw	$7, %%mm1		\n\t"
		"psrlw	$7, %%mm2		\n\t"
		"psrlw	$7, %%mm3		\n\t"

		//Pack mm1,2 in to mm1  with unsigned saturation
		"packuswb	%%mm2, %%mm1	\n\t"
		
		//At this point we only have 3 pixels, which won't leave the writing
		//position in a 64 bit aligned state, so do another 3.
	
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
		//Perform the conversion on the second set of 3 pixels. 


		////////////////////////////////////////////////////////////
		//
		//	Load and convert the chrominance values to mmxable things
		//

		//Create the register mm0 = * bu guv rv
		//Let ecx, edx contain (u-128), (v-128)

		"movzbl 9(%%esi), %%edx		\n\t"	//dx = v
		"movzbl	6(%%esi), %%ecx		\n\t"	//cx = u   move-(with-zero-extend)-byte-(in to)-long
		"sub	$128, %%edx			\n\t"
		"sub	$128, %%ecx			\n\t"	
		
		"mov	%%edx, %%eax		\n\t"	//Calculate and insert rv = v * 147
		"imul	$147, %%eax			\n\t"
		"pinsrw	$0, %%eax, %%mm0	\n\t"

		"mov 	%%ecx, %%ebx		\n\t"	//Calculate and insert bu = v * 256
		"imul	$256, %%ebx			\n\t"
		"pinsrw $2, %%ebx, %%mm0	\n\t"

		"imul	$-38, %%ecx			\n\t"	//Calculate and insert guv = -38u + -74v
		"imul	$-74, %%edx			\n\t"
		"add 	%%ecx, %%edx		\n\t"
		"pinsrw $1, %%edx, %%mm0   	\n\t" 	//now, mm0 = 0 bu guv rv


		////////////////////////////////////////////////////////////
		//
		// Load and convert luminance
		//
		
		//Put y4y3y2y1 in to eax
		"mov	8(%%esi), %%eax		\n\t" //Load y4y3?? in to eax
		"movw	7(%%esi), %%ax		\n\t" //load y2y1 in to ax
		
		//Unpack and shift left by 7, so that mm5 contains the fixed point version of y4--y1
		"movd	%%eax, %%mm6		\n\t"
		"punpcklbw %%mm7, %%mm6		\n\t"
		"psllw 	$7, %%mm6			\n\t"

		"pshufw $0x40, %%mm6, %%mm4	\n\t" //Create y2 y1 y1 y1 register mm1: 0x40 = 01 00 00 00
		"pshufw $0xa5, %%mm6, %%mm5	\n\t" //Create y3 y3 y2 y2 register mm2: 0xa5 = 10 10 01 01
		"pshufw $0xfe, %%mm6, %%mm6	\n\t" //Create y4 y4 y4 y3 register mm3: 0xfe = 11 11 11 10


		/////////////////////////////////////////////////////////////
		//
		// Perform fixed point conversion to RGB
		//

		//Perform mm1 += mm0, mm2 += mm0, etc
		//permute mm0 each time in to the right shape
		"pshufw $0x24, %%mm0, %%mm0 \n\t"	//mm0 = r b g r    0x24 = 00 10 01 00
		"paddw	%%mm0, %%mm4		\n\t"
		
		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = g r b g    0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm5		\n\t"

		"pshufw $0x49, %%mm0, %%mm0	\n\t"	//mm0 = b g r b	   0x49 = 01 00 10 01
		"paddw	%%mm0, %%mm6		\n\t"

		//Perform right shift of 7 (mm5)  on mm1, 2, 3, 4
		"psrlw	$7, %%mm4			\n\t"
		"psrlw	$7, %%mm5			\n\t"
		"psrlw	$7, %%mm6			\n\t"

		//Pack mm1,2 in to mm1 and mm3,4 in to mm3 with unsigned saturation
		"packuswb	%%mm4, %%mm3	\n\t"
		"packuswb	%%mm6, %%mm5	\n\t"
		
		//movntq uses write-combining. Fast.
		"movntq		%%mm1, (%%edi)	\n\t"
		"movntq		%%mm3, 8(%%edi)	\n\t"
		"movntq		%%mm5, 16(%%edi)\n\t"


////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		
////////////////////////////////////////////////////////////////////////////////		

		//Increment counters
		"add		$24, %%edi		\n\t"
		"add		$12, %%esi		\n\t"
		"dec		%%ebp			\n\t"
		"jnz		.Lyuv411dec		\n\t"
		
		"popl		%%ebp			\n\t"   //restore ebp
		"popl		%%ebx			\n\t"	//restore ebx
		"emms						\n\t"	//End mmx
	//   0          1        2   	
	:
	: "m" (out), "m" (in), "g" (size)
	: "eax", "ecx", "edx",  "edi", "esi", "mm0", "mm1", "mm2", "mm3", "mm4", "mm5"
	);
}

void yuv411_to_y(const unsigned char* in, int size, unsigned char* out)
{
	//Time, 2.5ms. Comparison, memcpy on the same number of output bytes=1.5ms

	size /=16;
	__asm__ __volatile__(
		"push	%%ebx			\n\t"
		"push	%%ebp			\n\t"
		"mov %0, %%edi			\n\t"
		"mov %1, %%esi			\n\t"
		"mov %2, %%ebp			\n\t"
".Lyuvtolum:					\n\t"
		"prefetchnta	64(%%esi)	\n\t"
		"mov  2(%%esi), %%eax	\n\t"
		"movw 1(%%esi), %%ax	\n\t"
		"mov  8(%%esi), %%ebx	\n\t"
		"movw 7(%%esi), %%bx	\n\t"
		"mov  14(%%esi), %%ecx	\n\t"
		"movw 13(%%esi), %%cx	\n\t"
		"mov  20(%%esi), %%edx	\n\t"
		"movw 19(%%esi), %%dx	\n\t"

		//Write combining stuff, movnt seems only to work (SIGILL)
		//on the P3 from MMX registers, so pack the data in to MMX
		//registers before moving it.

		"movd	%%eax, %%mm0	\n\t"
		"movd	%%ebx, %%mm1	\n\t"
		"psllq  $32, %%mm1		\n\t"
		"por	%%mm1, %%mm0	\n\t"  //mm0 = ebx,eax

		"movd	%%ecx, %%mm1	\n\t"
		"movd	%%edx, %%mm2	\n\t"
		"psllq  $32, %%mm2		\n\t"
		"por	%%mm2, %%mm1	\n\t"  //mm1 = ebx,eax

		"movntq	%%mm0, (%%edi)	\n\t"
		"movntq	%%mm1, 8(%%edi)	\n\t"

		
/* this doens't help
		"mov  26(%%esi), %%eax	\n\t"
		"movw 25(%%esi), %%ax	\n\t"
		"mov  32(%%esi), %%ebx	\n\t"
		"movw 31(%%esi), %%bx	\n\t"
		"mov  38(%%esi), %%ecx	\n\t"
		"movw 37(%%esi), %%cx	\n\t"
		"mov  44(%%esi), %%edx	\n\t"
		"movw 43(%%esi), %%dx	\n\t"

		"movd	%%eax, %%mm2	\n\t"
		"movd	%%ebx, %%mm3	\n\t"
		"psllq  $32, %%mm3		\n\t"
		"por	%%mm3, %%mm2	\n\t"  //mm2 = ebx,eax

		"movd	%%ecx, %%mm3	\n\t"
		"movd	%%edx, %%mm4	\n\t"
		"psllq  $32, %%mm4		\n\t"
		"por	%%mm4, %%mm3	\n\t"  //mm1 = ebx,eax

		"movntq	%%mm0, (%%edi)	\n\t"
		"movntq	%%mm1, 8(%%edi)	\n\t"
*/

		"add  $24, %%esi		\n\t"
		"add  $16, %%edi		\n\t"
		"dec  %%ebp				\n\t"
		"jnz .Lyuvtolum			\n\t"
		"popl %%ebp				\n\t"
		"popl %%ebx				\n\t"
		"emms					\n\t"
	:
	: "m" (out), "m" (in), "g" (size)
	: "eax", "ecx",  "edi", "esi"
	);
}

}
}
