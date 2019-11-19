AC_INIT(CVD,0.8)
AC_COPYRIGHT(Copyright foo)

AC_CONFIG_HEADERS
AC_HEADER_STDC
#Use builtin autoconf checks for all the programs we need.
AC_PROG_AWK
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_RANLIB
#Switch testing language to C++
AC_LANG(C++)
AC_REQUIRE_CPP
#Start the compiler

cvd_major=`echo $PACKAGE_VERSION | $AWK -F'\\\\.' '{print $1}'`
cvd_minor=`echo $PACKAGE_VERSION | $AWK -F'\\\\.' '{print $2}'`

AC_DEFINE_UNQUOTED(CVD_MAJOR_VERSION,$cvd_major)
AC_SUBST(major, $cvd_major)
AC_DEFINE_UNQUOTED(CVD_MINOR_VERSION,$cvd_minor)
AC_SUBST(minor, $cvd_minor)


################################################################################
#
#Lists of  optional components
#

all_options="videodisplay jpeg tiff png ffmpeg toon lapack assembler inline_asm libuvc"
dodginess_options=
options=
simd_all_options=
simd_options=
os_all_options=

################################################################################
#
# Useful macros
#

dnl APPEND(var, value)
dnl This appends vale to a shell variable var
define(APPEND, [$1="$$1 $2"])



dnl TEST_AND_SET_CXXFLAG(flag, [program], [run])
dnl
dnl This attempts to compile a and run program with a certain compiler flag.
dnl If no program is given, then the minimal C++ program is compiled, and 
dnl this tests just the validity of the compiler flag. 
dnl
define([TEST_AND_SET_CXXFLAG],[
	if test "$3" == ""
	then
		AC_MSG_CHECKING([if compiler flag $1 works])	
	else
		AC_MSG_CHECKING([$3])	
	fi
	save_CXXFLAGS="$CXXFLAGS"
	APPEND(CXXFLAGS, [$1])

	m4_if([$2],[],[pushdef(prog, [int main(){}])], [pushdef(prog, [$2])])

	m4_if([$4],[run],
		[AC_RUN_IFELSE([AC_LANG_SOURCE([prog])], [cvd_conf_test=1],[cvd_conf_test=0], [cvd_conf_test=0])],
		[AC_COMPILE_IFELSE([AC_LANG_SOURCE([prog])], [cvd_conf_test=1],[cvd_conf_test=0])]
	)
	
	
	popdef([prog])

	if test $cvd_conf_test = 1
	then
		AC_MSG_RESULT(yes)
		ts_success=yes
	else
		AC_MSG_RESULT(no)
		CXXFLAGS="$save_CXXFLAGS"
		ts_success=no
	fi
])


dnl CHECK_FOR_OPTION(option, code, [success], [fail])
dnl option corresponds to the first parameter for the corresponding
dnl AC_ARG_WITH. If the library is present, then have_option and CVD_HAVE_OPTION
dnl are set. If with_option is "", it will be set to yes.
dnl
dnl The test code shall do nothing on success, and set the shell variable a
dnl on failure. For convenience, LIBS  and CXXFLAGS are checkpointed 
dnl automatically. On failure, they are restored automatically.
dnl
dnl On success, "option" is appended to options, and the code "success" is run
dnl On failure including --without) fail is run.
define(CHECK_FOR_OPTION,[
	if test "$with_$1" != no && test "$have_$1" != yes
	then
		if test "$with_$1" == ""
		then
			with_$1=yes
		fi

		save_LIBS="$LIBS"
		save_CXXFLAGS="$CXXFLAGS"
		a=
		$2
		
		if test x$a == x
		then
			AC_DEFINE(CVD_HAVE_[]m4_toupper($1), 1)
			AC_SUBST(have_$1, yes)
			APPEND(options, $1)
			$3
		else
			LIBS="$save_LIBS"
			CXXFLAGS="$save_CXXFLAGS"
		fi
	fi
	
	if test "$have_$1" != yes
	then
		:
		$4
	fi
	])


dnl SIMD_TEST(name, flag, code)
dnl This tests for SIMD code (with name), using the compiler flag "flag"
dnl and the provided test code. It respects with_simd and with_name
dnl
dnl On success, "name" is appended to simd_options
define(SIMD_TEST, [
	if test "$with_simd" != no
	then
		if test "$with_[]m4_tolower($1)" == "yes"
		then
			have_[]m4_tolower($1)=yes
		elif test "$with_[]m4_tolower($1)" != "no"
		then
			TEST_AND_SET_CXXFLAG([$2], [$3], [for $1 support],run)
			have_[]m4_tolower($1)="$ts_success"
		fi

		if test "$have_[]m4_tolower($1)" == yes
		then
			AC_DEFINE(CVD_HAVE_$1, 1)
			AC_SUBST(have_[]m4_tolower($1), yes)
			APPEND(simd_options, m4_tolower($1))
		fi
	fi])

#Host detection
AC_CANONICAL_HOST

################################################################################
#
# Compiler flags and optimization
#
#
TEST_AND_SET_CXXFLAG(-Wall)
TEST_AND_SET_CXXFLAG(-Wextra)
TEST_AND_SET_CXXFLAG(-pipe)


cpp11=no
for i in "" -std=c++14 -std=c++1y
do
	
	flag="$i"

	if test x$i == x
	then
		AC_MSG_CHECKING(For C++1y support)
	else
		AC_MSG_CHECKING(For C++1y support with flag $flag)
	fi

	save_CXXFLAGS="$CXXFLAGS"
	APPEND(CXXFLAGS, $flag)

	AC_COMPILE_IFELSE([AC_LANG_SOURCE([
	#include <array>
	constexpr int foo(int a)
	{
		int i=0;
		for(i=0; i < a ;i *=2)
			i++;
		return i;
	}
	int main()
	{
		auto a=[[]](auto b){};
		auto b=1;
	}
	])], [cpp11=yes])

	AC_MSG_RESULT($cpp11)

	if test x$cpp11 != xno
	then
		cpp11="$flag"
		break
	fi
	CXXFLAGS="$save_CXXFLAGS"
done


if test x$cpp11 == xno
then
	AC_MSG_ERROR("No C++14 support. Please add to CXXFLAGS")
fi


#Hack: do this properly
AC_CHECK_LIB(pthread, main,[],[a=1])


APPEND(CPPFLAGS, [-I"$(eval echo $includedir)"])

########################################
#
# Remove -O2 from CXXFLAGS
# 
CXXFLAGS="$(echo "$CXXFLAGS" | sed -e's/-O2//')"


########################################
#
# Always use ggdb if possible
#
AC_ARG_ENABLE(ggdb, [   --disable-ggdb           do not use ggdb flag])
if test "$enable_ggdb" != no
then 
	TEST_AND_SET_CXXFLAG(-ggdb)
fi


########################################
#
# Debug build
#
AC_ARG_ENABLE(debug,   [  --disable-debug            disable debug build])
if test "$enable_debug" != no
then
	AC_MSG_CHECKING([if -D_GLIBCXX_DEBUG can works...])
	AC_COMPILE_IFELSE([AC_LANG_SOURCE([
	#define _GLIBCXX_DEBUG
	#include <iostream>
	#include <vector>
	int main()
	{}
	])], [debug=yes],[debug=no])]

	AC_MSG_RESULT([$debug])

	if test "$debug" == yes
	then
		AC_SUBST(STLDEBUG, [-D_GLIBCXX_DEBUG])
	fi

	AC_SUBST(debug_build, yes)
else
	AC_SUBST(debug_build, no)
fi


########################################
#
# Position independent code
#
# MinGW is always PIC
if ! echo $host | grep mingw && test "$host_os" != "cygwin"
then
	TEST_AND_SET_CXXFLAG(-fPIC)
	if test $ts_success == no
	then
		AC_MSG_WARN(["I don't know how to compile position independent code."])
	fi
fi


########################################
#
# CPU type detection and optimization
#
noopt_CXXFLAGS="$CXXFLAGS"
CXXFLAGS=

AC_ARG_WITH(optimize,   [  --without-optimize      compile without optimizations ],[optimize=no],[])
AC_ARG_WITH(optimize,   [  --with-optimize=flags   compile with optimization flags],[optimize="$withval"], [optimize=-O3])

if test "$optimize" != no
then
	TEST_AND_SET_CXXFLAG(-O3)
	if test $ts_success == no
	then
		AC_MSG_WARN([I do not know how to switch on optimizing for this compiler])
	fi
fi
OFLAGS="$CXXFLAGS"
CXXFLAGS="$noopt_CXXFLAGS"

################################################################################
#
# Processor/Platform specific parts
#

echo
echo ------------------------------------
echo Checking processor specific features
echo ------------------------------------
echo

AC_C_BIGENDIAN

if test x$ac_cv_c_bigendian = xno
then
	AC_DEFINE(CVD_INTERNAL_ARCH_LITTLE_ENDIAN)
else
	AC_DEFINE(CVD_INTERNAL_ARCH_BIG_ENDIAN)
fi



AC_ARG_WITH(simd, [  --without-simd          disable SIMD code. Use --without- or --with-
                             to force presence/absence and disable autodetection.])
AC_ARG_WITH(mmx, [    --with-mmx            force presence or absence of MMX  (x86 only)])	
AC_ARG_WITH(mmxext, [    --with-mmxext         force presence or absence of MMXEXT (x86 only) ])	
AC_ARG_WITH(sse, [    --with-sse            force presence or absence of SSE  (x86 only)])	
AC_ARG_WITH(sse2, [    --with-sse2           force presence or absence of SSE2  (x86 only)])	
AC_ARG_WITH(assembler, [    --with-assembler     force presence or absence of asm])
AC_ARG_WITH(inline_asm, [    --with-inline_asm     force presence or absence of inline asm])

case "$host" in
	*86*)
		AC_SUBST(have_x86, yes)
		simd_all_options="mmx mmxext sse sse2 sse3"

		SIMD_TEST(MMX, -mmmx, [
			#include <mmintrin.h>
			int main()
			{
				__m64 a;
				a = _mm_setzero_si64();
				a = _mm_and_si64(a, a);
				return 0;
			}], [for MMX support])

		SIMD_TEST(MMXEXT, -msse,[
			#include <xmmintrin.h>
			int main()
			{
				__m64 a;
				a = _mm_setzero_si64();
				a = _mm_shuffle_pi16(a, 0);
			return 0;}], [for MMXEXT support])

		SIMD_TEST(SSE, -msse, [
			#include <xmmintrin.h>
			int main()
			{
				__m128 a;
				a = _mm_setzero_ps();
				a = _mm_add_ps(a, a);
				return 0;
			}], [for SSE spport])

		SIMD_TEST(SSE2, -msse2, [
			#include <emmintrin.h>
			int main()
			{
				__m128d a;
				a = _mm_setzero_pd();
				a = _mm_add_pd(a, a);
				return 0;
			}], [for SSE2 support])

		SIMD_TEST(SSE3, -msse3, [
			#include <pmmintrin.h>
			int main()
			{ 
				__m128 a;
				a = _mm_setzero_ps();
				a = _mm_hadd_ps(a, a);
				return 0;
			}], [for SSE3 support])
		;;
esac

AC_CHECK_SIZEOF([void*])
if test "$ac_cv_sizeof_voidp" == 4
then
	AC_SUBST(have_32bit,yes)
fi



CHECK_FOR_OPTION(inline_asm, [
	AC_MSG_CHECKING([for inline asm statement])
	AC_COMPILE_IFELSE([AC_LANG_SOURCE([int main() { asm(""); }])],
		[AC_MSG_RESULT(yes)],
		[AC_MSG_RESULT(no);a=1])
])

CHECK_FOR_OPTION(assembler, [
	AC_MSG_CHECKING([assembler supports .type pseudo-op])
	AS_IF([echo '.globl test
	.type test, @function' | as &> /dev/null], 
		[AC_MSG_RESULT(yes)],
		[AC_MSG_RESULT(no);a=1])
])

################################################################################
#
# Operating system specific parts
#

########################################
#
# An OSX specific hack to get at stuff in /sw automatically
#

echo
echo -----------------------------------------------
echo Checking for operating system specific features
echo -----------------------------------------------

AC_ARG_WITH(dc1394v2,  [  --without-dc1394v2      do not use libDC1394v2 (required for dvbuffer3) (Linux/OSX). ])
AC_ARG_WITH(v4l2buffer,[  --without-v4l2buffer	  compile without v4l2buffer (Video4Linux2) support (Linux only)])

if test "$host_vendor" == apple
then
	AC_CHECK_FILE(/opt/local, [
		APPEND(CPPFLAGS, [-I/opt/local/include])
		APPEND(LIBS, [-L/opt/local/lib])], 
		[])
	AC_CHECK_FILE(/sw, [
		APPEND(CPPFLAGS, [-I/sw/include])
		APPEND(LIBS, [-L/sw/lib])], 
		[])
	APPEND(CPPFLAGS, [-I/usr/X11R6/include])
	AC_MSG_NOTICE([Adding /usr/X11R6/include to the build path.])
	AC_SUBST(osx_hacks,yes)
	os_all_options="dc1394v2"
fi

CHECK_FOR_OPTION(dc1394v2, [
	AC_CHECK_HEADERS(dc1394/dc1394.h, [], [a=1])
	AC_CHECK_LIB(dc1394, main,[],[a=1])
])

case "$host" in
	*linux*)
		os_all_options="dc1394v2 v4l2buffer"
		
		CHECK_FOR_OPTION(v4l2buffer,[
			AC_MSG_CHECKING(for v4l2)
			AC_COMPILE_IFELSE([AC_LANG_SOURCE([	#include <sys/types.h>
								#include <linux/types.h>
								#include <linux/videodev2.h>
								int main(){V4L2_PIX_FMT_GREY;return 0;}])], 
								[AC_MSG_RESULT(yes)],
								[AC_MSG_RESULT(no); a=1])
		])
		
		
		;;
	*windows*)
		AC_SUBST(have_windows, yes)
		;;
	*cygwin*)
		AC_SUBST(cygwin_hacks,yes)
		;;

		
esac


if test "$host_vendor" == apple
then
	CHECK_FOR_OPTION(qtbuffer,[
		APPEND(LIBS, [-framework Carbon -framework QuickTime])
		AC_MSG_CHECKING(Carbon and QuickTime framework)
		AC_COMPILE_IFELSE([AC_LANG_SOURCE([ #include <Carbon/Carbon.h>
							#include <QuickTime/QuickTime.h>
							int main(){InitCursor();EnterMovies();return 0;}])],
							[AC_MSG_RESULT(yes)], 
							[AC_MSG_RESULT(no); a=1]) 
	])
fi


################################################################################
#
#Check for optional libraries
#

echo
echo -------------------------------
echo Checking for optional libraries
echo -------------------------------

AC_ARG_WITH(videodisplay, [  --without-videodisplay  compile without videodisplay support])
AC_ARG_WITH(jpeg,    [  --without-jpeg          compile without JPEG support])
AC_ARG_WITH(jpegbuffer, [  --with-jpegbuffer=N     Set JPEG read buffer size. N>1 will not be able to read multiple JPEGS from a stream.])
AC_ARG_WITH(tiff, [  --without-tiff          compile without TIFF support])
AC_ARG_WITH(png, [  --without-png           compile without PNG support])
AC_ARG_WITH(ffmpeg, [  --without-ffmpeg	  compile without FFMPEG support.])
AC_ARG_WITH(toon, [  --without-toon          Compile without TooN support])
AC_ARG_WITH(lapack, [  --without-lapack        Disable BLAS and LAPACK.])
AC_ARG_WITH(acml, [  --with-acml[[=directory]] Use BLAS and LAPACK routines from the ACML library [[located in the specified directory]] ], [opt_acml_dir=$withval])

CHECK_FOR_OPTION(videodisplay,[

	if test "$host_vendor" == apple
	then
		save_LIBS="$LIBS"

		APPEND(LIBS, [ -framework OpenGL /usr/X11R6/lib/libX11.dylib /usr/X11R6/lib/libGLU.dylib /usr/X11R6/lib/libGL.dylib])
		# all of the following is already covered in the framework
		# AC_CHECK_LIB(GL,glDrawPixels, [], [a=1])
		# AC_CHECK_HEADERS(GL/glu.h, [], [a=1])
		# AC_CHECK_LIB(GLU,gluGetString, [], [a=1])

		AC_MSG_CHECKING([for X11 using dubious methods...])

		a=
		AC_COMPILE_IFELSE([AC_LANG_SOURCE([#include <X11/Xlib.h>
		int main()
		{
			XOpenDisplay(NULL);
		}])],  [AC_MSG_RESULT(yes)],[AC_MSG_RESULT(no); a=1]) 
		
		if test x$a == x1
		then
			LIBS="$save_LIBS"
		fi
	else

		AC_PATH_X
		if test x"$x_includes" != x
		then
			APPEND(CXXFLAGS, -I$x_includes)
		fi

		if test "$x_libraries" != ""
		then
			APPEND(LIBS, [-L$x_libraries])
		fi

		APPEND(LIBS, [-lX11 -lXext])
		AC_CHECK_LIB(GL,glDrawPixels, [], [a=1])
		AC_CHECK_HEADERS(GL/glu.h, [], [a=1])
		AC_CHECK_LIB(GLU,gluGetString, [], [a=1])
	fi
])

CHECK_FOR_OPTION(toon, [
	AC_MSG_CHECKING([for TooN])
	AC_COMPILE_IFELSE([AC_LANG_SOURCE([#include <TooN/TooN.h>
	int main(){TooN::Vector<2, double> v=TooN::Zeros;return 0;}])], [AC_MSG_RESULT(yes)],[AC_MSG_RESULT(no); a=1]) 
])

CHECK_FOR_OPTION(lapack, [
	if test "$with_acml" != no
	then
		if test "$with_acml" != yes & test "$with_acml" != ""
		then
			APPEND(LIBS, -L $with_acml)
		else
			APPEND(LIBS, -L/opt/acml/gnu64/lib)
		fi
		APPEND(LIBS, -lacml_mv)

		AC_CHECK_LIB(acml, dgesvd_, [], [a=1])
	else
		a=1
	fi
])

CHECK_FOR_OPTION(lapack, [
	AC_MSG_CHECKING(if Accelerate framework is needed for LAPACK)
	AC_MSG_RESULT()
	APPEND(LIBS, -framework Accelerate)
	AC_CHECK_FUNC(dgesvd_, [], [a=1])
])



CHECK_FOR_OPTION(lapack, [
	AC_CHECK_LIB(lapack, dgesvd_, [], [a=1])
])

CHECK_FOR_OPTION(lapack, [
	AC_MSG_CHECKING(if blas is needed for LAPACK)
	AC_MSG_RESULT()
	APPEND(LIBS, -lblas)
	AC_CHECK_LIB(lapack, zupmtr_, [], [a=1])
])

CHECK_FOR_OPTION(lapack, [
	AC_MSG_CHECKING(if f2c is needed for LAPACK)
	AC_MSG_RESULT()
	APPEND(LIBS, -lf2c)
	AC_CHECK_LIB(lapack, dpotrf_, [], [a=1])
])

CHECK_FOR_OPTION(lapack, [
	AC_MSG_CHECKING(if g2c is needed for LAPACK)
	AC_MSG_RESULT()
	APPEND(LIBS, -lg2c)
	AC_CHECK_LIB(lapack, dpotrs_, [], [a=1])
])

CHECK_FOR_OPTION(lapack, [
	AC_MSG_CHECKING(if gfortran is needed for LAPACK)
	AC_MSG_RESULT()
	APPEND(LIBS, -lgfortran)
	AC_CHECK_LIB(lapack, dpotri_, [], [a=1])
])



CHECK_FOR_OPTION(libuvc, [
	AC_CHECK_HEADERS(libuvc/libuvc.h, [], [a=1])
	AC_CHECK_LIB(uvc, uvc_stream_get_frame, [], [a=1])
])

CHECK_FOR_OPTION(png, [
	AC_CHECK_HEADERS(png.h, [], [a=1])
	AC_CHECK_LIB(png, png_init_io, [], [a=1])
])

CHECK_FOR_OPTION(jpeg, [
	AC_CHECK_HEADERS(jpeglib.h, [], [a=1])
	AC_CHECK_LIB(jpeg, jpeg_destroy_decompress, [], [a=1])
],[

	AC_MSG_CHECKING([JPEG read buffer size])

	jpegbuffersize="`echo $with_jpegbuffer | awk '$1 = ($1+0<= 0)?1:$1'`"

	if test "$jpegbuffersize" -gt 1
	then
		AC_MSG_RESULT([$jpegbuffersize (unsafe reading)]);
		APPEND(dodginess_options, unsafe_jpeg)
	else
		jpegbuffersize=1
		AC_MSG_RESULT([1 (safe reading)]);
	fi
	AC_DEFINE_UNQUOTED(CVD_INTERNAL_JPEG_BUFFER_SIZE, $jpegbuffersize)
])


CHECK_FOR_OPTION(tiff, [
	AC_CHECK_HEADERS(tiffio.h,[],[a=1])
	AC_CHECK_LIB(tiff, TIFFReadRGBAImage,[],[a=1])
])



CHECK_FOR_OPTION(tiff, [
	AC_MSG_CHECKING([if perhaps libtiff requires zlib])
	AC_MSG_RESULT([])
	unset ac_cv_lib_tiff_TIFFReadRGBAImage
	AC_CHECK_HEADERS(tiffio.h,[],[a=1])
	AC_CHECK_LIB(z, deflate,[],[a=1])
	AC_CHECK_LIB(tiff, TIFFReadRGBAImage,[],[a=1])
],[
	APPEND(dodginess_options, tiff_needs_zlib)
])


CHECK_FOR_OPTION(ffmpeg, [
	AC_MSG_CHECKING([for ffmpeg headers])

	AC_CHECK_LIB(avutil, main, [], [a=1])
	AC_CHECK_LIB(avformat, avformat_open_input, [], [a=1])
	AC_CHECK_LIB(avformat, av_read_frame, [ffmpeg_recent=yes], [a=1]) #only in >= 0.4.9
	AC_CHECK_LIB(avcodec, avcodec_open2, [], [a=1])
	AC_CHECK_LIB(swscale, sws_getContext, [], [a=1])
	AC_CHECK_LIB(avdevice, avdevice_register_all, [], [a=1])
])

AC_CHECK_PROG(docs, doxygen, docs, no)
################################################################################
#
# platform compatibility
# All of these ought to be yes
echo
echo -----------------------------------
echo Checking for platform compatibility 
echo -----------------------------------

a=
AC_CHECK_DECL(feenableexcept, [a=1], [a= ], [#include <fenv.h>])
if test x$a = x1
then
	AC_DEFINE(CVD_HAVE_FENV_H)
else
	dodginess_options="$dodginess_options missing_c99_feenableexcept"
fi

################################################################################
#
# Extra build options
echo
echo --------------------------------
echo Checking for extra build options
echo --------------------------------

AC_ARG_ENABLE(fast, [  --disable-fast               compile without the FAST-9,10,12 detectors (use on low memory systems).     ])
AC_ARG_ENABLE(fastobscure, [  --disable-obscure             compile without the FAST-7,8,11 detectors (use on low memory systems).     ])


test "$enable_fast" != no && AC_SUBST(have_fast, yes)
test "$enable_fastobscure" != no && test "$enable_fast" != no && AC_SUBST(have_fastobscure, yes)

################################################################################
#
# Compute dependencies
#

define(DEPOBJ, [APPEND(dep_objects, cvd_src/$1.o)])

#Obviously this doesn't do Windows.
#
# For Windows, you want the stuff in the Win32 directory
# and not videodisplay and globlist

################################################################################
#
# Architecture specific dependencies
#

DEPOBJ(noarch/convert_rgb_to_y)


#SIMD dependencies
#i686/yuv411_to_stuff_MMX                             inline_asm mmxext 32bit x86
#i686/yuv411_to_stuff_MMX_64                          inline_asm mmxext x86
#yuv411_to_stuff                                      END

if test "$have_sse2" == yes
then 
	DEPOBJ(SSE2/half_sample)
	DEPOBJ(SSE2/gradient)
	DEPOBJ(SSE2/median_3x3)
	DEPOBJ(SSE2/two_thirds_sample)
	DEPOBJ(SSE2/utility_double_int)
else
	DEPOBJ(noarch/half_sample)
	DEPOBJ(noarch/gradient)
	DEPOBJ(noarch/median_3x3)
	DEPOBJ(noarch/two_thirds_sample)
	DEPOBJ(noarch/utility_double_int)
fi

if test "$have_sse" == yes
then 
	DEPOBJ(SSE/convolve_gaussian)
	DEPOBJ(SSE/utility_float)
else
	DEPOBJ(noarch/convolve_gaussian)
	DEPOBJ(noarch/utility_float)
fi	

if test "$have_sse" == yes
then 
	DEPOBJ(MMX/utility_byte_differences)
else
	DEPOBJ(noarch/utility_byte_differences)
fi	


if test "$have_fast" == yes
then 

	if test "$have_sse2" == yes
	then 
		DEPOBJ(SSE2/faster_corner_9)
		DEPOBJ(SSE2/faster_corner_10)
		DEPOBJ(SSE2/faster_corner_12)
	else
		DEPOBJ(noarch/slower_corner_9)
		DEPOBJ(noarch/slower_corner_10)
		DEPOBJ(noarch/slower_corner_12)
	fi

	DEPOBJ(fast/fast_9_detect)
	DEPOBJ(fast/fast_9_score)
	DEPOBJ(fast/fast_corner_9_nonmax)

	DEPOBJ(fast/fast_10_detect)
	DEPOBJ(fast/fast_10_score)

	DEPOBJ(fast/fast_12_detect)
	DEPOBJ(fast/fast_12_score)
fi

if test "$have_fastobscure" == yes
then 
	DEPOBJ(fast/fast_7_detect)
	DEPOBJ(fast/fast_7_score)
	DEPOBJ(fast/slower_corner_7)
	DEPOBJ(fast/fast_8_detect)
	DEPOBJ(fast/fast_8_score)
	DEPOBJ(fast/slower_corner_8)
	DEPOBJ(fast/fast_11_detect)
	DEPOBJ(fast/fast_11_score)
	DEPOBJ(fast/slower_corner_11)
fi

################################################################################
#
# Video grabber dependencies
#

if test "$have_dc1394v2" == yes
then
	DEPOBJ(Linux/dvbuffer3_dc1394v2)
	DEPOBJ(videosource_dvbuffer)
else
	DEPOBJ(videosource_nodvbuffer)
fi

if test "$have_v4l2buffer" == yes
then
	DEPOBJ(Linux/v4lbuffer)
	DEPOBJ(Linux/videosource_v4lbuffer)
	DEPOBJ(Linux/v4lcontrol)
else
	DEPOBJ(Linux/videosource_nov4lbuffer)
fi

if test "$have_libuvc" == yes
then 
	DEPOBJ(uvcbuffer)
	DEPOBJ(videosource_uvcbuffer)
else
	DEPOBJ(videosource_nouvcbuffer)
fi


if test "$have_ffmpeg" == yes
then 
	DEPOBJ(videofilebuffer2)
	DEPOBJ(videofilebuffer_exceptions)
	DEPOBJ(videosource_videofilebuffer)
else
	DEPOBJ(videosource_novideofilebuffer)
fi

################################################################################
#
# Other library dependencies
#


if test "$have_toon" == yes
then 
	DEPOBJ(tensor_voting)
	DEPOBJ(brezenham)
	DEPOBJ(draw_toon)
	DEPOBJ(quartic)
	DEPOBJ(threepointpose)
fi

if test "$have_jpeg" == yes
then 
	DEPOBJ(image_io/jpeg)
fi

if test "$have_tiff" == yes
then 
	DEPOBJ(image_io/tiff)
	DEPOBJ(image_io/tiffwrite)
fi

if test "$have_png" == yes
then 
	DEPOBJ(image_io/png)
fi

if test "$have_videodisplay"
then 
	DEPOBJ(videodisplay)
	DEPOBJ(glwindow)
	DEPOBJ(gltext)
fi

AC_SUBST(dep_objects, [$dep_objects])

################################################################################
#
# Program dependencies
#

define(DEPPROG, [APPEND(progs, $1)])

if test "$have_toon$have_lapack" == yesyes
then
	DEPPROG(progs/se3_exp)
	DEPPROG(progs/se3_ln)
	DEPPROG(progs/se3_pre_mul)
	DEPPROG(progs/se3_post_mul)
	DEPPROG(progs/se3_inv)
fi

if test "$have_videodisplay" == yes
then 
	DEPPROG(progs/cvd_display_image)
	DEPPROG(progs/video_play_source)

	if test "$have_toon" == yes
	then
		DEPPROG(progs/calibrate)
	fi
fi

AC_SUBST(progs, [$progs])

echo
echo
echo Options:
echo $options $os_options
echo
echo Missing options for $host_os:
echo $options $all_options $os_all_options |  tr ' ' '\n' | sort | uniq -u | xargs echo
echo 
echo Dodgy things:
echo $dodginess_options 
echo
echo
echo SIMD support:
echo $simd_options
echo
echo Missing SIMD support for $host:
echo $simd_all_options $simd_options | tr ' ' '\n' |  sort | uniq -u | xargs echo
echo

AC_SUBST(CXXFLAGS, $CXXFLAGS)
AC_SUBST(OFLAGS, $OFLAGS)
	
echo $CXXFLAGS
echo $LIBS


AC_CONFIG_HEADERS([cvd/config.h cvd_src/config_internal.h])
AC_OUTPUT(Makefile)
