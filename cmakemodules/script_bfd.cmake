# Check for system BFD library for debug symbols (GNU/Linux only)
# ===================================================
set(CMAKE_MRPT_HAS_BFD 0)
set(CMAKE_MRPT_HAS_BFD_SYSTEM 0)

option(DISABLE_BFD "Force not using BFD library" "OFF")
mark_as_advanced(DISABLE_BFD)
if(DISABLE_BFD)
	return()
endif()

include(CheckIncludeFile)
include(CheckSymbolExists)
include(CheckPrototypeDefinition)
include(CheckCSourceCompiles)

CHECK_INCLUDE_FILE("bfd.h" HAVE_BFD_H)
# Prefer static libraries, per:
# https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=976803
find_library(BFD_LIBRARY NAMES libbfd.a)
mark_as_advanced(BFD_LIBRARY)

if ((NOT HAVE_BFD_H) OR (NOT BFD_LIBRARY))
	# We need both, the .h and the .a to consider we have bfd available
	return()
endif()

# Other Deps of BFD (needed since we are enforced to use static linking)
find_library(Z_LIBRARY NAMES z libz.so libz)
mark_as_advanced(Z_LIBRARY)

find_library(IBERTY_LIBRARY NAMES iberty libiberty.a) 

set(BFD_LIBRARIES ${BFD_LIBRARY} ${Z_LIBRARY} ${CMAKE_DL_LIBS} ${IBERTY_LIBRARY})

# Ok, we have the library, now detect different API versions:
set(CMAKE_MRPT_HAS_BFD 1)
set(CMAKE_MRPT_HAS_BFD_SYSTEM 1)

check_symbol_exists(bfd_get_section_flags  "bfd.h" HAVE_DECL_BFD_GET_SECTION_FLAGS)
check_symbol_exists(bfd_section_flags      "bfd.h" HAVE_DECL_BFD_SECTION_FLAGS)
check_symbol_exists(bfd_get_section_vma    "bfd.h" HAVE_DECL_BFD_GET_SECTION_VMA)
check_symbol_exists(bfd_section_vma        "bfd.h" HAVE_DECL_BFD_SECTION_VMA)

set(CMAKE_REQUIRED_LINK_OPTIONS ${BFD_LIBRARY})
check_c_source_compiles("\
#include <stddef.h>\n\
#include <bfd.h>\n\
int main(int ac, char**av){
const asection *sec = NULL; \
bfd_size_type a = bfd_section_size(sec); \
(void)a;\
return 0;\
}"
HAVE_1_ARG_BFD_SECTION_SIZE)
unset(CMAKE_REQUIRED_LINK_OPTIONS)
