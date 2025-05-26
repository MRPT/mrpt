/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#if defined(_MSC_VER)
#define STB_DISABLE_WARNINGS __pragma(warning(push)) __pragma(warning(disable : 4309))
#elif defined(__GNUC__)
#define STB_DISABLE_WARNINGS                                                            \
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wold-style-cast\"") \
      _Pragma("GCC diagnostic ignored \"-Wconversion\"")                                \
          _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")                       \
              _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")                  \
                  _Pragma("GCC diagnostic ignored \"-Wmissing-field-initializers\"")
#elif defined(__clang__)
#define STB_DISABLE_WARNINGS                                                                \
  _Pragma("clang diagnostic push") _Pragma("clang diagnostic ignored \"-Wold-style-cast\"") \
      _Pragma("clang diagnostic ignored \"-Wconversion\"")                                  \
          _Pragma("clang diagnostic ignored \"-Wsign-conversion\"")                         \
              _Pragma("clang diagnostic ignored \"-Wdouble-promotion\"")                    \
                  _Pragma("clang diagnostic ignored \"-Wmissing-field-initializers\"")
#endif

#if defined(_MSC_VER)
#define STB_RESTORE_WARNINGS __pragma(warning(pop))
#elif defined(__GNUC__)
#define STB_RESTORE_WARNINGS _Pragma("GCC diagnostic pop")
#elif defined(__clang__)
#define STB_RESTORE_WARNINGS _Pragma("clang diagnostic pop")
#endif
