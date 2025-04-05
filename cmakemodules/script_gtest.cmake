# By default, use embedded version of gtest, unless it is found in the system:

find_package(GTest QUIET)

if (GTest_FOUND)
	set(CMAKE_MRPT_HAS_GTEST         1 CACHE INTERNAL "")
	set(CMAKE_MRPT_HAS_GTEST_SYSTEM  1 CACHE INTERNAL "")
	# We can use the imported targets: GTest::GTest and GTest::Main
 else()
	set(CMAKE_MRPT_HAS_GTEST         1 CACHE INTERNAL "")
	set(CMAKE_MRPT_HAS_GTEST_SYSTEM  0 CACHE INTERNAL "")
	set(CMAKE_MRPT_GTEST_SRC_DIR     "${MRPT_SOURCE_DIR}/3rdparty/googletest/googletest/" CACHE INTERNAL "")
endif()
