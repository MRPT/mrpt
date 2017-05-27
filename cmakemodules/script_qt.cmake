# Check for Qt
#  If it is found, will set CMAKE_MRPT_HAS_Qt5=1

set(BUILD_QT ON CACHE BOOL "Build Qt")
set(CMAKE_MRPT_HAS_Qt5 0)

if (BUILD_QT)
    set(QT_MRPT_COMPONENTS_TO_SEARCH "Widgets;Core" CACHE STRING "Components to search in Qt")

    find_package(Qt5 COMPONENTS ${QT_MRPT_COMPONENTS_TO_SEARCH})

    if (Qt5_FOUND)
        set(CMAKE_MRPT_HAS_Qt5 1)
    endif(Qt5_FOUND)

endif(BUILD_QT)
