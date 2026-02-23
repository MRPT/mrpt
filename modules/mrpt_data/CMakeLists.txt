cmake_minimum_required(VERSION 3.16)
project(mrpt_data LANGUAGES NONE)

# This package ships no code, only data files for unit tests and
# example configuration files.
#
# After find_package(mrpt_data), the following variable is defined:
#   mrpt_data_DATA_DIR  - Absolute path to the installed data root directory
#                         containing: config_files/, datasets/, tests/

# Install data files preserving directory structure:
install(DIRECTORY
  config_files
  datasets
  tests
  DESTINATION share/${PROJECT_NAME}
)

# Generate and install a CMake config file so that downstream packages
# can do find_package(mrpt_data) and get mrpt_data_DATA_DIR:
include(CMakePackageConfigHelpers)

# The config file will compute the data dir relative to its own location.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/mrpt_dataConfig.cmake"
"# mrpt_data CMake config file
# Provides: mrpt_data_DATA_DIR
get_filename_component(_mrpt_data_CONFIG_DIR \"\${CMAKE_CURRENT_LIST_DIR}\" ABSOLUTE)
get_filename_component(mrpt_data_DATA_DIR \"\${_mrpt_data_CONFIG_DIR}/../../../share/mrpt_data\" ABSOLUTE)
unset(_mrpt_data_CONFIG_DIR)
")

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mrpt_dataConfig.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)


# Silent cmake warning on unused variable:
set(dummy_ ${CMAKE_EXPORT_COMPILE_COMMANDS})
unset(dummy_)
