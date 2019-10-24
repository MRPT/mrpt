# Idea: this cmake script just sets the global property to reflect whether we have or not this dependency.
# Interested modules will re-include this file again to have all variables defined.
# We don't just include this one at root cmake file because PCL cmake module
# is not well-behaved and introduces *huge* amounts of polution in global build flags, etc.
# even in parts where we don't use PCL.

project(probe_pcl)
include(${MRPT_SOURCE_DIR}/cmakemodules/script_pcl.cmake REQUIRED)
