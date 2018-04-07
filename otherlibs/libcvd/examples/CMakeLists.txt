link_libraries(CVD)
project(examples)

# if(CVD_HAVE_TOON)
# 	add_executable(tensor_voting tensor_voting.cc)
# endif()

add_executable(colourmaps colourmaps.cc)
add_executable(distance_transform distance_transform.cc)

