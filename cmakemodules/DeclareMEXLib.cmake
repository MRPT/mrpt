macro(define_mex_lib name)
        # 32-bit or 64-bit mex
        if(WIN32)
          if (CMAKE_CL_64)
                  SET_TARGET_PROPERTIES(${name} 
					PROPERTIES SUFFIX _.mexw64 
					LIBRARY_OUTPUT_DIRECTORY "${MEX_LIBRARY_OUTPUT_PATH}${ARGN}"
					RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
					)
          else(CMAKE_CL_64)
                  SET_TARGET_PROPERTIES(${name} 
				  PROPERTIES SUFFIX _.mexw32 
				  LIBRARY_OUTPUT_DIRECTORY "${MEX_LIBRARY_OUTPUT_PATH}${ARGN}"
				  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
				  )
          endif(CMAKE_CL_64)
        else(WIN32)
          if (CMAKE_SIZEOF_VOID_P MATCHES "8")
                  SET_TARGET_PROPERTIES(${name} PROPERTIES 
				  SUFFIX _.mexa64 PREFIX "" 
				  LIBRARY_OUTPUT_DIRECTORY "${MEX_LIBRARY_OUTPUT_PATH}${ARGN}"
				  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
				  )
          else(CMAKE_SIZEOF_VOID_P MATCHES "8")
                  SET_TARGET_PROPERTIES(${name} PROPERTIES 
				  SUFFIX _.mexglx PREFIX "" 
				  LIBRARY_OUTPUT_DIRECTORY "${MEX_LIBRARY_OUTPUT_PATH}${ARGN}"
				  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
				  )
          endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
        endif(WIN32)
endmacro(define_mex_lib)

macro(define_mex_test name)
        SET_TARGET_PROPERTIES(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${MEX_EXECUTABLE_OUTPUT_PATH})
endmacro(define_mex_test name)
