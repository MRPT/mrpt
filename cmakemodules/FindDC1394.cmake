#
# try to find the dc1394 library (version 2) and include files
#
# DC1394_INCLUDE_DIR, where to find dc1394/dc1394_control.h, etc.
# DC1394_LIBRARIES, the libraries to link against to use DC1394.
# DC1394_FOUND, If false, do not try to use DC1394.
#


FIND_PATH( DC1394_INCLUDE_DIR dc1394/control.h
  /usr/include
  /usr/local/include
)

FIND_LIBRARY( DC1394_LIBRARY dc1394
  /usr/lib64
  /usr/lib
  /usr/local/lib
)


SET( DC1394_FOUND "NO" )
IF(DC1394_INCLUDE_DIR)
  IF(DC1394_LIBRARY)
    SET( DC1394_LIBRARIES
      ${DC1394_LIBRARY}
    )
    SET( DC1394_FOUND "YES" )

#The following deprecated settings are for backwards compatibility with CMake1.4
    SET (DC1394_INCLUDE_PATH ${DC1394_INCLUDE_DIR})

  ENDIF(DC1394_LIBRARY)
ENDIF(DC1394_INCLUDE_DIR)

