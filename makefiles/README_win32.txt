 The "makefiles" directory keeps the "Project" files for Windows Compilers 
(Visual C++, MinGW...), grouped by compiler and version, 
and the scripts to invoke CMake and generate those Project files automatically.

 Since October 2007, the MRPT has moved to "CMake" (http://www.cmake.org/), a 
tool for generating the Makefiles in your system as an alternative, cross-platform
version of GNU autoconf. 

 ***** You need CMake installed in your system before compiling the MRPT. *****

   Execute the batch file for your compiler, for example, "win32_rebuild_MSVC8_GUI.bat",
   then use the solution file (.sln) from the corresponding directory (e.g. "MSVC8").
   Of course, any other path can be chosen arbitraryly. 
   
   
 **NOTE**: These .bat files assume "cmake-gui" is in the PATH. 

 
   	Also, see the general instructions: 
 http://www.mrpt.org/Building_and_Installing_Instructions  
 

JLBC 
