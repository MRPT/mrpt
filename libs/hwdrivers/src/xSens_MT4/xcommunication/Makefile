include ../Makefile.inc

SHELL	= /bin/sh

OBJLIBS	= libxcommunication.a
CPP_FILES := $(wildcard src/*.cpp)
C_FILES := $(wildcard src/*.c)
OBJECTS := $(CPP_FILES:.cpp=.o) $(C_FILES:.c=.o)
INCLUDE := -Iinclude/xsens -I../xstypes/include
CFLAGS:= -g -c -include include/xsens/xcommunicationconfig.h

all : $(OBJLIBS)

.cpp.o:
	$(CXX) $(INCLUDE) $(CFLAGS) $< -o $@
	
.c.o:
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJLIBS) : $(OBJECTS)
	ar rcs $@ $^
	
clean :
	$(ECHO) cleaning up in libxcommunication
	-$(RM) -f $(OBJECTS)
	-$(RM) -f $(OBJLIBS)
	