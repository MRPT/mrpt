include ../Makefile.inc

SHELL	= /bin/sh

OBJLIBS	= libxstypes.a 
CPP_FILES := $(wildcard src/*.c)
OBJECTS := $(CPP_FILES:.c=.o)
INCLUDE := -Iinclude/xsens
CFLAGS:= -g -c -include include/xsens/xstypesconfig.h

all : $(OBJLIBS) 

.c.o:
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@
	
$(OBJLIBS) : $(OBJECTS)
	ar rcs $@ $^
	
clean :
	$(ECHO) cleaning up in libxstypes
	-$(RM) -f $(OBJECTS)
	-$(RM) -f $(OBJLIBS)
	
