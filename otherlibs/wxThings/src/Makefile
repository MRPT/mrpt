# ----------------------------------------------------------------------------
# Purpose : Makefile for wxthings lib - UNIX or cygwin using gmake (gcc)
# Author : John Labenski
# Copyright : John Labenski 2006
# License : wxWidgets license
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# wxWidgets build settings

WXCONFIG   := wx-config

WXDIR      := $(shell $(WXCONFIG) --prefix)
WXEXECDIR  := $(shell $(WXCONFIG) --exec-prefix)
WXLIBDIR   := $(WXEXECDIR)/lib

WXBASENAME := $(shell $(WXCONFIG) --basename)
WXVERSION  := $(shell $(WXCONFIG) --version)
WXRELEASE  := $(shell $(WXCONFIG) --release)

WXCXX      := $(shell $(WXCONFIG) --cxx)
WXCXXFLAGS := $(shell $(WXCONFIG) --cxxflags)
WXLIBS     := $(shell $(WXCONFIG) --libs)

WXSTC_LIB  := $(WXBASENAME)_stc-$(WXRELEASE)

# ----------------------------------------------------------------------------
# Settings for the library

WXTHINGS_LIBDIR  = $(WXLIBDIR)
WXTHINGS_LIBNAME = $(WXBASENAME)_wxthings-$(WXRELEASE)
WXTHINGS_LIBNAME := $(patsubst wx_%,wxcode_%,$(WXTHINGS_LIBNAME))

LIBVERSION_CURRENT  = 1
LIBVERSION_REVISION = 0
LIBVERSION_AGE      = 0

TARGET_LIBNAME = lib$(WXTHINGS_LIBNAME)
TARGET_LIBNAME_STATIC = $(TARGET_LIBNAME).a
TARGET_LIBNAME_SHARED = $(TARGET_LIBNAME).so.$(LIBVERSION_CURRENT).$(LIBVERSION_REVISION).$(LIBVERSION_AGE)
TARGET_LIB_STATIC = $(WXTHINGS_LIBDIR)/$(TARGET_LIBNAME_STATIC)
TARGET_LIB_SHARED = $(WXTHINGS_LIBDIR)/$(TARGET_LIBNAME_SHARED)
TARGET_LIB_LINK1  = $(TARGET_LIBNAME).so.$(LIBVERSION_CURRENT)
TARGET_LIB_LINK2  = $(TARGET_LIBNAME).so

# ----------------------------------------------------------------------------
# find the GTK headers

# in filebrws.cpp we need to resize comboboxes in toolbar
ifndef GTK_CFLAGS
    ifeq (gtk2, $(findstring gtk2, $(WXBASENAME)))
        GTK_CFLAGS := $(shell pkg-config --cflags gtk+-2.0)
    else
        ifeq (gtk, $(findstring gtk, $(WXBASENAME)))
            GTK_CFLAGS := $(shell gtk-config --cflags)
        else
            GTK_CFLAGS =
        endif
    endif
endif

# ----------------------------------------------------------------------------
# the wxstedit component is used for the wxFileBrowser class

WXSTEDIT_DIR = ../../wxstedit
WXSTEDIT_LIBNAME = $(WXBASENAME)_stedit-$(WXRELEASE)
WXSTEDIT_LIBNAME := $(patsubst wx_%,wxcode_%,$(WXSTEDIT_LIBNAME))

# ----------------------------------------------------------------------------
# CXX flags and libs

CXXFLAGS = $(WXCXXFLAGS) $(GTK_CFLAGS) -MMD -g -Wall -Wunused -Wunused-parameter -fPIC -DPIC -Wextra -I$(WXDIR)/src
LDLIBS   = $(WXLIBS)
CXX      = $(WXCXX)

#-fdump-class-hierarchy  this dumps out a file.cpp.class file
# -pg in libs and defs for profiling
PROFILE = #-pg
APPEXTRADEFS = -I../include -I$(WXDIR)/contrib/include -I$(WXSTEDIT_DIR)/include $(PROFILE)
APPEXTRALIBS = -l$(WXSTC_LIB) -l$(WXSTEDIT_LIBNAME) $(PROFILE)

# ----------------------------------------------------------------------------
# Programs to use

RM        = rm -f
AR        = ar
AROPTIONS = rcu
RANLIB    = ranlib
LN_S      = ln -s

# ----------------------------------------------------------------------------
# Source files, objects to link

OBJECTS = \
	block.o \
	bmpcombo.o \
	dropdown.o \
	filebrws.o \
	filedlgg.o \
	genergdi.o \
	geometry.o \
	matrix2d.o \
	menubtn.o \
	optvalue.o \
	range.o \
	spinctld.o \
	toggle.o

DEPFILES := $(OBJECTS:.o=.d)

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(APPEXTRADEFS) -o $@ $<

all: $(TARGET_LIB_STATIC) $(TARGET_LIB_SHARED)

$(TARGET_LIB_STATIC) : $(OBJECTS)
	@$(RM) $(TARGET_LIB_STATIC)
	$(AR) $(AROPTIONS) $(TARGET_LIB_STATIC) $(OBJECTS)
	$(RANLIB) $(TARGET_LIB_STATIC)

$(TARGET_LIB_SHARED) : $(OBJECTS)
	$(CXX) -shared -o $(TARGET_LIB_SHARED) $(OBJECTS) $(WXLIBS)
	cd $(WXTHINGS_LIBDIR) \
	&& $(RM) $(TARGET_LIB_LINK1) $(TARGET_LIB_LINK2) \
	&& $(LN_S) $(TARGET_LIB_SHARED) $(TARGET_LIB_LINK1) \
	&& $(LN_S) $(TARGET_LIB_SHARED) $(TARGET_LIB_LINK2)

clean:
	rm -f $(OBJECTS) $(DEPFILES) core \
		$(TARGET_LIB_STATIC) \
		$(TARGET_LIB_SHARED) \
		$(WXTHINGS_LIBDIR)/$(TARGET_LIB_LINK1) \
		$(WXTHINGS_LIBDIR)/$(TARGET_LIB_LINK2)


-include $(DEPFILES)
