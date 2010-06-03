# Makefile
# Invoke with "make" or "make clean"
 
#### Start of system configuration section. ####
# Use the g++ compiler:
CC = gcc
LINKER = gcc -O
 
DEFS = `VFlib3-config --cflags`
LIBS = `VFlib3-config --libs`
 
CDEBUG = 
CFLAGS = -Wall -pedantic -I.  $(DEFS)
LDFLAGS = $(LIBS) 
 
#### End of system configuration section. ####
TARGET = fonts2cpp
SRCS =  main.c
OBJS =  main.c
 
all:    $(TARGET)
$(TARGET): $(OBJS)
	$(LINKER) -o $@ $(OBJS) $(LDFLAGS)
.cpp.o: 
	$(CC) $(CFLAGS) -c $<  
clean:
	rm -f *.o $(TARGET)

