SHELL = /bin/bash
CC = g++

FAST_CFLAGS =  -O3 -ansi -DNDEBUG
DEV_CFLAGS =  -Wall -Wno-long-long -Wno-deprecated -g -ggdb -ansi -pedantic

# locations of library files program depends on
ifeq ("$(OPENGL)", "STUB")
  _CFLAGS += -I../driver/STUB/ -I../driver/STUB/GL/ -DNO_OPENGL
else # OSX
ifeq ("$(findstring Darwin, "$(shell uname -s)")", "Darwin")
  _CFLAGS += -DOS_MAC -I/opt/local/include/ -I/usr/local/include/ 
  _CFLAGS += -I/System/Library/Frameworks/GLUT.framework/Versions/A/Headers/
  _CFLAGS += -I/System/Library/Frameworks/OpenGL.framework/Versions/A/Headers/
  _CFLAGS += -I/System/Library/Frameworks/AGL.framework/Versions/A/Headers/
  _CFLAGS += -I/System/Library/Frameworks/Foundation.framework/Versions/A/Headers/
  _CFLAGS += -I/System/Library/Frameworks/AppKit.framework/Versions/A/Headers/
  LIBFLAGS = -framework GLUT -framework OpenGL -framework AppKit -framework Foundation
else # Cygwin/Windows
ifeq ("$(findstring CYGWIN, $(shell uname -s))", "CYGWIN")
  _CFLAGS += -Dlinux -I/usr/include/opengl
  LIBFLAGS = -Lapps/libs -lopengl32 -lglu32 -lglut32 -L/lib/w32api
else # Linux et al 
 LIBFLAGS = -Lapps/libs -lGL -lGLU -lglut -lXi -lXmu -L/usr/X11R6/lib64 -L/usr/X11R6/lib -L/usr/lib -L$(HOME)/lib -L/opt/local/lib -L/usr/local/lib
 _CFLAGS += -Dlinux -I/usr/include/GL 
endif
endif
endif
_CFLAGS += -I../jump -I../hpa -I../rsr -I../abstraction -I../driver -I../shared -I../simulation -I../util -I../competition

ABSTRACTION_HDR = $(wildcard ../abstraction/*.h)
DRIVER_HDR = $(wildcard ../driver/*.h)
SHARED_HDR = $(wildcard ../shared/*.h)
SIMULATION_HDR = $(wildcard ../simulation/*.h)
UTIL_HDR = $(wildcard ../util/*.h)
HPASTAR_HDR = $(wildcard ../hpa/*.h)
RSR_HDR = $(wildcard ../rsr/*.h)
JUMP_HDR = $(wildcard ../jump/*.h)
COMPETITION_HDR = $(wildcard ../competition/*.h)

HOGCORE_HDR = $(UTIL_HDR) $(SIMULATION_HDR) $(ABSTRACTION_HDR) \
		$(SHARED_HDR) $(HPASTAR_HDR) $(RSR_HDR) $(JUMP_HDR) $(COMPETITION_HDR)
