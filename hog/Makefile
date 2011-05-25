VPATH = tests/util/:tests/hpa/:tests/aha/:jump/:opthpa/:hpa/:aha/:abstraction/:driver/:shared/:simulation/:util/:objs/:apps/libs:bin/

# source targets
ABSTRACTION_SRC = $(wildcard abstraction/*.cpp)
DRIVER_SRC = $(wildcard driver/*.cpp)
SHARED_SRC = $(wildcard shared/*.cpp)
SIMULATION_SRC = $(wildcard simulation/*.cpp)
UTIL_SRC = $(wildcard util/*.cpp)
HPASTAR_SRC = $(wildcard hpa/*.cpp)
OPTHPA_SRC = $(wildcard opthpa/*.cpp)
JUMP_SRC = $(wildcard jump/*.cpp)
UTILTESTS_SRC = $(wildcard tests/util/*.cpp)
HPASTARTESTS_SRC = $(wildcard tests/hpa/*.cpp)
OPTHPATESTS_SRC = $(wildcard tests/opthpa/*.cpp)

# object targets
DRIVER_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(DRIVER_SRC))))
ABSTRACTION_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(ABSTRACTION_SRC))))
SHARED_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(SHARED_SRC))))
SIMULATION_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(SIMULATION_SRC))))
UTIL_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(UTIL_SRC))))
HPASTAR_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(HPASTAR_SRC))))
OPTHPA_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(OPTHPA_SRC))))
JUMP_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(JUMP_SRC))))
UTILTESTS_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(UTILTESTS_SRC))))
HPASTARTESTS_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(HPASTARTESTS_SRC))))
OPTHPATESTS_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(OPTHPATESTS_SRC))))

# header file locations
HOGINCLUDES = -I./jump -I./hpa -I./aha -I./opthpa -I./abstraction -I./driver -I./shared -I./simulation -I./util
TESTINCLUDES = -I./tests/util -I./tests/aha -I./tests/hpa -I./tests/opthpa 

# compiler flags
CC = c++
CFLAGS = -Wall -Wno-long-long -g -ggdb -ansi -pedantic $(HOGINCLUDES) $(TESTINCLUDES)
#CFLAGS = -O3 $(HOGINCLUDES) -ansi

# locations of library files program depends on
LIBFLAGS = -Lapps/libs 

# platform specific header/library paths
ifeq ($(findstring "Darwin", "$(shell uname -s)"), "Darwin")
 TESTLIBFLAGS = -L/opt/local/lib -L/usr/local/lib -lcppunit -lmockpp
 CFLAGS += -DOS_MAC -I/opt/local/include/ -I/usr/local/include/ -DUNITTEST
 ifeq ("$(OPENGL)", "STUB")
  CFLAGS = -I./driver/STUB/ -I./driver/STUB/GL/ -DNO_OPENGL
 else
  LIBFLAGS += -framework GLUT -framework OpenGL -framework AppKit -framework Foundation
  CFLAGS += -I/System/Library/Frameworks/GLUT.framework/Versions/A/Headers/
  CFLAGS += -I/System/Library/Frameworks/OpenGL.framework/Versions/A/Headers/
  CFLAGS += -I/System/Library/Frameworks/AGL.framework/Versions/A/Headers/
  CFLAGS += -I/System/Library/Frameworks/Foundation.framework/Versions/A/Headers/
  CFLAGS += -I/System/Library/Frameworks/AppKit.framework/Versions/A/Headers/
 endif
else # not darwin
LIBFLAGS = -Lapps/libs -L/usr/X11R6/lib64 -L/usr/X11R6/lib -L/usr/lib -L$(HOME)/lib -L/opt/local/lib -L/usr/local/lib
 ifeq ("$(OPENGL)", "STUB")
  CFLAGS += -I./driver/STUB/ -I./driver/STUB/GL/ -DNO_OPENGL
 else
  CFLAGS += -I/usr/include/GL 
  LIBFLAGS +=  -lGL -lGLU -lglut -lXi -lXmu  
 endif
 CFLAGS += -Dlinux
endif

ifeq ("$(CPU)", "G5")
 CFLAGS += -mcpu=970 -mpowerpc64 -mtune=970
 CFLAGS += -mpowerpc-gpopt -force_cpusubtype_ALL
endif

# every directory in ./apps, except those filtered out, is a target for compilation
TARGETS = $(filter-out %~ Makefile %.mk tests libs, $(notdir $(wildcard apps/*)))

all: cleanapps $(TARGETS)

targets: cleanapps $(TARGETS)

$(TARGETS) : % : lib%.a hogcore
	$(CC)	$(CFLAGS) $(LIBFLAGS) -o $(addprefix bin/,$(@)) \
		$(DRIVER_OBJ) $(UTIL_OBJ) $(SIMULATION_OBJ) $(ABSTRACTION_OBJ) $(SHARED_OBJ) \
		$(HPASTAR_OBJ) $(OPTHPA_OBJ) $(JUMP_OBJ) \
		-l$(@:.mk=)

$(addprefix lib, $(addsuffix .a, $(TARGETS))) :
	@echo $(MAKE) -f $(patsubst lib%,%.mk,$(basename $(@))) OPENGL=$(OPENGL) $(@); cd ..
	@cd apps; $(MAKE); $(MAKE) -f $(patsubst lib%,%.mk,$(basename $(@))) OPENGL=$(OPENGL) $(@); cd ..

.PHONY: hogcore
hogcore : $(DRIVER_OBJ) $(UTIL_OBJ) $(SIMULATION_OBJ) $(ABSTRACTION_OBJ) $(SHARED_OBJ) \
	  $(HPASTAR_OBJ) $(OPTHPA_OBJ) $(JUMP_OBJ)

$(UTIL_OBJ) : $(UTIL_SRC) $(UTIL_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,util/,$(@)))

$(SIMULATION_OBJ) : $(SIMULATION_SRC) $(SIMULATION_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,simulation/,$(@)))

$(ABSTRACTION_OBJ) : $(ABSTRACTION_SRC) $(ABSTRACTION_SRC.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,abstraction/,$(@)))

$(SHARED_OBJ) : $(SHARED_SRC) $(SHARED_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,shared/,$(@)))

$(DRIVER_OBJ) : $(DRIVER_SRC) $(DRIVER_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,driver/,$(@)))

$(HPASTAR_OBJ) : $(HPASTAR_SRC) $(HPASTAR_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,hpa/,$(@)))

$(OPTHPA_OBJ) : $(OPTHPA_SRC) $(OPTHPA_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,opthpa/,$(@)))

$(JUMP_OBJ) : $(JUMP_SRC) $(JUMP_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,jump/,$(@)))

.PHONY: tests
tests : hogcore $(UTILTESTS_OBJ) $(HPASTARTESTS_OBJ) \
		$(OPTHPATESTS_OBJ) libtests.a
	$(CC)	$(CFLAGS) $(LIBFLAGS) ${TESTLIBFLAGS} -o $(addprefix bin/,$(@)) \
		$(UTIL_OBJ) $(SIMULATION_OBJ) $(ABSTRACTION_OBJ) $(SHARED_OBJ) \
		$(HPASTAR_OBJ) $(OPTHPA_OBJ) \
		$(HPASTARTESTS_OBJ) $(OPTHPATESTS_OBJ) \
		-l$(@)

# build this separately so we don't require *TEST_OBJ dependencies
.PHONY: libtests.a
libtests.a :
	@cd apps; $(MAKE); $(MAKE) -f tests.mk clean; $(MAKE) -f tests.mk OPENGL=$(OPENGL) $(@); cd ..

$(UTILTESTS_OBJ) : $(UTILTESTS_SRC) $(UTILTESTS_SRC:.cpp=.h) \
	$(UTIL_SRC) $(UTIL_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,tests/util/,$(@)))

$(HPASTARTESTS_OBJ) : $(HPASTARTESTS_SRC) $(HPASTARTESTS_SRC:.cpp=.h) \
	$(HPASTAR_SRC) $(HPASTAR_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,tests/hpa/,$(@)))

$(OPTHPATESTS_OBJ) : $(OPTHPATESTS_SRC) $(OPTHPATESTS_SRC:.cpp=.h) \
	$(OPTHPA_SRC) $(OPTHPA_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,tests/opthpa/,$(@)))

clean:
	@-$(RM) objs/*.o
	@-$(RM) bin/*
	@cd apps; $(MAKE) clean; cd ..

cleanapps:
	@cd apps; $(MAKE) clean; cd ..

cleantests: 
	@-$(RM) objs/*Test.o
	@-$(RM) bin/tests
	@cd apps; $(MAKE) clean; cd ..

.PHONY: tags
tags:
	ctags -R .
