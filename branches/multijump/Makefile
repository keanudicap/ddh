VPATH = tests/util/:tests/hpa/:jump/:rsr/:hpa/:abstraction/:driver/:shared/:simulation/:util/:objs/:apps/libs:bin/

# source targets
ABSTRACTION_SRC = $(wildcard abstraction/*.cpp)
DRIVER_SRC = $(wildcard driver/*.cpp)
SHARED_SRC = $(wildcard shared/*.cpp)
SIMULATION_SRC = $(wildcard simulation/*.cpp)
UTIL_SRC = $(wildcard util/*.cpp)
HPASTAR_SRC = $(wildcard hpa/*.cpp)
RSR_SRC = $(wildcard rsr/*.cpp)
JUMP_SRC = $(wildcard jump/*.cpp)
UTILTESTS_SRC = $(wildcard tests/util/*.cpp)
HPASTARTESTS_SRC = $(wildcard tests/hpa/*.cpp)
RSRTESTS_SRC = $(wildcard tests/rsr/*.cpp)

# object targets
DRIVER_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(DRIVER_SRC))))
ABSTRACTION_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(ABSTRACTION_SRC))))
SHARED_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(SHARED_SRC))))
SIMULATION_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(SIMULATION_SRC))))
UTIL_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(UTIL_SRC))))
HPASTAR_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(HPASTAR_SRC))))
RSR_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(RSR_SRC))))
JUMP_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(JUMP_SRC))))
UTILTESTS_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(UTILTESTS_SRC))))
HPASTARTESTS_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(HPASTARTESTS_SRC))))
RSRTESTS_OBJ = $(subst .cpp,.o,$(addprefix objs/, $(notdir $(RSRTESTS_SRC))))

# header file locations
HOGINCLUDES = -I./jump -I./hpa -I./rsr -I./abstraction -I./driver -I./shared -I./simulation -I./util
TESTINCLUDES = -I./tests/util -I./tests/hpa -I./tests/rsr 

# compiler flags
CC = c++
FAST_CFLAGS = -O3 -ansi -DNDEBUG
DEV_CFLAGS = -Wall -Wno-long-long -Wno-deprecated -g -ggdb -ansi -pedantic

# locations of library files program depends on
ifeq ("$(OPENGL)", "STUB") 
  SYS_CFLAGS = -I./driver/STUB/ -I./driver/STUB/GL/ -DNO_OPENGL
else # OSX
ifeq ("$(findstring Darwin, "$(shell uname -s)")", "Darwin")
  SYS_CFLAGS = -DOS_MAC -I/opt/local/include/ -I/usr/local/include/ 
  SYS_CFLAGS += -I/System/Library/Frameworks/GLUT.framework/Versions/A/Headers/
  SYS_CFLAGS += -I/System/Library/Frameworks/OpenGL.framework/Versions/A/Headers/
  SYS_CFLAGS += -I/System/Library/Frameworks/AGL.framework/Versions/A/Headers/
  SYS_CFLAGS += -I/System/Library/Frameworks/Foundation.framework/Versions/A/Headers/
  SYS_CFLAGS += -I/System/Library/Frameworks/AppKit.framework/Versions/A/Headers/
  LIBFLAGS = -framework GLUT -framework OpenGL -framework AppKit -framework Foundation
else # Cygwin/Windows
ifeq ("$(findstring CYGWIN, $(shell uname -s))", "CYGWIN")
  SYS_CFLAGS = -Dlinux -I/usr/include/opengl
  LIBFLAGS = -Lapps/libs -lopengl32 -lglu32 -lglut32 -L/lib/w32api
else # Linux et al 
 LIBFLAGS = -Lapps/libs -lGL -lGLU -lglut -lXi -lXmu -L/usr/X11R6/lib64 -L/usr/X11R6/lib -L/usr/lib -L$(HOME)/lib -L/opt/local/lib -L/usr/local/lib
 SYS_CFLAGS = -Dlinux -I/usr/include/GL 
endif
endif
endif

ifeq ("$(CPU)", "G5")
 CFLAGS += -mcpu=970 -mpowerpc64 -mtune=970
 CFLAGS += -mpowerpc-gpopt -force_cpusubtype_ALL
endif

# every directory in ./apps, except those filtered out, is a target for compilation
TARGETS = $(filter-out %~ tests Makefile %.mk libs, $(notdir $(wildcard apps/*)))

all: fast
targets: $(TARGETS)

dev: CFLAGS = $(DEV_CFLAGS) $(SYS_CFLAGS) $(HOGINCLUDES) 
dev: APPSTARGET = dev
dev: $(TARGETS)

fast: CFLAGS = $(FAST_CFLAGS) $(SYS_CFLAGS) $(HOGINCLUDES)
fast: APPSTARGET = fast
fast: $(TARGETS)

$(TARGETS) : % : lib%.a hogcore
	$(CC) -o $(addprefix bin/,$(@)) \
		$(DRIVER_OBJ) $(UTIL_OBJ) $(SIMULATION_OBJ) $(ABSTRACTION_OBJ) $(SHARED_OBJ) \
		$(HPASTAR_OBJ) $(RSR_OBJ) $(JUMP_OBJ) \
		-l$(@:.mk=) $(LIBFLAGS)

$(addprefix lib, $(addsuffix .a, $(TARGETS))) :
	@echo "making app: "$(@)" ("$(APPSTARGET)")"
	@echo $(MAKE) -f $(patsubst lib%,%.mk,$(basename $(@))) OPENGL=$(OPENGL) $(@); cd ..
	@cd apps; $(MAKE) $(APPSTARGET); $(MAKE) -f $(patsubst lib%,%.mk,$(basename $(@))) OPENGL=$(OPENGL) $(@); cd ..

.PHONY: hogcore
hogcore : $(DRIVER_OBJ) $(UTIL_OBJ) $(SIMULATION_OBJ) $(ABSTRACTION_OBJ) $(SHARED_OBJ) \
	  $(HPASTAR_OBJ) $(RSR_OBJ) $(JUMP_OBJ)

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

$(RSR_OBJ) : $(RSR_SRC) $(RSR_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,rsr/,$(@)))

$(JUMP_OBJ) : $(JUMP_SRC) $(JUMP_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,jump/,$(@)))

.PHONY: tests
tests : hogcore $(UTILTESTS_OBJ) $(HPASTARTESTS_OBJ) \
		$(RSRTESTS_OBJ) libtests.a
	$(CC)	$(CFLAGS) $(LIBFLAGS) ${TESTLIBFLAGS} -o $(addprefix bin/,$(@)) \
		$(UTIL_OBJ) $(SIMULATION_OBJ) $(ABSTRACTION_OBJ) $(SHARED_OBJ) \
		$(HPASTAR_OBJ) $(RSR_OBJ) \
		$(HPASTARTESTS_OBJ) $(RSRTESTS_OBJ) \
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

$(RSRTESTS_OBJ) : $(RSRTESTS_SRC) $(RSRTESTS_SRC:.cpp=.h) \
	$(RSR_SRC) $(RSR_SRC:.cpp=.h)
	$(CC) $(CFLAGS) -c -o $(@) $(subst .o,.cpp, $(subst objs/,tests/rsr/,$(@)))

clean: cleanapps
	@-$(RM) objs/*.o
	@-$(RM) bin/*

cleanapps:
	@echo cleaning ./apps
	@cd apps; $(MAKE) clean; cd ..

cleantests: 
	@-$(RM) objs/*Test.o
	@-$(RM) bin/tests
	@cd apps; $(MAKE) clean; cd ..

.PHONY: tags
tags:
	ctags -R .
