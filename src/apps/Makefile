VPATH = libs/
SHELL = /bin/bash
DAT = $(@)
VCC = $(CC)
VCFLG = $(CFLAGS)
DLR = $

APPS := $(filter-out %~ CVS Makefile %.mk tests libs,$(wildcard *))

TARGS = $(foreach x, $(APPS), $(addsuffix .mk, $(x)))

HDRS = $(foreach T, $(APPS), $(wildcard $(T)/*.h))

fast : CFLAGS = -O3 -ansi -DNDEBUG
fast : clean apps.mk

dev : CFLAGS = -Wall -Wno-deprecated -g -ggdb
dev : clean apps.mk

all : fast


$($(strip TARGS)) : $(HDRS) Makefile
	@echo '#' > $(@)
	@echo '# --- Auto-generated file for target $(basename $(@)) --- ' >> $(@)
	@echo '#' >> $(@)
	@echo >> $(@)
	@echo "include apps.mk" >> $(@)
	@echo "VPATH = $(basename $(@))/:$(basename $(@))/objs" >> $(@)
	@echo >> $(@)
	@echo $(basename $(@))_OBJS = $(notdir $(patsubst %.cpp, %.o, $(wildcard $(basename $(@))/*.cpp))) >> $(@)
	@echo >> $(@)
	@echo 'lib$(basename $(@)).a : $(value DLR)($(basename $(@))_OBJS)' >> $(@)
	@echo '	@if [[ ! -d libs ]]; then mkdir libs; fi;' >> $(@)
	@echo '	@ar -crs libs/$(value DAT) $(basename $(@))/objs/*.o' >> $(@)
#	@echo '	@ranlib libs/$(value DAT)' >> $(@)
	@echo >> $(@)
	@echo '$(value DLR)($(basename $(@))_OBJS)' \
 ' : %.o : %.cpp $(value DLR)(DRIVER_SRC:.cpp=.h) $(value DLR)(JUMP_SRC:.cpp=.h) $(value DLR)(RSR_SRC:.cpp=.h) $(value DLR)(HPASTAR_SRC:.cpp=.h) $(value DLR)(SHARED_SRC:.cpp=.h) $(value DLR)(UTIL_SRC:.cpp=.h) $(value DLR)(SIMULATION_SRC:.cpp=.h) $(value DLR)(RTS_OBJS:.o=.h)' >> $(@)
	@echo '	@if [[ ! -d $(basename $(@))/objs ]]; then mkdir $(basename $(@))/objs; fi;' >> $(@)
	@echo '	@-$(value DLR)(RM) libs/lib$(basename $(@)).a' >> $(@)
	@echo '	$(value DLR)(CC) $(value VCFLG) -c -o $(basename $(@))/objs/$(value DLR)(notdir $(value DAT)) $(basename $(@))/$(value DLR)(patsubst %.o,%.cpp,$(value DAT))' >> $(@)
	@echo >> $(@)



apps.mk : $(TARGS) Makefile
	@echo "CFLAGS="$(CFLAGS)
	@echo "#auto-generated file, do not edit!" > apps.mk
	@echo "SHELL = /bin/bash" >> apps.mk
	@echo "CC = g++" >> apps.mk
	@echo "CFLAGS = " $(CFLAGS) >> apps.mk
	@echo >> $(@)

	@echo '# locations of library files program depends on' >> apps.mk
	@echo 'ifeq ("$(value DLR)(OPENGL)", "STUB")' >> apps.mk
	@echo "  CFLAGS += -I./driver/STUB/ -I./driver/STUB/GL/ -DNO_OPENGL" >> apps.mk
	@echo "else # OSX" >> apps.mk
	@echo 'ifeq ("$(value DLR)(findstring Darwin, "$(value DLR)(shell uname -s)")", "Darwin")' >> apps.mk
	@echo "  CFLAGS += -DOS_MAC -I/opt/local/include/ -I/usr/local/include/ " >> apps.mk
	@echo "  CFLAGS += -I/System/Library/Frameworks/GLUT.framework/Versions/A/Headers/" >> apps.mk
	@echo "  CFLAGS += -I/System/Library/Frameworks/OpenGL.framework/Versions/A/Headers/" >> apps.mk
	@echo "  CFLAGS += -I/System/Library/Frameworks/AGL.framework/Versions/A/Headers/" >> apps.mk
	@echo "  CFLAGS += -I/System/Library/Frameworks/Foundation.framework/Versions/A/Headers/" >> apps.mk
	@echo "  CFLAGS += -I/System/Library/Frameworks/AppKit.framework/Versions/A/Headers/" >> apps.mk
	@echo "  LIBFLAGS = -framework GLUT -framework OpenGL -framework AppKit -framework Foundation" >> apps.mk
	@echo "else # Cygwin/Windows" >> apps.mk
	@echo 'ifeq ("$(value DLR)(findstring CYGWIN, $(value DLR)(shell uname -s))", "CYGWIN")' >> apps.mk
	@echo "  CFLAGS += -Dlinux -I/usr/include/opengl/ " >> apps.mk
	@echo "  LIBFLAGS = -Lapps/libs -lopengl32 -lglu32 -lglut32 -L/lib/w32api" >> apps.mk
	@echo "else # Linux et al " >> apps.mk
	@echo ' LIBFLAGS = -Lapps/libs -lGL -lGLU -lglut -lXi -lXmu -L/usr/X11R6/lib64 -L/usr/X11R6/lib -L/usr/lib -L$(value DLR)(HOME)/lib -L/opt/local/lib -L/usr/local/lib' >> apps.mk
	@echo " CFLAGS += -Dlinux -I/usr/include/GL " >> apps.mk
	@echo "endif" >> apps.mk
	@echo "endif" >> apps.mk
	@echo "endif" >> apps.mk

	@echo "CFLAGS += -I../jump -I../hpa -I../rsr -I../abstraction -I../driver -I../shared -I../simulation -I../util" >> apps.mk

	@echo >> $(@)
	@echo 'ABSTRACTION_SRC = $(value DLR)(wildcard ../abstraction/*.cpp)' >> $(@)
	@echo 'DRIVER_SRC = $(value DLR)(wildcard ../driver/*.cpp)' >> $(@)
	@echo 'SHARED_SRC = $(value DLR)(wildcard ../shared/*.cpp)' >> $(@)
	@echo 'SIMULATION_SRC = $(value DLR)(wildcard ../simulation/*.cpp)' >> $(@)
	@echo 'UTIL_SRC = $(value DLR)(wildcard ../util/*.cpp)' >> $(@)
	@echo 'HPASTAR_SRC = $(value DLR)(wildcard ../hpa/*.cpp)' >> $(@)
	@echo 'RSR_SRC = $(value DLR)(wildcard ../rsr/*.cpp)' >> $(@)
	@echo 'JUMP_SRC = $(value DLR)(wildcard ../jump/*.cpp)' >> $(@)

clean:
	@-$(RM) -f $(addsuffix /objs/*.o,$(APPS))
	@-$(RM) apps.mk $(TARGS)
	@-$(RM) $(addsuffix .a,$(addprefix libs/lib,$(APPS)))
	@-$(RM) libs/libtests.a;
	@-$(RM) tests/objs/RunTests.o
