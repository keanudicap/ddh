WARTHOG_SRC = $(wildcard domains/*.cpp) $(wildcard util/*.cpp) $(wildcard search/*.cpp) $(wildcard experimental/*) $(wildcard heuristics/*)
WARTHOG_OBJ = $(subst .cpp,.o, $(addprefix obj/, $(notdir $(WARTHOG_SRC))))

D_WARTHOG_INCLUDES = -I./domains -I./util -I./search -I./experimental -I./heuristics
D_INCLUDES = -I/usr/include -I/opt/local/include $(D_WARTHOG_INCLUDES)
D_LIBS = -L/usr/local/lib -L./lib

CC = g++
CFLAGS = -ansi  -pedantic -Wall -Wno-long-long -Wno-deprecated -Werror
FAST_CFLAGS = -O3 -DNDEBUG
DEV_CFLAGS = -g -ggdb -O0
PROFILE_CFLAGS = -g -ggdb -O0 -DNDEBUG

.PHONY: all
all: dev

.PHONY: fast
fast: CFLAGS += $(FAST_CFLAGS) $(D_INCLUDES) $(D_LIBS)
fast: main

.PHONY: dev
dev: CFLAGS += $(DEV_CFLAGS) $(D_INCLUDES) $(D_LIBS)
dev: main

.PHONY: profile
profile: CFLAGS += $(PROFILE_CFLAGS) $(D_INCLUDES) $(D_LIBS)
profile: main

.PHONY: tags
tags:
	ctags -R .

.PHONY: clean
clean:
	@rm -rf ./bin/*
	@-$(RM) ./obj/*
	@-$(RM) ./lib/*

.PHONY: main
main: makedirs warthog
	$(CC) warthog.cpp -o ./bin/warthog -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	$(CC) tests.cpp -o ./bin/tests -lwarthog $(CFLAGS) $(D_LIBS) $(D_INCLUDES)
	#$(CC) warthog.cpp -o ./bin/warthog $(WARTHOG_SRC)  $(CFLAGS) $(D_LIBS) $(D_INCLUDES)

.PHONY: warthog
warthog: $(WARTHOG_SRC) 
	@echo "###  Archiving object files ###"
	@echo "sources "$(WARTHOG_SRC)
	ar -crs lib/lib$(@).a $(WARTHOG_OBJ)

.PHONY: makedirs
makedirs:
	@echo "### Creating output directories ###"
	@$(shell mkdir ./bin)
	@$(shell mkdir ./lib)
	@$(shell mkdir ./obj)

.PHONY: $(WARTHOG_SRC)
$(WARTHOG_SRC):
	@echo "compiling..."
	$(CC) -c $(@) -o $(subst .cpp,.o, $(addprefix obj/, $(notdir $(@)))) $(CFLAGS)

