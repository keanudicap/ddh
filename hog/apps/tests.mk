
# --- Auto-generated file for target tests --- 
#

include apps.mk
VPATH = tests/:tests/objs

CFLAGS += -I../hpa -I../tests/aha -I../tests/hpa -I/usr/local/include -I/opt/local/include

tests_OBJS = RunTests.o

libtests.a : $(clean) $(tests_OBJS)
	@if (! -d libs) mkdir libs
	@ar -crs libs/$(@) tests/objs/*.o

$(tests_OBJS)  : %.o : %.cpp $(HPASTARTESTS_SRC:.cpp=.h) $(HPASTAR_SRC:.cpp=.h) $(HPASTARTESTS_SRC:.cpp=.h) $(HPASTAR_SRC:.cpp=.h) $(SHARED_SRC:.cpp=.h) $(UTIL_SRC:.cpp=.h) $(SIMULATION_SRC:.cpp=.h) $(RTS_OBJS:.o=.h)
	@if (! -d tests/objs) mkdir tests/objs
	@-$(RM) libs/libtests.a
	$(CC) $(CFLAGS) -c -o tests/objs/$(notdir $(@)) tests/$(patsubst %.o,%.cpp,$(@))

clean: 
	rm -f libs/libtests.a tests/objs/*.o
