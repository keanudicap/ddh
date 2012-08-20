include apps.mk
VPATH = entry_jps/:entry_jps/objs

entry_jps_OBJS = main.o Entry.o ScenarioLoader.o Timer.o

.PHONY: fast
fast: CFLAGS=$(FAST_CFLAGS) $(_CFLAGS)
fast: libentry_jps.a

.PHONY: dev
dev: CFLAGS=$(DEV_CFLAGS) $(_CFLAGS) 
dev: libentry_jps.a

libentry_jps.a :clean $(entry_jps_OBJS)
	@if [[ ! -d libs ]]; then mkdir libs; fi;
	@ar -crs libs/$(@) entry_jps/objs/*.o

$(entry_jps_OBJS)  : 
	@if [[ ! -d entry_jps/objs ]]; then mkdir entry_jps/objs; fi;
	$(CC) -c -o entry_jps/objs/$(notdir $(@)) entry_jps/$(patsubst %.o,%.cpp,$(@)) $(CFLAGS) $(HOGCORE_INCLUDE)

clean:
	$(RM) entry_jps/objs/*.o
	$(RM) libs/libentry_jps.a
