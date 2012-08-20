include apps.mk
VPATH = entry_jps_plus/:entry_jps_plus/objs

entry_jps_plus_OBJS = main.o Entry.o ScenarioLoader.o Timer.o

.PHONY: fast
fast: CFLAGS=$(FAST_CFLAGS) $(_CFLAGS)
fast: libentry_jps_plus.a

.PHONY: dev
dev: CFLAGS=$(DEV_CFLAGS) $(_CFLAGS) 
dev: libentry_jps_plus.a

libentry_jps_plus.a :clean $(entry_jps_plus_OBJS)
	@if [[ ! -d libs ]]; then mkdir libs; fi;
	@ar -crs libs/$(@) entry_jps_plus/objs/*.o

$(entry_jps_plus_OBJS)  : 
	@if [[ ! -d entry_jps_plus/objs ]]; then mkdir entry_jps_plus/objs; fi;
	$(CC) -c -o entry_jps_plus/objs/$(notdir $(@)) entry_jps_plus/$(patsubst %.o,%.cpp,$(@)) $(CFLAGS) $(HOGCORE_INCLUDE)

clean:
	$(RM) entry_jps_plus/objs/*.o
	$(RM) libs/libentry_jps_plus.a
