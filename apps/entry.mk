include apps.mk
VPATH = entry/:entry/objs

entry_OBJS = main.o Entry.o ScenarioLoader.o Timer.o

.PHONY: fast
fast: CFLAGS=$(FAST_CFLAGS) $(_CFLAGS)
fast: libentry.a

.PHONY: dev
dev: CFLAGS=$(DEV_CFLAGS) $(_CFLAGS) 
dev: libentry.a

libentry.a :clean $(entry_OBJS)
	@if [[ ! -d libs ]]; then mkdir libs; fi;
	@ar -crs libs/$(@) entry/objs/*.o

$(entry_OBJS)  : 
	@if [[ ! -d entry/objs ]]; then mkdir entry/objs; fi;
	$(CC) -c -o entry/objs/$(notdir $(@)) entry/$(patsubst %.o,%.cpp,$(@)) $(CFLAGS) $(HOGCORE_INCLUDE)

clean:
	$(RM) entry/objs/*.o
	$(RM) libs/libentry.a
