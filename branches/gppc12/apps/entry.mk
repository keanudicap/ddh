include apps.mk
VPATH = entry/:entry/objs

entry_OBJS = main.o Entry.o

.PHONY: fast
fast: CFLAGS=$(FAST_CFLAGS) $(_CFLAGS)
fast: libentry.a

.PHONY: dev
dev: CFLAGS=$(DEV_CFLAGS) $(_CFLAGS) 
dev: libentry.a

libentry.a : clean $(entry_OBJS)
	@if [[ ! -d libs ]]; then mkdir libs; fi;
	@ar -crs libs/$(@) entry/objs/*.o

$(entry_OBJS)  : 
	@if [[ ! -d entry/objs ]]; then mkdir entry/objs; fi;
	$(CC) $(CFLAGS) -c -o entry/objs/$(notdir $(@)) entry/$(patsubst %.o,%.cpp,$(@))

clean:
	$(RM) entry/objs/*.o
	$(RM) libs/libentry.a
