include apps.mk
VPATH = hog/:hog/objs

hog_OBJS = hog.o

.PHONY: fast
fast: CFLAGS=$(FAST_CFLAGS) $(_CFLAGS)
fast: libhog.a

.PHONY: dev
dev: CFLAGS=$(DEV_CFLAGS) $(_CFLAGS) 
dev: libhog.a

libhog.a : clean $(hog_OBJS)
	@if [[ ! -d libs ]]; then mkdir libs; fi;
	@ar -crs libs/$(@) hog/objs/*.o

$(hog_OBJS)  : 
	@if [[ ! -d hog/objs ]]; then mkdir hog/objs; fi;
	$(CC) $(CFLAGS) -c -o hog/objs/$(notdir $(@)) hog/$(patsubst %.o,%.cpp,$(@))

clean:
	@$(RM) hog/objs/*.o
	@$(RM) libs/libhog.a
