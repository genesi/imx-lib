CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
INC = $(INCLUDE) -I../ipu

OBJS = ScreenLayer.o

# list of platforms which want this test case
INCLUDE_LIST:= IMX37_3STACK IMX51 IMX53

LIBNAME=libIpuScreenLayer
LIB_SONAME=$(LIBNAME).so
LIB_SONAME_FULL=$(LIB_SONAME).$(LIB_VERSION_MAJOR).$(LIB_VERSION_MINOR)

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))
all: $(LIB_SONAME_FULL)

install:
	@mkdir -p $(DESTDIR)/usr/lib
	cp $(LIBNAME).* $(DESTDIR)/usr/lib
	@mkdir -p $(DESTDIR)/usr/include
	cp ScreenLayer.h $(DESTDIR)/usr/include
else
all install:
	@echo "Not support platform, will not complile"
endif

$(LIB_SONAME_FULL): $(OBJS)
	$(CC) -shared -nostartfiles -Wl,-soname,$@ -o $@ $^ -I/usr/src/linux/include/ -L../ipu -lipu
	ln -s $(LIB_SONAME_FULL) $(LIB_SONAME).$(LIB_VERSION_MAJOR)
	ln -s $(LIB_SONAME_FULL) $(LIB_SONAME)

%.o: %.c
ifeq "$(PLATFORM)" ""
        $(error "Unspecified PLATFORM variable")
endif
	$(CC) -D$(PLATFORM) $(INC) -Wall -O2 -fPIC -c $^ -o $@

.PHONY: clean
clean:
	@rm -f $(OBJS) $(LIBNAME).*
