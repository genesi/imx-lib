CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
INC = $(INCLUDE) -I../ipu

OBJS = ScreenLayer.o

# list of platforms which want this test case
INCLUDE_LIST:= IMX37_3STACK IMX51 IMX53

LIBNAME=libIpuScreenLayer.so
LIBSONAME=$(LIBNAME).$(LIB_VERSION_MAJOR).$(LIB_VERSION_MINOR)

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))
all: $(LIBSONAME)

install:
	@mkdir -p $(DESTDIR)/usr/lib
	cp $(LIBNAME)* $(DESTDIR)/usr/lib
	@mkdir -p $(DESTDIR)/usr/include
	cp ScreenLayer.h $(DESTDIR)/usr/include
else
all install:
	@echo "Not support platform, will not complile"
endif

$(LIBSONAME): $(OBJS)
	$(CC) -shared -nostartfiles -Wl,-soname,$@ -o $@ $^ -L../ipu -lipu
	ln -s $(LIBSONAME) $(LIBNAME).$(LIB_VERSION_MAJOR)
	ln -s $(LIBSONAME) $(LIBNAME)

%.o: %.c
	$(CC) $(INC) -Wall -O2 -fPIC -c $^ -o $@

.PHONY: clean
clean:
	@rm -f $(OBJS) $(LIBNAME)*
