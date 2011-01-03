CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar

# list of platforms which want this test case
INCLUDE_LIST:= IMX27ADS IMX31ADS IMX32ADS IMX37_3STACK IMX51 IMX53

OBJ = vpu_io.o vpu_util.o vpu_lib.o

LIBNAME=libvpu
LIB_SONAME=$(LIBNAME).so
LIB_SONAME_FULL=$(LIB_SONAME).$(LIB_VERSION_MAJOR).$(LIB_VERSION_MINOR)

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))

# Same platform ID is used when building for imx31ads and imx32ads.
# Change version number in bin filename to imx32 if passed platform is
# imx31ads since there is no VPU module in imx31ads.
ifeq ($(PLATFORM), IMX31ADS)
PLATFORM_STR = IMX32ADS
else
PLATFORM_STR = $(PLATFORM)
endif

ifeq ($(findstring IMX5, $(PLATFORM)), IMX5)
VERSION = imx5
else
VERSION = $(shell echo $(PLATFORM_STR) | awk '{print substr($$0, 1, 5)}' \
	| awk '{print  tolower($$0) }' )
endif

all: $(LIB_SONAME_FULL) $(LIBNAME).a

install: install_headers
	@mkdir -p $(DESTDIR)/usr/lib
	cp $(LIBNAME)* $(DESTDIR)/usr/lib

install_headers:
	@mkdir -p $(DESTDIR)/usr/include
	cp vpu_lib.h $(DESTDIR)/usr/include
	cp vpu_io.h $(DESTDIR)/usr/include
else
all install :
endif

%.o: %.c
	$(CC) -D$(PLATFORM) -Wall -O2 -fPIC -c $^ -o $@

$(LIB_SONAME_FULL): $(OBJ)
	$(CC) -shared -nostartfiles -Wl,-soname,$@ $^ -o $@
	ln -s $(LIB_SONAME_FULL) $(LIB_SONAME).$(LIB_VERSION_MAJOR)
	ln -s $(LIB_SONAME_FULL) $(LIB_SONAME)

$(LIBNAME).a: $(OBJ)
	$(AR) -rc $@  $^

.PHONY: clean
clean:
	rm -f $(LIBNAME)* $(OBJ)
