CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar

# list of platforms which want this test case
INCLUDE_LIST:=IMX50 IMX51 IMX5

OBJ = pxp_lib.o

LIBNAME = libpxp

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))

all: $(LIBNAME).so $(LIBNAME).a

install: install_headers
	@mkdir -p $(DESTDIR)/usr/lib
	cp $(LIBNAME).* $(DESTDIR)/usr/lib

install_headers:
	@mkdir -p $(DESTDIR)/usr/include
	cp pxp_lib.h $(DESTDIR)/usr/include
else
all install :
endif

%.o: %.c
	$(CC) -D$(PLATFORM) $(INCLUDE) -Wall -O2 -fPIC -c $^ -o $@

$(LIBNAME).so: $(OBJ)
	$(CC) -shared -nostartfiles -Wl,-soname,$@ $^ -o $@

$(LIBNAME).a: $(OBJ)
	$(AR) -rc $@  $^

.PHONY: clean
clean:
	rm -f $(LIBNAME).* $(OBJ)
