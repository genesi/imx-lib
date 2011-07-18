AR=$(CROSS_COMPILE)ar
RM=rm -f
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld

INC := $(INCLUDE)

DIAGS=


LIBNAME=libsahara
LIB_SONAME=$(LIBNAME).so
LIB_SONAME_FULL=$(LIB_SONAME).$(LIB_VERSION_MAJOR).$(LIB_VERSION_MINOR)


ifeq ("$(PLATFORM)", "IMX27ADS")
TARGET_ARCH=-DCONFIG_ARCH_MX27
endif
ifeq ($(findstring IMX5, $(PLATFORM)), IMX5)
TARGET_ARCH=-DCONFIG_ARCH_MX5
endif

# Add compilation checks
CFLAGS += -g -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wdeclaration-after-statement -I/usr/src/linux/include/ -Ikernel/ -fPIC

OBJS= fsl_shw_rand.o  fsl_shw_hash.o  fsl_shw_sym.o  fsl_shw_user.o         \
      fsl_shw_auth.o  fsl_shw_hmac.o  fsl_shw_wrap.o  fsl_shw_keystore.o um_adaptor.o  sf_util.o


#CFLAGS += -DSAHARA -DLINUX -I$(APIINCDIR)
CFLAGS += -DSAHARA -DLINUX $(INC)
CFLAGS += $(TARGET_ARCH)
# Uncomment to debug Library's creation of structures for driver
#CFLAGS +=  -DDIAG_SECURITY_FUNC
# Uncomment to simulate memory allocation errors
#CFLAGS += -DDIAG_MEM_ERRORS -DDIAG_MEM_CONST=5

ifeq ("$(TARGET_ARCH)", "")
all install:
else
all: $(LIB_SONAME_FULL) $(LIBNAME).a

install: install_headers
	@mkdir -p $(DESTDIR)/usr/lib
	cp $(LIBNAME).* $(DESTDIR)/usr/lib

install_headers:
	@mkdir -p $(DESTDIR)/usr/include
endif


$(LIBNAME).a: $(OBJS)
	$(AR) -rc $@ $^

$(LIB_SONAME_FULL): $(OBJS)
	$(CC) -shared -nostartfiles -Wl,-soname,$@ -o $@ $^
	ln -s $(LIB_SONAME_FULL) $(LIB_SONAME).$(LIB_VERSION_MAJOR)
	ln -s $(LIB_SONAME_FULL) $(LIB_SONAME)

.PHONY: clean
clean:
	@rm -f $(OBJS) $(LIBNAME).*

fsl_shw_rand.o: fsl_shw_rand.c $(API_INCL)
fsl_shw_hash.o: fsl_shw_hash.c $(API_INCL)
fsl_shw_sym.o: fsl_shw_sym.c $(API_INCL)
fsl_shw_user.o: fsl_shw_user.c $(API_INCL)
fsl_shw_hmac.o: fsl_shw_hmac.c $(API_INCL)
fsl_shw_auth.o: fsl_shw_auth.c $(API_INCL)
fsl_shw_wrap.o: fsl_shw_wrap.c
fsl_shw_keystore.o: fsl_shw_keystore.c
um_adaptor.o: um_adaptor.o
sf_util.o: sf_util.c
