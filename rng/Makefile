AR = ar -crv
RM = rm -f
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld

INC := $(INCLUDE)

# Add compilation checks
CFLAGS += -g -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wdeclaration-after-statement -fPIC

# In absence of kernel CONFIG flags, set API library to build on
# 'unknown' platform.
CFLAGS += -DFSL_PLATFORM_OTHER -DFSL_HAVE_RNGC $(INC)

OBJS= fsl_shw_rand.o  fsl_shw_hash.o  fsl_shw_sym.o  fsl_shw_user.o         \
		fsl_shw_keystore.o fsl_shw_auth.o  fsl_shw_hmac.o  fsl_shw_wrap.o

LIBNAME=libfsl_shw

all: $(LIBNAME).so $(LIBNAME).a

install: install_headers
	@mkdir -p $(DESTDIR)/usr/lib
	cp $(LIBNAME).* $(DESTDIR)/usr/lib

install_headers:
	@mkdir -p $(DESTDIR)/usr/include

$(LIBNAME).a: $(OBJS)
	$(AR) $@ $^

$(LIBNAME).so: $(OBJS)
	$(CC) -shared -nostartfiles -Wl,-soname,$@ -o $@ $^

.PHONY: clean
clean:
	@rm -f *.o $(LIBNAME).*

# The keystore file is located in the Sahara directory
fsl_shw_keystore.o: ../sahara2/fsl_shw_keystore.c
	$(CC) $(CFLAGS) -c ../sahara2/fsl_shw_keystore.c

