# Get all dirs with a Makefile
TMP_DIRS := $(foreach dir, $(wildcard *), $(wildcard $(dir)/Makefile))
DIRS := $(patsubst %/Makefile,%,$(TMP_DIRS))

.PHONY: all install clean
.PHONY: $(DIRS)
all: TARGET=all
all: $(DIRS)

install: TARGET=install
install: $(DIRS)

clean: TARGET=clean
clean : $(DIRS)

$(DIRS):
	$(MAKE) -C $@ $(TARGET)

