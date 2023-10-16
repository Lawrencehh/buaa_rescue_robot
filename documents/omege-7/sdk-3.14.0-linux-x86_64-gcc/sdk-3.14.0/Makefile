#  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  Force Dimension SDK 3.14.0

# project structure
TOP_DIR = .
BIN_DIR = $(TOP_DIR)/bin
include Makefile.common

# build dependencies
SUBDIRS = examples

# build rules

all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done

