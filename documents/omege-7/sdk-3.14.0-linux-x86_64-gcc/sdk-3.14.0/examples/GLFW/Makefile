#  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  Force Dimension SDK 3.14.0

# DO NOT MODIFY THIS FILE
# project-specific changes should go in Makefile.project(.*)

# default targets
SUBDIRS = $(patsubst %/,%,$(sort $(dir $(wildcard */Makefile))))
all: $(SUBDIRS)

# project-specific rules
TOP_DIR = ../..
ifneq (,$(wildcard $(TOP_DIR)/Makefile.common))
    include $(TOP_DIR)/Makefile.common
endif
ifneq (,$(wildcard ./Makefile.project))
    include ./Makefile.project
endif

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
