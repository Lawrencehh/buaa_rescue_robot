#  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  Force Dimension SDK 3.14.0

# DO NOT MODIFY THIS FILE
# project-specific changes should go in Makefile.project(.*)

# default target
all: project

# common settings
TOP_DIR = ../../..
ifneq (,$(wildcard $(TOP_DIR)/Makefile.common))
    include $(TOP_DIR)/Makefile.common
endif

# directories
SRC_DIR = .
OBJ_DIR = ./obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# files
SOURCES  = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDES = $(wildcard $(SRC_DIR)/*.h)
OBJECTS  = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
NAME     = $(notdir $(shell pwd)) 
TARGET   = $(BIN_DIR)/$(NAME)

# import project settings
ifneq (,$(wildcard ./Makefile.project))
    include ./Makefile.project
endif

# import platform-specific project settings
ifneq (,$(wildcard ./Makefile.project.$(OS)))
    include ./Makefile.project.$(OS)
endif

# build rules

project: $(TARGET)

$(OBJECTS): $(INCLUDES) | $(OBJ_DIR)

$(TARGET): $(LIB_STATIC) $(OBJECTS) $(LIB_INCLUDES) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(TARGET)

$(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: project

clean:
	rm -f $(TARGET) $(OBJECTS) *~ TAGS core *.bak #*#
	-rmdir $(OBJ_DIR)
