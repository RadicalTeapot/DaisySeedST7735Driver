# Project Name
TARGET = test

# Sources
CPP_SOURCES = test.cpp

# Library Locations
LIBDAISY_DIR ?= ../DaisyExamples/libDaisy
DAISYSP_DIR ?= ../DaisyExamples/DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
