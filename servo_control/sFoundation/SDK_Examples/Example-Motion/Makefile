# Example Project Makefile
# Example-Motion
#
# Creates an executable in this directory called Example-Motion

EXEC_NAME := "Example-Motion"
INCLUDE_DIR := "../../inc/inc-pub"
LIBS := -lsFoundation20 -lpthread
CC := g++
OPTIMIZATION := -O3
DEBUG_OPTIMIZATION := -O0
CXXFLAGS := -I$(INCLUDE_DIR) $(OPTIMIZATION)
DEBUG_CXXFLAGS := $(CXXFLAGS) -Wall -Wextra -pedantic -g3 $(DEBUG_OPTIMIZATION)

# Specify source files here
ALL_SRC_FILES := $(wildcard *.cpp)
ALL_OBJS := $(patsubst %.cpp,%.o,$(ALL_SRC_FILES))

# Default target
all: Example-Motion

# Generic rule to compile a CPP file into an object
%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) -o "$@" $<

Example-Motion: $(ALL_OBJS)
    ifneq ($(RPATH),)
	$(CC) -L$(RPATH) -Wl,-rpath=$(RPATH) -o $(EXEC_NAME) $(ALL_OBJS) $(LIBS)
    else
	$(CC) -o $(EXEC_NAME) $(ALL_OBJS) $(LIBS)
    endif

# Remove all object files
.PHONY: clean
clean:
	-find . -type f -name "*.o" -delete

# Sayonara. Viciously destroys all build artifacts, including the executable.
.PHONY: real_clean
real_clean: clean
	-rm $(EXEC_NAME)
