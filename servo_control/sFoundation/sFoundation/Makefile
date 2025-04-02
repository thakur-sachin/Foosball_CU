# Teknic sFoundation Makefile
#
# To build libsFoundation20.so, simply change to this directory and type make.
# 
# By default, this makefile configures g++ to suppress all warning output. If
# you plan on modifying sFoundation, you will probably want to disable that.
# Simply modify the CXXFLAGS variable below with the warnings you want to see,
# and be sure to remove the -w option.

SO_NAME := libsFoundation20.so
SO_MAJOR_VERSION := 1

# The names of the directories all of the built artifacts will be stored in
BUILD_DIR_BASE := build
RELEASE_SUBDIR := release
RELEASE_BUILD_DIR := $(BUILD_DIR_BASE)/$(RELEASE_SUBDIR)

# Active build directory -- default to release
BUILD_DIR := $(RELEASE_BUILD_DIR)

# Build flags
CXX := g++
OPTIMIZATION := -O3
CXXFLAGS := -std=c++11                   \
            -fPIC                        \
            -fsigned-char                \
            -Wno-stringop-overflow       \
            -Wno-format-truncation       \
            -Wno-deprecated-declarations \
            -Wno-format-overflow         \
            $(OPTIMIZATION)
LIBS := -lrt -ldl
INCLUDES := ../inc/inc-pub \
            ../inc/inc-private \
            ../inc/inc-private/linux \
            ../inc/inc-private/sFound \
            ../LibINI/inc \
            ../LibLinuxOS/inc \
            ../LibXML/inc \

CLANG_TIDY_CHECKS := -*,portability-*,bugprone-*,performance-*,-bugprone-narrowing-conversions,-bugprone-incorrect-roundings,-bugprone-signed-char-misuse,-bugprone-branch-clone

# Gather required source files
SRC_DIR := src
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
SRC_LINUX_DIR := src-linux
SRC_LINUX_FILES := $(wildcard $(SRC_LINUX_DIR)/*.cpp)
LIBINI_SRC_DIR := ../LibINI/src
LIBINI_SRC_FILES := $(wildcard $(LIBINI_SRC_DIR)/*.cpp)
LIBXML_SRC_DIR := ../LibXML/src
LIBXML_SRC_FILES := $(wildcard $(LIBXML_SRC_DIR)/*.cpp)
LIBLINUXOS_SRC_DIR := ../LibLinuxOS/src
LIBLINUXOS_SRC_FILES := $(wildcard $(LIBLINUXOS_SRC_DIR)/*.cpp)

ALL_SRC_FILES := $(SRC_FILES) $(SRC_LINUX_FILES) $(LIBINI_SRC_FILES) $(LIBXML_SRC_FILES) $(LIBLINUXOS_SRC_FILES)
ALL_OBJ_FILES := $(patsubst %.cpp,%.o,$(ALL_SRC_FILES))
REL_OBJ_FILES := $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJ_FILES))
INCLUDE_FLAGS := $(patsubst %, -I"%", $(INCLUDES))

DEPDIR := $(BUILD_DIR)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d

# Default Build Target (release)
all: libsFoundation20

# Compile individual source files
.SECONDEXPANSION:
$(BUILD_DIR)/%.o: %.cpp | $$(@D)/. $(DEPDIR)
	$(CXX) -c $(CXXFLAGS) $(DEPFLAGS) $(INCLUDE_FLAGS) -o "$@" $<

# Common Build Target -- build object files and link them together
libsFoundation20: dir $(REL_OBJ_FILES)
	$(CXX) $(CXXFLAGS) -shared -Wl,-soname=$(SO_NAME).$(SO_MAJOR_VERSION) -o "$(SO_NAME)" $(REL_OBJ_FILES) $(LIBS)

# Create the root directory for all of the build artifacts
.PHONY: dir
dir: | $(BUILD_DIR)/.

.PRECIOUS: $(BUILD_DIR)/. $(BUILD_DIR)%/.

# Create the build dir
$(BUILD_DIR)/.:
	mkdir -p $@

# Create a dir within the build dir
$(BUILD_DIR)%/.:
	mkdir -p $@

# Create dependencies directory
$(DEPDIR):
	mkdir -p $@

# Remove intermediate build files from the BUILD_DIR directory. This will delete
# object files only.
.PHONY: clean
clean:
	-rm -rf $(BUILD_DIR_BASE)

# Sayonara. Viciously destroys any and all build artifacts.
.PHONY: real_clean
real_clean: clean
	-rm $(SO_NAME)

.PHONY: lint
lint: $(SRC_FILES)
	clang-tidy -checks=$(CLANG_TIDY_CHECKS) -header-filter=.* $(SRC_FILES) -- $(CXXFLAGS) $(INCLUDE_FLAGS)

DEPFILES := $(SRC_FILES:%.cpp=$(DEPDIR)/%.d)
$(DEPFILES):

include $(wildcard $(DEPFILES))
