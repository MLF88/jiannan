
PROJECT := caffe

CXX ?= /usr/bin/g++

CONFIG_FILE := Makefile.config
include $(CONFIG_FILE)


TOP_DIR = $(shell pwd)
SRC_DIR = $(TOP_DIR)/src  

CXX_FLAGS = -I$(TOP_DIR)/include


DYNAMIC_VERSION_MAJOR 		:= 1
DYNAMIC_VERSION_MINOR 		:= 0
DYNAMIC_VERSION_REVISION 	:= 0-rc3
COMMON_FLAGS += -DCAFFE_VERSION=$(DYNAMIC_VERSION_MAJOR).$(DYNAMIC_VERSION_MINOR).$(DYNAMIC_VERSION_REVISION)

COMMON_FLAGS += -DNDEBUG -O2


COMMON_FLAGS += -DUSE_CUDNN
COMMON_FLAGS += -DUSE_OPENCV
COMMON_FLAGS += -DUSE_LEVELDB
COMMON_FLAGS += -DUSE_LMDB
COMMON_FLAGS += -DWITH_PYTHON_LAYER

ifeq ($(SAVE_FRAME_IN), 1)
	COMMON_FLAGS += -DSAVE_FRAME_IN 
	COMMON_FLAGS += -DSAVE_FRAME
endif

ifeq ($(SAVE_FRAME_OUT), 1)
	COMMON_FLAGS += -DSAVE_FRAME_OUT
	COMMON_FLAGS += -DSAVE_FRAME
endif

CUDA_INCLUDE_DIR := $(CUDA_DIR)/include
INCLUDE_DIRS += $(CUDA_INCLUDE_DIR)


COMMON_FLAGS += $(foreach includedir,$(INCLUDE_DIRS),-isystem $(includedir))

WARNINGS := -Wall -Wno-sign-compare

LINKFLAGS += -pthread -fPIC $(COMMON_FLAGS) $(WARNINGS)

LIBRARY_NAME := $(PROJECT)

LIBRARIES := cudart cublas curand
LIBRARIES += glog gflags protobuf boost_system boost_filesystem boost_regex m hdf5_hl hdf5
LIBRARIES += openblas
LIBRARIES += leveldb snappy
LIBRARIES += lmdb
LIBRARIES += opencv_core opencv_highgui opencv_imgproc

LIBRARIES += boost_thread stdc++
LIBRARIES += cudnn
LIBRARIES += caffe

PYTHON_LIBRARIES ?= boost_python python2.7
LIBRARIES += $(PYTHON_LIBRARIES)

CUDA_LIB_DIR += $(CUDA_DIR)/lib64

LIBRARY_DIRS += $(CUDA_LIB_DIR)

LDFLAGS += $(foreach librarydir,$(LIBRARY_DIRS),-L$(librarydir)) \
		$(foreach library,$(LIBRARIES),-l$(library))

SRC_BASE   =  $(shell find $(SRC_DIR) -name *.cpp)

all: $(SRC_BASE)
	$(Q)$(CXX) $^ -o $@ $(LINKFLAGS) -lmycaffe $(LDFLAGS) $(CXX_FLAGS)\
			-Wl,-rpath
	
clean:
	rm -rf all 










