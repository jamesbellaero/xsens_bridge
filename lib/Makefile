OBJLIBS	= libxcommunication libxstypes
CPP_FILES := $(wildcard *.cpp)
OBJECTS := $(CPP_FILES:.cpp=.o)
HEADERS = $(wildcard *.h) $(wildcard include/*.h)
INCLUDE=-I. -Iinclude
CFLAGS=-g $(INCLUDE) -include config.h
CXXFLAGS=-std=c++11 $(CFLAGS)
LFLAGS=-Lxcommunication -Lxstypes -lxcommunication -lxstypes -lpthread -lrt -ldl

all : $(OBJLIBS) $(TARGET)

libxcommunication : libxstypes
	$(MAKE) -C xcommunication $(MFLAGS)

libxstypes :
	$(MAKE) -C xstypes $(MFLAGS) libxstypes.a

