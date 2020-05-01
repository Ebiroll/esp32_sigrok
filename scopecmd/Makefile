

CC = gcc
CXX = g++


LIBS = -lstdc++ 


CFLAGS = -g -fsanitize=address -std=c++11
#CFLAGS += `pkg-config --cflags libusb-1.0`

#LIBS += `pkg-config --libs libusb-1.0`

OBJS = obj/scopecmd.cpp.o
OBJS += obj/cfgmap.cpp.o
OBJS += obj/netcomm.cpp.o
#OBJS += obj/freetmc_local.cpp.o
OBJS += obj/freetmc_remote.cpp.o


all: scopecmd

obj:
	mkdir -p obj

scopecmd: obj src/simple_except.h src/freetmc.h src/netcomm.h src/freetmc_remote.h src/rigoltmc.h $(OBJS)
	$(CXX) $(CFLAGS) $(OBJS) $(LIBS) -o scopecmd


obj/%.cpp.o: src/%.cpp
	$(CXX) -c $(CFLAGS) $< -o $@


clean:
	rm -rf obj/*.o

