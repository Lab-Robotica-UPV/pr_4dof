include = -I../../include
libpath = -L../../lib
libs = -lNatNetLibShared

all: build/BroadcastTrigger

build/BroadcastTrigger: 
	g++ BroadcastTrigger.cpp $(include) $(libpath) $(libs) -o build/BroadcastTrigger

.PHONY: clean
clean:
	@rm -f ./build/BroadcastTrigger