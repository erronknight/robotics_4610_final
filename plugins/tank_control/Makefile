
LIB := libtank_control_plugin.so

all: $(LIB)

build/Makefile: Makefile CMakeLists.txt
	mkdir -p build
	(cd build && cmake ..)

$(LIB): $(wildcard *.cc) build/Makefile
	(cd build && make)
	cp build/$(LIB) .

clean:
	rm -rf build $(LIB)
