
BIN := brain

all: $(BIN)

build/Makefile: Makefile CMakeLists.txt
	mkdir -p build
	(cd build && cmake ..)

$(BIN): $(wildcard *.cc) build/Makefile
	(cd build && make)
	cp build/$(BIN) .

clean:
	rm -rf build $(BIN)
