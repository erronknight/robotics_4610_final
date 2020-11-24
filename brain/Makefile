
BIN := kickerbot goaliebot

all: $(BIN)

build/Makefile: Makefile CMakeLists.txt
	mkdir -p build
	(cd build && cmake ..)

$(BIN): $(wildcard *.cc) build/Makefile
	(cd build && make)
	(cd build && cp $(BIN) ..)

clean:
	rm -rf build $(BIN)
