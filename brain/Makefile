
BIN := kickerbot goaliebot

THREADS := 4

all: $(BIN)

build/Makefile: Makefile CMakeLists.txt
	mkdir -p build
	(cd build && cmake ..)

cmake: build/Makefile

$(BIN): $(wildcard *.cc) build/Makefile
	(cd build && make -j $(THREADS))
	(cd build && cp $(BIN) ..)

clean:
	rm -rf build $(BIN)
