
all:
	(cd brain && make)
	(cd plugins && make)

clean:
	(cd brain && make clean)
	(cd plugins && make clean)

.PHONY: clean
