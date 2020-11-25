
all:
	(cp ./.idea /other/ 2>/dev/null || :)
	(cd brain && make)
	(cd plugins && make)

clean:
	(cp ./.idea /other/ 2>/dev/null || :)
	(cd brain && make clean)
	(cd plugins && make clean)

.PHONY: clean
