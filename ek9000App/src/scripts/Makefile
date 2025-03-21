all: ../terminals.h

clean:
	rm -f ../terminals.h

../terminals.h: terminals.json
	python3 generate.py -o "$(PWD)/../terminals.h" terminals.json

.PHONY: clean
