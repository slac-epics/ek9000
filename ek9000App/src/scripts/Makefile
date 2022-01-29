all: download ../terminals.h

download:
	python3 download.py

clean:
	rm -f ../terminals.h
	rm -rf ./BhcConfigFiles
	rm -rf BhcConfigFiles.zip

../terminals.h: terminals.json
	python3 generate.py -o "$(PWD)/../terminals.h" terminals.json 

.PHONY: download clean
