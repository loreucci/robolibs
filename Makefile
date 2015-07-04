.PHONY: clean utilities

utilities:
	mkdir -p build/utilities
	cd build/utilities && qmake ../../utilities/utilities.pro && make

utilities-install:
	cd build/utilities && make install

install: utilities-install

all: utilities

clean:
	rm -fr build
