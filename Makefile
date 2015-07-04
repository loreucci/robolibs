.PHONY: clean utilities sec uninstall

CURR_DIR=$(shell pwd)
BUILD_DIR=$(CURR_DIR)/build
INSTALL_PATH=~/usr

all: utilities sec

utilities:
	mkdir -p $(BUILD_DIR)/utilities
	cd $(BUILD_DIR)/utilities && qmake ../../utilities/utilities.pro "INSTALL_PATH=$(BUILD_DIR)" && make && make install

sec:
	mkdir -p $(BUILD_DIR)/sec
	cd $(BUILD_DIR)/sec && qmake ../../sec/sec.pro "INSTALL_PATH=$(BUILD_DIR)" && make && make install

install:
	cp -r $(BUILD_DIR)/lib $(INSTALL_PATH)
	cp -r $(BUILD_DIR)/include $(INSTALL_PATH)

uninstall:
	rm -rf $(INSTALL_PATH)/include/utilities
	rm -rf $(INSTALL_PATH)/lib/libutilities*
	rm -rf $(INSTALL_PATH)/include/sec
	rm -rf $(INSTALL_PATH)/lib/libsec*

clean:
	rm -fr build
