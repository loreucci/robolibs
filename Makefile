.PHONY: clean utilities sec uninstall test doc

CURR_DIR=$(shell pwd)
BUILD_DIR=$(CURR_DIR)/build
INSTALL_PATH=~/usr

all: utilities sec

utilities:
	mkdir -p $(BUILD_DIR)/utilities
	cd $(BUILD_DIR)/utilities && qmake ../../utilities/utilities.pro "CONFIG-=debug" "INSTALL_PATH=$(BUILD_DIR)" && make && make install

sec:
	mkdir -p $(BUILD_DIR)/sec
	cd $(BUILD_DIR)/sec && qmake ../../sec/sec.pro "CONFIG-=debug" "INSTALL_PATH=$(BUILD_DIR)" && make && make install

install:
	cp -r $(BUILD_DIR)/lib $(INSTALL_PATH)
	cp -r $(BUILD_DIR)/include $(INSTALL_PATH)

uninstall:
	rm -rf $(INSTALL_PATH)/include/utilities
	rm -rf $(INSTALL_PATH)/lib/libutilities*
	rm -rf $(INSTALL_PATH)/include/sec
	rm -rf $(INSTALL_PATH)/lib/libsec*

test:
	mkdir -p $(BUILD_DIR)/test/utilities
	cd $(BUILD_DIR)/test/utilities && cmake -DROBOLIBS_PATH=$(BUILD_DIR) ../../../utilities/test && make && make test

doc:
	mkdir -p $(BUILD_DIR)/doc/utilities
	cd utilities/doc && doxygen Doxyfile && mv html $(BUILD_DIR)/doc/utilities && mv latex $(BUILD_DIR)/doc/utilities
	mkdir -p $(BUILD_DIR)/doc/sec
	cd sec/doc && doxygen Doxyfile && mv html $(BUILD_DIR)/doc/sec && mv latex $(BUILD_DIR)/doc/sec

clean:
	rm -fr build
