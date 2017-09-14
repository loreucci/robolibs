.PHONY: clean utilities sec uninstall test doc robots neural

CURR_DIR=$(shell pwd)
BUILD_DIR=$(CURR_DIR)/build
INSTALL_PATH=~/usr

all: utilities sec robots neural

utilities:
	mkdir -p $(BUILD_DIR)/build-utilities
	cd $(BUILD_DIR)/build-utilities && cmake ../../utilities -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(BUILD_DIR) && make -j4 && make install

sec: utilities
	mkdir -p $(BUILD_DIR)/build-sec
	cd $(BUILD_DIR)/build-sec && cmake ../../sec -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(BUILD_DIR) -DROBOLIBS_PATH=$(BUILD_DIR) && make -j4 && make install

robots: sec
	mkdir -p $(BUILD_DIR)/build-robots
	cd $(BUILD_DIR)/build-robots && cmake ../../robots -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(BUILD_DIR) -DROBOLIBS_PATH=$(BUILD_DIR) && make -j4 && make install

neural: sec
	mkdir -p $(BUILD_DIR)/build-neural
	cd $(BUILD_DIR)/build-neural && cmake ../../neural -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(BUILD_DIR) -DROBOLIBS_PATH=$(BUILD_DIR) && make -j4 && make install

install:
	cp -r $(BUILD_DIR)/lib $(INSTALL_PATH)
	cp -r $(BUILD_DIR)/include $(INSTALL_PATH)

uninstall:
	rm -rf $(INSTALL_PATH)/include/utilities
	rm -rf $(INSTALL_PATH)/lib/libutilities*
	rm -rf $(INSTALL_PATH)/include/sec
	rm -rf $(INSTALL_PATH)/lib/libsec*
	rm -rf $(INSTALL_PATH)/include/robots
	rm -rf $(INSTALL_PATH)/lib/librobots*
	rm -rf $(INSTALL_PATH)/include/neural
	rm -rf $(INSTALL_PATH)/lib/libneural*

test:
	cd $(BUILD_DIR)/build-utilities && make test

doc:
	-rm -r $(BUILD_DIR)/doc/utilities
	mkdir -p $(BUILD_DIR)/doc/utilities
	cd $(BUILD_DIR)/build-utilities && make doc && cp -r doc/html $(BUILD_DIR)/doc/utilities && cp -r doc/latex $(BUILD_DIR)/doc/utilities
	-rm -r $(BUILD_DIR)/doc/sec
	mkdir -p $(BUILD_DIR)/doc/sec
	cd $(BUILD_DIR)/build-sec && make doc && cp -r doc/html $(BUILD_DIR)/doc/sec && cp -r doc/latex $(BUILD_DIR)/doc/sec

clean:
	rm -fr build
