CC = ~/rpi-toolchain/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-cc
CFLAGS := -Wall -pedantic

SUBDIRS = MP1593 PCA9685 PCF8591 SF0180 TB6612FNG

MAKE += --silent

all: build

build: subdirs

subdirs: $(SUBDIRS)

clean:
	@for dir in $(SUBDIRS); do \
        echo "Cleaning driver/$$dir"; \
	    $(MAKE) -C $$dir clean ; \
	done

$(SUBDIRS) :
	@echo "CC driver/$@"
	@$(MAKE) -C $@

.PHONY: subdirs $(SUBDIRS)
