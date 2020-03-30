CC = ~/rpi-toolchain/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-cc
export CC
INSTALL = /media/exports/rpi4/
export INSTALL
CFLAGS := -Wall -pedantic
export CFLAGS

SUBDIRS = PCA9685 SF0180 TB6612FNG

MAKE += --silent

INCLUDE = $(realpath ../include)
export INCLUDE


all: build

build: subdirs

subdirs: $(SUBDIRS)

tests: $(SUBDIRS)
	@for dir in $(SUBDIRS); do \
		echo "CC tests driver/$$dir"; \
		$(MAKE) -C $$dir test; \
	done

clean:
	@for dir in $(SUBDIRS); do \
        echo "Cleaning driver/$$dir"; \
	    $(MAKE) -C $$dir clean ; \
	done

$(SUBDIRS) :
	@echo "CC driver/$@"
	@$(MAKE) -C $@

.PHONY: subdirs $(SUBDIRS)
