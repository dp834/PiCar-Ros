CC = /opt/pi-tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-cc
export CC
INSTALL = /media/exports/rpi4/
export INSTALL
CFLAGS := -Wall -pedantic
export CFLAGS

AR = /opt/pi-tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-ar

SUBDIRS = PCA9685 SF0180 TB6612FNG
OBJECTS=$(foreach %, $(SUBDIRS), $%/$%.o)

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

lib: all
	@echo "Creating library picar"
	@$(AR) rcs libpicar.a $(OBJECTS)
	@cp -p libpicar.a $(INSTALL)

$(SUBDIRS) :
	@echo "CC driver/$@"
	@$(MAKE) -C $@

.PHONY: subdirs $(SUBDIRS)
