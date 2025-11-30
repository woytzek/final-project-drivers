# Makefile for Device Tree Overlay compilation
DTC = dtc
OVERLAY_NAME = gpio-reader-overlay
DTS_FILE = $(OVERLAY_NAME).dts
DTBO_FILE = $(OVERLAY_NAME).dtbo

# Default target
all: $(DTBO_FILE)

# Compile the device tree overlay
$(DTBO_FILE): $(DTS_FILE)
	$(DTC) -@ -I dts -O dtb -o $@ $<

# Clean compiled files
clean:
	rm -f $(DTBO_FILE)

# Install to boot overlays directory (requires sudo)
install: $(DTBO_FILE)
	sudo cp $(DTBO_FILE) /boot/overlays/

.PHONY: all clean install
