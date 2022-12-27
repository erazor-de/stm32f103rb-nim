GDB := PYTHONHOME=/home/stefan/opt/python-3.8 arm-none-eabi-gdb
JLINK := JLinkExe
JLINKGDBSERVER := JLinkGDBServer
STFLASH := st-flash
STUTIL := st-util
OPENOCD := openocd-st

OPENOCDFLAGS := -f interface/stlink.cfg -f target/stm32f1x.cfg
JLINKFLAGS := -device STM32F103RB -if SWD -speed 4000

# Short targets can be configured
flash: flash_stlink
debug: debug_openocd

# Uses st-flash via stlink
flash_stflash: firmware.bin
	$(STFLASH) write $< 0x8000000

# Uses openocd via stlink
flash_openocd: firmware.bin
	$(OPENOCD) $(OPENOCDFLAGS) -c "program $< 0x08000000 verify reset exit"

# Uses JLink
define flash_jlink_script
$(JLINK) $(JLINKFLAGS) -autoconnect 1 <<EOF
	LoadFile $< 0x08000000
	Reset
	Exit
EOF
endef
export flash_jlink_script
flash_jlink: firmware.bin
	eval "$$flash_jlink_script"

# Debugserver is JLink
debug_jlink: firmware.elf
	{ \
		set -e ;\
		$(JLINKGDBSERVER) $(JLINKFLAGS) &\
		pid=$$! ;\
		$(GDB) -x debug_jlink.gdb $< ;\
		kill $$pid ;\
	}

# Debugserver is st-util
debug_stutil: firmware.elf
	{ \
		set -e ;\
		$(STUTIL) &\
		pid=$$! ;\
		$(GDB) -x debug_stutil.gdb $< ;\
		kill $$pid ;\
	}

# Debugserver is OpenOCD via stlink
debug_openocd: firmware.elf
	{ \
		set -e ;\
		$(OPENOCD) $(OPENOCDFLAGS) &\
		pid=$$! ;\
		$(GDB) -x debug_openocd.gdb $< ;\
		kill $$pid ;\
	}

# Uses Black Magic Probe via serial port
debug_bmp: firmware.elf
	$(GDB) -x debug_bmp.gdb $<

.PHONY: flash flash_jlink flash_openocd flash_stflash
.PHONY: debug debug_bmp debug_jlink debug_stutil debug_openocd
