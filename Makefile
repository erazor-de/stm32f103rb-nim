NIM := nim
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
SVD2NIM := svd2nim

# Additional compiler flags
# Nim compiles for release but debug symbols are added
CFLAGS := -g -pedantic
CFLAGS += -Wall -Wextra -Wno-unused-but-set-variable
CFLAGS += -mcpu=cortex-m3 -march=armv7-m -mtune=cortex-m3 -mthumb
CFLAGS += -finline-limit=200 -fno-strict-aliasing -fno-ident -fno-common -ffreestanding -ffunction-sections -fdata-sections
CFLAGS += -DSTM32F103xB -DUSE_HAL_DRIVER -DUSE_STM32F1xx_NUCLEO
CFLAGS += -Istm32cube/include -Istm32cube

# Flags meant for gcc in linking stage
LDFLAGS := --specs=rdimon.specs
LDFLAGS += -Wl,-Tstm32cube/STM32F103XB_FLASH.ld
LDFLAGS += -Wl,--gc-sections,--no-warn-rwx-segment
LDFLAGS += -Wl,-Lstm32cube/lib
LDLIBS := -Wl,-lstm32cube,-lrdimon

# Used compiler is configured in nim.cfg
NIMFLAGS := --os:standalone --cpu:arm --cc:gcc
NIMFLAGS += --define:release --define:nimPreviewSlimSystem
NIMFLAGS += --noMain:on --mm:none --nimcache:nimcache --implicitStatic:on --threads:off --opt:size
NIMFLAGS += --passC:"${CFLAGS}" --passL:"${CFLAGS} ${LDFLAGS} ${LDLIBS}"
NIMFLAGS += --app:console --genScript:off

all: elf

# Generate firmware
elf: firmware.elf
firmware.elf: $(wildcard src/*.nim) stm32cube/lib/libstm32cube.a src/STM32F103.nim
	$(NIM) compileToC $(NIMFLAGS) --out: $@ src/firmware.nim
	$(SIZE) $@

# Compiles new firmware
bin: firmware.bin
firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@

# Generates peripheral api from svd into src
src/STM32F103.nim: stm32cube/STM32F103.svd
	$(SVD2NIM) $< -o $@

# Generates static library with st cube code
stm32cube/lib/libstm32cube.a:
	make -C stm32cube

clean:
	rm -rf nimcache
	rm -f firmware.bin
	rm -f firmware.elf
	# STM32F103 is manually modified, don't delete
	#rm -f src/STM32F103.nim

.PHONY: all bin clean elf

include Debug.mk
