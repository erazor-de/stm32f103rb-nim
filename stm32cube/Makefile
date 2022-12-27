AR := arm-none-eabi-ar
CC := arm-none-eabi-gcc
RANLIB := arm-none-eabi-ranlib

ARFLAGS :=
CFLAGS := -g -Wall -O3
CFLAGS += -mcpu=cortex-m3 -march=armv7-m -mtune=cortex-m3 -mthumb
CFLAGS += -fno-common -ffreestanding -ffunction-sections -fdata-sections
CFLAGS += -DSTM32F103xB -DUSE_HAL_DRIVER -DUSE_STM32F1xx_NUCLEO
CFLAGS += -I. -Iinclude

# This is needed for clangd language server:
# __SOFTFP__ is part of gcc built-in defines but not clang
# This way it's present in compile_commands.json
CFLAGS += -D__SOFTFP__=1

# Original c files
OBJS := $(subst .c,.o,$(wildcard src/*.c))
# Original s file
OBJS += src/startup_stm32f103xb.o
# Modified template c files
OBJS += $(subst .c,.o,$(wildcard *.c))

all: lib/libstm32cube.a

lib/libstm32cube.a: $(OBJS)
	$(AR) ru $(ARFLAGS) $@ $^
	$(RANLIB) $(ARFLAGS) $@

clean:
	rm -f $(OBJS)
	rm -f lib/libstm32cube.a

%.o: %.s
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY: all clean
