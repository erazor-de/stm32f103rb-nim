Used versions:
  arm-none-eabi-gcc 12.2
  nim 1.9.1
  svd2nim 0.3.0 (github.com/auxym/svd2nim)
  openocd from STMicro (github.com/STMicroelectronics/OpenOCD)
  stlink (github.com/stlink-org/stlink)

Contains a stripped down version of STM32Cube and STMs SVD.
The Cube headers have been slightly edited to make clangd language server happier.
The SVD has been edited to remove double definitions and a typo. Also all descriptions have been
edited since svd2nim couldn't handle them right.

The result ov svd2nim needed edits as well. 32bit Literals with set msb need cast to uint32 or else
they are treated as int64.

The project uses the STM32Cube linkerscript, startup and sytem code.
The project uses semihosting but read operations do not work.

One can use the peripheral api generated with svd2nim
Or the STM32Cube c-api can be used after generating wrappers.
