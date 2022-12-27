type
  HAL_StatusTypeDef* = enum
    HAL_OK       = 0x00,
    HAL_ERROR    = 0x01,
    HAL_BUSY     = 0x02,
    HAL_TIMEOUT  = 0x03

proc HAL_Init*(): HAL_StatusTypeDef {.importc, header: "stm32f1xx_hal.h".}
proc HAL_IncTick*() {.importc, header: "stm32f1xx_hal.h".}
proc HAL_Delay*(delay: uint32) {.importc, header: "stm32f1xx_hal.h".}
