import STM32F103

import gpio
import hal
import semihosting

proc main() {.exportc, noreturn.} =
  discard HAL_Init()
  initialise_monitor_handles()

  RCC.APB2ENR.modifyIt:
    it.IOPAEN = true
    it.IOPCEN = true

  GPIOA.CRL.modifyIt:
    it.MODE5 = Output_50
    it.CNF5 = PushPull

  GPIOC.CRH.modifyIt:
    it.MODE13 = Input
    it.CNF13 = Pull

  GPIOC.ODR.modifyIt:
    it.ODR13 = PullUp

  var number: int

  discard printf("Hello, World!\l")

  number = getchar()
  discard putchar(number)

  discard scanf("%d", addr number)
  discard printf("%d\l", number)

  while true:
    if GPIOC.IDR.read().IDR13:
      GPIOA.BRR.write(GPIOA_BRR_Fields(
        BR5: true,
      ))
    else:
      GPIOA.BSRR.write(GPIOA_BSRR_Fields(
        BS5: true,
      ))

proc NMI_Handler() {.exportc.} =
  discard

proc HardFault_Handler() {.exportc, noreturn.} =
  while true:
    discard

proc MemManage_Handler() {.exportc, noreturn.} =
  while true:
    discard

proc BusFault_Handler() {.exportc, noreturn.} =
  while true:
    discard

proc UsageFault_Handler() {.exportc, noreturn.} =
  while true:
    discard

proc SVC_Handler() {.exportc.} =
  discard

proc DebugMon_Handler() {.exportc.} =
  discard

proc PendSV_Handler() {.exportc.} =
  discard

proc SysTick_Handler() {.exportc.} =
  HAL_IncTick()
