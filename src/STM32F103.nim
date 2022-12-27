# Peripheral access API for STM32F103 microcontrollers (generated using svd2nim)

import std/volatile

{.hint[name]: off.}

# Some information about this device.
const DEVICE* = "STM32F103"
const CM3_REV* = 0x0001
const MPU_PRESENT* = 0
const FPU_PRESENT* = 0
const VTOR_PRESENT* = 1
const NVIC_PRIO_BITS* = 4
const Vendor_SysTickConfig* = 0

################################################################################
# Interrupt Number Definition
################################################################################
type IRQn* = enum
# #### Cortex-M Processor Exception Numbers ####################################
  NonMaskableInt_IRQn = -14,            # Exception 2: Non Maskable Interrupt
  HardFault_IRQn = -13,                 # Exception 3: Hard fault Interrupt
  MemoryManagement_IRQn = -12,          # Exception 4: Memory Management Interrupt [Not on Cortex M0 variants]
  BusFault_IRQn = -11,                  # Exception 5: Bus Fault Interrupt [Not on Cortex M0 variants]
  UsageFault_IRQn = -10,                # Exception 6: Usage Fault Interrupt [Not on Cortex M0 variants]
  SVCall_IRQn = -5,                     # Exception 11: SV Call Interrupt
  DebugMonitor_IRQn = -4,               # Exception 12: Debug Monitor Interrupt [Not on Cortex M0 variants]
  PendSV_IRQn = -2,                     # Exception 14: Pend SV Interrupt [Not on Cortex M0 variants]
  SysTick_IRQn = -1,                    # Exception 15: System Tick Interrupt
# #### Device specific Interrupt numbers #######################################
  WWDG_IRQn = 0,                        # Window WatchDog Interrupt
  PVD_IRQn = 1,                         # PVD through EXTI Line detection Interrupt
  TAMPER_IRQn = 2,                                          # Tamper interrupt
  RTC_IRQn = 3,                                             # RTC global interrupt
  FLASH_IRQn = 4,                                           # Flash global interrupt
  RCC_IRQn = 5,                                             # RCC global interrupt
  EXTI0_IRQn = 6,                                           # EXTI Line0 interrupt
  EXTI1_IRQn = 7,                                           # EXTI Line1 interrupt
  EXTI2_IRQn = 8,                                           # EXTI Line2 interrupt
  EXTI3_IRQn = 9,                                           # EXTI Line3 interrupt
  EXTI4_IRQn = 10,                                          # EXTI Line4 interrupt
  DMA1_CHANNEL1_IRQn = 11,                                  # DMA1 Channel1 global interrupt
  DMA1_CHANNEL2_IRQn = 12,                                  # DMA1 Channel2 global interrupt
  DMA1_CHANNEL3_IRQn = 13,                                  # DMA1 Channel3 global interrupt
  DMA1_CHANNEL4_IRQn = 14,                                  # DMA1 Channel4 global interrupt
  DMA1_CHANNEL5_IRQn = 15,                                  # DMA1 Channel5 global interrupt
  DMA1_CHANNEL6_IRQn = 16,                                  # DMA1 Channel6 global interrupt
  DMA1_CHANNEL7_IRQn = 17,                                  # DMA1 Channel7 global interrupt
  ADC1_2_IRQn = 18,                                         # ADC1 and ADC2 global interrupt
  USB_HP_CAN_TX_IRQn = 19,                                  # USB High Priority or CAN TX interrupts
  USB_LP_CAN_RX0_IRQn = 20,                                 # USB Low Priority or CAN RX0 interrupts
  CAN_RX1_IRQn = 21,                                        # CAN RX1 interrupt
  CAN_SCE_IRQn = 22,                                        # CAN SCE interrupt
  EXTI9_5_IRQn = 23,                                        # EXTI Line[9:5] interrupts
  TIM1_BRK_IRQn = 24,                                       # TIM1 Break interrupt
  TIM1_UP_IRQn = 25,                                        # TIM1 Update interrupt
  TIM1_TRG_COM_IRQn = 26,                                   # TIM1 Trigger and Commutation interrupts
  TIM1_CC_IRQn = 27,                                        # TIM1 Capture Compare interrupt
  TIM2_IRQn = 28,                                           # TIM2 global interrupt
  TIM3_IRQn = 29,                                           # TIM3 global interrupt
  TIM4_IRQn = 30,                                           # TIM4 global interrupt
  I2C1_EV_IRQn = 31,                                        # I2C1 event interrupt
  I2C1_ER_IRQn = 32,                                        # I2C1 error interrupt
  I2C2_EV_IRQn = 33,                                        # I2C2 event interrupt
  I2C2_ER_IRQn = 34,                                        # I2C2 error interrupt
  SPI1_IRQn = 35,                                           # SPI1 global interrupt
  SPI2_IRQn = 36,                                           # SPI2 global interrupt
  USART1_IRQn = 37,                                         # USART1 global interrupt
  USART2_IRQn = 38,                                         # USART2 global interrupt
  USART3_IRQn = 39,                                         # USART3 global interrupt
  EXTI15_10_IRQn = 40,                                      # EXTI Line[15:10] interrupts
  RTCALARM_IRQn = 41,                                       # RTC Alarms through EXTI line interrupt
  TIM8_BRK_IRQn = 43,                                       # TIM8 Break interrupt
  TIM8_UP_IRQn = 44,                                        # TIM8 Update interrupt
  TIM8_TRG_COM_IRQn = 45,                                   # TIM8 Trigger and Commutation interrupts
  TIM8_CC_IRQn = 46,                                        # TIM8 Capture Compare interrupt
  ADC3_IRQn = 47,                                           # ADC3 global interrupt
  FSMC_IRQn = 48,                                           # FSMC global interrupt
  SDIO_IRQn = 49,                                           # SDIO global interrupt
  TIM5_IRQn = 50,                                           # TIM5 global interrupt
  SPI3_IRQn = 51,                                           # SPI3 global interrupt
  UART4_IRQn = 52,                                          # UART4 global interrupt
  UART5_IRQn = 53,                                          # UART5 global interrupt
  TIM6_IRQn = 54,                                           # TIM6 global interrupt
  TIM7_IRQn = 55,                                           # TIM7 global interrupt
  DMA2_CHANNEL1_IRQn = 56,                                  # DMA2 Channel1 global interrupt
  DMA2_CHANNEL2_IRQn = 57,                                  # DMA2 Channel2 global interrupt
  DMA2_CHANNEL3_IRQn = 58,                                  # DMA2 Channel3 global interrupt
  DMA2_CHANNEL4_5_IRQn = 59,                                # DMA2 Channel4 and DMA2 Channel5 global interrupt

################################################################################
# Type definitions for peripheral registers
################################################################################
type FSMC_BWTR4_Type = object
  loc: uint

type FSMC_BWTR3_Type = object
  loc: uint

type FSMC_BWTR2_Type = object
  loc: uint

type FSMC_BWTR1_Type = object
  loc: uint

type FSMC_PIO4_Type = object
  loc: uint

type FSMC_PATT4_Type = object
  loc: uint

type FSMC_PMEM4_Type = object
  loc: uint

type FSMC_SR4_Type = object
  loc: uint

type FSMC_PCR4_Type = object
  loc: uint

type FSMC_ECCR3_Type = object
  loc: uint

type FSMC_PATT3_Type = object
  loc: uint

type FSMC_PMEM3_Type = object
  loc: uint

type FSMC_SR3_Type = object
  loc: uint

type FSMC_PCR3_Type = object
  loc: uint

type FSMC_ECCR2_Type = object
  loc: uint

type FSMC_PATT2_Type = object
  loc: uint

type FSMC_PMEM2_Type = object
  loc: uint

type FSMC_SR2_Type = object
  loc: uint

type FSMC_PCR2_Type = object
  loc: uint

type FSMC_BTR4_Type = object
  loc: uint

type FSMC_BCR4_Type = object
  loc: uint

type FSMC_BTR3_Type = object
  loc: uint

type FSMC_BCR3_Type = object
  loc: uint

type FSMC_BTR2_Type = object
  loc: uint

type FSMC_BCR2_Type = object
  loc: uint

type FSMC_BTR1_Type = object
  loc: uint

type FSMC_BCR1_Type = object
  loc: uint

type FSMC_Type = object
  BCR1*: FSMC_BCR1_Type
  BTR1*: FSMC_BTR1_Type
  BCR2*: FSMC_BCR2_Type
  BTR2*: FSMC_BTR2_Type
  BCR3*: FSMC_BCR3_Type
  BTR3*: FSMC_BTR3_Type
  BCR4*: FSMC_BCR4_Type
  BTR4*: FSMC_BTR4_Type
  PCR2*: FSMC_PCR2_Type
  SR2*: FSMC_SR2_Type
  PMEM2*: FSMC_PMEM2_Type
  PATT2*: FSMC_PATT2_Type
  ECCR2*: FSMC_ECCR2_Type
  PCR3*: FSMC_PCR3_Type
  SR3*: FSMC_SR3_Type
  PMEM3*: FSMC_PMEM3_Type
  PATT3*: FSMC_PATT3_Type
  ECCR3*: FSMC_ECCR3_Type
  PCR4*: FSMC_PCR4_Type
  SR4*: FSMC_SR4_Type
  PMEM4*: FSMC_PMEM4_Type
  PATT4*: FSMC_PATT4_Type
  PIO4*: FSMC_PIO4_Type
  BWTR1*: FSMC_BWTR1_Type
  BWTR2*: FSMC_BWTR2_Type
  BWTR3*: FSMC_BWTR3_Type
  BWTR4*: FSMC_BWTR4_Type

type PWR_CSR_Type = object
  loc: uint

type PWR_CR_Type = object
  loc: uint

type PWR_Type = object
  CR*: PWR_CR_Type
  CSR*: PWR_CSR_Type

type RCC_CSR_Type = object
  loc: uint

type RCC_BDCR_Type = object
  loc: uint

type RCC_APB1ENR_Type = object
  loc: uint

type RCC_APB2ENR_Type = object
  loc: uint

type RCC_AHBENR_Type = object
  loc: uint

type RCC_APB1RSTR_Type = object
  loc: uint

type RCC_APB2RSTR_Type = object
  loc: uint

type RCC_CIR_Type = object
  loc: uint

type RCC_CFGR_Type = object
  loc: uint

type RCC_CR_Type = object
  loc: uint

type RCC_Type = object
  CR*: RCC_CR_Type
  CFGR*: RCC_CFGR_Type
  CIR*: RCC_CIR_Type
  APB2RSTR*: RCC_APB2RSTR_Type
  APB1RSTR*: RCC_APB1RSTR_Type
  AHBENR*: RCC_AHBENR_Type
  APB2ENR*: RCC_APB2ENR_Type
  APB1ENR*: RCC_APB1ENR_Type
  BDCR*: RCC_BDCR_Type
  CSR*: RCC_CSR_Type

type GPIOA_LCKR_Type = object
  loc: uint

type GPIOA_BRR_Type = object
  loc: uint

type GPIOA_BSRR_Type = object
  loc: uint

type GPIOA_ODR_Type = object
  loc: uint

type GPIOA_IDR_Type = object
  loc: uint

type GPIOA_CRH_Type = object
  loc: uint

type GPIOA_CRL_Type = object
  loc: uint

type GPIOA_Type = object
  CRL*: GPIOA_CRL_Type
  CRH*: GPIOA_CRH_Type
  IDR*: GPIOA_IDR_Type
  ODR*: GPIOA_ODR_Type
  BSRR*: GPIOA_BSRR_Type
  BRR*: GPIOA_BRR_Type
  LCKR*: GPIOA_LCKR_Type

type AFIO_MAPR2_Type = object
  loc: uint

type AFIO_EXTICR4_Type = object
  loc: uint

type AFIO_EXTICR3_Type = object
  loc: uint

type AFIO_EXTICR2_Type = object
  loc: uint

type AFIO_EXTICR1_Type = object
  loc: uint

type AFIO_MAPR_Type = object
  loc: uint

type AFIO_EVCR_Type = object
  loc: uint

type AFIO_Type = object
  EVCR*: AFIO_EVCR_Type
  MAPR*: AFIO_MAPR_Type
  EXTICR1*: AFIO_EXTICR1_Type
  EXTICR2*: AFIO_EXTICR2_Type
  EXTICR3*: AFIO_EXTICR3_Type
  EXTICR4*: AFIO_EXTICR4_Type
  MAPR2*: AFIO_MAPR2_Type

type EXTI_PR_Type = object
  loc: uint

type EXTI_SWIER_Type = object
  loc: uint

type EXTI_FTSR_Type = object
  loc: uint

type EXTI_RTSR_Type = object
  loc: uint

type EXTI_EMR_Type = object
  loc: uint

type EXTI_IMR_Type = object
  loc: uint

type EXTI_Type = object
  IMR*: EXTI_IMR_Type
  EMR*: EXTI_EMR_Type
  RTSR*: EXTI_RTSR_Type
  FTSR*: EXTI_FTSR_Type
  SWIER*: EXTI_SWIER_Type
  PR*: EXTI_PR_Type

type DMA1_CMAR7_Type = object
  loc: uint

type DMA1_CPAR7_Type = object
  loc: uint

type DMA1_CNDTR7_Type = object
  loc: uint

type DMA1_CCR7_Type = object
  loc: uint

type DMA1_CMAR6_Type = object
  loc: uint

type DMA1_CPAR6_Type = object
  loc: uint

type DMA1_CNDTR6_Type = object
  loc: uint

type DMA1_CCR6_Type = object
  loc: uint

type DMA1_CMAR5_Type = object
  loc: uint

type DMA1_CPAR5_Type = object
  loc: uint

type DMA1_CNDTR5_Type = object
  loc: uint

type DMA1_CCR5_Type = object
  loc: uint

type DMA1_CMAR4_Type = object
  loc: uint

type DMA1_CPAR4_Type = object
  loc: uint

type DMA1_CNDTR4_Type = object
  loc: uint

type DMA1_CCR4_Type = object
  loc: uint

type DMA1_CMAR3_Type = object
  loc: uint

type DMA1_CPAR3_Type = object
  loc: uint

type DMA1_CNDTR3_Type = object
  loc: uint

type DMA1_CCR3_Type = object
  loc: uint

type DMA1_CMAR2_Type = object
  loc: uint

type DMA1_CPAR2_Type = object
  loc: uint

type DMA1_CNDTR2_Type = object
  loc: uint

type DMA1_CCR2_Type = object
  loc: uint

type DMA1_CMAR1_Type = object
  loc: uint

type DMA1_CPAR1_Type = object
  loc: uint

type DMA1_CNDTR1_Type = object
  loc: uint

type DMA1_CCR1_Type = object
  loc: uint

type DMA1_IFCR_Type = object
  loc: uint

type DMA1_ISR_Type = object
  loc: uint

type DMA1_Type = object
  ISR*: DMA1_ISR_Type
  IFCR*: DMA1_IFCR_Type
  CCR1*: DMA1_CCR1_Type
  CNDTR1*: DMA1_CNDTR1_Type
  CPAR1*: DMA1_CPAR1_Type
  CMAR1*: DMA1_CMAR1_Type
  CCR2*: DMA1_CCR2_Type
  CNDTR2*: DMA1_CNDTR2_Type
  CPAR2*: DMA1_CPAR2_Type
  CMAR2*: DMA1_CMAR2_Type
  CCR3*: DMA1_CCR3_Type
  CNDTR3*: DMA1_CNDTR3_Type
  CPAR3*: DMA1_CPAR3_Type
  CMAR3*: DMA1_CMAR3_Type
  CCR4*: DMA1_CCR4_Type
  CNDTR4*: DMA1_CNDTR4_Type
  CPAR4*: DMA1_CPAR4_Type
  CMAR4*: DMA1_CMAR4_Type
  CCR5*: DMA1_CCR5_Type
  CNDTR5*: DMA1_CNDTR5_Type
  CPAR5*: DMA1_CPAR5_Type
  CMAR5*: DMA1_CMAR5_Type
  CCR6*: DMA1_CCR6_Type
  CNDTR6*: DMA1_CNDTR6_Type
  CPAR6*: DMA1_CPAR6_Type
  CMAR6*: DMA1_CMAR6_Type
  CCR7*: DMA1_CCR7_Type
  CNDTR7*: DMA1_CNDTR7_Type
  CPAR7*: DMA1_CPAR7_Type
  CMAR7*: DMA1_CMAR7_Type

type SDIO_FIFO_Type = object
  loc: uint

type SDIO_FIFOCNT_Type = object
  loc: uint

type SDIO_MASK_Type = object
  loc: uint

type SDIO_ICR_Type = object
  loc: uint

type SDIO_STA_Type = object
  loc: uint

type SDIO_DCOUNT_Type = object
  loc: uint

type SDIO_DCTRL_Type = object
  loc: uint

type SDIO_DLEN_Type = object
  loc: uint

type SDIO_DTIMER_Type = object
  loc: uint

type SDIO_RESP4_Type = object
  loc: uint

type SDIO_RESP3_Type = object
  loc: uint

type SDIO_RESP2_Type = object
  loc: uint

type SDIO_RESPI1_Type = object
  loc: uint

type SDIO_RESPCMD_Type = object
  loc: uint

type SDIO_CMD_Type = object
  loc: uint

type SDIO_ARG_Type = object
  loc: uint

type SDIO_CLKCR_Type = object
  loc: uint

type SDIO_POWER_Type = object
  loc: uint

type SDIO_Type = object
  POWER*: SDIO_POWER_Type
  CLKCR*: SDIO_CLKCR_Type
  ARG*: SDIO_ARG_Type
  CMD*: SDIO_CMD_Type
  RESPCMD*: SDIO_RESPCMD_Type
  RESPI1*: SDIO_RESPI1_Type
  RESP2*: SDIO_RESP2_Type
  RESP3*: SDIO_RESP3_Type
  RESP4*: SDIO_RESP4_Type
  DTIMER*: SDIO_DTIMER_Type
  DLEN*: SDIO_DLEN_Type
  DCTRL*: SDIO_DCTRL_Type
  DCOUNT*: SDIO_DCOUNT_Type
  STA*: SDIO_STA_Type
  ICR*: SDIO_ICR_Type
  MASK*: SDIO_MASK_Type
  FIFOCNT*: SDIO_FIFOCNT_Type
  FIFO*: SDIO_FIFO_Type

type RTC_ALRL_Type = object
  loc: uint

type RTC_ALRH_Type = object
  loc: uint

type RTC_CNTL_Type = object
  loc: uint

type RTC_CNTH_Type = object
  loc: uint

type RTC_DIVL_Type = object
  loc: uint

type RTC_DIVH_Type = object
  loc: uint

type RTC_PRLL_Type = object
  loc: uint

type RTC_PRLH_Type = object
  loc: uint

type RTC_CRL_Type = object
  loc: uint

type RTC_CRH_Type = object
  loc: uint

type RTC_Type = object
  CRH*: RTC_CRH_Type
  CRL*: RTC_CRL_Type
  PRLH*: RTC_PRLH_Type
  PRLL*: RTC_PRLL_Type
  DIVH*: RTC_DIVH_Type
  DIVL*: RTC_DIVL_Type
  CNTH*: RTC_CNTH_Type
  CNTL*: RTC_CNTL_Type
  ALRH*: RTC_ALRH_Type
  ALRL*: RTC_ALRL_Type

type BKP_CSR_Type = object
  loc: uint

type BKP_CR_Type = object
  loc: uint

type BKP_RTCCR_Type = object
  loc: uint

type BKP_DR42_Type = object
  loc: uint

type BKP_DR41_Type = object
  loc: uint

type BKP_DR40_Type = object
  loc: uint

type BKP_DR39_Type = object
  loc: uint

type BKP_DR38_Type = object
  loc: uint

type BKP_DR37_Type = object
  loc: uint

type BKP_DR36_Type = object
  loc: uint

type BKP_DR35_Type = object
  loc: uint

type BKP_DR34_Type = object
  loc: uint

type BKP_DR33_Type = object
  loc: uint

type BKP_DR32_Type = object
  loc: uint

type BKP_DR31_Type = object
  loc: uint

type BKP_DR30_Type = object
  loc: uint

type BKP_DR29_Type = object
  loc: uint

type BKP_DR28_Type = object
  loc: uint

type BKP_DR27_Type = object
  loc: uint

type BKP_DR26_Type = object
  loc: uint

type BKP_DR25_Type = object
  loc: uint

type BKP_DR24_Type = object
  loc: uint

type BKP_DR23_Type = object
  loc: uint

type BKP_DR22_Type = object
  loc: uint

type BKP_DR21_Type = object
  loc: uint

type BKP_DR20_Type = object
  loc: uint

type BKP_DR19_Type = object
  loc: uint

type BKP_DR18_Type = object
  loc: uint

type BKP_DR17_Type = object
  loc: uint

type BKP_DR16_Type = object
  loc: uint

type BKP_DR15_Type = object
  loc: uint

type BKP_DR14_Type = object
  loc: uint

type BKP_DR13_Type = object
  loc: uint

type BKP_DR12_Type = object
  loc: uint

type BKP_DR11_Type = object
  loc: uint

type BKP_DR10_Type = object
  loc: uint

type BKP_DR9_Type = object
  loc: uint

type BKP_DR8_Type = object
  loc: uint

type BKP_DR7_Type = object
  loc: uint

type BKP_DR6_Type = object
  loc: uint

type BKP_DR5_Type = object
  loc: uint

type BKP_DR4_Type = object
  loc: uint

type BKP_DR3_Type = object
  loc: uint

type BKP_DR2_Type = object
  loc: uint

type BKP_DR1_Type = object
  loc: uint

type BKP_Type = object
  DR1*: BKP_DR1_Type
  DR2*: BKP_DR2_Type
  DR3*: BKP_DR3_Type
  DR4*: BKP_DR4_Type
  DR5*: BKP_DR5_Type
  DR6*: BKP_DR6_Type
  DR7*: BKP_DR7_Type
  DR8*: BKP_DR8_Type
  DR9*: BKP_DR9_Type
  DR10*: BKP_DR10_Type
  RTCCR*: BKP_RTCCR_Type
  CR*: BKP_CR_Type
  CSR*: BKP_CSR_Type
  DR11*: BKP_DR11_Type
  DR12*: BKP_DR12_Type
  DR13*: BKP_DR13_Type
  DR14*: BKP_DR14_Type
  DR15*: BKP_DR15_Type
  DR16*: BKP_DR16_Type
  DR17*: BKP_DR17_Type
  DR18*: BKP_DR18_Type
  DR19*: BKP_DR19_Type
  DR20*: BKP_DR20_Type
  DR21*: BKP_DR21_Type
  DR22*: BKP_DR22_Type
  DR23*: BKP_DR23_Type
  DR24*: BKP_DR24_Type
  DR25*: BKP_DR25_Type
  DR26*: BKP_DR26_Type
  DR27*: BKP_DR27_Type
  DR28*: BKP_DR28_Type
  DR29*: BKP_DR29_Type
  DR30*: BKP_DR30_Type
  DR31*: BKP_DR31_Type
  DR32*: BKP_DR32_Type
  DR33*: BKP_DR33_Type
  DR34*: BKP_DR34_Type
  DR35*: BKP_DR35_Type
  DR36*: BKP_DR36_Type
  DR37*: BKP_DR37_Type
  DR38*: BKP_DR38_Type
  DR39*: BKP_DR39_Type
  DR40*: BKP_DR40_Type
  DR41*: BKP_DR41_Type
  DR42*: BKP_DR42_Type

type IWDG_SR_Type = object
  loc: uint

type IWDG_RLR_Type = object
  loc: uint

type IWDG_PR_Type = object
  loc: uint

type IWDG_KR_Type = object
  loc: uint

type IWDG_Type = object
  KR*: IWDG_KR_Type
  PR*: IWDG_PR_Type
  RLR*: IWDG_RLR_Type
  SR*: IWDG_SR_Type

type WWDG_SR_Type = object
  loc: uint

type WWDG_CFR_Type = object
  loc: uint

type WWDG_CR_Type = object
  loc: uint

type WWDG_Type = object
  CR*: WWDG_CR_Type
  CFR*: WWDG_CFR_Type
  SR*: WWDG_SR_Type

type TIM1_BDTR_Type = object
  loc: uint

type TIM1_RCR_Type = object
  loc: uint

type TIM1_DMAR_Type = object
  loc: uint

type TIM1_DCR_Type = object
  loc: uint

type TIM1_CCR4_Type = object
  loc: uint

type TIM1_CCR3_Type = object
  loc: uint

type TIM1_CCR2_Type = object
  loc: uint

type TIM1_CCR1_Type = object
  loc: uint

type TIM1_ARR_Type = object
  loc: uint

type TIM1_PSC_Type = object
  loc: uint

type TIM1_CNT_Type = object
  loc: uint

type TIM1_CCER_Type = object
  loc: uint

type TIM1_CCMR2_Input_Type = object
  loc: uint

type TIM1_CCMR2_Output_Type = object
  loc: uint

type TIM1_CCMR1_Input_Type = object
  loc: uint

type TIM1_CCMR1_Output_Type = object
  loc: uint

type TIM1_EGR_Type = object
  loc: uint

type TIM1_SR_Type = object
  loc: uint

type TIM1_DIER_Type = object
  loc: uint

type TIM1_SMCR_Type = object
  loc: uint

type TIM1_CR2_Type = object
  loc: uint

type TIM1_CR1_Type = object
  loc: uint

type TIM1_Type = object
  CR1*: TIM1_CR1_Type
  CR2*: TIM1_CR2_Type
  SMCR*: TIM1_SMCR_Type
  DIER*: TIM1_DIER_Type
  SR*: TIM1_SR_Type
  EGR*: TIM1_EGR_Type
  CCMR1_Output*: TIM1_CCMR1_Output_Type
  CCMR1_Input*: TIM1_CCMR1_Input_Type
  CCMR2_Output*: TIM1_CCMR2_Output_Type
  CCMR2_Input*: TIM1_CCMR2_Input_Type
  CCER*: TIM1_CCER_Type
  CNT*: TIM1_CNT_Type
  PSC*: TIM1_PSC_Type
  ARR*: TIM1_ARR_Type
  RCR*: TIM1_RCR_Type
  CCR1*: TIM1_CCR1_Type
  CCR2*: TIM1_CCR2_Type
  CCR3*: TIM1_CCR3_Type
  CCR4*: TIM1_CCR4_Type
  BDTR*: TIM1_BDTR_Type
  DCR*: TIM1_DCR_Type
  DMAR*: TIM1_DMAR_Type

type TIM2_DMAR_Type = object
  loc: uint

type TIM2_DCR_Type = object
  loc: uint

type TIM2_CCR4_Type = object
  loc: uint

type TIM2_CCR3_Type = object
  loc: uint

type TIM2_CCR2_Type = object
  loc: uint

type TIM2_CCR1_Type = object
  loc: uint

type TIM2_ARR_Type = object
  loc: uint

type TIM2_PSC_Type = object
  loc: uint

type TIM2_CNT_Type = object
  loc: uint

type TIM2_CCER_Type = object
  loc: uint

type TIM2_CCMR2_Input_Type = object
  loc: uint

type TIM2_CCMR2_Output_Type = object
  loc: uint

type TIM2_CCMR1_Input_Type = object
  loc: uint

type TIM2_CCMR1_Output_Type = object
  loc: uint

type TIM2_EGR_Type = object
  loc: uint

type TIM2_SR_Type = object
  loc: uint

type TIM2_DIER_Type = object
  loc: uint

type TIM2_SMCR_Type = object
  loc: uint

type TIM2_CR2_Type = object
  loc: uint

type TIM2_CR1_Type = object
  loc: uint

type TIM2_Type = object
  CR1*: TIM2_CR1_Type
  CR2*: TIM2_CR2_Type
  SMCR*: TIM2_SMCR_Type
  DIER*: TIM2_DIER_Type
  SR*: TIM2_SR_Type
  EGR*: TIM2_EGR_Type
  CCMR1_Output*: TIM2_CCMR1_Output_Type
  CCMR1_Input*: TIM2_CCMR1_Input_Type
  CCMR2_Output*: TIM2_CCMR2_Output_Type
  CCMR2_Input*: TIM2_CCMR2_Input_Type
  CCER*: TIM2_CCER_Type
  CNT*: TIM2_CNT_Type
  PSC*: TIM2_PSC_Type
  ARR*: TIM2_ARR_Type
  CCR1*: TIM2_CCR1_Type
  CCR2*: TIM2_CCR2_Type
  CCR3*: TIM2_CCR3_Type
  CCR4*: TIM2_CCR4_Type
  DCR*: TIM2_DCR_Type
  DMAR*: TIM2_DMAR_Type

type TIM9_CCR2_Type = object
  loc: uint

type TIM9_CCR1_Type = object
  loc: uint

type TIM9_ARR_Type = object
  loc: uint

type TIM9_PSC_Type = object
  loc: uint

type TIM9_CNT_Type = object
  loc: uint

type TIM9_CCER_Type = object
  loc: uint

type TIM9_CCMR1_Input_Type = object
  loc: uint

type TIM9_CCMR1_Output_Type = object
  loc: uint

type TIM9_EGR_Type = object
  loc: uint

type TIM9_SR_Type = object
  loc: uint

type TIM9_DIER_Type = object
  loc: uint

type TIM9_SMCR_Type = object
  loc: uint

type TIM9_CR2_Type = object
  loc: uint

type TIM9_CR1_Type = object
  loc: uint

type TIM9_Type = object
  CR1*: TIM9_CR1_Type
  CR2*: TIM9_CR2_Type
  SMCR*: TIM9_SMCR_Type
  DIER*: TIM9_DIER_Type
  SR*: TIM9_SR_Type
  EGR*: TIM9_EGR_Type
  CCMR1_Output*: TIM9_CCMR1_Output_Type
  CCMR1_Input*: TIM9_CCMR1_Input_Type
  CCER*: TIM9_CCER_Type
  CNT*: TIM9_CNT_Type
  PSC*: TIM9_PSC_Type
  ARR*: TIM9_ARR_Type
  CCR1*: TIM9_CCR1_Type
  CCR2*: TIM9_CCR2_Type

type TIM10_CCR1_Type = object
  loc: uint

type TIM10_ARR_Type = object
  loc: uint

type TIM10_PSC_Type = object
  loc: uint

type TIM10_CNT_Type = object
  loc: uint

type TIM10_CCER_Type = object
  loc: uint

type TIM10_CCMR1_Input_Type = object
  loc: uint

type TIM10_CCMR1_Output_Type = object
  loc: uint

type TIM10_EGR_Type = object
  loc: uint

type TIM10_SR_Type = object
  loc: uint

type TIM10_DIER_Type = object
  loc: uint

type TIM10_CR2_Type = object
  loc: uint

type TIM10_CR1_Type = object
  loc: uint

type TIM10_Type = object
  CR1*: TIM10_CR1_Type
  CR2*: TIM10_CR2_Type
  DIER*: TIM10_DIER_Type
  SR*: TIM10_SR_Type
  EGR*: TIM10_EGR_Type
  CCMR1_Output*: TIM10_CCMR1_Output_Type
  CCMR1_Input*: TIM10_CCMR1_Input_Type
  CCER*: TIM10_CCER_Type
  CNT*: TIM10_CNT_Type
  PSC*: TIM10_PSC_Type
  ARR*: TIM10_ARR_Type
  CCR1*: TIM10_CCR1_Type

type TIM6_ARR_Type = object
  loc: uint

type TIM6_PSC_Type = object
  loc: uint

type TIM6_CNT_Type = object
  loc: uint

type TIM6_EGR_Type = object
  loc: uint

type TIM6_SR_Type = object
  loc: uint

type TIM6_DIER_Type = object
  loc: uint

type TIM6_CR2_Type = object
  loc: uint

type TIM6_CR1_Type = object
  loc: uint

type TIM6_Type = object
  CR1*: TIM6_CR1_Type
  CR2*: TIM6_CR2_Type
  DIER*: TIM6_DIER_Type
  SR*: TIM6_SR_Type
  EGR*: TIM6_EGR_Type
  CNT*: TIM6_CNT_Type
  PSC*: TIM6_PSC_Type
  ARR*: TIM6_ARR_Type

type I2C1_TRISE_Type = object
  loc: uint

type I2C1_CCR_Type = object
  loc: uint

type I2C1_SR2_Type = object
  loc: uint

type I2C1_SR1_Type = object
  loc: uint

type I2C1_DR_Type = object
  loc: uint

type I2C1_OAR2_Type = object
  loc: uint

type I2C1_OAR1_Type = object
  loc: uint

type I2C1_CR2_Type = object
  loc: uint

type I2C1_CR1_Type = object
  loc: uint

type I2C1_Type = object
  CR1*: I2C1_CR1_Type
  CR2*: I2C1_CR2_Type
  OAR1*: I2C1_OAR1_Type
  OAR2*: I2C1_OAR2_Type
  DR*: I2C1_DR_Type
  SR1*: I2C1_SR1_Type
  SR2*: I2C1_SR2_Type
  CCR*: I2C1_CCR_Type
  TRISE*: I2C1_TRISE_Type

type SPI1_I2SPR_Type = object
  loc: uint

type SPI1_I2SCFGR_Type = object
  loc: uint

type SPI1_TXCRCR_Type = object
  loc: uint

type SPI1_RXCRCR_Type = object
  loc: uint

type SPI1_CRCPR_Type = object
  loc: uint

type SPI1_DR_Type = object
  loc: uint

type SPI1_SR_Type = object
  loc: uint

type SPI1_CR2_Type = object
  loc: uint

type SPI1_CR1_Type = object
  loc: uint

type SPI1_Type = object
  CR1*: SPI1_CR1_Type
  CR2*: SPI1_CR2_Type
  SR*: SPI1_SR_Type
  DR*: SPI1_DR_Type
  CRCPR*: SPI1_CRCPR_Type
  RXCRCR*: SPI1_RXCRCR_Type
  TXCRCR*: SPI1_TXCRCR_Type
  I2SCFGR*: SPI1_I2SCFGR_Type
  I2SPR*: SPI1_I2SPR_Type

type USART1_GTPR_Type = object
  loc: uint

type USART1_CR3_Type = object
  loc: uint

type USART1_CR2_Type = object
  loc: uint

type USART1_CR1_Type = object
  loc: uint

type USART1_BRR_Type = object
  loc: uint

type USART1_DR_Type = object
  loc: uint

type USART1_SR_Type = object
  loc: uint

type USART1_Type = object
  SR*: USART1_SR_Type
  DR*: USART1_DR_Type
  BRR*: USART1_BRR_Type
  CR1*: USART1_CR1_Type
  CR2*: USART1_CR2_Type
  CR3*: USART1_CR3_Type
  GTPR*: USART1_GTPR_Type

type ADC1_DR_Type = object
  loc: uint

type ADC1_JDR4_Type = object
  loc: uint

type ADC1_JDR3_Type = object
  loc: uint

type ADC1_JDR2_Type = object
  loc: uint

type ADC1_JDR1_Type = object
  loc: uint

type ADC1_JSQR_Type = object
  loc: uint

type ADC1_SQR3_Type = object
  loc: uint

type ADC1_SQR2_Type = object
  loc: uint

type ADC1_SQR1_Type = object
  loc: uint

type ADC1_LTR_Type = object
  loc: uint

type ADC1_HTR_Type = object
  loc: uint

type ADC1_JOFR4_Type = object
  loc: uint

type ADC1_JOFR3_Type = object
  loc: uint

type ADC1_JOFR2_Type = object
  loc: uint

type ADC1_JOFR1_Type = object
  loc: uint

type ADC1_SMPR2_Type = object
  loc: uint

type ADC1_SMPR1_Type = object
  loc: uint

type ADC1_CR2_Type = object
  loc: uint

type ADC1_CR1_Type = object
  loc: uint

type ADC1_SR_Type = object
  loc: uint

type ADC1_Type = object
  SR*: ADC1_SR_Type
  CR1*: ADC1_CR1_Type
  CR2*: ADC1_CR2_Type
  SMPR1*: ADC1_SMPR1_Type
  SMPR2*: ADC1_SMPR2_Type
  JOFR1*: ADC1_JOFR1_Type
  JOFR2*: ADC1_JOFR2_Type
  JOFR3*: ADC1_JOFR3_Type
  JOFR4*: ADC1_JOFR4_Type
  HTR*: ADC1_HTR_Type
  LTR*: ADC1_LTR_Type
  SQR1*: ADC1_SQR1_Type
  SQR2*: ADC1_SQR2_Type
  SQR3*: ADC1_SQR3_Type
  JSQR*: ADC1_JSQR_Type
  JDR1*: ADC1_JDR1_Type
  JDR2*: ADC1_JDR2_Type
  JDR3*: ADC1_JDR3_Type
  JDR4*: ADC1_JDR4_Type
  DR*: ADC1_DR_Type

type ADC2_DR_Type = object
  loc: uint

type ADC2_JDR4_Type = object
  loc: uint

type ADC2_JDR3_Type = object
  loc: uint

type ADC2_JDR2_Type = object
  loc: uint

type ADC2_JDR1_Type = object
  loc: uint

type ADC2_JSQR_Type = object
  loc: uint

type ADC2_SQR3_Type = object
  loc: uint

type ADC2_SQR2_Type = object
  loc: uint

type ADC2_SQR1_Type = object
  loc: uint

type ADC2_LTR_Type = object
  loc: uint

type ADC2_HTR_Type = object
  loc: uint

type ADC2_JOFR4_Type = object
  loc: uint

type ADC2_JOFR3_Type = object
  loc: uint

type ADC2_JOFR2_Type = object
  loc: uint

type ADC2_JOFR1_Type = object
  loc: uint

type ADC2_SMPR2_Type = object
  loc: uint

type ADC2_SMPR1_Type = object
  loc: uint

type ADC2_CR2_Type = object
  loc: uint

type ADC2_CR1_Type = object
  loc: uint

type ADC2_SR_Type = object
  loc: uint

type ADC2_Type = object
  SR*: ADC2_SR_Type
  CR1*: ADC2_CR1_Type
  CR2*: ADC2_CR2_Type
  SMPR1*: ADC2_SMPR1_Type
  SMPR2*: ADC2_SMPR2_Type
  JOFR1*: ADC2_JOFR1_Type
  JOFR2*: ADC2_JOFR2_Type
  JOFR3*: ADC2_JOFR3_Type
  JOFR4*: ADC2_JOFR4_Type
  HTR*: ADC2_HTR_Type
  LTR*: ADC2_LTR_Type
  SQR1*: ADC2_SQR1_Type
  SQR2*: ADC2_SQR2_Type
  SQR3*: ADC2_SQR3_Type
  JSQR*: ADC2_JSQR_Type
  JDR1*: ADC2_JDR1_Type
  JDR2*: ADC2_JDR2_Type
  JDR3*: ADC2_JDR3_Type
  JDR4*: ADC2_JDR4_Type
  DR*: ADC2_DR_Type

type CAN1_F13R2_Type = object
  loc: uint

type CAN1_F13R1_Type = object
  loc: uint

type CAN1_F12R2_Type = object
  loc: uint

type CAN1_F12R1_Type = object
  loc: uint

type CAN1_F11R2_Type = object
  loc: uint

type CAN1_F11R1_Type = object
  loc: uint

type CAN1_F10R2_Type = object
  loc: uint

type CAN1_F10R1_Type = object
  loc: uint

type CAN1_F9R2_Type = object
  loc: uint

type CAN1_F9R1_Type = object
  loc: uint

type CAN1_F8R2_Type = object
  loc: uint

type CAN1_F8R1_Type = object
  loc: uint

type CAN1_F7R2_Type = object
  loc: uint

type CAN1_F7R1_Type = object
  loc: uint

type CAN1_F6R2_Type = object
  loc: uint

type CAN1_F6R1_Type = object
  loc: uint

type CAN1_F5R2_Type = object
  loc: uint

type CAN1_F5R1_Type = object
  loc: uint

type CAN1_F4R2_Type = object
  loc: uint

type CAN1_F4R1_Type = object
  loc: uint

type CAN1_F3R2_Type = object
  loc: uint

type CAN1_F3R1_Type = object
  loc: uint

type CAN1_F2R2_Type = object
  loc: uint

type CAN1_F2R1_Type = object
  loc: uint

type CAN1_F1R2_Type = object
  loc: uint

type CAN1_F1R1_Type = object
  loc: uint

type CAN1_F0R2_Type = object
  loc: uint

type CAN1_F0R1_Type = object
  loc: uint

type CAN1_CAN_FA1R_Type = object
  loc: uint

type CAN1_CAN_FFA1R_Type = object
  loc: uint

type CAN1_CAN_FS1R_Type = object
  loc: uint

type CAN1_CAN_FM1R_Type = object
  loc: uint

type CAN1_CAN_FMR_Type = object
  loc: uint

type CAN1_CAN_RDH1R_Type = object
  loc: uint

type CAN1_CAN_RDL1R_Type = object
  loc: uint

type CAN1_CAN_RDT1R_Type = object
  loc: uint

type CAN1_CAN_RI1R_Type = object
  loc: uint

type CAN1_CAN_RDH0R_Type = object
  loc: uint

type CAN1_CAN_RDL0R_Type = object
  loc: uint

type CAN1_CAN_RDT0R_Type = object
  loc: uint

type CAN1_CAN_RI0R_Type = object
  loc: uint

type CAN1_CAN_TDH2R_Type = object
  loc: uint

type CAN1_CAN_TDL2R_Type = object
  loc: uint

type CAN1_CAN_TDT2R_Type = object
  loc: uint

type CAN1_CAN_TI2R_Type = object
  loc: uint

type CAN1_CAN_TDH1R_Type = object
  loc: uint

type CAN1_CAN_TDL1R_Type = object
  loc: uint

type CAN1_CAN_TDT1R_Type = object
  loc: uint

type CAN1_CAN_TI1R_Type = object
  loc: uint

type CAN1_CAN_TDH0R_Type = object
  loc: uint

type CAN1_CAN_TDL0R_Type = object
  loc: uint

type CAN1_CAN_TDT0R_Type = object
  loc: uint

type CAN1_CAN_TI0R_Type = object
  loc: uint

type CAN1_CAN_BTR_Type = object
  loc: uint

type CAN1_CAN_ESR_Type = object
  loc: uint

type CAN1_CAN_IER_Type = object
  loc: uint

type CAN1_CAN_RF1R_Type = object
  loc: uint

type CAN1_CAN_RF0R_Type = object
  loc: uint

type CAN1_CAN_TSR_Type = object
  loc: uint

type CAN1_CAN_MSR_Type = object
  loc: uint

type CAN1_CAN_MCR_Type = object
  loc: uint

type CAN1_Type = object
  CAN_MCR*: CAN1_CAN_MCR_Type
  CAN_MSR*: CAN1_CAN_MSR_Type
  CAN_TSR*: CAN1_CAN_TSR_Type
  CAN_RF0R*: CAN1_CAN_RF0R_Type
  CAN_RF1R*: CAN1_CAN_RF1R_Type
  CAN_IER*: CAN1_CAN_IER_Type
  CAN_ESR*: CAN1_CAN_ESR_Type
  CAN_BTR*: CAN1_CAN_BTR_Type
  CAN_TI0R*: CAN1_CAN_TI0R_Type
  CAN_TDT0R*: CAN1_CAN_TDT0R_Type
  CAN_TDL0R*: CAN1_CAN_TDL0R_Type
  CAN_TDH0R*: CAN1_CAN_TDH0R_Type
  CAN_TI1R*: CAN1_CAN_TI1R_Type
  CAN_TDT1R*: CAN1_CAN_TDT1R_Type
  CAN_TDL1R*: CAN1_CAN_TDL1R_Type
  CAN_TDH1R*: CAN1_CAN_TDH1R_Type
  CAN_TI2R*: CAN1_CAN_TI2R_Type
  CAN_TDT2R*: CAN1_CAN_TDT2R_Type
  CAN_TDL2R*: CAN1_CAN_TDL2R_Type
  CAN_TDH2R*: CAN1_CAN_TDH2R_Type
  CAN_RI0R*: CAN1_CAN_RI0R_Type
  CAN_RDT0R*: CAN1_CAN_RDT0R_Type
  CAN_RDL0R*: CAN1_CAN_RDL0R_Type
  CAN_RDH0R*: CAN1_CAN_RDH0R_Type
  CAN_RI1R*: CAN1_CAN_RI1R_Type
  CAN_RDT1R*: CAN1_CAN_RDT1R_Type
  CAN_RDL1R*: CAN1_CAN_RDL1R_Type
  CAN_RDH1R*: CAN1_CAN_RDH1R_Type
  CAN_FMR*: CAN1_CAN_FMR_Type
  CAN_FM1R*: CAN1_CAN_FM1R_Type
  CAN_FS1R*: CAN1_CAN_FS1R_Type
  CAN_FFA1R*: CAN1_CAN_FFA1R_Type
  CAN_FA1R*: CAN1_CAN_FA1R_Type
  F0R1*: CAN1_F0R1_Type
  F0R2*: CAN1_F0R2_Type
  F1R1*: CAN1_F1R1_Type
  F1R2*: CAN1_F1R2_Type
  F2R1*: CAN1_F2R1_Type
  F2R2*: CAN1_F2R2_Type
  F3R1*: CAN1_F3R1_Type
  F3R2*: CAN1_F3R2_Type
  F4R1*: CAN1_F4R1_Type
  F4R2*: CAN1_F4R2_Type
  F5R1*: CAN1_F5R1_Type
  F5R2*: CAN1_F5R2_Type
  F6R1*: CAN1_F6R1_Type
  F6R2*: CAN1_F6R2_Type
  F7R1*: CAN1_F7R1_Type
  F7R2*: CAN1_F7R2_Type
  F8R1*: CAN1_F8R1_Type
  F8R2*: CAN1_F8R2_Type
  F9R1*: CAN1_F9R1_Type
  F9R2*: CAN1_F9R2_Type
  F10R1*: CAN1_F10R1_Type
  F10R2*: CAN1_F10R2_Type
  F11R1*: CAN1_F11R1_Type
  F11R2*: CAN1_F11R2_Type
  F12R1*: CAN1_F12R1_Type
  F12R2*: CAN1_F12R2_Type
  F13R1*: CAN1_F13R1_Type
  F13R2*: CAN1_F13R2_Type

type DAC_DOR2_Type = object
  loc: uint

type DAC_DOR1_Type = object
  loc: uint

type DAC_DHR8RD_Type = object
  loc: uint

type DAC_DHR12LD_Type = object
  loc: uint

type DAC_DHR12RD_Type = object
  loc: uint

type DAC_DHR8R2_Type = object
  loc: uint

type DAC_DHR12L2_Type = object
  loc: uint

type DAC_DHR12R2_Type = object
  loc: uint

type DAC_DHR8R1_Type = object
  loc: uint

type DAC_DHR12L1_Type = object
  loc: uint

type DAC_DHR12R1_Type = object
  loc: uint

type DAC_SWTRIGR_Type = object
  loc: uint

type DAC_CR_Type = object
  loc: uint

type DAC_Type = object
  CR*: DAC_CR_Type
  SWTRIGR*: DAC_SWTRIGR_Type
  DHR12R1*: DAC_DHR12R1_Type
  DHR12L1*: DAC_DHR12L1_Type
  DHR8R1*: DAC_DHR8R1_Type
  DHR12R2*: DAC_DHR12R2_Type
  DHR12L2*: DAC_DHR12L2_Type
  DHR8R2*: DAC_DHR8R2_Type
  DHR12RD*: DAC_DHR12RD_Type
  DHR12LD*: DAC_DHR12LD_Type
  DHR8RD*: DAC_DHR8RD_Type
  DOR1*: DAC_DOR1_Type
  DOR2*: DAC_DOR2_Type

type DBG_CR_Type = object
  loc: uint

type DBG_IDCODE_Type = object
  loc: uint

type DBG_Type = object
  IDCODE*: DBG_IDCODE_Type
  CR*: DBG_CR_Type

type UART4_CR3_Type = object
  loc: uint

type UART4_CR2_Type = object
  loc: uint

type UART4_CR1_Type = object
  loc: uint

type UART4_BRR_Type = object
  loc: uint

type UART4_DR_Type = object
  loc: uint

type UART4_SR_Type = object
  loc: uint

type UART4_Type = object
  SR*: UART4_SR_Type
  DR*: UART4_DR_Type
  BRR*: UART4_BRR_Type
  CR1*: UART4_CR1_Type
  CR2*: UART4_CR2_Type
  CR3*: UART4_CR3_Type

type UART5_CR3_Type = object
  loc: uint

type UART5_CR2_Type = object
  loc: uint

type UART5_CR1_Type = object
  loc: uint

type UART5_BRR_Type = object
  loc: uint

type UART5_DR_Type = object
  loc: uint

type UART5_SR_Type = object
  loc: uint

type UART5_Type = object
  SR*: UART5_SR_Type
  DR*: UART5_DR_Type
  BRR*: UART5_BRR_Type
  CR1*: UART5_CR1_Type
  CR2*: UART5_CR2_Type
  CR3*: UART5_CR3_Type

type CRC_CR_Type = object
  loc: uint

type CRC_IDR_Type = object
  loc: uint

type CRC_DR_Type = object
  loc: uint

type CRC_Type = object
  DR*: CRC_DR_Type
  IDR*: CRC_IDR_Type
  CR*: CRC_CR_Type

type FLASH_WRPR_Type = object
  loc: uint

type FLASH_OBR_Type = object
  loc: uint

type FLASH_AR_Type = object
  loc: uint

type FLASH_CR_Type = object
  loc: uint

type FLASH_SR_Type = object
  loc: uint

type FLASH_OPTKEYR_Type = object
  loc: uint

type FLASH_KEYR_Type = object
  loc: uint

type FLASH_ACR_Type = object
  loc: uint

type FLASH_Type = object
  ACR*: FLASH_ACR_Type
  KEYR*: FLASH_KEYR_Type
  OPTKEYR*: FLASH_OPTKEYR_Type
  SR*: FLASH_SR_Type
  CR*: FLASH_CR_Type
  AR*: FLASH_AR_Type
  OBR*: FLASH_OBR_Type
  WRPR*: FLASH_WRPR_Type

type USB_BTABLE_Type = object
  loc: uint

type USB_DADDR_Type = object
  loc: uint

type USB_FNR_Type = object
  loc: uint

type USB_ISTR_Type = object
  loc: uint

type USB_CNTR_Type = object
  loc: uint

type USB_EP7R_Type = object
  loc: uint

type USB_EP6R_Type = object
  loc: uint

type USB_EP5R_Type = object
  loc: uint

type USB_EP4R_Type = object
  loc: uint

type USB_EP3R_Type = object
  loc: uint

type USB_EP2R_Type = object
  loc: uint

type USB_EP1R_Type = object
  loc: uint

type USB_EP0R_Type = object
  loc: uint

type USB_Type = object
  EP0R*: USB_EP0R_Type
  EP1R*: USB_EP1R_Type
  EP2R*: USB_EP2R_Type
  EP3R*: USB_EP3R_Type
  EP4R*: USB_EP4R_Type
  EP5R*: USB_EP5R_Type
  EP6R*: USB_EP6R_Type
  EP7R*: USB_EP7R_Type
  CNTR*: USB_CNTR_Type
  ISTR*: USB_ISTR_Type
  FNR*: USB_FNR_Type
  DADDR*: USB_DADDR_Type
  BTABLE*: USB_BTABLE_Type

type OTG_FS_DEVICE_DOEPTSIZ3_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPTSIZ2_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPTSIZ1_Type = object
  loc: uint

type OTG_FS_DEVICE_DTXFSTS3_Type = object
  loc: uint

type OTG_FS_DEVICE_DTXFSTS2_Type = object
  loc: uint

type OTG_FS_DEVICE_DTXFSTS1_Type = object
  loc: uint

type OTG_FS_DEVICE_DTXFSTS0_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPTSIZ3_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPTSIZ2_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPTSIZ1_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPTSIZ0_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPTSIZ0_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPINT3_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPINT2_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPINT1_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPINT0_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPINT3_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPINT2_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPINT1_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPINT0_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPCTL3_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPCTL2_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPCTL1_Type = object
  loc: uint

type OTG_FS_DEVICE_DOEPCTL0_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPCTL3_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPCTL2_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPCTL1_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DIEPCTL0_Type = object
  loc: uint

type OTG_FS_DEVICE_DIEPEMPMSK_Type = object
  loc: uint

type OTG_FS_DEVICE_DVBUSPULSE_Type = object
  loc: uint

type OTG_FS_DEVICE_DVBUSDIS_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DAINTMSK_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DAINT_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DOEPMSK_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DIEPMSK_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DSTS_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DCTL_Type = object
  loc: uint

type OTG_FS_DEVICE_FS_DCFG_Type = object
  loc: uint

type OTG_FS_DEVICE_Type = object
  FS_DCFG*: OTG_FS_DEVICE_FS_DCFG_Type
  FS_DCTL*: OTG_FS_DEVICE_FS_DCTL_Type
  FS_DSTS*: OTG_FS_DEVICE_FS_DSTS_Type
  FS_DIEPMSK*: OTG_FS_DEVICE_FS_DIEPMSK_Type
  FS_DOEPMSK*: OTG_FS_DEVICE_FS_DOEPMSK_Type
  FS_DAINT*: OTG_FS_DEVICE_FS_DAINT_Type
  FS_DAINTMSK*: OTG_FS_DEVICE_FS_DAINTMSK_Type
  DVBUSDIS*: OTG_FS_DEVICE_DVBUSDIS_Type
  DVBUSPULSE*: OTG_FS_DEVICE_DVBUSPULSE_Type
  DIEPEMPMSK*: OTG_FS_DEVICE_DIEPEMPMSK_Type
  FS_DIEPCTL0*: OTG_FS_DEVICE_FS_DIEPCTL0_Type
  DIEPINT0*: OTG_FS_DEVICE_DIEPINT0_Type
  DIEPTSIZ0*: OTG_FS_DEVICE_DIEPTSIZ0_Type
  DTXFSTS0*: OTG_FS_DEVICE_DTXFSTS0_Type
  DIEPCTL1*: OTG_FS_DEVICE_DIEPCTL1_Type
  DIEPINT1*: OTG_FS_DEVICE_DIEPINT1_Type
  DIEPTSIZ1*: OTG_FS_DEVICE_DIEPTSIZ1_Type
  DTXFSTS1*: OTG_FS_DEVICE_DTXFSTS1_Type
  DIEPCTL2*: OTG_FS_DEVICE_DIEPCTL2_Type
  DIEPINT2*: OTG_FS_DEVICE_DIEPINT2_Type
  DIEPTSIZ2*: OTG_FS_DEVICE_DIEPTSIZ2_Type
  DTXFSTS2*: OTG_FS_DEVICE_DTXFSTS2_Type
  DIEPCTL3*: OTG_FS_DEVICE_DIEPCTL3_Type
  DIEPINT3*: OTG_FS_DEVICE_DIEPINT3_Type
  DIEPTSIZ3*: OTG_FS_DEVICE_DIEPTSIZ3_Type
  DTXFSTS3*: OTG_FS_DEVICE_DTXFSTS3_Type
  DOEPCTL0*: OTG_FS_DEVICE_DOEPCTL0_Type
  DOEPINT0*: OTG_FS_DEVICE_DOEPINT0_Type
  DOEPTSIZ0*: OTG_FS_DEVICE_DOEPTSIZ0_Type
  DOEPCTL1*: OTG_FS_DEVICE_DOEPCTL1_Type
  DOEPINT1*: OTG_FS_DEVICE_DOEPINT1_Type
  DOEPTSIZ1*: OTG_FS_DEVICE_DOEPTSIZ1_Type
  DOEPCTL2*: OTG_FS_DEVICE_DOEPCTL2_Type
  DOEPINT2*: OTG_FS_DEVICE_DOEPINT2_Type
  DOEPTSIZ2*: OTG_FS_DEVICE_DOEPTSIZ2_Type
  DOEPCTL3*: OTG_FS_DEVICE_DOEPCTL3_Type
  DOEPINT3*: OTG_FS_DEVICE_DOEPINT3_Type
  DOEPTSIZ3*: OTG_FS_DEVICE_DOEPTSIZ3_Type

type OTG_FS_GLOBAL_FS_DIEPTXF3_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_DIEPTXF2_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_DIEPTXF1_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_HPTXFSIZ_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_CID_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GCCFG_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GNPTXSTS_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GRXFSIZ_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GRXSTSR_Host_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GRXSTSR_Device_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GINTMSK_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GINTSTS_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GRSTCTL_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GUSBCFG_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GAHBCFG_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GOTGINT_Type = object
  loc: uint

type OTG_FS_GLOBAL_FS_GOTGCTL_Type = object
  loc: uint

type OTG_FS_GLOBAL_Type = object
  FS_GOTGCTL*: OTG_FS_GLOBAL_FS_GOTGCTL_Type
  FS_GOTGINT*: OTG_FS_GLOBAL_FS_GOTGINT_Type
  FS_GAHBCFG*: OTG_FS_GLOBAL_FS_GAHBCFG_Type
  FS_GUSBCFG*: OTG_FS_GLOBAL_FS_GUSBCFG_Type
  FS_GRSTCTL*: OTG_FS_GLOBAL_FS_GRSTCTL_Type
  FS_GINTSTS*: OTG_FS_GLOBAL_FS_GINTSTS_Type
  FS_GINTMSK*: OTG_FS_GLOBAL_FS_GINTMSK_Type
  FS_GRXSTSR_Device*: OTG_FS_GLOBAL_FS_GRXSTSR_Device_Type
  FS_GRXSTSR_Host*: OTG_FS_GLOBAL_FS_GRXSTSR_Host_Type
  FS_GRXFSIZ*: OTG_FS_GLOBAL_FS_GRXFSIZ_Type
  FS_GNPTXFSIZ_Device*: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Type
  FS_GNPTXFSIZ_Host*: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Type
  FS_GNPTXSTS*: OTG_FS_GLOBAL_FS_GNPTXSTS_Type
  FS_GCCFG*: OTG_FS_GLOBAL_FS_GCCFG_Type
  FS_CID*: OTG_FS_GLOBAL_FS_CID_Type
  FS_HPTXFSIZ*: OTG_FS_GLOBAL_FS_HPTXFSIZ_Type
  FS_DIEPTXF1*: OTG_FS_GLOBAL_FS_DIEPTXF1_Type
  FS_DIEPTXF2*: OTG_FS_GLOBAL_FS_DIEPTXF2_Type
  FS_DIEPTXF3*: OTG_FS_GLOBAL_FS_DIEPTXF3_Type

type OTG_FS_HOST_FS_HCTSIZ7_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ6_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ5_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ4_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ3_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ2_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ1_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCTSIZ0_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK7_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK6_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK5_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK4_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK3_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK2_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK1_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINTMSK0_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT7_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT6_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT5_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT4_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT3_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT2_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT1_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCINT0_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR7_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR6_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR5_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR4_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR3_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR2_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR1_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCCHAR0_Type = object
  loc: uint

type OTG_FS_HOST_FS_HPRT_Type = object
  loc: uint

type OTG_FS_HOST_HAINTMSK_Type = object
  loc: uint

type OTG_FS_HOST_HAINT_Type = object
  loc: uint

type OTG_FS_HOST_FS_HPTXSTS_Type = object
  loc: uint

type OTG_FS_HOST_FS_HFNUM_Type = object
  loc: uint

type OTG_FS_HOST_HFIR_Type = object
  loc: uint

type OTG_FS_HOST_FS_HCFG_Type = object
  loc: uint

type OTG_FS_HOST_Type = object
  FS_HCFG*: OTG_FS_HOST_FS_HCFG_Type
  HFIR*: OTG_FS_HOST_HFIR_Type
  FS_HFNUM*: OTG_FS_HOST_FS_HFNUM_Type
  FS_HPTXSTS*: OTG_FS_HOST_FS_HPTXSTS_Type
  HAINT*: OTG_FS_HOST_HAINT_Type
  HAINTMSK*: OTG_FS_HOST_HAINTMSK_Type
  FS_HPRT*: OTG_FS_HOST_FS_HPRT_Type
  FS_HCCHAR0*: OTG_FS_HOST_FS_HCCHAR0_Type
  FS_HCINT0*: OTG_FS_HOST_FS_HCINT0_Type
  FS_HCINTMSK0*: OTG_FS_HOST_FS_HCINTMSK0_Type
  FS_HCTSIZ0*: OTG_FS_HOST_FS_HCTSIZ0_Type
  FS_HCCHAR1*: OTG_FS_HOST_FS_HCCHAR1_Type
  FS_HCINT1*: OTG_FS_HOST_FS_HCINT1_Type
  FS_HCINTMSK1*: OTG_FS_HOST_FS_HCINTMSK1_Type
  FS_HCTSIZ1*: OTG_FS_HOST_FS_HCTSIZ1_Type
  FS_HCCHAR2*: OTG_FS_HOST_FS_HCCHAR2_Type
  FS_HCINT2*: OTG_FS_HOST_FS_HCINT2_Type
  FS_HCINTMSK2*: OTG_FS_HOST_FS_HCINTMSK2_Type
  FS_HCTSIZ2*: OTG_FS_HOST_FS_HCTSIZ2_Type
  FS_HCCHAR3*: OTG_FS_HOST_FS_HCCHAR3_Type
  FS_HCINT3*: OTG_FS_HOST_FS_HCINT3_Type
  FS_HCINTMSK3*: OTG_FS_HOST_FS_HCINTMSK3_Type
  FS_HCTSIZ3*: OTG_FS_HOST_FS_HCTSIZ3_Type
  FS_HCCHAR4*: OTG_FS_HOST_FS_HCCHAR4_Type
  FS_HCINT4*: OTG_FS_HOST_FS_HCINT4_Type
  FS_HCINTMSK4*: OTG_FS_HOST_FS_HCINTMSK4_Type
  FS_HCTSIZ4*: OTG_FS_HOST_FS_HCTSIZ4_Type
  FS_HCCHAR5*: OTG_FS_HOST_FS_HCCHAR5_Type
  FS_HCINT5*: OTG_FS_HOST_FS_HCINT5_Type
  FS_HCINTMSK5*: OTG_FS_HOST_FS_HCINTMSK5_Type
  FS_HCTSIZ5*: OTG_FS_HOST_FS_HCTSIZ5_Type
  FS_HCCHAR6*: OTG_FS_HOST_FS_HCCHAR6_Type
  FS_HCINT6*: OTG_FS_HOST_FS_HCINT6_Type
  FS_HCINTMSK6*: OTG_FS_HOST_FS_HCINTMSK6_Type
  FS_HCTSIZ6*: OTG_FS_HOST_FS_HCTSIZ6_Type
  FS_HCCHAR7*: OTG_FS_HOST_FS_HCCHAR7_Type
  FS_HCINT7*: OTG_FS_HOST_FS_HCINT7_Type
  FS_HCINTMSK7*: OTG_FS_HOST_FS_HCINTMSK7_Type
  FS_HCTSIZ7*: OTG_FS_HOST_FS_HCTSIZ7_Type

type OTG_FS_PWRCLK_FS_PCGCCTL_Type = object
  loc: uint

type OTG_FS_PWRCLK_Type = object
  FS_PCGCCTL*: OTG_FS_PWRCLK_FS_PCGCCTL_Type

type ETHERNET_MMC_MMCRGUFCR_Type = object
  loc: uint

type ETHERNET_MMC_MMCRFAECR_Type = object
  loc: uint

type ETHERNET_MMC_MMCRFCECR_Type = object
  loc: uint

type ETHERNET_MMC_MMCTGFCR_Type = object
  loc: uint

type ETHERNET_MMC_MMCTGFMSCCR_Type = object
  loc: uint

type ETHERNET_MMC_MMCTGFSCCR_Type = object
  loc: uint

type ETHERNET_MMC_MMCTIMR_Type = object
  loc: uint

type ETHERNET_MMC_MMCRIMR_Type = object
  loc: uint

type ETHERNET_MMC_MMCTIR_Type = object
  loc: uint

type ETHERNET_MMC_MMCRIR_Type = object
  loc: uint

type ETHERNET_MMC_MMCCR_Type = object
  loc: uint

type ETHERNET_MMC_Type = object
  MMCCR*: ETHERNET_MMC_MMCCR_Type
  MMCRIR*: ETHERNET_MMC_MMCRIR_Type
  MMCTIR*: ETHERNET_MMC_MMCTIR_Type
  MMCRIMR*: ETHERNET_MMC_MMCRIMR_Type
  MMCTIMR*: ETHERNET_MMC_MMCTIMR_Type
  MMCTGFSCCR*: ETHERNET_MMC_MMCTGFSCCR_Type
  MMCTGFMSCCR*: ETHERNET_MMC_MMCTGFMSCCR_Type
  MMCTGFCR*: ETHERNET_MMC_MMCTGFCR_Type
  MMCRFCECR*: ETHERNET_MMC_MMCRFCECR_Type
  MMCRFAECR*: ETHERNET_MMC_MMCRFAECR_Type
  MMCRGUFCR*: ETHERNET_MMC_MMCRGUFCR_Type

type ETHERNET_MAC_MACA3LR_Type = object
  loc: uint

type ETHERNET_MAC_MACA3HR_Type = object
  loc: uint

type ETHERNET_MAC_MACA2LR_Type = object
  loc: uint

type ETHERNET_MAC_MACA2HR_Type = object
  loc: uint

type ETHERNET_MAC_MACA1LR_Type = object
  loc: uint

type ETHERNET_MAC_MACA1HR_Type = object
  loc: uint

type ETHERNET_MAC_MACA0LR_Type = object
  loc: uint

type ETHERNET_MAC_MACA0HR_Type = object
  loc: uint

type ETHERNET_MAC_MACIMR_Type = object
  loc: uint

type ETHERNET_MAC_MACSR_Type = object
  loc: uint

type ETHERNET_MAC_MACPMTCSR_Type = object
  loc: uint

type ETHERNET_MAC_MACRWUFFR_Type = object
  loc: uint

type ETHERNET_MAC_MACVLANTR_Type = object
  loc: uint

type ETHERNET_MAC_MACFCR_Type = object
  loc: uint

type ETHERNET_MAC_MACMIIDR_Type = object
  loc: uint

type ETHERNET_MAC_MACMIIAR_Type = object
  loc: uint

type ETHERNET_MAC_MACHTLR_Type = object
  loc: uint

type ETHERNET_MAC_MACHTHR_Type = object
  loc: uint

type ETHERNET_MAC_MACFFR_Type = object
  loc: uint

type ETHERNET_MAC_MACCR_Type = object
  loc: uint

type ETHERNET_MAC_Type = object
  MACCR*: ETHERNET_MAC_MACCR_Type
  MACFFR*: ETHERNET_MAC_MACFFR_Type
  MACHTHR*: ETHERNET_MAC_MACHTHR_Type
  MACHTLR*: ETHERNET_MAC_MACHTLR_Type
  MACMIIAR*: ETHERNET_MAC_MACMIIAR_Type
  MACMIIDR*: ETHERNET_MAC_MACMIIDR_Type
  MACFCR*: ETHERNET_MAC_MACFCR_Type
  MACVLANTR*: ETHERNET_MAC_MACVLANTR_Type
  MACRWUFFR*: ETHERNET_MAC_MACRWUFFR_Type
  MACPMTCSR*: ETHERNET_MAC_MACPMTCSR_Type
  MACSR*: ETHERNET_MAC_MACSR_Type
  MACIMR*: ETHERNET_MAC_MACIMR_Type
  MACA0HR*: ETHERNET_MAC_MACA0HR_Type
  MACA0LR*: ETHERNET_MAC_MACA0LR_Type
  MACA1HR*: ETHERNET_MAC_MACA1HR_Type
  MACA1LR*: ETHERNET_MAC_MACA1LR_Type
  MACA2HR*: ETHERNET_MAC_MACA2HR_Type
  MACA2LR*: ETHERNET_MAC_MACA2LR_Type
  MACA3HR*: ETHERNET_MAC_MACA3HR_Type
  MACA3LR*: ETHERNET_MAC_MACA3LR_Type

type ETHERNET_PTP_PTPTTLR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTTHR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTSAR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTSLUR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTSHUR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTSLR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTSHR_Type = object
  loc: uint

type ETHERNET_PTP_PTPSSIR_Type = object
  loc: uint

type ETHERNET_PTP_PTPTSCR_Type = object
  loc: uint

type ETHERNET_PTP_Type = object
  PTPTSCR*: ETHERNET_PTP_PTPTSCR_Type
  PTPSSIR*: ETHERNET_PTP_PTPSSIR_Type
  PTPTSHR*: ETHERNET_PTP_PTPTSHR_Type
  PTPTSLR*: ETHERNET_PTP_PTPTSLR_Type
  PTPTSHUR*: ETHERNET_PTP_PTPTSHUR_Type
  PTPTSLUR*: ETHERNET_PTP_PTPTSLUR_Type
  PTPTSAR*: ETHERNET_PTP_PTPTSAR_Type
  PTPTTHR*: ETHERNET_PTP_PTPTTHR_Type
  PTPTTLR*: ETHERNET_PTP_PTPTTLR_Type

type ETHERNET_DMA_DMACHRBAR_Type = object
  loc: uint

type ETHERNET_DMA_DMACHTBAR_Type = object
  loc: uint

type ETHERNET_DMA_DMACHRDR_Type = object
  loc: uint

type ETHERNET_DMA_DMACHTDR_Type = object
  loc: uint

type ETHERNET_DMA_DMAMFBOCR_Type = object
  loc: uint

type ETHERNET_DMA_DMAIER_Type = object
  loc: uint

type ETHERNET_DMA_DMAOMR_Type = object
  loc: uint

type ETHERNET_DMA_DMASR_Type = object
  loc: uint

type ETHERNET_DMA_DMATDLAR_Type = object
  loc: uint

type ETHERNET_DMA_DMARDLAR_Type = object
  loc: uint

type ETHERNET_DMA_DMARPDR_Type = object
  loc: uint

type ETHERNET_DMA_DMATPDR_Type = object
  loc: uint

type ETHERNET_DMA_DMABMR_Type = object
  loc: uint

type ETHERNET_DMA_Type = object
  DMABMR*: ETHERNET_DMA_DMABMR_Type
  DMATPDR*: ETHERNET_DMA_DMATPDR_Type
  DMARPDR*: ETHERNET_DMA_DMARPDR_Type
  DMARDLAR*: ETHERNET_DMA_DMARDLAR_Type
  DMATDLAR*: ETHERNET_DMA_DMATDLAR_Type
  DMASR*: ETHERNET_DMA_DMASR_Type
  DMAOMR*: ETHERNET_DMA_DMAOMR_Type
  DMAIER*: ETHERNET_DMA_DMAIER_Type
  DMAMFBOCR*: ETHERNET_DMA_DMAMFBOCR_Type
  DMACHTDR*: ETHERNET_DMA_DMACHTDR_Type
  DMACHRDR*: ETHERNET_DMA_DMACHRDR_Type
  DMACHTBAR*: ETHERNET_DMA_DMACHTBAR_Type
  DMACHRBAR*: ETHERNET_DMA_DMACHRBAR_Type

type NVIC_IPR14_Type = object
  loc: uint

type NVIC_IPR13_Type = object
  loc: uint

type NVIC_IPR12_Type = object
  loc: uint

type NVIC_IPR11_Type = object
  loc: uint

type NVIC_IPR10_Type = object
  loc: uint

type NVIC_IPR9_Type = object
  loc: uint

type NVIC_IPR8_Type = object
  loc: uint

type NVIC_IPR7_Type = object
  loc: uint

type NVIC_IPR6_Type = object
  loc: uint

type NVIC_IPR5_Type = object
  loc: uint

type NVIC_IPR4_Type = object
  loc: uint

type NVIC_IPR3_Type = object
  loc: uint

type NVIC_IPR2_Type = object
  loc: uint

type NVIC_IPR1_Type = object
  loc: uint

type NVIC_IPR0_Type = object
  loc: uint

type NVIC_IABR1_Type = object
  loc: uint

type NVIC_IABR0_Type = object
  loc: uint

type NVIC_ICPR1_Type = object
  loc: uint

type NVIC_ICPR0_Type = object
  loc: uint

type NVIC_ISPR1_Type = object
  loc: uint

type NVIC_ISPR0_Type = object
  loc: uint

type NVIC_ICER1_Type = object
  loc: uint

type NVIC_ICER0_Type = object
  loc: uint

type NVIC_ISER1_Type = object
  loc: uint

type NVIC_ISER0_Type = object
  loc: uint

type NVIC_Type = object
  ISER0*: NVIC_ISER0_Type
  ISER1*: NVIC_ISER1_Type
  ICER0*: NVIC_ICER0_Type
  ICER1*: NVIC_ICER1_Type
  ISPR0*: NVIC_ISPR0_Type
  ISPR1*: NVIC_ISPR1_Type
  ICPR0*: NVIC_ICPR0_Type
  ICPR1*: NVIC_ICPR1_Type
  IABR0*: NVIC_IABR0_Type
  IABR1*: NVIC_IABR1_Type
  IPR0*: NVIC_IPR0_Type
  IPR1*: NVIC_IPR1_Type
  IPR2*: NVIC_IPR2_Type
  IPR3*: NVIC_IPR3_Type
  IPR4*: NVIC_IPR4_Type
  IPR5*: NVIC_IPR5_Type
  IPR6*: NVIC_IPR6_Type
  IPR7*: NVIC_IPR7_Type
  IPR8*: NVIC_IPR8_Type
  IPR9*: NVIC_IPR9_Type
  IPR10*: NVIC_IPR10_Type
  IPR11*: NVIC_IPR11_Type
  IPR12*: NVIC_IPR12_Type
  IPR13*: NVIC_IPR13_Type
  IPR14*: NVIC_IPR14_Type

type MPU_MPU_RASR_Type = object
  loc: uint

type MPU_MPU_RBAR_Type = object
  loc: uint

type MPU_MPU_RNR_Type = object
  loc: uint

type MPU_MPU_CTRL_Type = object
  loc: uint

type MPU_MPU_TYPER_Type = object
  loc: uint

type MPU_Type = object
  MPU_TYPER*: MPU_MPU_TYPER_Type
  MPU_CTRL*: MPU_MPU_CTRL_Type
  MPU_RNR*: MPU_MPU_RNR_Type
  MPU_RBAR*: MPU_MPU_RBAR_Type
  MPU_RASR*: MPU_MPU_RASR_Type

type SCB_ACTRL_ACTRL_Type = object
  loc: uint

type SCB_ACTRL_Type = object
  ACTRL*: SCB_ACTRL_ACTRL_Type

type NVIC_STIR_STIR_Type = object
  loc: uint

type NVIC_STIR_Type = object
  STIR*: NVIC_STIR_STIR_Type

type SCB_BFAR_Type = object
  loc: uint

type SCB_MMFAR_Type = object
  loc: uint

type SCB_HFSR_Type = object
  loc: uint

type SCB_CFSR_UFSR_BFSR_MMFSR_Type = object
  loc: uint

type SCB_SHCRS_Type = object
  loc: uint

type SCB_SHPR3_Type = object
  loc: uint

type SCB_SHPR2_Type = object
  loc: uint

type SCB_SHPR1_Type = object
  loc: uint

type SCB_CCR_Type = object
  loc: uint

type SCB_SCR_Type = object
  loc: uint

type SCB_AIRCR_Type = object
  loc: uint

type SCB_VTOR_Type = object
  loc: uint

type SCB_ICSR_Type = object
  loc: uint

type SCB_CPUID_Type = object
  loc: uint

type SCB_Type = object
  CPUID*: SCB_CPUID_Type
  ICSR*: SCB_ICSR_Type
  VTOR*: SCB_VTOR_Type
  AIRCR*: SCB_AIRCR_Type
  SCR*: SCB_SCR_Type
  CCR*: SCB_CCR_Type
  SHPR1*: SCB_SHPR1_Type
  SHPR2*: SCB_SHPR2_Type
  SHPR3*: SCB_SHPR3_Type
  SHCRS*: SCB_SHCRS_Type
  CFSR_UFSR_BFSR_MMFSR*: SCB_CFSR_UFSR_BFSR_MMFSR_Type
  HFSR*: SCB_HFSR_Type
  MMFAR*: SCB_MMFAR_Type
  BFAR*: SCB_BFAR_Type

type STK_CALIB_Type = object
  loc: uint

type STK_VAL_Type = object
  loc: uint

type STK_LOAD_Type = object
  loc: uint

type STK_CTRL_Type = object
  loc: uint

type STK_Type = object
  CTRL*: STK_CTRL_Type
  LOAD*: STK_LOAD_Type
  VAL*: STK_VAL_Type
  CALIB*: STK_CALIB_Type


################################################################################
# Peripheral object instances
################################################################################
const FSMC* = FSMC_Type(
  BCR1: FSMC_BCR1_Type(loc: 0xa0000000),
  BTR1: FSMC_BTR1_Type(loc: 0xa0000004),
  BCR2: FSMC_BCR2_Type(loc: 0xa0000008),
  BTR2: FSMC_BTR2_Type(loc: 0xa000000c),
  BCR3: FSMC_BCR3_Type(loc: 0xa0000010),
  BTR3: FSMC_BTR3_Type(loc: 0xa0000014),
  BCR4: FSMC_BCR4_Type(loc: 0xa0000018),
  BTR4: FSMC_BTR4_Type(loc: 0xa000001c),
  PCR2: FSMC_PCR2_Type(loc: 0xa0000060),
  SR2: FSMC_SR2_Type(loc: 0xa0000064),
  PMEM2: FSMC_PMEM2_Type(loc: 0xa0000068),
  PATT2: FSMC_PATT2_Type(loc: 0xa000006c),
  ECCR2: FSMC_ECCR2_Type(loc: 0xa0000074),
  PCR3: FSMC_PCR3_Type(loc: 0xa0000080),
  SR3: FSMC_SR3_Type(loc: 0xa0000084),
  PMEM3: FSMC_PMEM3_Type(loc: 0xa0000088),
  PATT3: FSMC_PATT3_Type(loc: 0xa000008c),
  ECCR3: FSMC_ECCR3_Type(loc: 0xa0000094),
  PCR4: FSMC_PCR4_Type(loc: 0xa00000a0),
  SR4: FSMC_SR4_Type(loc: 0xa00000a4),
  PMEM4: FSMC_PMEM4_Type(loc: 0xa00000a8),
  PATT4: FSMC_PATT4_Type(loc: 0xa00000ac),
  PIO4: FSMC_PIO4_Type(loc: 0xa00000b0),
  BWTR1: FSMC_BWTR1_Type(loc: 0xa0000104),
  BWTR2: FSMC_BWTR2_Type(loc: 0xa000010c),
  BWTR3: FSMC_BWTR3_Type(loc: 0xa0000114),
  BWTR4: FSMC_BWTR4_Type(loc: 0xa000011c),
)

const PWR* = PWR_Type(
  CR: PWR_CR_Type(loc: 0x40007000),
  CSR: PWR_CSR_Type(loc: 0x40007004),
)

const RCC* = RCC_Type(
  CR: RCC_CR_Type(loc: 0x40021000),
  CFGR: RCC_CFGR_Type(loc: 0x40021004),
  CIR: RCC_CIR_Type(loc: 0x40021008),
  APB2RSTR: RCC_APB2RSTR_Type(loc: 0x4002100c),
  APB1RSTR: RCC_APB1RSTR_Type(loc: 0x40021010),
  AHBENR: RCC_AHBENR_Type(loc: 0x40021014),
  APB2ENR: RCC_APB2ENR_Type(loc: 0x40021018),
  APB1ENR: RCC_APB1ENR_Type(loc: 0x4002101c),
  BDCR: RCC_BDCR_Type(loc: 0x40021020),
  CSR: RCC_CSR_Type(loc: 0x40021024),
)

const GPIOA* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40010800),
  CRH: GPIOA_CRH_Type(loc: 0x40010804),
  IDR: GPIOA_IDR_Type(loc: 0x40010808),
  ODR: GPIOA_ODR_Type(loc: 0x4001080c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40010810),
  BRR: GPIOA_BRR_Type(loc: 0x40010814),
  LCKR: GPIOA_LCKR_Type(loc: 0x40010818),
)

const GPIOB* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40010c00),
  CRH: GPIOA_CRH_Type(loc: 0x40010c04),
  IDR: GPIOA_IDR_Type(loc: 0x40010c08),
  ODR: GPIOA_ODR_Type(loc: 0x40010c0c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40010c10),
  BRR: GPIOA_BRR_Type(loc: 0x40010c14),
  LCKR: GPIOA_LCKR_Type(loc: 0x40010c18),
)

const GPIOC* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40011000),
  CRH: GPIOA_CRH_Type(loc: 0x40011004),
  IDR: GPIOA_IDR_Type(loc: 0x40011008),
  ODR: GPIOA_ODR_Type(loc: 0x4001100c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40011010),
  BRR: GPIOA_BRR_Type(loc: 0x40011014),
  LCKR: GPIOA_LCKR_Type(loc: 0x40011018),
)

const GPIOD* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40011400),
  CRH: GPIOA_CRH_Type(loc: 0x40011404),
  IDR: GPIOA_IDR_Type(loc: 0x40011408),
  ODR: GPIOA_ODR_Type(loc: 0x4001140c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40011410),
  BRR: GPIOA_BRR_Type(loc: 0x40011414),
  LCKR: GPIOA_LCKR_Type(loc: 0x40011418),
)

const GPIOE* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40011800),
  CRH: GPIOA_CRH_Type(loc: 0x40011804),
  IDR: GPIOA_IDR_Type(loc: 0x40011808),
  ODR: GPIOA_ODR_Type(loc: 0x4001180c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40011810),
  BRR: GPIOA_BRR_Type(loc: 0x40011814),
  LCKR: GPIOA_LCKR_Type(loc: 0x40011818),
)

const GPIOF* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40011c00),
  CRH: GPIOA_CRH_Type(loc: 0x40011c04),
  IDR: GPIOA_IDR_Type(loc: 0x40011c08),
  ODR: GPIOA_ODR_Type(loc: 0x40011c0c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40011c10),
  BRR: GPIOA_BRR_Type(loc: 0x40011c14),
  LCKR: GPIOA_LCKR_Type(loc: 0x40011c18),
)

const GPIOG* = GPIOA_Type(
  CRL: GPIOA_CRL_Type(loc: 0x40012000),
  CRH: GPIOA_CRH_Type(loc: 0x40012004),
  IDR: GPIOA_IDR_Type(loc: 0x40012008),
  ODR: GPIOA_ODR_Type(loc: 0x4001200c),
  BSRR: GPIOA_BSRR_Type(loc: 0x40012010),
  BRR: GPIOA_BRR_Type(loc: 0x40012014),
  LCKR: GPIOA_LCKR_Type(loc: 0x40012018),
)

const AFIO* = AFIO_Type(
  EVCR: AFIO_EVCR_Type(loc: 0x40010000),
  MAPR: AFIO_MAPR_Type(loc: 0x40010004),
  EXTICR1: AFIO_EXTICR1_Type(loc: 0x40010008),
  EXTICR2: AFIO_EXTICR2_Type(loc: 0x4001000c),
  EXTICR3: AFIO_EXTICR3_Type(loc: 0x40010010),
  EXTICR4: AFIO_EXTICR4_Type(loc: 0x40010014),
  MAPR2: AFIO_MAPR2_Type(loc: 0x4001001c),
)

const EXTI* = EXTI_Type(
  IMR: EXTI_IMR_Type(loc: 0x40010400),
  EMR: EXTI_EMR_Type(loc: 0x40010404),
  RTSR: EXTI_RTSR_Type(loc: 0x40010408),
  FTSR: EXTI_FTSR_Type(loc: 0x4001040c),
  SWIER: EXTI_SWIER_Type(loc: 0x40010410),
  PR: EXTI_PR_Type(loc: 0x40010414),
)

const DMA1* = DMA1_Type(
  ISR: DMA1_ISR_Type(loc: 0x40020000),
  IFCR: DMA1_IFCR_Type(loc: 0x40020004),
  CCR1: DMA1_CCR1_Type(loc: 0x40020008),
  CNDTR1: DMA1_CNDTR1_Type(loc: 0x4002000c),
  CPAR1: DMA1_CPAR1_Type(loc: 0x40020010),
  CMAR1: DMA1_CMAR1_Type(loc: 0x40020014),
  CCR2: DMA1_CCR2_Type(loc: 0x4002001c),
  CNDTR2: DMA1_CNDTR2_Type(loc: 0x40020020),
  CPAR2: DMA1_CPAR2_Type(loc: 0x40020024),
  CMAR2: DMA1_CMAR2_Type(loc: 0x40020028),
  CCR3: DMA1_CCR3_Type(loc: 0x40020030),
  CNDTR3: DMA1_CNDTR3_Type(loc: 0x40020034),
  CPAR3: DMA1_CPAR3_Type(loc: 0x40020038),
  CMAR3: DMA1_CMAR3_Type(loc: 0x4002003c),
  CCR4: DMA1_CCR4_Type(loc: 0x40020044),
  CNDTR4: DMA1_CNDTR4_Type(loc: 0x40020048),
  CPAR4: DMA1_CPAR4_Type(loc: 0x4002004c),
  CMAR4: DMA1_CMAR4_Type(loc: 0x40020050),
  CCR5: DMA1_CCR5_Type(loc: 0x40020058),
  CNDTR5: DMA1_CNDTR5_Type(loc: 0x4002005c),
  CPAR5: DMA1_CPAR5_Type(loc: 0x40020060),
  CMAR5: DMA1_CMAR5_Type(loc: 0x40020064),
  CCR6: DMA1_CCR6_Type(loc: 0x4002006c),
  CNDTR6: DMA1_CNDTR6_Type(loc: 0x40020070),
  CPAR6: DMA1_CPAR6_Type(loc: 0x40020074),
  CMAR6: DMA1_CMAR6_Type(loc: 0x40020078),
  CCR7: DMA1_CCR7_Type(loc: 0x40020080),
  CNDTR7: DMA1_CNDTR7_Type(loc: 0x40020084),
  CPAR7: DMA1_CPAR7_Type(loc: 0x40020088),
  CMAR7: DMA1_CMAR7_Type(loc: 0x4002008c),
)

const DMA2* = DMA1_Type(
  ISR: DMA1_ISR_Type(loc: 0x40020400),
  IFCR: DMA1_IFCR_Type(loc: 0x40020404),
  CCR1: DMA1_CCR1_Type(loc: 0x40020408),
  CNDTR1: DMA1_CNDTR1_Type(loc: 0x4002040c),
  CPAR1: DMA1_CPAR1_Type(loc: 0x40020410),
  CMAR1: DMA1_CMAR1_Type(loc: 0x40020414),
  CCR2: DMA1_CCR2_Type(loc: 0x4002041c),
  CNDTR2: DMA1_CNDTR2_Type(loc: 0x40020420),
  CPAR2: DMA1_CPAR2_Type(loc: 0x40020424),
  CMAR2: DMA1_CMAR2_Type(loc: 0x40020428),
  CCR3: DMA1_CCR3_Type(loc: 0x40020430),
  CNDTR3: DMA1_CNDTR3_Type(loc: 0x40020434),
  CPAR3: DMA1_CPAR3_Type(loc: 0x40020438),
  CMAR3: DMA1_CMAR3_Type(loc: 0x4002043c),
  CCR4: DMA1_CCR4_Type(loc: 0x40020444),
  CNDTR4: DMA1_CNDTR4_Type(loc: 0x40020448),
  CPAR4: DMA1_CPAR4_Type(loc: 0x4002044c),
  CMAR4: DMA1_CMAR4_Type(loc: 0x40020450),
  CCR5: DMA1_CCR5_Type(loc: 0x40020458),
  CNDTR5: DMA1_CNDTR5_Type(loc: 0x4002045c),
  CPAR5: DMA1_CPAR5_Type(loc: 0x40020460),
  CMAR5: DMA1_CMAR5_Type(loc: 0x40020464),
  CCR6: DMA1_CCR6_Type(loc: 0x4002046c),
  CNDTR6: DMA1_CNDTR6_Type(loc: 0x40020470),
  CPAR6: DMA1_CPAR6_Type(loc: 0x40020474),
  CMAR6: DMA1_CMAR6_Type(loc: 0x40020478),
  CCR7: DMA1_CCR7_Type(loc: 0x40020480),
  CNDTR7: DMA1_CNDTR7_Type(loc: 0x40020484),
  CPAR7: DMA1_CPAR7_Type(loc: 0x40020488),
  CMAR7: DMA1_CMAR7_Type(loc: 0x4002048c),
)

const SDIO* = SDIO_Type(
  POWER: SDIO_POWER_Type(loc: 0x40018000),
  CLKCR: SDIO_CLKCR_Type(loc: 0x40018004),
  ARG: SDIO_ARG_Type(loc: 0x40018008),
  CMD: SDIO_CMD_Type(loc: 0x4001800c),
  RESPCMD: SDIO_RESPCMD_Type(loc: 0x40018010),
  RESPI1: SDIO_RESPI1_Type(loc: 0x40018014),
  RESP2: SDIO_RESP2_Type(loc: 0x40018018),
  RESP3: SDIO_RESP3_Type(loc: 0x4001801c),
  RESP4: SDIO_RESP4_Type(loc: 0x40018020),
  DTIMER: SDIO_DTIMER_Type(loc: 0x40018024),
  DLEN: SDIO_DLEN_Type(loc: 0x40018028),
  DCTRL: SDIO_DCTRL_Type(loc: 0x4001802c),
  DCOUNT: SDIO_DCOUNT_Type(loc: 0x40018030),
  STA: SDIO_STA_Type(loc: 0x40018034),
  ICR: SDIO_ICR_Type(loc: 0x40018038),
  MASK: SDIO_MASK_Type(loc: 0x4001803c),
  FIFOCNT: SDIO_FIFOCNT_Type(loc: 0x40018048),
  FIFO: SDIO_FIFO_Type(loc: 0x40018080),
)

const RTC* = RTC_Type(
  CRH: RTC_CRH_Type(loc: 0x40002800),
  CRL: RTC_CRL_Type(loc: 0x40002804),
  PRLH: RTC_PRLH_Type(loc: 0x40002808),
  PRLL: RTC_PRLL_Type(loc: 0x4000280c),
  DIVH: RTC_DIVH_Type(loc: 0x40002810),
  DIVL: RTC_DIVL_Type(loc: 0x40002814),
  CNTH: RTC_CNTH_Type(loc: 0x40002818),
  CNTL: RTC_CNTL_Type(loc: 0x4000281c),
  ALRH: RTC_ALRH_Type(loc: 0x40002820),
  ALRL: RTC_ALRL_Type(loc: 0x40002824),
)

const BKP* = BKP_Type(
  DR1: BKP_DR1_Type(loc: 0x40006c00),
  DR2: BKP_DR2_Type(loc: 0x40006c04),
  DR3: BKP_DR3_Type(loc: 0x40006c08),
  DR4: BKP_DR4_Type(loc: 0x40006c0c),
  DR5: BKP_DR5_Type(loc: 0x40006c10),
  DR6: BKP_DR6_Type(loc: 0x40006c14),
  DR7: BKP_DR7_Type(loc: 0x40006c18),
  DR8: BKP_DR8_Type(loc: 0x40006c1c),
  DR9: BKP_DR9_Type(loc: 0x40006c20),
  DR10: BKP_DR10_Type(loc: 0x40006c24),
  RTCCR: BKP_RTCCR_Type(loc: 0x40006c28),
  CR: BKP_CR_Type(loc: 0x40006c2c),
  CSR: BKP_CSR_Type(loc: 0x40006c30),
  DR11: BKP_DR11_Type(loc: 0x40006c3c),
  DR12: BKP_DR12_Type(loc: 0x40006c40),
  DR13: BKP_DR13_Type(loc: 0x40006c44),
  DR14: BKP_DR14_Type(loc: 0x40006c48),
  DR15: BKP_DR15_Type(loc: 0x40006c4c),
  DR16: BKP_DR16_Type(loc: 0x40006c50),
  DR17: BKP_DR17_Type(loc: 0x40006c54),
  DR18: BKP_DR18_Type(loc: 0x40006c58),
  DR19: BKP_DR19_Type(loc: 0x40006c5c),
  DR20: BKP_DR20_Type(loc: 0x40006c60),
  DR21: BKP_DR21_Type(loc: 0x40006c64),
  DR22: BKP_DR22_Type(loc: 0x40006c68),
  DR23: BKP_DR23_Type(loc: 0x40006c6c),
  DR24: BKP_DR24_Type(loc: 0x40006c70),
  DR25: BKP_DR25_Type(loc: 0x40006c74),
  DR26: BKP_DR26_Type(loc: 0x40006c78),
  DR27: BKP_DR27_Type(loc: 0x40006c7c),
  DR28: BKP_DR28_Type(loc: 0x40006c80),
  DR29: BKP_DR29_Type(loc: 0x40006c84),
  DR30: BKP_DR30_Type(loc: 0x40006c88),
  DR31: BKP_DR31_Type(loc: 0x40006c8c),
  DR32: BKP_DR32_Type(loc: 0x40006c90),
  DR33: BKP_DR33_Type(loc: 0x40006c94),
  DR34: BKP_DR34_Type(loc: 0x40006c98),
  DR35: BKP_DR35_Type(loc: 0x40006c9c),
  DR36: BKP_DR36_Type(loc: 0x40006ca0),
  DR37: BKP_DR37_Type(loc: 0x40006ca4),
  DR38: BKP_DR38_Type(loc: 0x40006ca8),
  DR39: BKP_DR39_Type(loc: 0x40006cac),
  DR40: BKP_DR40_Type(loc: 0x40006cb0),
  DR41: BKP_DR41_Type(loc: 0x40006cb4),
  DR42: BKP_DR42_Type(loc: 0x40006cb8),
)

const IWDG* = IWDG_Type(
  KR: IWDG_KR_Type(loc: 0x40003000),
  PR: IWDG_PR_Type(loc: 0x40003004),
  RLR: IWDG_RLR_Type(loc: 0x40003008),
  SR: IWDG_SR_Type(loc: 0x4000300c),
)

const WWDG* = WWDG_Type(
  CR: WWDG_CR_Type(loc: 0x40002c00),
  CFR: WWDG_CFR_Type(loc: 0x40002c04),
  SR: WWDG_SR_Type(loc: 0x40002c08),
)

const TIM1* = TIM1_Type(
  CR1: TIM1_CR1_Type(loc: 0x40012c00),
  CR2: TIM1_CR2_Type(loc: 0x40012c04),
  SMCR: TIM1_SMCR_Type(loc: 0x40012c08),
  DIER: TIM1_DIER_Type(loc: 0x40012c0c),
  SR: TIM1_SR_Type(loc: 0x40012c10),
  EGR: TIM1_EGR_Type(loc: 0x40012c14),
  CCMR1_Output: TIM1_CCMR1_Output_Type(loc: 0x40012c18),
  CCMR1_Input: TIM1_CCMR1_Input_Type(loc: 0x40012c18),
  CCMR2_Output: TIM1_CCMR2_Output_Type(loc: 0x40012c1c),
  CCMR2_Input: TIM1_CCMR2_Input_Type(loc: 0x40012c1c),
  CCER: TIM1_CCER_Type(loc: 0x40012c20),
  CNT: TIM1_CNT_Type(loc: 0x40012c24),
  PSC: TIM1_PSC_Type(loc: 0x40012c28),
  ARR: TIM1_ARR_Type(loc: 0x40012c2c),
  RCR: TIM1_RCR_Type(loc: 0x40012c30),
  CCR1: TIM1_CCR1_Type(loc: 0x40012c34),
  CCR2: TIM1_CCR2_Type(loc: 0x40012c38),
  CCR3: TIM1_CCR3_Type(loc: 0x40012c3c),
  CCR4: TIM1_CCR4_Type(loc: 0x40012c40),
  BDTR: TIM1_BDTR_Type(loc: 0x40012c44),
  DCR: TIM1_DCR_Type(loc: 0x40012c48),
  DMAR: TIM1_DMAR_Type(loc: 0x40012c4c),
)

const TIM8* = TIM1_Type(
  CR1: TIM1_CR1_Type(loc: 0x40013400),
  CR2: TIM1_CR2_Type(loc: 0x40013404),
  SMCR: TIM1_SMCR_Type(loc: 0x40013408),
  DIER: TIM1_DIER_Type(loc: 0x4001340c),
  SR: TIM1_SR_Type(loc: 0x40013410),
  EGR: TIM1_EGR_Type(loc: 0x40013414),
  CCMR1_Output: TIM1_CCMR1_Output_Type(loc: 0x40013418),
  CCMR1_Input: TIM1_CCMR1_Input_Type(loc: 0x40013418),
  CCMR2_Output: TIM1_CCMR2_Output_Type(loc: 0x4001341c),
  CCMR2_Input: TIM1_CCMR2_Input_Type(loc: 0x4001341c),
  CCER: TIM1_CCER_Type(loc: 0x40013420),
  CNT: TIM1_CNT_Type(loc: 0x40013424),
  PSC: TIM1_PSC_Type(loc: 0x40013428),
  ARR: TIM1_ARR_Type(loc: 0x4001342c),
  RCR: TIM1_RCR_Type(loc: 0x40013430),
  CCR1: TIM1_CCR1_Type(loc: 0x40013434),
  CCR2: TIM1_CCR2_Type(loc: 0x40013438),
  CCR3: TIM1_CCR3_Type(loc: 0x4001343c),
  CCR4: TIM1_CCR4_Type(loc: 0x40013440),
  BDTR: TIM1_BDTR_Type(loc: 0x40013444),
  DCR: TIM1_DCR_Type(loc: 0x40013448),
  DMAR: TIM1_DMAR_Type(loc: 0x4001344c),
)

const TIM2* = TIM2_Type(
  CR1: TIM2_CR1_Type(loc: 0x40000000),
  CR2: TIM2_CR2_Type(loc: 0x40000004),
  SMCR: TIM2_SMCR_Type(loc: 0x40000008),
  DIER: TIM2_DIER_Type(loc: 0x4000000c),
  SR: TIM2_SR_Type(loc: 0x40000010),
  EGR: TIM2_EGR_Type(loc: 0x40000014),
  CCMR1_Output: TIM2_CCMR1_Output_Type(loc: 0x40000018),
  CCMR1_Input: TIM2_CCMR1_Input_Type(loc: 0x40000018),
  CCMR2_Output: TIM2_CCMR2_Output_Type(loc: 0x4000001c),
  CCMR2_Input: TIM2_CCMR2_Input_Type(loc: 0x4000001c),
  CCER: TIM2_CCER_Type(loc: 0x40000020),
  CNT: TIM2_CNT_Type(loc: 0x40000024),
  PSC: TIM2_PSC_Type(loc: 0x40000028),
  ARR: TIM2_ARR_Type(loc: 0x4000002c),
  CCR1: TIM2_CCR1_Type(loc: 0x40000034),
  CCR2: TIM2_CCR2_Type(loc: 0x40000038),
  CCR3: TIM2_CCR3_Type(loc: 0x4000003c),
  CCR4: TIM2_CCR4_Type(loc: 0x40000040),
  DCR: TIM2_DCR_Type(loc: 0x40000048),
  DMAR: TIM2_DMAR_Type(loc: 0x4000004c),
)

const TIM3* = TIM2_Type(
  CR1: TIM2_CR1_Type(loc: 0x40000400),
  CR2: TIM2_CR2_Type(loc: 0x40000404),
  SMCR: TIM2_SMCR_Type(loc: 0x40000408),
  DIER: TIM2_DIER_Type(loc: 0x4000040c),
  SR: TIM2_SR_Type(loc: 0x40000410),
  EGR: TIM2_EGR_Type(loc: 0x40000414),
  CCMR1_Output: TIM2_CCMR1_Output_Type(loc: 0x40000418),
  CCMR1_Input: TIM2_CCMR1_Input_Type(loc: 0x40000418),
  CCMR2_Output: TIM2_CCMR2_Output_Type(loc: 0x4000041c),
  CCMR2_Input: TIM2_CCMR2_Input_Type(loc: 0x4000041c),
  CCER: TIM2_CCER_Type(loc: 0x40000420),
  CNT: TIM2_CNT_Type(loc: 0x40000424),
  PSC: TIM2_PSC_Type(loc: 0x40000428),
  ARR: TIM2_ARR_Type(loc: 0x4000042c),
  CCR1: TIM2_CCR1_Type(loc: 0x40000434),
  CCR2: TIM2_CCR2_Type(loc: 0x40000438),
  CCR3: TIM2_CCR3_Type(loc: 0x4000043c),
  CCR4: TIM2_CCR4_Type(loc: 0x40000440),
  DCR: TIM2_DCR_Type(loc: 0x40000448),
  DMAR: TIM2_DMAR_Type(loc: 0x4000044c),
)

const TIM4* = TIM2_Type(
  CR1: TIM2_CR1_Type(loc: 0x40000800),
  CR2: TIM2_CR2_Type(loc: 0x40000804),
  SMCR: TIM2_SMCR_Type(loc: 0x40000808),
  DIER: TIM2_DIER_Type(loc: 0x4000080c),
  SR: TIM2_SR_Type(loc: 0x40000810),
  EGR: TIM2_EGR_Type(loc: 0x40000814),
  CCMR1_Output: TIM2_CCMR1_Output_Type(loc: 0x40000818),
  CCMR1_Input: TIM2_CCMR1_Input_Type(loc: 0x40000818),
  CCMR2_Output: TIM2_CCMR2_Output_Type(loc: 0x4000081c),
  CCMR2_Input: TIM2_CCMR2_Input_Type(loc: 0x4000081c),
  CCER: TIM2_CCER_Type(loc: 0x40000820),
  CNT: TIM2_CNT_Type(loc: 0x40000824),
  PSC: TIM2_PSC_Type(loc: 0x40000828),
  ARR: TIM2_ARR_Type(loc: 0x4000082c),
  CCR1: TIM2_CCR1_Type(loc: 0x40000834),
  CCR2: TIM2_CCR2_Type(loc: 0x40000838),
  CCR3: TIM2_CCR3_Type(loc: 0x4000083c),
  CCR4: TIM2_CCR4_Type(loc: 0x40000840),
  DCR: TIM2_DCR_Type(loc: 0x40000848),
  DMAR: TIM2_DMAR_Type(loc: 0x4000084c),
)

const TIM5* = TIM2_Type(
  CR1: TIM2_CR1_Type(loc: 0x40000c00),
  CR2: TIM2_CR2_Type(loc: 0x40000c04),
  SMCR: TIM2_SMCR_Type(loc: 0x40000c08),
  DIER: TIM2_DIER_Type(loc: 0x40000c0c),
  SR: TIM2_SR_Type(loc: 0x40000c10),
  EGR: TIM2_EGR_Type(loc: 0x40000c14),
  CCMR1_Output: TIM2_CCMR1_Output_Type(loc: 0x40000c18),
  CCMR1_Input: TIM2_CCMR1_Input_Type(loc: 0x40000c18),
  CCMR2_Output: TIM2_CCMR2_Output_Type(loc: 0x40000c1c),
  CCMR2_Input: TIM2_CCMR2_Input_Type(loc: 0x40000c1c),
  CCER: TIM2_CCER_Type(loc: 0x40000c20),
  CNT: TIM2_CNT_Type(loc: 0x40000c24),
  PSC: TIM2_PSC_Type(loc: 0x40000c28),
  ARR: TIM2_ARR_Type(loc: 0x40000c2c),
  CCR1: TIM2_CCR1_Type(loc: 0x40000c34),
  CCR2: TIM2_CCR2_Type(loc: 0x40000c38),
  CCR3: TIM2_CCR3_Type(loc: 0x40000c3c),
  CCR4: TIM2_CCR4_Type(loc: 0x40000c40),
  DCR: TIM2_DCR_Type(loc: 0x40000c48),
  DMAR: TIM2_DMAR_Type(loc: 0x40000c4c),
)

const TIM9* = TIM9_Type(
  CR1: TIM9_CR1_Type(loc: 0x40014c00),
  CR2: TIM9_CR2_Type(loc: 0x40014c04),
  SMCR: TIM9_SMCR_Type(loc: 0x40014c08),
  DIER: TIM9_DIER_Type(loc: 0x40014c0c),
  SR: TIM9_SR_Type(loc: 0x40014c10),
  EGR: TIM9_EGR_Type(loc: 0x40014c14),
  CCMR1_Output: TIM9_CCMR1_Output_Type(loc: 0x40014c18),
  CCMR1_Input: TIM9_CCMR1_Input_Type(loc: 0x40014c18),
  CCER: TIM9_CCER_Type(loc: 0x40014c20),
  CNT: TIM9_CNT_Type(loc: 0x40014c24),
  PSC: TIM9_PSC_Type(loc: 0x40014c28),
  ARR: TIM9_ARR_Type(loc: 0x40014c2c),
  CCR1: TIM9_CCR1_Type(loc: 0x40014c34),
  CCR2: TIM9_CCR2_Type(loc: 0x40014c38),
)

const TIM12* = TIM9_Type(
  CR1: TIM9_CR1_Type(loc: 0x40001800),
  CR2: TIM9_CR2_Type(loc: 0x40001804),
  SMCR: TIM9_SMCR_Type(loc: 0x40001808),
  DIER: TIM9_DIER_Type(loc: 0x4000180c),
  SR: TIM9_SR_Type(loc: 0x40001810),
  EGR: TIM9_EGR_Type(loc: 0x40001814),
  CCMR1_Output: TIM9_CCMR1_Output_Type(loc: 0x40001818),
  CCMR1_Input: TIM9_CCMR1_Input_Type(loc: 0x40001818),
  CCER: TIM9_CCER_Type(loc: 0x40001820),
  CNT: TIM9_CNT_Type(loc: 0x40001824),
  PSC: TIM9_PSC_Type(loc: 0x40001828),
  ARR: TIM9_ARR_Type(loc: 0x4000182c),
  CCR1: TIM9_CCR1_Type(loc: 0x40001834),
  CCR2: TIM9_CCR2_Type(loc: 0x40001838),
)

const TIM10* = TIM10_Type(
  CR1: TIM10_CR1_Type(loc: 0x40015000),
  CR2: TIM10_CR2_Type(loc: 0x40015004),
  DIER: TIM10_DIER_Type(loc: 0x4001500c),
  SR: TIM10_SR_Type(loc: 0x40015010),
  EGR: TIM10_EGR_Type(loc: 0x40015014),
  CCMR1_Output: TIM10_CCMR1_Output_Type(loc: 0x40015018),
  CCMR1_Input: TIM10_CCMR1_Input_Type(loc: 0x40015018),
  CCER: TIM10_CCER_Type(loc: 0x40015020),
  CNT: TIM10_CNT_Type(loc: 0x40015024),
  PSC: TIM10_PSC_Type(loc: 0x40015028),
  ARR: TIM10_ARR_Type(loc: 0x4001502c),
  CCR1: TIM10_CCR1_Type(loc: 0x40015034),
)

const TIM11* = TIM10_Type(
  CR1: TIM10_CR1_Type(loc: 0x40015400),
  CR2: TIM10_CR2_Type(loc: 0x40015404),
  DIER: TIM10_DIER_Type(loc: 0x4001540c),
  SR: TIM10_SR_Type(loc: 0x40015410),
  EGR: TIM10_EGR_Type(loc: 0x40015414),
  CCMR1_Output: TIM10_CCMR1_Output_Type(loc: 0x40015418),
  CCMR1_Input: TIM10_CCMR1_Input_Type(loc: 0x40015418),
  CCER: TIM10_CCER_Type(loc: 0x40015420),
  CNT: TIM10_CNT_Type(loc: 0x40015424),
  PSC: TIM10_PSC_Type(loc: 0x40015428),
  ARR: TIM10_ARR_Type(loc: 0x4001542c),
  CCR1: TIM10_CCR1_Type(loc: 0x40015434),
)

const TIM13* = TIM10_Type(
  CR1: TIM10_CR1_Type(loc: 0x40001c00),
  CR2: TIM10_CR2_Type(loc: 0x40001c04),
  DIER: TIM10_DIER_Type(loc: 0x40001c0c),
  SR: TIM10_SR_Type(loc: 0x40001c10),
  EGR: TIM10_EGR_Type(loc: 0x40001c14),
  CCMR1_Output: TIM10_CCMR1_Output_Type(loc: 0x40001c18),
  CCMR1_Input: TIM10_CCMR1_Input_Type(loc: 0x40001c18),
  CCER: TIM10_CCER_Type(loc: 0x40001c20),
  CNT: TIM10_CNT_Type(loc: 0x40001c24),
  PSC: TIM10_PSC_Type(loc: 0x40001c28),
  ARR: TIM10_ARR_Type(loc: 0x40001c2c),
  CCR1: TIM10_CCR1_Type(loc: 0x40001c34),
)

const TIM14* = TIM10_Type(
  CR1: TIM10_CR1_Type(loc: 0x40002000),
  CR2: TIM10_CR2_Type(loc: 0x40002004),
  DIER: TIM10_DIER_Type(loc: 0x4000200c),
  SR: TIM10_SR_Type(loc: 0x40002010),
  EGR: TIM10_EGR_Type(loc: 0x40002014),
  CCMR1_Output: TIM10_CCMR1_Output_Type(loc: 0x40002018),
  CCMR1_Input: TIM10_CCMR1_Input_Type(loc: 0x40002018),
  CCER: TIM10_CCER_Type(loc: 0x40002020),
  CNT: TIM10_CNT_Type(loc: 0x40002024),
  PSC: TIM10_PSC_Type(loc: 0x40002028),
  ARR: TIM10_ARR_Type(loc: 0x4000202c),
  CCR1: TIM10_CCR1_Type(loc: 0x40002034),
)

const TIM6* = TIM6_Type(
  CR1: TIM6_CR1_Type(loc: 0x40001000),
  CR2: TIM6_CR2_Type(loc: 0x40001004),
  DIER: TIM6_DIER_Type(loc: 0x4000100c),
  SR: TIM6_SR_Type(loc: 0x40001010),
  EGR: TIM6_EGR_Type(loc: 0x40001014),
  CNT: TIM6_CNT_Type(loc: 0x40001024),
  PSC: TIM6_PSC_Type(loc: 0x40001028),
  ARR: TIM6_ARR_Type(loc: 0x4000102c),
)

const TIM7* = TIM6_Type(
  CR1: TIM6_CR1_Type(loc: 0x40001400),
  CR2: TIM6_CR2_Type(loc: 0x40001404),
  DIER: TIM6_DIER_Type(loc: 0x4000140c),
  SR: TIM6_SR_Type(loc: 0x40001410),
  EGR: TIM6_EGR_Type(loc: 0x40001414),
  CNT: TIM6_CNT_Type(loc: 0x40001424),
  PSC: TIM6_PSC_Type(loc: 0x40001428),
  ARR: TIM6_ARR_Type(loc: 0x4000142c),
)

const I2C1* = I2C1_Type(
  CR1: I2C1_CR1_Type(loc: 0x40005400),
  CR2: I2C1_CR2_Type(loc: 0x40005404),
  OAR1: I2C1_OAR1_Type(loc: 0x40005408),
  OAR2: I2C1_OAR2_Type(loc: 0x4000540c),
  DR: I2C1_DR_Type(loc: 0x40005410),
  SR1: I2C1_SR1_Type(loc: 0x40005414),
  SR2: I2C1_SR2_Type(loc: 0x40005418),
  CCR: I2C1_CCR_Type(loc: 0x4000541c),
  TRISE: I2C1_TRISE_Type(loc: 0x40005420),
)

const I2C2* = I2C1_Type(
  CR1: I2C1_CR1_Type(loc: 0x40005800),
  CR2: I2C1_CR2_Type(loc: 0x40005804),
  OAR1: I2C1_OAR1_Type(loc: 0x40005808),
  OAR2: I2C1_OAR2_Type(loc: 0x4000580c),
  DR: I2C1_DR_Type(loc: 0x40005810),
  SR1: I2C1_SR1_Type(loc: 0x40005814),
  SR2: I2C1_SR2_Type(loc: 0x40005818),
  CCR: I2C1_CCR_Type(loc: 0x4000581c),
  TRISE: I2C1_TRISE_Type(loc: 0x40005820),
)

const SPI1* = SPI1_Type(
  CR1: SPI1_CR1_Type(loc: 0x40013000),
  CR2: SPI1_CR2_Type(loc: 0x40013004),
  SR: SPI1_SR_Type(loc: 0x40013008),
  DR: SPI1_DR_Type(loc: 0x4001300c),
  CRCPR: SPI1_CRCPR_Type(loc: 0x40013010),
  RXCRCR: SPI1_RXCRCR_Type(loc: 0x40013014),
  TXCRCR: SPI1_TXCRCR_Type(loc: 0x40013018),
  I2SCFGR: SPI1_I2SCFGR_Type(loc: 0x4001301c),
  I2SPR: SPI1_I2SPR_Type(loc: 0x40013020),
)

const SPI2* = SPI1_Type(
  CR1: SPI1_CR1_Type(loc: 0x40003800),
  CR2: SPI1_CR2_Type(loc: 0x40003804),
  SR: SPI1_SR_Type(loc: 0x40003808),
  DR: SPI1_DR_Type(loc: 0x4000380c),
  CRCPR: SPI1_CRCPR_Type(loc: 0x40003810),
  RXCRCR: SPI1_RXCRCR_Type(loc: 0x40003814),
  TXCRCR: SPI1_TXCRCR_Type(loc: 0x40003818),
  I2SCFGR: SPI1_I2SCFGR_Type(loc: 0x4000381c),
  I2SPR: SPI1_I2SPR_Type(loc: 0x40003820),
)

const SPI3* = SPI1_Type(
  CR1: SPI1_CR1_Type(loc: 0x40003c00),
  CR2: SPI1_CR2_Type(loc: 0x40003c04),
  SR: SPI1_SR_Type(loc: 0x40003c08),
  DR: SPI1_DR_Type(loc: 0x40003c0c),
  CRCPR: SPI1_CRCPR_Type(loc: 0x40003c10),
  RXCRCR: SPI1_RXCRCR_Type(loc: 0x40003c14),
  TXCRCR: SPI1_TXCRCR_Type(loc: 0x40003c18),
  I2SCFGR: SPI1_I2SCFGR_Type(loc: 0x40003c1c),
  I2SPR: SPI1_I2SPR_Type(loc: 0x40003c20),
)

const USART1* = USART1_Type(
  SR: USART1_SR_Type(loc: 0x40013800),
  DR: USART1_DR_Type(loc: 0x40013804),
  BRR: USART1_BRR_Type(loc: 0x40013808),
  CR1: USART1_CR1_Type(loc: 0x4001380c),
  CR2: USART1_CR2_Type(loc: 0x40013810),
  CR3: USART1_CR3_Type(loc: 0x40013814),
  GTPR: USART1_GTPR_Type(loc: 0x40013818),
)

const USART2* = USART1_Type(
  SR: USART1_SR_Type(loc: 0x40004400),
  DR: USART1_DR_Type(loc: 0x40004404),
  BRR: USART1_BRR_Type(loc: 0x40004408),
  CR1: USART1_CR1_Type(loc: 0x4000440c),
  CR2: USART1_CR2_Type(loc: 0x40004410),
  CR3: USART1_CR3_Type(loc: 0x40004414),
  GTPR: USART1_GTPR_Type(loc: 0x40004418),
)

const USART3* = USART1_Type(
  SR: USART1_SR_Type(loc: 0x40004800),
  DR: USART1_DR_Type(loc: 0x40004804),
  BRR: USART1_BRR_Type(loc: 0x40004808),
  CR1: USART1_CR1_Type(loc: 0x4000480c),
  CR2: USART1_CR2_Type(loc: 0x40004810),
  CR3: USART1_CR3_Type(loc: 0x40004814),
  GTPR: USART1_GTPR_Type(loc: 0x40004818),
)

const ADC1* = ADC1_Type(
  SR: ADC1_SR_Type(loc: 0x40012400),
  CR1: ADC1_CR1_Type(loc: 0x40012404),
  CR2: ADC1_CR2_Type(loc: 0x40012408),
  SMPR1: ADC1_SMPR1_Type(loc: 0x4001240c),
  SMPR2: ADC1_SMPR2_Type(loc: 0x40012410),
  JOFR1: ADC1_JOFR1_Type(loc: 0x40012414),
  JOFR2: ADC1_JOFR2_Type(loc: 0x40012418),
  JOFR3: ADC1_JOFR3_Type(loc: 0x4001241c),
  JOFR4: ADC1_JOFR4_Type(loc: 0x40012420),
  HTR: ADC1_HTR_Type(loc: 0x40012424),
  LTR: ADC1_LTR_Type(loc: 0x40012428),
  SQR1: ADC1_SQR1_Type(loc: 0x4001242c),
  SQR2: ADC1_SQR2_Type(loc: 0x40012430),
  SQR3: ADC1_SQR3_Type(loc: 0x40012434),
  JSQR: ADC1_JSQR_Type(loc: 0x40012438),
  JDR1: ADC1_JDR1_Type(loc: 0x4001243c),
  JDR2: ADC1_JDR2_Type(loc: 0x40012440),
  JDR3: ADC1_JDR3_Type(loc: 0x40012444),
  JDR4: ADC1_JDR4_Type(loc: 0x40012448),
  DR: ADC1_DR_Type(loc: 0x4001244c),
)

const ADC2* = ADC2_Type(
  SR: ADC2_SR_Type(loc: 0x40012800),
  CR1: ADC2_CR1_Type(loc: 0x40012804),
  CR2: ADC2_CR2_Type(loc: 0x40012808),
  SMPR1: ADC2_SMPR1_Type(loc: 0x4001280c),
  SMPR2: ADC2_SMPR2_Type(loc: 0x40012810),
  JOFR1: ADC2_JOFR1_Type(loc: 0x40012814),
  JOFR2: ADC2_JOFR2_Type(loc: 0x40012818),
  JOFR3: ADC2_JOFR3_Type(loc: 0x4001281c),
  JOFR4: ADC2_JOFR4_Type(loc: 0x40012820),
  HTR: ADC2_HTR_Type(loc: 0x40012824),
  LTR: ADC2_LTR_Type(loc: 0x40012828),
  SQR1: ADC2_SQR1_Type(loc: 0x4001282c),
  SQR2: ADC2_SQR2_Type(loc: 0x40012830),
  SQR3: ADC2_SQR3_Type(loc: 0x40012834),
  JSQR: ADC2_JSQR_Type(loc: 0x40012838),
  JDR1: ADC2_JDR1_Type(loc: 0x4001283c),
  JDR2: ADC2_JDR2_Type(loc: 0x40012840),
  JDR3: ADC2_JDR3_Type(loc: 0x40012844),
  JDR4: ADC2_JDR4_Type(loc: 0x40012848),
  DR: ADC2_DR_Type(loc: 0x4001284c),
)

const ADC3* = ADC2_Type(
  SR: ADC2_SR_Type(loc: 0x40013c00),
  CR1: ADC2_CR1_Type(loc: 0x40013c04),
  CR2: ADC2_CR2_Type(loc: 0x40013c08),
  SMPR1: ADC2_SMPR1_Type(loc: 0x40013c0c),
  SMPR2: ADC2_SMPR2_Type(loc: 0x40013c10),
  JOFR1: ADC2_JOFR1_Type(loc: 0x40013c14),
  JOFR2: ADC2_JOFR2_Type(loc: 0x40013c18),
  JOFR3: ADC2_JOFR3_Type(loc: 0x40013c1c),
  JOFR4: ADC2_JOFR4_Type(loc: 0x40013c20),
  HTR: ADC2_HTR_Type(loc: 0x40013c24),
  LTR: ADC2_LTR_Type(loc: 0x40013c28),
  SQR1: ADC2_SQR1_Type(loc: 0x40013c2c),
  SQR2: ADC2_SQR2_Type(loc: 0x40013c30),
  SQR3: ADC2_SQR3_Type(loc: 0x40013c34),
  JSQR: ADC2_JSQR_Type(loc: 0x40013c38),
  JDR1: ADC2_JDR1_Type(loc: 0x40013c3c),
  JDR2: ADC2_JDR2_Type(loc: 0x40013c40),
  JDR3: ADC2_JDR3_Type(loc: 0x40013c44),
  JDR4: ADC2_JDR4_Type(loc: 0x40013c48),
  DR: ADC2_DR_Type(loc: 0x40013c4c),
)

const CAN1* = CAN1_Type(
  CAN_MCR: CAN1_CAN_MCR_Type(loc: 0x40006400),
  CAN_MSR: CAN1_CAN_MSR_Type(loc: 0x40006404),
  CAN_TSR: CAN1_CAN_TSR_Type(loc: 0x40006408),
  CAN_RF0R: CAN1_CAN_RF0R_Type(loc: 0x4000640c),
  CAN_RF1R: CAN1_CAN_RF1R_Type(loc: 0x40006410),
  CAN_IER: CAN1_CAN_IER_Type(loc: 0x40006414),
  CAN_ESR: CAN1_CAN_ESR_Type(loc: 0x40006418),
  CAN_BTR: CAN1_CAN_BTR_Type(loc: 0x4000641c),
  CAN_TI0R: CAN1_CAN_TI0R_Type(loc: 0x40006580),
  CAN_TDT0R: CAN1_CAN_TDT0R_Type(loc: 0x40006584),
  CAN_TDL0R: CAN1_CAN_TDL0R_Type(loc: 0x40006588),
  CAN_TDH0R: CAN1_CAN_TDH0R_Type(loc: 0x4000658c),
  CAN_TI1R: CAN1_CAN_TI1R_Type(loc: 0x40006590),
  CAN_TDT1R: CAN1_CAN_TDT1R_Type(loc: 0x40006594),
  CAN_TDL1R: CAN1_CAN_TDL1R_Type(loc: 0x40006598),
  CAN_TDH1R: CAN1_CAN_TDH1R_Type(loc: 0x4000659c),
  CAN_TI2R: CAN1_CAN_TI2R_Type(loc: 0x400065a0),
  CAN_TDT2R: CAN1_CAN_TDT2R_Type(loc: 0x400065a4),
  CAN_TDL2R: CAN1_CAN_TDL2R_Type(loc: 0x400065a8),
  CAN_TDH2R: CAN1_CAN_TDH2R_Type(loc: 0x400065ac),
  CAN_RI0R: CAN1_CAN_RI0R_Type(loc: 0x400065b0),
  CAN_RDT0R: CAN1_CAN_RDT0R_Type(loc: 0x400065b4),
  CAN_RDL0R: CAN1_CAN_RDL0R_Type(loc: 0x400065b8),
  CAN_RDH0R: CAN1_CAN_RDH0R_Type(loc: 0x400065bc),
  CAN_RI1R: CAN1_CAN_RI1R_Type(loc: 0x400065c0),
  CAN_RDT1R: CAN1_CAN_RDT1R_Type(loc: 0x400065c4),
  CAN_RDL1R: CAN1_CAN_RDL1R_Type(loc: 0x400065c8),
  CAN_RDH1R: CAN1_CAN_RDH1R_Type(loc: 0x400065cc),
  CAN_FMR: CAN1_CAN_FMR_Type(loc: 0x40006600),
  CAN_FM1R: CAN1_CAN_FM1R_Type(loc: 0x40006604),
  CAN_FS1R: CAN1_CAN_FS1R_Type(loc: 0x4000660c),
  CAN_FFA1R: CAN1_CAN_FFA1R_Type(loc: 0x40006614),
  CAN_FA1R: CAN1_CAN_FA1R_Type(loc: 0x4000661c),
  F0R1: CAN1_F0R1_Type(loc: 0x40006640),
  F0R2: CAN1_F0R2_Type(loc: 0x40006644),
  F1R1: CAN1_F1R1_Type(loc: 0x40006648),
  F1R2: CAN1_F1R2_Type(loc: 0x4000664c),
  F2R1: CAN1_F2R1_Type(loc: 0x40006650),
  F2R2: CAN1_F2R2_Type(loc: 0x40006654),
  F3R1: CAN1_F3R1_Type(loc: 0x40006658),
  F3R2: CAN1_F3R2_Type(loc: 0x4000665c),
  F4R1: CAN1_F4R1_Type(loc: 0x40006660),
  F4R2: CAN1_F4R2_Type(loc: 0x40006664),
  F5R1: CAN1_F5R1_Type(loc: 0x40006668),
  F5R2: CAN1_F5R2_Type(loc: 0x4000666c),
  F6R1: CAN1_F6R1_Type(loc: 0x40006670),
  F6R2: CAN1_F6R2_Type(loc: 0x40006674),
  F7R1: CAN1_F7R1_Type(loc: 0x40006678),
  F7R2: CAN1_F7R2_Type(loc: 0x4000667c),
  F8R1: CAN1_F8R1_Type(loc: 0x40006680),
  F8R2: CAN1_F8R2_Type(loc: 0x40006684),
  F9R1: CAN1_F9R1_Type(loc: 0x40006688),
  F9R2: CAN1_F9R2_Type(loc: 0x4000668c),
  F10R1: CAN1_F10R1_Type(loc: 0x40006690),
  F10R2: CAN1_F10R2_Type(loc: 0x40006694),
  F11R1: CAN1_F11R1_Type(loc: 0x40006698),
  F11R2: CAN1_F11R2_Type(loc: 0x4000669c),
  F12R1: CAN1_F12R1_Type(loc: 0x400066a0),
  F12R2: CAN1_F12R2_Type(loc: 0x400066a4),
  F13R1: CAN1_F13R1_Type(loc: 0x400066a8),
  F13R2: CAN1_F13R2_Type(loc: 0x400066ac),
)

const CAN2* = CAN1_Type(
  CAN_MCR: CAN1_CAN_MCR_Type(loc: 0x40006800),
  CAN_MSR: CAN1_CAN_MSR_Type(loc: 0x40006804),
  CAN_TSR: CAN1_CAN_TSR_Type(loc: 0x40006808),
  CAN_RF0R: CAN1_CAN_RF0R_Type(loc: 0x4000680c),
  CAN_RF1R: CAN1_CAN_RF1R_Type(loc: 0x40006810),
  CAN_IER: CAN1_CAN_IER_Type(loc: 0x40006814),
  CAN_ESR: CAN1_CAN_ESR_Type(loc: 0x40006818),
  CAN_BTR: CAN1_CAN_BTR_Type(loc: 0x4000681c),
  CAN_TI0R: CAN1_CAN_TI0R_Type(loc: 0x40006980),
  CAN_TDT0R: CAN1_CAN_TDT0R_Type(loc: 0x40006984),
  CAN_TDL0R: CAN1_CAN_TDL0R_Type(loc: 0x40006988),
  CAN_TDH0R: CAN1_CAN_TDH0R_Type(loc: 0x4000698c),
  CAN_TI1R: CAN1_CAN_TI1R_Type(loc: 0x40006990),
  CAN_TDT1R: CAN1_CAN_TDT1R_Type(loc: 0x40006994),
  CAN_TDL1R: CAN1_CAN_TDL1R_Type(loc: 0x40006998),
  CAN_TDH1R: CAN1_CAN_TDH1R_Type(loc: 0x4000699c),
  CAN_TI2R: CAN1_CAN_TI2R_Type(loc: 0x400069a0),
  CAN_TDT2R: CAN1_CAN_TDT2R_Type(loc: 0x400069a4),
  CAN_TDL2R: CAN1_CAN_TDL2R_Type(loc: 0x400069a8),
  CAN_TDH2R: CAN1_CAN_TDH2R_Type(loc: 0x400069ac),
  CAN_RI0R: CAN1_CAN_RI0R_Type(loc: 0x400069b0),
  CAN_RDT0R: CAN1_CAN_RDT0R_Type(loc: 0x400069b4),
  CAN_RDL0R: CAN1_CAN_RDL0R_Type(loc: 0x400069b8),
  CAN_RDH0R: CAN1_CAN_RDH0R_Type(loc: 0x400069bc),
  CAN_RI1R: CAN1_CAN_RI1R_Type(loc: 0x400069c0),
  CAN_RDT1R: CAN1_CAN_RDT1R_Type(loc: 0x400069c4),
  CAN_RDL1R: CAN1_CAN_RDL1R_Type(loc: 0x400069c8),
  CAN_RDH1R: CAN1_CAN_RDH1R_Type(loc: 0x400069cc),
  CAN_FMR: CAN1_CAN_FMR_Type(loc: 0x40006a00),
  CAN_FM1R: CAN1_CAN_FM1R_Type(loc: 0x40006a04),
  CAN_FS1R: CAN1_CAN_FS1R_Type(loc: 0x40006a0c),
  CAN_FFA1R: CAN1_CAN_FFA1R_Type(loc: 0x40006a14),
  CAN_FA1R: CAN1_CAN_FA1R_Type(loc: 0x40006a1c),
  F0R1: CAN1_F0R1_Type(loc: 0x40006a40),
  F0R2: CAN1_F0R2_Type(loc: 0x40006a44),
  F1R1: CAN1_F1R1_Type(loc: 0x40006a48),
  F1R2: CAN1_F1R2_Type(loc: 0x40006a4c),
  F2R1: CAN1_F2R1_Type(loc: 0x40006a50),
  F2R2: CAN1_F2R2_Type(loc: 0x40006a54),
  F3R1: CAN1_F3R1_Type(loc: 0x40006a58),
  F3R2: CAN1_F3R2_Type(loc: 0x40006a5c),
  F4R1: CAN1_F4R1_Type(loc: 0x40006a60),
  F4R2: CAN1_F4R2_Type(loc: 0x40006a64),
  F5R1: CAN1_F5R1_Type(loc: 0x40006a68),
  F5R2: CAN1_F5R2_Type(loc: 0x40006a6c),
  F6R1: CAN1_F6R1_Type(loc: 0x40006a70),
  F6R2: CAN1_F6R2_Type(loc: 0x40006a74),
  F7R1: CAN1_F7R1_Type(loc: 0x40006a78),
  F7R2: CAN1_F7R2_Type(loc: 0x40006a7c),
  F8R1: CAN1_F8R1_Type(loc: 0x40006a80),
  F8R2: CAN1_F8R2_Type(loc: 0x40006a84),
  F9R1: CAN1_F9R1_Type(loc: 0x40006a88),
  F9R2: CAN1_F9R2_Type(loc: 0x40006a8c),
  F10R1: CAN1_F10R1_Type(loc: 0x40006a90),
  F10R2: CAN1_F10R2_Type(loc: 0x40006a94),
  F11R1: CAN1_F11R1_Type(loc: 0x40006a98),
  F11R2: CAN1_F11R2_Type(loc: 0x40006a9c),
  F12R1: CAN1_F12R1_Type(loc: 0x40006aa0),
  F12R2: CAN1_F12R2_Type(loc: 0x40006aa4),
  F13R1: CAN1_F13R1_Type(loc: 0x40006aa8),
  F13R2: CAN1_F13R2_Type(loc: 0x40006aac),
)

const DAC* = DAC_Type(
  CR: DAC_CR_Type(loc: 0x40007400),
  SWTRIGR: DAC_SWTRIGR_Type(loc: 0x40007404),
  DHR12R1: DAC_DHR12R1_Type(loc: 0x40007408),
  DHR12L1: DAC_DHR12L1_Type(loc: 0x4000740c),
  DHR8R1: DAC_DHR8R1_Type(loc: 0x40007410),
  DHR12R2: DAC_DHR12R2_Type(loc: 0x40007414),
  DHR12L2: DAC_DHR12L2_Type(loc: 0x40007418),
  DHR8R2: DAC_DHR8R2_Type(loc: 0x4000741c),
  DHR12RD: DAC_DHR12RD_Type(loc: 0x40007420),
  DHR12LD: DAC_DHR12LD_Type(loc: 0x40007424),
  DHR8RD: DAC_DHR8RD_Type(loc: 0x40007428),
  DOR1: DAC_DOR1_Type(loc: 0x4000742c),
  DOR2: DAC_DOR2_Type(loc: 0x40007430),
)

const DBG* = DBG_Type(
  IDCODE: DBG_IDCODE_Type(loc: 0xe0042000),
  CR: DBG_CR_Type(loc: 0xe0042004),
)

const UART4* = UART4_Type(
  SR: UART4_SR_Type(loc: 0x40004c00),
  DR: UART4_DR_Type(loc: 0x40004c04),
  BRR: UART4_BRR_Type(loc: 0x40004c08),
  CR1: UART4_CR1_Type(loc: 0x40004c0c),
  CR2: UART4_CR2_Type(loc: 0x40004c10),
  CR3: UART4_CR3_Type(loc: 0x40004c14),
)

const UART5* = UART5_Type(
  SR: UART5_SR_Type(loc: 0x40005000),
  DR: UART5_DR_Type(loc: 0x40005004),
  BRR: UART5_BRR_Type(loc: 0x40005008),
  CR1: UART5_CR1_Type(loc: 0x4000500c),
  CR2: UART5_CR2_Type(loc: 0x40005010),
  CR3: UART5_CR3_Type(loc: 0x40005014),
)

const CRC* = CRC_Type(
  DR: CRC_DR_Type(loc: 0x40023000),
  IDR: CRC_IDR_Type(loc: 0x40023004),
  CR: CRC_CR_Type(loc: 0x40023008),
)

const FLASH* = FLASH_Type(
  ACR: FLASH_ACR_Type(loc: 0x40022000),
  KEYR: FLASH_KEYR_Type(loc: 0x40022004),
  OPTKEYR: FLASH_OPTKEYR_Type(loc: 0x40022008),
  SR: FLASH_SR_Type(loc: 0x4002200c),
  CR: FLASH_CR_Type(loc: 0x40022010),
  AR: FLASH_AR_Type(loc: 0x40022014),
  OBR: FLASH_OBR_Type(loc: 0x4002201c),
  WRPR: FLASH_WRPR_Type(loc: 0x40022020),
)

const USB* = USB_Type(
  EP0R: USB_EP0R_Type(loc: 0x40005c00),
  EP1R: USB_EP1R_Type(loc: 0x40005c04),
  EP2R: USB_EP2R_Type(loc: 0x40005c08),
  EP3R: USB_EP3R_Type(loc: 0x40005c0c),
  EP4R: USB_EP4R_Type(loc: 0x40005c10),
  EP5R: USB_EP5R_Type(loc: 0x40005c14),
  EP6R: USB_EP6R_Type(loc: 0x40005c18),
  EP7R: USB_EP7R_Type(loc: 0x40005c1c),
  CNTR: USB_CNTR_Type(loc: 0x40005c40),
  ISTR: USB_ISTR_Type(loc: 0x40005c44),
  FNR: USB_FNR_Type(loc: 0x40005c48),
  DADDR: USB_DADDR_Type(loc: 0x40005c4c),
  BTABLE: USB_BTABLE_Type(loc: 0x40005c50),
)

const OTG_FS_DEVICE* = OTG_FS_DEVICE_Type(
  FS_DCFG: OTG_FS_DEVICE_FS_DCFG_Type(loc: 0x50000800),
  FS_DCTL: OTG_FS_DEVICE_FS_DCTL_Type(loc: 0x50000804),
  FS_DSTS: OTG_FS_DEVICE_FS_DSTS_Type(loc: 0x50000808),
  FS_DIEPMSK: OTG_FS_DEVICE_FS_DIEPMSK_Type(loc: 0x50000810),
  FS_DOEPMSK: OTG_FS_DEVICE_FS_DOEPMSK_Type(loc: 0x50000814),
  FS_DAINT: OTG_FS_DEVICE_FS_DAINT_Type(loc: 0x50000818),
  FS_DAINTMSK: OTG_FS_DEVICE_FS_DAINTMSK_Type(loc: 0x5000081c),
  DVBUSDIS: OTG_FS_DEVICE_DVBUSDIS_Type(loc: 0x50000828),
  DVBUSPULSE: OTG_FS_DEVICE_DVBUSPULSE_Type(loc: 0x5000082c),
  DIEPEMPMSK: OTG_FS_DEVICE_DIEPEMPMSK_Type(loc: 0x50000834),
  FS_DIEPCTL0: OTG_FS_DEVICE_FS_DIEPCTL0_Type(loc: 0x50000900),
  DIEPINT0: OTG_FS_DEVICE_DIEPINT0_Type(loc: 0x50000908),
  DIEPTSIZ0: OTG_FS_DEVICE_DIEPTSIZ0_Type(loc: 0x50000910),
  DTXFSTS0: OTG_FS_DEVICE_DTXFSTS0_Type(loc: 0x50000918),
  DIEPCTL1: OTG_FS_DEVICE_DIEPCTL1_Type(loc: 0x50000920),
  DIEPINT1: OTG_FS_DEVICE_DIEPINT1_Type(loc: 0x50000928),
  DIEPTSIZ1: OTG_FS_DEVICE_DIEPTSIZ1_Type(loc: 0x50000930),
  DTXFSTS1: OTG_FS_DEVICE_DTXFSTS1_Type(loc: 0x50000938),
  DIEPCTL2: OTG_FS_DEVICE_DIEPCTL2_Type(loc: 0x50000940),
  DIEPINT2: OTG_FS_DEVICE_DIEPINT2_Type(loc: 0x50000948),
  DIEPTSIZ2: OTG_FS_DEVICE_DIEPTSIZ2_Type(loc: 0x50000950),
  DTXFSTS2: OTG_FS_DEVICE_DTXFSTS2_Type(loc: 0x50000958),
  DIEPCTL3: OTG_FS_DEVICE_DIEPCTL3_Type(loc: 0x50000960),
  DIEPINT3: OTG_FS_DEVICE_DIEPINT3_Type(loc: 0x50000968),
  DIEPTSIZ3: OTG_FS_DEVICE_DIEPTSIZ3_Type(loc: 0x50000970),
  DTXFSTS3: OTG_FS_DEVICE_DTXFSTS3_Type(loc: 0x50000978),
  DOEPCTL0: OTG_FS_DEVICE_DOEPCTL0_Type(loc: 0x50000b00),
  DOEPINT0: OTG_FS_DEVICE_DOEPINT0_Type(loc: 0x50000b08),
  DOEPTSIZ0: OTG_FS_DEVICE_DOEPTSIZ0_Type(loc: 0x50000b10),
  DOEPCTL1: OTG_FS_DEVICE_DOEPCTL1_Type(loc: 0x50000b20),
  DOEPINT1: OTG_FS_DEVICE_DOEPINT1_Type(loc: 0x50000b28),
  DOEPTSIZ1: OTG_FS_DEVICE_DOEPTSIZ1_Type(loc: 0x50000b30),
  DOEPCTL2: OTG_FS_DEVICE_DOEPCTL2_Type(loc: 0x50000b40),
  DOEPINT2: OTG_FS_DEVICE_DOEPINT2_Type(loc: 0x50000b48),
  DOEPTSIZ2: OTG_FS_DEVICE_DOEPTSIZ2_Type(loc: 0x50000b50),
  DOEPCTL3: OTG_FS_DEVICE_DOEPCTL3_Type(loc: 0x50000b60),
  DOEPINT3: OTG_FS_DEVICE_DOEPINT3_Type(loc: 0x50000b68),
  DOEPTSIZ3: OTG_FS_DEVICE_DOEPTSIZ3_Type(loc: 0x50000b70),
)

const OTG_FS_GLOBAL* = OTG_FS_GLOBAL_Type(
  FS_GOTGCTL: OTG_FS_GLOBAL_FS_GOTGCTL_Type(loc: 0x50000000),
  FS_GOTGINT: OTG_FS_GLOBAL_FS_GOTGINT_Type(loc: 0x50000004),
  FS_GAHBCFG: OTG_FS_GLOBAL_FS_GAHBCFG_Type(loc: 0x50000008),
  FS_GUSBCFG: OTG_FS_GLOBAL_FS_GUSBCFG_Type(loc: 0x5000000c),
  FS_GRSTCTL: OTG_FS_GLOBAL_FS_GRSTCTL_Type(loc: 0x50000010),
  FS_GINTSTS: OTG_FS_GLOBAL_FS_GINTSTS_Type(loc: 0x50000014),
  FS_GINTMSK: OTG_FS_GLOBAL_FS_GINTMSK_Type(loc: 0x50000018),
  FS_GRXSTSR_Device: OTG_FS_GLOBAL_FS_GRXSTSR_Device_Type(loc: 0x5000001c),
  FS_GRXSTSR_Host: OTG_FS_GLOBAL_FS_GRXSTSR_Host_Type(loc: 0x5000001c),
  FS_GRXFSIZ: OTG_FS_GLOBAL_FS_GRXFSIZ_Type(loc: 0x50000024),
  FS_GNPTXFSIZ_Device: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Type(loc: 0x50000028),
  FS_GNPTXFSIZ_Host: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Type(loc: 0x50000028),
  FS_GNPTXSTS: OTG_FS_GLOBAL_FS_GNPTXSTS_Type(loc: 0x5000002c),
  FS_GCCFG: OTG_FS_GLOBAL_FS_GCCFG_Type(loc: 0x50000038),
  FS_CID: OTG_FS_GLOBAL_FS_CID_Type(loc: 0x5000003c),
  FS_HPTXFSIZ: OTG_FS_GLOBAL_FS_HPTXFSIZ_Type(loc: 0x50000100),
  FS_DIEPTXF1: OTG_FS_GLOBAL_FS_DIEPTXF1_Type(loc: 0x50000104),
  FS_DIEPTXF2: OTG_FS_GLOBAL_FS_DIEPTXF2_Type(loc: 0x50000108),
  FS_DIEPTXF3: OTG_FS_GLOBAL_FS_DIEPTXF3_Type(loc: 0x5000010c),
)

const OTG_FS_HOST* = OTG_FS_HOST_Type(
  FS_HCFG: OTG_FS_HOST_FS_HCFG_Type(loc: 0x50000400),
  HFIR: OTG_FS_HOST_HFIR_Type(loc: 0x50000404),
  FS_HFNUM: OTG_FS_HOST_FS_HFNUM_Type(loc: 0x50000408),
  FS_HPTXSTS: OTG_FS_HOST_FS_HPTXSTS_Type(loc: 0x50000410),
  HAINT: OTG_FS_HOST_HAINT_Type(loc: 0x50000414),
  HAINTMSK: OTG_FS_HOST_HAINTMSK_Type(loc: 0x50000418),
  FS_HPRT: OTG_FS_HOST_FS_HPRT_Type(loc: 0x50000440),
  FS_HCCHAR0: OTG_FS_HOST_FS_HCCHAR0_Type(loc: 0x50000500),
  FS_HCINT0: OTG_FS_HOST_FS_HCINT0_Type(loc: 0x50000508),
  FS_HCINTMSK0: OTG_FS_HOST_FS_HCINTMSK0_Type(loc: 0x5000050c),
  FS_HCTSIZ0: OTG_FS_HOST_FS_HCTSIZ0_Type(loc: 0x50000510),
  FS_HCCHAR1: OTG_FS_HOST_FS_HCCHAR1_Type(loc: 0x50000520),
  FS_HCINT1: OTG_FS_HOST_FS_HCINT1_Type(loc: 0x50000528),
  FS_HCINTMSK1: OTG_FS_HOST_FS_HCINTMSK1_Type(loc: 0x5000052c),
  FS_HCTSIZ1: OTG_FS_HOST_FS_HCTSIZ1_Type(loc: 0x50000530),
  FS_HCCHAR2: OTG_FS_HOST_FS_HCCHAR2_Type(loc: 0x50000540),
  FS_HCINT2: OTG_FS_HOST_FS_HCINT2_Type(loc: 0x50000548),
  FS_HCINTMSK2: OTG_FS_HOST_FS_HCINTMSK2_Type(loc: 0x5000054c),
  FS_HCTSIZ2: OTG_FS_HOST_FS_HCTSIZ2_Type(loc: 0x50000550),
  FS_HCCHAR3: OTG_FS_HOST_FS_HCCHAR3_Type(loc: 0x50000560),
  FS_HCINT3: OTG_FS_HOST_FS_HCINT3_Type(loc: 0x50000568),
  FS_HCINTMSK3: OTG_FS_HOST_FS_HCINTMSK3_Type(loc: 0x5000056c),
  FS_HCTSIZ3: OTG_FS_HOST_FS_HCTSIZ3_Type(loc: 0x50000570),
  FS_HCCHAR4: OTG_FS_HOST_FS_HCCHAR4_Type(loc: 0x50000580),
  FS_HCINT4: OTG_FS_HOST_FS_HCINT4_Type(loc: 0x50000588),
  FS_HCINTMSK4: OTG_FS_HOST_FS_HCINTMSK4_Type(loc: 0x5000058c),
  FS_HCTSIZ4: OTG_FS_HOST_FS_HCTSIZ4_Type(loc: 0x50000590),
  FS_HCCHAR5: OTG_FS_HOST_FS_HCCHAR5_Type(loc: 0x500005a0),
  FS_HCINT5: OTG_FS_HOST_FS_HCINT5_Type(loc: 0x500005a8),
  FS_HCINTMSK5: OTG_FS_HOST_FS_HCINTMSK5_Type(loc: 0x500005ac),
  FS_HCTSIZ5: OTG_FS_HOST_FS_HCTSIZ5_Type(loc: 0x500005b0),
  FS_HCCHAR6: OTG_FS_HOST_FS_HCCHAR6_Type(loc: 0x500005c0),
  FS_HCINT6: OTG_FS_HOST_FS_HCINT6_Type(loc: 0x500005c8),
  FS_HCINTMSK6: OTG_FS_HOST_FS_HCINTMSK6_Type(loc: 0x500005cc),
  FS_HCTSIZ6: OTG_FS_HOST_FS_HCTSIZ6_Type(loc: 0x500005d0),
  FS_HCCHAR7: OTG_FS_HOST_FS_HCCHAR7_Type(loc: 0x500005e0),
  FS_HCINT7: OTG_FS_HOST_FS_HCINT7_Type(loc: 0x500005e8),
  FS_HCINTMSK7: OTG_FS_HOST_FS_HCINTMSK7_Type(loc: 0x500005ec),
  FS_HCTSIZ7: OTG_FS_HOST_FS_HCTSIZ7_Type(loc: 0x500005f0),
)

const OTG_FS_PWRCLK* = OTG_FS_PWRCLK_Type(
  FS_PCGCCTL: OTG_FS_PWRCLK_FS_PCGCCTL_Type(loc: 0x50000e00),
)

const ETHERNET_MMC* = ETHERNET_MMC_Type(
  MMCCR: ETHERNET_MMC_MMCCR_Type(loc: 0x40028100),
  MMCRIR: ETHERNET_MMC_MMCRIR_Type(loc: 0x40028104),
  MMCTIR: ETHERNET_MMC_MMCTIR_Type(loc: 0x40028108),
  MMCRIMR: ETHERNET_MMC_MMCRIMR_Type(loc: 0x4002810c),
  MMCTIMR: ETHERNET_MMC_MMCTIMR_Type(loc: 0x40028110),
  MMCTGFSCCR: ETHERNET_MMC_MMCTGFSCCR_Type(loc: 0x4002814c),
  MMCTGFMSCCR: ETHERNET_MMC_MMCTGFMSCCR_Type(loc: 0x40028150),
  MMCTGFCR: ETHERNET_MMC_MMCTGFCR_Type(loc: 0x40028168),
  MMCRFCECR: ETHERNET_MMC_MMCRFCECR_Type(loc: 0x40028194),
  MMCRFAECR: ETHERNET_MMC_MMCRFAECR_Type(loc: 0x40028198),
  MMCRGUFCR: ETHERNET_MMC_MMCRGUFCR_Type(loc: 0x400281c4),
)

const ETHERNET_MAC* = ETHERNET_MAC_Type(
  MACCR: ETHERNET_MAC_MACCR_Type(loc: 0x40028000),
  MACFFR: ETHERNET_MAC_MACFFR_Type(loc: 0x40028004),
  MACHTHR: ETHERNET_MAC_MACHTHR_Type(loc: 0x40028008),
  MACHTLR: ETHERNET_MAC_MACHTLR_Type(loc: 0x4002800c),
  MACMIIAR: ETHERNET_MAC_MACMIIAR_Type(loc: 0x40028010),
  MACMIIDR: ETHERNET_MAC_MACMIIDR_Type(loc: 0x40028014),
  MACFCR: ETHERNET_MAC_MACFCR_Type(loc: 0x40028018),
  MACVLANTR: ETHERNET_MAC_MACVLANTR_Type(loc: 0x4002801c),
  MACRWUFFR: ETHERNET_MAC_MACRWUFFR_Type(loc: 0x40028028),
  MACPMTCSR: ETHERNET_MAC_MACPMTCSR_Type(loc: 0x4002802c),
  MACSR: ETHERNET_MAC_MACSR_Type(loc: 0x40028038),
  MACIMR: ETHERNET_MAC_MACIMR_Type(loc: 0x4002803c),
  MACA0HR: ETHERNET_MAC_MACA0HR_Type(loc: 0x40028040),
  MACA0LR: ETHERNET_MAC_MACA0LR_Type(loc: 0x40028044),
  MACA1HR: ETHERNET_MAC_MACA1HR_Type(loc: 0x40028048),
  MACA1LR: ETHERNET_MAC_MACA1LR_Type(loc: 0x4002804c),
  MACA2HR: ETHERNET_MAC_MACA2HR_Type(loc: 0x40028050),
  MACA2LR: ETHERNET_MAC_MACA2LR_Type(loc: 0x40028054),
  MACA3HR: ETHERNET_MAC_MACA3HR_Type(loc: 0x40028058),
  MACA3LR: ETHERNET_MAC_MACA3LR_Type(loc: 0x4002805c),
)

const ETHERNET_PTP* = ETHERNET_PTP_Type(
  PTPTSCR: ETHERNET_PTP_PTPTSCR_Type(loc: 0x40028700),
  PTPSSIR: ETHERNET_PTP_PTPSSIR_Type(loc: 0x40028704),
  PTPTSHR: ETHERNET_PTP_PTPTSHR_Type(loc: 0x40028708),
  PTPTSLR: ETHERNET_PTP_PTPTSLR_Type(loc: 0x4002870c),
  PTPTSHUR: ETHERNET_PTP_PTPTSHUR_Type(loc: 0x40028710),
  PTPTSLUR: ETHERNET_PTP_PTPTSLUR_Type(loc: 0x40028714),
  PTPTSAR: ETHERNET_PTP_PTPTSAR_Type(loc: 0x40028718),
  PTPTTHR: ETHERNET_PTP_PTPTTHR_Type(loc: 0x4002871c),
  PTPTTLR: ETHERNET_PTP_PTPTTLR_Type(loc: 0x40028720),
)

const ETHERNET_DMA* = ETHERNET_DMA_Type(
  DMABMR: ETHERNET_DMA_DMABMR_Type(loc: 0x40029000),
  DMATPDR: ETHERNET_DMA_DMATPDR_Type(loc: 0x40029004),
  DMARPDR: ETHERNET_DMA_DMARPDR_Type(loc: 0x40029008),
  DMARDLAR: ETHERNET_DMA_DMARDLAR_Type(loc: 0x4002900c),
  DMATDLAR: ETHERNET_DMA_DMATDLAR_Type(loc: 0x40029010),
  DMASR: ETHERNET_DMA_DMASR_Type(loc: 0x40029014),
  DMAOMR: ETHERNET_DMA_DMAOMR_Type(loc: 0x40029018),
  DMAIER: ETHERNET_DMA_DMAIER_Type(loc: 0x4002901c),
  DMAMFBOCR: ETHERNET_DMA_DMAMFBOCR_Type(loc: 0x40029020),
  DMACHTDR: ETHERNET_DMA_DMACHTDR_Type(loc: 0x40029048),
  DMACHRDR: ETHERNET_DMA_DMACHRDR_Type(loc: 0x4002904c),
  DMACHTBAR: ETHERNET_DMA_DMACHTBAR_Type(loc: 0x40029050),
  DMACHRBAR: ETHERNET_DMA_DMACHRBAR_Type(loc: 0x40029054),
)

const NVIC* = NVIC_Type(
  ISER0: NVIC_ISER0_Type(loc: 0xe000e100),
  ISER1: NVIC_ISER1_Type(loc: 0xe000e104),
  ICER0: NVIC_ICER0_Type(loc: 0xe000e180),
  ICER1: NVIC_ICER1_Type(loc: 0xe000e184),
  ISPR0: NVIC_ISPR0_Type(loc: 0xe000e200),
  ISPR1: NVIC_ISPR1_Type(loc: 0xe000e204),
  ICPR0: NVIC_ICPR0_Type(loc: 0xe000e280),
  ICPR1: NVIC_ICPR1_Type(loc: 0xe000e284),
  IABR0: NVIC_IABR0_Type(loc: 0xe000e300),
  IABR1: NVIC_IABR1_Type(loc: 0xe000e304),
  IPR0: NVIC_IPR0_Type(loc: 0xe000e400),
  IPR1: NVIC_IPR1_Type(loc: 0xe000e404),
  IPR2: NVIC_IPR2_Type(loc: 0xe000e408),
  IPR3: NVIC_IPR3_Type(loc: 0xe000e40c),
  IPR4: NVIC_IPR4_Type(loc: 0xe000e410),
  IPR5: NVIC_IPR5_Type(loc: 0xe000e414),
  IPR6: NVIC_IPR6_Type(loc: 0xe000e418),
  IPR7: NVIC_IPR7_Type(loc: 0xe000e41c),
  IPR8: NVIC_IPR8_Type(loc: 0xe000e420),
  IPR9: NVIC_IPR9_Type(loc: 0xe000e424),
  IPR10: NVIC_IPR10_Type(loc: 0xe000e428),
  IPR11: NVIC_IPR11_Type(loc: 0xe000e42c),
  IPR12: NVIC_IPR12_Type(loc: 0xe000e430),
  IPR13: NVIC_IPR13_Type(loc: 0xe000e434),
  IPR14: NVIC_IPR14_Type(loc: 0xe000e438),
)

const MPU* = MPU_Type(
  MPU_TYPER: MPU_MPU_TYPER_Type(loc: 0xe000ed90),
  MPU_CTRL: MPU_MPU_CTRL_Type(loc: 0xe000ed94),
  MPU_RNR: MPU_MPU_RNR_Type(loc: 0xe000ed98),
  MPU_RBAR: MPU_MPU_RBAR_Type(loc: 0xe000ed9c),
  MPU_RASR: MPU_MPU_RASR_Type(loc: 0xe000eda0),
)

const SCB_ACTRL* = SCB_ACTRL_Type(
  ACTRL: SCB_ACTRL_ACTRL_Type(loc: 0xe000e008),
)

const NVIC_STIR* = NVIC_STIR_Type(
  STIR: NVIC_STIR_STIR_Type(loc: 0xe000ef00),
)

const SCB* = SCB_Type(
  CPUID: SCB_CPUID_Type(loc: 0xe000ed00),
  ICSR: SCB_ICSR_Type(loc: 0xe000ed04),
  VTOR: SCB_VTOR_Type(loc: 0xe000ed08),
  AIRCR: SCB_AIRCR_Type(loc: 0xe000ed0c),
  SCR: SCB_SCR_Type(loc: 0xe000ed10),
  CCR: SCB_CCR_Type(loc: 0xe000ed14),
  SHPR1: SCB_SHPR1_Type(loc: 0xe000ed18),
  SHPR2: SCB_SHPR2_Type(loc: 0xe000ed1c),
  SHPR3: SCB_SHPR3_Type(loc: 0xe000ed20),
  SHCRS: SCB_SHCRS_Type(loc: 0xe000ed24),
  CFSR_UFSR_BFSR_MMFSR: SCB_CFSR_UFSR_BFSR_MMFSR_Type(loc: 0xe000ed28),
  HFSR: SCB_HFSR_Type(loc: 0xe000ed2c),
  MMFAR: SCB_MMFAR_Type(loc: 0xe000ed34),
  BFAR: SCB_BFAR_Type(loc: 0xe000ed38),
)

const STK* = STK_Type(
  CTRL: STK_CTRL_Type(loc: 0xe000e010),
  LOAD: STK_LOAD_Type(loc: 0xe000e014),
  VAL: STK_VAL_Type(loc: 0xe000e018),
  CALIB: STK_CALIB_Type(loc: 0xe000e01c),
)


################################################################################
# Accessors for peripheral registers
################################################################################
type FSMC_BCR1_Fields* = object
  MBKEN* {.bitsize:1.}: bool
  MUXEN* {.bitsize:1.}: bool
  MTYP* {.bitsize:2.}: 0'u .. 3'u
  MWID* {.bitsize:2.}: 0'u .. 3'u
  FACCEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  BURSTEN* {.bitsize:1.}: bool
  WAITPOL* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  WAITCFG* {.bitsize:1.}: bool
  WREN* {.bitsize:1.}: bool
  WAITEN* {.bitsize:1.}: bool
  EXTMOD* {.bitsize:1.}: bool
  ASYNCWAIT* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:3.}: 0'u .. 7'u
  CBURSTRW* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_BTR1_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  BUSTURN* {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type FSMC_BCR2_Fields* = object
  MBKEN* {.bitsize:1.}: bool
  MUXEN* {.bitsize:1.}: bool
  MTYP* {.bitsize:2.}: 0'u .. 3'u
  MWID* {.bitsize:2.}: 0'u .. 3'u
  FACCEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  BURSTEN* {.bitsize:1.}: bool
  WAITPOL* {.bitsize:1.}: bool
  WRAPMOD* {.bitsize:1.}: bool
  WAITCFG* {.bitsize:1.}: bool
  WREN* {.bitsize:1.}: bool
  WAITEN* {.bitsize:1.}: bool
  EXTMOD* {.bitsize:1.}: bool
  ASYNCWAIT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  CBURSTRW* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_BTR2_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  BUSTURN* {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type FSMC_BCR3_Fields* = object
  MBKEN* {.bitsize:1.}: bool
  MUXEN* {.bitsize:1.}: bool
  MTYP* {.bitsize:2.}: 0'u .. 3'u
  MWID* {.bitsize:2.}: 0'u .. 3'u
  FACCEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  BURSTEN* {.bitsize:1.}: bool
  WAITPOL* {.bitsize:1.}: bool
  WRAPMOD* {.bitsize:1.}: bool
  WAITCFG* {.bitsize:1.}: bool
  WREN* {.bitsize:1.}: bool
  WAITEN* {.bitsize:1.}: bool
  EXTMOD* {.bitsize:1.}: bool
  ASYNCWAIT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  CBURSTRW* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_BTR3_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  BUSTURN* {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type FSMC_BCR4_Fields* = object
  MBKEN* {.bitsize:1.}: bool
  MUXEN* {.bitsize:1.}: bool
  MTYP* {.bitsize:2.}: 0'u .. 3'u
  MWID* {.bitsize:2.}: 0'u .. 3'u
  FACCEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  BURSTEN* {.bitsize:1.}: bool
  WAITPOL* {.bitsize:1.}: bool
  WRAPMOD* {.bitsize:1.}: bool
  WAITCFG* {.bitsize:1.}: bool
  WREN* {.bitsize:1.}: bool
  WAITEN* {.bitsize:1.}: bool
  EXTMOD* {.bitsize:1.}: bool
  ASYNCWAIT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  CBURSTRW* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_BTR4_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  BUSTURN* {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type FSMC_PCR2_Fields* = object
  RESERVED {.bitsize:1.}: bool
  PWAITEN* {.bitsize:1.}: bool
  PBKEN* {.bitsize:1.}: bool
  PTYP* {.bitsize:1.}: bool
  PWID* {.bitsize:2.}: 0'u .. 3'u
  ECCEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  TCLR* {.bitsize:4.}: 0'u .. 15'u
  TAR* {.bitsize:4.}: 0'u .. 15'u
  ECCPS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_SR2_Fields* = object
  IRS* {.bitsize:1.}: bool
  ILS* {.bitsize:1.}: bool
  IFS* {.bitsize:1.}: bool
  IREN* {.bitsize:1.}: bool
  ILEN* {.bitsize:1.}: bool
  IFEN* {.bitsize:1.}: bool
  FEMPT* {.bitsize:1.}: bool
  RESERVED {.bitsize:25.}: 0'u .. 33554431'u

type FSMC_PMEM2_Fields* = object
  MEMSETx* {.bitsize:8.}: 0'u .. 255'u
  MEMWAITx* {.bitsize:8.}: 0'u .. 255'u
  MEMHOLDx* {.bitsize:8.}: 0'u .. 255'u
  MEMHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_PATT2_Fields* = object
  ATTSETx* {.bitsize:8.}: 0'u .. 255'u
  ATTWAITx* {.bitsize:8.}: 0'u .. 255'u
  ATTHOLDx* {.bitsize:8.}: 0'u .. 255'u
  ATTHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_PCR3_Fields* = object
  RESERVED {.bitsize:1.}: bool
  PWAITEN* {.bitsize:1.}: bool
  PBKEN* {.bitsize:1.}: bool
  PTYP* {.bitsize:1.}: bool
  PWID* {.bitsize:2.}: 0'u .. 3'u
  ECCEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  TCLR* {.bitsize:4.}: 0'u .. 15'u
  TAR* {.bitsize:4.}: 0'u .. 15'u
  ECCPS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_SR3_Fields* = object
  IRS* {.bitsize:1.}: bool
  ILS* {.bitsize:1.}: bool
  IFS* {.bitsize:1.}: bool
  IREN* {.bitsize:1.}: bool
  ILEN* {.bitsize:1.}: bool
  IFEN* {.bitsize:1.}: bool
  FEMPT* {.bitsize:1.}: bool
  RESERVED {.bitsize:25.}: 0'u .. 33554431'u

type FSMC_PMEM3_Fields* = object
  MEMSETx* {.bitsize:8.}: 0'u .. 255'u
  MEMWAITx* {.bitsize:8.}: 0'u .. 255'u
  MEMHOLDx* {.bitsize:8.}: 0'u .. 255'u
  MEMHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_PATT3_Fields* = object
  ATTSETx* {.bitsize:8.}: 0'u .. 255'u
  ATTWAITx* {.bitsize:8.}: 0'u .. 255'u
  ATTHOLDx* {.bitsize:8.}: 0'u .. 255'u
  ATTHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_PCR4_Fields* = object
  RESERVED {.bitsize:1.}: bool
  PWAITEN* {.bitsize:1.}: bool
  PBKEN* {.bitsize:1.}: bool
  PTYP* {.bitsize:1.}: bool
  PWID* {.bitsize:2.}: 0'u .. 3'u
  ECCEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  TCLR* {.bitsize:4.}: 0'u .. 15'u
  TAR* {.bitsize:4.}: 0'u .. 15'u
  ECCPS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type FSMC_SR4_Fields* = object
  IRS* {.bitsize:1.}: bool
  ILS* {.bitsize:1.}: bool
  IFS* {.bitsize:1.}: bool
  IREN* {.bitsize:1.}: bool
  ILEN* {.bitsize:1.}: bool
  IFEN* {.bitsize:1.}: bool
  FEMPT* {.bitsize:1.}: bool
  RESERVED {.bitsize:25.}: 0'u .. 33554431'u

type FSMC_PMEM4_Fields* = object
  MEMSETx* {.bitsize:8.}: 0'u .. 255'u
  MEMWAITx* {.bitsize:8.}: 0'u .. 255'u
  MEMHOLDx* {.bitsize:8.}: 0'u .. 255'u
  MEMHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_PATT4_Fields* = object
  ATTSETx* {.bitsize:8.}: 0'u .. 255'u
  ATTWAITx* {.bitsize:8.}: 0'u .. 255'u
  ATTHOLDx* {.bitsize:8.}: 0'u .. 255'u
  ATTHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_PIO4_Fields* = object
  IOSETx* {.bitsize:8.}: 0'u .. 255'u
  IOWAITx* {.bitsize:8.}: 0'u .. 255'u
  IOHOLDx* {.bitsize:8.}: 0'u .. 255'u
  IOHIZx* {.bitsize:8.}: 0'u .. 255'u

type FSMC_BWTR1_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u

type FSMC_BWTR2_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u

type FSMC_BWTR3_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u

type FSMC_BWTR4_Fields* = object
  ADDSET* {.bitsize:4.}: 0'u .. 15'u
  ADDHLD* {.bitsize:4.}: 0'u .. 15'u
  DATAST* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  CLKDIV* {.bitsize:4.}: 0'u .. 15'u
  DATLAT* {.bitsize:4.}: 0'u .. 15'u
  ACCMOD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u

template read*(reg: FSMC_BCR1_Type): FSMC_BCR1_Fields =
  cast[FSMC_BCR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BCR1_Type, val: FSMC_BCR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BCR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BTR1_Type): FSMC_BTR1_Fields =
  cast[FSMC_BTR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BTR1_Type, val: FSMC_BTR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BTR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BCR2_Type): FSMC_BCR2_Fields =
  cast[FSMC_BCR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BCR2_Type, val: FSMC_BCR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BCR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BTR2_Type): FSMC_BTR2_Fields =
  cast[FSMC_BTR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BTR2_Type, val: FSMC_BTR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BTR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BCR3_Type): FSMC_BCR3_Fields =
  cast[FSMC_BCR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BCR3_Type, val: FSMC_BCR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BCR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BTR3_Type): FSMC_BTR3_Fields =
  cast[FSMC_BTR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BTR3_Type, val: FSMC_BTR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BTR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BCR4_Type): FSMC_BCR4_Fields =
  cast[FSMC_BCR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BCR4_Type, val: FSMC_BCR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BCR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BTR4_Type): FSMC_BTR4_Fields =
  cast[FSMC_BTR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BTR4_Type, val: FSMC_BTR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BTR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PCR2_Type): FSMC_PCR2_Fields =
  cast[FSMC_PCR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PCR2_Type, val: FSMC_PCR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PCR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_SR2_Type): FSMC_SR2_Fields =
  cast[FSMC_SR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_SR2_Type, val: FSMC_SR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_SR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PMEM2_Type): FSMC_PMEM2_Fields =
  cast[FSMC_PMEM2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PMEM2_Type, val: FSMC_PMEM2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PMEM2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PATT2_Type): FSMC_PATT2_Fields =
  cast[FSMC_PATT2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PATT2_Type, val: FSMC_PATT2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PATT2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_ECCR2_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: FSMC_PCR3_Type): FSMC_PCR3_Fields =
  cast[FSMC_PCR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PCR3_Type, val: FSMC_PCR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PCR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_SR3_Type): FSMC_SR3_Fields =
  cast[FSMC_SR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_SR3_Type, val: FSMC_SR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_SR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PMEM3_Type): FSMC_PMEM3_Fields =
  cast[FSMC_PMEM3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PMEM3_Type, val: FSMC_PMEM3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PMEM3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PATT3_Type): FSMC_PATT3_Fields =
  cast[FSMC_PATT3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PATT3_Type, val: FSMC_PATT3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PATT3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_ECCR3_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: FSMC_PCR4_Type): FSMC_PCR4_Fields =
  cast[FSMC_PCR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PCR4_Type, val: FSMC_PCR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PCR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_SR4_Type): FSMC_SR4_Fields =
  cast[FSMC_SR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_SR4_Type, val: FSMC_SR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_SR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PMEM4_Type): FSMC_PMEM4_Fields =
  cast[FSMC_PMEM4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PMEM4_Type, val: FSMC_PMEM4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PMEM4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PATT4_Type): FSMC_PATT4_Fields =
  cast[FSMC_PATT4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PATT4_Type, val: FSMC_PATT4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PATT4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_PIO4_Type): FSMC_PIO4_Fields =
  cast[FSMC_PIO4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_PIO4_Type, val: FSMC_PIO4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_PIO4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BWTR1_Type): FSMC_BWTR1_Fields =
  cast[FSMC_BWTR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BWTR1_Type, val: FSMC_BWTR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BWTR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BWTR2_Type): FSMC_BWTR2_Fields =
  cast[FSMC_BWTR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BWTR2_Type, val: FSMC_BWTR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BWTR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BWTR3_Type): FSMC_BWTR3_Fields =
  cast[FSMC_BWTR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BWTR3_Type, val: FSMC_BWTR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BWTR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FSMC_BWTR4_Type): FSMC_BWTR4_Fields =
  cast[FSMC_BWTR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FSMC_BWTR4_Type, val: FSMC_BWTR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FSMC_BWTR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type PWR_CR_Fields* = object
  LPDS* {.bitsize:1.}: bool
  PDDS* {.bitsize:1.}: bool
  CWUF* {.bitsize:1.}: bool
  CSBF* {.bitsize:1.}: bool
  PVDE* {.bitsize:1.}: bool
  PLS* {.bitsize:3.}: 0'u .. 7'u
  DBP* {.bitsize:1.}: bool
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

type PWR_CSR_Fields* = object
  WUF* {.bitsize:1.}: bool
  SBF* {.bitsize:1.}: bool
  PVDO* {.bitsize:1.}: bool
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  EWUP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:23.}: 0'u .. 8388607'u

template read*(reg: PWR_CR_Type): PWR_CR_Fields =
  cast[PWR_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: PWR_CR_Type, val: PWR_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: PWR_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: PWR_CSR_Type): PWR_CSR_Fields =
  cast[PWR_CSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: PWR_CSR_Type, val: PWR_CSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: PWR_CSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type RCC_CR_Fields* = object
  HSION* {.bitsize:1.}: bool
  HSIRDY* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  HSITRIM* {.bitsize:5.}: 0'u .. 31'u
  HSICAL* {.bitsize:8.}: 0'u .. 255'u
  HSEON* {.bitsize:1.}: bool
  HSERDY* {.bitsize:1.}: bool
  HSEBYP* {.bitsize:1.}: bool
  CSSON* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  PLLON* {.bitsize:1.}: bool
  PLLRDY* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:6.}: 0'u .. 63'u

type RCC_CFGR_Fields* = object
  SW* {.bitsize:2.}: 0'u .. 3'u
  SWS* {.bitsize:2.}: 0'u .. 3'u
  HPRE* {.bitsize:4.}: 0'u .. 15'u
  PPRE1* {.bitsize:3.}: 0'u .. 7'u
  PPRE2* {.bitsize:3.}: 0'u .. 7'u
  ADCPRE* {.bitsize:2.}: 0'u .. 3'u
  PLLSRC* {.bitsize:1.}: bool
  PLLXTPRE* {.bitsize:1.}: bool
  PLLMUL* {.bitsize:4.}: 0'u .. 15'u
  OTGFSPRE* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  MCO* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u

type RCC_CIR_Fields* = object
  LSIRDYF* {.bitsize:1.}: bool
  LSERDYF* {.bitsize:1.}: bool
  HSIRDYF* {.bitsize:1.}: bool
  HSERDYF* {.bitsize:1.}: bool
  PLLRDYF* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  CSSF* {.bitsize:1.}: bool
  LSIRDYIE* {.bitsize:1.}: bool
  LSERDYIE* {.bitsize:1.}: bool
  HSIRDYIE* {.bitsize:1.}: bool
  HSERDYIE* {.bitsize:1.}: bool
  PLLRDYIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  LSIRDYC* {.bitsize:1.}: bool
  LSERDYC* {.bitsize:1.}: bool
  HSIRDYC* {.bitsize:1.}: bool
  HSERDYC* {.bitsize:1.}: bool
  PLLRDYC* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:2.}: 0'u .. 3'u
  CSSC* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:8.}: 0'u .. 255'u

type RCC_APB2RSTR_Fields* = object
  AFIORST* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  IOPARST* {.bitsize:1.}: bool
  IOPBRST* {.bitsize:1.}: bool
  IOPCRST* {.bitsize:1.}: bool
  IOPDRST* {.bitsize:1.}: bool
  IOPERST* {.bitsize:1.}: bool
  IOPFRST* {.bitsize:1.}: bool
  IOPGRST* {.bitsize:1.}: bool
  ADC1RST* {.bitsize:1.}: bool
  ADC2RST* {.bitsize:1.}: bool
  TIM1RST* {.bitsize:1.}: bool
  SPI1RST* {.bitsize:1.}: bool
  TIM8RST* {.bitsize:1.}: bool
  USART1RST* {.bitsize:1.}: bool
  ADC3RST* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  TIM9RST* {.bitsize:1.}: bool
  TIM10RST* {.bitsize:1.}: bool
  TIM11RST* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:10.}: 0'u .. 1023'u

type RCC_APB1RSTR_Fields* = object
  TIM2RST* {.bitsize:1.}: bool
  TIM3RST* {.bitsize:1.}: bool
  TIM4RST* {.bitsize:1.}: bool
  TIM5RST* {.bitsize:1.}: bool
  TIM6RST* {.bitsize:1.}: bool
  TIM7RST* {.bitsize:1.}: bool
  TIM12RST* {.bitsize:1.}: bool
  TIM13RST* {.bitsize:1.}: bool
  TIM14RST* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  WWDGRST* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  SPI2RST* {.bitsize:1.}: bool
  SPI3RST* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  USART2RST* {.bitsize:1.}: bool
  USART3RST* {.bitsize:1.}: bool
  UART4RST* {.bitsize:1.}: bool
  UART5RST* {.bitsize:1.}: bool
  I2C1RST* {.bitsize:1.}: bool
  I2C2RST* {.bitsize:1.}: bool
  USBRST* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:1.}: bool
  CANRST* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:1.}: bool
  BKPRST* {.bitsize:1.}: bool
  PWRRST* {.bitsize:1.}: bool
  DACRST* {.bitsize:1.}: bool
  RESERVED5 {.bitsize:2.}: 0'u .. 3'u

type RCC_AHBENR_Fields* = object
  DMA1EN* {.bitsize:1.}: bool
  DMA2EN* {.bitsize:1.}: bool
  SRAMEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  FLITFEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  CRCEN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  FSMCEN* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:1.}: bool
  SDIOEN* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:21.}: 0'u .. 2097151'u

type RCC_APB2ENR_Fields* = object
  AFIOEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  IOPAEN* {.bitsize:1.}: bool
  IOPBEN* {.bitsize:1.}: bool
  IOPCEN* {.bitsize:1.}: bool
  IOPDEN* {.bitsize:1.}: bool
  IOPEEN* {.bitsize:1.}: bool
  IOPFEN* {.bitsize:1.}: bool
  IOPGEN* {.bitsize:1.}: bool
  ADC1EN* {.bitsize:1.}: bool
  ADC2EN* {.bitsize:1.}: bool
  TIM1EN* {.bitsize:1.}: bool
  SPI1EN* {.bitsize:1.}: bool
  TIM8EN* {.bitsize:1.}: bool
  USART1EN* {.bitsize:1.}: bool
  ADC3EN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  TIM9EN* {.bitsize:1.}: bool
  TIM10EN* {.bitsize:1.}: bool
  TIM11EN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:10.}: 0'u .. 1023'u

type RCC_APB1ENR_Fields* = object
  TIM2EN* {.bitsize:1.}: bool
  TIM3EN* {.bitsize:1.}: bool
  TIM4EN* {.bitsize:1.}: bool
  TIM5EN* {.bitsize:1.}: bool
  TIM6EN* {.bitsize:1.}: bool
  TIM7EN* {.bitsize:1.}: bool
  TIM12EN* {.bitsize:1.}: bool
  TIM13EN* {.bitsize:1.}: bool
  TIM14EN* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  WWDGEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  SPI2EN* {.bitsize:1.}: bool
  SPI3EN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  USART2EN* {.bitsize:1.}: bool
  USART3EN* {.bitsize:1.}: bool
  UART4EN* {.bitsize:1.}: bool
  UART5EN* {.bitsize:1.}: bool
  I2C1EN* {.bitsize:1.}: bool
  I2C2EN* {.bitsize:1.}: bool
  USBEN* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:1.}: bool
  CANEN* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:1.}: bool
  BKPEN* {.bitsize:1.}: bool
  PWREN* {.bitsize:1.}: bool
  DACEN* {.bitsize:1.}: bool
  RESERVED5 {.bitsize:2.}: 0'u .. 3'u

type RCC_BDCR_Fields* = object
  LSEON* {.bitsize:1.}: bool
  LSERDY* {.bitsize:1.}: bool
  LSEBYP* {.bitsize:1.}: bool
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  RTCSEL* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  RTCEN* {.bitsize:1.}: bool
  BDRST* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:15.}: 0'u .. 32767'u

type RCC_CSR_Fields* = object
  LSION* {.bitsize:1.}: bool
  LSIRDY* {.bitsize:1.}: bool
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u
  RMVF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  PINRSTF* {.bitsize:1.}: bool
  PORRSTF* {.bitsize:1.}: bool
  SFTRSTF* {.bitsize:1.}: bool
  IWDGRSTF* {.bitsize:1.}: bool
  WWDGRSTF* {.bitsize:1.}: bool
  LPWRRSTF* {.bitsize:1.}: bool

template read*(reg: RCC_CR_Type): RCC_CR_Fields =
  cast[RCC_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_CR_Type, val: RCC_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_CFGR_Type): RCC_CFGR_Fields =
  cast[RCC_CFGR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_CFGR_Type, val: RCC_CFGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_CFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_CIR_Type): RCC_CIR_Fields =
  cast[RCC_CIR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_CIR_Type, val: RCC_CIR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_CIR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_APB2RSTR_Type): RCC_APB2RSTR_Fields =
  cast[RCC_APB2RSTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_APB2RSTR_Type, val: RCC_APB2RSTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_APB2RSTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_APB1RSTR_Type): RCC_APB1RSTR_Fields =
  cast[RCC_APB1RSTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_APB1RSTR_Type, val: RCC_APB1RSTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_APB1RSTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_AHBENR_Type): RCC_AHBENR_Fields =
  cast[RCC_AHBENR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_AHBENR_Type, val: RCC_AHBENR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_AHBENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_APB2ENR_Type): RCC_APB2ENR_Fields =
  cast[RCC_APB2ENR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_APB2ENR_Type, val: RCC_APB2ENR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_APB2ENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_APB1ENR_Type): RCC_APB1ENR_Fields =
  cast[RCC_APB1ENR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_APB1ENR_Type, val: RCC_APB1ENR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_APB1ENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_BDCR_Type): RCC_BDCR_Fields =
  cast[RCC_BDCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_BDCR_Type, val: RCC_BDCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_BDCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RCC_CSR_Type): RCC_CSR_Fields =
  cast[RCC_CSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RCC_CSR_Type, val: RCC_CSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RCC_CSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type GPIOA_CRL_Fields* = object
  MODE0* {.bitsize:2.}: 0'u .. 3'u
  CNF0* {.bitsize:2.}: 0'u .. 3'u
  MODE1* {.bitsize:2.}: 0'u .. 3'u
  CNF1* {.bitsize:2.}: 0'u .. 3'u
  MODE2* {.bitsize:2.}: 0'u .. 3'u
  CNF2* {.bitsize:2.}: 0'u .. 3'u
  MODE3* {.bitsize:2.}: 0'u .. 3'u
  CNF3* {.bitsize:2.}: 0'u .. 3'u
  MODE4* {.bitsize:2.}: 0'u .. 3'u
  CNF4* {.bitsize:2.}: 0'u .. 3'u
  MODE5* {.bitsize:2.}: 0'u .. 3'u
  CNF5* {.bitsize:2.}: 0'u .. 3'u
  MODE6* {.bitsize:2.}: 0'u .. 3'u
  CNF6* {.bitsize:2.}: 0'u .. 3'u
  MODE7* {.bitsize:2.}: 0'u .. 3'u
  CNF7* {.bitsize:2.}: 0'u .. 3'u

type GPIOA_CRH_Fields* = object
  MODE8* {.bitsize:2.}: 0'u .. 3'u
  CNF8* {.bitsize:2.}: 0'u .. 3'u
  MODE9* {.bitsize:2.}: 0'u .. 3'u
  CNF9* {.bitsize:2.}: 0'u .. 3'u
  MODE10* {.bitsize:2.}: 0'u .. 3'u
  CNF10* {.bitsize:2.}: 0'u .. 3'u
  MODE11* {.bitsize:2.}: 0'u .. 3'u
  CNF11* {.bitsize:2.}: 0'u .. 3'u
  MODE12* {.bitsize:2.}: 0'u .. 3'u
  CNF12* {.bitsize:2.}: 0'u .. 3'u
  MODE13* {.bitsize:2.}: 0'u .. 3'u
  CNF13* {.bitsize:2.}: 0'u .. 3'u
  MODE14* {.bitsize:2.}: 0'u .. 3'u
  CNF14* {.bitsize:2.}: 0'u .. 3'u
  MODE15* {.bitsize:2.}: 0'u .. 3'u
  CNF15* {.bitsize:2.}: 0'u .. 3'u

type GPIOA_IDR_Fields* = object
  IDR0* {.bitsize:1.}: bool
  IDR1* {.bitsize:1.}: bool
  IDR2* {.bitsize:1.}: bool
  IDR3* {.bitsize:1.}: bool
  IDR4* {.bitsize:1.}: bool
  IDR5* {.bitsize:1.}: bool
  IDR6* {.bitsize:1.}: bool
  IDR7* {.bitsize:1.}: bool
  IDR8* {.bitsize:1.}: bool
  IDR9* {.bitsize:1.}: bool
  IDR10* {.bitsize:1.}: bool
  IDR11* {.bitsize:1.}: bool
  IDR12* {.bitsize:1.}: bool
  IDR13* {.bitsize:1.}: bool
  IDR14* {.bitsize:1.}: bool
  IDR15* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type GPIOA_ODR_Fields* = object
  ODR0* {.bitsize:1.}: bool
  ODR1* {.bitsize:1.}: bool
  ODR2* {.bitsize:1.}: bool
  ODR3* {.bitsize:1.}: bool
  ODR4* {.bitsize:1.}: bool
  ODR5* {.bitsize:1.}: bool
  ODR6* {.bitsize:1.}: bool
  ODR7* {.bitsize:1.}: bool
  ODR8* {.bitsize:1.}: bool
  ODR9* {.bitsize:1.}: bool
  ODR10* {.bitsize:1.}: bool
  ODR11* {.bitsize:1.}: bool
  ODR12* {.bitsize:1.}: bool
  ODR13* {.bitsize:1.}: bool
  ODR14* {.bitsize:1.}: bool
  ODR15* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type GPIOA_BSRR_Fields* = object
  BS0* {.bitsize:1.}: bool
  BS1* {.bitsize:1.}: bool
  BS2* {.bitsize:1.}: bool
  BS3* {.bitsize:1.}: bool
  BS4* {.bitsize:1.}: bool
  BS5* {.bitsize:1.}: bool
  BS6* {.bitsize:1.}: bool
  BS7* {.bitsize:1.}: bool
  BS8* {.bitsize:1.}: bool
  BS9* {.bitsize:1.}: bool
  BS10* {.bitsize:1.}: bool
  BS11* {.bitsize:1.}: bool
  BS12* {.bitsize:1.}: bool
  BS13* {.bitsize:1.}: bool
  BS14* {.bitsize:1.}: bool
  BS15* {.bitsize:1.}: bool
  BR0* {.bitsize:1.}: bool
  BR1* {.bitsize:1.}: bool
  BR2* {.bitsize:1.}: bool
  BR3* {.bitsize:1.}: bool
  BR4* {.bitsize:1.}: bool
  BR5* {.bitsize:1.}: bool
  BR6* {.bitsize:1.}: bool
  BR7* {.bitsize:1.}: bool
  BR8* {.bitsize:1.}: bool
  BR9* {.bitsize:1.}: bool
  BR10* {.bitsize:1.}: bool
  BR11* {.bitsize:1.}: bool
  BR12* {.bitsize:1.}: bool
  BR13* {.bitsize:1.}: bool
  BR14* {.bitsize:1.}: bool
  BR15* {.bitsize:1.}: bool

type GPIOA_BRR_Fields* = object
  BR0* {.bitsize:1.}: bool
  BR1* {.bitsize:1.}: bool
  BR2* {.bitsize:1.}: bool
  BR3* {.bitsize:1.}: bool
  BR4* {.bitsize:1.}: bool
  BR5* {.bitsize:1.}: bool
  BR6* {.bitsize:1.}: bool
  BR7* {.bitsize:1.}: bool
  BR8* {.bitsize:1.}: bool
  BR9* {.bitsize:1.}: bool
  BR10* {.bitsize:1.}: bool
  BR11* {.bitsize:1.}: bool
  BR12* {.bitsize:1.}: bool
  BR13* {.bitsize:1.}: bool
  BR14* {.bitsize:1.}: bool
  BR15* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type GPIOA_LCKR_Fields* = object
  LCK0* {.bitsize:1.}: bool
  LCK1* {.bitsize:1.}: bool
  LCK2* {.bitsize:1.}: bool
  LCK3* {.bitsize:1.}: bool
  LCK4* {.bitsize:1.}: bool
  LCK5* {.bitsize:1.}: bool
  LCK6* {.bitsize:1.}: bool
  LCK7* {.bitsize:1.}: bool
  LCK8* {.bitsize:1.}: bool
  LCK9* {.bitsize:1.}: bool
  LCK10* {.bitsize:1.}: bool
  LCK11* {.bitsize:1.}: bool
  LCK12* {.bitsize:1.}: bool
  LCK13* {.bitsize:1.}: bool
  LCK14* {.bitsize:1.}: bool
  LCK15* {.bitsize:1.}: bool
  LCKK* {.bitsize:1.}: bool
  RESERVED {.bitsize:15.}: 0'u .. 32767'u

template read*(reg: GPIOA_CRL_Type): GPIOA_CRL_Fields =
  cast[GPIOA_CRL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: GPIOA_CRL_Type, val: GPIOA_CRL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: GPIOA_CRL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: GPIOA_CRH_Type): GPIOA_CRH_Fields =
  cast[GPIOA_CRH_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: GPIOA_CRH_Type, val: GPIOA_CRH_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: GPIOA_CRH_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: GPIOA_IDR_Type): GPIOA_IDR_Fields =
  cast[GPIOA_IDR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: GPIOA_ODR_Type): GPIOA_ODR_Fields =
  cast[GPIOA_ODR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: GPIOA_ODR_Type, val: GPIOA_ODR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: GPIOA_ODR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: GPIOA_BSRR_Type, val: GPIOA_BSRR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template write*(reg: GPIOA_BRR_Type, val: GPIOA_BRR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: GPIOA_LCKR_Type): GPIOA_LCKR_Fields =
  cast[GPIOA_LCKR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: GPIOA_LCKR_Type, val: GPIOA_LCKR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: GPIOA_LCKR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type AFIO_EVCR_Fields* = object
  PIN* {.bitsize:4.}: 0'u .. 15'u
  PORT* {.bitsize:3.}: 0'u .. 7'u
  EVOE* {.bitsize:1.}: bool
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type AFIO_MAPR_Fields* = object
  SPI1_REMAP* {.bitsize:1.}: bool
  I2C1_REMAP* {.bitsize:1.}: bool
  USART1_REMAP* {.bitsize:1.}: bool
  USART2_REMAP* {.bitsize:1.}: bool
  USART3_REMAP* {.bitsize:2.}: 0'u .. 3'u
  TIM1_REMAP* {.bitsize:2.}: 0'u .. 3'u
  TIM2_REMAP* {.bitsize:2.}: 0'u .. 3'u
  TIM3_REMAP* {.bitsize:2.}: 0'u .. 3'u
  TIM4_REMAP* {.bitsize:1.}: bool
  CAN_REMAP* {.bitsize:2.}: 0'u .. 3'u
  PD01_REMAP* {.bitsize:1.}: bool
  TIM5CH4_IREMAP* {.bitsize:1.}: bool
  ADC1_ETRGINJ_REMAP* {.bitsize:1.}: bool
  ADC1_ETRGREG_REMAP* {.bitsize:1.}: bool
  ADC2_ETRGINJ_REMAP* {.bitsize:1.}: bool
  ADC2_ETRGREG_REMAP* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  SWJ_CFG* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u

type AFIO_EXTICR1_Fields* = object
  EXTI0* {.bitsize:4.}: 0'u .. 15'u
  EXTI1* {.bitsize:4.}: 0'u .. 15'u
  EXTI2* {.bitsize:4.}: 0'u .. 15'u
  EXTI3* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type AFIO_EXTICR2_Fields* = object
  EXTI4* {.bitsize:4.}: 0'u .. 15'u
  EXTI5* {.bitsize:4.}: 0'u .. 15'u
  EXTI6* {.bitsize:4.}: 0'u .. 15'u
  EXTI7* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type AFIO_EXTICR3_Fields* = object
  EXTI8* {.bitsize:4.}: 0'u .. 15'u
  EXTI9* {.bitsize:4.}: 0'u .. 15'u
  EXTI10* {.bitsize:4.}: 0'u .. 15'u
  EXTI11* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type AFIO_EXTICR4_Fields* = object
  EXTI12* {.bitsize:4.}: 0'u .. 15'u
  EXTI13* {.bitsize:4.}: 0'u .. 15'u
  EXTI14* {.bitsize:4.}: 0'u .. 15'u
  EXTI15* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type AFIO_MAPR2_Fields* = object
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  TIM9_REMAP* {.bitsize:1.}: bool
  TIM10_REMAP* {.bitsize:1.}: bool
  TIM11_REMAP* {.bitsize:1.}: bool
  TIM13_REMAP* {.bitsize:1.}: bool
  TIM14_REMAP* {.bitsize:1.}: bool
  FSMC_NADV* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

template read*(reg: AFIO_EVCR_Type): AFIO_EVCR_Fields =
  cast[AFIO_EVCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_EVCR_Type, val: AFIO_EVCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_EVCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: AFIO_MAPR_Type): AFIO_MAPR_Fields =
  cast[AFIO_MAPR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_MAPR_Type, val: AFIO_MAPR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_MAPR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: AFIO_EXTICR1_Type): AFIO_EXTICR1_Fields =
  cast[AFIO_EXTICR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_EXTICR1_Type, val: AFIO_EXTICR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_EXTICR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: AFIO_EXTICR2_Type): AFIO_EXTICR2_Fields =
  cast[AFIO_EXTICR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_EXTICR2_Type, val: AFIO_EXTICR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_EXTICR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: AFIO_EXTICR3_Type): AFIO_EXTICR3_Fields =
  cast[AFIO_EXTICR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_EXTICR3_Type, val: AFIO_EXTICR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_EXTICR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: AFIO_EXTICR4_Type): AFIO_EXTICR4_Fields =
  cast[AFIO_EXTICR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_EXTICR4_Type, val: AFIO_EXTICR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_EXTICR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: AFIO_MAPR2_Type): AFIO_MAPR2_Fields =
  cast[AFIO_MAPR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: AFIO_MAPR2_Type, val: AFIO_MAPR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: AFIO_MAPR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type EXTI_IMR_Fields* = object
  MR0* {.bitsize:1.}: bool
  MR1* {.bitsize:1.}: bool
  MR2* {.bitsize:1.}: bool
  MR3* {.bitsize:1.}: bool
  MR4* {.bitsize:1.}: bool
  MR5* {.bitsize:1.}: bool
  MR6* {.bitsize:1.}: bool
  MR7* {.bitsize:1.}: bool
  MR8* {.bitsize:1.}: bool
  MR9* {.bitsize:1.}: bool
  MR10* {.bitsize:1.}: bool
  MR11* {.bitsize:1.}: bool
  MR12* {.bitsize:1.}: bool
  MR13* {.bitsize:1.}: bool
  MR14* {.bitsize:1.}: bool
  MR15* {.bitsize:1.}: bool
  MR16* {.bitsize:1.}: bool
  MR17* {.bitsize:1.}: bool
  MR18* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u

type EXTI_EMR_Fields* = object
  MR0* {.bitsize:1.}: bool
  MR1* {.bitsize:1.}: bool
  MR2* {.bitsize:1.}: bool
  MR3* {.bitsize:1.}: bool
  MR4* {.bitsize:1.}: bool
  MR5* {.bitsize:1.}: bool
  MR6* {.bitsize:1.}: bool
  MR7* {.bitsize:1.}: bool
  MR8* {.bitsize:1.}: bool
  MR9* {.bitsize:1.}: bool
  MR10* {.bitsize:1.}: bool
  MR11* {.bitsize:1.}: bool
  MR12* {.bitsize:1.}: bool
  MR13* {.bitsize:1.}: bool
  MR14* {.bitsize:1.}: bool
  MR15* {.bitsize:1.}: bool
  MR16* {.bitsize:1.}: bool
  MR17* {.bitsize:1.}: bool
  MR18* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u

type EXTI_RTSR_Fields* = object
  TR0* {.bitsize:1.}: bool
  TR1* {.bitsize:1.}: bool
  TR2* {.bitsize:1.}: bool
  TR3* {.bitsize:1.}: bool
  TR4* {.bitsize:1.}: bool
  TR5* {.bitsize:1.}: bool
  TR6* {.bitsize:1.}: bool
  TR7* {.bitsize:1.}: bool
  TR8* {.bitsize:1.}: bool
  TR9* {.bitsize:1.}: bool
  TR10* {.bitsize:1.}: bool
  TR11* {.bitsize:1.}: bool
  TR12* {.bitsize:1.}: bool
  TR13* {.bitsize:1.}: bool
  TR14* {.bitsize:1.}: bool
  TR15* {.bitsize:1.}: bool
  TR16* {.bitsize:1.}: bool
  TR17* {.bitsize:1.}: bool
  TR18* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u

type EXTI_FTSR_Fields* = object
  TR0* {.bitsize:1.}: bool
  TR1* {.bitsize:1.}: bool
  TR2* {.bitsize:1.}: bool
  TR3* {.bitsize:1.}: bool
  TR4* {.bitsize:1.}: bool
  TR5* {.bitsize:1.}: bool
  TR6* {.bitsize:1.}: bool
  TR7* {.bitsize:1.}: bool
  TR8* {.bitsize:1.}: bool
  TR9* {.bitsize:1.}: bool
  TR10* {.bitsize:1.}: bool
  TR11* {.bitsize:1.}: bool
  TR12* {.bitsize:1.}: bool
  TR13* {.bitsize:1.}: bool
  TR14* {.bitsize:1.}: bool
  TR15* {.bitsize:1.}: bool
  TR16* {.bitsize:1.}: bool
  TR17* {.bitsize:1.}: bool
  TR18* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u

type EXTI_SWIER_Fields* = object
  SWIER0* {.bitsize:1.}: bool
  SWIER1* {.bitsize:1.}: bool
  SWIER2* {.bitsize:1.}: bool
  SWIER3* {.bitsize:1.}: bool
  SWIER4* {.bitsize:1.}: bool
  SWIER5* {.bitsize:1.}: bool
  SWIER6* {.bitsize:1.}: bool
  SWIER7* {.bitsize:1.}: bool
  SWIER8* {.bitsize:1.}: bool
  SWIER9* {.bitsize:1.}: bool
  SWIER10* {.bitsize:1.}: bool
  SWIER11* {.bitsize:1.}: bool
  SWIER12* {.bitsize:1.}: bool
  SWIER13* {.bitsize:1.}: bool
  SWIER14* {.bitsize:1.}: bool
  SWIER15* {.bitsize:1.}: bool
  SWIER16* {.bitsize:1.}: bool
  SWIER17* {.bitsize:1.}: bool
  SWIER18* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u

type EXTI_PR_Fields* = object
  PR0* {.bitsize:1.}: bool
  PR1* {.bitsize:1.}: bool
  PR2* {.bitsize:1.}: bool
  PR3* {.bitsize:1.}: bool
  PR4* {.bitsize:1.}: bool
  PR5* {.bitsize:1.}: bool
  PR6* {.bitsize:1.}: bool
  PR7* {.bitsize:1.}: bool
  PR8* {.bitsize:1.}: bool
  PR9* {.bitsize:1.}: bool
  PR10* {.bitsize:1.}: bool
  PR11* {.bitsize:1.}: bool
  PR12* {.bitsize:1.}: bool
  PR13* {.bitsize:1.}: bool
  PR14* {.bitsize:1.}: bool
  PR15* {.bitsize:1.}: bool
  PR16* {.bitsize:1.}: bool
  PR17* {.bitsize:1.}: bool
  PR18* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u

template read*(reg: EXTI_IMR_Type): EXTI_IMR_Fields =
  cast[EXTI_IMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: EXTI_IMR_Type, val: EXTI_IMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: EXTI_IMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: EXTI_EMR_Type): EXTI_EMR_Fields =
  cast[EXTI_EMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: EXTI_EMR_Type, val: EXTI_EMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: EXTI_EMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: EXTI_RTSR_Type): EXTI_RTSR_Fields =
  cast[EXTI_RTSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: EXTI_RTSR_Type, val: EXTI_RTSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: EXTI_RTSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: EXTI_FTSR_Type): EXTI_FTSR_Fields =
  cast[EXTI_FTSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: EXTI_FTSR_Type, val: EXTI_FTSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: EXTI_FTSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: EXTI_SWIER_Type): EXTI_SWIER_Fields =
  cast[EXTI_SWIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: EXTI_SWIER_Type, val: EXTI_SWIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: EXTI_SWIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: EXTI_PR_Type): EXTI_PR_Fields =
  cast[EXTI_PR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: EXTI_PR_Type, val: EXTI_PR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: EXTI_PR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type DMA1_ISR_Fields* = object
  GIF1* {.bitsize:1.}: bool
  TCIF1* {.bitsize:1.}: bool
  HTIF1* {.bitsize:1.}: bool
  TEIF1* {.bitsize:1.}: bool
  GIF2* {.bitsize:1.}: bool
  TCIF2* {.bitsize:1.}: bool
  HTIF2* {.bitsize:1.}: bool
  TEIF2* {.bitsize:1.}: bool
  GIF3* {.bitsize:1.}: bool
  TCIF3* {.bitsize:1.}: bool
  HTIF3* {.bitsize:1.}: bool
  TEIF3* {.bitsize:1.}: bool
  GIF4* {.bitsize:1.}: bool
  TCIF4* {.bitsize:1.}: bool
  HTIF4* {.bitsize:1.}: bool
  TEIF4* {.bitsize:1.}: bool
  GIF5* {.bitsize:1.}: bool
  TCIF5* {.bitsize:1.}: bool
  HTIF5* {.bitsize:1.}: bool
  TEIF5* {.bitsize:1.}: bool
  GIF6* {.bitsize:1.}: bool
  TCIF6* {.bitsize:1.}: bool
  HTIF6* {.bitsize:1.}: bool
  TEIF6* {.bitsize:1.}: bool
  GIF7* {.bitsize:1.}: bool
  TCIF7* {.bitsize:1.}: bool
  HTIF7* {.bitsize:1.}: bool
  TEIF7* {.bitsize:1.}: bool
  RESERVED {.bitsize:4.}: 0'u .. 15'u

type DMA1_IFCR_Fields* = object
  CGIF1* {.bitsize:1.}: bool
  CTCIF1* {.bitsize:1.}: bool
  CHTIF1* {.bitsize:1.}: bool
  CTEIF1* {.bitsize:1.}: bool
  CGIF2* {.bitsize:1.}: bool
  CTCIF2* {.bitsize:1.}: bool
  CHTIF2* {.bitsize:1.}: bool
  CTEIF2* {.bitsize:1.}: bool
  CGIF3* {.bitsize:1.}: bool
  CTCIF3* {.bitsize:1.}: bool
  CHTIF3* {.bitsize:1.}: bool
  CTEIF3* {.bitsize:1.}: bool
  CGIF4* {.bitsize:1.}: bool
  CTCIF4* {.bitsize:1.}: bool
  CHTIF4* {.bitsize:1.}: bool
  CTEIF4* {.bitsize:1.}: bool
  CGIF5* {.bitsize:1.}: bool
  CTCIF5* {.bitsize:1.}: bool
  CHTIF5* {.bitsize:1.}: bool
  CTEIF5* {.bitsize:1.}: bool
  CGIF6* {.bitsize:1.}: bool
  CTCIF6* {.bitsize:1.}: bool
  CHTIF6* {.bitsize:1.}: bool
  CTEIF6* {.bitsize:1.}: bool
  CGIF7* {.bitsize:1.}: bool
  CTCIF7* {.bitsize:1.}: bool
  CHTIF7* {.bitsize:1.}: bool
  CTEIF7* {.bitsize:1.}: bool
  RESERVED {.bitsize:4.}: 0'u .. 15'u

type DMA1_CCR1_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR1_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DMA1_CCR2_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR2_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DMA1_CCR3_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR3_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DMA1_CCR4_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR4_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DMA1_CCR5_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR5_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DMA1_CCR6_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR6_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DMA1_CCR7_Fields* = object
  EN* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  HTIE* {.bitsize:1.}: bool
  TEIE* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CIRC* {.bitsize:1.}: bool
  PINC* {.bitsize:1.}: bool
  MINC* {.bitsize:1.}: bool
  PSIZE* {.bitsize:2.}: 0'u .. 3'u
  MSIZE* {.bitsize:2.}: 0'u .. 3'u
  PL* {.bitsize:2.}: 0'u .. 3'u
  MEM2MEM* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type DMA1_CNDTR7_Fields* = object
  NDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: DMA1_ISR_Type): DMA1_ISR_Fields =
  cast[DMA1_ISR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_IFCR_Type, val: DMA1_IFCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: DMA1_CCR1_Type): DMA1_CCR1_Fields =
  cast[DMA1_CCR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR1_Type, val: DMA1_CCR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR1_Type): DMA1_CNDTR1_Fields =
  cast[DMA1_CNDTR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR1_Type, val: DMA1_CNDTR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR1_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR1_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CCR2_Type): DMA1_CCR2_Fields =
  cast[DMA1_CCR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR2_Type, val: DMA1_CCR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR2_Type): DMA1_CNDTR2_Fields =
  cast[DMA1_CNDTR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR2_Type, val: DMA1_CNDTR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR2_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR2_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR2_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR2_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CCR3_Type): DMA1_CCR3_Fields =
  cast[DMA1_CCR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR3_Type, val: DMA1_CCR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR3_Type): DMA1_CNDTR3_Fields =
  cast[DMA1_CNDTR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR3_Type, val: DMA1_CNDTR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR3_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR3_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR3_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR3_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CCR4_Type): DMA1_CCR4_Fields =
  cast[DMA1_CCR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR4_Type, val: DMA1_CCR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR4_Type): DMA1_CNDTR4_Fields =
  cast[DMA1_CNDTR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR4_Type, val: DMA1_CNDTR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR4_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR4_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR4_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR4_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CCR5_Type): DMA1_CCR5_Fields =
  cast[DMA1_CCR5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR5_Type, val: DMA1_CCR5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR5_Type): DMA1_CNDTR5_Fields =
  cast[DMA1_CNDTR5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR5_Type, val: DMA1_CNDTR5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR5_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR5_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR5_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR5_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CCR6_Type): DMA1_CCR6_Fields =
  cast[DMA1_CCR6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR6_Type, val: DMA1_CCR6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR6_Type): DMA1_CNDTR6_Fields =
  cast[DMA1_CNDTR6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR6_Type, val: DMA1_CNDTR6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR6_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR6_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR6_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR6_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CCR7_Type): DMA1_CCR7_Fields =
  cast[DMA1_CCR7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CCR7_Type, val: DMA1_CCR7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CCR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CNDTR7_Type): DMA1_CNDTR7_Fields =
  cast[DMA1_CNDTR7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DMA1_CNDTR7_Type, val: DMA1_CNDTR7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DMA1_CNDTR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CPAR7_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CPAR7_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CPAR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DMA1_CMAR7_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: DMA1_CMAR7_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_CMAR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type SDIO_POWER_Fields* = object
  PWRCTRL* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:30.}: 0'u .. 1073741823'u

type SDIO_CLKCR_Fields* = object
  CLKDIV* {.bitsize:8.}: 0'u .. 255'u
  CLKEN* {.bitsize:1.}: bool
  PWRSAV* {.bitsize:1.}: bool
  BYPASS* {.bitsize:1.}: bool
  WIDBUS* {.bitsize:2.}: 0'u .. 3'u
  NEGEDGE* {.bitsize:1.}: bool
  HWFC_EN* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type SDIO_CMD_Fields* = object
  CMDINDEX* {.bitsize:6.}: 0'u .. 63'u
  WAITRESP* {.bitsize:2.}: 0'u .. 3'u
  WAITINT* {.bitsize:1.}: bool
  WAITPEND* {.bitsize:1.}: bool
  CPSMEN* {.bitsize:1.}: bool
  SDIOSuspend* {.bitsize:1.}: bool
  ENCMDcompl* {.bitsize:1.}: bool
  nIEN* {.bitsize:1.}: bool
  CE_ATACMD* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type SDIO_RESPCMD_Fields* = object
  RESPCMD* {.bitsize:6.}: 0'u .. 63'u
  RESERVED {.bitsize:26.}: 0'u .. 67108863'u

type SDIO_DLEN_Fields* = object
  DATALENGTH* {.bitsize:25.}: 0'u .. 33554431'u
  RESERVED {.bitsize:7.}: 0'u .. 127'u

type SDIO_DCTRL_Fields* = object
  DTEN* {.bitsize:1.}: bool
  DTDIR* {.bitsize:1.}: bool
  DTMODE* {.bitsize:1.}: bool
  DMAEN* {.bitsize:1.}: bool
  DBLOCKSIZE* {.bitsize:4.}: 0'u .. 15'u
  PWSTART* {.bitsize:1.}: bool
  PWSTOP* {.bitsize:1.}: bool
  RWMOD* {.bitsize:1.}: bool
  SDIOEN* {.bitsize:1.}: bool
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type SDIO_DCOUNT_Fields* = object
  DATACOUNT* {.bitsize:25.}: 0'u .. 33554431'u
  RESERVED {.bitsize:7.}: 0'u .. 127'u

type SDIO_STA_Fields* = object
  CCRCFAIL* {.bitsize:1.}: bool
  DCRCFAIL* {.bitsize:1.}: bool
  CTIMEOUT* {.bitsize:1.}: bool
  DTIMEOUT* {.bitsize:1.}: bool
  TXUNDERR* {.bitsize:1.}: bool
  RXOVERR* {.bitsize:1.}: bool
  CMDREND* {.bitsize:1.}: bool
  CMDSENT* {.bitsize:1.}: bool
  DATAEND* {.bitsize:1.}: bool
  STBITERR* {.bitsize:1.}: bool
  DBCKEND* {.bitsize:1.}: bool
  CMDACT* {.bitsize:1.}: bool
  TXACT* {.bitsize:1.}: bool
  RXACT* {.bitsize:1.}: bool
  TXFIFOHE* {.bitsize:1.}: bool
  RXFIFOHF* {.bitsize:1.}: bool
  TXFIFOF* {.bitsize:1.}: bool
  RXFIFOF* {.bitsize:1.}: bool
  TXFIFOE* {.bitsize:1.}: bool
  RXFIFOE* {.bitsize:1.}: bool
  TXDAVL* {.bitsize:1.}: bool
  RXDAVL* {.bitsize:1.}: bool
  SDIOIT* {.bitsize:1.}: bool
  CEATAEND* {.bitsize:1.}: bool
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type SDIO_ICR_Fields* = object
  CCRCFAILC* {.bitsize:1.}: bool
  DCRCFAILC* {.bitsize:1.}: bool
  CTIMEOUTC* {.bitsize:1.}: bool
  DTIMEOUTC* {.bitsize:1.}: bool
  TXUNDERRC* {.bitsize:1.}: bool
  RXOVERRC* {.bitsize:1.}: bool
  CMDRENDC* {.bitsize:1.}: bool
  CMDSENTC* {.bitsize:1.}: bool
  DATAENDC* {.bitsize:1.}: bool
  STBITERRC* {.bitsize:1.}: bool
  DBCKENDC* {.bitsize:1.}: bool
  RESERVED {.bitsize:11.}: 0'u .. 2047'u
  SDIOITC* {.bitsize:1.}: bool
  CEATAENDC* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:8.}: 0'u .. 255'u

type SDIO_MASK_Fields* = object
  CCRCFAILIE* {.bitsize:1.}: bool
  DCRCFAILIE* {.bitsize:1.}: bool
  CTIMEOUTIE* {.bitsize:1.}: bool
  DTIMEOUTIE* {.bitsize:1.}: bool
  TXUNDERRIE* {.bitsize:1.}: bool
  RXOVERRIE* {.bitsize:1.}: bool
  CMDRENDIE* {.bitsize:1.}: bool
  CMDSENTIE* {.bitsize:1.}: bool
  DATAENDIE* {.bitsize:1.}: bool
  STBITERRIE* {.bitsize:1.}: bool
  DBACKENDIE* {.bitsize:1.}: bool
  CMDACTIE* {.bitsize:1.}: bool
  TXACTIE* {.bitsize:1.}: bool
  RXACTIE* {.bitsize:1.}: bool
  TXFIFOHEIE* {.bitsize:1.}: bool
  RXFIFOHFIE* {.bitsize:1.}: bool
  TXFIFOFIE* {.bitsize:1.}: bool
  RXFIFOFIE* {.bitsize:1.}: bool
  TXFIFOEIE* {.bitsize:1.}: bool
  RXFIFOEIE* {.bitsize:1.}: bool
  TXDAVLIE* {.bitsize:1.}: bool
  RXDAVLIE* {.bitsize:1.}: bool
  SDIOITIE* {.bitsize:1.}: bool
  CEATENDIE* {.bitsize:1.}: bool
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type SDIO_FIFOCNT_Fields* = object
  FIF0COUNT* {.bitsize:24.}: 0'u .. 16777215'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

template read*(reg: SDIO_POWER_Type): SDIO_POWER_Fields =
  cast[SDIO_POWER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_POWER_Type, val: SDIO_POWER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_POWER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_CLKCR_Type): SDIO_CLKCR_Fields =
  cast[SDIO_CLKCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_CLKCR_Type, val: SDIO_CLKCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_CLKCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_ARG_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: SDIO_ARG_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: SDIO_ARG_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_CMD_Type): SDIO_CMD_Fields =
  cast[SDIO_CMD_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_CMD_Type, val: SDIO_CMD_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_CMD_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_RESPCMD_Type): SDIO_RESPCMD_Fields =
  cast[SDIO_RESPCMD_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SDIO_RESPI1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: SDIO_RESP2_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: SDIO_RESP3_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: SDIO_RESP4_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: SDIO_DTIMER_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: SDIO_DTIMER_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: SDIO_DTIMER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_DLEN_Type): SDIO_DLEN_Fields =
  cast[SDIO_DLEN_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_DLEN_Type, val: SDIO_DLEN_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_DLEN_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_DCTRL_Type): SDIO_DCTRL_Fields =
  cast[SDIO_DCTRL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_DCTRL_Type, val: SDIO_DCTRL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_DCTRL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_DCOUNT_Type): SDIO_DCOUNT_Fields =
  cast[SDIO_DCOUNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SDIO_STA_Type): SDIO_STA_Fields =
  cast[SDIO_STA_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SDIO_ICR_Type): SDIO_ICR_Fields =
  cast[SDIO_ICR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_ICR_Type, val: SDIO_ICR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_ICR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_MASK_Type): SDIO_MASK_Fields =
  cast[SDIO_MASK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SDIO_MASK_Type, val: SDIO_MASK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SDIO_MASK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SDIO_FIFOCNT_Type): SDIO_FIFOCNT_Fields =
  cast[SDIO_FIFOCNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SDIO_FIFO_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: SDIO_FIFO_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: SDIO_FIFO_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type RTC_CRH_Fields* = object
  SECIE* {.bitsize:1.}: bool
  ALRIE* {.bitsize:1.}: bool
  OWIE* {.bitsize:1.}: bool
  RESERVED {.bitsize:29.}: 0'u .. 536870911'u

type RTC_CRL_Fields* = object
  SECF* {.bitsize:1.}: bool
  ALRF* {.bitsize:1.}: bool
  OWF* {.bitsize:1.}: bool
  RSF* {.bitsize:1.}: bool
  CNF* {.bitsize:1.}: bool
  RTOFF* {.bitsize:1.}: bool
  RESERVED {.bitsize:26.}: 0'u .. 67108863'u

type RTC_PRLH_Fields* = object
  PRLH* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:28.}: 0'u .. 268435455'u

type RTC_PRLL_Fields* = object
  PRLL* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type RTC_DIVH_Fields* = object
  DIVH* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:28.}: 0'u .. 268435455'u

type RTC_DIVL_Fields* = object
  DIVL* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type RTC_CNTH_Fields* = object
  CNTH* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type RTC_CNTL_Fields* = object
  CNTL* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type RTC_ALRH_Fields* = object
  ALRH* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type RTC_ALRL_Fields* = object
  ALRL* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: RTC_CRH_Type): RTC_CRH_Fields =
  cast[RTC_CRH_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RTC_CRH_Type, val: RTC_CRH_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RTC_CRH_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RTC_CRL_Type): RTC_CRL_Fields =
  cast[RTC_CRL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RTC_CRL_Type, val: RTC_CRL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RTC_CRL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: RTC_PRLH_Type, val: RTC_PRLH_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template write*(reg: RTC_PRLL_Type, val: RTC_PRLL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: RTC_DIVH_Type): RTC_DIVH_Fields =
  cast[RTC_DIVH_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: RTC_DIVL_Type): RTC_DIVL_Fields =
  cast[RTC_DIVL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: RTC_CNTH_Type): RTC_CNTH_Fields =
  cast[RTC_CNTH_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RTC_CNTH_Type, val: RTC_CNTH_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RTC_CNTH_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: RTC_CNTL_Type): RTC_CNTL_Fields =
  cast[RTC_CNTL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: RTC_CNTL_Type, val: RTC_CNTL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: RTC_CNTL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: RTC_ALRH_Type, val: RTC_ALRH_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template write*(reg: RTC_ALRL_Type, val: RTC_ALRL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

type BKP_DR1_Fields* = object
  D1* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR2_Fields* = object
  D2* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR3_Fields* = object
  D3* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR4_Fields* = object
  D4* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR5_Fields* = object
  D5* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR6_Fields* = object
  D6* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR7_Fields* = object
  D7* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR8_Fields* = object
  D8* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR9_Fields* = object
  D9* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR10_Fields* = object
  D10* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR11_Fields* = object
  DR11* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR12_Fields* = object
  DR12* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR13_Fields* = object
  DR13* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR14_Fields* = object
  D14* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR15_Fields* = object
  D15* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR16_Fields* = object
  D16* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR17_Fields* = object
  D17* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR18_Fields* = object
  D18* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR19_Fields* = object
  D19* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR20_Fields* = object
  D20* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR21_Fields* = object
  D21* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR22_Fields* = object
  D22* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR23_Fields* = object
  D23* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR24_Fields* = object
  D24* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR25_Fields* = object
  D25* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR26_Fields* = object
  D26* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR27_Fields* = object
  D27* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR28_Fields* = object
  D28* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR29_Fields* = object
  D29* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR30_Fields* = object
  D30* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR31_Fields* = object
  D31* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR32_Fields* = object
  D32* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR33_Fields* = object
  D33* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR34_Fields* = object
  D34* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR35_Fields* = object
  D35* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR36_Fields* = object
  D36* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR37_Fields* = object
  D37* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR38_Fields* = object
  D38* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR39_Fields* = object
  D39* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR40_Fields* = object
  D40* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR41_Fields* = object
  D41* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_DR42_Fields* = object
  D42* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type BKP_RTCCR_Fields* = object
  CAL* {.bitsize:7.}: 0'u .. 127'u
  CCO* {.bitsize:1.}: bool
  ASOE* {.bitsize:1.}: bool
  ASOS* {.bitsize:1.}: bool
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u

type BKP_CR_Fields* = object
  TPE* {.bitsize:1.}: bool
  TPAL* {.bitsize:1.}: bool
  RESERVED {.bitsize:30.}: 0'u .. 1073741823'u

type BKP_CSR_Fields* = object
  CTE* {.bitsize:1.}: bool
  CTI* {.bitsize:1.}: bool
  TPIE* {.bitsize:1.}: bool
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  TEF* {.bitsize:1.}: bool
  TIF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:22.}: 0'u .. 4194303'u

template read*(reg: BKP_DR1_Type): BKP_DR1_Fields =
  cast[BKP_DR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR1_Type, val: BKP_DR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR2_Type): BKP_DR2_Fields =
  cast[BKP_DR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR2_Type, val: BKP_DR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR3_Type): BKP_DR3_Fields =
  cast[BKP_DR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR3_Type, val: BKP_DR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR4_Type): BKP_DR4_Fields =
  cast[BKP_DR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR4_Type, val: BKP_DR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR5_Type): BKP_DR5_Fields =
  cast[BKP_DR5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR5_Type, val: BKP_DR5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR6_Type): BKP_DR6_Fields =
  cast[BKP_DR6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR6_Type, val: BKP_DR6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR7_Type): BKP_DR7_Fields =
  cast[BKP_DR7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR7_Type, val: BKP_DR7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR8_Type): BKP_DR8_Fields =
  cast[BKP_DR8_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR8_Type, val: BKP_DR8_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR8_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR9_Type): BKP_DR9_Fields =
  cast[BKP_DR9_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR9_Type, val: BKP_DR9_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR9_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR10_Type): BKP_DR10_Fields =
  cast[BKP_DR10_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR10_Type, val: BKP_DR10_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR10_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR11_Type): BKP_DR11_Fields =
  cast[BKP_DR11_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR11_Type, val: BKP_DR11_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR11_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR12_Type): BKP_DR12_Fields =
  cast[BKP_DR12_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR12_Type, val: BKP_DR12_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR12_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR13_Type): BKP_DR13_Fields =
  cast[BKP_DR13_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR13_Type, val: BKP_DR13_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR13_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR14_Type): BKP_DR14_Fields =
  cast[BKP_DR14_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR14_Type, val: BKP_DR14_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR14_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR15_Type): BKP_DR15_Fields =
  cast[BKP_DR15_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR15_Type, val: BKP_DR15_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR15_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR16_Type): BKP_DR16_Fields =
  cast[BKP_DR16_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR16_Type, val: BKP_DR16_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR16_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR17_Type): BKP_DR17_Fields =
  cast[BKP_DR17_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR17_Type, val: BKP_DR17_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR17_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR18_Type): BKP_DR18_Fields =
  cast[BKP_DR18_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR18_Type, val: BKP_DR18_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR18_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR19_Type): BKP_DR19_Fields =
  cast[BKP_DR19_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR19_Type, val: BKP_DR19_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR19_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR20_Type): BKP_DR20_Fields =
  cast[BKP_DR20_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR20_Type, val: BKP_DR20_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR20_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR21_Type): BKP_DR21_Fields =
  cast[BKP_DR21_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR21_Type, val: BKP_DR21_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR21_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR22_Type): BKP_DR22_Fields =
  cast[BKP_DR22_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR22_Type, val: BKP_DR22_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR22_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR23_Type): BKP_DR23_Fields =
  cast[BKP_DR23_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR23_Type, val: BKP_DR23_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR23_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR24_Type): BKP_DR24_Fields =
  cast[BKP_DR24_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR24_Type, val: BKP_DR24_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR24_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR25_Type): BKP_DR25_Fields =
  cast[BKP_DR25_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR25_Type, val: BKP_DR25_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR25_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR26_Type): BKP_DR26_Fields =
  cast[BKP_DR26_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR26_Type, val: BKP_DR26_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR26_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR27_Type): BKP_DR27_Fields =
  cast[BKP_DR27_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR27_Type, val: BKP_DR27_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR27_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR28_Type): BKP_DR28_Fields =
  cast[BKP_DR28_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR28_Type, val: BKP_DR28_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR28_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR29_Type): BKP_DR29_Fields =
  cast[BKP_DR29_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR29_Type, val: BKP_DR29_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR29_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR30_Type): BKP_DR30_Fields =
  cast[BKP_DR30_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR30_Type, val: BKP_DR30_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR30_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR31_Type): BKP_DR31_Fields =
  cast[BKP_DR31_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR31_Type, val: BKP_DR31_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR31_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR32_Type): BKP_DR32_Fields =
  cast[BKP_DR32_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR32_Type, val: BKP_DR32_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR32_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR33_Type): BKP_DR33_Fields =
  cast[BKP_DR33_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR33_Type, val: BKP_DR33_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR33_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR34_Type): BKP_DR34_Fields =
  cast[BKP_DR34_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR34_Type, val: BKP_DR34_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR34_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR35_Type): BKP_DR35_Fields =
  cast[BKP_DR35_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR35_Type, val: BKP_DR35_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR35_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR36_Type): BKP_DR36_Fields =
  cast[BKP_DR36_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR36_Type, val: BKP_DR36_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR36_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR37_Type): BKP_DR37_Fields =
  cast[BKP_DR37_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR37_Type, val: BKP_DR37_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR37_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR38_Type): BKP_DR38_Fields =
  cast[BKP_DR38_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR38_Type, val: BKP_DR38_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR38_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR39_Type): BKP_DR39_Fields =
  cast[BKP_DR39_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR39_Type, val: BKP_DR39_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR39_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR40_Type): BKP_DR40_Fields =
  cast[BKP_DR40_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR40_Type, val: BKP_DR40_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR40_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR41_Type): BKP_DR41_Fields =
  cast[BKP_DR41_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR41_Type, val: BKP_DR41_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR41_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_DR42_Type): BKP_DR42_Fields =
  cast[BKP_DR42_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_DR42_Type, val: BKP_DR42_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_DR42_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_RTCCR_Type): BKP_RTCCR_Fields =
  cast[BKP_RTCCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_RTCCR_Type, val: BKP_RTCCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_RTCCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_CR_Type): BKP_CR_Fields =
  cast[BKP_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_CR_Type, val: BKP_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: BKP_CSR_Type): BKP_CSR_Fields =
  cast[BKP_CSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: BKP_CSR_Type, val: BKP_CSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: BKP_CSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type IWDG_KR_Fields* = object
  KEY* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type IWDG_PR_Fields* = object
  PR* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:29.}: 0'u .. 536870911'u

type IWDG_RLR_Fields* = object
  RL* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type IWDG_SR_Fields* = object
  PVU* {.bitsize:1.}: bool
  RVU* {.bitsize:1.}: bool
  RESERVED {.bitsize:30.}: 0'u .. 1073741823'u

template write*(reg: IWDG_KR_Type, val: IWDG_KR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: IWDG_PR_Type): IWDG_PR_Fields =
  cast[IWDG_PR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: IWDG_PR_Type, val: IWDG_PR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: IWDG_PR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: IWDG_RLR_Type): IWDG_RLR_Fields =
  cast[IWDG_RLR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: IWDG_RLR_Type, val: IWDG_RLR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: IWDG_RLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: IWDG_SR_Type): IWDG_SR_Fields =
  cast[IWDG_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

type WWDG_CR_Fields* = object
  T* {.bitsize:7.}: 0'u .. 127'u
  WDGA* {.bitsize:1.}: bool
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type WWDG_CFR_Fields* = object
  W* {.bitsize:7.}: 0'u .. 127'u
  WDGTB* {.bitsize:2.}: 0'u .. 3'u
  EWI* {.bitsize:1.}: bool
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u

type WWDG_SR_Fields* = object
  EWI* {.bitsize:1.}: bool
  RESERVED {.bitsize:31.}: 0'u .. 2147483647'u

template read*(reg: WWDG_CR_Type): WWDG_CR_Fields =
  cast[WWDG_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: WWDG_CR_Type, val: WWDG_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: WWDG_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: WWDG_CFR_Type): WWDG_CFR_Fields =
  cast[WWDG_CFR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: WWDG_CFR_Type, val: WWDG_CFR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: WWDG_CFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: WWDG_SR_Type): WWDG_SR_Fields =
  cast[WWDG_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: WWDG_SR_Type, val: WWDG_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: WWDG_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type TIM1_CR1_Fields* = object
  CEN* {.bitsize:1.}: bool
  UDIS* {.bitsize:1.}: bool
  URS* {.bitsize:1.}: bool
  OPM* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CMS* {.bitsize:2.}: 0'u .. 3'u
  ARPE* {.bitsize:1.}: bool
  CKD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u

type TIM1_CR2_Fields* = object
  CCPC* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  CCUS* {.bitsize:1.}: bool
  CCDS* {.bitsize:1.}: bool
  MMS* {.bitsize:3.}: 0'u .. 7'u
  TI1S* {.bitsize:1.}: bool
  OIS1* {.bitsize:1.}: bool
  OIS1N* {.bitsize:1.}: bool
  OIS2* {.bitsize:1.}: bool
  OIS2N* {.bitsize:1.}: bool
  OIS3* {.bitsize:1.}: bool
  OIS3N* {.bitsize:1.}: bool
  OIS4* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:17.}: 0'u .. 131071'u

type TIM1_SMCR_Fields* = object
  SMS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:1.}: bool
  TS* {.bitsize:3.}: 0'u .. 7'u
  MSM* {.bitsize:1.}: bool
  ETF* {.bitsize:4.}: 0'u .. 15'u
  ETPS* {.bitsize:2.}: 0'u .. 3'u
  ECE* {.bitsize:1.}: bool
  ETP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type TIM1_DIER_Fields* = object
  UIE* {.bitsize:1.}: bool
  CC1IE* {.bitsize:1.}: bool
  CC2IE* {.bitsize:1.}: bool
  CC3IE* {.bitsize:1.}: bool
  CC4IE* {.bitsize:1.}: bool
  COMIE* {.bitsize:1.}: bool
  TIE* {.bitsize:1.}: bool
  BIE* {.bitsize:1.}: bool
  UDE* {.bitsize:1.}: bool
  CC1DE* {.bitsize:1.}: bool
  CC2DE* {.bitsize:1.}: bool
  CC3DE* {.bitsize:1.}: bool
  CC4DE* {.bitsize:1.}: bool
  COMDE* {.bitsize:1.}: bool
  TDE* {.bitsize:1.}: bool
  RESERVED {.bitsize:17.}: 0'u .. 131071'u

type TIM1_SR_Fields* = object
  UIF* {.bitsize:1.}: bool
  CC1IF* {.bitsize:1.}: bool
  CC2IF* {.bitsize:1.}: bool
  CC3IF* {.bitsize:1.}: bool
  CC4IF* {.bitsize:1.}: bool
  COMIF* {.bitsize:1.}: bool
  TIF* {.bitsize:1.}: bool
  BIF* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  CC1OF* {.bitsize:1.}: bool
  CC2OF* {.bitsize:1.}: bool
  CC3OF* {.bitsize:1.}: bool
  CC4OF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:19.}: 0'u .. 524287'u

type TIM1_EGR_Fields* = object
  UG* {.bitsize:1.}: bool
  CC1G* {.bitsize:1.}: bool
  CC2G* {.bitsize:1.}: bool
  CC3G* {.bitsize:1.}: bool
  CC4G* {.bitsize:1.}: bool
  COMG* {.bitsize:1.}: bool
  TG* {.bitsize:1.}: bool
  BG* {.bitsize:1.}: bool
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type TIM1_CCMR1_Output_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  OC1FE* {.bitsize:1.}: bool
  OC1PE* {.bitsize:1.}: bool
  OC1M* {.bitsize:3.}: 0'u .. 7'u
  OC1CE* {.bitsize:1.}: bool
  CC2S* {.bitsize:2.}: 0'u .. 3'u
  OC2FE* {.bitsize:1.}: bool
  OC2PE* {.bitsize:1.}: bool
  OC2M* {.bitsize:3.}: 0'u .. 7'u
  OC2CE* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCMR1_Input_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  ICPCS* {.bitsize:2.}: 0'u .. 3'u
  IC1F* {.bitsize:4.}: 0'u .. 15'u
  CC2S* {.bitsize:2.}: 0'u .. 3'u
  IC2PCS* {.bitsize:2.}: 0'u .. 3'u
  IC2F* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCMR2_Output_Fields* = object
  CC3S* {.bitsize:2.}: 0'u .. 3'u
  OC3FE* {.bitsize:1.}: bool
  OC3PE* {.bitsize:1.}: bool
  OC3M* {.bitsize:3.}: 0'u .. 7'u
  OC3CE* {.bitsize:1.}: bool
  CC4S* {.bitsize:2.}: 0'u .. 3'u
  OC4FE* {.bitsize:1.}: bool
  OC4PE* {.bitsize:1.}: bool
  OC4M* {.bitsize:3.}: 0'u .. 7'u
  OC4CE* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCMR2_Input_Fields* = object
  CC3S* {.bitsize:2.}: 0'u .. 3'u
  IC3PSC* {.bitsize:2.}: 0'u .. 3'u
  IC3F* {.bitsize:4.}: 0'u .. 15'u
  CC4S* {.bitsize:2.}: 0'u .. 3'u
  IC4PSC* {.bitsize:2.}: 0'u .. 3'u
  IC4F* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCER_Fields* = object
  CC1E* {.bitsize:1.}: bool
  CC1P* {.bitsize:1.}: bool
  CC1NE* {.bitsize:1.}: bool
  CC1NP* {.bitsize:1.}: bool
  CC2E* {.bitsize:1.}: bool
  CC2P* {.bitsize:1.}: bool
  CC2NE* {.bitsize:1.}: bool
  CC2NP* {.bitsize:1.}: bool
  CC3E* {.bitsize:1.}: bool
  CC3P* {.bitsize:1.}: bool
  CC3NE* {.bitsize:1.}: bool
  CC3NP* {.bitsize:1.}: bool
  CC4E* {.bitsize:1.}: bool
  CC4P* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type TIM1_CNT_Fields* = object
  CNT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_PSC_Fields* = object
  PSC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_ARR_Fields* = object
  ARR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCR1_Fields* = object
  CCR1* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCR2_Fields* = object
  CCR2* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCR3_Fields* = object
  CCR3* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_CCR4_Fields* = object
  CCR4* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_DCR_Fields* = object
  DBA* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  DBL* {.bitsize:5.}: 0'u .. 31'u
  RESERVED1 {.bitsize:19.}: 0'u .. 524287'u

type TIM1_DMAR_Fields* = object
  DMAB* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM1_RCR_Fields* = object
  REP* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type TIM1_BDTR_Fields* = object
  DTG* {.bitsize:8.}: 0'u .. 255'u
  LOCK* {.bitsize:2.}: 0'u .. 3'u
  OSSI* {.bitsize:1.}: bool
  OSSR* {.bitsize:1.}: bool
  BKE* {.bitsize:1.}: bool
  BKP* {.bitsize:1.}: bool
  AOE* {.bitsize:1.}: bool
  MOE* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: TIM1_CR1_Type): TIM1_CR1_Fields =
  cast[TIM1_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CR1_Type, val: TIM1_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CR2_Type): TIM1_CR2_Fields =
  cast[TIM1_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CR2_Type, val: TIM1_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_SMCR_Type): TIM1_SMCR_Fields =
  cast[TIM1_SMCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_SMCR_Type, val: TIM1_SMCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_SMCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_DIER_Type): TIM1_DIER_Fields =
  cast[TIM1_DIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_DIER_Type, val: TIM1_DIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_DIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_SR_Type): TIM1_SR_Fields =
  cast[TIM1_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_SR_Type, val: TIM1_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: TIM1_EGR_Type, val: TIM1_EGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: TIM1_CCMR1_Output_Type): TIM1_CCMR1_Output_Fields =
  cast[TIM1_CCMR1_Output_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCMR1_Output_Type, val: TIM1_CCMR1_Output_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCMR1_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCMR1_Input_Type): TIM1_CCMR1_Input_Fields =
  cast[TIM1_CCMR1_Input_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCMR1_Input_Type, val: TIM1_CCMR1_Input_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCMR1_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCMR2_Output_Type): TIM1_CCMR2_Output_Fields =
  cast[TIM1_CCMR2_Output_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCMR2_Output_Type, val: TIM1_CCMR2_Output_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCMR2_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCMR2_Input_Type): TIM1_CCMR2_Input_Fields =
  cast[TIM1_CCMR2_Input_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCMR2_Input_Type, val: TIM1_CCMR2_Input_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCMR2_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCER_Type): TIM1_CCER_Fields =
  cast[TIM1_CCER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCER_Type, val: TIM1_CCER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CNT_Type): TIM1_CNT_Fields =
  cast[TIM1_CNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CNT_Type, val: TIM1_CNT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_PSC_Type): TIM1_PSC_Fields =
  cast[TIM1_PSC_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_PSC_Type, val: TIM1_PSC_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_ARR_Type): TIM1_ARR_Fields =
  cast[TIM1_ARR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_ARR_Type, val: TIM1_ARR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_ARR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCR1_Type): TIM1_CCR1_Fields =
  cast[TIM1_CCR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCR1_Type, val: TIM1_CCR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCR2_Type): TIM1_CCR2_Fields =
  cast[TIM1_CCR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCR2_Type, val: TIM1_CCR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCR3_Type): TIM1_CCR3_Fields =
  cast[TIM1_CCR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCR3_Type, val: TIM1_CCR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_CCR4_Type): TIM1_CCR4_Fields =
  cast[TIM1_CCR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_CCR4_Type, val: TIM1_CCR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_CCR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_DCR_Type): TIM1_DCR_Fields =
  cast[TIM1_DCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_DCR_Type, val: TIM1_DCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_DCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_DMAR_Type): TIM1_DMAR_Fields =
  cast[TIM1_DMAR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_DMAR_Type, val: TIM1_DMAR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_DMAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_RCR_Type): TIM1_RCR_Fields =
  cast[TIM1_RCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_RCR_Type, val: TIM1_RCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_RCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM1_BDTR_Type): TIM1_BDTR_Fields =
  cast[TIM1_BDTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM1_BDTR_Type, val: TIM1_BDTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM1_BDTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type TIM2_CR1_Fields* = object
  CEN* {.bitsize:1.}: bool
  UDIS* {.bitsize:1.}: bool
  URS* {.bitsize:1.}: bool
  OPM* {.bitsize:1.}: bool
  DIR* {.bitsize:1.}: bool
  CMS* {.bitsize:2.}: 0'u .. 3'u
  ARPE* {.bitsize:1.}: bool
  CKD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u

type TIM2_CR2_Fields* = object
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  CCDS* {.bitsize:1.}: bool
  MMS* {.bitsize:3.}: 0'u .. 7'u
  TI1S* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:24.}: 0'u .. 16777215'u

type TIM2_SMCR_Fields* = object
  SMS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:1.}: bool
  TS* {.bitsize:3.}: 0'u .. 7'u
  MSM* {.bitsize:1.}: bool
  ETF* {.bitsize:4.}: 0'u .. 15'u
  ETPS* {.bitsize:2.}: 0'u .. 3'u
  ECE* {.bitsize:1.}: bool
  ETP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type TIM2_DIER_Fields* = object
  UIE* {.bitsize:1.}: bool
  CC1IE* {.bitsize:1.}: bool
  CC2IE* {.bitsize:1.}: bool
  CC3IE* {.bitsize:1.}: bool
  CC4IE* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  UDE* {.bitsize:1.}: bool
  CC1DE* {.bitsize:1.}: bool
  CC2DE* {.bitsize:1.}: bool
  CC3DE* {.bitsize:1.}: bool
  CC4DE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  TDE* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:17.}: 0'u .. 131071'u

type TIM2_SR_Fields* = object
  UIF* {.bitsize:1.}: bool
  CC1IF* {.bitsize:1.}: bool
  CC2IF* {.bitsize:1.}: bool
  CC3IF* {.bitsize:1.}: bool
  CC4IF* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TIF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  CC1OF* {.bitsize:1.}: bool
  CC2OF* {.bitsize:1.}: bool
  CC3OF* {.bitsize:1.}: bool
  CC4OF* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:19.}: 0'u .. 524287'u

type TIM2_EGR_Fields* = object
  UG* {.bitsize:1.}: bool
  CC1G* {.bitsize:1.}: bool
  CC2G* {.bitsize:1.}: bool
  CC3G* {.bitsize:1.}: bool
  CC4G* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TG* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM2_CCMR1_Output_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  OC1FE* {.bitsize:1.}: bool
  OC1PE* {.bitsize:1.}: bool
  OC1M* {.bitsize:3.}: 0'u .. 7'u
  OC1CE* {.bitsize:1.}: bool
  CC2S* {.bitsize:2.}: 0'u .. 3'u
  OC2FE* {.bitsize:1.}: bool
  OC2PE* {.bitsize:1.}: bool
  OC2M* {.bitsize:3.}: 0'u .. 7'u
  OC2CE* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCMR1_Input_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  IC1PSC* {.bitsize:2.}: 0'u .. 3'u
  IC1F* {.bitsize:4.}: 0'u .. 15'u
  CC2S* {.bitsize:2.}: 0'u .. 3'u
  IC2PSC* {.bitsize:2.}: 0'u .. 3'u
  IC2F* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCMR2_Output_Fields* = object
  CC3S* {.bitsize:2.}: 0'u .. 3'u
  OC3FE* {.bitsize:1.}: bool
  OC3PE* {.bitsize:1.}: bool
  OC3M* {.bitsize:3.}: 0'u .. 7'u
  OC3CE* {.bitsize:1.}: bool
  CC4S* {.bitsize:2.}: 0'u .. 3'u
  OC4FE* {.bitsize:1.}: bool
  OC4PE* {.bitsize:1.}: bool
  OC4M* {.bitsize:3.}: 0'u .. 7'u
  O24CE* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCMR2_Input_Fields* = object
  CC3S* {.bitsize:2.}: 0'u .. 3'u
  IC3PSC* {.bitsize:2.}: 0'u .. 3'u
  IC3F* {.bitsize:4.}: 0'u .. 15'u
  CC4S* {.bitsize:2.}: 0'u .. 3'u
  IC4PSC* {.bitsize:2.}: 0'u .. 3'u
  IC4F* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCER_Fields* = object
  CC1E* {.bitsize:1.}: bool
  CC1P* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  CC2E* {.bitsize:1.}: bool
  CC2P* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  CC3E* {.bitsize:1.}: bool
  CC3P* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:2.}: 0'u .. 3'u
  CC4E* {.bitsize:1.}: bool
  CC4P* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:18.}: 0'u .. 262143'u

type TIM2_CNT_Fields* = object
  CNT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_PSC_Fields* = object
  PSC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_ARR_Fields* = object
  ARR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCR1_Fields* = object
  CCR1* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCR2_Fields* = object
  CCR2* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCR3_Fields* = object
  CCR3* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_CCR4_Fields* = object
  CCR4* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM2_DCR_Fields* = object
  DBA* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  DBL* {.bitsize:5.}: 0'u .. 31'u
  RESERVED1 {.bitsize:19.}: 0'u .. 524287'u

type TIM2_DMAR_Fields* = object
  DMAB* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: TIM2_CR1_Type): TIM2_CR1_Fields =
  cast[TIM2_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CR1_Type, val: TIM2_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CR2_Type): TIM2_CR2_Fields =
  cast[TIM2_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CR2_Type, val: TIM2_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_SMCR_Type): TIM2_SMCR_Fields =
  cast[TIM2_SMCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_SMCR_Type, val: TIM2_SMCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_SMCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_DIER_Type): TIM2_DIER_Fields =
  cast[TIM2_DIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_DIER_Type, val: TIM2_DIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_DIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_SR_Type): TIM2_SR_Fields =
  cast[TIM2_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_SR_Type, val: TIM2_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: TIM2_EGR_Type, val: TIM2_EGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: TIM2_CCMR1_Output_Type): TIM2_CCMR1_Output_Fields =
  cast[TIM2_CCMR1_Output_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCMR1_Output_Type, val: TIM2_CCMR1_Output_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCMR1_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCMR1_Input_Type): TIM2_CCMR1_Input_Fields =
  cast[TIM2_CCMR1_Input_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCMR1_Input_Type, val: TIM2_CCMR1_Input_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCMR1_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCMR2_Output_Type): TIM2_CCMR2_Output_Fields =
  cast[TIM2_CCMR2_Output_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCMR2_Output_Type, val: TIM2_CCMR2_Output_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCMR2_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCMR2_Input_Type): TIM2_CCMR2_Input_Fields =
  cast[TIM2_CCMR2_Input_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCMR2_Input_Type, val: TIM2_CCMR2_Input_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCMR2_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCER_Type): TIM2_CCER_Fields =
  cast[TIM2_CCER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCER_Type, val: TIM2_CCER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CNT_Type): TIM2_CNT_Fields =
  cast[TIM2_CNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CNT_Type, val: TIM2_CNT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_PSC_Type): TIM2_PSC_Fields =
  cast[TIM2_PSC_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_PSC_Type, val: TIM2_PSC_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_ARR_Type): TIM2_ARR_Fields =
  cast[TIM2_ARR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_ARR_Type, val: TIM2_ARR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_ARR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCR1_Type): TIM2_CCR1_Fields =
  cast[TIM2_CCR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCR1_Type, val: TIM2_CCR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCR2_Type): TIM2_CCR2_Fields =
  cast[TIM2_CCR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCR2_Type, val: TIM2_CCR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCR3_Type): TIM2_CCR3_Fields =
  cast[TIM2_CCR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCR3_Type, val: TIM2_CCR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_CCR4_Type): TIM2_CCR4_Fields =
  cast[TIM2_CCR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_CCR4_Type, val: TIM2_CCR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_CCR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_DCR_Type): TIM2_DCR_Fields =
  cast[TIM2_DCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_DCR_Type, val: TIM2_DCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_DCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM2_DMAR_Type): TIM2_DMAR_Fields =
  cast[TIM2_DMAR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM2_DMAR_Type, val: TIM2_DMAR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM2_DMAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type TIM9_CR1_Fields* = object
  CEN* {.bitsize:1.}: bool
  UDIS* {.bitsize:1.}: bool
  URS* {.bitsize:1.}: bool
  OPM* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  ARPE* {.bitsize:1.}: bool
  CKD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:22.}: 0'u .. 4194303'u

type TIM9_CR2_Fields* = object
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  MMS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM9_SMCR_Fields* = object
  SMS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:1.}: bool
  TS* {.bitsize:3.}: 0'u .. 7'u
  MSM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:24.}: 0'u .. 16777215'u

type TIM9_DIER_Fields* = object
  UIE* {.bitsize:1.}: bool
  CC1IE* {.bitsize:1.}: bool
  CC2IE* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  TIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM9_SR_Fields* = object
  UIF* {.bitsize:1.}: bool
  CC1IF* {.bitsize:1.}: bool
  CC2IF* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  TIF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  CC1OF* {.bitsize:1.}: bool
  CC2OF* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type TIM9_EGR_Fields* = object
  UG* {.bitsize:1.}: bool
  CC1G* {.bitsize:1.}: bool
  CC2G* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  TG* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM9_CCMR1_Output_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  OC1FE* {.bitsize:1.}: bool
  OC1PE* {.bitsize:1.}: bool
  OC1M* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:1.}: bool
  CC2S* {.bitsize:2.}: 0'u .. 3'u
  OC2FE* {.bitsize:1.}: bool
  OC2PE* {.bitsize:1.}: bool
  OC2M* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:17.}: 0'u .. 131071'u

type TIM9_CCMR1_Input_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  IC1PSC* {.bitsize:2.}: 0'u .. 3'u
  IC1F* {.bitsize:4.}: 0'u .. 15'u
  CC2S* {.bitsize:2.}: 0'u .. 3'u
  IC2PSC* {.bitsize:2.}: 0'u .. 3'u
  IC2F* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM9_CCER_Fields* = object
  CC1E* {.bitsize:1.}: bool
  CC1P* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  CC1NP* {.bitsize:1.}: bool
  CC2E* {.bitsize:1.}: bool
  CC2P* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  CC2NP* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:24.}: 0'u .. 16777215'u

type TIM9_CNT_Fields* = object
  CNT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM9_PSC_Fields* = object
  PSC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM9_ARR_Fields* = object
  ARR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM9_CCR1_Fields* = object
  CCR1* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM9_CCR2_Fields* = object
  CCR2* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: TIM9_CR1_Type): TIM9_CR1_Fields =
  cast[TIM9_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CR1_Type, val: TIM9_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_CR2_Type): TIM9_CR2_Fields =
  cast[TIM9_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CR2_Type, val: TIM9_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_SMCR_Type): TIM9_SMCR_Fields =
  cast[TIM9_SMCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_SMCR_Type, val: TIM9_SMCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_SMCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_DIER_Type): TIM9_DIER_Fields =
  cast[TIM9_DIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_DIER_Type, val: TIM9_DIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_DIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_SR_Type): TIM9_SR_Fields =
  cast[TIM9_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_SR_Type, val: TIM9_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: TIM9_EGR_Type, val: TIM9_EGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: TIM9_CCMR1_Output_Type): TIM9_CCMR1_Output_Fields =
  cast[TIM9_CCMR1_Output_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CCMR1_Output_Type, val: TIM9_CCMR1_Output_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CCMR1_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_CCMR1_Input_Type): TIM9_CCMR1_Input_Fields =
  cast[TIM9_CCMR1_Input_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CCMR1_Input_Type, val: TIM9_CCMR1_Input_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CCMR1_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_CCER_Type): TIM9_CCER_Fields =
  cast[TIM9_CCER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CCER_Type, val: TIM9_CCER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CCER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_CNT_Type): TIM9_CNT_Fields =
  cast[TIM9_CNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CNT_Type, val: TIM9_CNT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_PSC_Type): TIM9_PSC_Fields =
  cast[TIM9_PSC_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_PSC_Type, val: TIM9_PSC_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_ARR_Type): TIM9_ARR_Fields =
  cast[TIM9_ARR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_ARR_Type, val: TIM9_ARR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_ARR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_CCR1_Type): TIM9_CCR1_Fields =
  cast[TIM9_CCR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CCR1_Type, val: TIM9_CCR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CCR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM9_CCR2_Type): TIM9_CCR2_Fields =
  cast[TIM9_CCR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM9_CCR2_Type, val: TIM9_CCR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM9_CCR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type TIM10_CR1_Fields* = object
  CEN* {.bitsize:1.}: bool
  UDIS* {.bitsize:1.}: bool
  URS* {.bitsize:1.}: bool
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  ARPE* {.bitsize:1.}: bool
  CKD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:22.}: 0'u .. 4194303'u

type TIM10_CR2_Fields* = object
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  MMS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM10_DIER_Fields* = object
  UIE* {.bitsize:1.}: bool
  CC1IE* {.bitsize:1.}: bool
  RESERVED {.bitsize:30.}: 0'u .. 1073741823'u

type TIM10_SR_Fields* = object
  UIF* {.bitsize:1.}: bool
  CC1IF* {.bitsize:1.}: bool
  RESERVED {.bitsize:7.}: 0'u .. 127'u
  CC1OF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:22.}: 0'u .. 4194303'u

type TIM10_EGR_Fields* = object
  UG* {.bitsize:1.}: bool
  CC1G* {.bitsize:1.}: bool
  RESERVED {.bitsize:30.}: 0'u .. 1073741823'u

type TIM10_CCMR1_Output_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool
  OC1PE* {.bitsize:1.}: bool
  OC1M* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM10_CCMR1_Input_Fields* = object
  CC1S* {.bitsize:2.}: 0'u .. 3'u
  IC1PSC* {.bitsize:2.}: 0'u .. 3'u
  IC1F* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type TIM10_CCER_Fields* = object
  CC1E* {.bitsize:1.}: bool
  CC1P* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  CC1NP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:28.}: 0'u .. 268435455'u

type TIM10_CNT_Fields* = object
  CNT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM10_PSC_Fields* = object
  PSC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM10_ARR_Fields* = object
  ARR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM10_CCR1_Fields* = object
  CCR1* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: TIM10_CR1_Type): TIM10_CR1_Fields =
  cast[TIM10_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CR1_Type, val: TIM10_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_CR2_Type): TIM10_CR2_Fields =
  cast[TIM10_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CR2_Type, val: TIM10_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_DIER_Type): TIM10_DIER_Fields =
  cast[TIM10_DIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_DIER_Type, val: TIM10_DIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_DIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_SR_Type): TIM10_SR_Fields =
  cast[TIM10_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_SR_Type, val: TIM10_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: TIM10_EGR_Type, val: TIM10_EGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: TIM10_CCMR1_Output_Type): TIM10_CCMR1_Output_Fields =
  cast[TIM10_CCMR1_Output_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CCMR1_Output_Type, val: TIM10_CCMR1_Output_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CCMR1_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_CCMR1_Input_Type): TIM10_CCMR1_Input_Fields =
  cast[TIM10_CCMR1_Input_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CCMR1_Input_Type, val: TIM10_CCMR1_Input_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CCMR1_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_CCER_Type): TIM10_CCER_Fields =
  cast[TIM10_CCER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CCER_Type, val: TIM10_CCER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CCER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_CNT_Type): TIM10_CNT_Fields =
  cast[TIM10_CNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CNT_Type, val: TIM10_CNT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_PSC_Type): TIM10_PSC_Fields =
  cast[TIM10_PSC_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_PSC_Type, val: TIM10_PSC_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_ARR_Type): TIM10_ARR_Fields =
  cast[TIM10_ARR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_ARR_Type, val: TIM10_ARR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_ARR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM10_CCR1_Type): TIM10_CCR1_Fields =
  cast[TIM10_CCR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM10_CCR1_Type, val: TIM10_CCR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM10_CCR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type TIM6_CR1_Fields* = object
  CEN* {.bitsize:1.}: bool
  UDIS* {.bitsize:1.}: bool
  URS* {.bitsize:1.}: bool
  OPM* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  ARPE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:24.}: 0'u .. 16777215'u

type TIM6_CR2_Fields* = object
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  MMS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type TIM6_DIER_Fields* = object
  UIE* {.bitsize:1.}: bool
  RESERVED {.bitsize:7.}: 0'u .. 127'u
  UDE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:23.}: 0'u .. 8388607'u

type TIM6_SR_Fields* = object
  UIF* {.bitsize:1.}: bool
  RESERVED {.bitsize:31.}: 0'u .. 2147483647'u

type TIM6_EGR_Fields* = object
  UG* {.bitsize:1.}: bool
  RESERVED {.bitsize:31.}: 0'u .. 2147483647'u

type TIM6_CNT_Fields* = object
  CNT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM6_PSC_Fields* = object
  PSC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type TIM6_ARR_Fields* = object
  ARR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: TIM6_CR1_Type): TIM6_CR1_Fields =
  cast[TIM6_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_CR1_Type, val: TIM6_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM6_CR2_Type): TIM6_CR2_Fields =
  cast[TIM6_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_CR2_Type, val: TIM6_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM6_DIER_Type): TIM6_DIER_Fields =
  cast[TIM6_DIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_DIER_Type, val: TIM6_DIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_DIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM6_SR_Type): TIM6_SR_Fields =
  cast[TIM6_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_SR_Type, val: TIM6_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: TIM6_EGR_Type, val: TIM6_EGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: TIM6_CNT_Type): TIM6_CNT_Fields =
  cast[TIM6_CNT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_CNT_Type, val: TIM6_CNT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM6_PSC_Type): TIM6_PSC_Fields =
  cast[TIM6_PSC_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_PSC_Type, val: TIM6_PSC_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: TIM6_ARR_Type): TIM6_ARR_Fields =
  cast[TIM6_ARR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: TIM6_ARR_Type, val: TIM6_ARR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: TIM6_ARR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type I2C1_CR1_Fields* = object
  PE* {.bitsize:1.}: bool
  SMBUS* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  SMBTYPE* {.bitsize:1.}: bool
  ENARP* {.bitsize:1.}: bool
  ENPEC* {.bitsize:1.}: bool
  ENGC* {.bitsize:1.}: bool
  NOSTRETCH* {.bitsize:1.}: bool
  START* {.bitsize:1.}: bool
  STOP* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  POS* {.bitsize:1.}: bool
  PEC* {.bitsize:1.}: bool
  ALERT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  SWRST* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:16.}: 0'u .. 65535'u

type I2C1_CR2_Fields* = object
  FREQ* {.bitsize:6.}: 0'u .. 63'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  ITERREN* {.bitsize:1.}: bool
  ITEVTEN* {.bitsize:1.}: bool
  ITBUFEN* {.bitsize:1.}: bool
  DMAEN* {.bitsize:1.}: bool
  LAST* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:19.}: 0'u .. 524287'u

type I2C1_OAR1_Fields* = object
  ADD0* {.bitsize:1.}: bool
  ADD7* {.bitsize:7.}: 0'u .. 127'u
  ADD10* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  ADDMODE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type I2C1_OAR2_Fields* = object
  ENDUAL* {.bitsize:1.}: bool
  ADD2* {.bitsize:7.}: 0'u .. 127'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type I2C1_DR_Fields* = object
  DR* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type I2C1_SR1_Fields* = object
  SB* {.bitsize:1.}: bool
  ADDRx* {.bitsize:1.}: bool
  BTF* {.bitsize:1.}: bool
  ADD10* {.bitsize:1.}: bool
  STOPF* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  RxNE* {.bitsize:1.}: bool
  TxE* {.bitsize:1.}: bool
  BERR* {.bitsize:1.}: bool
  ARLO* {.bitsize:1.}: bool
  AF* {.bitsize:1.}: bool
  OVR* {.bitsize:1.}: bool
  PECERR* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TIMEOUT* {.bitsize:1.}: bool
  SMBALERT* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:16.}: 0'u .. 65535'u

type I2C1_SR2_Fields* = object
  MSL* {.bitsize:1.}: bool
  BUSY* {.bitsize:1.}: bool
  TRA* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  GENCALL* {.bitsize:1.}: bool
  SMBDEFAULT* {.bitsize:1.}: bool
  SMBHOST* {.bitsize:1.}: bool
  DUALF* {.bitsize:1.}: bool
  PEC* {.bitsize:8.}: 0'u .. 255'u
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type I2C1_CCR_Fields* = object
  CCR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  DUTY* {.bitsize:1.}: bool
  F_S* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type I2C1_TRISE_Fields* = object
  TRISE* {.bitsize:6.}: 0'u .. 63'u
  RESERVED {.bitsize:26.}: 0'u .. 67108863'u

template read*(reg: I2C1_CR1_Type): I2C1_CR1_Fields =
  cast[I2C1_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_CR1_Type, val: I2C1_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_CR2_Type): I2C1_CR2_Fields =
  cast[I2C1_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_CR2_Type, val: I2C1_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_OAR1_Type): I2C1_OAR1_Fields =
  cast[I2C1_OAR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_OAR1_Type, val: I2C1_OAR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_OAR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_OAR2_Type): I2C1_OAR2_Fields =
  cast[I2C1_OAR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_OAR2_Type, val: I2C1_OAR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_OAR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_DR_Type): I2C1_DR_Fields =
  cast[I2C1_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_DR_Type, val: I2C1_DR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_DR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_SR1_Type): I2C1_SR1_Fields =
  cast[I2C1_SR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_SR1_Type, val: I2C1_SR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_SR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_SR2_Type): I2C1_SR2_Fields =
  cast[I2C1_SR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: I2C1_CCR_Type): I2C1_CCR_Fields =
  cast[I2C1_CCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_CCR_Type, val: I2C1_CCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_CCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: I2C1_TRISE_Type): I2C1_TRISE_Fields =
  cast[I2C1_TRISE_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: I2C1_TRISE_Type, val: I2C1_TRISE_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: I2C1_TRISE_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type SPI1_CR1_Fields* = object
  CPHA* {.bitsize:1.}: bool
  CPOL* {.bitsize:1.}: bool
  MSTR* {.bitsize:1.}: bool
  BR* {.bitsize:3.}: 0'u .. 7'u
  SPE* {.bitsize:1.}: bool
  LSBFIRST* {.bitsize:1.}: bool
  SSI* {.bitsize:1.}: bool
  SSM* {.bitsize:1.}: bool
  RXONLY* {.bitsize:1.}: bool
  DFF* {.bitsize:1.}: bool
  CRCNEXT* {.bitsize:1.}: bool
  CRCEN* {.bitsize:1.}: bool
  BIDIOE* {.bitsize:1.}: bool
  BIDIMODE* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type SPI1_CR2_Fields* = object
  RXDMAEN* {.bitsize:1.}: bool
  TXDMAEN* {.bitsize:1.}: bool
  SSOE* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  ERRIE* {.bitsize:1.}: bool
  RXNEIE* {.bitsize:1.}: bool
  TXEIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:24.}: 0'u .. 16777215'u

type SPI1_SR_Fields* = object
  RXNE* {.bitsize:1.}: bool
  TXE* {.bitsize:1.}: bool
  CHSIDE* {.bitsize:1.}: bool
  UDR* {.bitsize:1.}: bool
  CRCERR* {.bitsize:1.}: bool
  MODF* {.bitsize:1.}: bool
  OVR* {.bitsize:1.}: bool
  BSY* {.bitsize:1.}: bool
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type SPI1_DR_Fields* = object
  DR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type SPI1_CRCPR_Fields* = object
  CRCPOLY* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type SPI1_RXCRCR_Fields* = object
  RxCRC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type SPI1_TXCRCR_Fields* = object
  TxCRC* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type SPI1_I2SCFGR_Fields* = object
  CHLEN* {.bitsize:1.}: bool
  DATLEN* {.bitsize:2.}: 0'u .. 3'u
  CKPOL* {.bitsize:1.}: bool
  I2SSTD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool
  PCMSYNC* {.bitsize:1.}: bool
  I2SCFG* {.bitsize:2.}: 0'u .. 3'u
  I2SE* {.bitsize:1.}: bool
  I2SMOD* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:20.}: 0'u .. 1048575'u

type SPI1_I2SPR_Fields* = object
  I2SDIV* {.bitsize:8.}: 0'u .. 255'u
  ODD* {.bitsize:1.}: bool
  MCKOE* {.bitsize:1.}: bool
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u

template read*(reg: SPI1_CR1_Type): SPI1_CR1_Fields =
  cast[SPI1_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_CR1_Type, val: SPI1_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SPI1_CR2_Type): SPI1_CR2_Fields =
  cast[SPI1_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_CR2_Type, val: SPI1_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SPI1_SR_Type): SPI1_SR_Fields =
  cast[SPI1_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_SR_Type, val: SPI1_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SPI1_DR_Type): SPI1_DR_Fields =
  cast[SPI1_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_DR_Type, val: SPI1_DR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_DR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SPI1_CRCPR_Type): SPI1_CRCPR_Fields =
  cast[SPI1_CRCPR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_CRCPR_Type, val: SPI1_CRCPR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_CRCPR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SPI1_RXCRCR_Type): SPI1_RXCRCR_Fields =
  cast[SPI1_RXCRCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SPI1_TXCRCR_Type): SPI1_TXCRCR_Fields =
  cast[SPI1_TXCRCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SPI1_I2SCFGR_Type): SPI1_I2SCFGR_Fields =
  cast[SPI1_I2SCFGR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_I2SCFGR_Type, val: SPI1_I2SCFGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_I2SCFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SPI1_I2SPR_Type): SPI1_I2SPR_Fields =
  cast[SPI1_I2SPR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SPI1_I2SPR_Type, val: SPI1_I2SPR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SPI1_I2SPR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type USART1_SR_Fields* = object
  PE* {.bitsize:1.}: bool
  FE* {.bitsize:1.}: bool
  NE* {.bitsize:1.}: bool
  ORE* {.bitsize:1.}: bool
  IDLE* {.bitsize:1.}: bool
  RXNE* {.bitsize:1.}: bool
  TC* {.bitsize:1.}: bool
  TXE* {.bitsize:1.}: bool
  LBD* {.bitsize:1.}: bool
  CTS* {.bitsize:1.}: bool
  RESERVED {.bitsize:22.}: 0'u .. 4194303'u

type USART1_DR_Fields* = object
  DR* {.bitsize:9.}: 0'u .. 511'u
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

type USART1_BRR_Fields* = object
  DIV_Fraction* {.bitsize:4.}: 0'u .. 15'u
  DIV_Mantissa* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USART1_CR1_Fields* = object
  SBK* {.bitsize:1.}: bool
  RWU* {.bitsize:1.}: bool
  RE* {.bitsize:1.}: bool
  TE* {.bitsize:1.}: bool
  IDLEIE* {.bitsize:1.}: bool
  RXNEIE* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  TXEIE* {.bitsize:1.}: bool
  PEIE* {.bitsize:1.}: bool
  PS* {.bitsize:1.}: bool
  PCE* {.bitsize:1.}: bool
  WAKE* {.bitsize:1.}: bool
  M* {.bitsize:1.}: bool
  UE* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type USART1_CR2_Fields* = object
  ADD* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:1.}: bool
  LBDL* {.bitsize:1.}: bool
  LBDIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  LBCL* {.bitsize:1.}: bool
  CPHA* {.bitsize:1.}: bool
  CPOL* {.bitsize:1.}: bool
  CLKEN* {.bitsize:1.}: bool
  STOP* {.bitsize:2.}: 0'u .. 3'u
  LINEN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:17.}: 0'u .. 131071'u

type USART1_CR3_Fields* = object
  EIE* {.bitsize:1.}: bool
  IREN* {.bitsize:1.}: bool
  IRLP* {.bitsize:1.}: bool
  HDSEL* {.bitsize:1.}: bool
  NACK* {.bitsize:1.}: bool
  SCEN* {.bitsize:1.}: bool
  DMAR* {.bitsize:1.}: bool
  DMAT* {.bitsize:1.}: bool
  RTSE* {.bitsize:1.}: bool
  CTSE* {.bitsize:1.}: bool
  CTSIE* {.bitsize:1.}: bool
  RESERVED {.bitsize:21.}: 0'u .. 2097151'u

type USART1_GTPR_Fields* = object
  PSC* {.bitsize:8.}: 0'u .. 255'u
  GT* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: USART1_SR_Type): USART1_SR_Fields =
  cast[USART1_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_SR_Type, val: USART1_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USART1_DR_Type): USART1_DR_Fields =
  cast[USART1_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_DR_Type, val: USART1_DR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_DR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USART1_BRR_Type): USART1_BRR_Fields =
  cast[USART1_BRR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_BRR_Type, val: USART1_BRR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_BRR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USART1_CR1_Type): USART1_CR1_Fields =
  cast[USART1_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_CR1_Type, val: USART1_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USART1_CR2_Type): USART1_CR2_Fields =
  cast[USART1_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_CR2_Type, val: USART1_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USART1_CR3_Type): USART1_CR3_Fields =
  cast[USART1_CR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_CR3_Type, val: USART1_CR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_CR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USART1_GTPR_Type): USART1_GTPR_Fields =
  cast[USART1_GTPR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USART1_GTPR_Type, val: USART1_GTPR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USART1_GTPR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type ADC1_SR_Fields* = object
  AWD* {.bitsize:1.}: bool
  EOC* {.bitsize:1.}: bool
  JEOC* {.bitsize:1.}: bool
  JSTRT* {.bitsize:1.}: bool
  STRT* {.bitsize:1.}: bool
  RESERVED {.bitsize:27.}: 0'u .. 134217727'u

type ADC1_CR1_Fields* = object
  AWDCH* {.bitsize:5.}: 0'u .. 31'u
  EOCIE* {.bitsize:1.}: bool
  AWDIE* {.bitsize:1.}: bool
  JEOCIE* {.bitsize:1.}: bool
  SCAN* {.bitsize:1.}: bool
  AWDSGL* {.bitsize:1.}: bool
  JAUTO* {.bitsize:1.}: bool
  DISCEN* {.bitsize:1.}: bool
  JDISCEN* {.bitsize:1.}: bool
  DISCNUM* {.bitsize:3.}: 0'u .. 7'u
  DUALMOD* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  JAWDEN* {.bitsize:1.}: bool
  AWDEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:8.}: 0'u .. 255'u

type ADC1_CR2_Fields* = object
  ADON* {.bitsize:1.}: bool
  CONT* {.bitsize:1.}: bool
  CAL* {.bitsize:1.}: bool
  RSTCAL* {.bitsize:1.}: bool
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  DMA* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  ALIGN* {.bitsize:1.}: bool
  JEXTSEL* {.bitsize:3.}: 0'u .. 7'u
  JEXTTRIG* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  EXTSEL* {.bitsize:3.}: 0'u .. 7'u
  EXTTRIG* {.bitsize:1.}: bool
  JSWSTART* {.bitsize:1.}: bool
  SWSTART* {.bitsize:1.}: bool
  TSVREFE* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:8.}: 0'u .. 255'u

type ADC1_SMPR1_Fields* = object
  SMP10* {.bitsize:3.}: 0'u .. 7'u
  SMP11* {.bitsize:3.}: 0'u .. 7'u
  SMP12* {.bitsize:3.}: 0'u .. 7'u
  SMP13* {.bitsize:3.}: 0'u .. 7'u
  SMP14* {.bitsize:3.}: 0'u .. 7'u
  SMP15* {.bitsize:3.}: 0'u .. 7'u
  SMP16* {.bitsize:3.}: 0'u .. 7'u
  SMP17* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type ADC1_SMPR2_Fields* = object
  SMP0* {.bitsize:3.}: 0'u .. 7'u
  SMP1* {.bitsize:3.}: 0'u .. 7'u
  SMP2* {.bitsize:3.}: 0'u .. 7'u
  SMP3* {.bitsize:3.}: 0'u .. 7'u
  SMP4* {.bitsize:3.}: 0'u .. 7'u
  SMP5* {.bitsize:3.}: 0'u .. 7'u
  SMP6* {.bitsize:3.}: 0'u .. 7'u
  SMP7* {.bitsize:3.}: 0'u .. 7'u
  SMP8* {.bitsize:3.}: 0'u .. 7'u
  SMP9* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type ADC1_JOFR1_Fields* = object
  JOFFSET1* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC1_JOFR2_Fields* = object
  JOFFSET2* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC1_JOFR3_Fields* = object
  JOFFSET3* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC1_JOFR4_Fields* = object
  JOFFSET4* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC1_HTR_Fields* = object
  HT* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC1_LTR_Fields* = object
  LT* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC1_SQR1_Fields* = object
  SQ13* {.bitsize:5.}: 0'u .. 31'u
  SQ14* {.bitsize:5.}: 0'u .. 31'u
  SQ15* {.bitsize:5.}: 0'u .. 31'u
  SQ16* {.bitsize:5.}: 0'u .. 31'u
  L* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type ADC1_SQR2_Fields* = object
  SQ7* {.bitsize:5.}: 0'u .. 31'u
  SQ8* {.bitsize:5.}: 0'u .. 31'u
  SQ9* {.bitsize:5.}: 0'u .. 31'u
  SQ10* {.bitsize:5.}: 0'u .. 31'u
  SQ11* {.bitsize:5.}: 0'u .. 31'u
  SQ12* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type ADC1_SQR3_Fields* = object
  SQ1* {.bitsize:5.}: 0'u .. 31'u
  SQ2* {.bitsize:5.}: 0'u .. 31'u
  SQ3* {.bitsize:5.}: 0'u .. 31'u
  SQ4* {.bitsize:5.}: 0'u .. 31'u
  SQ5* {.bitsize:5.}: 0'u .. 31'u
  SQ6* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type ADC1_JSQR_Fields* = object
  JSQ1* {.bitsize:5.}: 0'u .. 31'u
  JSQ2* {.bitsize:5.}: 0'u .. 31'u
  JSQ3* {.bitsize:5.}: 0'u .. 31'u
  JSQ4* {.bitsize:5.}: 0'u .. 31'u
  JL* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:10.}: 0'u .. 1023'u

type ADC1_JDR1_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC1_JDR2_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC1_JDR3_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC1_JDR4_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC1_DR_Fields* = object
  DATA* {.bitsize:16.}: 0'u .. 65535'u
  ADC2DATA* {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: ADC1_SR_Type): ADC1_SR_Fields =
  cast[ADC1_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_SR_Type, val: ADC1_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_CR1_Type): ADC1_CR1_Fields =
  cast[ADC1_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_CR1_Type, val: ADC1_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_CR2_Type): ADC1_CR2_Fields =
  cast[ADC1_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_CR2_Type, val: ADC1_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_SMPR1_Type): ADC1_SMPR1_Fields =
  cast[ADC1_SMPR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_SMPR1_Type, val: ADC1_SMPR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_SMPR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_SMPR2_Type): ADC1_SMPR2_Fields =
  cast[ADC1_SMPR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_SMPR2_Type, val: ADC1_SMPR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_SMPR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_JOFR1_Type): ADC1_JOFR1_Fields =
  cast[ADC1_JOFR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_JOFR1_Type, val: ADC1_JOFR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_JOFR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_JOFR2_Type): ADC1_JOFR2_Fields =
  cast[ADC1_JOFR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_JOFR2_Type, val: ADC1_JOFR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_JOFR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_JOFR3_Type): ADC1_JOFR3_Fields =
  cast[ADC1_JOFR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_JOFR3_Type, val: ADC1_JOFR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_JOFR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_JOFR4_Type): ADC1_JOFR4_Fields =
  cast[ADC1_JOFR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_JOFR4_Type, val: ADC1_JOFR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_JOFR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_HTR_Type): ADC1_HTR_Fields =
  cast[ADC1_HTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_HTR_Type, val: ADC1_HTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_HTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_LTR_Type): ADC1_LTR_Fields =
  cast[ADC1_LTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_LTR_Type, val: ADC1_LTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_LTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_SQR1_Type): ADC1_SQR1_Fields =
  cast[ADC1_SQR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_SQR1_Type, val: ADC1_SQR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_SQR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_SQR2_Type): ADC1_SQR2_Fields =
  cast[ADC1_SQR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_SQR2_Type, val: ADC1_SQR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_SQR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_SQR3_Type): ADC1_SQR3_Fields =
  cast[ADC1_SQR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_SQR3_Type, val: ADC1_SQR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_SQR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_JSQR_Type): ADC1_JSQR_Fields =
  cast[ADC1_JSQR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC1_JSQR_Type, val: ADC1_JSQR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC1_JSQR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC1_JDR1_Type): ADC1_JDR1_Fields =
  cast[ADC1_JDR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC1_JDR2_Type): ADC1_JDR2_Fields =
  cast[ADC1_JDR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC1_JDR3_Type): ADC1_JDR3_Fields =
  cast[ADC1_JDR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC1_JDR4_Type): ADC1_JDR4_Fields =
  cast[ADC1_JDR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC1_DR_Type): ADC1_DR_Fields =
  cast[ADC1_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

type ADC2_SR_Fields* = object
  AWD* {.bitsize:1.}: bool
  EOC* {.bitsize:1.}: bool
  JEOC* {.bitsize:1.}: bool
  JSTRT* {.bitsize:1.}: bool
  STRT* {.bitsize:1.}: bool
  RESERVED {.bitsize:27.}: 0'u .. 134217727'u

type ADC2_CR1_Fields* = object
  AWDCH* {.bitsize:5.}: 0'u .. 31'u
  EOCIE* {.bitsize:1.}: bool
  AWDIE* {.bitsize:1.}: bool
  JEOCIE* {.bitsize:1.}: bool
  SCAN* {.bitsize:1.}: bool
  AWDSGL* {.bitsize:1.}: bool
  JAUTO* {.bitsize:1.}: bool
  DISCEN* {.bitsize:1.}: bool
  JDISCEN* {.bitsize:1.}: bool
  DISCNUM* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:6.}: 0'u .. 63'u
  JAWDEN* {.bitsize:1.}: bool
  AWDEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:8.}: 0'u .. 255'u

type ADC2_CR2_Fields* = object
  ADON* {.bitsize:1.}: bool
  CONT* {.bitsize:1.}: bool
  CAL* {.bitsize:1.}: bool
  RSTCAL* {.bitsize:1.}: bool
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  DMA* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  ALIGN* {.bitsize:1.}: bool
  JEXTSEL* {.bitsize:3.}: 0'u .. 7'u
  JEXTTRIG* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  EXTSEL* {.bitsize:3.}: 0'u .. 7'u
  EXTTRIG* {.bitsize:1.}: bool
  JSWSTART* {.bitsize:1.}: bool
  SWSTART* {.bitsize:1.}: bool
  TSVREFE* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:8.}: 0'u .. 255'u

type ADC2_SMPR1_Fields* = object
  SMP10* {.bitsize:3.}: 0'u .. 7'u
  SMP11* {.bitsize:3.}: 0'u .. 7'u
  SMP12* {.bitsize:3.}: 0'u .. 7'u
  SMP13* {.bitsize:3.}: 0'u .. 7'u
  SMP14* {.bitsize:3.}: 0'u .. 7'u
  SMP15* {.bitsize:3.}: 0'u .. 7'u
  SMP16* {.bitsize:3.}: 0'u .. 7'u
  SMP17* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type ADC2_SMPR2_Fields* = object
  SMP0* {.bitsize:3.}: 0'u .. 7'u
  SMP1* {.bitsize:3.}: 0'u .. 7'u
  SMP2* {.bitsize:3.}: 0'u .. 7'u
  SMP3* {.bitsize:3.}: 0'u .. 7'u
  SMP4* {.bitsize:3.}: 0'u .. 7'u
  SMP5* {.bitsize:3.}: 0'u .. 7'u
  SMP6* {.bitsize:3.}: 0'u .. 7'u
  SMP7* {.bitsize:3.}: 0'u .. 7'u
  SMP8* {.bitsize:3.}: 0'u .. 7'u
  SMP9* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type ADC2_JOFR1_Fields* = object
  JOFFSET1* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC2_JOFR2_Fields* = object
  JOFFSET2* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC2_JOFR3_Fields* = object
  JOFFSET3* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC2_JOFR4_Fields* = object
  JOFFSET4* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC2_HTR_Fields* = object
  HT* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC2_LTR_Fields* = object
  LT* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type ADC2_SQR1_Fields* = object
  SQ13* {.bitsize:5.}: 0'u .. 31'u
  SQ14* {.bitsize:5.}: 0'u .. 31'u
  SQ15* {.bitsize:5.}: 0'u .. 31'u
  SQ16* {.bitsize:5.}: 0'u .. 31'u
  L* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type ADC2_SQR2_Fields* = object
  SQ7* {.bitsize:5.}: 0'u .. 31'u
  SQ8* {.bitsize:5.}: 0'u .. 31'u
  SQ9* {.bitsize:5.}: 0'u .. 31'u
  SQ10* {.bitsize:5.}: 0'u .. 31'u
  SQ11* {.bitsize:5.}: 0'u .. 31'u
  SQ12* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type ADC2_SQR3_Fields* = object
  SQ1* {.bitsize:5.}: 0'u .. 31'u
  SQ2* {.bitsize:5.}: 0'u .. 31'u
  SQ3* {.bitsize:5.}: 0'u .. 31'u
  SQ4* {.bitsize:5.}: 0'u .. 31'u
  SQ5* {.bitsize:5.}: 0'u .. 31'u
  SQ6* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u

type ADC2_JSQR_Fields* = object
  JSQ1* {.bitsize:5.}: 0'u .. 31'u
  JSQ2* {.bitsize:5.}: 0'u .. 31'u
  JSQ3* {.bitsize:5.}: 0'u .. 31'u
  JSQ4* {.bitsize:5.}: 0'u .. 31'u
  JL* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:10.}: 0'u .. 1023'u

type ADC2_JDR1_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC2_JDR2_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC2_JDR3_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC2_JDR4_Fields* = object
  JDATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ADC2_DR_Fields* = object
  DATA* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: ADC2_SR_Type): ADC2_SR_Fields =
  cast[ADC2_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_SR_Type, val: ADC2_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_CR1_Type): ADC2_CR1_Fields =
  cast[ADC2_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_CR1_Type, val: ADC2_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_CR2_Type): ADC2_CR2_Fields =
  cast[ADC2_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_CR2_Type, val: ADC2_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_SMPR1_Type): ADC2_SMPR1_Fields =
  cast[ADC2_SMPR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_SMPR1_Type, val: ADC2_SMPR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_SMPR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_SMPR2_Type): ADC2_SMPR2_Fields =
  cast[ADC2_SMPR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_SMPR2_Type, val: ADC2_SMPR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_SMPR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_JOFR1_Type): ADC2_JOFR1_Fields =
  cast[ADC2_JOFR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_JOFR1_Type, val: ADC2_JOFR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_JOFR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_JOFR2_Type): ADC2_JOFR2_Fields =
  cast[ADC2_JOFR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_JOFR2_Type, val: ADC2_JOFR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_JOFR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_JOFR3_Type): ADC2_JOFR3_Fields =
  cast[ADC2_JOFR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_JOFR3_Type, val: ADC2_JOFR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_JOFR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_JOFR4_Type): ADC2_JOFR4_Fields =
  cast[ADC2_JOFR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_JOFR4_Type, val: ADC2_JOFR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_JOFR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_HTR_Type): ADC2_HTR_Fields =
  cast[ADC2_HTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_HTR_Type, val: ADC2_HTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_HTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_LTR_Type): ADC2_LTR_Fields =
  cast[ADC2_LTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_LTR_Type, val: ADC2_LTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_LTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_SQR1_Type): ADC2_SQR1_Fields =
  cast[ADC2_SQR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_SQR1_Type, val: ADC2_SQR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_SQR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_SQR2_Type): ADC2_SQR2_Fields =
  cast[ADC2_SQR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_SQR2_Type, val: ADC2_SQR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_SQR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_SQR3_Type): ADC2_SQR3_Fields =
  cast[ADC2_SQR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_SQR3_Type, val: ADC2_SQR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_SQR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_JSQR_Type): ADC2_JSQR_Fields =
  cast[ADC2_JSQR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ADC2_JSQR_Type, val: ADC2_JSQR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ADC2_JSQR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ADC2_JDR1_Type): ADC2_JDR1_Fields =
  cast[ADC2_JDR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC2_JDR2_Type): ADC2_JDR2_Fields =
  cast[ADC2_JDR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC2_JDR3_Type): ADC2_JDR3_Fields =
  cast[ADC2_JDR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC2_JDR4_Type): ADC2_JDR4_Fields =
  cast[ADC2_JDR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ADC2_DR_Type): ADC2_DR_Fields =
  cast[ADC2_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

type CAN1_CAN_MCR_Fields* = object
  INRQ* {.bitsize:1.}: bool
  SLEEP* {.bitsize:1.}: bool
  TXFP* {.bitsize:1.}: bool
  RFLM* {.bitsize:1.}: bool
  NART* {.bitsize:1.}: bool
  AWUM* {.bitsize:1.}: bool
  ABOM* {.bitsize:1.}: bool
  TTCM* {.bitsize:1.}: bool
  RESERVED {.bitsize:7.}: 0'u .. 127'u
  RESET* {.bitsize:1.}: bool
  DBF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:15.}: 0'u .. 32767'u

type CAN1_CAN_MSR_Fields* = object
  INAK* {.bitsize:1.}: bool
  SLAK* {.bitsize:1.}: bool
  ERRI* {.bitsize:1.}: bool
  WKUI* {.bitsize:1.}: bool
  SLAKI* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  TXM* {.bitsize:1.}: bool
  RXM* {.bitsize:1.}: bool
  SAMP* {.bitsize:1.}: bool
  RX* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:20.}: 0'u .. 1048575'u

type CAN1_CAN_TSR_Fields* = object
  RQCP0* {.bitsize:1.}: bool
  TXOK0* {.bitsize:1.}: bool
  ALST0* {.bitsize:1.}: bool
  TERR0* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  ABRQ0* {.bitsize:1.}: bool
  RQCP1* {.bitsize:1.}: bool
  TXOK1* {.bitsize:1.}: bool
  ALST1* {.bitsize:1.}: bool
  TERR1* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  ABRQ1* {.bitsize:1.}: bool
  RQCP2* {.bitsize:1.}: bool
  TXOK2* {.bitsize:1.}: bool
  ALST2* {.bitsize:1.}: bool
  TERR2* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:3.}: 0'u .. 7'u
  ABRQ2* {.bitsize:1.}: bool
  CODE* {.bitsize:2.}: 0'u .. 3'u
  TME0* {.bitsize:1.}: bool
  TME1* {.bitsize:1.}: bool
  TME2* {.bitsize:1.}: bool
  LOW0* {.bitsize:1.}: bool
  LOW1* {.bitsize:1.}: bool
  LOW2* {.bitsize:1.}: bool

type CAN1_CAN_RF0R_Fields* = object
  FMP0* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool
  FULL0* {.bitsize:1.}: bool
  FOVR0* {.bitsize:1.}: bool
  RFOM0* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:26.}: 0'u .. 67108863'u

type CAN1_CAN_RF1R_Fields* = object
  FMP1* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool
  FULL1* {.bitsize:1.}: bool
  FOVR1* {.bitsize:1.}: bool
  RFOM1* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:26.}: 0'u .. 67108863'u

type CAN1_CAN_IER_Fields* = object
  TMEIE* {.bitsize:1.}: bool
  FMPIE0* {.bitsize:1.}: bool
  FFIE0* {.bitsize:1.}: bool
  FOVIE0* {.bitsize:1.}: bool
  FMPIE1* {.bitsize:1.}: bool
  FFIE1* {.bitsize:1.}: bool
  FOVIE1* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  EWGIE* {.bitsize:1.}: bool
  EPVIE* {.bitsize:1.}: bool
  BOFIE* {.bitsize:1.}: bool
  LECIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  ERRIE* {.bitsize:1.}: bool
  WKUIE* {.bitsize:1.}: bool
  SLKIE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:14.}: 0'u .. 16383'u

type CAN1_CAN_ESR_Fields* = object
  EWGF* {.bitsize:1.}: bool
  EPVF* {.bitsize:1.}: bool
  BOFF* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LEC* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:9.}: 0'u .. 511'u
  TEC* {.bitsize:8.}: 0'u .. 255'u
  REC* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_BTR_Fields* = object
  BRP* {.bitsize:10.}: 0'u .. 1023'u
  RESERVED {.bitsize:6.}: 0'u .. 63'u
  TS1* {.bitsize:4.}: 0'u .. 15'u
  TS2* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:1.}: bool
  SJW* {.bitsize:2.}: 0'u .. 3'u
  RESERVED2 {.bitsize:4.}: 0'u .. 15'u
  LBKM* {.bitsize:1.}: bool
  SILM* {.bitsize:1.}: bool

type CAN1_CAN_TI0R_Fields* = object
  TXRQ* {.bitsize:1.}: bool
  RTR* {.bitsize:1.}: bool
  IDE* {.bitsize:1.}: bool
  EXID* {.bitsize:18.}: 0'u .. 262143'u
  STID* {.bitsize:11.}: 0'u .. 2047'u

type CAN1_CAN_TDT0R_Fields* = object
  DLC* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  TGT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:7.}: 0'u .. 127'u
  TIME* {.bitsize:16.}: 0'u .. 65535'u

type CAN1_CAN_TDL0R_Fields* = object
  DATA0* {.bitsize:8.}: 0'u .. 255'u
  DATA1* {.bitsize:8.}: 0'u .. 255'u
  DATA2* {.bitsize:8.}: 0'u .. 255'u
  DATA3* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_TDH0R_Fields* = object
  DATA4* {.bitsize:8.}: 0'u .. 255'u
  DATA5* {.bitsize:8.}: 0'u .. 255'u
  DATA6* {.bitsize:8.}: 0'u .. 255'u
  DATA7* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_TI1R_Fields* = object
  TXRQ* {.bitsize:1.}: bool
  RTR* {.bitsize:1.}: bool
  IDE* {.bitsize:1.}: bool
  EXID* {.bitsize:18.}: 0'u .. 262143'u
  STID* {.bitsize:11.}: 0'u .. 2047'u

type CAN1_CAN_TDT1R_Fields* = object
  DLC* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  TGT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:7.}: 0'u .. 127'u
  TIME* {.bitsize:16.}: 0'u .. 65535'u

type CAN1_CAN_TDL1R_Fields* = object
  DATA0* {.bitsize:8.}: 0'u .. 255'u
  DATA1* {.bitsize:8.}: 0'u .. 255'u
  DATA2* {.bitsize:8.}: 0'u .. 255'u
  DATA3* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_TDH1R_Fields* = object
  DATA4* {.bitsize:8.}: 0'u .. 255'u
  DATA5* {.bitsize:8.}: 0'u .. 255'u
  DATA6* {.bitsize:8.}: 0'u .. 255'u
  DATA7* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_TI2R_Fields* = object
  TXRQ* {.bitsize:1.}: bool
  RTR* {.bitsize:1.}: bool
  IDE* {.bitsize:1.}: bool
  EXID* {.bitsize:18.}: 0'u .. 262143'u
  STID* {.bitsize:11.}: 0'u .. 2047'u

type CAN1_CAN_TDT2R_Fields* = object
  DLC* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  TGT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:7.}: 0'u .. 127'u
  TIME* {.bitsize:16.}: 0'u .. 65535'u

type CAN1_CAN_TDL2R_Fields* = object
  DATA0* {.bitsize:8.}: 0'u .. 255'u
  DATA1* {.bitsize:8.}: 0'u .. 255'u
  DATA2* {.bitsize:8.}: 0'u .. 255'u
  DATA3* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_TDH2R_Fields* = object
  DATA4* {.bitsize:8.}: 0'u .. 255'u
  DATA5* {.bitsize:8.}: 0'u .. 255'u
  DATA6* {.bitsize:8.}: 0'u .. 255'u
  DATA7* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_RI0R_Fields* = object
  RESERVED {.bitsize:1.}: bool
  RTR* {.bitsize:1.}: bool
  IDE* {.bitsize:1.}: bool
  EXID* {.bitsize:18.}: 0'u .. 262143'u
  STID* {.bitsize:11.}: 0'u .. 2047'u

type CAN1_CAN_RDT0R_Fields* = object
  DLC* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  FMI* {.bitsize:8.}: 0'u .. 255'u
  TIME* {.bitsize:16.}: 0'u .. 65535'u

type CAN1_CAN_RDL0R_Fields* = object
  DATA0* {.bitsize:8.}: 0'u .. 255'u
  DATA1* {.bitsize:8.}: 0'u .. 255'u
  DATA2* {.bitsize:8.}: 0'u .. 255'u
  DATA3* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_RDH0R_Fields* = object
  DATA4* {.bitsize:8.}: 0'u .. 255'u
  DATA5* {.bitsize:8.}: 0'u .. 255'u
  DATA6* {.bitsize:8.}: 0'u .. 255'u
  DATA7* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_RI1R_Fields* = object
  RESERVED {.bitsize:1.}: bool
  RTR* {.bitsize:1.}: bool
  IDE* {.bitsize:1.}: bool
  EXID* {.bitsize:18.}: 0'u .. 262143'u
  STID* {.bitsize:11.}: 0'u .. 2047'u

type CAN1_CAN_RDT1R_Fields* = object
  DLC* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  FMI* {.bitsize:8.}: 0'u .. 255'u
  TIME* {.bitsize:16.}: 0'u .. 65535'u

type CAN1_CAN_RDL1R_Fields* = object
  DATA0* {.bitsize:8.}: 0'u .. 255'u
  DATA1* {.bitsize:8.}: 0'u .. 255'u
  DATA2* {.bitsize:8.}: 0'u .. 255'u
  DATA3* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_RDH1R_Fields* = object
  DATA4* {.bitsize:8.}: 0'u .. 255'u
  DATA5* {.bitsize:8.}: 0'u .. 255'u
  DATA6* {.bitsize:8.}: 0'u .. 255'u
  DATA7* {.bitsize:8.}: 0'u .. 255'u

type CAN1_CAN_FMR_Fields* = object
  FINIT* {.bitsize:1.}: bool
  RESERVED {.bitsize:31.}: 0'u .. 2147483647'u

type CAN1_CAN_FM1R_Fields* = object
  FBM0* {.bitsize:1.}: bool
  FBM1* {.bitsize:1.}: bool
  FBM2* {.bitsize:1.}: bool
  FBM3* {.bitsize:1.}: bool
  FBM4* {.bitsize:1.}: bool
  FBM5* {.bitsize:1.}: bool
  FBM6* {.bitsize:1.}: bool
  FBM7* {.bitsize:1.}: bool
  FBM8* {.bitsize:1.}: bool
  FBM9* {.bitsize:1.}: bool
  FBM10* {.bitsize:1.}: bool
  FBM11* {.bitsize:1.}: bool
  FBM12* {.bitsize:1.}: bool
  FBM13* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type CAN1_CAN_FS1R_Fields* = object
  FSC0* {.bitsize:1.}: bool
  FSC1* {.bitsize:1.}: bool
  FSC2* {.bitsize:1.}: bool
  FSC3* {.bitsize:1.}: bool
  FSC4* {.bitsize:1.}: bool
  FSC5* {.bitsize:1.}: bool
  FSC6* {.bitsize:1.}: bool
  FSC7* {.bitsize:1.}: bool
  FSC8* {.bitsize:1.}: bool
  FSC9* {.bitsize:1.}: bool
  FSC10* {.bitsize:1.}: bool
  FSC11* {.bitsize:1.}: bool
  FSC12* {.bitsize:1.}: bool
  FSC13* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type CAN1_CAN_FFA1R_Fields* = object
  FFA0* {.bitsize:1.}: bool
  FFA1* {.bitsize:1.}: bool
  FFA2* {.bitsize:1.}: bool
  FFA3* {.bitsize:1.}: bool
  FFA4* {.bitsize:1.}: bool
  FFA5* {.bitsize:1.}: bool
  FFA6* {.bitsize:1.}: bool
  FFA7* {.bitsize:1.}: bool
  FFA8* {.bitsize:1.}: bool
  FFA9* {.bitsize:1.}: bool
  FFA10* {.bitsize:1.}: bool
  FFA11* {.bitsize:1.}: bool
  FFA12* {.bitsize:1.}: bool
  FFA13* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type CAN1_CAN_FA1R_Fields* = object
  FACT0* {.bitsize:1.}: bool
  FACT1* {.bitsize:1.}: bool
  FACT2* {.bitsize:1.}: bool
  FACT3* {.bitsize:1.}: bool
  FACT4* {.bitsize:1.}: bool
  FACT5* {.bitsize:1.}: bool
  FACT6* {.bitsize:1.}: bool
  FACT7* {.bitsize:1.}: bool
  FACT8* {.bitsize:1.}: bool
  FACT9* {.bitsize:1.}: bool
  FACT10* {.bitsize:1.}: bool
  FACT11* {.bitsize:1.}: bool
  FACT12* {.bitsize:1.}: bool
  FACT13* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type CAN1_F0R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F0R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F1R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F1R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F2R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F2R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F3R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F3R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F4R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F4R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F5R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F5R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F6R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F6R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F7R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F7R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F8R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F8R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F9R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F9R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F10R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F10R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F11R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F11R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F12R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F12R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F13R1_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

type CAN1_F13R2_Fields* = object
  FB0* {.bitsize:1.}: bool
  FB1* {.bitsize:1.}: bool
  FB2* {.bitsize:1.}: bool
  FB3* {.bitsize:1.}: bool
  FB4* {.bitsize:1.}: bool
  FB5* {.bitsize:1.}: bool
  FB6* {.bitsize:1.}: bool
  FB7* {.bitsize:1.}: bool
  FB8* {.bitsize:1.}: bool
  FB9* {.bitsize:1.}: bool
  FB10* {.bitsize:1.}: bool
  FB11* {.bitsize:1.}: bool
  FB12* {.bitsize:1.}: bool
  FB13* {.bitsize:1.}: bool
  FB14* {.bitsize:1.}: bool
  FB15* {.bitsize:1.}: bool
  FB16* {.bitsize:1.}: bool
  FB17* {.bitsize:1.}: bool
  FB18* {.bitsize:1.}: bool
  FB19* {.bitsize:1.}: bool
  FB20* {.bitsize:1.}: bool
  FB21* {.bitsize:1.}: bool
  FB22* {.bitsize:1.}: bool
  FB23* {.bitsize:1.}: bool
  FB24* {.bitsize:1.}: bool
  FB25* {.bitsize:1.}: bool
  FB26* {.bitsize:1.}: bool
  FB27* {.bitsize:1.}: bool
  FB28* {.bitsize:1.}: bool
  FB29* {.bitsize:1.}: bool
  FB30* {.bitsize:1.}: bool
  FB31* {.bitsize:1.}: bool

template read*(reg: CAN1_CAN_MCR_Type): CAN1_CAN_MCR_Fields =
  cast[CAN1_CAN_MCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_MCR_Type, val: CAN1_CAN_MCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_MCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_MSR_Type): CAN1_CAN_MSR_Fields =
  cast[CAN1_CAN_MSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_MSR_Type, val: CAN1_CAN_MSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_MSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TSR_Type): CAN1_CAN_TSR_Fields =
  cast[CAN1_CAN_TSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TSR_Type, val: CAN1_CAN_TSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_RF0R_Type): CAN1_CAN_RF0R_Fields =
  cast[CAN1_CAN_RF0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_RF0R_Type, val: CAN1_CAN_RF0R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_RF0R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_RF1R_Type): CAN1_CAN_RF1R_Fields =
  cast[CAN1_CAN_RF1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_RF1R_Type, val: CAN1_CAN_RF1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_RF1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_IER_Type): CAN1_CAN_IER_Fields =
  cast[CAN1_CAN_IER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_IER_Type, val: CAN1_CAN_IER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_IER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_ESR_Type): CAN1_CAN_ESR_Fields =
  cast[CAN1_CAN_ESR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_ESR_Type, val: CAN1_CAN_ESR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_ESR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_BTR_Type): CAN1_CAN_BTR_Fields =
  cast[CAN1_CAN_BTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_BTR_Type, val: CAN1_CAN_BTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_BTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TI0R_Type): CAN1_CAN_TI0R_Fields =
  cast[CAN1_CAN_TI0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TI0R_Type, val: CAN1_CAN_TI0R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TI0R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDT0R_Type): CAN1_CAN_TDT0R_Fields =
  cast[CAN1_CAN_TDT0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDT0R_Type, val: CAN1_CAN_TDT0R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDT0R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDL0R_Type): CAN1_CAN_TDL0R_Fields =
  cast[CAN1_CAN_TDL0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDL0R_Type, val: CAN1_CAN_TDL0R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDL0R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDH0R_Type): CAN1_CAN_TDH0R_Fields =
  cast[CAN1_CAN_TDH0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDH0R_Type, val: CAN1_CAN_TDH0R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDH0R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TI1R_Type): CAN1_CAN_TI1R_Fields =
  cast[CAN1_CAN_TI1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TI1R_Type, val: CAN1_CAN_TI1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TI1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDT1R_Type): CAN1_CAN_TDT1R_Fields =
  cast[CAN1_CAN_TDT1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDT1R_Type, val: CAN1_CAN_TDT1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDT1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDL1R_Type): CAN1_CAN_TDL1R_Fields =
  cast[CAN1_CAN_TDL1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDL1R_Type, val: CAN1_CAN_TDL1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDL1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDH1R_Type): CAN1_CAN_TDH1R_Fields =
  cast[CAN1_CAN_TDH1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDH1R_Type, val: CAN1_CAN_TDH1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDH1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TI2R_Type): CAN1_CAN_TI2R_Fields =
  cast[CAN1_CAN_TI2R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TI2R_Type, val: CAN1_CAN_TI2R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TI2R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDT2R_Type): CAN1_CAN_TDT2R_Fields =
  cast[CAN1_CAN_TDT2R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDT2R_Type, val: CAN1_CAN_TDT2R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDT2R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDL2R_Type): CAN1_CAN_TDL2R_Fields =
  cast[CAN1_CAN_TDL2R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDL2R_Type, val: CAN1_CAN_TDL2R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDL2R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_TDH2R_Type): CAN1_CAN_TDH2R_Fields =
  cast[CAN1_CAN_TDH2R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_TDH2R_Type, val: CAN1_CAN_TDH2R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_TDH2R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_RI0R_Type): CAN1_CAN_RI0R_Fields =
  cast[CAN1_CAN_RI0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RDT0R_Type): CAN1_CAN_RDT0R_Fields =
  cast[CAN1_CAN_RDT0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RDL0R_Type): CAN1_CAN_RDL0R_Fields =
  cast[CAN1_CAN_RDL0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RDH0R_Type): CAN1_CAN_RDH0R_Fields =
  cast[CAN1_CAN_RDH0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RI1R_Type): CAN1_CAN_RI1R_Fields =
  cast[CAN1_CAN_RI1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RDT1R_Type): CAN1_CAN_RDT1R_Fields =
  cast[CAN1_CAN_RDT1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RDL1R_Type): CAN1_CAN_RDL1R_Fields =
  cast[CAN1_CAN_RDL1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_RDH1R_Type): CAN1_CAN_RDH1R_Fields =
  cast[CAN1_CAN_RDH1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: CAN1_CAN_FMR_Type): CAN1_CAN_FMR_Fields =
  cast[CAN1_CAN_FMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_FMR_Type, val: CAN1_CAN_FMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_FMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_FM1R_Type): CAN1_CAN_FM1R_Fields =
  cast[CAN1_CAN_FM1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_FM1R_Type, val: CAN1_CAN_FM1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_FM1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_FS1R_Type): CAN1_CAN_FS1R_Fields =
  cast[CAN1_CAN_FS1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_FS1R_Type, val: CAN1_CAN_FS1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_FS1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_FFA1R_Type): CAN1_CAN_FFA1R_Fields =
  cast[CAN1_CAN_FFA1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_FFA1R_Type, val: CAN1_CAN_FFA1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_FFA1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_CAN_FA1R_Type): CAN1_CAN_FA1R_Fields =
  cast[CAN1_CAN_FA1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_CAN_FA1R_Type, val: CAN1_CAN_FA1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_CAN_FA1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F0R1_Type): CAN1_F0R1_Fields =
  cast[CAN1_F0R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F0R1_Type, val: CAN1_F0R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F0R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F0R2_Type): CAN1_F0R2_Fields =
  cast[CAN1_F0R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F0R2_Type, val: CAN1_F0R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F0R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F1R1_Type): CAN1_F1R1_Fields =
  cast[CAN1_F1R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F1R1_Type, val: CAN1_F1R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F1R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F1R2_Type): CAN1_F1R2_Fields =
  cast[CAN1_F1R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F1R2_Type, val: CAN1_F1R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F1R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F2R1_Type): CAN1_F2R1_Fields =
  cast[CAN1_F2R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F2R1_Type, val: CAN1_F2R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F2R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F2R2_Type): CAN1_F2R2_Fields =
  cast[CAN1_F2R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F2R2_Type, val: CAN1_F2R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F2R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F3R1_Type): CAN1_F3R1_Fields =
  cast[CAN1_F3R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F3R1_Type, val: CAN1_F3R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F3R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F3R2_Type): CAN1_F3R2_Fields =
  cast[CAN1_F3R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F3R2_Type, val: CAN1_F3R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F3R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F4R1_Type): CAN1_F4R1_Fields =
  cast[CAN1_F4R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F4R1_Type, val: CAN1_F4R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F4R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F4R2_Type): CAN1_F4R2_Fields =
  cast[CAN1_F4R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F4R2_Type, val: CAN1_F4R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F4R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F5R1_Type): CAN1_F5R1_Fields =
  cast[CAN1_F5R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F5R1_Type, val: CAN1_F5R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F5R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F5R2_Type): CAN1_F5R2_Fields =
  cast[CAN1_F5R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F5R2_Type, val: CAN1_F5R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F5R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F6R1_Type): CAN1_F6R1_Fields =
  cast[CAN1_F6R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F6R1_Type, val: CAN1_F6R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F6R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F6R2_Type): CAN1_F6R2_Fields =
  cast[CAN1_F6R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F6R2_Type, val: CAN1_F6R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F6R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F7R1_Type): CAN1_F7R1_Fields =
  cast[CAN1_F7R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F7R1_Type, val: CAN1_F7R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F7R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F7R2_Type): CAN1_F7R2_Fields =
  cast[CAN1_F7R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F7R2_Type, val: CAN1_F7R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F7R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F8R1_Type): CAN1_F8R1_Fields =
  cast[CAN1_F8R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F8R1_Type, val: CAN1_F8R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F8R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F8R2_Type): CAN1_F8R2_Fields =
  cast[CAN1_F8R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F8R2_Type, val: CAN1_F8R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F8R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F9R1_Type): CAN1_F9R1_Fields =
  cast[CAN1_F9R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F9R1_Type, val: CAN1_F9R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F9R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F9R2_Type): CAN1_F9R2_Fields =
  cast[CAN1_F9R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F9R2_Type, val: CAN1_F9R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F9R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F10R1_Type): CAN1_F10R1_Fields =
  cast[CAN1_F10R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F10R1_Type, val: CAN1_F10R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F10R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F10R2_Type): CAN1_F10R2_Fields =
  cast[CAN1_F10R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F10R2_Type, val: CAN1_F10R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F10R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F11R1_Type): CAN1_F11R1_Fields =
  cast[CAN1_F11R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F11R1_Type, val: CAN1_F11R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F11R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F11R2_Type): CAN1_F11R2_Fields =
  cast[CAN1_F11R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F11R2_Type, val: CAN1_F11R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F11R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F12R1_Type): CAN1_F12R1_Fields =
  cast[CAN1_F12R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F12R1_Type, val: CAN1_F12R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F12R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F12R2_Type): CAN1_F12R2_Fields =
  cast[CAN1_F12R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F12R2_Type, val: CAN1_F12R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F12R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F13R1_Type): CAN1_F13R1_Fields =
  cast[CAN1_F13R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F13R1_Type, val: CAN1_F13R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F13R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CAN1_F13R2_Type): CAN1_F13R2_Fields =
  cast[CAN1_F13R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CAN1_F13R2_Type, val: CAN1_F13R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CAN1_F13R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type DAC_CR_Fields* = object
  EN1* {.bitsize:1.}: bool
  BOFF1* {.bitsize:1.}: bool
  TEN1* {.bitsize:1.}: bool
  TSEL1* {.bitsize:3.}: 0'u .. 7'u
  WAVE1* {.bitsize:2.}: 0'u .. 3'u
  MAMP1* {.bitsize:4.}: 0'u .. 15'u
  DMAEN1* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  EN2* {.bitsize:1.}: bool
  BOFF2* {.bitsize:1.}: bool
  TEN2* {.bitsize:1.}: bool
  TSEL2* {.bitsize:3.}: 0'u .. 7'u
  WAVE2* {.bitsize:2.}: 0'u .. 3'u
  MAMP2* {.bitsize:4.}: 0'u .. 15'u
  DMAEN2* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u

type DAC_SWTRIGR_Fields* = object
  SWTRIG1* {.bitsize:1.}: bool
  SWTRIG2* {.bitsize:1.}: bool
  RESERVED {.bitsize:30.}: 0'u .. 1073741823'u

type DAC_DHR12R1_Fields* = object
  DACC1DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type DAC_DHR12L1_Fields* = object
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  DACC1DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type DAC_DHR8R1_Fields* = object
  DACC1DHR* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type DAC_DHR12R2_Fields* = object
  DACC2DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type DAC_DHR12L2_Fields* = object
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  DACC2DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type DAC_DHR8R2_Fields* = object
  DACC2DHR* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type DAC_DHR12RD_Fields* = object
  DACC1DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  DACC2DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u

type DAC_DHR12LD_Fields* = object
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  DACC1DHR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  DACC2DHR* {.bitsize:12.}: 0'u .. 4095'u

type DAC_DHR8RD_Fields* = object
  DACC1DHR* {.bitsize:8.}: 0'u .. 255'u
  DACC2DHR* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type DAC_DOR1_Fields* = object
  DACC1DOR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type DAC_DOR2_Fields* = object
  DACC2DOR* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

template read*(reg: DAC_CR_Type): DAC_CR_Fields =
  cast[DAC_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_CR_Type, val: DAC_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: DAC_SWTRIGR_Type, val: DAC_SWTRIGR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template read*(reg: DAC_DHR12R1_Type): DAC_DHR12R1_Fields =
  cast[DAC_DHR12R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR12R1_Type, val: DAC_DHR12R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR12R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR12L1_Type): DAC_DHR12L1_Fields =
  cast[DAC_DHR12L1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR12L1_Type, val: DAC_DHR12L1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR12L1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR8R1_Type): DAC_DHR8R1_Fields =
  cast[DAC_DHR8R1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR8R1_Type, val: DAC_DHR8R1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR8R1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR12R2_Type): DAC_DHR12R2_Fields =
  cast[DAC_DHR12R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR12R2_Type, val: DAC_DHR12R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR12R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR12L2_Type): DAC_DHR12L2_Fields =
  cast[DAC_DHR12L2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR12L2_Type, val: DAC_DHR12L2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR12L2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR8R2_Type): DAC_DHR8R2_Fields =
  cast[DAC_DHR8R2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR8R2_Type, val: DAC_DHR8R2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR8R2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR12RD_Type): DAC_DHR12RD_Fields =
  cast[DAC_DHR12RD_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR12RD_Type, val: DAC_DHR12RD_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR12RD_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR12LD_Type): DAC_DHR12LD_Fields =
  cast[DAC_DHR12LD_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR12LD_Type, val: DAC_DHR12LD_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR12LD_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DHR8RD_Type): DAC_DHR8RD_Fields =
  cast[DAC_DHR8RD_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DAC_DHR8RD_Type, val: DAC_DHR8RD_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DAC_DHR8RD_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: DAC_DOR1_Type): DAC_DOR1_Fields =
  cast[DAC_DOR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: DAC_DOR2_Type): DAC_DOR2_Fields =
  cast[DAC_DOR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

type DBG_IDCODE_Fields* = object
  DEV_ID* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  REV_ID* {.bitsize:16.}: 0'u .. 65535'u

type DBG_CR_Fields* = object
  DBG_SLEEP* {.bitsize:1.}: bool
  DBG_STOP* {.bitsize:1.}: bool
  DBG_STANDBY* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  TRACE_IOEN* {.bitsize:1.}: bool
  TRACE_MODE* {.bitsize:2.}: 0'u .. 3'u
  DBG_IWDG_STOP* {.bitsize:1.}: bool
  DBG_WWDG_STOP* {.bitsize:1.}: bool
  DBG_TIM1_STOP* {.bitsize:1.}: bool
  DBG_TIM2_STOP* {.bitsize:1.}: bool
  DBG_TIM3_STOP* {.bitsize:1.}: bool
  DBG_TIM4_STOP* {.bitsize:1.}: bool
  DBG_CAN1_STOP* {.bitsize:1.}: bool
  DBG_I2C1_SMBUS_TIMEOUT* {.bitsize:1.}: bool
  DBG_I2C2_SMBUS_TIMEOUT* {.bitsize:1.}: bool
  DBG_TIM8_STOP* {.bitsize:1.}: bool
  DBG_TIM5_STOP* {.bitsize:1.}: bool
  DBG_TIM6_STOP* {.bitsize:1.}: bool
  DBG_TIM7_STOP* {.bitsize:1.}: bool
  DBG_CAN2_STOP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:10.}: 0'u .. 1023'u

template read*(reg: DBG_IDCODE_Type): DBG_IDCODE_Fields =
  cast[DBG_IDCODE_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: DBG_CR_Type): DBG_CR_Fields =
  cast[DBG_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: DBG_CR_Type, val: DBG_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: DBG_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type UART4_SR_Fields* = object
  PE* {.bitsize:1.}: bool
  FE* {.bitsize:1.}: bool
  NE* {.bitsize:1.}: bool
  ORE* {.bitsize:1.}: bool
  IDLE* {.bitsize:1.}: bool
  RXNE* {.bitsize:1.}: bool
  TC* {.bitsize:1.}: bool
  TXE* {.bitsize:1.}: bool
  LBD* {.bitsize:1.}: bool
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

type UART4_DR_Fields* = object
  DR* {.bitsize:9.}: 0'u .. 511'u
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

type UART4_BRR_Fields* = object
  DIV_Fraction* {.bitsize:4.}: 0'u .. 15'u
  DIV_Mantissa* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type UART4_CR1_Fields* = object
  SBK* {.bitsize:1.}: bool
  RWU* {.bitsize:1.}: bool
  RE* {.bitsize:1.}: bool
  TE* {.bitsize:1.}: bool
  IDLEIE* {.bitsize:1.}: bool
  RXNEIE* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  TXEIE* {.bitsize:1.}: bool
  PEIE* {.bitsize:1.}: bool
  PS* {.bitsize:1.}: bool
  PCE* {.bitsize:1.}: bool
  WAKE* {.bitsize:1.}: bool
  M* {.bitsize:1.}: bool
  UE* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type UART4_CR2_Fields* = object
  ADD* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:1.}: bool
  LBDL* {.bitsize:1.}: bool
  LBDIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  STOP* {.bitsize:2.}: 0'u .. 3'u
  LINEN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:17.}: 0'u .. 131071'u

type UART4_CR3_Fields* = object
  EIE* {.bitsize:1.}: bool
  IREN* {.bitsize:1.}: bool
  IRLP* {.bitsize:1.}: bool
  HDSEL* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  DMAR* {.bitsize:1.}: bool
  DMAT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:24.}: 0'u .. 16777215'u

template read*(reg: UART4_SR_Type): UART4_SR_Fields =
  cast[UART4_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART4_SR_Type, val: UART4_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART4_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART4_DR_Type): UART4_DR_Fields =
  cast[UART4_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART4_DR_Type, val: UART4_DR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART4_DR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART4_BRR_Type): UART4_BRR_Fields =
  cast[UART4_BRR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART4_BRR_Type, val: UART4_BRR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART4_BRR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART4_CR1_Type): UART4_CR1_Fields =
  cast[UART4_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART4_CR1_Type, val: UART4_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART4_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART4_CR2_Type): UART4_CR2_Fields =
  cast[UART4_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART4_CR2_Type, val: UART4_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART4_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART4_CR3_Type): UART4_CR3_Fields =
  cast[UART4_CR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART4_CR3_Type, val: UART4_CR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART4_CR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type UART5_SR_Fields* = object
  PE* {.bitsize:1.}: bool
  FE* {.bitsize:1.}: bool
  NE* {.bitsize:1.}: bool
  ORE* {.bitsize:1.}: bool
  IDLE* {.bitsize:1.}: bool
  RXNE* {.bitsize:1.}: bool
  TC* {.bitsize:1.}: bool
  TXE* {.bitsize:1.}: bool
  LBD* {.bitsize:1.}: bool
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

type UART5_DR_Fields* = object
  DR* {.bitsize:9.}: 0'u .. 511'u
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

type UART5_BRR_Fields* = object
  DIV_Fraction* {.bitsize:4.}: 0'u .. 15'u
  DIV_Mantissa* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type UART5_CR1_Fields* = object
  SBK* {.bitsize:1.}: bool
  RWU* {.bitsize:1.}: bool
  RE* {.bitsize:1.}: bool
  TE* {.bitsize:1.}: bool
  IDLEIE* {.bitsize:1.}: bool
  RXNEIE* {.bitsize:1.}: bool
  TCIE* {.bitsize:1.}: bool
  TXEIE* {.bitsize:1.}: bool
  PEIE* {.bitsize:1.}: bool
  PS* {.bitsize:1.}: bool
  PCE* {.bitsize:1.}: bool
  WAKE* {.bitsize:1.}: bool
  M* {.bitsize:1.}: bool
  UE* {.bitsize:1.}: bool
  RESERVED {.bitsize:18.}: 0'u .. 262143'u

type UART5_CR2_Fields* = object
  ADD* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:1.}: bool
  LBDL* {.bitsize:1.}: bool
  LBDIE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  STOP* {.bitsize:2.}: 0'u .. 3'u
  LINEN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:17.}: 0'u .. 131071'u

type UART5_CR3_Fields* = object
  EIE* {.bitsize:1.}: bool
  IREN* {.bitsize:1.}: bool
  IRLP* {.bitsize:1.}: bool
  HDSEL* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  DMAT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:24.}: 0'u .. 16777215'u

template read*(reg: UART5_SR_Type): UART5_SR_Fields =
  cast[UART5_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART5_SR_Type, val: UART5_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART5_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART5_DR_Type): UART5_DR_Fields =
  cast[UART5_DR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART5_DR_Type, val: UART5_DR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART5_DR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART5_BRR_Type): UART5_BRR_Fields =
  cast[UART5_BRR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART5_BRR_Type, val: UART5_BRR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART5_BRR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART5_CR1_Type): UART5_CR1_Fields =
  cast[UART5_CR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART5_CR1_Type, val: UART5_CR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART5_CR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART5_CR2_Type): UART5_CR2_Fields =
  cast[UART5_CR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART5_CR2_Type, val: UART5_CR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART5_CR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: UART5_CR3_Type): UART5_CR3_Fields =
  cast[UART5_CR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: UART5_CR3_Type, val: UART5_CR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: UART5_CR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type CRC_IDR_Fields* = object
  IDR* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type CRC_CR_Fields* = object
  RESET* {.bitsize:1.}: bool
  RESERVED {.bitsize:31.}: 0'u .. 2147483647'u

template read*(reg: CRC_DR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: CRC_DR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: CRC_DR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: CRC_IDR_Type): CRC_IDR_Fields =
  cast[CRC_IDR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: CRC_IDR_Type, val: CRC_IDR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: CRC_IDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: CRC_CR_Type, val: CRC_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

type FLASH_ACR_Fields* = object
  LATENCY* {.bitsize:3.}: 0'u .. 7'u
  HLFCYA* {.bitsize:1.}: bool
  PRFTBE* {.bitsize:1.}: bool
  PRFTBS* {.bitsize:1.}: bool
  RESERVED {.bitsize:26.}: 0'u .. 67108863'u

type FLASH_SR_Fields* = object
  BSY* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  PGERR* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  WRPRTERR* {.bitsize:1.}: bool
  EOP* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:26.}: 0'u .. 67108863'u

type FLASH_CR_Fields* = object
  PG* {.bitsize:1.}: bool
  PER* {.bitsize:1.}: bool
  MER* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  OPTPG* {.bitsize:1.}: bool
  OPTER* {.bitsize:1.}: bool
  STRT* {.bitsize:1.}: bool
  LOCK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  OPTWRE* {.bitsize:1.}: bool
  ERRIE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  EOPIE* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:19.}: 0'u .. 524287'u

type FLASH_OBR_Fields* = object
  OPTERR* {.bitsize:1.}: bool
  RDPRT* {.bitsize:1.}: bool
  WDG_SW* {.bitsize:1.}: bool
  nRST_STOP* {.bitsize:1.}: bool
  nRST_STDBY* {.bitsize:1.}: bool
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  Data0* {.bitsize:8.}: 0'u .. 255'u
  Data1* {.bitsize:8.}: 0'u .. 255'u
  RESERVED1 {.bitsize:6.}: 0'u .. 63'u

template read*(reg: FLASH_ACR_Type): FLASH_ACR_Fields =
  cast[FLASH_ACR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FLASH_ACR_Type, val: FLASH_ACR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FLASH_ACR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: FLASH_KEYR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template write*(reg: FLASH_OPTKEYR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template read*(reg: FLASH_SR_Type): FLASH_SR_Fields =
  cast[FLASH_SR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FLASH_SR_Type, val: FLASH_SR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FLASH_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: FLASH_CR_Type): FLASH_CR_Fields =
  cast[FLASH_CR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: FLASH_CR_Type, val: FLASH_CR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: FLASH_CR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template write*(reg: FLASH_AR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template read*(reg: FLASH_OBR_Type): FLASH_OBR_Fields =
  cast[FLASH_OBR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: FLASH_WRPR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

type USB_EP0R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP1R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP2R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP3R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP4R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP5R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP6R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_EP7R_Fields* = object
  EA* {.bitsize:4.}: 0'u .. 15'u
  STAT_TX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_TX* {.bitsize:1.}: bool
  CTR_TX* {.bitsize:1.}: bool
  EP_KIND* {.bitsize:1.}: bool
  EP_TYPE* {.bitsize:2.}: 0'u .. 3'u
  SETUP* {.bitsize:1.}: bool
  STAT_RX* {.bitsize:2.}: 0'u .. 3'u
  DTOG_RX* {.bitsize:1.}: bool
  CTR_RX* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_CNTR_Fields* = object
  FRES* {.bitsize:1.}: bool
  PDWN* {.bitsize:1.}: bool
  LPMODE* {.bitsize:1.}: bool
  FSUSP* {.bitsize:1.}: bool
  RESUME* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  ESOFM* {.bitsize:1.}: bool
  SOFM* {.bitsize:1.}: bool
  RESETM* {.bitsize:1.}: bool
  SUSPM* {.bitsize:1.}: bool
  WKUPM* {.bitsize:1.}: bool
  ERRM* {.bitsize:1.}: bool
  PMAOVRM* {.bitsize:1.}: bool
  CTRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type USB_ISTR_Fields* = object
  EP_ID* {.bitsize:4.}: 0'u .. 15'u
  DIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  ESOF* {.bitsize:1.}: bool
  SOF* {.bitsize:1.}: bool
  RESET* {.bitsize:1.}: bool
  SUSP* {.bitsize:1.}: bool
  WKUP* {.bitsize:1.}: bool
  ERR* {.bitsize:1.}: bool
  PMAOVR* {.bitsize:1.}: bool
  CTR* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type USB_FNR_Fields* = object
  FN* {.bitsize:11.}: 0'u .. 2047'u
  LSOF* {.bitsize:2.}: 0'u .. 3'u
  LCK* {.bitsize:1.}: bool
  RXDM* {.bitsize:1.}: bool
  RXDP* {.bitsize:1.}: bool
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type USB_DADDR_Fields* = object
  ADD* {.bitsize:7.}: 0'u .. 127'u
  EF* {.bitsize:1.}: bool
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type USB_BTABLE_Fields* = object
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  BTABLE* {.bitsize:13.}: 0'u .. 8191'u
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: USB_EP0R_Type): USB_EP0R_Fields =
  cast[USB_EP0R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP0R_Type, val: USB_EP0R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP0R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP1R_Type): USB_EP1R_Fields =
  cast[USB_EP1R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP1R_Type, val: USB_EP1R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP1R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP2R_Type): USB_EP2R_Fields =
  cast[USB_EP2R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP2R_Type, val: USB_EP2R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP2R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP3R_Type): USB_EP3R_Fields =
  cast[USB_EP3R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP3R_Type, val: USB_EP3R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP3R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP4R_Type): USB_EP4R_Fields =
  cast[USB_EP4R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP4R_Type, val: USB_EP4R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP4R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP5R_Type): USB_EP5R_Fields =
  cast[USB_EP5R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP5R_Type, val: USB_EP5R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP5R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP6R_Type): USB_EP6R_Fields =
  cast[USB_EP6R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP6R_Type, val: USB_EP6R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP6R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_EP7R_Type): USB_EP7R_Fields =
  cast[USB_EP7R_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_EP7R_Type, val: USB_EP7R_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_EP7R_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_CNTR_Type): USB_CNTR_Fields =
  cast[USB_CNTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_CNTR_Type, val: USB_CNTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_CNTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_ISTR_Type): USB_ISTR_Fields =
  cast[USB_ISTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_ISTR_Type, val: USB_ISTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_ISTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_FNR_Type): USB_FNR_Fields =
  cast[USB_FNR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: USB_DADDR_Type): USB_DADDR_Fields =
  cast[USB_DADDR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_DADDR_Type, val: USB_DADDR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_DADDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: USB_BTABLE_Type): USB_BTABLE_Fields =
  cast[USB_BTABLE_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: USB_BTABLE_Type, val: USB_BTABLE_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: USB_BTABLE_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type OTG_FS_DEVICE_FS_DCFG_Fields* = object
  DSPD* {.bitsize:2.}: 0'u .. 3'u
  NZLSOHSK* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  DAD* {.bitsize:7.}: 0'u .. 127'u
  PFIVL* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:19.}: 0'u .. 524287'u

type OTG_FS_DEVICE_FS_DCTL_Fields* = object
  RWUSIG* {.bitsize:1.}: bool
  SDIS* {.bitsize:1.}: bool
  GINSTS* {.bitsize:1.}: bool
  GONSTS* {.bitsize:1.}: bool
  TCTL* {.bitsize:3.}: 0'u .. 7'u
  SGINAK* {.bitsize:1.}: bool
  CGINAK* {.bitsize:1.}: bool
  SGONAK* {.bitsize:1.}: bool
  CGONAK* {.bitsize:1.}: bool
  POPRGDNE* {.bitsize:1.}: bool
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type OTG_FS_DEVICE_FS_DSTS_Fields* = object
  SUSPSTS* {.bitsize:1.}: bool
  ENUMSPD* {.bitsize:2.}: 0'u .. 3'u
  EERR* {.bitsize:1.}: bool
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  FNSOF* {.bitsize:14.}: 0'u .. 16383'u
  RESERVED1 {.bitsize:10.}: 0'u .. 1023'u

type OTG_FS_DEVICE_FS_DIEPMSK_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  EPDM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TOM* {.bitsize:1.}: bool
  ITTXFEMSK* {.bitsize:1.}: bool
  INEPNMM* {.bitsize:1.}: bool
  INEPNEM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:25.}: 0'u .. 33554431'u

type OTG_FS_DEVICE_FS_DOEPMSK_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  EPDM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STUPM* {.bitsize:1.}: bool
  OTEPDM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:27.}: 0'u .. 134217727'u

type OTG_FS_DEVICE_FS_DAINT_Fields* = object
  IEPINT* {.bitsize:16.}: 0'u .. 65535'u
  OEPINT* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_FS_DAINTMSK_Fields* = object
  IEPM* {.bitsize:16.}: 0'u .. 65535'u
  OEPINT* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_DVBUSDIS_Fields* = object
  VBUSDT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_DVBUSPULSE_Fields* = object
  DVBUSP* {.bitsize:12.}: 0'u .. 4095'u
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u

type OTG_FS_DEVICE_DIEPEMPMSK_Fields* = object
  INEPTXFEM* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_FS_DIEPCTL0_Fields* = object
  MPSIZ* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:13.}: 0'u .. 8191'u
  USBAEP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  RESERVED2 {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  TXFNUM* {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:2.}: 0'u .. 3'u
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPCTL1_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  USBAEP* {.bitsize:1.}: bool
  EONUM_DPID* {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  TXFNUM* {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  SD0PID_SEVNFRM* {.bitsize:1.}: bool
  SODDFRM_SD1PID* {.bitsize:1.}: bool
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPCTL2_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  USBAEP* {.bitsize:1.}: bool
  EONUM_DPID* {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  TXFNUM* {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  SD0PID_SEVNFRM* {.bitsize:1.}: bool
  SODDFRM* {.bitsize:1.}: bool
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPCTL3_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  USBAEP* {.bitsize:1.}: bool
  EONUM_DPID* {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  TXFNUM* {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  SD0PID_SEVNFRM* {.bitsize:1.}: bool
  SODDFRM* {.bitsize:1.}: bool
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DOEPCTL0_Fields* = object
  MPSIZ* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:13.}: 0'u .. 8191'u
  USBAEP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  SNPM* {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:2.}: 0'u .. 3'u
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DOEPCTL1_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  USBAEP* {.bitsize:1.}: bool
  EONUM_DPID* {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  SNPM* {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  SD0PID_SEVNFRM* {.bitsize:1.}: bool
  SODDFRM* {.bitsize:1.}: bool
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DOEPCTL2_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  USBAEP* {.bitsize:1.}: bool
  EONUM_DPID* {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  SNPM* {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  SD0PID_SEVNFRM* {.bitsize:1.}: bool
  SODDFRM* {.bitsize:1.}: bool
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DOEPCTL3_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  RESERVED {.bitsize:4.}: 0'u .. 15'u
  USBAEP* {.bitsize:1.}: bool
  EONUM_DPID* {.bitsize:1.}: bool
  NAKSTS* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  SNPM* {.bitsize:1.}: bool
  Stall* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  CNAK* {.bitsize:1.}: bool
  SNAK* {.bitsize:1.}: bool
  SD0PID_SEVNFRM* {.bitsize:1.}: bool
  SODDFRM* {.bitsize:1.}: bool
  EPDIS* {.bitsize:1.}: bool
  EPENA* {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPINT0_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TOC* {.bitsize:1.}: bool
  ITTXFE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  INEPNE* {.bitsize:1.}: bool
  TXFE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:24.}: 0'u .. 16777215'u

type OTG_FS_DEVICE_DIEPINT1_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TOC* {.bitsize:1.}: bool
  ITTXFE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  INEPNE* {.bitsize:1.}: bool
  TXFE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:24.}: 0'u .. 16777215'u

type OTG_FS_DEVICE_DIEPINT2_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TOC* {.bitsize:1.}: bool
  ITTXFE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  INEPNE* {.bitsize:1.}: bool
  TXFE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:24.}: 0'u .. 16777215'u

type OTG_FS_DEVICE_DIEPINT3_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  TOC* {.bitsize:1.}: bool
  ITTXFE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  INEPNE* {.bitsize:1.}: bool
  TXFE* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:24.}: 0'u .. 16777215'u

type OTG_FS_DEVICE_DOEPINT0_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STUP* {.bitsize:1.}: bool
  OTEPDIS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  B2BSTUP* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:25.}: 0'u .. 33554431'u

type OTG_FS_DEVICE_DOEPINT1_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STUP* {.bitsize:1.}: bool
  OTEPDIS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  B2BSTUP* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:25.}: 0'u .. 33554431'u

type OTG_FS_DEVICE_DOEPINT2_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STUP* {.bitsize:1.}: bool
  OTEPDIS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  B2BSTUP* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:25.}: 0'u .. 33554431'u

type OTG_FS_DEVICE_DOEPINT3_Fields* = object
  XFRC* {.bitsize:1.}: bool
  EPDISD* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STUP* {.bitsize:1.}: bool
  OTEPDIS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  B2BSTUP* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:25.}: 0'u .. 33554431'u

type OTG_FS_DEVICE_DIEPTSIZ0_Fields* = object
  XFRSIZ* {.bitsize:7.}: 0'u .. 127'u
  RESERVED {.bitsize:12.}: 0'u .. 4095'u
  PKTCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:11.}: 0'u .. 2047'u

type OTG_FS_DEVICE_DOEPTSIZ0_Fields* = object
  XFRSIZ* {.bitsize:7.}: 0'u .. 127'u
  RESERVED {.bitsize:12.}: 0'u .. 4095'u
  PKTCNT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:9.}: 0'u .. 511'u
  STUPCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED2 {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPTSIZ1_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPTSIZ2_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_DEVICE_DIEPTSIZ3_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_DEVICE_DTXFSTS0_Fields* = object
  INEPTFSAV* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_DTXFSTS1_Fields* = object
  INEPTFSAV* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_DTXFSTS2_Fields* = object
  INEPTFSAV* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_DTXFSTS3_Fields* = object
  INEPTFSAV* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_DEVICE_DOEPTSIZ1_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  RXDPID_STUPCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_DEVICE_DOEPTSIZ2_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  RXDPID_STUPCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_DEVICE_DOEPTSIZ3_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  RXDPID_STUPCNT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

template read*(reg: OTG_FS_DEVICE_FS_DCFG_Type): OTG_FS_DEVICE_FS_DCFG_Fields =
  cast[OTG_FS_DEVICE_FS_DCFG_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_FS_DCFG_Type, val: OTG_FS_DEVICE_FS_DCFG_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_FS_DCFG_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_FS_DCTL_Type): OTG_FS_DEVICE_FS_DCTL_Fields =
  cast[OTG_FS_DEVICE_FS_DCTL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_FS_DCTL_Type, val: OTG_FS_DEVICE_FS_DCTL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_FS_DCTL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_FS_DSTS_Type): OTG_FS_DEVICE_FS_DSTS_Fields =
  cast[OTG_FS_DEVICE_FS_DSTS_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_DEVICE_FS_DIEPMSK_Type): OTG_FS_DEVICE_FS_DIEPMSK_Fields =
  cast[OTG_FS_DEVICE_FS_DIEPMSK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_FS_DIEPMSK_Type, val: OTG_FS_DEVICE_FS_DIEPMSK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_FS_DIEPMSK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_FS_DOEPMSK_Type): OTG_FS_DEVICE_FS_DOEPMSK_Fields =
  cast[OTG_FS_DEVICE_FS_DOEPMSK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_FS_DOEPMSK_Type, val: OTG_FS_DEVICE_FS_DOEPMSK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_FS_DOEPMSK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_FS_DAINT_Type): OTG_FS_DEVICE_FS_DAINT_Fields =
  cast[OTG_FS_DEVICE_FS_DAINT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_DEVICE_FS_DAINTMSK_Type): OTG_FS_DEVICE_FS_DAINTMSK_Fields =
  cast[OTG_FS_DEVICE_FS_DAINTMSK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_FS_DAINTMSK_Type, val: OTG_FS_DEVICE_FS_DAINTMSK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_FS_DAINTMSK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DVBUSDIS_Type): OTG_FS_DEVICE_DVBUSDIS_Fields =
  cast[OTG_FS_DEVICE_DVBUSDIS_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DVBUSDIS_Type, val: OTG_FS_DEVICE_DVBUSDIS_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DVBUSDIS_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DVBUSPULSE_Type): OTG_FS_DEVICE_DVBUSPULSE_Fields =
  cast[OTG_FS_DEVICE_DVBUSPULSE_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DVBUSPULSE_Type, val: OTG_FS_DEVICE_DVBUSPULSE_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DVBUSPULSE_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPEMPMSK_Type): OTG_FS_DEVICE_DIEPEMPMSK_Fields =
  cast[OTG_FS_DEVICE_DIEPEMPMSK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPEMPMSK_Type, val: OTG_FS_DEVICE_DIEPEMPMSK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPEMPMSK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_FS_DIEPCTL0_Type): OTG_FS_DEVICE_FS_DIEPCTL0_Fields =
  cast[OTG_FS_DEVICE_FS_DIEPCTL0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_FS_DIEPCTL0_Type, val: OTG_FS_DEVICE_FS_DIEPCTL0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_FS_DIEPCTL0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPCTL1_Type): OTG_FS_DEVICE_DIEPCTL1_Fields =
  cast[OTG_FS_DEVICE_DIEPCTL1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPCTL1_Type, val: OTG_FS_DEVICE_DIEPCTL1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPCTL1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPCTL2_Type): OTG_FS_DEVICE_DIEPCTL2_Fields =
  cast[OTG_FS_DEVICE_DIEPCTL2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPCTL2_Type, val: OTG_FS_DEVICE_DIEPCTL2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPCTL2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPCTL3_Type): OTG_FS_DEVICE_DIEPCTL3_Fields =
  cast[OTG_FS_DEVICE_DIEPCTL3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPCTL3_Type, val: OTG_FS_DEVICE_DIEPCTL3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPCTL3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPCTL0_Type): OTG_FS_DEVICE_DOEPCTL0_Fields =
  cast[OTG_FS_DEVICE_DOEPCTL0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPCTL0_Type, val: OTG_FS_DEVICE_DOEPCTL0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPCTL0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPCTL1_Type): OTG_FS_DEVICE_DOEPCTL1_Fields =
  cast[OTG_FS_DEVICE_DOEPCTL1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPCTL1_Type, val: OTG_FS_DEVICE_DOEPCTL1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPCTL1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPCTL2_Type): OTG_FS_DEVICE_DOEPCTL2_Fields =
  cast[OTG_FS_DEVICE_DOEPCTL2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPCTL2_Type, val: OTG_FS_DEVICE_DOEPCTL2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPCTL2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPCTL3_Type): OTG_FS_DEVICE_DOEPCTL3_Fields =
  cast[OTG_FS_DEVICE_DOEPCTL3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPCTL3_Type, val: OTG_FS_DEVICE_DOEPCTL3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPCTL3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPINT0_Type): OTG_FS_DEVICE_DIEPINT0_Fields =
  cast[OTG_FS_DEVICE_DIEPINT0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPINT0_Type, val: OTG_FS_DEVICE_DIEPINT0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPINT0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPINT1_Type): OTG_FS_DEVICE_DIEPINT1_Fields =
  cast[OTG_FS_DEVICE_DIEPINT1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPINT1_Type, val: OTG_FS_DEVICE_DIEPINT1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPINT1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPINT2_Type): OTG_FS_DEVICE_DIEPINT2_Fields =
  cast[OTG_FS_DEVICE_DIEPINT2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPINT2_Type, val: OTG_FS_DEVICE_DIEPINT2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPINT2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPINT3_Type): OTG_FS_DEVICE_DIEPINT3_Fields =
  cast[OTG_FS_DEVICE_DIEPINT3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPINT3_Type, val: OTG_FS_DEVICE_DIEPINT3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPINT3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPINT0_Type): OTG_FS_DEVICE_DOEPINT0_Fields =
  cast[OTG_FS_DEVICE_DOEPINT0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPINT0_Type, val: OTG_FS_DEVICE_DOEPINT0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPINT0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPINT1_Type): OTG_FS_DEVICE_DOEPINT1_Fields =
  cast[OTG_FS_DEVICE_DOEPINT1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPINT1_Type, val: OTG_FS_DEVICE_DOEPINT1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPINT1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPINT2_Type): OTG_FS_DEVICE_DOEPINT2_Fields =
  cast[OTG_FS_DEVICE_DOEPINT2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPINT2_Type, val: OTG_FS_DEVICE_DOEPINT2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPINT2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPINT3_Type): OTG_FS_DEVICE_DOEPINT3_Fields =
  cast[OTG_FS_DEVICE_DOEPINT3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPINT3_Type, val: OTG_FS_DEVICE_DOEPINT3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPINT3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPTSIZ0_Type): OTG_FS_DEVICE_DIEPTSIZ0_Fields =
  cast[OTG_FS_DEVICE_DIEPTSIZ0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPTSIZ0_Type, val: OTG_FS_DEVICE_DIEPTSIZ0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPTSIZ0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPTSIZ0_Type): OTG_FS_DEVICE_DOEPTSIZ0_Fields =
  cast[OTG_FS_DEVICE_DOEPTSIZ0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPTSIZ0_Type, val: OTG_FS_DEVICE_DOEPTSIZ0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPTSIZ0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPTSIZ1_Type): OTG_FS_DEVICE_DIEPTSIZ1_Fields =
  cast[OTG_FS_DEVICE_DIEPTSIZ1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPTSIZ1_Type, val: OTG_FS_DEVICE_DIEPTSIZ1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPTSIZ1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPTSIZ2_Type): OTG_FS_DEVICE_DIEPTSIZ2_Fields =
  cast[OTG_FS_DEVICE_DIEPTSIZ2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPTSIZ2_Type, val: OTG_FS_DEVICE_DIEPTSIZ2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPTSIZ2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DIEPTSIZ3_Type): OTG_FS_DEVICE_DIEPTSIZ3_Fields =
  cast[OTG_FS_DEVICE_DIEPTSIZ3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DIEPTSIZ3_Type, val: OTG_FS_DEVICE_DIEPTSIZ3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DIEPTSIZ3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DTXFSTS0_Type): OTG_FS_DEVICE_DTXFSTS0_Fields =
  cast[OTG_FS_DEVICE_DTXFSTS0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_DEVICE_DTXFSTS1_Type): OTG_FS_DEVICE_DTXFSTS1_Fields =
  cast[OTG_FS_DEVICE_DTXFSTS1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_DEVICE_DTXFSTS2_Type): OTG_FS_DEVICE_DTXFSTS2_Fields =
  cast[OTG_FS_DEVICE_DTXFSTS2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_DEVICE_DTXFSTS3_Type): OTG_FS_DEVICE_DTXFSTS3_Fields =
  cast[OTG_FS_DEVICE_DTXFSTS3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_DEVICE_DOEPTSIZ1_Type): OTG_FS_DEVICE_DOEPTSIZ1_Fields =
  cast[OTG_FS_DEVICE_DOEPTSIZ1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPTSIZ1_Type, val: OTG_FS_DEVICE_DOEPTSIZ1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPTSIZ1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPTSIZ2_Type): OTG_FS_DEVICE_DOEPTSIZ2_Fields =
  cast[OTG_FS_DEVICE_DOEPTSIZ2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPTSIZ2_Type, val: OTG_FS_DEVICE_DOEPTSIZ2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPTSIZ2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_DEVICE_DOEPTSIZ3_Type): OTG_FS_DEVICE_DOEPTSIZ3_Fields =
  cast[OTG_FS_DEVICE_DOEPTSIZ3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_DEVICE_DOEPTSIZ3_Type, val: OTG_FS_DEVICE_DOEPTSIZ3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_DEVICE_DOEPTSIZ3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type OTG_FS_GLOBAL_FS_GOTGCTL_Fields* = object
  SRQSCS* {.bitsize:1.}: bool
  SRQ* {.bitsize:1.}: bool
  RESERVED {.bitsize:6.}: 0'u .. 63'u
  HNGSCS* {.bitsize:1.}: bool
  HNPRQ* {.bitsize:1.}: bool
  HSHNPEN* {.bitsize:1.}: bool
  DHNPEN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  CIDSTS* {.bitsize:1.}: bool
  DBCT* {.bitsize:1.}: bool
  ASVLD* {.bitsize:1.}: bool
  BSVLD* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:12.}: 0'u .. 4095'u

type OTG_FS_GLOBAL_FS_GOTGINT_Fields* = object
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  SEDET* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  SRSSCHG* {.bitsize:1.}: bool
  HNSSCHG* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:7.}: 0'u .. 127'u
  HNGDET* {.bitsize:1.}: bool
  ADTOCHG* {.bitsize:1.}: bool
  DBCDNE* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:12.}: 0'u .. 4095'u

type OTG_FS_GLOBAL_FS_GAHBCFG_Fields* = object
  GINT* {.bitsize:1.}: bool
  RESERVED {.bitsize:6.}: 0'u .. 63'u
  TXFELVL* {.bitsize:1.}: bool
  PTXFELVL* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:23.}: 0'u .. 8388607'u

type OTG_FS_GLOBAL_FS_GUSBCFG_Fields* = object
  TOCAL* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  PHYSEL* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  SRPCAP* {.bitsize:1.}: bool
  HNPCAP* {.bitsize:1.}: bool
  TRDT* {.bitsize:4.}: 0'u .. 15'u
  RESERVED2 {.bitsize:15.}: 0'u .. 32767'u
  FHMOD* {.bitsize:1.}: bool
  FDMOD* {.bitsize:1.}: bool
  CTXPKT* {.bitsize:1.}: bool

type OTG_FS_GLOBAL_FS_GRSTCTL_Fields* = object
  CSRST* {.bitsize:1.}: bool
  HSRST* {.bitsize:1.}: bool
  FCRST* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  RXFFLSH* {.bitsize:1.}: bool
  TXFFLSH* {.bitsize:1.}: bool
  TXFNUM* {.bitsize:5.}: 0'u .. 31'u
  RESERVED1 {.bitsize:20.}: 0'u .. 1048575'u
  AHBIDL* {.bitsize:1.}: bool

type OTG_FS_GLOBAL_FS_GINTSTS_Fields* = object
  CMOD* {.bitsize:1.}: bool
  MMIS* {.bitsize:1.}: bool
  OTGINT* {.bitsize:1.}: bool
  SOF* {.bitsize:1.}: bool
  RXFLVL* {.bitsize:1.}: bool
  NPTXFE* {.bitsize:1.}: bool
  GINAKEFF* {.bitsize:1.}: bool
  GOUTNAKEFF* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  ESUSP* {.bitsize:1.}: bool
  USBSUSP* {.bitsize:1.}: bool
  USBRST* {.bitsize:1.}: bool
  ENUMDNE* {.bitsize:1.}: bool
  ISOODRP* {.bitsize:1.}: bool
  EOPF* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  IEPINT* {.bitsize:1.}: bool
  OEPINT* {.bitsize:1.}: bool
  IISOIXFR* {.bitsize:1.}: bool
  IPXFR_INCOMPISOOUT* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:2.}: 0'u .. 3'u
  HPRTINT* {.bitsize:1.}: bool
  HCINT* {.bitsize:1.}: bool
  PTXFE* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:1.}: bool
  CIDSCHG* {.bitsize:1.}: bool
  DISCINT* {.bitsize:1.}: bool
  SRQINT* {.bitsize:1.}: bool
  WKUPINT* {.bitsize:1.}: bool

type OTG_FS_GLOBAL_FS_GINTMSK_Fields* = object
  RESERVED {.bitsize:1.}: bool
  MMISM* {.bitsize:1.}: bool
  OTGINT* {.bitsize:1.}: bool
  SOFM* {.bitsize:1.}: bool
  RXFLVLM* {.bitsize:1.}: bool
  NPTXFEM* {.bitsize:1.}: bool
  GINAKEFFM* {.bitsize:1.}: bool
  GONAKEFFM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  ESUSPM* {.bitsize:1.}: bool
  USBSUSPM* {.bitsize:1.}: bool
  USBRST* {.bitsize:1.}: bool
  ENUMDNEM* {.bitsize:1.}: bool
  ISOODRPM* {.bitsize:1.}: bool
  EOPFM* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  EPMISM* {.bitsize:1.}: bool
  IEPINT* {.bitsize:1.}: bool
  OEPINT* {.bitsize:1.}: bool
  IISOIXFRM* {.bitsize:1.}: bool
  IPXFRM_IISOOXFRM* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:2.}: 0'u .. 3'u
  PRTIM* {.bitsize:1.}: bool
  HCIM* {.bitsize:1.}: bool
  PTXFEM* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:1.}: bool
  CIDSCHGM* {.bitsize:1.}: bool
  DISCINT* {.bitsize:1.}: bool
  SRQIM* {.bitsize:1.}: bool
  WUIM* {.bitsize:1.}: bool

type OTG_FS_GLOBAL_FS_GRXSTSR_Device_Fields* = object
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  BCNT* {.bitsize:11.}: 0'u .. 2047'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  PKTSTS* {.bitsize:4.}: 0'u .. 15'u
  FRMNUM* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:7.}: 0'u .. 127'u

type OTG_FS_GLOBAL_FS_GRXSTSR_Host_Fields* = object
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  BCNT* {.bitsize:11.}: 0'u .. 2047'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  PKTSTS* {.bitsize:4.}: 0'u .. 15'u
  FRMNUM* {.bitsize:4.}: 0'u .. 15'u
  RESERVED {.bitsize:7.}: 0'u .. 127'u

type OTG_FS_GLOBAL_FS_GRXFSIZ_Fields* = object
  RXFD* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Fields* = object
  TX0FSA* {.bitsize:16.}: 0'u .. 65535'u
  TX0FD* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Fields* = object
  NPTXFSA* {.bitsize:16.}: 0'u .. 65535'u
  NPTXFD* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_GLOBAL_FS_GNPTXSTS_Fields* = object
  NPTXFSAV* {.bitsize:16.}: 0'u .. 65535'u
  NPTQXSAV* {.bitsize:8.}: 0'u .. 255'u
  NPTXQTOP* {.bitsize:7.}: 0'u .. 127'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_GLOBAL_FS_GCCFG_Fields* = object
  RESERVED {.bitsize:16.}: 0'u .. 65535'u
  PWRDWN* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  VBUSASEN* {.bitsize:1.}: bool
  VBUSBSEN* {.bitsize:1.}: bool
  SOFOUTEN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:11.}: 0'u .. 2047'u

type OTG_FS_GLOBAL_FS_HPTXFSIZ_Fields* = object
  PTXSA* {.bitsize:16.}: 0'u .. 65535'u
  PTXFSIZ* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_GLOBAL_FS_DIEPTXF1_Fields* = object
  INEPTXSA* {.bitsize:16.}: 0'u .. 65535'u
  INEPTXFD* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_GLOBAL_FS_DIEPTXF2_Fields* = object
  INEPTXSA* {.bitsize:16.}: 0'u .. 65535'u
  INEPTXFD* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_GLOBAL_FS_DIEPTXF3_Fields* = object
  INEPTXSA* {.bitsize:16.}: 0'u .. 65535'u
  INEPTXFD* {.bitsize:16.}: 0'u .. 65535'u

template read*(reg: OTG_FS_GLOBAL_FS_GOTGCTL_Type): OTG_FS_GLOBAL_FS_GOTGCTL_Fields =
  cast[OTG_FS_GLOBAL_FS_GOTGCTL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GOTGCTL_Type, val: OTG_FS_GLOBAL_FS_GOTGCTL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GOTGCTL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GOTGINT_Type): OTG_FS_GLOBAL_FS_GOTGINT_Fields =
  cast[OTG_FS_GLOBAL_FS_GOTGINT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GOTGINT_Type, val: OTG_FS_GLOBAL_FS_GOTGINT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GOTGINT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GAHBCFG_Type): OTG_FS_GLOBAL_FS_GAHBCFG_Fields =
  cast[OTG_FS_GLOBAL_FS_GAHBCFG_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GAHBCFG_Type, val: OTG_FS_GLOBAL_FS_GAHBCFG_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GAHBCFG_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GUSBCFG_Type): OTG_FS_GLOBAL_FS_GUSBCFG_Fields =
  cast[OTG_FS_GLOBAL_FS_GUSBCFG_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GUSBCFG_Type, val: OTG_FS_GLOBAL_FS_GUSBCFG_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GUSBCFG_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GRSTCTL_Type): OTG_FS_GLOBAL_FS_GRSTCTL_Fields =
  cast[OTG_FS_GLOBAL_FS_GRSTCTL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GRSTCTL_Type, val: OTG_FS_GLOBAL_FS_GRSTCTL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GRSTCTL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GINTSTS_Type): OTG_FS_GLOBAL_FS_GINTSTS_Fields =
  cast[OTG_FS_GLOBAL_FS_GINTSTS_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GINTSTS_Type, val: OTG_FS_GLOBAL_FS_GINTSTS_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GINTSTS_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GINTMSK_Type): OTG_FS_GLOBAL_FS_GINTMSK_Fields =
  cast[OTG_FS_GLOBAL_FS_GINTMSK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GINTMSK_Type, val: OTG_FS_GLOBAL_FS_GINTMSK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GINTMSK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GRXSTSR_Device_Type): OTG_FS_GLOBAL_FS_GRXSTSR_Device_Fields =
  cast[OTG_FS_GLOBAL_FS_GRXSTSR_Device_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_GLOBAL_FS_GRXSTSR_Host_Type): OTG_FS_GLOBAL_FS_GRXSTSR_Host_Fields =
  cast[OTG_FS_GLOBAL_FS_GRXSTSR_Host_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_GLOBAL_FS_GRXFSIZ_Type): OTG_FS_GLOBAL_FS_GRXFSIZ_Fields =
  cast[OTG_FS_GLOBAL_FS_GRXFSIZ_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GRXFSIZ_Type, val: OTG_FS_GLOBAL_FS_GRXFSIZ_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GRXFSIZ_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Type): OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Fields =
  cast[OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Type, val: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Device_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Type): OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Fields =
  cast[OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Type, val: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GNPTXFSIZ_Host_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_GNPTXSTS_Type): OTG_FS_GLOBAL_FS_GNPTXSTS_Fields =
  cast[OTG_FS_GLOBAL_FS_GNPTXSTS_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_GLOBAL_FS_GCCFG_Type): OTG_FS_GLOBAL_FS_GCCFG_Fields =
  cast[OTG_FS_GLOBAL_FS_GCCFG_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_GCCFG_Type, val: OTG_FS_GLOBAL_FS_GCCFG_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_GCCFG_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_CID_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: OTG_FS_GLOBAL_FS_CID_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: OTG_FS_GLOBAL_FS_CID_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_HPTXFSIZ_Type): OTG_FS_GLOBAL_FS_HPTXFSIZ_Fields =
  cast[OTG_FS_GLOBAL_FS_HPTXFSIZ_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_HPTXFSIZ_Type, val: OTG_FS_GLOBAL_FS_HPTXFSIZ_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_HPTXFSIZ_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_DIEPTXF1_Type): OTG_FS_GLOBAL_FS_DIEPTXF1_Fields =
  cast[OTG_FS_GLOBAL_FS_DIEPTXF1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_DIEPTXF1_Type, val: OTG_FS_GLOBAL_FS_DIEPTXF1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_DIEPTXF1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_DIEPTXF2_Type): OTG_FS_GLOBAL_FS_DIEPTXF2_Fields =
  cast[OTG_FS_GLOBAL_FS_DIEPTXF2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_DIEPTXF2_Type, val: OTG_FS_GLOBAL_FS_DIEPTXF2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_DIEPTXF2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_GLOBAL_FS_DIEPTXF3_Type): OTG_FS_GLOBAL_FS_DIEPTXF3_Fields =
  cast[OTG_FS_GLOBAL_FS_DIEPTXF3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_GLOBAL_FS_DIEPTXF3_Type, val: OTG_FS_GLOBAL_FS_DIEPTXF3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_GLOBAL_FS_DIEPTXF3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type OTG_FS_HOST_FS_HCFG_Fields* = object
  FSLSPCS* {.bitsize:2.}: 0'u .. 3'u
  FSLSS* {.bitsize:1.}: bool
  RESERVED {.bitsize:29.}: 0'u .. 536870911'u

type OTG_FS_HOST_HFIR_Fields* = object
  FRIVL* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_HOST_FS_HFNUM_Fields* = object
  FRNUM* {.bitsize:16.}: 0'u .. 65535'u
  FTREM* {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_HOST_FS_HPTXSTS_Fields* = object
  PTXFSAVL* {.bitsize:16.}: 0'u .. 65535'u
  PTXQSAV* {.bitsize:8.}: 0'u .. 255'u
  PTXQTOP* {.bitsize:8.}: 0'u .. 255'u

type OTG_FS_HOST_HAINT_Fields* = object
  HAINT* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_HOST_HAINTMSK_Fields* = object
  HAINTM* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type OTG_FS_HOST_FS_HPRT_Fields* = object
  PCSTS* {.bitsize:1.}: bool
  PCDET* {.bitsize:1.}: bool
  PENA* {.bitsize:1.}: bool
  PENCHNG* {.bitsize:1.}: bool
  POCA* {.bitsize:1.}: bool
  POCCHNG* {.bitsize:1.}: bool
  PRES* {.bitsize:1.}: bool
  PSUSP* {.bitsize:1.}: bool
  PRST* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  PLSTS* {.bitsize:2.}: 0'u .. 3'u
  PPWR* {.bitsize:1.}: bool
  PTCTL* {.bitsize:4.}: 0'u .. 15'u
  PSPD* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:13.}: 0'u .. 8191'u

type OTG_FS_HOST_FS_HCCHAR0_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR1_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR2_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR3_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR4_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR5_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR6_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCCHAR7_Fields* = object
  MPSIZ* {.bitsize:11.}: 0'u .. 2047'u
  EPNUM* {.bitsize:4.}: 0'u .. 15'u
  EPDIR* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  LSDEV* {.bitsize:1.}: bool
  EPTYP* {.bitsize:2.}: 0'u .. 3'u
  MCNT* {.bitsize:2.}: 0'u .. 3'u
  DAD* {.bitsize:7.}: 0'u .. 127'u
  ODDFRM* {.bitsize:1.}: bool
  CHDIS* {.bitsize:1.}: bool
  CHENA* {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCINT0_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT1_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT2_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT3_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT4_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT5_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT6_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINT7_Fields* = object
  XFRC* {.bitsize:1.}: bool
  CHH* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALL* {.bitsize:1.}: bool
  NAK* {.bitsize:1.}: bool
  ACK* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  TXERR* {.bitsize:1.}: bool
  BBERR* {.bitsize:1.}: bool
  FRMOR* {.bitsize:1.}: bool
  DTERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK0_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK1_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK2_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK3_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK4_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK5_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK6_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCINTMSK7_Fields* = object
  XFRCM* {.bitsize:1.}: bool
  CHHM* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  STALLM* {.bitsize:1.}: bool
  NAKM* {.bitsize:1.}: bool
  ACKM* {.bitsize:1.}: bool
  NYET* {.bitsize:1.}: bool
  TXERRM* {.bitsize:1.}: bool
  BBERRM* {.bitsize:1.}: bool
  FRMORM* {.bitsize:1.}: bool
  DTERRM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:21.}: 0'u .. 2097151'u

type OTG_FS_HOST_FS_HCTSIZ0_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ1_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ2_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ3_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ4_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ5_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ6_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

type OTG_FS_HOST_FS_HCTSIZ7_Fields* = object
  XFRSIZ* {.bitsize:19.}: 0'u .. 524287'u
  PKTCNT* {.bitsize:10.}: 0'u .. 1023'u
  DPID* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool

template read*(reg: OTG_FS_HOST_FS_HCFG_Type): OTG_FS_HOST_FS_HCFG_Fields =
  cast[OTG_FS_HOST_FS_HCFG_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCFG_Type, val: OTG_FS_HOST_FS_HCFG_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCFG_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_HFIR_Type): OTG_FS_HOST_HFIR_Fields =
  cast[OTG_FS_HOST_HFIR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_HFIR_Type, val: OTG_FS_HOST_HFIR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_HFIR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HFNUM_Type): OTG_FS_HOST_FS_HFNUM_Fields =
  cast[OTG_FS_HOST_FS_HFNUM_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_HOST_FS_HPTXSTS_Type): OTG_FS_HOST_FS_HPTXSTS_Fields =
  cast[OTG_FS_HOST_FS_HPTXSTS_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HPTXSTS_Type, val: OTG_FS_HOST_FS_HPTXSTS_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HPTXSTS_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_HAINT_Type): OTG_FS_HOST_HAINT_Fields =
  cast[OTG_FS_HOST_HAINT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: OTG_FS_HOST_HAINTMSK_Type): OTG_FS_HOST_HAINTMSK_Fields =
  cast[OTG_FS_HOST_HAINTMSK_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_HAINTMSK_Type, val: OTG_FS_HOST_HAINTMSK_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_HAINTMSK_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HPRT_Type): OTG_FS_HOST_FS_HPRT_Fields =
  cast[OTG_FS_HOST_FS_HPRT_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HPRT_Type, val: OTG_FS_HOST_FS_HPRT_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HPRT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR0_Type): OTG_FS_HOST_FS_HCCHAR0_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR0_Type, val: OTG_FS_HOST_FS_HCCHAR0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR1_Type): OTG_FS_HOST_FS_HCCHAR1_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR1_Type, val: OTG_FS_HOST_FS_HCCHAR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR2_Type): OTG_FS_HOST_FS_HCCHAR2_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR2_Type, val: OTG_FS_HOST_FS_HCCHAR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR3_Type): OTG_FS_HOST_FS_HCCHAR3_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR3_Type, val: OTG_FS_HOST_FS_HCCHAR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR4_Type): OTG_FS_HOST_FS_HCCHAR4_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR4_Type, val: OTG_FS_HOST_FS_HCCHAR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR5_Type): OTG_FS_HOST_FS_HCCHAR5_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR5_Type, val: OTG_FS_HOST_FS_HCCHAR5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR6_Type): OTG_FS_HOST_FS_HCCHAR6_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR6_Type, val: OTG_FS_HOST_FS_HCCHAR6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCCHAR7_Type): OTG_FS_HOST_FS_HCCHAR7_Fields =
  cast[OTG_FS_HOST_FS_HCCHAR7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCCHAR7_Type, val: OTG_FS_HOST_FS_HCCHAR7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCCHAR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT0_Type): OTG_FS_HOST_FS_HCINT0_Fields =
  cast[OTG_FS_HOST_FS_HCINT0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT0_Type, val: OTG_FS_HOST_FS_HCINT0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT1_Type): OTG_FS_HOST_FS_HCINT1_Fields =
  cast[OTG_FS_HOST_FS_HCINT1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT1_Type, val: OTG_FS_HOST_FS_HCINT1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT2_Type): OTG_FS_HOST_FS_HCINT2_Fields =
  cast[OTG_FS_HOST_FS_HCINT2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT2_Type, val: OTG_FS_HOST_FS_HCINT2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT3_Type): OTG_FS_HOST_FS_HCINT3_Fields =
  cast[OTG_FS_HOST_FS_HCINT3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT3_Type, val: OTG_FS_HOST_FS_HCINT3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT4_Type): OTG_FS_HOST_FS_HCINT4_Fields =
  cast[OTG_FS_HOST_FS_HCINT4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT4_Type, val: OTG_FS_HOST_FS_HCINT4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT5_Type): OTG_FS_HOST_FS_HCINT5_Fields =
  cast[OTG_FS_HOST_FS_HCINT5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT5_Type, val: OTG_FS_HOST_FS_HCINT5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT6_Type): OTG_FS_HOST_FS_HCINT6_Fields =
  cast[OTG_FS_HOST_FS_HCINT6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT6_Type, val: OTG_FS_HOST_FS_HCINT6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINT7_Type): OTG_FS_HOST_FS_HCINT7_Fields =
  cast[OTG_FS_HOST_FS_HCINT7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINT7_Type, val: OTG_FS_HOST_FS_HCINT7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINT7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK0_Type): OTG_FS_HOST_FS_HCINTMSK0_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK0_Type, val: OTG_FS_HOST_FS_HCINTMSK0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK1_Type): OTG_FS_HOST_FS_HCINTMSK1_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK1_Type, val: OTG_FS_HOST_FS_HCINTMSK1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK2_Type): OTG_FS_HOST_FS_HCINTMSK2_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK2_Type, val: OTG_FS_HOST_FS_HCINTMSK2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK3_Type): OTG_FS_HOST_FS_HCINTMSK3_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK3_Type, val: OTG_FS_HOST_FS_HCINTMSK3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK4_Type): OTG_FS_HOST_FS_HCINTMSK4_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK4_Type, val: OTG_FS_HOST_FS_HCINTMSK4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK5_Type): OTG_FS_HOST_FS_HCINTMSK5_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK5_Type, val: OTG_FS_HOST_FS_HCINTMSK5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK6_Type): OTG_FS_HOST_FS_HCINTMSK6_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK6_Type, val: OTG_FS_HOST_FS_HCINTMSK6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCINTMSK7_Type): OTG_FS_HOST_FS_HCINTMSK7_Fields =
  cast[OTG_FS_HOST_FS_HCINTMSK7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCINTMSK7_Type, val: OTG_FS_HOST_FS_HCINTMSK7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCINTMSK7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ0_Type): OTG_FS_HOST_FS_HCTSIZ0_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ0_Type, val: OTG_FS_HOST_FS_HCTSIZ0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ1_Type): OTG_FS_HOST_FS_HCTSIZ1_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ1_Type, val: OTG_FS_HOST_FS_HCTSIZ1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ2_Type): OTG_FS_HOST_FS_HCTSIZ2_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ2_Type, val: OTG_FS_HOST_FS_HCTSIZ2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ3_Type): OTG_FS_HOST_FS_HCTSIZ3_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ3_Type, val: OTG_FS_HOST_FS_HCTSIZ3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ4_Type): OTG_FS_HOST_FS_HCTSIZ4_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ4_Type, val: OTG_FS_HOST_FS_HCTSIZ4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ5_Type): OTG_FS_HOST_FS_HCTSIZ5_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ5_Type, val: OTG_FS_HOST_FS_HCTSIZ5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ6_Type): OTG_FS_HOST_FS_HCTSIZ6_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ6_Type, val: OTG_FS_HOST_FS_HCTSIZ6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: OTG_FS_HOST_FS_HCTSIZ7_Type): OTG_FS_HOST_FS_HCTSIZ7_Fields =
  cast[OTG_FS_HOST_FS_HCTSIZ7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_HOST_FS_HCTSIZ7_Type, val: OTG_FS_HOST_FS_HCTSIZ7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_HOST_FS_HCTSIZ7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type OTG_FS_PWRCLK_FS_PCGCCTL_Fields* = object
  STPPCLK* {.bitsize:1.}: bool
  GATEHCLK* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  PHYSUSP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:27.}: 0'u .. 134217727'u

template read*(reg: OTG_FS_PWRCLK_FS_PCGCCTL_Type): OTG_FS_PWRCLK_FS_PCGCCTL_Fields =
  cast[OTG_FS_PWRCLK_FS_PCGCCTL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: OTG_FS_PWRCLK_FS_PCGCCTL_Type, val: OTG_FS_PWRCLK_FS_PCGCCTL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: OTG_FS_PWRCLK_FS_PCGCCTL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type ETHERNET_MMC_MMCCR_Fields* = object
  CR* {.bitsize:1.}: bool
  CSR* {.bitsize:1.}: bool
  ROR* {.bitsize:1.}: bool
  RESERVED {.bitsize:28.}: 0'u .. 268435455'u
  MCF* {.bitsize:1.}: bool

type ETHERNET_MMC_MMCRIR_Fields* = object
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  RFCES* {.bitsize:1.}: bool
  RFAES* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:10.}: 0'u .. 1023'u
  RGUFS* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:14.}: 0'u .. 16383'u

type ETHERNET_MMC_MMCTIR_Fields* = object
  RESERVED {.bitsize:14.}: 0'u .. 16383'u
  TGFSCS* {.bitsize:1.}: bool
  TGFMSCS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  TGFS* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:10.}: 0'u .. 1023'u

type ETHERNET_MMC_MMCRIMR_Fields* = object
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  RFCEM* {.bitsize:1.}: bool
  RFAEM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:10.}: 0'u .. 1023'u
  RGUFM* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:14.}: 0'u .. 16383'u

type ETHERNET_MMC_MMCTIMR_Fields* = object
  RESERVED {.bitsize:14.}: 0'u .. 16383'u
  TGFSCM* {.bitsize:1.}: bool
  TGFMSCM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  TGFM* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:10.}: 0'u .. 1023'u

template read*(reg: ETHERNET_MMC_MMCCR_Type): ETHERNET_MMC_MMCCR_Fields =
  cast[ETHERNET_MMC_MMCCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MMC_MMCCR_Type, val: ETHERNET_MMC_MMCCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MMC_MMCCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MMC_MMCRIR_Type): ETHERNET_MMC_MMCRIR_Fields =
  cast[ETHERNET_MMC_MMCRIR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MMC_MMCRIR_Type, val: ETHERNET_MMC_MMCRIR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MMC_MMCRIR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MMC_MMCTIR_Type): ETHERNET_MMC_MMCTIR_Fields =
  cast[ETHERNET_MMC_MMCTIR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MMC_MMCTIR_Type, val: ETHERNET_MMC_MMCTIR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MMC_MMCTIR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MMC_MMCRIMR_Type): ETHERNET_MMC_MMCRIMR_Fields =
  cast[ETHERNET_MMC_MMCRIMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MMC_MMCRIMR_Type, val: ETHERNET_MMC_MMCRIMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MMC_MMCRIMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MMC_MMCTIMR_Type): ETHERNET_MMC_MMCTIMR_Fields =
  cast[ETHERNET_MMC_MMCTIMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MMC_MMCTIMR_Type, val: ETHERNET_MMC_MMCTIMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MMC_MMCTIMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MMC_MMCTGFSCCR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_MMC_MMCTGFMSCCR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_MMC_MMCTGFCR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_MMC_MMCRFCECR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_MMC_MMCRFAECR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_MMC_MMCRGUFCR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

type ETHERNET_MAC_MACCR_Fields* = object
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  RE* {.bitsize:1.}: bool
  TE* {.bitsize:1.}: bool
  DC* {.bitsize:1.}: bool
  BL* {.bitsize:2.}: 0'u .. 3'u
  APCS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  RD* {.bitsize:1.}: bool
  IPCO* {.bitsize:1.}: bool
  DM* {.bitsize:1.}: bool
  LM* {.bitsize:1.}: bool
  ROD* {.bitsize:1.}: bool
  FES* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  CSD* {.bitsize:1.}: bool
  IFG* {.bitsize:3.}: 0'u .. 7'u
  RESERVED3 {.bitsize:2.}: 0'u .. 3'u
  JD* {.bitsize:1.}: bool
  WD* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:8.}: 0'u .. 255'u

type ETHERNET_MAC_MACFFR_Fields* = object
  PM* {.bitsize:1.}: bool
  HU* {.bitsize:1.}: bool
  HM* {.bitsize:1.}: bool
  DAIF* {.bitsize:1.}: bool
  PAM* {.bitsize:1.}: bool
  BFD* {.bitsize:1.}: bool
  PCF* {.bitsize:2.}: 0'u .. 3'u
  SAIF* {.bitsize:1.}: bool
  SAF* {.bitsize:1.}: bool
  HPF* {.bitsize:1.}: bool
  RESERVED {.bitsize:20.}: 0'u .. 1048575'u
  RA* {.bitsize:1.}: bool

type ETHERNET_MAC_MACMIIAR_Fields* = object
  MB* {.bitsize:1.}: bool
  MW* {.bitsize:1.}: bool
  CR* {.bitsize:3.}: 0'u .. 7'u
  RESERVED {.bitsize:1.}: bool
  MR* {.bitsize:5.}: 0'u .. 31'u
  PA* {.bitsize:5.}: 0'u .. 31'u
  RESERVED1 {.bitsize:16.}: 0'u .. 65535'u

type ETHERNET_MAC_MACMIIDR_Fields* = object
  MD* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:16.}: 0'u .. 65535'u

type ETHERNET_MAC_MACFCR_Fields* = object
  FCB_BPA* {.bitsize:1.}: bool
  TFCE* {.bitsize:1.}: bool
  RFCE* {.bitsize:1.}: bool
  UPFD* {.bitsize:1.}: bool
  PLT* {.bitsize:2.}: 0'u .. 3'u
  RESERVED {.bitsize:1.}: bool
  ZQPD* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:8.}: 0'u .. 255'u
  PT* {.bitsize:16.}: 0'u .. 65535'u

type ETHERNET_MAC_MACVLANTR_Fields* = object
  VLANTI* {.bitsize:16.}: 0'u .. 65535'u
  VLANTC* {.bitsize:1.}: bool
  RESERVED {.bitsize:15.}: 0'u .. 32767'u

type ETHERNET_MAC_MACPMTCSR_Fields* = object
  PD* {.bitsize:1.}: bool
  MPE* {.bitsize:1.}: bool
  WFE* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  MPR* {.bitsize:1.}: bool
  WFR* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  GU* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:21.}: 0'u .. 2097151'u
  WFFRPR* {.bitsize:1.}: bool

type ETHERNET_MAC_MACSR_Fields* = object
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  PMTS* {.bitsize:1.}: bool
  MMCS* {.bitsize:1.}: bool
  MMCRS* {.bitsize:1.}: bool
  MMCTS* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  TSTS* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:22.}: 0'u .. 4194303'u

type ETHERNET_MAC_MACIMR_Fields* = object
  RESERVED {.bitsize:3.}: 0'u .. 7'u
  PMTIM* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:5.}: 0'u .. 31'u
  TSTIM* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:22.}: 0'u .. 4194303'u

type ETHERNET_MAC_MACA0HR_Fields* = object
  MACA0H* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:15.}: 0'u .. 32767'u
  MO* {.bitsize:1.}: bool

type ETHERNET_MAC_MACA1HR_Fields* = object
  MACA1H* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u
  MBC* {.bitsize:6.}: 0'u .. 63'u
  SA* {.bitsize:1.}: bool
  AE* {.bitsize:1.}: bool

type ETHERNET_MAC_MACA2HR_Fields* = object
  ETH_MACA2HR* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u
  MBC* {.bitsize:6.}: 0'u .. 63'u
  SA* {.bitsize:1.}: bool
  AE* {.bitsize:1.}: bool

type ETHERNET_MAC_MACA2LR_Fields* = object
  MACA2L* {.bitsize:31.}: 0'u .. 2147483647'u
  RESERVED {.bitsize:1.}: bool

type ETHERNET_MAC_MACA3HR_Fields* = object
  MACA3H* {.bitsize:16.}: 0'u .. 65535'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u
  MBC* {.bitsize:6.}: 0'u .. 63'u
  SA* {.bitsize:1.}: bool
  AE* {.bitsize:1.}: bool

template read*(reg: ETHERNET_MAC_MACCR_Type): ETHERNET_MAC_MACCR_Fields =
  cast[ETHERNET_MAC_MACCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACCR_Type, val: ETHERNET_MAC_MACCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACFFR_Type): ETHERNET_MAC_MACFFR_Fields =
  cast[ETHERNET_MAC_MACFFR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACFFR_Type, val: ETHERNET_MAC_MACFFR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACFFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACHTHR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_MAC_MACHTHR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_MAC_MACHTHR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACHTLR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_MAC_MACHTLR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_MAC_MACHTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACMIIAR_Type): ETHERNET_MAC_MACMIIAR_Fields =
  cast[ETHERNET_MAC_MACMIIAR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACMIIAR_Type, val: ETHERNET_MAC_MACMIIAR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACMIIAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACMIIDR_Type): ETHERNET_MAC_MACMIIDR_Fields =
  cast[ETHERNET_MAC_MACMIIDR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACMIIDR_Type, val: ETHERNET_MAC_MACMIIDR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACMIIDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACFCR_Type): ETHERNET_MAC_MACFCR_Fields =
  cast[ETHERNET_MAC_MACFCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACFCR_Type, val: ETHERNET_MAC_MACFCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACFCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACVLANTR_Type): ETHERNET_MAC_MACVLANTR_Fields =
  cast[ETHERNET_MAC_MACVLANTR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACVLANTR_Type, val: ETHERNET_MAC_MACVLANTR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACVLANTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACRWUFFR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_MAC_MACRWUFFR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_MAC_MACRWUFFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACPMTCSR_Type): ETHERNET_MAC_MACPMTCSR_Fields =
  cast[ETHERNET_MAC_MACPMTCSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACPMTCSR_Type, val: ETHERNET_MAC_MACPMTCSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACPMTCSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACSR_Type): ETHERNET_MAC_MACSR_Fields =
  cast[ETHERNET_MAC_MACSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACSR_Type, val: ETHERNET_MAC_MACSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACIMR_Type): ETHERNET_MAC_MACIMR_Fields =
  cast[ETHERNET_MAC_MACIMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACIMR_Type, val: ETHERNET_MAC_MACIMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACIMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA0HR_Type): ETHERNET_MAC_MACA0HR_Fields =
  cast[ETHERNET_MAC_MACA0HR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACA0HR_Type, val: ETHERNET_MAC_MACA0HR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACA0HR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA0LR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_MAC_MACA0LR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_MAC_MACA0LR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA1HR_Type): ETHERNET_MAC_MACA1HR_Fields =
  cast[ETHERNET_MAC_MACA1HR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACA1HR_Type, val: ETHERNET_MAC_MACA1HR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACA1HR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA1LR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_MAC_MACA1LR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_MAC_MACA1LR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA2HR_Type): ETHERNET_MAC_MACA2HR_Fields =
  cast[ETHERNET_MAC_MACA2HR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACA2HR_Type, val: ETHERNET_MAC_MACA2HR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACA2HR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA2LR_Type): ETHERNET_MAC_MACA2LR_Fields =
  cast[ETHERNET_MAC_MACA2LR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACA2LR_Type, val: ETHERNET_MAC_MACA2LR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACA2LR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA3HR_Type): ETHERNET_MAC_MACA3HR_Fields =
  cast[ETHERNET_MAC_MACA3HR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_MAC_MACA3HR_Type, val: ETHERNET_MAC_MACA3HR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_MAC_MACA3HR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_MAC_MACA3LR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_MAC_MACA3LR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_MAC_MACA3LR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type ETHERNET_PTP_PTPTSCR_Fields* = object
  TSE* {.bitsize:1.}: bool
  TSFCU* {.bitsize:1.}: bool
  TSSTI* {.bitsize:1.}: bool
  TSSTU* {.bitsize:1.}: bool
  TSITE* {.bitsize:1.}: bool
  TSARU* {.bitsize:1.}: bool
  RESERVED {.bitsize:26.}: 0'u .. 67108863'u

type ETHERNET_PTP_PTPSSIR_Fields* = object
  STSSI* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type ETHERNET_PTP_PTPTSLR_Fields* = object
  STSS* {.bitsize:31.}: 0'u .. 2147483647'u
  STPNS* {.bitsize:1.}: bool

type ETHERNET_PTP_PTPTSLUR_Fields* = object
  TSUSS* {.bitsize:31.}: 0'u .. 2147483647'u
  TSUPNS* {.bitsize:1.}: bool

template read*(reg: ETHERNET_PTP_PTPTSCR_Type): ETHERNET_PTP_PTPTSCR_Fields =
  cast[ETHERNET_PTP_PTPTSCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_PTP_PTPTSCR_Type, val: ETHERNET_PTP_PTPTSCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_PTP_PTPTSCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_PTP_PTPSSIR_Type): ETHERNET_PTP_PTPSSIR_Fields =
  cast[ETHERNET_PTP_PTPSSIR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_PTP_PTPSSIR_Type, val: ETHERNET_PTP_PTPSSIR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_PTP_PTPSSIR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_PTP_PTPTSHR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_PTP_PTPTSLR_Type): ETHERNET_PTP_PTPTSLR_Fields =
  cast[ETHERNET_PTP_PTPTSLR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ETHERNET_PTP_PTPTSHUR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_PTP_PTPTSHUR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_PTP_PTPTSHUR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_PTP_PTPTSLUR_Type): ETHERNET_PTP_PTPTSLUR_Fields =
  cast[ETHERNET_PTP_PTPTSLUR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_PTP_PTPTSLUR_Type, val: ETHERNET_PTP_PTPTSLUR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_PTP_PTPTSLUR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_PTP_PTPTSAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_PTP_PTPTSAR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_PTP_PTPTSAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_PTP_PTPTTHR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_PTP_PTPTTHR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_PTP_PTPTTHR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_PTP_PTPTTLR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_PTP_PTPTTLR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_PTP_PTPTTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type ETHERNET_DMA_DMABMR_Fields* = object
  SR* {.bitsize:1.}: bool
  DA* {.bitsize:1.}: bool
  DSL* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:1.}: bool
  PBL* {.bitsize:6.}: 0'u .. 63'u
  RTPR* {.bitsize:2.}: 0'u .. 3'u
  FB* {.bitsize:1.}: bool
  RDP* {.bitsize:6.}: 0'u .. 63'u
  USP* {.bitsize:1.}: bool
  FPM* {.bitsize:1.}: bool
  AAB* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:6.}: 0'u .. 63'u

type ETHERNET_DMA_DMASR_Fields* = object
  TS* {.bitsize:1.}: bool
  TPSS* {.bitsize:1.}: bool
  TBUS* {.bitsize:1.}: bool
  TJTS* {.bitsize:1.}: bool
  ROS* {.bitsize:1.}: bool
  TUS* {.bitsize:1.}: bool
  RS* {.bitsize:1.}: bool
  RBUS* {.bitsize:1.}: bool
  RPSS* {.bitsize:1.}: bool
  PWTS* {.bitsize:1.}: bool
  ETS* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  FBES* {.bitsize:1.}: bool
  ERS* {.bitsize:1.}: bool
  AIS* {.bitsize:1.}: bool
  NIS* {.bitsize:1.}: bool
  RPS* {.bitsize:3.}: 0'u .. 7'u
  TPS* {.bitsize:3.}: 0'u .. 7'u
  EBS* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:1.}: bool
  MMCS* {.bitsize:1.}: bool
  PMTS* {.bitsize:1.}: bool
  TSTS* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:2.}: 0'u .. 3'u

type ETHERNET_DMA_DMAOMR_Fields* = object
  RESERVED {.bitsize:1.}: bool
  SR* {.bitsize:1.}: bool
  OSF* {.bitsize:1.}: bool
  RTC* {.bitsize:2.}: 0'u .. 3'u
  RESERVED1 {.bitsize:1.}: bool
  FUGF* {.bitsize:1.}: bool
  FEF* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:5.}: 0'u .. 31'u
  ST* {.bitsize:1.}: bool
  TTC* {.bitsize:3.}: 0'u .. 7'u
  RESERVED3 {.bitsize:3.}: 0'u .. 7'u
  FTF* {.bitsize:1.}: bool
  TSF* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:2.}: 0'u .. 3'u
  DFRF* {.bitsize:1.}: bool
  RSF* {.bitsize:1.}: bool
  DTCEFD* {.bitsize:1.}: bool
  RESERVED5 {.bitsize:5.}: 0'u .. 31'u

type ETHERNET_DMA_DMAIER_Fields* = object
  TIE* {.bitsize:1.}: bool
  TPSIE* {.bitsize:1.}: bool
  TBUIE* {.bitsize:1.}: bool
  TJTIE* {.bitsize:1.}: bool
  ROIE* {.bitsize:1.}: bool
  TUIE* {.bitsize:1.}: bool
  RIE* {.bitsize:1.}: bool
  RBUIE* {.bitsize:1.}: bool
  RPSIE* {.bitsize:1.}: bool
  RWTIE* {.bitsize:1.}: bool
  ETIE* {.bitsize:1.}: bool
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  FBEIE* {.bitsize:1.}: bool
  ERIE* {.bitsize:1.}: bool
  AISE* {.bitsize:1.}: bool
  NISE* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:15.}: 0'u .. 32767'u

type ETHERNET_DMA_DMAMFBOCR_Fields* = object
  MFC* {.bitsize:16.}: 0'u .. 65535'u
  OMFC* {.bitsize:1.}: bool
  MFA* {.bitsize:11.}: 0'u .. 2047'u
  OFOC* {.bitsize:1.}: bool
  RESERVED {.bitsize:3.}: 0'u .. 7'u

template read*(reg: ETHERNET_DMA_DMABMR_Type): ETHERNET_DMA_DMABMR_Fields =
  cast[ETHERNET_DMA_DMABMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_DMA_DMABMR_Type, val: ETHERNET_DMA_DMABMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_DMA_DMABMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMATPDR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_DMA_DMATPDR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_DMA_DMATPDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMARPDR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_DMA_DMARPDR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_DMA_DMARPDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMARDLAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_DMA_DMARDLAR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_DMA_DMARDLAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMATDLAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: ETHERNET_DMA_DMATDLAR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: ETHERNET_DMA_DMATDLAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMASR_Type): ETHERNET_DMA_DMASR_Fields =
  cast[ETHERNET_DMA_DMASR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_DMA_DMASR_Type, val: ETHERNET_DMA_DMASR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_DMA_DMASR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMAOMR_Type): ETHERNET_DMA_DMAOMR_Fields =
  cast[ETHERNET_DMA_DMAOMR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_DMA_DMAOMR_Type, val: ETHERNET_DMA_DMAOMR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_DMA_DMAOMR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMAIER_Type): ETHERNET_DMA_DMAIER_Fields =
  cast[ETHERNET_DMA_DMAIER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: ETHERNET_DMA_DMAIER_Type, val: ETHERNET_DMA_DMAIER_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: ETHERNET_DMA_DMAIER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: ETHERNET_DMA_DMAMFBOCR_Type): ETHERNET_DMA_DMAMFBOCR_Fields =
  cast[ETHERNET_DMA_DMAMFBOCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: ETHERNET_DMA_DMACHTDR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_DMA_DMACHRDR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_DMA_DMACHTBAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: ETHERNET_DMA_DMACHRBAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

type NVIC_IPR0_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR1_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR2_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR3_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR4_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR5_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR6_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR7_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR8_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR9_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR10_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR11_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR12_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR13_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

type NVIC_IPR14_Fields* = object
  IPR_N0* {.bitsize:8.}: 0'u .. 255'u
  IPR_N1* {.bitsize:8.}: 0'u .. 255'u
  IPR_N2* {.bitsize:8.}: 0'u .. 255'u
  IPR_N3* {.bitsize:8.}: 0'u .. 255'u

template read*(reg: NVIC_ISER0_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ISER0_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ISER0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ISER1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ISER1_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ISER1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ICER0_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ICER0_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ICER0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ICER1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ICER1_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ICER1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ISPR0_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ISPR0_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ISPR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ISPR1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ISPR1_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ISPR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ICPR0_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ICPR0_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ICPR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_ICPR1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: NVIC_ICPR1_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: NVIC_ICPR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IABR0_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: NVIC_IABR1_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template read*(reg: NVIC_IPR0_Type): NVIC_IPR0_Fields =
  cast[NVIC_IPR0_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR0_Type, val: NVIC_IPR0_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR1_Type): NVIC_IPR1_Fields =
  cast[NVIC_IPR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR1_Type, val: NVIC_IPR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR2_Type): NVIC_IPR2_Fields =
  cast[NVIC_IPR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR2_Type, val: NVIC_IPR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR3_Type): NVIC_IPR3_Fields =
  cast[NVIC_IPR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR3_Type, val: NVIC_IPR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR4_Type): NVIC_IPR4_Fields =
  cast[NVIC_IPR4_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR4_Type, val: NVIC_IPR4_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR5_Type): NVIC_IPR5_Fields =
  cast[NVIC_IPR5_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR5_Type, val: NVIC_IPR5_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR6_Type): NVIC_IPR6_Fields =
  cast[NVIC_IPR6_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR6_Type, val: NVIC_IPR6_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR7_Type): NVIC_IPR7_Fields =
  cast[NVIC_IPR7_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR7_Type, val: NVIC_IPR7_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR8_Type): NVIC_IPR8_Fields =
  cast[NVIC_IPR8_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR8_Type, val: NVIC_IPR8_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR8_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR9_Type): NVIC_IPR9_Fields =
  cast[NVIC_IPR9_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR9_Type, val: NVIC_IPR9_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR9_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR10_Type): NVIC_IPR10_Fields =
  cast[NVIC_IPR10_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR10_Type, val: NVIC_IPR10_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR10_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR11_Type): NVIC_IPR11_Fields =
  cast[NVIC_IPR11_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR11_Type, val: NVIC_IPR11_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR11_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR12_Type): NVIC_IPR12_Fields =
  cast[NVIC_IPR12_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR12_Type, val: NVIC_IPR12_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR12_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR13_Type): NVIC_IPR13_Fields =
  cast[NVIC_IPR13_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR13_Type, val: NVIC_IPR13_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR13_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: NVIC_IPR14_Type): NVIC_IPR14_Fields =
  cast[NVIC_IPR14_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_IPR14_Type, val: NVIC_IPR14_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_IPR14_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type MPU_MPU_TYPER_Fields* = object
  SEPARATE* {.bitsize:1.}: bool
  RESERVED {.bitsize:7.}: 0'u .. 127'u
  DREGION* {.bitsize:8.}: 0'u .. 255'u
  IREGION* {.bitsize:8.}: 0'u .. 255'u
  RESERVED1 {.bitsize:8.}: 0'u .. 255'u

type MPU_MPU_CTRL_Fields* = object
  ENABLE* {.bitsize:1.}: bool
  HFNMIENA* {.bitsize:1.}: bool
  PRIVDEFENA* {.bitsize:1.}: bool
  RESERVED {.bitsize:29.}: 0'u .. 536870911'u

type MPU_MPU_RNR_Fields* = object
  REGION* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u

type MPU_MPU_RBAR_Fields* = object
  REGION* {.bitsize:4.}: 0'u .. 15'u
  VALID* {.bitsize:1.}: bool
  ADDRx* {.bitsize:27.}: 0'u .. 134217727'u

type MPU_MPU_RASR_Fields* = object
  ENABLE* {.bitsize:1.}: bool
  SIZE* {.bitsize:5.}: 0'u .. 31'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  SRD* {.bitsize:8.}: 0'u .. 255'u
  B* {.bitsize:1.}: bool
  C* {.bitsize:1.}: bool
  S* {.bitsize:1.}: bool
  TEX* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u
  AP* {.bitsize:3.}: 0'u .. 7'u
  RESERVED2 {.bitsize:1.}: bool
  XN* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:3.}: 0'u .. 7'u

template read*(reg: MPU_MPU_TYPER_Type): MPU_MPU_TYPER_Fields =
  cast[MPU_MPU_TYPER_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: MPU_MPU_CTRL_Type): MPU_MPU_CTRL_Fields =
  cast[MPU_MPU_CTRL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: MPU_MPU_RNR_Type): MPU_MPU_RNR_Fields =
  cast[MPU_MPU_RNR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: MPU_MPU_RNR_Type, val: MPU_MPU_RNR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: MPU_MPU_RNR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: MPU_MPU_RBAR_Type): MPU_MPU_RBAR_Fields =
  cast[MPU_MPU_RBAR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: MPU_MPU_RBAR_Type, val: MPU_MPU_RBAR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: MPU_MPU_RBAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: MPU_MPU_RASR_Type): MPU_MPU_RASR_Fields =
  cast[MPU_MPU_RASR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: MPU_MPU_RASR_Type, val: MPU_MPU_RASR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: MPU_MPU_RASR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type SCB_ACTRL_ACTRL_Fields* = object
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  DISFOLD* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:7.}: 0'u .. 127'u
  FPEXCODIS* {.bitsize:1.}: bool
  DISRAMODE* {.bitsize:1.}: bool
  DISITMATBFLUSH* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:19.}: 0'u .. 524287'u

template read*(reg: SCB_ACTRL_ACTRL_Type): SCB_ACTRL_ACTRL_Fields =
  cast[SCB_ACTRL_ACTRL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_ACTRL_ACTRL_Type, val: SCB_ACTRL_ACTRL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_ACTRL_ACTRL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type NVIC_STIR_STIR_Fields* = object
  INTID* {.bitsize:9.}: 0'u .. 511'u
  RESERVED {.bitsize:23.}: 0'u .. 8388607'u

template read*(reg: NVIC_STIR_STIR_Type): NVIC_STIR_STIR_Fields =
  cast[NVIC_STIR_STIR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: NVIC_STIR_STIR_Type, val: NVIC_STIR_STIR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: NVIC_STIR_STIR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type SCB_CPUID_Fields* = object
  Revision* {.bitsize:4.}: 0'u .. 15'u
  PartNo* {.bitsize:12.}: 0'u .. 4095'u
  Constant* {.bitsize:4.}: 0'u .. 15'u
  Variant* {.bitsize:4.}: 0'u .. 15'u
  Implementer* {.bitsize:8.}: 0'u .. 255'u

type SCB_ICSR_Fields* = object
  VECTACTIVE* {.bitsize:9.}: 0'u .. 511'u
  RESERVED {.bitsize:2.}: 0'u .. 3'u
  RETTOBASE* {.bitsize:1.}: bool
  VECTPENDING* {.bitsize:7.}: 0'u .. 127'u
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  ISRPENDING* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:2.}: 0'u .. 3'u
  PENDSTCLR* {.bitsize:1.}: bool
  PENDSTSET* {.bitsize:1.}: bool
  PENDSVCLR* {.bitsize:1.}: bool
  PENDSVSET* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:2.}: 0'u .. 3'u
  NMIPENDSET* {.bitsize:1.}: bool

type SCB_VTOR_Fields* = object
  RESERVED {.bitsize:9.}: 0'u .. 511'u
  TBLOFF* {.bitsize:21.}: 0'u .. 2097151'u
  RESERVED1 {.bitsize:2.}: 0'u .. 3'u

type SCB_AIRCR_Fields* = object
  VECTRESET* {.bitsize:1.}: bool
  VECTCLRACTIVE* {.bitsize:1.}: bool
  SYSRESETREQ* {.bitsize:1.}: bool
  RESERVED {.bitsize:5.}: 0'u .. 31'u
  PRIGROUP* {.bitsize:3.}: 0'u .. 7'u
  RESERVED1 {.bitsize:4.}: 0'u .. 15'u
  ENDIANESS* {.bitsize:1.}: bool
  VECTKEYSTAT* {.bitsize:16.}: 0'u .. 65535'u

type SCB_SCR_Fields* = object
  RESERVED {.bitsize:1.}: bool
  SLEEPONEXIT* {.bitsize:1.}: bool
  SLEEPDEEP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  SEVEONPEND* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:27.}: 0'u .. 134217727'u

type SCB_CCR_Fields* = object
  NONBASETHRDENA* {.bitsize:1.}: bool
  USERSETMPEND* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  UNALIGN_TRP* {.bitsize:1.}: bool
  DIV_0_TRP* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  BFHFNMIGN* {.bitsize:1.}: bool
  STKALIGN* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:22.}: 0'u .. 4194303'u

type SCB_SHPR1_Fields* = object
  PRI_4* {.bitsize:8.}: 0'u .. 255'u
  PRI_5* {.bitsize:8.}: 0'u .. 255'u
  PRI_6* {.bitsize:8.}: 0'u .. 255'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type SCB_SHPR2_Fields* = object
  RESERVED {.bitsize:24.}: 0'u .. 16777215'u
  PRI_11* {.bitsize:8.}: 0'u .. 255'u

type SCB_SHPR3_Fields* = object
  RESERVED {.bitsize:16.}: 0'u .. 65535'u
  PRI_14* {.bitsize:8.}: 0'u .. 255'u
  PRI_15* {.bitsize:8.}: 0'u .. 255'u

type SCB_SHCRS_Fields* = object
  MEMFAULTACT* {.bitsize:1.}: bool
  BUSFAULTACT* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  USGFAULTACT* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:3.}: 0'u .. 7'u
  SVCALLACT* {.bitsize:1.}: bool
  MONITORACT* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  PENDSVACT* {.bitsize:1.}: bool
  SYSTICKACT* {.bitsize:1.}: bool
  USGFAULTPENDED* {.bitsize:1.}: bool
  MEMFAULTPENDED* {.bitsize:1.}: bool
  BUSFAULTPENDED* {.bitsize:1.}: bool
  SVCALLPENDED* {.bitsize:1.}: bool
  MEMFAULTENA* {.bitsize:1.}: bool
  BUSFAULTENA* {.bitsize:1.}: bool
  USGFAULTENA* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:13.}: 0'u .. 8191'u

type SCB_CFSR_UFSR_BFSR_MMFSR_Fields* = object
  IACCVIOL* {.bitsize:1.}: bool
  DACCVIOL* {.bitsize:1.}: bool
  RESERVED {.bitsize:1.}: bool
  MUNSTKERR* {.bitsize:1.}: bool
  MSTKERR* {.bitsize:1.}: bool
  MLSPERR* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:1.}: bool
  MMARVALID* {.bitsize:1.}: bool
  IBUSERR* {.bitsize:1.}: bool
  PRECISERR* {.bitsize:1.}: bool
  IMPRECISERR* {.bitsize:1.}: bool
  UNSTKERR* {.bitsize:1.}: bool
  STKERR* {.bitsize:1.}: bool
  LSPERR* {.bitsize:1.}: bool
  RESERVED2 {.bitsize:1.}: bool
  BFARVALID* {.bitsize:1.}: bool
  UNDEFINSTR* {.bitsize:1.}: bool
  INVSTATE* {.bitsize:1.}: bool
  INVPC* {.bitsize:1.}: bool
  NOCP* {.bitsize:1.}: bool
  RESERVED3 {.bitsize:4.}: 0'u .. 15'u
  UNALIGNED* {.bitsize:1.}: bool
  DIVBYZERO* {.bitsize:1.}: bool
  RESERVED4 {.bitsize:6.}: 0'u .. 63'u

type SCB_HFSR_Fields* = object
  RESERVED {.bitsize:1.}: bool
  VECTTBL* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:28.}: 0'u .. 268435455'u
  FORCED* {.bitsize:1.}: bool
  DEBUG_VT* {.bitsize:1.}: bool

template read*(reg: SCB_CPUID_Type): SCB_CPUID_Fields =
  cast[SCB_CPUID_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template read*(reg: SCB_ICSR_Type): SCB_ICSR_Fields =
  cast[SCB_ICSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_ICSR_Type, val: SCB_ICSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_ICSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_VTOR_Type): SCB_VTOR_Fields =
  cast[SCB_VTOR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_VTOR_Type, val: SCB_VTOR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_VTOR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_AIRCR_Type): SCB_AIRCR_Fields =
  cast[SCB_AIRCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_AIRCR_Type, val: SCB_AIRCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_AIRCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_SCR_Type): SCB_SCR_Fields =
  cast[SCB_SCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_SCR_Type, val: SCB_SCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_SCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_CCR_Type): SCB_CCR_Fields =
  cast[SCB_CCR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_CCR_Type, val: SCB_CCR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_CCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_SHPR1_Type): SCB_SHPR1_Fields =
  cast[SCB_SHPR1_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_SHPR1_Type, val: SCB_SHPR1_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_SHPR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_SHPR2_Type): SCB_SHPR2_Fields =
  cast[SCB_SHPR2_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_SHPR2_Type, val: SCB_SHPR2_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_SHPR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_SHPR3_Type): SCB_SHPR3_Fields =
  cast[SCB_SHPR3_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_SHPR3_Type, val: SCB_SHPR3_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_SHPR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_SHCRS_Type): SCB_SHCRS_Fields =
  cast[SCB_SHCRS_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_SHCRS_Type, val: SCB_SHCRS_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_SHCRS_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_CFSR_UFSR_BFSR_MMFSR_Type): SCB_CFSR_UFSR_BFSR_MMFSR_Fields =
  cast[SCB_CFSR_UFSR_BFSR_MMFSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_CFSR_UFSR_BFSR_MMFSR_Type, val: SCB_CFSR_UFSR_BFSR_MMFSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_CFSR_UFSR_BFSR_MMFSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_HFSR_Type): SCB_HFSR_Fields =
  cast[SCB_HFSR_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: SCB_HFSR_Type, val: SCB_HFSR_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: SCB_HFSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_MMFAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: SCB_MMFAR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: SCB_MMFAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: SCB_BFAR_Type): uint32 =
  volatileLoad(cast[ptr uint32](reg.loc))

template write*(reg: SCB_BFAR_Type, val: uint32) =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: SCB_BFAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

type STK_CTRL_Fields* = object
  ENABLE* {.bitsize:1.}: bool
  TICKINT* {.bitsize:1.}: bool
  CLKSOURCE* {.bitsize:1.}: bool
  RESERVED {.bitsize:13.}: 0'u .. 8191'u
  COUNTFLAG* {.bitsize:1.}: bool
  RESERVED1 {.bitsize:15.}: 0'u .. 32767'u

type STK_LOAD_Fields* = object
  RELOAD* {.bitsize:24.}: 0'u .. 16777215'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type STK_VAL_Fields* = object
  CURRENT* {.bitsize:24.}: 0'u .. 16777215'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

type STK_CALIB_Fields* = object
  TENMS* {.bitsize:24.}: 0'u .. 16777215'u
  RESERVED {.bitsize:8.}: 0'u .. 255'u

template read*(reg: STK_CTRL_Type): STK_CTRL_Fields =
  cast[STK_CTRL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: STK_CTRL_Type, val: STK_CTRL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: STK_CTRL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: STK_LOAD_Type): STK_LOAD_Fields =
  cast[STK_LOAD_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: STK_LOAD_Type, val: STK_LOAD_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: STK_LOAD_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: STK_VAL_Type): STK_VAL_Fields =
  cast[STK_VAL_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: STK_VAL_Type, val: STK_VAL_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: STK_VAL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

template read*(reg: STK_CALIB_Type): STK_CALIB_Fields =
  cast[STK_CALIB_Fields](volatileLoad(cast[ptr uint32](reg.loc)))

template write*(reg: STK_CALIB_Type, val: STK_CALIB_Fields) =
  volatileStore(cast[ptr uint32](reg.loc), cast[uint32](val))

template modifyIt*(reg: STK_CALIB_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

