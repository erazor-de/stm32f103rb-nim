# GPIOx_CRx_MODEx
const Input* = 0b00
const Output_10* = 0b01
const Output_2* = 0b10
const Output_50* = 0b11

# GPIOx_CRx_CNFx_INPUT
const Analog* = 0b00
const FLoating* = 0b01
const Pull* = 0b10

# GPIOx_CRx_CNFx_OUTPUT
const PushPull* = 0b00
const OpenDrain* = 0b01
const AltPushPull* = 0x10
const AltOpenDrain* = 0x11

# GPIOx_ODR_ODRx
const PullUp* = true
const PullDown* = false
