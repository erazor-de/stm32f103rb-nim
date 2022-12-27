# This module is required for --os:standalone to work

# Subset of semihosting
proc printf(format: cstring): cint {.importc, varargs, header: "stdio.h".}

{.push stack_trace: off, profiler:off.}

proc rawoutput(s: cstring) =
  discard printf("%s\l", s)

proc panic(s: cstring) {.noreturn.} =
  rawoutput(s)
  while true: discard

{.pop.}
