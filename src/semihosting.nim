when (NimMajor, NimMinor, NimPatch) >= (0, 19, 1):
  import std/syncio
else:
  import system/io

proc initialise_monitor_handles*() {.importc.}

proc fopen*(name, mode: cstring): File {.importc, header: "stdio.h".}
proc fclose*(file: File): cint {.importc, header: "stdio.h".}
proc fflush*(file: File): cint {.importc, header: "stdio.h".}
proc fprintf*(file: File, format: cstring): cint {.importc, varargs, header: "stdio.h".}
proc fwrite*(s: cstring, size: uint32, n: uint32, file: File): uint32 {.importc, header: "stdio.h".}

proc printf*(format: cstring): cint {.importc, varargs, header: "stdio.h".}
proc putchar*(c: cint): cint {.importc, header: "stdio.h".}

# These do not seem to work - reads do not block
proc scanf*(format: cstring): cint {.importc, varargs, header: "stdio.h".}
proc getchar*(): cint {.importc, header: "stdio.h".}
