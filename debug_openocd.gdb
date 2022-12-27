target extended-remote localhost:3333
set mem inaccessible-by-default off
monitor arm semihosting enable
load
continue
