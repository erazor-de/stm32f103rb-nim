target extended-remote /dev/ttyBmpGdb
set mem inaccessible-by-default off
monitor swdp_scan
attach 1
load
#continue
