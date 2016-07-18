target extended-remote /dev/cu.usbmodemBDEAA9F1
monitor swdp_scan
attach 1
set confirm off
define lc
  load
  continue
end
define lr
  load
  run
end
define ls
  load
  start
end
