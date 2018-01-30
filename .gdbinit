target extended /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_B6D79407-if00
monitor swdp_scan
attach 1

#b up_hardfault
b AP_HAL::panic
b HardFault_Handler
b chSysHalt
set confirm off
