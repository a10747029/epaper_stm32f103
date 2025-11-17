openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg -c 'init' -c 'program nuttx.bin 0x08000000 verify reset'  -c 'shutdown'
