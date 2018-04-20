# bf70x-bootloader
openocd bootloader sketch for bf70x boards

This is a bootloader for easily writing to the external spi flash on a bf70x blackfin board. It depends on my fork of openocd that supports blackfin plus processors, as well as the arduino core for blackfin plus processors. 

This bootloader file is first written to instruction RAM on the processor. The bootloader initializes the flash and creates a virtual NVM flash interface in data RAM B. Openocd will read and write from these virtual registers.

Example usage:
```
openocd.exe -f bf706-ez-loader.cfg -c "load_bootloader bootloader.bin; flash write_image write_this_to_flash.ldr 0 bin; shutdown
```
