/* Linker script to configure memory regions. */

MEMORY
{
  /* 4k MBR, 104k Softdevice S110, 4K Application Data, 20k Bootloader, 1k Bootloader Settings
  
  Update: on the actual devices shipped from the MFG the bootloader starts at 0x3b000
  And the soft device ends at 0x18000.   
  
  And we want to use 4KB for data storage (at the end of the app region) FIXME not sure if I should
  keep this in fs_data?
  
  But the mfg was dumb and used the 2 bank bootloader, so we are limited to half that space
  
  old length: 256k - 4k - 104k - 20k - 1k
  new length: (0x3b000 − 0x18000 - 0x1000)÷2 = 69632
  */
  FLASH (rx) : ORIGIN = 0x18000, LENGTH = 69632
  /* 11k Softdevice S130 */
  RAM (rwx) :  ORIGIN = 0x20002C00, LENGTH = 32k - 11k
}

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(.pwr_mgmt_data))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
