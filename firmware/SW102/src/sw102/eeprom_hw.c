/*
 * Bafang LCD SW102 Bluetooth firmware
 *
 * Copyright (C) lowPerformer, 2019.
 *
 * Released under the GPL License, Version 3
 */

#include <string.h>
#include "section_vars.h"
#include "eeprom_hw.h"
#include "common.h"
#include "fds.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "assert.h"

// volatile fs_ret_t last_fs_ret;

/** Note: on SD110 bootloader starts at 0x3C000, but we want this hack to work on
 * SD130 (during development) also.  So we use the lower address for that bootloader
 *
 */
#define BOOTLOADER_START 0x3AC00
#define DATAFLASH_SIZE   4096
#define DATAFLASH_START  (BOOTLOADER_START - DATAFLASH_SIZE)
#define DATAFLASH_MAGIC  0x04954403

/** @brief Function for erasing a page in flash. (from flashwrite example in SDK9)
 *
 * @param page_address Address of the first word in the page to be erased.
 */
static void flash_page_erase(uint32_t * page_address)
{
    // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
static void flash_word_write(uint32_t * address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

static bool flash_write_words_raw(const void *value, uint16_t length_words)
{
  // FIXME, someday do flippy flop between two pages, but this write is only one page so
  // the chance of power loss during this super quick write is low.

  uint32_t *flash = (uint32_t *) DATAFLASH_START;
  flash_page_erase(flash);

  const uint32_t *src = value;
  for(int i = 0; i < length_words; i++)
    flash_word_write(&flash[i+1], src[i]);
  flash_word_write(flash, DATAFLASH_MAGIC);
  return true;
}

static bool flash_read_words_raw(void *dest, uint16_t length_words)
{
  uint32_t *flash = (uint32_t *) DATAFLASH_START;

  if(flash[0] != DATAFLASH_MAGIC)
    return false; // must be blank

  assert(length_words * sizeof(uint32_t) + 4 <= DATAFLASH_SIZE);
  memcpy(dest, &flash[1], length_words * sizeof(uint32_t));

  return true;
}

/* Event handler */

volatile static bool gc_done, init_done, write_done;

/* Register fs_sys_event_handler with softdevice_sys_evt_handler_set in ble_stack_init or this doesn't fire! */
static void fds_evt_handler(fds_evt_t const *const evt)
{
  switch (evt->id)
  {
  case FDS_EVT_INIT:
    APP_ERROR_CHECK(evt->result);
    init_done = true;
    break;
  case FDS_EVT_GC:
    gc_done = true;
    break;
  case FDS_EVT_UPDATE:
  case FDS_EVT_WRITE:
    write_done = true;
    break;
  case FDS_EVT_DEL_RECORD:
    break;

  default:
    break;
  }
}

#define FILE_ID     0x1001
#define REC_KEY     0x2002

// read using the soft device
// returns true if our preferences were found
static bool flash_read_words_sd(void *dest, uint16_t length_words)
{
  fds_flash_record_t flash_record;
  fds_record_desc_t record_desc;
  fds_find_token_t ftok;

  bool did_read = false;

  memset(&record_desc, 0x00, sizeof(record_desc));
  memset(&ftok, 0x00, sizeof(ftok));
  // Loop until all records with the given key and file ID have been found.
  while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
  {
    if(!did_read) {
      // Found our first match (there should be only one unless someone else screwed up)

      APP_ERROR_CHECK(fds_record_open(&record_desc, &flash_record));

      // Access the record through the flash_record structure.
      memcpy(dest, flash_record.p_data, length_words * sizeof(uint32_t));
      did_read = true;

      // Close the record when done.
      APP_ERROR_CHECK(fds_record_close(&record_desc));
    }
    else {
      // Found a second record with the same key, delete it to prevent confusion when we go to write
      APP_ERROR_CHECK(fds_record_delete(&record_desc));
    }
  }

  return did_read;
}

bool flash_read_words(void *dest, uint16_t length_words)
{
  return useSoftDevice ?
    flash_read_words_sd(dest, length_words) :
    flash_read_words_raw(dest, length_words);
}

static bool wait_gc()
{
  if(!useSoftDevice)
    return true; // assume success

  gc_done = false;
  fds_gc();
  for (volatile int count = 0; count < 1000 && !gc_done; count++) {
    sd_app_evt_wait();
    nrf_delay_ms(1);
  }
  // Note: this can fail if the soft device is not enabled (normally performed in ble init)
  // assert(gc_done);
  return gc_done;
}


/// write using the sd
static bool flash_write_words_sd(const void *value, uint16_t length_words)
{
  fds_record_t record;
  fds_record_desc_t record_desc;
  fds_record_chunk_t record_chunk;
  fds_find_token_t ftok;

  if(!useSoftDevice)
    return true; // FIXME: for this test of working without soft device, we let writes to flash silently fail

  wait_gc(); // Before writing we always GC (to ensure there is at least one free record we can use)

  // Do we already have one of these records?
  memset(&record_desc, 0x00, sizeof(record_desc));
  memset(&ftok, 0x00, sizeof(ftok));
  bool has_old = fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok)
      == FDS_SUCCESS;

// Set up data.
  record_chunk.p_data = value;
  record_chunk.length_words = length_words;

// Set up record.
  record.file_id = FILE_ID;
  record.key = REC_KEY;
  record.data.p_chunks = &record_chunk;
  record.data.num_chunks = 1;

  write_done = false;

  // either make a new record or update an old one (if we lose power during update the old record is preserved)
  ret_code_t retcode = (has_old) ?
    fds_record_update(&record_desc, &record)
  :
    fds_record_write(&record_desc, &record);
  // Note - we intentionally don't check error codes here, because the developer might have turned off softdevice for debugging
  // APP_ERROR_CHECK(retcode);
  (void) retcode;

  for (volatile int count = 0; count < 1000 && !write_done; count++) {
    sd_app_evt_wait();
    nrf_delay_ms(1);
  }

  return write_done;
}



bool flash_write_words(const void *value, uint16_t length_words)
{
  return useSoftDevice ?
    flash_write_words_sd(value, length_words) :
    flash_write_words_raw(value, length_words);
}

/**
 * @brief Init eeprom emulation system
 */
void eeprom_hw_init(void)
{
  if(!useSoftDevice)
    return; // assume success

  APP_ERROR_CHECK(fds_register(fds_evt_handler));

  APP_ERROR_CHECK(fds_init());

  for (volatile int count = 0; count < 1000 && !init_done; count++) {
    sd_app_evt_wait();
    nrf_delay_ms(1);
  }
  // Note: this can fail if the soft device is not enabled (normally performed in ble init)
  // assert(init_done);

  wait_gc();
}

