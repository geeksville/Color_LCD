#ifndef INCLUDE_DEFINITIONS_H_
#define INCLUDE_DEFINITIONS_H_

// #include "boards.h"
#include "custom_board.h"
#include "sdk_config.h"
#include "app_error.h"
#include "app_util_platform.h"

// #include "nrf_drv_config.h"
#if (SPI0_ENABLED == 1)
#define SPI0_USE_EASY_DMA 0

#define SPI0_CONFIG_SCK_PIN         2
#define SPI0_CONFIG_MOSI_PIN        3
#define SPI0_CONFIG_MISO_PIN        4
#define SPI0_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPI0_INSTANCE_INDEX 0
#endif

/*  Peripheral usage by SoftDevice 130 */
/* Blocked
 *  TIMER0
 *  RTC0
 *  CCM
 *  AAR
 *  SWI2, 4, 5
 *  FICR
*/
/* Restricted
 *  TEMP
 *  RNG
 *  ECB
 *  SWI1
 *  NVMC
 *  UICR
 *  NVIC
 */

/* TIMER / RTC */
#define APP_TIMER_PRESCALER       0   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE   4   /**< Size of timer operation queues. */

/* SPI CPU -> LCD controller */
#define LCD_SPI_INSTANCE    0

/* UART Display <-> Motor */
#define UART0   0

#endif /* INCLUDE_DEFINITIONS_H_ */
