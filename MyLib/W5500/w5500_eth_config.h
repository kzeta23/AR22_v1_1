/**
 * @file    w5500_eth_config.h
 * @brief   AR22 build configuration for w5500_eth_lib.
 */
#ifndef W5500_ETH_CONFIG_H_
#define W5500_ETH_CONFIG_H_

/* SPI clock is set by the HAL SPI1 prescaler in main.c (MX_SPI1_Init),
 * not by this value. Kept for reference (informational only). */
#define W5500_SPI_CLOCK_HZ          12500000U   /* APB2 100MHz / 8 */

/* Link debouncing: ignore spurious PHYCFGR DOWN caused by SPI bit errors. */
#define W5500_LINK_POLL_INTERVAL_MS 200U
#define W5500_LINK_DEBOUNCE_COUNT   5U

/* W5500 has 8 hardware sockets (0..7). */
#define W5500_SOCKET_COUNT          8U

/* Debug log: 0=off, 1=errors, 2=info, 3=spi. Routed to printf (USART2). */
#define W5500_LOG_LEVEL             1

#if W5500_LOG_LEVEL > 0
  #include <stdio.h>
  #define W5500_LOG(fmt, ...)  printf("[w5500] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define W5500_LOG(fmt, ...)  ((void)0)
#endif

#endif /* W5500_ETH_CONFIG_H_ */
