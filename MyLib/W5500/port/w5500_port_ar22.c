/**
 * @file    w5500_port_ar22.c
 * @brief   AR22 W5500 port (HAL): SPI1, CS=PA4 (ETH_CS), RST=PC4 (ETH_RST).
 *          Implements the w5500_port.h interface used by the ioLibrary glue.
 */
#include "port/w5500_port.h"
#include "port/w5500_port_ar22.h"
#include "stm32f4xx_hal.h"

/* SPI1 handle is created/initialised by MX_SPI1_Init() in main.c */
extern SPI_HandleTypeDef hspi1;

/* AR22 new-board-rev pin map */
#define W5500_CS_PORT     GPIOA
#define W5500_CS_PIN      GPIO_PIN_4      /* ETH_CS  */
#define W5500_RST_PORT    GPIOC
#define W5500_RST_PIN     GPIO_PIN_4      /* ETH_RST */

#define W5500_SPI_TIMEOUT 100U            /* ms */

void w5500_port_ar22_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* CS (PA4): push-pull output, idle HIGH (deselected) */
    g.Pin   = W5500_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(W5500_CS_PORT, &g);
    HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);

    /* RST (PC4): push-pull output, idle HIGH (reset inactive, active-low) */
    g.Pin = W5500_RST_PIN;
    HAL_GPIO_Init(W5500_RST_PORT, &g);
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);
}

/* ───── SPI (single byte, full-duplex) ───── */
uint8_t w5500_port_spi_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, W5500_SPI_TIMEOUT);
    return rx;
}

void w5500_port_spi_tx(const uint8_t *buf, size_t len)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, (uint16_t)len, W5500_SPI_TIMEOUT);
}

void w5500_port_spi_rx(uint8_t *buf, size_t len)
{
    HAL_SPI_Receive(&hspi1, buf, (uint16_t)len, W5500_SPI_TIMEOUT);
}

/* ───── CS ───── */
void w5500_port_cs_select(void)  { HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET); }
void w5500_port_cs_release(void) { HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);   }

/* ───── RST ───── */
void w5500_port_rst_low(void)    { HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET); }
void w5500_port_rst_high(void)   { HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);   }

/* ───── time ───── */
uint32_t w5500_port_millis(void)      { return HAL_GetTick(); }
void     w5500_port_delay_ms(uint32_t ms) { HAL_Delay(ms); }

/* ───── critical section (polling mode) ───── */
void w5500_port_enter_critical(void) { __disable_irq(); }
void w5500_port_exit_critical(void)  { __enable_irq();  }
