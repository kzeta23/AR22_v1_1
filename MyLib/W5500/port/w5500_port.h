/**
 * @file    w5500_port.h
 * @brief   Porting interface that user/HW layer must implement.
 *
 * 각 보드/MCU 별로 이 인터페이스를 1번 구현하면 라이브러리가 동작한다.
 * 예: STM32F4 LL 기반 구현 → Src/port/w5500_port_stm32f4_ll.c
 */
#ifndef W5500_PORT_H_
#define W5500_PORT_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ───── SPI 송수신 (1바이트 단위) ───── */
uint8_t w5500_port_spi_xfer(uint8_t tx);
void    w5500_port_spi_tx(const uint8_t *buf, size_t len);
void    w5500_port_spi_rx(uint8_t *buf, size_t len);

/* ───── CS 제어 ───── */
void w5500_port_cs_select(void);    /* CS LOW  */
void w5500_port_cs_release(void);   /* CS HIGH */

/* ───── RST 제어 ───── */
void w5500_port_rst_low(void);
void w5500_port_rst_high(void);

/* ───── 시간 ───── */
uint32_t w5500_port_millis(void);
void     w5500_port_delay_ms(uint32_t ms);

/* ───── (선택) 크리티컬 섹션 ─────
 * 인터럽트 모드에서 SPI 트랜잭션 보호용.
 * 폴링 모드만 쓰면 빈 함수로 둬도 됨.
 */
void w5500_port_enter_critical(void);
void w5500_port_exit_critical(void);

#ifdef __cplusplus
}
#endif
#endif /* W5500_PORT_H_ */
