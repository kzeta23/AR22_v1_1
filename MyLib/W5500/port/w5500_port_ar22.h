/**
 * @file    w5500_port_ar22.h
 * @brief   AR22 board-specific W5500 port init (HAL SPI1, CS=PA4, RST=PC4).
 */
#ifndef W5500_PORT_AR22_H_
#define W5500_PORT_AR22_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Configure W5500 control GPIO (CS=PA4, RST=PC4). Call once before
 * w5500_init(). SPI1 itself is initialised by MX_SPI1_Init() in main.c. */
void w5500_port_ar22_init(void);

#ifdef __cplusplus
}
#endif
#endif /* W5500_PORT_AR22_H_ */
