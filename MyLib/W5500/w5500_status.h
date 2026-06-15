/**
 * @file    w5500_status.h
 * @brief   Common return codes for w5500_eth_lib.
 */
#ifndef W5500_STATUS_H_
#define W5500_STATUS_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    W5500_OK              = 0,
    W5500_ERR_INVALID_ARG = -1,
    W5500_ERR_NO_HARDWARE = -2,   /* VERSIONR != 0x04 */
    W5500_ERR_NO_LINK     = -3,   /* PHY link DOWN */
    W5500_ERR_NO_SOCKET   = -4,   /* No free socket */
    W5500_ERR_NOT_OPEN    = -5,
    W5500_ERR_NOT_CONN    = -6,
    W5500_ERR_TIMEOUT     = -7,
    W5500_ERR_TX_FAIL     = -8,
    W5500_ERR_RX_FAIL     = -9,
    W5500_ERR_INTERNAL    = -99,
} w5500_err_t;

#ifdef __cplusplus
}
#endif
#endif /* W5500_STATUS_H_ */
