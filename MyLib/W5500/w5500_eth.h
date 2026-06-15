/**
 * @file    w5500_eth.h
 * @brief   Public API - W5500 이더넷 코어 (초기화, 네트워크 설정, 폴링).
 *
 * 사용 예:
 *   w5500_net_t net = { .mac = {...}, .ip = {192,168,1,100}, ... };
 *   w5500_init(&net);
 *   while (1) { w5500_poll(); ... }
 */
#ifndef W5500_ETH_H_
#define W5500_ETH_H_

#include <stdint.h>
#include <stdbool.h>
#include "w5500_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t subnet[4];
    uint8_t gateway[4];
    uint8_t dns[4];        /* 사용 안 하면 {0,0,0,0} */
} w5500_net_t;

/**
 * 라이브러리 초기화:
 *   - W5500 하드웨어 리셋 (RST 토글)
 *   - VERSIONR 검증
 *   - MAC/SIPR/SUBR/GAR 레지스터 기록
 *   - 링크 대기 (최대 5초)
 *
 * @note SPI/GPIO 초기화는 사용자 측에서 미리 끝나있어야 함.
 *       port_*() 인터페이스가 동작 가능한 상태여야 한다.
 */
w5500_err_t w5500_init(const w5500_net_t *net);

/**
 * 메인 루프에서 주기적으로 호출.
 *   - 소켓 상태 머신 진행
 *   - 콜백 디스패치
 *   - 링크 상태 디바운싱
 *
 * 호출 주기는 빠를수록 좋음 (10ms 이내 권장).
 */
void w5500_poll(void);

/* ───── 상태 조회 ───── */
bool         w5500_is_link_up(void);
uint8_t      w5500_get_phy_speed_mbps(void);   /* 10 or 100 */
bool         w5500_is_full_duplex(void);
void         w5500_get_mac(uint8_t out[6]);
void         w5500_get_ip(uint8_t out[4]);

/* ───── 런타임 IP 변경 (선택) ───── */
w5500_err_t  w5500_set_ip(const uint8_t ip[4],
                          const uint8_t subnet[4],
                          const uint8_t gateway[4]);

#ifdef __cplusplus
}
#endif
#endif /* W5500_ETH_H_ */
