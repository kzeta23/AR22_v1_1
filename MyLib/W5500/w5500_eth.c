/**
 * @file    w5500_eth.c
 * @brief   W5500 이더넷 코어 구현 (스텁 - P1~P2 단계에서 채움).
 */
#include "w5500_eth.h"
#include "w5500_eth_config.h"
#include "port/w5500_port.h"

/* WIZnet ioLibrary */
#include "wizchip_conf.h"
#include "socket.h"

#include <string.h>

/* ───── 내부 상태 ───── */
static struct {
    bool     initialized;
    bool     link_up;
    uint32_t last_link_check_ms;
    uint8_t  link_diff_count;
    uint8_t  reported_link;
    w5500_net_t cached_net;
} s_eth;

/* ───── ioLibrary용 콜백 (port → ioLibrary 연결) ───── */
static void cb_cs_sel(void)         { w5500_port_cs_select();  }
static void cb_cs_desel(void)       { w5500_port_cs_release(); }
static uint8_t cb_spi_rb(void)      { return w5500_port_spi_xfer(0xFF); }
static void cb_spi_wb(uint8_t b)    { (void)w5500_port_spi_xfer(b); }
static void cb_spi_rb_burst(uint8_t *buf, uint16_t len) { w5500_port_spi_rx(buf, len); }
static void cb_spi_wb_burst(uint8_t *buf, uint16_t len) { w5500_port_spi_tx(buf, len); }
static void cb_crit_enter(void)     { w5500_port_enter_critical(); }
static void cb_crit_exit(void)      { w5500_port_exit_critical();  }

/* ───── 내부 함수 ───── */
static void hw_reset(void)
{
    w5500_port_rst_low();
    w5500_port_delay_ms(20);
    w5500_port_rst_high();
    w5500_port_delay_ms(200);
}

static void register_callbacks(void)
{
    reg_wizchip_cs_cbfunc(cb_cs_sel, cb_cs_desel);
    reg_wizchip_spi_cbfunc(cb_spi_rb, cb_spi_wb);
    reg_wizchip_spiburst_cbfunc(cb_spi_rb_burst, cb_spi_wb_burst);
    reg_wizchip_cris_cbfunc(cb_crit_enter, cb_crit_exit);
}

static bool wait_link_up(uint32_t timeout_ms)
{
    uint32_t t0 = w5500_port_millis();
    while (w5500_port_millis() - t0 < timeout_ms) {
        uint8_t phy = 0;
        ctlwizchip(CW_GET_PHYLINK, &phy);
        if (phy == PHY_LINK_ON) return true;
        w5500_port_delay_ms(50);
    }
    return false;
}

/* ───── Public API ───── */
w5500_err_t w5500_init(const w5500_net_t *net)
{
    if (!net) return W5500_ERR_INVALID_ARG;

    /* 1) 하드웨어 리셋 */
    hw_reset();

    /* 2) ioLibrary 콜백 등록 (이후 모든 SPI 통신은 이 함수들 경유) */
    register_callbacks();

    /* 3) 칩 살아있는지 검증 (VERSIONR == 0x04) */
    uint8_t ver = getVERSIONR();
    if (ver != 0x04) {
        W5500_LOG("init fail: VERSIONR=0x%02X (expect 0x04)", ver);
        return W5500_ERR_NO_HARDWARE;
    }
    W5500_LOG("VERSIONR=0x04 OK");

    /* 4) 8개 소켓 모두 RX/TX 2KB 할당 */
    uint8_t bufsize[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    wizchip_init(bufsize, bufsize);

    /* 5) 네트워크 설정 기록 */
    wiz_NetInfo info = { 0 };
    memcpy(info.mac, net->mac, 6);
    memcpy(info.ip,  net->ip,  4);
    memcpy(info.sn,  net->subnet, 4);
    memcpy(info.gw,  net->gateway, 4);
    memcpy(info.dns, net->dns, 4);
    info.dhcp = NETINFO_STATIC;
    ctlnetwork(CN_SET_NETINFO, (void *)&info);

    /* 6) Readback 검증 (이전 ESP32 작업에서 SPI 비트 에러 경험 → 확인 필수) */
    wiz_NetInfo rb = { 0 };
    ctlnetwork(CN_GET_NETINFO, (void *)&rb);
    if (memcmp(rb.ip, net->ip, 4) != 0 ||
        memcmp(rb.sn, net->subnet, 4) != 0 ||
        memcmp(rb.mac, net->mac, 6) != 0) {
        W5500_LOG("readback mismatch - SPI integrity issue?");
        return W5500_ERR_INTERNAL;
    }

    /* 7) 링크 대기 */
    if (!wait_link_up(5000)) {
        W5500_LOG("link timeout (5s)");
        /* 링크 없어도 초기화는 성공으로 간주 (나중에 케이블 연결 가능) */
    }

    s_eth.cached_net          = *net;
    s_eth.initialized         = true;
    s_eth.last_link_check_ms  = w5500_port_millis();
    s_eth.link_diff_count     = 0;
    s_eth.reported_link       = (uint8_t)w5500_is_link_up();

    W5500_LOG("init OK: IP=%u.%u.%u.%u",
              net->ip[0], net->ip[1], net->ip[2], net->ip[3]);
    return W5500_OK;
}

void w5500_poll(void)
{
    if (!s_eth.initialized) return;

    /* 링크 디바운싱 */
    uint32_t now = w5500_port_millis();
    if (now - s_eth.last_link_check_ms >= W5500_LINK_POLL_INTERVAL_MS) {
        s_eth.last_link_check_ms = now;
        uint8_t phy = 0;
        ctlwizchip(CW_GET_PHYLINK, &phy);
        uint8_t cur = (phy == PHY_LINK_ON) ? 1 : 0;

        if (cur == s_eth.reported_link) {
            s_eth.link_diff_count = 0;
        } else {
            if (++s_eth.link_diff_count >= W5500_LINK_DEBOUNCE_COUNT) {
                s_eth.reported_link   = cur;
                s_eth.link_diff_count = 0;
                s_eth.link_up         = (cur != 0);
                W5500_LOG("link %s", cur ? "UP" : "DOWN");
            }
        }
    }

    /* TODO: TCP/UDP 소켓 상태 머신 진행 (tcp_server_poll 등 내부 호출) */
}

bool w5500_is_link_up(void)
{
    uint8_t phy = 0;
    ctlwizchip(CW_GET_PHYLINK, &phy);
    return phy == PHY_LINK_ON;
}

uint8_t w5500_get_phy_speed_mbps(void)
{
    /* PHYCFGR bit 1: SPD (1=100M, 0=10M) */
    uint8_t phycfg = getPHYCFGR();
    return (phycfg & 0x02) ? 100 : 10;
}

bool w5500_is_full_duplex(void)
{
    /* PHYCFGR bit 2: DPX (1=Full, 0=Half) */
    return (getPHYCFGR() & 0x04) != 0;
}

void w5500_get_mac(uint8_t out[6])
{
    if (out) getSHAR(out);
}

void w5500_get_ip(uint8_t out[4])
{
    if (out) getSIPR(out);
}

w5500_err_t w5500_set_ip(const uint8_t ip[4],
                         const uint8_t subnet[4],
                         const uint8_t gateway[4])
{
    if (!ip || !subnet || !gateway) return W5500_ERR_INVALID_ARG;
    setSIPR((uint8_t *)ip);
    setSUBR((uint8_t *)subnet);
    setGAR((uint8_t *)gateway);
    memcpy(s_eth.cached_net.ip,      ip,      4);
    memcpy(s_eth.cached_net.subnet,  subnet,  4);
    memcpy(s_eth.cached_net.gateway, gateway, 4);
    return W5500_OK;
}
