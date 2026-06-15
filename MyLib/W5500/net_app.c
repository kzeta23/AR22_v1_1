/**
 * @file    net_app.c
 * @brief   W5500 TCP server (port 15022) streaming dose-rate JSON.
 *          Uses the WIZnet ioLibrary socket API directly (the bundled
 *          w5500_tcp_server wrapper is an unimplemented stub).
 */
#include "net_app.h"
#include "w5500_eth.h"
#include "port/w5500_port_ar22.h"

#include "socket.h"
#include "wizchip_conf.h"
#include "W5500/w5500.h"

#include <stdio.h>

#define NET_SOCK   0          /* W5500 hardware socket index */
#define NET_PORT   15022      /* TCP listen port */

static bool s_ready = false;

bool net_app_init(void)
{
    static const w5500_net_t net = {
        .mac     = {0x00, 0x08, 0xDC, 0x22, 0x00, 0x01},
        .ip      = {192, 168, 1, 22},
        .subnet  = {255, 255, 255, 0},
        .gateway = {192, 168, 1, 1},
        .dns     = {0, 0, 0, 0},
    };

    w5500_port_ar22_init();                  /* CS=PA4, RST=PC4 GPIO */
    s_ready = (w5500_init(&net) == W5500_OK);
    return s_ready;
}

void net_app_poll(void)
{
    if (!s_ready) return;

    switch (getSn_SR(NET_SOCK)) {
        case SOCK_CLOSED:
            socket(NET_SOCK, Sn_MR_TCP, NET_PORT, 0);
            break;

        case SOCK_INIT:
            listen(NET_SOCK);
            break;

        case SOCK_ESTABLISHED: {
            /* Discard anything the client sends (e.g. telnet negotiation)
             * so the RX buffer never fills. */
            uint16_t rsr = getSn_RX_RSR(NET_SOCK);
            if (rsr > 0) {
                uint8_t scratch[64];
                uint16_t n = (rsr > sizeof scratch) ? sizeof scratch : rsr;
                recv(NET_SOCK, scratch, n);
            }
            break;
        }

        case SOCK_CLOSE_WAIT:
            disconnect(NET_SOCK);
            break;

        case SOCK_FIN_WAIT:
        case SOCK_CLOSING:
        case SOCK_TIME_WAIT:
        case SOCK_LAST_ACK:
            close(NET_SOCK);
            break;

        default:
            break;
    }
}

void net_app_send_dose(float dose_uSv)
{
    if (!s_ready) return;
    if (getSn_SR(NET_SOCK) != SOCK_ESTABLISHED) return;

    float       val;
    const char *unit;
    if (dose_uSv >= 100.0f) { val = dose_uSv / 1000.0f; unit = "mSv/h"; }
    else                    { val = dose_uSv;            unit = "uSv/h"; }

    char json[64];
    int  n = snprintf(json, sizeof json,
                      "{\"val\":%.2f,\"unit\":\"%s\"}\r\n", val, unit);
    if (n > 0) {
        send(NET_SOCK, (uint8_t *)json, (uint16_t)n);
    }
}
