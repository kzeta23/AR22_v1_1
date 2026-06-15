/**
 * @file    net_app.h
 * @brief   AR22 network application: W5500 TCP server that streams the
 *          current dose-rate as JSON to a connected client every 2 s.
 *
 *          Client: `telnet 192.168.1.22 15022`
 *          Line:   {"val":1.23,"unit":"uSv/h"}\r\n   (unit auto: uSv/h|mSv/h)
 */
#ifndef NET_APP_H
#define NET_APP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up W5500 (reset, VERSIONR check, static network config).
 * Returns true if the chip responded (VERSIONR == 0x04). */
bool net_app_init(void);

/* Run the TCP-server socket state machine. Call frequently (e.g. every 0.2s). */
void net_app_poll(void);

/* If a client is connected, send the dose as JSON. dose_uSv is the current
 * range's dose in uSv/h; unit auto-switches to mSv/h at >= 100 uSv/h to
 * match the OLED display. Call every 2 s. */
void net_app_send_dose(float dose_uSv);

#ifdef __cplusplus
}
#endif

#endif /* NET_APP_H */
