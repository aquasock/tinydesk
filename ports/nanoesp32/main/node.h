/*
 * node.h - the Reticulum node, running alongside the desktop.
 *
 * This is the TinyNode firmware's job moved inside TinyDesk: WiFi, a
 * Reticulum instance with a persisted identity, and a UDP interface, all on
 * the same board that draws the desktop. Reticulum runs in its own FreeRTOS
 * task so a slow announce or a big resource transfer never stalls the UI.
 *
 * Configuration comes from /fs/etc/wifi.conf, two "key = value" lines:
 *
 *     ssid = mynetwork
 *     password = mypassword
 *
 * That file can be edited in the desktop's Editor or written from the
 * Terminal's shell, which is why there is no setup wizard here.
 */
#ifndef TD_NODE_H
#define TD_NODE_H

#include <stdbool.h>
#include <stdint.h>

/* node.cpp is C++ and link_lcd.c is C, so the interface needs C linkage. */
#ifdef __cplusplus
extern "C" {
#endif

/* Start WiFi and Reticulum in the background. Returns immediately; poll
 * node_state() for progress. Safe to call once, after the filesystem is up. */
void node_start(void);

typedef enum
{
    NODE_OFF = 0,      /* not started                                          */
    NODE_WIFI,         /* bringing up the station                              */
    NODE_NO_CONFIG,    /* /fs/etc/wifi.conf missing or unreadable              */
    NODE_WIFI_FAILED,  /* credentials present, association failed              */
    NODE_RETICULUM,    /* WiFi up, Reticulum starting                          */
    NODE_READY,        /* interface up and announcing                          */
    NODE_FAILED,       /* Reticulum reported an error                          */
} node_state_t;

node_state_t node_state(void);

/* Human-readable one-line state, for the desktop's status line. */
const char *node_state_text(void);

/* The node's LXMF/Reticulum identity hash, or "" before Reticulum starts. */
const char *node_identity_hex(void);

/* Local IPv4 address, or "" while not associated. */
const char *node_ip(void);

/* Counters, for a status window. */
struct node_stats
{
    uint32_t announces_sent;
    uint32_t announces_heard;
    uint32_t packets_in;
    uint32_t packets_out;
    int32_t rssi_dbm;
};
void node_get_stats(struct node_stats *out);

/* Ask Reticulum to flush its path table to flash. Called periodically by the
 * desktop when storage is writable; cheap enough to call from the UI task. */
void node_persist(void);

/* Announce now instead of waiting for the scheduled interval. */
void node_announce_now(void);

#ifdef __cplusplus
}
#endif

#endif
