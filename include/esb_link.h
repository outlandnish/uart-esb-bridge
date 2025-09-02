/*
 * ESB Protocol Communication Header
 * 
 * This header defines the interface for Nordic Enhanced ShockBurst (ESB) protocol
 * communication between nRF52840 boards.
 */

#ifndef ESB_LINK_H
#define ESB_LINK_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* ESB configuration */
#define ESB_CHANNEL             2       /* RF channel (0-100) */
#define ESB_PIPE_COUNT          8       /* Number of data pipes */
#define ESB_MAX_PAYLOAD         252     /* Maximum payload per packet (ESB supports up to 252) */
#define ESB_DEFAULT_PAYLOAD     32      /* Default payload size for compatibility */

/* Device mode is set by build flag ESB_DEVICE_MODE (0=Primary TX, 1=Primary RX) */
#ifndef ESB_DEVICE_MODE
#define ESB_DEVICE_MODE         0       /* Default to Primary TX mode */
#endif

/* ESB packet structure */
typedef struct {
    uint8_t pipe_id;
    uint8_t length;
    uint8_t data[ESB_MAX_PAYLOAD];
    int8_t rssi;
    uint32_t timestamp;
    bool no_ack;
} esb_packet_t;

/* ESB callbacks */
typedef void (*esb_rx_callback_t)(esb_packet_t *packet);
typedef void (*esb_tx_complete_callback_t)(uint8_t pipe_id, bool success);
typedef void (*esb_error_callback_t)(uint8_t error_code);

/* ESB API functions */
int esb_init(void);
int esb_enable(void);
int esb_disable(void);
int esb_set_callbacks(esb_rx_callback_t rx_cb, 
                     esb_tx_complete_callback_t tx_cb,
                     esb_error_callback_t err_cb);
int esb_send_packet(uint8_t pipe_id, uint8_t *data, uint8_t length, bool no_ack);
int esb_set_channel(uint8_t channel);
int esb_set_power(int8_t power_dbm);
int esb_set_bitrate(uint32_t bitrate); /* 1Mbps, 2Mbps */
int esb_set_crc(uint8_t crc_length);   /* 0=off, 1=8bit, 2=16bit */
bool esb_is_enabled(void);
uint32_t esb_get_packets_sent(void);
uint32_t esb_get_packets_received(void);
uint32_t esb_get_packets_lost(void);

#endif /* ESB_LINK_H */