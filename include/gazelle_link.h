/*
 * Gazelle Protocol Communication Header
 * 
 * This header defines the interface for Nordic Gazelle protocol
 * communication between nRF52840 boards.
 */

#ifndef GAZELLE_LINK_H
#define GAZELLE_LINK_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* Gazelle configuration */
#define GAZELLE_CHANNEL         2       /* RF channel (0-100) */
#define GAZELLE_PIPE_COUNT      8       /* Number of data pipes */
#define GAZELLE_MAX_PAYLOAD     32      /* Maximum payload per packet */
#define GAZELLE_TIMESLOT_PERIOD 600     /* Timeslot period in microseconds */

/* Device mode is set by build flag GAZELLE_DEVICE_MODE (0=Host, 1=Device) */
#ifndef GAZELLE_DEVICE_MODE
#define GAZELLE_DEVICE_MODE     0       /* Default to Host mode */
#endif

/* Gazelle packet structure */
typedef struct {
    uint8_t pipe_id;
    uint8_t length;
    uint8_t data[GAZELLE_MAX_PAYLOAD];
    uint8_t rssi;
    uint32_t timestamp;
} gazelle_packet_t;

/* Gazelle callbacks */
typedef void (*gazelle_rx_callback_t)(gazelle_packet_t *packet);
typedef void (*gazelle_tx_complete_callback_t)(uint8_t pipe_id, bool success);
typedef void (*gazelle_error_callback_t)(uint8_t error_code);

/* Gazelle API functions */
int gazelle_init(void);
int gazelle_enable(void);
int gazelle_disable(void);
int gazelle_set_callbacks(gazelle_rx_callback_t rx_cb, 
                         gazelle_tx_complete_callback_t tx_cb,
                         gazelle_error_callback_t err_cb);
int gazelle_send_packet(uint8_t pipe_id, uint8_t *data, uint8_t length);
int gazelle_set_channel(uint8_t channel);
int gazelle_set_power(int8_t power_dbm);
bool gazelle_is_enabled(void);
uint32_t gazelle_get_packets_sent(void);
uint32_t gazelle_get_packets_received(void);
uint32_t gazelle_get_packets_lost(void);

#endif /* GAZELLE_LINK_H */