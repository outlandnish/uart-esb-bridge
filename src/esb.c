/*
 * Nordic Enhanced ShockBurst (ESB) Protocol Implementation
 * 
 * This module implements the Nordic Enhanced ShockBurst protocol for
 * wireless communication between nRF52840 boards.
 */

#include "esb_link.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <esb.h>

LOG_MODULE_REGISTER(esb, LOG_LEVEL_DBG);

/* Static variables */
static bool esb_initialized = false;
static bool esb_enabled = false;
static esb_rx_callback_t rx_callback = NULL;
static esb_tx_complete_callback_t tx_callback = NULL;
static esb_error_callback_t error_callback = NULL;

/* Statistics */
static uint32_t packets_sent = 0;
static uint32_t packets_received = 0;
static uint32_t packets_lost = 0;

/* Configuration */
static uint8_t current_channel = ESB_CHANNEL;
static int8_t current_power = 0;

/* ESB payload buffer */
static struct esb_payload tx_payload;
static struct esb_payload rx_payload;

/* Forward declarations */
static void esb_event_handler(struct esb_evt const *p_event);

/* ESB event handler */
static void esb_event_handler(struct esb_evt const *p_event)
{
    esb_packet_t packet;
    
    switch (p_event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            packets_sent++;
            LOG_DBG("ESB TX success: pipe=%d", p_event->tx_success.pipe);
            
            if (tx_callback != NULL) {
                tx_callback(p_event->tx_success.pipe, true);
            }
            break;
            
        case ESB_EVENT_TX_FAILED:
            packets_lost++;
            LOG_WRN("ESB TX failed: pipe=%d", p_event->tx_failed.pipe);
            
            if (tx_callback != NULL) {
                tx_callback(p_event->tx_failed.pipe, false);
            }
            
            if (error_callback != NULL) {
                error_callback(0x01); /* TX failed error code */
            }
            break;
            
        case ESB_EVENT_RX_RECEIVED:
            while (esb_read_rx_payload(&rx_payload) == 0) {
                packets_received++;
                
                /* Fill packet structure */
                packet.pipe_id = rx_payload.pipe;
                packet.length = rx_payload.length;
                packet.rssi = rx_payload.rssi;
                packet.timestamp = k_cycle_get_32();
                packet.no_ack = rx_payload.noack;
                
                /* Copy payload data */
                memcpy(packet.data, rx_payload.data, 
                       rx_payload.length > ESB_MAX_PAYLOAD ? 
                       ESB_MAX_PAYLOAD : rx_payload.length);
                
                LOG_DBG("ESB RX: pipe=%d, len=%d, rssi=%d", 
                        packet.pipe_id, packet.length, packet.rssi);
                
                /* Call application callback */
                if (rx_callback != NULL) {
                    rx_callback(&packet);
                }
            }
            break;
            
        default:
            LOG_WRN("Unknown ESB event: %d", p_event->evt_id);
            if (error_callback != NULL) {
                error_callback(0x02); /* Unknown event error code */
            }
            break;
    }
}

/* Initialize ESB protocol */
int esb_init(void)
{
    int err;
    struct esb_config config = ESB_DEFAULT_CONFIG;
    
    if (esb_initialized) {
        return 0;
    }
    
    /* Configure ESB */
    config.protocol = ESB_PROTOCOL_ESB_DPL;  /* Dynamic Payload Length */
    config.mode = ESB_MODE_PTX;              /* Primary TX mode by default */
    config.event_handler = esb_event_handler;
    config.bitrate = ESB_BITRATE_1MBPS;      /* 1 Mbps to match UART speed */
    config.crc = ESB_CRC_16BIT;              /* 16-bit CRC for reliability */
    config.tx_output_power = ESB_TX_POWER_0DBM;
    config.retransmit_delay = 250;           /* 250µs delay */
    config.retransmit_count = 3;             /* 3 retransmissions */
    config.tx_mode = ESB_TXMODE_AUTO;        /* Auto mode */
    config.payload_length = ESB_DEFAULT_PAYLOAD;
    config.selective_auto_ack = false;
    
    /* Set RF channel */
    config.rf_channel = current_channel;
    
    /* Set mode based on compile-time flag */
    #if ESB_DEVICE_MODE == 0
    config.mode = ESB_MODE_PTX;  /* Primary TX */
    LOG_INF("Initializing ESB in Primary TX mode");
    #else
    config.mode = ESB_MODE_PRX;  /* Primary RX */
    LOG_INF("Initializing ESB in Primary RX mode");
    #endif
    
    /* Set base addresses */
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
    
    memcpy(config.base_addr_0, base_addr_0, 4);
    memcpy(config.base_addr_1, base_addr_1, 4);
    memcpy(config.pipe_prefixes, addr_prefix, 8);
    
    /* Initialize ESB */
    err = esb_init(&config);
    if (err) {
        LOG_ERR("Failed to initialize ESB: %d", err);
        return -1;
    }
    
    esb_initialized = true;
    LOG_INF("ESB initialized successfully");
    
    return 0;
}

/* Enable ESB protocol */
int esb_enable(void)
{
    int err;
    
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -1;
    }
    
    if (esb_enabled) {
        return 0;
    }
    
    err = esb_enable();
    if (err) {
        LOG_ERR("Failed to enable ESB: %d", err);
        return -2;
    }
    
    esb_enabled = true;
    
    #if ESB_DEVICE_MODE == 0
    LOG_INF("ESB Primary TX enabled on channel %d", current_channel);
    #else
    LOG_INF("ESB Primary RX enabled on channel %d", current_channel);
    #endif
    
    return 0;
}

/* Disable ESB protocol */
int esb_disable(void)
{
    if (!esb_enabled) {
        return 0;
    }
    
    esb_disable();
    esb_enabled = false;
    LOG_INF("ESB disabled");
    
    return 0;
}

/* Set callbacks */
int esb_set_callbacks(esb_rx_callback_t rx_cb, 
                     esb_tx_complete_callback_t tx_cb,
                     esb_error_callback_t err_cb)
{
    rx_callback = rx_cb;
    tx_callback = tx_cb;
    error_callback = err_cb;
    
    LOG_DBG("ESB callbacks set");
    
    return 0;
}

/* Send packet */
int esb_send_packet(uint8_t pipe_id, uint8_t *data, uint8_t length, bool no_ack)
{
    int err;
    
    if (!esb_enabled) {
        LOG_ERR("ESB not enabled");
        return -1;
    }
    
    if (pipe_id >= ESB_PIPE_COUNT) {
        LOG_ERR("Invalid pipe ID: %d", pipe_id);
        return -2;
    }
    
    if (length > ESB_MAX_PAYLOAD) {
        LOG_ERR("Payload too large: %d", length);
        return -3;
    }
    
    if (data == NULL) {
        LOG_ERR("Invalid data pointer");
        return -4;
    }
    
    /* Prepare payload */
    tx_payload.pipe = pipe_id;
    tx_payload.length = length;
    tx_payload.noack = no_ack;
    memcpy(tx_payload.data, data, length);
    
    err = esb_write_payload(&tx_payload);
    if (err) {
        LOG_WRN("Failed to write ESB payload: %d", err);
        return -5;
    }
    
    LOG_DBG("ESB TX queued: pipe=%d, len=%d, no_ack=%d", pipe_id, length, no_ack);
    
    return 0;
}

/* Set channel */
int esb_set_channel(uint8_t channel)
{
    int err;
    
    if (channel > 100) {
        LOG_ERR("Invalid channel: %d", channel);
        return -1;
    }
    
    err = esb_set_rf_channel(channel);
    if (err) {
        LOG_ERR("Failed to set ESB channel: %d", err);
        return -2;
    }
    
    current_channel = channel;
    LOG_INF("ESB channel set to %d", channel);
    
    return 0;
}

/* Set power */
int esb_set_power(int8_t power_dbm)
{
    enum esb_tx_power power_level;
    int err;
    
    if (power_dbm >= 4) {
        power_level = ESB_TX_POWER_4DBM;
    } else if (power_dbm >= 0) {
        power_level = ESB_TX_POWER_0DBM;
    } else if (power_dbm >= -4) {
        power_level = ESB_TX_POWER_NEG4DBM;
    } else if (power_dbm >= -8) {
        power_level = ESB_TX_POWER_NEG8DBM;
    } else if (power_dbm >= -12) {
        power_level = ESB_TX_POWER_NEG12DBM;
    } else if (power_dbm >= -16) {
        power_level = ESB_TX_POWER_NEG16DBM;
    } else if (power_dbm >= -20) {
        power_level = ESB_TX_POWER_NEG20DBM;
    } else {
        power_level = ESB_TX_POWER_NEG30DBM;
    }
    
    err = esb_set_tx_power(power_level);
    if (err) {
        LOG_ERR("Failed to set ESB TX power: %d", err);
        return -1;
    }
    
    current_power = power_dbm;
    LOG_INF("ESB TX power set to %d dBm", power_dbm);
    
    return 0;
}

/* Set bitrate */
int esb_set_bitrate(uint32_t bitrate)
{
    enum esb_bitrate esb_bitrate;
    int err;
    
    switch (bitrate) {
        case 250000:
            esb_bitrate = ESB_BITRATE_250KBPS;
            break;
        case 1000000:
            esb_bitrate = ESB_BITRATE_1MBPS;
            break;
        case 2000000:
            esb_bitrate = ESB_BITRATE_2MBPS;
            break;
        default:
            LOG_ERR("Invalid bitrate: %d", bitrate);
            return -1;
    }
    
    err = esb_set_bitrate(esb_bitrate);
    if (err) {
        LOG_ERR("Failed to set ESB bitrate: %d", err);
        return -2;
    }
    
    LOG_INF("ESB bitrate set to %d bps", bitrate);
    
    return 0;
}

/* Set CRC */
int esb_set_crc(uint8_t crc_length)
{
    enum esb_crc crc_mode;
    int err;
    
    switch (crc_length) {
        case 0:
            crc_mode = ESB_CRC_OFF;
            break;
        case 1:
            crc_mode = ESB_CRC_8BIT;
            break;
        case 2:
            crc_mode = ESB_CRC_16BIT;
            break;
        default:
            LOG_ERR("Invalid CRC length: %d", crc_length);
            return -1;
    }
    
    err = esb_set_crc(crc_mode);
    if (err) {
        LOG_ERR("Failed to set ESB CRC: %d", err);
        return -2;
    }
    
    LOG_INF("ESB CRC set to %d bit", crc_length == 0 ? 0 : (crc_length == 1 ? 8 : 16));
    
    return 0;
}

/* Check if enabled */
bool esb_is_enabled(void)
{
    return esb_enabled;
}

/* Get statistics */
uint32_t esb_get_packets_sent(void)
{
    return packets_sent;
}

uint32_t esb_get_packets_received(void)
{
    return packets_received;
}

uint32_t esb_get_packets_lost(void)
{
    return packets_lost;
}