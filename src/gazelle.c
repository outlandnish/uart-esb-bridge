/*
 * Nordic Gazelle Protocol Implementation
 * 
 * This module implements the Nordic Gazelle protocol for
 * wireless communication between nRF52840 boards.
 */

#include "gazelle_link.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <nrf_gzll.h>
#include <nrf_gzp.h>

LOG_MODULE_REGISTER(gazelle, LOG_LEVEL_DBG);

/* Static variables */
static bool gazelle_initialized = false;
static bool gazelle_enabled = false;
static gazelle_rx_callback_t rx_callback = NULL;
static gazelle_tx_complete_callback_t tx_callback = NULL;
static gazelle_error_callback_t error_callback = NULL;

/* Statistics */
static uint32_t packets_sent = 0;
static uint32_t packets_received = 0;
static uint32_t packets_lost = 0;

/* Internal buffers */
static uint8_t rx_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
static uint32_t rx_payload_length;

/* Forward declarations */
static void gazelle_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info);
static void gazelle_host_tx_success(uint32_t pipe, nrf_gzll_host_tx_info_t tx_info);
static void gazelle_host_tx_failed(uint32_t pipe, nrf_gzll_host_tx_info_t tx_info);
static void gazelle_device_rx_data_ready(uint32_t pipe, nrf_gzll_device_rx_info_t rx_info);
static void gazelle_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info);
static void gazelle_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info);
static void gazelle_disabled(void);

/* Gazelle Host callback structure */
static const nrf_gzll_host_callbacks_t gazelle_host_callbacks = {
    .gzll_host_rx_data_ready = gazelle_host_rx_data_ready,
    .gzll_host_tx_success = gazelle_host_tx_success,
    .gzll_host_tx_failed = gazelle_host_tx_failed,
    .gzll_disabled = gazelle_disabled
};

/* Gazelle Device callback structure */
static const nrf_gzll_device_callbacks_t gazelle_device_callbacks = {
    .gzll_device_rx_data_ready = gazelle_device_rx_data_ready,
    .gzll_device_tx_success = gazelle_device_tx_success,
    .gzll_device_tx_failed = gazelle_device_tx_failed,
    .gzll_disabled = gazelle_disabled
};

/* RX data ready callback */
static void gazelle_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    gazelle_packet_t packet;
    bool result_value;
    
    /* Read the received payload */
    result_value = nrf_gzll_fetch_packet_host_rx_fifo(pipe, rx_payload, 
                                                     &rx_payload_length);
    
    if (result_value && rx_payload_length > 0) {
        /* Fill packet structure */
        packet.pipe_id = (uint8_t)pipe;
        packet.length = (uint8_t)rx_payload_length;
        packet.rssi = rx_info.rssi;
        packet.timestamp = k_cycle_get_32();
        
        /* Copy payload data */
        memcpy(packet.data, rx_payload, 
               rx_payload_length > GAZELLE_MAX_PAYLOAD ? 
               GAZELLE_MAX_PAYLOAD : rx_payload_length);
        
        packets_received++;
        
        LOG_DBG("Gazelle RX: pipe=%d, len=%d, rssi=%d", 
                packet.pipe_id, packet.length, packet.rssi);
        
        /* Call application callback */
        if (rx_callback != NULL) {
            rx_callback(&packet);
        }
    }
}

/* TX success callback */
static void gazelle_host_tx_success(uint32_t pipe, nrf_gzll_host_tx_info_t tx_info)
{
    packets_sent++;
    
    LOG_DBG("Gazelle TX success: pipe=%d", pipe);
    
    if (tx_callback != NULL) {
        tx_callback((uint8_t)pipe, true);
    }
}

/* TX failed callback */
static void gazelle_host_tx_failed(uint32_t pipe, nrf_gzll_host_tx_info_t tx_info)
{
    packets_lost++;
    
    LOG_WRN("Gazelle TX failed: pipe=%d", pipe);
    
    if (tx_callback != NULL) {
        tx_callback((uint8_t)pipe, false);
    }
    
    if (error_callback != NULL) {
        error_callback(0x01); /* TX failed error code */
    }
}

/* Device RX data ready callback */
static void gazelle_device_rx_data_ready(uint32_t pipe, nrf_gzll_device_rx_info_t rx_info)
{
    gazelle_packet_t packet;
    bool result_value;
    
    /* Read the received payload */
    result_value = nrf_gzll_fetch_packet_device_rx_fifo(pipe, rx_payload, 
                                                       &rx_payload_length);
    
    if (result_value && rx_payload_length > 0) {
        /* Fill packet structure */
        packet.pipe_id = (uint8_t)pipe;
        packet.length = (uint8_t)rx_payload_length;
        packet.rssi = rx_info.rssi;
        packet.timestamp = k_cycle_get_32();
        
        /* Copy payload data */
        memcpy(packet.data, rx_payload, 
               rx_payload_length > GAZELLE_MAX_PAYLOAD ? 
               GAZELLE_MAX_PAYLOAD : rx_payload_length);
        
        packets_received++;
        
        LOG_DBG("Gazelle Device RX: pipe=%d, len=%d, rssi=%d", 
                packet.pipe_id, packet.length, packet.rssi);
        
        /* Call application callback */
        if (rx_callback != NULL) {
            rx_callback(&packet);
        }
    }
}

/* Device TX success callback */
static void gazelle_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    packets_sent++;
    
    LOG_DBG("Gazelle Device TX success: pipe=%d", pipe);
    
    if (tx_callback != NULL) {
        tx_callback((uint8_t)pipe, true);
    }
}

/* Device TX failed callback */
static void gazelle_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    packets_lost++;
    
    LOG_WRN("Gazelle Device TX failed: pipe=%d", pipe);
    
    if (tx_callback != NULL) {
        tx_callback((uint8_t)pipe, false);
    }
    
    if (error_callback != NULL) {
        error_callback(0x01); /* TX failed error code */
    }
}

/* Gazelle disabled callback */
static void gazelle_disabled(void)
{
    gazelle_enabled = false;
    LOG_INF("Gazelle disabled");
}

/* Initialize Gazelle protocol */
int gazelle_init(void)
{
    bool result_value;
    
    if (gazelle_initialized) {
        return 0;
    }
    
    /* Initialize Gazelle in appropriate mode */
    #if GAZELLE_DEVICE_MODE == 0
    result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    LOG_INF("Initializing Gazelle in HOST mode");
    #else
    result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    LOG_INF("Initializing Gazelle in DEVICE mode");
    #endif
    
    if (!result_value) {
        LOG_ERR("Failed to initialize Gazelle");
        return -1;
    }
    
    /* Set base address */
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    
    result_value = nrf_gzll_set_base_address_0(base_addr_0);
    if (!result_value) {
        LOG_ERR("Failed to set base address 0");
        return -2;
    }
    
    result_value = nrf_gzll_set_base_address_1(base_addr_1);
    if (!result_value) {
        LOG_ERR("Failed to set base address 1");
        return -3;
    }
    
    /* Set channel table */
    uint32_t channel_table[] = {GAZELLE_CHANNEL, GAZELLE_CHANNEL + 1, GAZELLE_CHANNEL + 2};
    result_value = nrf_gzll_set_channel_table(channel_table, 3);
    if (!result_value) {
        LOG_ERR("Failed to set channel table");
        return -4;
    }
    
    /* Set timeslot period */
    result_value = nrf_gzll_set_timeslot_period(GAZELLE_TIMESLOT_PERIOD);
    if (!result_value) {
        LOG_ERR("Failed to set timeslot period");
        return -5;
    }
    
    /* Set power level (0 dBm) */
    result_value = nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM);
    if (!result_value) {
        LOG_ERR("Failed to set TX power");
        return -6;
    }
    
    /* Set callbacks based on mode */
    #if GAZELLE_DEVICE_MODE == 0
    nrf_gzll_set_host_callbacks(&gazelle_host_callbacks);
    #else
    nrf_gzll_set_device_callbacks(&gazelle_device_callbacks);
    #endif
    
    gazelle_initialized = true;
    LOG_INF("Gazelle initialized successfully");
    
    return 0;
}

/* Enable Gazelle protocol */
int gazelle_enable(void)
{
    bool result_value;
    
    if (!gazelle_initialized) {
        LOG_ERR("Gazelle not initialized");
        return -1;
    }
    
    if (gazelle_enabled) {
        return 0;
    }
    
    result_value = nrf_gzll_enable();
    if (!result_value) {
        LOG_ERR("Failed to enable Gazelle");
        return -2;
    }
    
    gazelle_enabled = true;
    LOG_INF("Gazelle enabled");
    
    return 0;
}

/* Disable Gazelle protocol */
int gazelle_disable(void)
{
    if (!gazelle_enabled) {
        return 0;
    }
    
    nrf_gzll_disable();
    
    /* Wait for disable to complete */
    while (nrf_gzll_is_enabled()) {
        k_msleep(1);
    }
    
    LOG_INF("Gazelle disabled");
    
    return 0;
}

/* Set callbacks */
int gazelle_set_callbacks(gazelle_rx_callback_t rx_cb, 
                         gazelle_tx_complete_callback_t tx_cb,
                         gazelle_error_callback_t err_cb)
{
    rx_callback = rx_cb;
    tx_callback = tx_cb;
    error_callback = err_cb;
    
    LOG_DBG("Gazelle callbacks set");
    
    return 0;
}

/* Send packet */
int gazelle_send_packet(uint8_t pipe_id, uint8_t *data, uint8_t length)
{
    bool result_value;
    
    if (!gazelle_enabled) {
        LOG_ERR("Gazelle not enabled");
        return -1;
    }
    
    if (pipe_id >= GAZELLE_PIPE_COUNT) {
        LOG_ERR("Invalid pipe ID: %d", pipe_id);
        return -2;
    }
    
    if (length > GAZELLE_MAX_PAYLOAD) {
        LOG_ERR("Payload too large: %d", length);
        return -3;
    }
    
    if (data == NULL) {
        LOG_ERR("Invalid data pointer");
        return -4;
    }
    
    #if GAZELLE_DEVICE_MODE == 0
    /* Host mode: add to host TX FIFO */
    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe_id, data, length);
    #else
    /* Device mode: add to device TX FIFO */
    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe_id, data, length);
    #endif
    
    if (!result_value) {
        LOG_WRN("Failed to add packet to TX FIFO");
        return -5;
    }
    
    LOG_DBG("Gazelle TX queued: pipe=%d, len=%d", pipe_id, length);
    
    return 0;
}

/* Set channel */
int gazelle_set_channel(uint8_t channel)
{
    if (channel > 100) {
        LOG_ERR("Invalid channel: %d", channel);
        return -1;
    }
    
    uint32_t channel_table[] = {channel, channel + 1, channel + 2};
    bool result_value = nrf_gzll_set_channel_table(channel_table, 3);
    
    if (!result_value) {
        LOG_ERR("Failed to set channel");
        return -2;
    }
    
    LOG_INF("Gazelle channel set to %d", channel);
    
    return 0;
}

/* Set power */
int gazelle_set_power(int8_t power_dbm)
{
    nrf_gzll_tx_power_t power_level;
    
    if (power_dbm >= 4) {
        power_level = NRF_GZLL_TX_POWER_4_DBM;
    } else if (power_dbm >= 0) {
        power_level = NRF_GZLL_TX_POWER_0_DBM;
    } else if (power_dbm >= -4) {
        power_level = NRF_GZLL_TX_POWER_NEG4_DBM;
    } else if (power_dbm >= -8) {
        power_level = NRF_GZLL_TX_POWER_NEG8_DBM;
    } else if (power_dbm >= -12) {
        power_level = NRF_GZLL_TX_POWER_NEG12_DBM;
    } else if (power_dbm >= -16) {
        power_level = NRF_GZLL_TX_POWER_NEG16_DBM;
    } else if (power_dbm >= -20) {
        power_level = NRF_GZLL_TX_POWER_NEG20_DBM;
    } else {
        power_level = NRF_GZLL_TX_POWER_NEG30_DBM;
    }
    
    bool result_value = nrf_gzll_set_tx_power(power_level);
    if (!result_value) {
        LOG_ERR("Failed to set TX power");
        return -1;
    }
    
    LOG_INF("Gazelle TX power set to %d dBm", power_dbm);
    
    return 0;
}

/* Check if enabled */
bool gazelle_is_enabled(void)
{
    return gazelle_enabled;
}

/* Get statistics */
uint32_t gazelle_get_packets_sent(void)
{
    return packets_sent;
}

uint32_t gazelle_get_packets_received(void)
{
    return packets_received;
}

uint32_t gazelle_get_packets_lost(void)
{
    return packets_lost;
}