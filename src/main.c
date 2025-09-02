/*
 * nRF52840 USB-UART-Gazelle Bridge
 * 
 * This firmware creates a USB-UART bridge using the Gazelle protocol
 * to communicate with another identical nRF52840 board.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_cdc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "gazelle_link.h"

LOG_MODULE_REGISTER(gazelle_link, LOG_LEVEL_DBG);

/* LED definitions for status indication */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/* USB CDC ACM device */
static const struct device *const usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

/* UART device for 1Mbps communication */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));

/* Buffer sizes */
#define BUFFER_SIZE 1024
#define RING_BUF_SIZE 4096

/* Data buffers */
static uint8_t usb_rx_buffer[BUFFER_SIZE];
static uint8_t uart_rx_buffer[BUFFER_SIZE];
static uint8_t gazelle_rx_buffer[BUFFER_SIZE];

/* Ring buffers for data flow */
RING_BUF_DECLARE(usb_to_gazelle_rb, RING_BUF_SIZE);
RING_BUF_DECLARE(gazelle_to_usb_rb, RING_BUF_SIZE);

/* Forward declarations */
static void usb_cdc_interrupt_handler(const struct device *dev, void *user_data);
static void uart_interrupt_handler(const struct device *dev, void *user_data);
static void configure_leds(void);
static void configure_usb_cdc(void);
static void configure_uart_1mbps(void);
static void configure_gazelle(void);
static void gazelle_rx_handler(gazelle_packet_t *packet);
static void gazelle_tx_complete_handler(uint8_t pipe_id, bool success);
static void gazelle_error_handler(uint8_t error_code);

/* LED status functions */
static void led_set_usb_status(bool connected)
{
    gpio_pin_set_dt(&led0, connected ? 1 : 0);
}

static void led_set_gazelle_status(bool connected)
{
    gpio_pin_set_dt(&led1, connected ? 1 : 0);
}

static void led_set_activity(bool active)
{
    gpio_pin_set_dt(&led2, active ? 1 : 0);
}

/* USB CDC ACM interrupt handler */
static void usb_cdc_interrupt_handler(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    
    uint8_t buffer[BUFFER_SIZE];
    int bytes_read;
    
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            bytes_read = uart_fifo_read(dev, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                LOG_DBG("USB RX: %d bytes", bytes_read);
                
                /* Put data into ring buffer for Gazelle transmission */
                uint32_t written = ring_buf_put(&usb_to_gazelle_rb, buffer, bytes_read);
                if (written < bytes_read) {
                    LOG_WRN("USB RX buffer overflow, lost %d bytes", bytes_read - written);
                }
                
                led_set_activity(true);
            }
        }
    }
}

/* UART interrupt handler */
static void uart_interrupt_handler(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    
    uint8_t buffer[BUFFER_SIZE];
    int bytes_read;
    
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            bytes_read = uart_fifo_read(dev, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                LOG_DBG("UART RX: %d bytes", bytes_read);
                
                /* Put data into ring buffer for USB transmission */
                uint32_t written = ring_buf_put(&gazelle_to_usb_rb, buffer, bytes_read);
                if (written < bytes_read) {
                    LOG_WRN("UART RX buffer overflow, lost %d bytes", bytes_read - written);
                }
                
                led_set_activity(true);
            }
        }
    }
}

/* Configure LEDs for status indication */
static void configure_leds(void)
{
    if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) || !gpio_is_ready_dt(&led2)) {
        LOG_ERR("LEDs not ready");
        return;
    }
    
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    
    LOG_INF("LEDs configured");
}

/* Configure USB CDC ACM */
static void configure_usb_cdc(void)
{
    int ret;
    
    if (!device_is_ready(usb_dev)) {
        LOG_ERR("USB CDC ACM device not ready");
        return;
    }
    
    /* Enable USB device */
    ret = usb_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return;
    }
    
    /* Set up interrupt handler */
    uart_irq_callback_user_data_set(usb_dev, usb_cdc_interrupt_handler, NULL);
    uart_irq_rx_enable(usb_dev);
    
    LOG_INF("USB CDC ACM configured");
    led_set_usb_status(true);
}

/* Configure UART for 1Mbps communication */
static void configure_uart_1mbps(void)
{
    struct uart_config uart_cfg = {
        .baudrate = 1000000,    /* 1Mbps */
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return;
    }
    
    int ret = uart_configure(uart_dev, &uart_cfg);
    if (ret != 0) {
        LOG_ERR("Failed to configure UART: %d", ret);
        return;
    }
    
    /* Set up interrupt handler */
    uart_irq_callback_user_data_set(uart_dev, uart_interrupt_handler, NULL);
    uart_irq_rx_enable(uart_dev);
    
    LOG_INF("UART configured for 1Mbps");
}

/* Configure Gazelle protocol */
static void configure_gazelle(void)
{
    int ret;
    
    ret = gazelle_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize Gazelle: %d", ret);
        return;
    }
    
    ret = gazelle_set_callbacks(gazelle_rx_handler, gazelle_tx_complete_handler, 
                               gazelle_error_handler);
    if (ret != 0) {
        LOG_ERR("Failed to set Gazelle callbacks: %d", ret);
        return;
    }
    
    ret = gazelle_set_channel(GAZELLE_CHANNEL);
    if (ret != 0) {
        LOG_ERR("Failed to set Gazelle channel: %d", ret);
        return;
    }
    
    ret = gazelle_set_power(0); /* 0 dBm */
    if (ret != 0) {
        LOG_ERR("Failed to set Gazelle power: %d", ret);
        return;
    }
    
    ret = gazelle_enable();
    if (ret != 0) {
        LOG_ERR("Failed to enable Gazelle: %d", ret);
        return;
    }
    
    LOG_INF("Gazelle configured and enabled");
    led_set_gazelle_status(true);
}

/* Gazelle RX handler */
static void gazelle_rx_handler(gazelle_packet_t *packet)
{
    if (packet == NULL || packet->length == 0) {
        LOG_WRN("Invalid Gazelle packet received");
        return;
    }
    
    LOG_DBG("Gazelle RX: pipe=%d, len=%d, rssi=%d", 
            packet->pipe_id, packet->length, packet->rssi);
    
    /* Put received data into ring buffer for USB transmission */
    uint32_t written = ring_buf_put(&gazelle_to_usb_rb, packet->data, packet->length);
    if (written < packet->length) {
        LOG_WRN("Gazelle RX buffer overflow, lost %d bytes", 
                packet->length - written);
    }
    
    led_set_activity(true);
}

/* Gazelle TX complete handler */
static void gazelle_tx_complete_handler(uint8_t pipe_id, bool success)
{
    if (success) {
        LOG_DBG("Gazelle TX success on pipe %d", pipe_id);
    } else {
        LOG_WRN("Gazelle TX failed on pipe %d", pipe_id);
    }
}

/* Gazelle error handler */
static void gazelle_error_handler(uint8_t error_code)
{
    LOG_ERR("Gazelle error: %d", error_code);
    
    /* Flash error LED pattern */
    for (int i = 0; i < 5; i++) {
        led_set_gazelle_status(false);
        k_msleep(100);
        led_set_gazelle_status(true);
        k_msleep(100);
    }
}

/* Data bridge thread */
static void data_bridge_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    uint8_t buffer[BUFFER_SIZE];
    uint32_t bytes_available;
    int bytes_written;
    int ret;
    
    LOG_INF("Data bridge thread started");
    
    while (1) {
        /* Forward data from USB to Gazelle */
        bytes_available = ring_buf_get(&usb_to_gazelle_rb, buffer, sizeof(buffer));
        if (bytes_available > 0) {
            /* Send data via Gazelle in chunks if necessary */
            uint32_t offset = 0;
            while (offset < bytes_available) {
                uint8_t chunk_size = MIN(GAZELLE_MAX_PAYLOAD, bytes_available - offset);
                
                ret = gazelle_send_packet(0, &buffer[offset], chunk_size);
                if (ret != 0) {
                    LOG_WRN("Failed to send Gazelle packet: %d", ret);
                    /* Put remaining data back in ring buffer */
                    ring_buf_put(&usb_to_gazelle_rb, &buffer[offset], 
                               bytes_available - offset);
                    break;
                }
                
                offset += chunk_size;
                LOG_DBG("USB->Gazelle: %d bytes", chunk_size);
            }
        }
        
        /* Forward data from Gazelle to USB */
        bytes_available = ring_buf_get(&gazelle_to_usb_rb, buffer, sizeof(buffer));
        if (bytes_available > 0) {
            bytes_written = uart_fifo_fill(usb_dev, buffer, bytes_available);
            if (bytes_written < bytes_available) {
                LOG_WRN("USB TX partial write: %d/%d bytes", bytes_written, bytes_available);
                /* Put remaining data back in ring buffer */
                ring_buf_put(&gazelle_to_usb_rb, &buffer[bytes_written], 
                           bytes_available - bytes_written);
            } else {
                LOG_DBG("Gazelle->USB: %d bytes", bytes_written);
            }
        }
        
        /* Brief sleep to prevent busy-waiting */
        k_msleep(1);
        
        /* Turn off activity LED */
        led_set_activity(false);
    }
}

/* Define the data bridge thread */
K_THREAD_DEFINE(bridge_thread, 2048, data_bridge_thread, NULL, NULL, NULL, 5, 0, 0);

/* Main function */
int main(void)
{
    #if GAZELLE_DEVICE_MODE == 0
    LOG_INF("nRF52840 USB-UART-Gazelle Bridge starting in HOST mode...");
    #else
    LOG_INF("nRF52840 USB-UART-Gazelle Bridge starting in DEVICE mode...");
    #endif
    
    /* Configure hardware */
    configure_leds();
    configure_usb_cdc();
    configure_uart_1mbps();
    configure_gazelle();
    
    LOG_INF("Bridge initialized, ready for operation");
    
    /* Main loop for status monitoring */
    while (1) {
        /* Monitor buffer usage and implement flow control */
        uint32_t usb_to_gazelle_used = ring_buf_size_get(&usb_to_gazelle_rb);
        uint32_t gazelle_to_usb_used = ring_buf_size_get(&gazelle_to_usb_rb);
        
        /* Log buffer usage warnings */
        if (usb_to_gazelle_used > RING_BUF_SIZE * 0.8 || 
            gazelle_to_usb_used > RING_BUF_SIZE * 0.8) {
            LOG_WRN("Buffer usage high - USB->Gazelle: %d, Gazelle->USB: %d", 
                    usb_to_gazelle_used, gazelle_to_usb_used);
        }
        
        /* Monitor Gazelle connectivity */
        if (!gazelle_is_enabled()) {
            LOG_ERR("Gazelle connection lost, attempting to re-enable");
            led_set_gazelle_status(false);
            
            /* Attempt to re-enable Gazelle */
            if (gazelle_enable() == 0) {
                LOG_INF("Gazelle re-enabled successfully");
                led_set_gazelle_status(true);
            } else {
                LOG_ERR("Failed to re-enable Gazelle");
            }
        }
        
        /* Log statistics periodically */
        static uint32_t last_stats_time = 0;
        uint32_t current_time = k_uptime_get_32();
        if (current_time - last_stats_time >= 10000) { /* Every 10 seconds */
            LOG_INF("Stats - Sent: %d, Received: %d, Lost: %d", 
                    gazelle_get_packets_sent(),
                    gazelle_get_packets_received(), 
                    gazelle_get_packets_lost());
            last_stats_time = current_time;
        }
        
        /* Sleep for status monitoring interval */
        k_msleep(1000);
    }
    
    return 0;
}