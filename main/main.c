/*
 * ESP32-C6 USB to CAN Adapter
 *
 * This firmware implements a USB to CAN bus adapter using ESP32-C6 Mini.
 * CAN interface is connected to UART0 (Tx0/Rx0) pins.
 *
 * Features:
 * - USB CDC (serial) interface for host communication
 * - CAN 2.0A/2.0B support via TWAI (Two-Wire Automotive Interface)
 * - SLCAN protocol support for compatibility with standard tools
 * - LED status indicators
 */

// Disable USB output for ESP_LOG to prevent interference with SLCAN protocol
// Set to ESP_LOG_NONE for production use with slcan_attach/slcand
// Set to ESP_LOG_INFO for debugging via JTAG monitor
#define LOG_LOCAL_LEVEL ESP_LOG_NONE

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "USB2CAN";

// Debug configuration
// Set to 0 to disable verbose CAN message logging
//
// WARNING: When set to 1, debug logs will be mixed with SLCAN protocol data
// on the USB serial port. This can interfere with Python scripts trying to
// parse SLCAN responses. Set to 0 for production use with Python scripts.
//
// Recommendation:
//   - Set to 1: During firmware development/debugging (monitor with idf.py monitor)
//   - Set to 0: When using Python scripts to control the motor
#define DEBUG_CAN_MESSAGES 0

// ESP32-C6 Mini TWAI (CAN) pins
// Note: For ESP32-C6, UART0 pins are typically GPIO16 (TX) and GPIO17 (RX)
// However, we'll use the default TWAI pins which are more suitable for CAN
#define TWAI_TX_GPIO    GPIO_NUM_0  // CAN TX
#define TWAI_RX_GPIO    GPIO_NUM_1  // CAN RX

// LED indicator (GPIO8 on ESP32-C6)
#define LED_GPIO        GPIO_NUM_8

// CAN timing configuration (500 kbps)
#define CAN_TIMING_CONFIG_500KBITS() {.brp = 8, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}

// Queues for CAN message handling
static QueueHandle_t can_tx_queue;
static QueueHandle_t can_rx_queue;

// SLCAN protocol state
typedef enum {
    SLCAN_STATE_CLOSED,
    SLCAN_STATE_OPEN,
    SLCAN_STATE_LISTEN
} slcan_state_t;

// Auto-open SLCAN at startup for compatibility with slcan_attach
// Set to SLCAN_STATE_OPEN for immediate operation, or SLCAN_STATE_CLOSED
// to require explicit 'O' command from host
static slcan_state_t slcan_state = SLCAN_STATE_CLOSED;
static uint8_t usb_rx_buffer[256];
static size_t usb_rx_pos = 0;

// Initialize TWAI (CAN) driver
static void init_twai(void)
{
    // Configure TWAI general settings
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    // Enable alerts for error monitoring
    g_config.alerts_enabled = TWAI_ALERT_ALL;

    // Configure TWAI timing (500 kbps)
    twai_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();

    // Configure TWAI filter (accept all messages)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TWAI driver installed successfully");
        ESP_LOGI(TAG, "  - TX Pin: GPIO%d", TWAI_TX_GPIO);
        ESP_LOGI(TAG, "  - RX Pin: GPIO%d", TWAI_RX_GPIO);
        ESP_LOGI(TAG, "  - Mode: Normal");
        ESP_LOGI(TAG, "  - Bitrate: 500 kbps");
    } else {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return;
    }

    // Start TWAI driver
    err = twai_start();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TWAI driver started - CAN bus active");
    } else {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
    }
}

// Initialize USB Serial JTAG
static void init_usb_serial(void)
{
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&usb_serial_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "USB Serial JTAG driver installed");
    } else {
        ESP_LOGE(TAG, "Failed to install USB Serial JTAG driver: %s", esp_err_to_name(err));
    }
}

// Convert hex character to integer
static uint8_t hex_to_int(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// Convert integer to hex character
static char int_to_hex(uint8_t i)
{
    if (i < 10) return '0' + i;
    return 'A' + (i - 10);
}

// Parse SLCAN command and execute
static void process_slcan_command(const char *cmd, size_t len)
{
    if (len == 0) return;

    char response[64];

    switch (cmd[0]) {
        case 'O': // Open CAN channel
            if (slcan_state == SLCAN_STATE_CLOSED) {
                slcan_state = SLCAN_STATE_OPEN;
                usb_serial_jtag_write_bytes("\r", 1, 10);
#if DEBUG_CAN_MESSAGES
                ESP_LOGI(TAG, "SLCAN: Channel OPENED (normal mode)");
#endif
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10); // Bell (error)
#if DEBUG_CAN_MESSAGES
                ESP_LOGW(TAG, "SLCAN: Open failed - channel already open");
#endif
            }
            break;

        case 'L': // Open in listen-only mode
            if (slcan_state == SLCAN_STATE_CLOSED) {
                slcan_state = SLCAN_STATE_LISTEN;
                usb_serial_jtag_write_bytes("\r", 1, 10);
#if DEBUG_CAN_MESSAGES
                ESP_LOGI(TAG, "SLCAN: Channel OPENED (listen-only mode)");
#endif
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10);
#if DEBUG_CAN_MESSAGES
                ESP_LOGW(TAG, "SLCAN: Listen-only open failed - channel already open");
#endif
            }
            break;

        case 'C': // Close CAN channel
            slcan_state = SLCAN_STATE_CLOSED;
            usb_serial_jtag_write_bytes("\r", 1, 10);
#if DEBUG_CAN_MESSAGES
            ESP_LOGI(TAG, "SLCAN: Channel CLOSED");
#endif
            break;

        case 'S': // Setup with standard CAN bit-rates
            if (slcan_state == SLCAN_STATE_CLOSED && len >= 2) {
                // S0-S8: 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k, 1000k
                usb_serial_jtag_write_bytes("\r", 1, 10);
#if DEBUG_CAN_MESSAGES
                const char* bitrates[] = {"10k", "20k", "50k", "100k", "125k", "250k", "500k", "800k", "1M"};
                int idx = cmd[1] - '0';
                if (idx >= 0 && idx <= 8) {
                    ESP_LOGI(TAG, "SLCAN: Bitrate set to S%d (%s bps)", idx, bitrates[idx]);
                } else {
                    ESP_LOGI(TAG, "SLCAN: Bitrate command S%c", cmd[1]);
                }
#endif
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10);
#if DEBUG_CAN_MESSAGES
                ESP_LOGW(TAG, "SLCAN: Bitrate set failed - channel must be closed");
#endif
            }
            break;

        case 'V': // Get hardware version
            snprintf(response, sizeof(response), "V0101\r");
            usb_serial_jtag_write_bytes(response, strlen(response), 10);
#if DEBUG_CAN_MESSAGES
            ESP_LOGI(TAG, "SLCAN: Version query - V0101");
#endif
            break;

        case 'N': // Get serial number
            snprintf(response, sizeof(response), "N2024\r");
            usb_serial_jtag_write_bytes(response, strlen(response), 10);
#if DEBUG_CAN_MESSAGES
            ESP_LOGI(TAG, "SLCAN: Serial number query - N2024");
#endif
            break;

        case 't': // Transmit standard frame
        case 'T': // Transmit extended frame
            if (slcan_state == SLCAN_STATE_OPEN && len >= 5) {
                twai_message_t tx_msg = {0};

                // Parse ID
                uint32_t id = 0;
                int id_len = (cmd[0] == 't') ? 3 : 8;
                for (int i = 0; i < id_len; i++) {
                    id = (id << 4) | hex_to_int(cmd[1 + i]);
                }
                tx_msg.identifier = id;
                tx_msg.extd = (cmd[0] == 'T') ? 1 : 0;

                // Parse DLC
                tx_msg.data_length_code = hex_to_int(cmd[1 + id_len]);
                if (tx_msg.data_length_code > 8) tx_msg.data_length_code = 8;

                // Parse data bytes
                for (int i = 0; i < tx_msg.data_length_code; i++) {
                    int pos = 2 + id_len + i * 2;
                    if (pos + 1 < len) {
                        tx_msg.data[i] = (hex_to_int(cmd[pos]) << 4) | hex_to_int(cmd[pos + 1]);
                    }
                }

                // Send message
                esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
                if (err == ESP_OK) {
                    usb_serial_jtag_write_bytes("z\r", 2, 10); // Success

#if DEBUG_CAN_MESSAGES
                    // Log TX message to monitor (only if debugging enabled)
                    ESP_LOGI(TAG, "CAN TX: ID=0x%03X %s DLC=%d Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                             tx_msg.identifier,
                             tx_msg.extd ? "EXT" : "STD",
                             tx_msg.data_length_code,
                             tx_msg.data[0], tx_msg.data[1], tx_msg.data[2], tx_msg.data[3],
                             tx_msg.data[4], tx_msg.data[5], tx_msg.data[6], tx_msg.data[7]);
#endif
                } else {
                    usb_serial_jtag_write_bytes("\x07", 1, 10); // Error
                    ESP_LOGE(TAG, "CAN TX Failed: %s", esp_err_to_name(err));
                }
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10);
            }
            break;

        default:
            usb_serial_jtag_write_bytes("\x07", 1, 10); // Unknown command
            break;
    }
}

// Task to handle USB to CAN direction
static void usb_to_can_task(void *arg)
{
    while (1) {
        // Read from USB
        int len = usb_serial_jtag_read_bytes(usb_rx_buffer + usb_rx_pos,
                                              sizeof(usb_rx_buffer) - usb_rx_pos - 1,
                                              pdMS_TO_TICKS(10));
        if (len > 0) {
            usb_rx_pos += len;

            // Process complete commands (terminated by \r or \n)
            for (size_t i = 0; i < usb_rx_pos; i++) {
                if (usb_rx_buffer[i] == '\r' || usb_rx_buffer[i] == '\n') {
                    // Found command terminator
                    usb_rx_buffer[i] = '\0';
                    process_slcan_command((char *)usb_rx_buffer, i);

                    // Shift remaining data
                    size_t remaining = usb_rx_pos - i - 1;
                    if (remaining > 0) {
                        memmove(usb_rx_buffer, usb_rx_buffer + i + 1, remaining);
                    }
                    usb_rx_pos = remaining;
                    i = 0; // Restart parsing
                }
            }

            // Prevent buffer overflow
            if (usb_rx_pos >= sizeof(usb_rx_buffer) - 1) {
                ESP_LOGW(TAG, "USB RX buffer overflow, clearing");
                usb_rx_pos = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Task to handle CAN to USB direction
static void can_to_usb_task(void *arg)
{
    twai_message_t rx_msg;
    char tx_buffer[64];

    while (1) {
        // Wait for CAN message
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
#if DEBUG_CAN_MESSAGES
            // Log RX message to monitor (only if debugging enabled)
            ESP_LOGI(TAG, "CAN RX: ID=0x%03X %s %s DLC=%d Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                     rx_msg.identifier,
                     rx_msg.extd ? "EXT" : "STD",
                     rx_msg.rtr ? "RTR" : "   ",
                     rx_msg.data_length_code,
                     rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                     rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
#endif

            if (slcan_state != SLCAN_STATE_CLOSED) {
                // Format message in SLCAN format
                int pos = 0;

                // Frame type and ID
                if (rx_msg.extd) {
                    tx_buffer[pos++] = 'T'; // Extended frame
                    for (int i = 7; i >= 0; i--) {
                        tx_buffer[pos++] = int_to_hex((rx_msg.identifier >> (i * 4)) & 0xF);
                    }
                } else {
                    tx_buffer[pos++] = 't'; // Standard frame
                    for (int i = 2; i >= 0; i--) {
                        tx_buffer[pos++] = int_to_hex((rx_msg.identifier >> (i * 4)) & 0xF);
                    }
                }

                // DLC
                tx_buffer[pos++] = int_to_hex(rx_msg.data_length_code);

                // Data bytes
                for (int i = 0; i < rx_msg.data_length_code; i++) {
                    tx_buffer[pos++] = int_to_hex((rx_msg.data[i] >> 4) & 0xF);
                    tx_buffer[pos++] = int_to_hex(rx_msg.data[i] & 0xF);
                }

                // Terminator
                tx_buffer[pos++] = '\r';

                // Send to USB
                usb_serial_jtag_write_bytes(tx_buffer, pos, pdMS_TO_TICKS(10));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// CAN bus status monitoring task
static void can_status_task(void *arg)
{
    twai_status_info_t status;
    uint32_t alerts;

    while (1) {
        // Check for alerts
        twai_read_alerts(&alerts, pdMS_TO_TICKS(0));

        if (alerts & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "CAN: Alert - Error Passive state");
        }
        if (alerts & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGW(TAG, "CAN: Alert - Bus error detected");
        }
        if (alerts & TWAI_ALERT_TX_FAILED) {
            ESP_LOGW(TAG, "CAN: Alert - TX failed (arbitration lost or error)");
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGW(TAG, "CAN: Alert - RX queue full, messages lost!");
        }
        if (alerts & TWAI_ALERT_TX_SUCCESS) {
            ESP_LOGD(TAG, "CAN: Alert - TX successful");
        }
        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "CAN: Alert - Bus OFF! Starting recovery...");
            twai_initiate_recovery();
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "CAN: Alert - Bus recovered from Bus OFF");
        }

        // Periodic status logging every 10 seconds
        static uint32_t last_status_time = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (current_time - last_status_time >= 10000) {
            if (twai_get_status_info(&status) == ESP_OK) {
                ESP_LOGI(TAG, "CAN Status: State=%s, TxErr=%d, RxErr=%d, TxQ=%d, RxQ=%d, Arb=%d",
                         (status.state == TWAI_STATE_RUNNING) ? "RUNNING" :
                         (status.state == TWAI_STATE_BUS_OFF) ? "BUS_OFF" :
                         (status.state == TWAI_STATE_RECOVERING) ? "RECOVERING" : "STOPPED",
                         status.tx_error_counter,
                         status.rx_error_counter,
                         status.msgs_to_tx,
                         status.msgs_to_rx,
                         status.arb_lost_count);
            }
            last_status_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// LED blink task
static void led_task(void *arg)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32-C6 USB to CAN Adapter");
    ESP_LOGI(TAG, "  Firmware Version: 1.0.0");
    ESP_LOGI(TAG, "========================================");

    // Initialize USB Serial
    ESP_LOGI(TAG, "Initializing USB Serial JTAG...");
    init_usb_serial();

    // Initialize TWAI (CAN)
    ESP_LOGI(TAG, "Initializing TWAI (CAN) driver...");
    init_twai();

    // Create tasks
    ESP_LOGI(TAG, "Creating tasks...");
    xTaskCreate(usb_to_can_task, "usb_to_can", 4096, NULL, 5, NULL);
    xTaskCreate(can_to_usb_task, "can_to_usb", 4096, NULL, 5, NULL);
    xTaskCreate(can_status_task, "can_status", 3072, NULL, 4, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  USB2CAN Adapter Ready!");
    ESP_LOGI(TAG, "  Connect via USB and use SLCAN protocol");
    ESP_LOGI(TAG, "  All CAN messages will be logged below");
    ESP_LOGI(TAG, "========================================");
}
