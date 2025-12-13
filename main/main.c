/*
 * ESP32-C6 USB to CAN Adapter with SN65HVD230 Transceiver
 *
 * This firmware implements a USB to CAN bus adapter using:
 * - ESP32-C6's built-in TWAI (CAN controller)
 * - SN65HVD230 CAN transceiver (physical layer)
 *
 * Hardware connections:
 * - ESP32-C6 GPIO0  → SN65HVD230 D (TXD)
 * - ESP32-C6 GPIO1  → SN65HVD230 R (RXD)
 * - ESP32-C6 3.3V   → SN65HVD230 VCC
 * - ESP32-C6 GND    → SN65HVD230 GND
 * - SN65HVD230 Rs   → 10kΩ to GND (high-speed mode)
 * - SN65HVD230 CANH → CAN Bus CANH (with 120Ω termination)
 * - SN65HVD230 CANL → CAN Bus CANL (with 120Ω termination)
 * - 100nF capacitor between VCC and GND (close to transceiver)
 *
 * Features:
 * - USB CDC (serial) interface for host communication (/dev/ttyACM*)
 * - SLCAN protocol support for compatibility with can-utils
 * - CAN 2.0A/2.0B support via TWAI
 * - Configurable bitrates (125k, 250k, 500k, 1M)
 * - LED status indicator
 */

// Set log level - use ESP_LOG_NONE for production with slcan tools
#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "USB2CAN";

// ESP32-C6 TWAI (CAN) pins - connects to SN65HVD230
#define TWAI_TX_GPIO    GPIO_NUM_16  // → SN65HVD230 D pin
#define TWAI_RX_GPIO    GPIO_NUM_17  // → SN65HVD230 R pin

// LED indicator
#define LED_GPIO        GPIO_NUM_8

// SLCAN protocol state
typedef enum {
    SLCAN_STATE_CLOSED,
    SLCAN_STATE_OPEN,
    SLCAN_STATE_LISTEN
} slcan_state_t;

static slcan_state_t slcan_state = SLCAN_STATE_CLOSED;
static uint8_t usb_rx_buffer[256];
static size_t usb_rx_pos = 0;

// Current bitrate configuration
static uint32_t current_bitrate = 500000; // Default 500kbps

// Bitrate configurations for different speeds
typedef struct {
    uint32_t baudrate;
    twai_timing_config_t timing;
} bitrate_config_t;

// Predefined bitrates (optimized for ESP32-C6 at 80MHz APB clock)
static const bitrate_config_t bitrate_configs[] = {
    {125000,  {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}},
    {250000,  {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}},
    {500000,  {.brp = 8,  .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}},
    {1000000, {.brp = 4,  .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}},
};

// Initialize TWAI (CAN controller)
static esp_err_t init_twai(uint32_t bitrate)
{
    // Find matching bitrate config
    const twai_timing_config_t *timing = NULL;
    for (int i = 0; i < sizeof(bitrate_configs) / sizeof(bitrate_configs[0]); i++) {
        if (bitrate_configs[i].baudrate == bitrate) {
            timing = &bitrate_configs[i].timing;
            break;
        }
    }

    if (!timing) {
        ESP_LOGE(TAG, "Unsupported bitrate: %lu", bitrate);
        return ESP_ERR_INVALID_ARG;
    }

    // Configure TWAI general settings
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    g_config.alerts_enabled = TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;

    // Configure TWAI filter (accept all messages)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, timing, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "TWAI driver installed successfully");
    ESP_LOGI(TAG, "  TX Pin: GPIO%d → SN65HVD230 D", TWAI_TX_GPIO);
    ESP_LOGI(TAG, "  RX Pin: GPIO%d → SN65HVD230 R", TWAI_RX_GPIO);
    ESP_LOGI(TAG, "  Bitrate: %lu bps", bitrate);

    // Start TWAI driver
    err = twai_start();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TWAI driver started - CAN bus active");
    } else {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }

    return ESP_OK;
}

// Deinitialize TWAI
static void deinit_twai(void)
{
    twai_stop();
    twai_driver_uninstall();
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
        ESP_LOGI(TAG, "  Device will appear as /dev/ttyACM* on Linux");
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
                ESP_LOGI(TAG, "SLCAN: Channel OPENED");
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10); // Bell (error)
                ESP_LOGW(TAG, "SLCAN: Channel already open");
            }
            break;

        case 'L': // Open in listen-only mode
            if (slcan_state == SLCAN_STATE_CLOSED) {
                slcan_state = SLCAN_STATE_LISTEN;
                usb_serial_jtag_write_bytes("\r", 1, 10);
                ESP_LOGI(TAG, "SLCAN: Channel OPENED (listen-only)");
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10);
                ESP_LOGW(TAG, "SLCAN: Channel already open");
            }
            break;

        case 'C': // Close CAN channel
            slcan_state = SLCAN_STATE_CLOSED;
            usb_serial_jtag_write_bytes("\r", 1, 10);
            ESP_LOGI(TAG, "SLCAN: Channel CLOSED");
            break;

        case 'S': // Setup with standard CAN bit-rates
            if (slcan_state == SLCAN_STATE_CLOSED && len >= 2) {
                // S0=10k, S1=20k, S2=50k, S3=100k, S4=125k, S5=250k, S6=500k, S7=800k, S8=1M
                const uint32_t bitrates[] = {10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000};
                int idx = cmd[1] - '0';

                if (idx >= 0 && idx <= 8) {
                    deinit_twai();
                    if (init_twai(bitrates[idx]) == ESP_OK) {
                        current_bitrate = bitrates[idx];
                        usb_serial_jtag_write_bytes("\r", 1, 10);
                        ESP_LOGI(TAG, "SLCAN: Bitrate set to %lu bps", bitrates[idx]);
                    } else {
                        usb_serial_jtag_write_bytes("\x07", 1, 10);
                        ESP_LOGE(TAG, "SLCAN: Failed to set bitrate");
                    }
                } else {
                    usb_serial_jtag_write_bytes("\x07", 1, 10);
                }
            } else {
                usb_serial_jtag_write_bytes("\x07", 1, 10);
                ESP_LOGW(TAG, "SLCAN: Channel must be closed to change bitrate");
            }
            break;

        case 'V': // Get hardware version
            snprintf(response, sizeof(response), "V0101\r");
            usb_serial_jtag_write_bytes(response, strlen(response), 10);
            ESP_LOGI(TAG, "SLCAN: Version query");
            break;

        case 'N': // Get serial number
            snprintf(response, sizeof(response), "N2024\r");
            usb_serial_jtag_write_bytes(response, strlen(response), 10);
            ESP_LOGI(TAG, "SLCAN: Serial number query");
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

                // Send message via TWAI → SN65HVD230 → CAN bus
                esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
                if (err == ESP_OK) {
                    usb_serial_jtag_write_bytes("z\r", 2, 10); // Success
                    ESP_LOGI(TAG, "CAN TX: ID=0x%03lX %s DLC=%d",
                             tx_msg.identifier, tx_msg.extd ? "EXT" : "STD", tx_msg.data_length_code);
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
            ESP_LOGW(TAG, "SLCAN: Unknown command '%c'", cmd[0]);
            break;
    }
}

// Task to handle USB to CAN direction (SLCAN commands)
static void usb_to_can_task(void *arg)
{
    ESP_LOGI(TAG, "USB→CAN task started");

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
                    usb_rx_buffer[i] = '\0';
                    process_slcan_command((char *)usb_rx_buffer, i);

                    // Shift remaining data
                    size_t remaining = usb_rx_pos - i - 1;
                    if (remaining > 0) {
                        memmove(usb_rx_buffer, usb_rx_buffer + i + 1, remaining);
                    }
                    usb_rx_pos = remaining;
                    i = 0;
                }
            }

            // Prevent buffer overflow
            if (usb_rx_pos >= sizeof(usb_rx_buffer) - 1) {
                ESP_LOGW(TAG, "USB RX buffer overflow");
                usb_rx_pos = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Task to handle CAN to USB direction (receive CAN messages)
static void can_to_usb_task(void *arg)
{
    twai_message_t rx_msg;
    char tx_buffer[64];

    ESP_LOGI(TAG, "CAN→USB task started");

    while (1) {
        // Wait for CAN message from SN65HVD230 → TWAI
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            ESP_LOGI(TAG, "CAN RX: ID=0x%03lX %s DLC=%d",
                     rx_msg.identifier, rx_msg.extd ? "EXT" : "STD", rx_msg.data_length_code);

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
        if (twai_read_alerts(&alerts, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
                ESP_LOGW(TAG, "CAN: Surpassed error warning threshold");
            }
            if (alerts & TWAI_ALERT_BUS_ERROR) {
                ESP_LOGW(TAG, "CAN: Bus error detected");
            }
            if (alerts & TWAI_ALERT_TX_FAILED) {
                ESP_LOGW(TAG, "CAN: TX failed (no ACK or arbitration lost)");
            }
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGW(TAG, "CAN: RX queue full - messages lost!");
            }
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGE(TAG, "CAN: BUS OFF! Starting recovery...");
                twai_initiate_recovery();
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                ESP_LOGI(TAG, "CAN: Bus recovered from Bus-Off");
            }
        }

        // Periodic status check every 30 seconds
        static uint32_t last_status_time = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (current_time - last_status_time >= 30000) {
            if (twai_get_status_info(&status) == ESP_OK) {
                ESP_LOGI(TAG, "CAN Status: State=%s, TxErr=%lu, RxErr=%lu",
                         (status.state == TWAI_STATE_RUNNING) ? "RUNNING" :
                         (status.state == TWAI_STATE_BUS_OFF) ? "BUS_OFF" : "STOPPED",
                         status.tx_error_counter, status.rx_error_counter);
            }
            last_status_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// LED blink task (shows device is alive)
static void led_task(void *arg)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32-C6 USB to CAN Adapter");
    ESP_LOGI(TAG, "  with SN65HVD230 Transceiver");
    ESP_LOGI(TAG, "  Firmware Version: 2.0.0");
    ESP_LOGI(TAG, "========================================");

    // Initialize USB Serial
    ESP_LOGI(TAG, "Initializing USB Serial JTAG...");
    init_usb_serial();

    // Initialize TWAI (CAN controller) with default bitrate
    ESP_LOGI(TAG, "Initializing TWAI (CAN Controller)...");
    if (init_twai(current_bitrate) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TWAI - check connections!");
    }

    // Create tasks
    ESP_LOGI(TAG, "Creating communication tasks...");
    xTaskCreate(usb_to_can_task, "usb_to_can", 4096, NULL, 5, NULL);
    xTaskCreate(can_to_usb_task, "can_to_usb", 4096, NULL, 5, NULL);
    xTaskCreate(can_status_task, "can_status", 3072, NULL, 4, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  USB2CAN Adapter Ready!");
    ESP_LOGI(TAG, "  USB: /dev/ttyACM*");
    ESP_LOGI(TAG, "  CAN: GPIO0/1 → SN65HVD230");
    ESP_LOGI(TAG, "  Protocol: SLCAN");
    ESP_LOGI(TAG, "  Default Bitrate: %lu bps", current_bitrate);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Hardware Checklist:");
    ESP_LOGI(TAG, "  [?] GPIO0 → SN65HVD230 pin 1 (D/TXD)");
    ESP_LOGI(TAG, "  [?] GPIO1 → SN65HVD230 pin 4 (R/RXD)");
    ESP_LOGI(TAG, "  [?] 3.3V → SN65HVD230 pin 3 (VCC)");
    ESP_LOGI(TAG, "  [?] GND → SN65HVD230 pin 2 (GND)");
    ESP_LOGI(TAG, "  [?] Rs pin 8 → 10kΩ to GND");
    ESP_LOGI(TAG, "  [?] 100nF cap between VCC and GND");
    ESP_LOGI(TAG, "  [?] 120Ω termination resistors installed");
    ESP_LOGI(TAG, "========================================");
}
