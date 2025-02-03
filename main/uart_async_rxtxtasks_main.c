/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "soc/uart_periph.h"

static const int RX_BUF_SIZE = 1024;
static const int RX_DATA_SIZE = 50000;
// esp32s3 free pin
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_2)

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, TXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

// connect RX & TX pin without wire, reeset tx connection
    gpio_set_direction(TXD_PIN, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(TXD_PIN, UART_PERIPH_SIGNAL(UART_NUM_1, SOC_UART_TX_PIN_IDX), 0, 0);

}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    uint8_t* data_txd = (uint8_t*) calloc(RX_DATA_SIZE + 1,1);
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        const int txBytes = uart_write_bytes(UART_NUM_1, data_txd, RX_DATA_SIZE);
        ESP_LOGI("UART_TX_TASK", "Wrote %d bytes", txBytes);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_DATA_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_DATA_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes", rxBytes);
        }
    }
    free(data);
}
void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 2, NULL);
}
