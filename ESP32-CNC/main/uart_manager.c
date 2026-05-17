#include "uart_manager.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UART_MANAGER";

#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_TX_PIN     21        /* GPIO por defecto UART0 ESP32-C3 */
#define UART_RX_PIN     20
#define UART_BUF_SIZE   256
#define UART_QUEUE_SIZE 10

#define OK_STRING       "ok"
#define OK_STRING_LEN   3

static QueueHandle_t s_uart_queue;

/* Bandera global accesible desde main y gcode_parser */
volatile bool uart_ok_flag = false;

/* ── Tarea UART (actua como ISR de software via eventos) ────────────────── */
/*
 * ESP-IDF no expone ISR directa para UART de forma sencilla en C3.
 * Se usa la uart_event_queue (patron recomendado) en una tarea de alta
 * prioridad que setea uart_ok_flag al detectar "OK\n" en el RX.
 */
static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t buf[UART_BUF_SIZE];

    for (;;) {
        if (xQueueReceive(s_uart_queue, &event, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Evento tipo: %d", event.type);  // <- agrega esto
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_PORT, buf,
                                         event.size, pdMS_TO_TICKS(20));
                if (len > 0) {
                    buf[len] = '\0';
                    /* Buscar "OK" en los bytes recibidos */
                    if (strstr((char *)buf, OK_STRING) != NULL) {
                        uart_ok_flag = true;
                        ESP_LOGI(TAG, "OK recibido por UART");
                    }
                }
            }
        }
    }
}

/* ── Inicializacion publica ─────────────────────────────────────────────── */
void uart_manager_init(void)
{
    uart_driver_delete(UART_NUM_0);
    
    uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    //gpio_set_pull_mode(UART_RX_PIN, GPIO_PULLUP_ONLY);
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2,
                                        UART_QUEUE_SIZE, &s_uart_queue, 0));

    /* Tarea de alta prioridad para detectar OK */
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    ESP_LOGI(TAG, "UART0 iniciado a %d baud", UART_BAUD_RATE);
}

/* ── Envio de linea por UART ────────────────────────────────────────────── */
void uart_send_line(const char *line)
{
    if (!line) return;
    uart_write_bytes(UART_PORT, line, strlen(line));
    uart_write_bytes(UART_PORT, "\n", 1);
    ESP_LOGI(TAG, "TX: %s", line);
}
