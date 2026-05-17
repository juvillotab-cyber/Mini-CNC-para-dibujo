#include "wifi_manager.h"
#include "http_manager.h"
#include "gcode_parser.h"
#include "uart_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

/* ── Estados de la maquina de envio ─────────────────────────────────────── */
typedef enum {
    STATE_WAIT_START,    /* Esperando boton /start por HTTP      */
    STATE_SEND_LINE,     /* Enviar linea actual por UART         */
    STATE_WAIT_OK,       /* Esperar OK del dispositivo por UART  */
    STATE_DONE           /* Todas las lineas enviadas            */
} gcode_state_t;

/* ── Tarea principal ─────────────────────────────────────────────────────── */
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    /* 1. Inicializar modulos */
    wifi_manager_init();
    gcode_parser_init();   /* Monta SPIFFS */
    uart_manager_init();
    http_manager_init();   /* Levanta servidor HTTP */

    ESP_LOGI(TAG, "Sistema listo. Esperando archivo .nc y boton START...");

    /* 2. Variables de control */
    gcode_state_t state    = STATE_WAIT_START;
    int           line_idx = 0;

    /* 3. Loop principal — sin delays fijos, solo polling de banderas */
    while(1) {
        switch (state) {

        /* ── Esperar que Flask mande POST /start ── */
        case STATE_WAIT_START:
            if (http_manager_is_start_requested()) {
                http_manager_clear_start_flag();
                line_idx = 0;

                if (gcode_parser_get_line_count() == 0) {
                    ESP_LOGW(TAG, "No hay lineas cargadas, ignorando START");
                    break;
                }

                ESP_LOGI(TAG, "START recibido. Enviando %d lineas...",
                         gcode_parser_get_line_count());
                state = STATE_SEND_LINE;
            }
            break;

        /* ── Enviar linea actual ── */
        case STATE_SEND_LINE:
            uart_ok_flag = false;                          /* Limpiar bandera antes de enviar */
            gcode_parser_send_line(line_idx);
            ESP_LOGI(TAG, "Linea %d/%d enviada", line_idx + 1,
                     gcode_parser_get_line_count());
            state = STATE_WAIT_OK;
            break;

        /* ── Esperar OK por UART (seteada por uart_event_task) ── */
        case STATE_WAIT_OK:
            if (uart_ok_flag) {
                uart_ok_flag = false;
                line_idx++;

                if (line_idx >= gcode_parser_get_line_count()) {
                    uart_send_line("f");         /* Apagar motores */
                    ESP_LOGI(TAG, "Todas las lineas enviadas. Trabajo completado.");
                    state = STATE_DONE;
                } else {
                    state = STATE_SEND_LINE;   /* Siguiente linea de inmediato */
                }
            }
            break;

        /* ── Trabajo terminado — volver a esperar un nuevo archivo ── */
        case STATE_DONE:
            /* Queda esperando un nuevo /upload + /start */
            state = STATE_WAIT_START;
            break;
        }

        /* Ceder CPU brevemente para no bloquear el scheduler */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
