#include "gcode_parser.h"
#include "uart_manager.h"

#include "esp_spiffs.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "GCODE_PARSER";

static char  s_lines[GCODE_MAX_LINES][GCODE_MAX_LINE_LEN];
static int   s_line_count = 0;
static FILE *s_file       = NULL;

/* ── SPIFFS ─────────────────────────────────────────────────────────────── */
void gcode_parser_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = SPIFFS_BASE_PATH,
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error montando SPIFFS (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS montado en %s", SPIFFS_BASE_PATH);
    }
}

/* ── Escritura del archivo recibido por HTTP ────────────────────────────── */
void gcode_parser_file_open_write(void)
{
    s_file = fopen(GCODE_FILE_PATH, "w");
    if (!s_file) {
        ESP_LOGE(TAG, "No se pudo abrir %s para escritura", GCODE_FILE_PATH);
    }
}

void gcode_parser_file_write_chunk(const char *data, int len)
{
    if (s_file && data && len > 0) {
        fwrite(data, 1, len, s_file);
    }
}

void gcode_parser_file_close(void)
{
    if (s_file) {
        fclose(s_file);
        s_file = NULL;
        ESP_LOGI(TAG, "Archivo cerrado en SPIFFS");
    }
}

/* ── Carga lineas desde SPIFFS ──────────────────────────────────────────── */
void gcode_parser_load_lines(void)
{
    s_line_count = 0;

    FILE *f = fopen(GCODE_FILE_PATH, "r");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir %s para lectura", GCODE_FILE_PATH);
        return;
    }

    char buf[GCODE_MAX_LINE_LEN];
    while (s_line_count < GCODE_MAX_LINES &&
           fgets(buf, sizeof(buf), f) != NULL)
    {
        /* Eliminar '\n' al final */
        int len = strlen(buf);
        if (len > 0 && buf[len - 1] == '\n') buf[len - 1] = '\0';
        if (len > 1 && buf[len - 2] == '\r') buf[len - 2] = '\0';

        /* Ignorar lineas vacias */
        if (strlen(buf) == 0) continue;

        strncpy(s_lines[s_line_count], buf, GCODE_MAX_LINE_LEN - 1);
        s_lines[s_line_count][GCODE_MAX_LINE_LEN - 1] = '\0';
        s_line_count++;
    }

    fclose(f);
    ESP_LOGI(TAG, "%d lineas cargadas desde SPIFFS", s_line_count);
}

/* ── Getters y envio ────────────────────────────────────────────────────── */
int gcode_parser_get_line_count(void)
{
    return s_line_count;
}

void gcode_parser_send_line(int index)
{
    if (index >= 0 && index < s_line_count) {
        uart_send_line(s_lines[index]);
    }
}
