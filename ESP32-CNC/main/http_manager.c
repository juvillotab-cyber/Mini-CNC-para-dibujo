#include "http_manager.h"
#include "gcode_parser.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "HTTP_MANAGER";

#define RECV_CHUNK_SIZE 512

static bool s_start_requested = false;

/* ── Handler POST /upload ───────────────────────────────────────────────── */
/*
 * Flask envia el contenido raw del archivo .nc como body del POST.
 * Se recibe en chunks y se reenvía directamente a gcode_parser para
 * que lo guarde en SPIFFS.
 */
static esp_err_t upload_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int received  = 0;
    char *chunk   = malloc(RECV_CHUNK_SIZE);

    if (!chunk) {
        ESP_LOGE(TAG, "Sin memoria para chunk");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Recibiendo archivo .nc (%d bytes)", total_len);

    gcode_parser_file_open_write();   /* Abre /spiffs/gcode.nc para escritura */

    while (received < total_len) {
        int to_read = (total_len - received < RECV_CHUNK_SIZE)
                      ? (total_len - received)
                      : RECV_CHUNK_SIZE;

        int ret = httpd_req_recv(req, chunk, to_read);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "Error recibiendo datos");
            gcode_parser_file_close();
            free(chunk);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        gcode_parser_file_write_chunk(chunk, ret);  /* Escribe chunk en SPIFFS */
        received += ret;
    }

    gcode_parser_file_close();     /* Cierra el archivo en SPIFFS */
    gcode_parser_load_lines();     /* Parsea lineas desde SPIFFS al array interno */

    free(chunk);

    ESP_LOGI(TAG, "Archivo recibido y almacenado en SPIFFS (%d bytes)", received);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* ── Handler POST /start ────────────────────────────────────────────────── */
static esp_err_t start_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Boton START recibido");
    s_start_requested = true;
    httpd_resp_sendstr(req, "START_OK");
    return ESP_OK;
}

/* ── Inicializacion publica ─────────────────────────────────────────────── */
void http_manager_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port    = 80;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando servidor HTTP");
        return;
    }

    /* Registrar URI /upload */
    httpd_uri_t upload_uri = {
        .uri      = "/upload",
        .method   = HTTP_POST,
        .handler  = upload_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &upload_uri);

    /* Registrar URI /start */
    httpd_uri_t start_uri = {
        .uri      = "/start",
        .method   = HTTP_POST,
        .handler  = start_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &start_uri);

    ESP_LOGI(TAG, "Servidor HTTP iniciado en puerto %d", config.server_port);
}

bool http_manager_is_start_requested(void)
{
    return s_start_requested;
}

void http_manager_clear_start_flag(void)
{
    s_start_requested = false;
}
