#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdbool.h>

#define GCODE_MAX_LINES     2000
#define GCODE_MAX_LINE_LEN  64
#define SPIFFS_BASE_PATH    "/spiffs"
#define GCODE_FILE_PATH     "/spiffs/gcode.nc"

/* Inicializacion: monta SPIFFS */
void gcode_parser_init(void);

/* Llamadas desde http_manager para escribir el archivo en SPIFFS */
void gcode_parser_file_open_write(void);
void gcode_parser_file_write_chunk(const char *data, int len);
void gcode_parser_file_close(void);

/* Carga lineas desde SPIFFS al array interno */
void gcode_parser_load_lines(void);

/* Devuelve numero de lineas cargadas */
int  gcode_parser_get_line_count(void);

/* Envia la linea indicada por UART */
void gcode_parser_send_line(int index);

#endif // GCODE_PARSER_H
