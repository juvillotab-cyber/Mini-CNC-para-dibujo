#ifndef UART_MANAGER_H
#define UART_MANAGER_H

#include <stdbool.h>

/* Bandera seteada por la ISR cuando llega "OK" por UART */
extern volatile bool uart_ok_flag;

void uart_manager_init(void);
void uart_send_line(const char *line);

#endif // UART_MANAGER_H
