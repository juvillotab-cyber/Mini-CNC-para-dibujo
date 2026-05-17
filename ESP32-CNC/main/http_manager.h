#ifndef HTTP_MANAGER_H
#define HTTP_MANAGER_H

#include <stdbool.h>

void http_manager_init(void);
bool http_manager_is_start_requested(void);
void http_manager_clear_start_flag(void);

#endif // HTTP_MANAGER_H
