#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

#define WIFI_SSID       "LA_OCULTA"
#define WIFI_PASSWORD   "Makala123"
#define WIFI_MAX_RETRY  10

void wifi_manager_init(void);
bool wifi_manager_is_connected(void);

#endif // WIFI_MANAGER_H
