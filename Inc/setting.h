#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(TEST)
#include <stdio.h>
#define CDC_Transmit_FS(c,s) puts((char*)c)
#else
#include "usbd_cdc_if.h"
#endif // TEST


typedef enum
{
    UP,
    DOWN,
    STOP
} TriggerDirection;

typedef struct
{
    int stride;
    uint16_t trigger_volt;
    int send_size;
    TriggerDirection trigger_direction;
} Setting;

void configSetting(const char* command, Setting* setting);