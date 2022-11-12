#ifndef setting_H
#define setting_H
#include <stdint.h>

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

#endif  // setting_H