#include "setting.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void parseTriggerSetting(const char* command, Setting* setting, int* char_parsed, const size_t command_size);

void configSetting(const char* command, Setting* setting)
{
    const size_t command_size = strlen(command);
    int char_parsed           = 0;

    switch (command[char_parsed++])
    {
        case 't':  // trigger
            switch (command[char_parsed++])
            {
                case 'u':  // up
                    setting->trigger_direction = UP;
                    parseTriggerSetting(command, setting, &char_parsed, command_size);
                    break;
                case 'd':  // down
                    setting->trigger_direction = DOWN;
                    parseTriggerSetting(command, setting, &char_parsed, command_size);
                    break;
                case 's':  // stop
                    setting->trigger_direction = STOP;
                    break;

                default:
                    printf("ERROR: command '%s'is not recognized in char %d\r\n", command, char_parsed);
                    break;
            }
            return;

            break;
        case 's':  // adc speed
            printf("ERROR: command '%s' is not setup yet\r\n", command);
            break;
        case 'o':  // output
            printf("ERROR: command '%s' is not setup yet\r\n", command);
            break;

        default:
            printf("ERROR: command '%s' is not setup yet\r\n", command);
            break;
    }
}

static void parseTriggerSetting(const char* command, Setting* setting, int* char_parsed, const size_t command_size)
{
    while (command_size > *char_parsed)
    {
        switch (command[(*char_parsed)++])
        {
            case 'v':  // volt
                if (command_size - *char_parsed > 0)
                {
                    setting->trigger_volt = atoi(command + *char_parsed);
                }
                break;
            case 'r':  // rewind
                if (command_size - *char_parsed > 0)
                {
                    setting->rewind = atoi(command + *char_parsed);
                }
                break;
        }
    }
}
