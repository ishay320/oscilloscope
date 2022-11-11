#ifdef __cplusplus
extern "C"
{
#endif

#include "setting.h"

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
                        break;
                    case 'd':  // down
                        setting->trigger_direction = DOWN;
                        break;
                    case 's':  // stop
                        setting->trigger_direction = STOP;
                        break;

                    default:
                        CDC_Transmit_FS((uint8_t*)"ERROR: command is not recognized\r\n", 35);
                        break;
                }
                if (command_size - char_parsed > 0)
                {
                    setting->trigger_volt = atoi(command + char_parsed);
                }
                return;

                break;
            case 's':  // adc speed
                CDC_Transmit_FS((uint8_t*)"ERROR: command is not setup yet\r\n", 34);
                break;
            case 'o':  // output
                CDC_Transmit_FS((uint8_t*)"ERROR: command is not setup yet\r\n", 34);
                break;

            default:
                CDC_Transmit_FS((uint8_t*)"ERROR: command is not recognized\r\n", 35);
                break;
        }
    }

#ifdef __cplusplus
}
#endif