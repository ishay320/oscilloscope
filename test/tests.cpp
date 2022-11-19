#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <string>
#include <vector>

#include "doctest.h"

extern "C"
{
#include "../Inc/setting.h"
}

#define _POS_ __FILE__ ":" + std::to_string(__LINE__) + ": "

void check_setting(const std::string subcase, const Setting* setting, int stride, uint16_t trigger_volt, int send_size, TriggerDirection trigger_direction, int rewind)
{
    SUBCASE(subcase.c_str())
    {
        CHECK_EQ(setting->send_size, send_size);
        CHECK_EQ(setting->stride, stride);
        CHECK_EQ(setting->trigger_direction, trigger_direction);
        CHECK_EQ(setting->trigger_volt, trigger_volt);
        CHECK_EQ(setting->rewind, rewind);
    }
}

TEST_CASE("setting struct functions")
{
    Setting setting = {0};

    // check for up, down and stop
    configSetting("td", &setting);
    check_setting(_POS_ + "td", &setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::DOWN, 0);
    configSetting("tu", &setting);
    check_setting(_POS_ + "tu", &setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::UP, 0);
    configSetting("ts", &setting);
    check_setting(_POS_ + "ts", &setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::STOP, 0);

    // check the number receiving on volt
    configSetting("tdv100", &setting);
    check_setting(_POS_ + "tdv100", &setting, 0, /* trigger_volt */ 100, 0, TriggerDirection::DOWN, 0);
    configSetting("tuv50", &setting);
    check_setting(_POS_ + "tuv50", &setting, 0, /* trigger_volt */ 50, 0, TriggerDirection::UP, 0);
    configSetting("tuv5", &setting);
    check_setting(_POS_ + "tuv5", &setting, 0, /* trigger_volt */ 5, 0, TriggerDirection::UP, 0);

    // check the number receiving on rewind
    configSetting("tdv100r100", &setting);
    check_setting(_POS_ + "tdv100r100", &setting, 0, 100, 0, TriggerDirection::DOWN, /* rewind */ 100);
    configSetting("tuv50r50", &setting);
    check_setting(_POS_ + "tuv50r50", &setting, 0, 50, 0, TriggerDirection::UP, /* rewind */ 50);
    configSetting("tuv5r5", &setting);
    check_setting(_POS_ + "tuv5r5", &setting, 0, 5, 0, TriggerDirection::UP, /* rewind */ 5);
    configSetting("tuv0r0", &setting);
    check_setting(_POS_ + "tuv0r0", &setting, 0, 0, 0, TriggerDirection::UP, /* rewind */ 0);

    // check if stop at random numbers
    configSetting("tdvp56", &setting);
    check_setting(_POS_ + "tdvp56", &setting, 0, 0, 0, TriggerDirection::DOWN, 0);
    configSetting("tdv56p", &setting);
    check_setting(_POS_ + "tdv56p", &setting, 0, /* trigger_volt */ 56, 0, TriggerDirection::DOWN, 0);
}