#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <string>
#include <vector>

#include "doctest.h"

extern "C"
{
#include "../Inc/setting.h"
}

void check_setting(const Setting* setting, int stride, uint16_t trigger_volt, int send_size, TriggerDirection trigger_direction)
{
    CHECK(setting->send_size == send_size);
    CHECK(setting->stride == stride);
    CHECK(setting->trigger_direction == trigger_direction);
    CHECK(setting->trigger_volt == trigger_volt);
}

TEST_CASE("setting struct functions")
{
    Setting setting = {0};

    // check for up and down
    configSetting("td", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::DOWN);
    configSetting("tu", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::UP);
    configSetting("ts", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::STOP);

    // check the number receiving
    configSetting("td100", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 100, 0, TriggerDirection::DOWN);
    configSetting("tu50", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 50, 0, TriggerDirection::UP);
    configSetting("tu5", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 5, 0, TriggerDirection::UP);

    // check if stop at random numbers
    configSetting("tdp56", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::DOWN);
    configSetting("tdp56", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 0, 0, TriggerDirection::DOWN);
    configSetting("td56p", &setting);
    check_setting(&setting, 0, /* trigger_volt */ 56, 0, TriggerDirection::DOWN);
}