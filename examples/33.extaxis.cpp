#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)

int main()
{
    JAKAZuRobot robot;
    
    int ret = robot.login_in("127.0.0.1");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    robot.ext_power_on();
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ExtStatus ext_status;
        robot.ext_get_status(1, ext_status);
        if (ext_status.powered)
        {
            break;
        }
        printf("enabled: %d, powered on: %d, pos %lf\n", ext_status.enabled, ext_status.powered, ext_status.pos);
    }


    robot.ext_enable_on();
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ExtStatus ext_status;
        robot.ext_get_status(1, ext_status);
        if (ext_status.enabled)
        {
            break;
        }
        printf("enabled: %d, powered on: %d, pos %lf\n", ext_status.enabled, ext_status.powered, ext_status.pos);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot.ext_jog_to(0, 10, 10, 10);



    robot.login_out();
    return 0;
}
