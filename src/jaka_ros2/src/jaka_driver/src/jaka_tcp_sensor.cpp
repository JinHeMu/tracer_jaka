#include "JAKAZuRobot.h"
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

JAKAZuRobot robot;

int main(void)
{
    int ret,cur_sensor;
    setlocale(LC_ALL, "");
    // robot.login_in(argv[1]);
    string default_ip = "10.5.5.100";
    robot.login_in(default_ip.c_str());
    robot.set_torque_sensor_mode(1);  




    return 0;
}
