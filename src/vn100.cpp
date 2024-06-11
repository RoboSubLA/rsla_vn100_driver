#include <iostream>
#include "vn/ezasyncdata.h"
#include "vn/thread.h"

#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rsla_interfaces/msg/imu.hpp"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

using namespace std::chrono_literals;

const string SensorPort = "/dev/serial/by-id/usb-VectorNav_Dev_Board_VN7ZFTL7-if00-port0";
const uint32_t SensorBaudrate = 115200;


int main(int argc, char *argv[])
{
    EzAsyncData* ez = EzAsyncData::connect(SensorPort, SensorBaudrate);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vn100");
    auto pub = node->create_publisher<rsla_interfaces::msg::Imu>("/rsla/controls/imu", 10);
    auto imu_msg = rsla_interfaces::msg::Imu();
    while(rclcpp::ok()) {
        CompositeData cd = ez->getNextData();
        if(cd.hasYawPitchRoll()) {
            imu_msg.orientation.yaw = cd.yawPitchRoll()[0];
            imu_msg.orientation.pitch = cd.yawPitchRoll()[1];
            imu_msg.orientation.roll = cd.yawPitchRoll()[2];
        }
        if(cd.hasAngularRate()) {
            imu_msg.angular_velocity.x = cd.angularRate().x;
            imu_msg.angular_velocity.y = cd.angularRate().y;
            imu_msg.angular_velocity.z = cd.angularRate().z;
        }
        if(cd.hasAccelerationLinearBody()) {
            imu_msg.linear_acceleration.x = cd.accelerationLinearBody().x;
            imu_msg.linear_acceleration.y = cd.accelerationLinearBody().y;
            imu_msg.linear_acceleration.z = cd.accelerationLinearBody().z;
        }

        pub->publish(imu_msg);
    }
    rclcpp::shutdown();
    printf("exited\n");

	ez->disconnect();

	delete ez;

	return 0;
}
