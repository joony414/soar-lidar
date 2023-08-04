#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <linux/types.h>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include <lidarlite_v3/lidarlite_v3.h>

using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node
{
public:
    LidarNode()
    : Node("lidar_node"), altitude_(0)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        altitude_publisher_ = this->create_publisher<std_msgs::msg::UInt16>("/lidar_node/altitude", qos_profile);
        timer_ = this->create_wall_timer(0.1s, std::bind(&LidarNode::publish_altitude, this));

        int initalized = myLidarLite_.i2c_init();
        if(initalized == 0 ) {
        // printf("Initialization Success");
        } else {
        // printf("Initialization Fail");
        }
        myLidarLite_.configure(0);
    }

private:
    void publish_altitude()
    {
        auto msg = std_msgs::msg::UInt16();
        busyFlag = myLidarLite_.getBusyFlag();
        if (busyFlag == 0x00)
        {
            // When no longer busy, immediately initialize another measurement
            // and then read the distance data from the last measurement.
            // This method will result in faster I2C rep rates.
            myLidarLite_.takeRange();
            altitude_ = myLidarLite_.readDistance();
            msg.data = altitude_;
            RCLCPP_INFO(this->get_logger(), "Altitude : %d cm", msg.data);
            altitude_publisher_->publish(msg);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr altitude_publisher_;
    uint16_t altitude_;
    LIDARLite_v3 myLidarLite_;
    __u8  busyFlag;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}