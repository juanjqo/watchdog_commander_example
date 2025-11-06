#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <dqrobotics/utils/DQ_Math.h>

using namespace DQ_robotics;

#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}


void send_watchdog_trigger(const Publisher<sas_msgs::msg::WatchdogTrigger> ::SharedPtr& pub,
                           const bool& watchdog_trigger_status)
{
    sas_msgs::msg::WatchdogTrigger ros_msg;
    ros_msg.header = std_msgs::msg::Header();
    ros_msg.header.stamp = rclcpp::Clock().now();
    ros_msg.status = watchdog_trigger_status;
    pub->publish(ros_msg);
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("watchdog_commander_example");

    // 100 ms clock
    sas::Clock clock{0.05};
    clock.init();



    auto rdi_1 = node->create_publisher<sas_msgs::msg::WatchdogTrigger>(std::string("/sas_b1/b1_1") + "/set/watchdog_trigger", 1);
    auto rdi_2 = node->create_publisher<sas_msgs::msg::WatchdogTrigger>(std::string("/sas_b1/b1_2") + "/set/watchdog_trigger", 1);
    auto rdi_3 = node->create_publisher<sas_msgs::msg::WatchdogTrigger>(std::string("/sas_z1/z1_1") + "/set/watchdog_trigger", 1);
    auto rdi_4 = node->create_publisher<sas_msgs::msg::WatchdogTrigger>(std::string("/sas_z1/z1_2") + "/set/watchdog_trigger", 1);

    // Initialize the RobotDriverClient
   // sas::RobotDriverClient rdi_1(node, "/sas_b1/b1_1");
   // sas::RobotDriverClient rdi_2(node, "/sas_b1/b1_2");
   // sas::RobotDriverClient rdi_3(node, "/sas_z1/z1_1");
   // sas::RobotDriverClient rdi_4(node, "/sas_z1/z1_2");

   // int i=0;



    // Get topic information
    //RCLCPP_INFO_STREAM(node->get_logger(),"topic_prefix = " << rdi_1.get_topic_prefix());
    //RCLCPP_INFO_STREAM(node->get_logger(),"topic_prefix = " << rdi_2.get_topic_prefix());
    //RCLCPP_INFO_STREAM(node->get_logger(),"topic_prefix = " << rdi_3.get_topic_prefix());
    //RCLCPP_INFO_STREAM(node->get_logger(),"topic_prefix = " << rdi_4.get_topic_prefix());
    // For some iterations. Note that this can be stopped with CTRL+C.
    while (!kill_this_process)
    {
        rclcpp::spin_some(node);
        clock.update_and_sleep();
        rclcpp::spin_some(node);
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"Watchdog: true");
        send_watchdog_trigger(rdi_1, true);
        send_watchdog_trigger(rdi_2, true);
        send_watchdog_trigger(rdi_3, true);
        send_watchdog_trigger(rdi_4, true);
    }
}
