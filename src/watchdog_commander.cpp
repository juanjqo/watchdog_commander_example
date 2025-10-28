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

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("watchdog_commander_example");

    // 1 ms clock
    sas::Clock clock{0.001};
    clock.init();

    // Initialize the RobotDriverClient
    sas::RobotDriverClient rdi(node, "/sas_mobile_robot/pioneer_1");

    int i=0;



    // Get topic information
    RCLCPP_INFO_STREAM(node->get_logger(),"topic_prefix = " << rdi.get_topic_prefix());

    // For some iterations. Note that this can be stopped with CTRL+C.
    while (!kill_this_process)
    {
        clock.update_and_sleep();
        if (i<10000)
        {
            RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"Watchdog: true");
            rdi.send_watchdog_trigger(true);
        }
        else
        {
            RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"Watchdog: false");
            rdi.send_watchdog_trigger(false);
        }
        i++;
        rclcpp::spin_some(node);
    }


}
