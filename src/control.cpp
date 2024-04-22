#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/data_server_status.hpp"
#include "vehicle_interfaces/srv/data_server.hpp"

#define SERVICE_NAME "dataserver_0"
#define NODE_NAME "dataservertest_0_node"

using namespace std::chrono_literals;

std::atomic<bool> __global_exit_flag = false;

class Params : public rclcpp::Node
{
public:
    bool control_enable_scan;
    bool control_enable_sample;
    bool control_enable_dump;
    bool control_enable_countdown;
    double control_scan_period_ms;
    double control_sample_period_ms;
    double control_dump_period_s;
    double control_countdown_duration_s;
    std::string serviceName;
    std::string nodeName;

public:
    Params(const std::string & node_name) : rclcpp::Node(node_name)
    {
        declare_parameter("control_enable_scan", true);
        declare_parameter("control_enable_sample", true);
        declare_parameter("control_enable_dump", true);
        declare_parameter("control_enable_countdown", true);
        declare_parameter("control_scan_period_ms", 1000.0);
        declare_parameter("control_sample_period_ms", 100.0);
        declare_parameter("control_dump_period_s", 10.0);
        declare_parameter("control_countdown_duration_s", 5.0);
        declare_parameter("serviceName", SERVICE_NAME);
        declare_parameter("nodeName", NODE_NAME);

        get_parameter("control_enable_scan", control_enable_scan);
        get_parameter("control_enable_sample", control_enable_sample);
        get_parameter("control_enable_dump", control_enable_dump);
        get_parameter("control_enable_countdown", control_enable_countdown);
        get_parameter("control_scan_period_ms", control_scan_period_ms);
        get_parameter("control_sample_period_ms", control_sample_period_ms);
        get_parameter("control_dump_period_s", control_dump_period_s);
        get_parameter("control_countdown_duration_s", control_countdown_duration_s);
        get_parameter("serviceName", serviceName);
        get_parameter("nodeName", nodeName);
    }
};


vehicle_interfaces::ReasonResult<bool> SendRequest(std::string serviceName, vehicle_interfaces::srv::DataServer::Request::SharedPtr req, vehicle_interfaces::msg::DataServerStatus& dst)
{
    auto node = rclcpp::Node::make_shared("dataservertest_tmp_node");
    auto client = node->create_client<vehicle_interfaces::srv::DataServer>(serviceName);
    auto result = client->async_send_request(req);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        dst = res->status;
        return { res->response, res->reason };
    }
    return { false, "Request failed." };
}

void PrintDataServerStatus(const vehicle_interfaces::msg::DataServerStatus status)
{
    std::cout << "============================" << std::endl;
    std::cout << "Server status: " << std::endl;
    std::cout << "  Server scan timer status: " << (status.server_scan_timer_status == vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_START ? "Start" : "Stop") << std::endl;
    std::cout << "  Server scan period: " << status.server_scan_period_ms << " ms" << std::endl;
    std::cout << "  Server sample timer status: " << (status.server_sample_timer_status == vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_START ? "Start" : "Stop") << std::endl;
    std::cout << "  Server sample period: " << status.server_sample_period_ms << " ms" << std::endl;
    std::cout << "  Server dump timer status: " << (status.server_dump_timer_status == vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_START ? "Start" : "Stop") << std::endl;
    std::cout << "  Server dump period: " << status.server_dump_period_ms << " ms" << std::endl;
    std::cout << "  Server countdown timer status: " << (status.server_countdown_timer_status == vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_START ? "Start" : "Stop") << std::endl;
    std::cout << "  Server countdown period: " << status.server_countdown_period_ms << " ms" << std::endl;
    std::cout << "============================" << std::endl;
}

int main(int argc, char * argv[])
{
    // ctrl-c handler
    signal(SIGINT, 
        [](int)
        {
            __global_exit_flag = true;
        });

    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("dataservertest_params_node");

    printf("/** \n\
 * Start record: \n\
 *   st\n\
 * Stop record: \n\
 *   stp\n\
 * \n\
 * Set timer status: \n\
 *   t <timer_id> <TIMER_STATUS_XXX>\n\
 *   p <timer_id> <period>\n\
 * Quit: \n\
 *   q\n\
 */\n");

    while (!__global_exit_flag)
    {
        std::this_thread::sleep_for(100ms);
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);

        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");
        if (inputStrVec.size() == 1 && inputStrVec[0] == "q")
            __global_exit_flag = true;
        else if (inputStrVec.size() == 1)
        {
            auto req = std::make_shared<vehicle_interfaces::srv::DataServer::Request>();
            vehicle_interfaces::msg::DataServerStatus dst;
            if (inputStrVec[0] == "st")
                req->request.server_action = vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_START;
            else if (inputStrVec[0] == "stp")
                req->request.server_action = vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_STOP;
            else
                continue;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
                PrintDataServerStatus(dst);
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec.size() == 3 && inputStrVec[0] == "t")// Set timer status.
        {
            auto req = std::make_shared<vehicle_interfaces::srv::DataServer::Request>();
            vehicle_interfaces::msg::DataServerStatus dst;
            req->request.server_action = vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_SET_TIMER;
            if (inputStrVec[1] == "0")// Scan timer.
                req->request.server_scan_timer_status = std::stoi(inputStrVec[2]);
            else if (inputStrVec[1] == "1")// Sample timer.
                req->request.server_sample_timer_status = std::stoi(inputStrVec[2]);
            else if (inputStrVec[1] == "2")// Dump timer.
                req->request.server_dump_timer_status = std::stoi(inputStrVec[2]);
            else if (inputStrVec[1] == "3")// Countdown timer.
                req->request.server_countdown_timer_status = std::stoi(inputStrVec[2]);
            else
                continue;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
                PrintDataServerStatus(dst);
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec.size() == 3 && inputStrVec[0] == "p")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::DataServer::Request>();
            vehicle_interfaces::msg::DataServerStatus dst;
            req->request.server_action = vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_SET_PERIOD;
            if (inputStrVec[1] == "0")// Scan timer.
                req->request.server_scan_period_ms = std::stod(inputStrVec[2]);
            else if (inputStrVec[1] == "1")// Sample timer.
                req->request.server_sample_period_ms = std::stod(inputStrVec[2]);
            else if (inputStrVec[1] == "2")// Dump timer.
                req->request.server_dump_period_ms = std::stod(inputStrVec[2]);
            else if (inputStrVec[1] == "3")// Countdown timer.
                req->request.server_countdown_period_ms = std::stod(inputStrVec[2]);
            else
                continue;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
                PrintDataServerStatus(dst);
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else
            continue;
    }

    rclcpp::shutdown();
    return 0;
}