#include "scannode.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Params> params;
    {
        auto tmp = std::make_shared<Params>("tmp_dataserver_params_node");
        params = std::make_shared<Params>(tmp->nodeName + "_params");
        std::cout << "scan_period_ms: " << params->scan_period_ms << std::endl;
        std::cout << "sampling_period_ms: " << params->sampling_period_ms << std::endl;
        std::cout << "dump_period_s: " << params->dump_period_s << std::endl;
        std::cout << "countdown_duration_s: " << params->countdown_duration_s << std::endl;
    }
    auto node = std::make_shared<ScanNode>(params);
    if (!params->enable_control)
    {
        node->startRecord();
    }

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}