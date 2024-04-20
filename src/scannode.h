#pragma once
#include <iostream>
#include <iomanip>
#include <fstream>

#include <memory>

#include <string>
#include <vector>
#include <deque>

#include <atomic>
#include <mutex>
#include <thread>
#include <map>
#include <set>

#include "rclcpp/rclcpp.hpp"

#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/vehicle_interfaces.h"

#include "dump_json.h"
#include "record_msg.h"
#include "savequeue.h"
#include "subnode.h"



class Params : public vehicle_interfaces::GenericParams
{
private:
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _paramsCallbackHandler;
    std::function<void(const rclcpp::Parameter)> cbFunc_;
    std::atomic<bool> cbFuncSetF_;

public:
    std::vector<std::string> msg_filter = { "vehicle_interfaces" };
    double scan_period_ms = 1000.0;
    double sampling_period_ms = 10.0;
    double dump_period_s = 60.0;
    double countdown_duration_s = -1.0;
    int img_threads = 4;
    int gnd_threads = 1;
    std::string dump_path;
    bool enable_control = true;

    bool control_enable_monitor = false;
    bool control_enable_countdown = false;
    double control_scan_period_ms = 1000.0;
    double control_sampling_period_ms = 10.0;
    double control_dump_period_s = 60.0;
    double control_countdown_duration_s = -1.0;

private:
    void _getParams()
    {
        this->get_parameter("msg_filter", this->msg_filter);
        this->get_parameter("scan_period_ms", this->scan_period_ms);
        this->get_parameter("sampling_period_ms", this->sampling_period_ms);
        this->get_parameter("dump_period_s", this->dump_period_s);
        this->get_parameter("countdown_duration_s", this->countdown_duration_s);
        this->get_parameter("img_threads", this->img_threads);
        this->get_parameter("gnd_threads", this->gnd_threads);
        this->get_parameter("dump_path", this->dump_path);
        this->get_parameter("enable_control", this->enable_control);

        this->get_parameter("control_enable_monitor", this->control_enable_monitor);
        this->get_parameter("control_enable_countdown", this->control_enable_countdown);
        this->get_parameter("control_scan_period_ms", this->control_scan_period_ms);
        this->get_parameter("control_sampling_period_ms", this->control_sampling_period_ms);
        this->get_parameter("control_dump_period_s", this->control_dump_period_s);
        this->get_parameter("control_countdown_duration_s", this->control_countdown_duration_s);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult ret;
        ret.successful = true;
        ret.reason = "";

        if (!this->cbFuncSetF_)
            return ret;

        for (const auto& param : params)
        {
            try
            {
                this->cbFunc_(param);
            }
            catch (...)
            {
                ret.successful = false;
                ret.reason = "[Params::_paramsCallback] Caught Unknown Exception!";
            }
        }

        return ret;
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName), 
        cbFuncSetF_(false)
    {
        this->declare_parameter<std::vector<std::string> >("msg_filter", this->msg_filter);
        this->declare_parameter<double>("scan_period_ms", this->scan_period_ms);
        this->declare_parameter<double>("sampling_period_ms", this->sampling_period_ms);
        this->declare_parameter<double>("dump_period_s", this->dump_period_s);
        this->declare_parameter<double>("countdown_duration_s", this->countdown_duration_s);
        this->declare_parameter<int>("img_threads", this->img_threads);
        this->declare_parameter<int>("gnd_threads", this->gnd_threads);
        this->declare_parameter<std::string>("dump_path", this->dump_path);
        this->declare_parameter<bool>("enable_control", this->enable_control);

        this->declare_parameter<bool>("control_enable_monitor", this->control_enable_monitor);
        this->declare_parameter<bool>("control_enable_countdown", this->control_enable_countdown);
        this->declare_parameter<double>("control_scan_period_ms", this->control_scan_period_ms);
        this->declare_parameter<double>("control_sampling_period_ms", this->control_sampling_period_ms);
        this->declare_parameter<double>("control_dump_period_s", this->control_dump_period_s);
        this->declare_parameter<double>("control_countdown_duration_s", this->control_countdown_duration_s);
        this->_getParams();

        this->_paramsCallbackHandler = this->add_on_set_parameters_callback(std::bind(&Params::_paramsCallback, this, std::placeholders::_1));
    }

    void addCallbackFunc(const std::function<void(const rclcpp::Parameter)>& callback)
    {
        this->cbFunc_ = callback;
        this->cbFuncSetF_ = true;
    }
};


struct RecordMsgTag
{
    double ts;// Timestamp.
    std::string topicName;// Topic name.
};


class ScanNode : public rclcpp::Node
{
private:
    std::shared_ptr<Params> params_;
    std::set<std::string> ifFilterSet_;// Interface filter set.
    std::map<std::string, std::pair<std::string, bool> > scannedTopicMap_;// { topicName : (msgType, isCreated), ... }
    std::shared_ptr<SaveQueue<cv::Mat> > imgSaveQue_;// Image save queue.
    fs::path dumpDirPath_;// Working directory for dumping files.

    std::unique_ptr<vehicle_interfaces::LiteTimer> scanTm_;// Scan topics timer.
    std::unique_ptr<vehicle_interfaces::LiteTimer> sampleTm_;// Sampling timer.
    std::unique_ptr<vehicle_interfaces::LiteTimer> dumpTm_;// Dump JSON timer.
    std::unique_ptr<vehicle_interfaces::LiteTimer> countdownTm_;// Countdown timer.

    std::map<std::string, std::shared_ptr<BaseSubNode> > subNodes_;// { topicName : BaseSubNode, ... }
    std::map<std::string, rclcpp::executors::SingleThreadedExecutor::SharedPtr> subNodeExecMap_;// { topicName : SingleThreadedExecutor, ... }
    std::map<std::string, vehicle_interfaces::unique_thread> subNodeThMap_;// { topicName : unique_thread, ... }
    std::mutex subNodesMtx_;// Lock for subNodes_, subNodeExecMap_, and subNodeThMap_.


    using RecordMsgs = std::deque<std::pair<RecordMsgTag, std::shared_ptr<BaseRecordMsg> > >;// [<RecordMsgTag, BaseRecordMsg>, ...]

    RecordMsgs recordMsgs_;// [<RecordMsgTag, BaseRecordMsg>, ...]
    RecordMsgs recordMsgsBk_;// [<RecordMsgTag, BaseRecordMsg>, ...]
    RecordMsgs* recordMsgsPtr_;// Pointer to the current record message queue.
    std::atomic<bool> recordMsgsBkF_;// Flag describing which record message queue is being used.
    std::mutex recordMsgsMtx_;// Lock for recordMsgsPtr_ and recordMsgsBkF_.

    std::mutex dumpMtx_;// Ensure only one thread access the _dumpJSON().
    std::atomic<bool> recordF_;// Record flag.

private:
    void _scanTmCbFunc()
    {
        // Search topics
        auto map = this->get_topic_names_and_types();
        for (const auto& [topicName, msgTypeVec] : map)
        {
            auto msgTypeSplit = vehicle_interfaces::split(msgTypeVec.back(), "/");// Format: "<interfaces_pkg_name>/msg/<msg_type>".
            if (msgTypeSplit.size() != 3)
                continue;
            if (this->ifFilterSet_.find(msgTypeSplit[0]) == this->ifFilterSet_.end() || msgTypeSplit[1] != "msg")// Interface not in the filter set, or not a message type.
                continue;

            if (this->scannedTopicMap_.find(topicName) == this->scannedTopicMap_.end())// New topic.
            {
                this->scannedTopicMap_[topicName] = {msgTypeSplit[2], false};
                RCLCPP_INFO(this->get_logger(), "[ScanNode::_scanTmCbFunc] New topic: [%s] (%s)", topicName.c_str(), msgTypeVec.back().c_str());
            }
        }

        std::deque<std::string> toCreateSubNodeQue;

        for (auto& [topicName, status] : this->scannedTopicMap_)
        {
            auto& [msgType, isCreatedF] = status;
            if (!isCreatedF)// Not created yet.
            {
                // Check if the message type is supported.
                if (msgType == "Chassis" || 
                    msgType == "Distance" || 
                    msgType == "Environment" || 
                    msgType == "GPS" || 
                    msgType == "IDTable" || 
                    msgType == "Image" || 
                    msgType == "IMU" || 
                    msgType == "MillitBrakeMotor" || 
                    msgType == "MillitPowerMotor" || 
                    msgType == "MotorAxle" || 
                    msgType == "MotorSteering" || 
                    msgType == "QosUpdate" || 
                    msgType == "SteeringWheel" || 
                    msgType == "UPS" || 
                    msgType == "WheelState")
                {
                    toCreateSubNodeQue.push_back(topicName);
                }
            }
        }

        for (const auto& topicName : toCreateSubNodeQue)
        {
            std::string subNodeName = topicName + "_subnode";
            vehicle_interfaces::replace_all(subNodeName, "/", "_");
            auto& [msgType, isCreatedF] = this->scannedTopicMap_[topicName];
            isCreatedF = true;
            std::lock_guard<std::mutex> subNodesLk(this->subNodesMtx_);
            if (msgType == "Chassis")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::Chassis> >(subNodeName, topicName);
            else if (msgType == "Distance")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::Distance> >(subNodeName, topicName);
            else if (msgType == "Environment")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::Environment> >(subNodeName, topicName);
            else if (msgType == "GPS")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::GPS> >(subNodeName, topicName);
            else if (msgType == "IDTable")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::IDTable> >(subNodeName, topicName);
            else if (msgType == "Image")
                this->subNodes_[topicName] = std::make_shared<SaveQueueSubNode<vehicle_interfaces::msg::Image, VI_ImageMsg, cv::Mat> >(subNodeName, topicName, this->imgSaveQue_, this->dumpDirPath_);
            else if (msgType == "IMU")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::IMU> >(subNodeName, topicName);
            else if (msgType == "MillitBrakeMotor")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::MillitBrakeMotor> >(subNodeName, topicName);
            else if (msgType == "MillitPowerMotor")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::MillitPowerMotor> >(subNodeName, topicName);
            else if (msgType == "MotorAxle")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::MotorAxle> >(subNodeName, topicName);
            else if (msgType == "MotorSteering")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::MotorSteering> >(subNodeName, topicName);
            else if (msgType == "QosUpdate")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::QosUpdate> >(subNodeName, topicName);
            else if (msgType == "SteeringWheel")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::SteeringWheel> >(subNodeName, topicName);
            else if (msgType == "UPS")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::UPS> >(subNodeName, topicName);
            else if (msgType == "WheelState")
                this->subNodes_[topicName] = std::make_shared<SubNode<vehicle_interfaces::msg::WheelState> >(subNodeName, topicName);
            else
            {
                RCLCPP_WARN(this->get_logger(), "[ScanNode::_scanTmCbFunc] Unsupported message type: [%s] (%s)", topicName.c_str(), msgType.c_str());
                return;
            }
            this->subNodeExecMap_[topicName] = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            this->subNodeExecMap_[topicName]->add_node(this->subNodes_[topicName]);
            this->subNodeThMap_[topicName] = vehicle_interfaces::make_unique_thread(vehicle_interfaces::SpinExecutor, this->subNodeExecMap_[topicName], subNodeName, 1000.0);
            RCLCPP_INFO(this->get_logger(), "[ScanNode::_scanTmCbFunc] SubNode created: [%s] (%s)", subNodeName.c_str(), msgType.c_str());
        }
    }

    void _sampleTmCbFunc()
    {
        std::lock_guard<std::mutex> recordMsgsLk(this->recordMsgsMtx_);
        std::lock_guard<std::mutex> subNodesLk(this->subNodesMtx_);
        double ts = this->get_clock()->now().seconds();// Timestamp of the current sample.
        for (const auto& [topicName, subNode] : this->subNodes_)
        {
            RecordMsgTag tag;
            tag.ts = ts;
            tag.topicName = topicName;
            auto msg = subNode->getRecordMsg();
            if (msg)
                this->recordMsgsPtr_-> emplace_back(std::move(tag), std::move(msg));
        }
    }

    void _dumpTmCbFunc()
    {
        this->_dumpJSON();
    }

    void _countdownTmCbFunc()
    {
        this->stopRecord();
    }

    void _dumpJSON()
    {
        std::lock_guard<std::mutex> dumpLk(this->dumpMtx_);

        RecordMsgs* dumpPtr;

        {// Swap the record message queue.
            std::lock_guard<std::mutex> recordMsgsLk(this->recordMsgsMtx_);
            if (this->recordMsgsBkF_)// Current pointer is recordMsgsBk_. Swap the pointer.
            {
                this->recordMsgsPtr_ = &this->recordMsgs_;
                dumpPtr = &this->recordMsgsBk_;
            }
            else
            {
                this->recordMsgsPtr_ = &this->recordMsgsBk_;
                dumpPtr = &this->recordMsgs_;
            }
            this->recordMsgsBkF_ = !this->recordMsgsBkF_;
        }

        RCLCPP_INFO(this->get_logger(), "[ScanNode::_dumpJSON] Record size: %lu", dumpPtr->size());
        if (dumpPtr->empty())
            return;

        double st = this->get_clock()->now().seconds();
        nlohmann::json json;
        for (auto& [tag, rMsg] : *dumpPtr)
            json[std::to_string(tag.ts)][tag.topicName] = std::move(BaseRecordMsgToJSON(rMsg));
        dumpPtr->clear();// Destruct the record messages.

        std::ofstream outFile(this->dumpDirPath_ / "json" / (std::to_string(this->get_clock()->now().seconds()) + ".json"));
        outFile << json << std::endl;
        RCLCPP_INFO(this->get_logger(), "[ScanNode::_dumpJSON] JSON dumped. Time: %.3f s", this->get_clock()->now().seconds() - st);
    }

public:
    ScanNode(const std::shared_ptr<Params> params) : 
        rclcpp::Node(params->nodeName), 
        params_(params), 
        imgSaveQue_(std::make_shared<SaveQueue<cv::Mat> >(params->img_threads)), 
        recordMsgsPtr_(&this->recordMsgs_), 
        recordMsgsBkF_(false), 
        recordF_(false)
    {
        this->ifFilterSet_ = std::set<std::string>(params->msg_filter.begin(), params->msg_filter.end());// Convert the filter list to a set.
        this->dumpDirPath_ = fs::path(params->dump_path) / std::to_string(this->get_clock()->now().seconds());

        {// Create the dump directory.
            if (!fs::exists(this->dumpDirPath_ / "json"))
                fs::create_directories(this->dumpDirPath_ / "json");
        }

        if (params->scan_period_ms > 0.0)
        {
            this->scanTm_ = vehicle_interfaces::make_unique_timer(params->scan_period_ms, std::bind(&ScanNode::_scanTmCbFunc, this));
            this->scanTm_->start();
        }

        if (params->sampling_period_ms > 0.0)
            this->sampleTm_ = vehicle_interfaces::make_unique_timer(params->sampling_period_ms, std::bind(&ScanNode::_sampleTmCbFunc, this));
        if (params->dump_period_s > 0.0)
            this->dumpTm_ = vehicle_interfaces::make_unique_timer(params->dump_period_s * 1000.0, std::bind(&ScanNode::_dumpTmCbFunc, this));
        if (params->countdown_duration_s > 0.0)
            this->countdownTm_ = vehicle_interfaces::make_unique_timer(params->countdown_duration_s * 1000.0, std::bind(&ScanNode::_countdownTmCbFunc, this));
    }

    ~ScanNode()
    {
        this->stopRecord();
        if (this->scanTm_)
            this->scanTm_->destroy();
        // Save to access all the class members.

        // Delete the executors and threads.
        for (auto& [topicName, subNodeExec] : this->subNodeExecMap_)
            subNodeExec->cancel();
        this->subNodeExecMap_.clear();
        this->subNodeThMap_.clear();
    }

    void startRecord()
    {
        if (this->recordF_)
            return;
        this->recordF_ = true;

        if (this->sampleTm_)
            this->sampleTm_->start();
        if (this->dumpTm_)
            this->dumpTm_->start();
        if (this->countdownTm_)
            this->countdownTm_->start();

        {// Enable record for all subnodes. For SaveQueueSubNode, set the output directory path.
            std::lock_guard<std::mutex> subNodesLk(this->subNodesMtx_);
            for (auto& [topicName, subNode] : this->subNodes_)
                StartBaseSubNodeRecord(subNode);
        }
    }

    void stopRecord()
    {
        if (!this->recordF_)
            return;
        this->recordF_ = false;

        {
            std::lock_guard<std::mutex> subNodesLk(this->subNodesMtx_);
            for (auto& [topicName, subNode] : this->subNodes_)
                StopBaseSubNodeRecord(subNode);
        }

        if (this->sampleTm_)
            this->sampleTm_->stop();
        if (this->dumpTm_)
            this->dumpTm_->stop();
        if (this->countdownTm_)
            this->countdownTm_->stop();

        this->_dumpJSON();// Dump the last record messages.
    }
};