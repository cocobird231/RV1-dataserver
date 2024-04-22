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

#include "rclcpp/rclcpp.hpp"

#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/vehicle_interfaces.h"

#include "record_msg.h"
#include "savequeue.h"

class BaseSubNode;

void SetBaseSubNodeRecord(std::shared_ptr<BaseSubNode> node, bool flag);

class BaseSubNode : public rclcpp::Node
{
private:
    std::shared_ptr<BaseRecordMsg> rMsg_;
    std::mutex msgMtx_;

    friend void SetBaseSubNodeRecord(std::shared_ptr<BaseSubNode> node, bool flag);

protected:
    rclcpp::SubscriptionBase::SharedPtr sub_;
    std::atomic<size_t> msgCnt_;
    std::atomic<bool> recordF_;

public:
    const std::string nodeName;
    const std::string topicName;

protected:
    BaseSubNode(const std::string& nodeName, const std::string& topicName) : 
        rclcpp::Node(nodeName), 
        recordF_(false), 
        nodeName(nodeName), 
        topicName(topicName) {}

    virtual ~BaseSubNode() {}

    void setRecordMsg(std::shared_ptr<BaseRecordMsg>&& rMsg)
    {
        std::lock_guard<std::mutex> lk(this->msgMtx_);
        this->rMsg_ = std::move(rMsg);
    }

public:
    std::shared_ptr<BaseRecordMsg> getRecordMsg(bool resetCnt = true)
    {
        std::lock_guard<std::mutex> lk(this->msgMtx_);
        if (resetCnt)
            this->msgCnt_ = 0;
        return this->rMsg_;
    }
};


template <typename msgT>
class SubNode : public BaseSubNode
{
public:
    SubNode(const std::string nodeName, const std::string topicName) : 
        BaseSubNode(nodeName, topicName) 
    {
        this->sub_ = this->create_subscription<msgT>(this->topicName, 10, 
            [this](const std::shared_ptr<msgT> msg)
            {
                if (!this->recordF_)
                    return;
                RecordMsgHeader rHd;
                rHd.record_stamp = this->get_clock()->now().seconds();
                rHd.record_frame_id = this->msgCnt_++;
                this->setRecordMsg(std::move(make_record_msg<msgT>(rHd, msg)));
            });
    }
};



template <typename msgT, typename rmsgT>
class SubNode2 : public BaseSubNode
{
public:
    SubNode2(const std::string nodeName, const std::string topicName) : 
        BaseSubNode(nodeName, topicName) 
    {
        this->sub_ = this->create_subscription<msgT>(this->topicName, 10, 
            [this](const std::shared_ptr<msgT> msg)
            {
                if (!this->recordF_)
                    return;
                RecordMsgHeader rHd;
                rHd.record_stamp = this->get_clock()->now().seconds();
                rHd.record_frame_id = this->msgCnt_++;
                this->setRecordMsg(std::move(make_record_msg_2<msgT, rmsgT>(rHd, msg)));
            });
    }
};



template <typename msgT, typename rmsgT, typename smsgT>
class SaveQueueSubNode : public BaseSubNode
{
public:
    SaveQueueSubNode(const std::string nodeName, const std::string topicName, std::shared_ptr<SaveQueue<smsgT> > saveQueue, const fs::path outputDirPath) : 
        BaseSubNode(nodeName, topicName)
    {
        this->sub_ = this->create_subscription<msgT>(this->topicName, 10, 
            [this, saveQueue, outputDirPath](const std::shared_ptr<msgT> msg)
            {
                if (!this->recordF_)
                    return;
                RecordMsgHeader rHd;
                rHd.record_stamp = this->get_clock()->now().seconds();
                rHd.record_frame_id = this->msgCnt_++;
                this->setRecordMsg(std::move(make_record_msg_3<msgT, rmsgT, smsgT>(rHd, msg, saveQueue, outputDirPath / this->nodeName)));
            });
    }
};

void SetBaseSubNodeRecord(std::shared_ptr<BaseSubNode> node, bool flag)
{
    node->recordF_ = flag;
}