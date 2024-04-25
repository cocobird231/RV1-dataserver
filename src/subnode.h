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


/**
 * @brief Set the BaseSubNode record flag.
 * @param[in] node BaseSubNode pointer.
 * @param[in] flag Record flag.
 */
void SetBaseSubNodeRecord(std::shared_ptr<BaseSubNode> node, bool flag);



/**
 * @brief Base class for subscriber nodes.
 * @note This class can only be created by derived classes.
 */
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
    /**
     * @brief Get the record message.
     * @param[in] resetCnt Reset the message count.
     * @return Base record message pointer.
     */
    std::shared_ptr<BaseRecordMsg> getRecordMsg(bool resetCnt = true)
    {
        std::lock_guard<std::mutex> lk(this->msgMtx_);
        if (resetCnt)
            this->msgCnt_ = 0;
        return this->rMsg_;
    }
};



/**
 * @brief Subscriber node class. The class will create a subscriber to the specified topic.
 * @tparam msgT Message type. Usually a ROS2 message type.
 * @note The default record message will copy the ROS2 message directly under `make_record_msg` function template.
 * @note User can modify the content of the record message by specializing the `make_record_msg` function template.
 */
template <typename msgT>
class SubNode : public BaseSubNode
{
public:
    /**
     * @brief SubNode constructor.
     * @param[in] nodeName Node name.
     * @param[in] topicName Topic name.
     */
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



/**
 * @brief Subscriber node class. The class will create a subscriber to the specified topic.
 * @tparam msgT Message type. Usually a ROS2 message type.
 * @tparam rmsgT Self-defined record message type.
 * @note The conversion function from msgT to rmsgT must be defined by `make_record_msg_2` function template specialization.
 */
template <typename msgT, typename rmsgT>
class SubNode2 : public BaseSubNode
{
public:
    /**
     * @brief SubNode2 constructor.
     * @param[in] nodeName Node name.
     * @param[in] topicName Topic name.
     */
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



/**
 * @brief Subscriber node class. The class will create a subscriber to the specified topic.
 * @tparam msgT Message type. Usually a ROS2 message type.
 * @tparam rmsgT Self-defined record message type.
 * @tparam smsgT Data type for saving. The data type must be copyable.
 * @note The conversion function from msgT to rmsgT must be defined by `make_record_msg_3` function template specialization.
 * @note The data saving method must be defined by `make_record_msg_3` function template specialization.
 * @note The `SaveQueue::_saveCbFunc` function template must specialize for the `smsgT` type.
 */
template <typename msgT, typename rmsgT, typename smsgT>
class SaveQueueSubNode : public BaseSubNode
{
private:
    const fs::path outputDirPath_;

public:
    /**
     * @brief SaveQueueSubNode constructor.
     * @param[in] nodeName Node name.
     * @param[in] topicName Topic name.
     * @param[in] saveQueue Save queue pointer.
     * @param[in] outputDirPath Output directory path.
     * @note The output directory will be set to `outputDirPath / nodeName`.
     * @note The output directory will be created if it does not exist.
     */
    SaveQueueSubNode(const std::string nodeName, const std::string topicName, std::shared_ptr<SaveQueue<smsgT> > saveQueue, const fs::path outputDirPath) : 
        BaseSubNode(nodeName, topicName), 
        outputDirPath_(outputDirPath / nodeName)
    {
        {// Create output directory for save queue.
            if (!fs::exists(this->outputDirPath_))
                fs::create_directories(this->outputDirPath_);
        }

        this->sub_ = this->create_subscription<msgT>(this->topicName, 10, 
            [this, saveQueue, outputDirPath](const std::shared_ptr<msgT> msg)
            {
                if (!this->recordF_)
                    return;
                RecordMsgHeader rHd;
                rHd.record_stamp = this->get_clock()->now().seconds();
                rHd.record_frame_id = this->msgCnt_++;
                this->setRecordMsg(std::move(make_record_msg_3<msgT, rmsgT, smsgT>(rHd, msg, saveQueue, this->outputDirPath_)));
            });
    }
};



/**
 * Function implementation.
 */

void SetBaseSubNodeRecord(std::shared_ptr<BaseSubNode> node, bool flag)
{
    node->recordF_ = flag;
}