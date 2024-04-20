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


#include "vehicle_interfaces/msg/image.hpp"


#include "vehicle_interfaces/utils.h"

#include "savequeue.h"

// Dependencies
#include <nlohmann/json.hpp>



struct RecordMsgHeader
{
    uint8_t record_stamp_type;
    double record_stamp;
    int64_t record_stamp_offset;
    uint64_t record_frame_id;

    RecordMsgHeader() : record_stamp_type(0), record_stamp(0), record_stamp_offset(0), record_frame_id(0) {}
};



class BaseRecordMsg
{
private:
    RecordMsgHeader header_;
    friend nlohmann::json BaseRecordMsgToJSON(const std::shared_ptr<BaseRecordMsg> rMsg);

protected:
    BaseRecordMsg(RecordMsgHeader header) : header_(header) {}
    virtual ~BaseRecordMsg() {}
};



template <typename rMsgT>
class RecordMsg : public BaseRecordMsg
{
private:
    std::shared_ptr<rMsgT> msg_;

    template <typename T>
    friend void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<T> > rMsg);

public:
    RecordMsg(RecordMsgHeader header, std::shared_ptr<rMsgT> msg) : BaseRecordMsg(header), msg_(msg) {}
};


/**
 * Self-defined record message type.
 */
struct VI_ImageMsg
{
    vehicle_interfaces::msg::Header header;
    uint8_t format_type;
    int16_t cvmat_type;
    uint8_t depth_unit_type;
    float depth_valid_min;
    float depth_valid_max;
    uint16_t width;
    uint16_t height;
    std::string file_name;
};



/**
 * make_record_msg function template for single message type.
 */
template <typename msgT>
std::shared_ptr<RecordMsg<msgT> > make_record_msg(const RecordMsgHeader header, const std::shared_ptr<msgT> msg)
{
    return std::make_shared<RecordMsg<msgT> >(header, msg);
}



/**
 * make_record_msg function template for ROS2 message type and self-defined record message type.
 */
template <typename msgT, typename rmsgT>
std::shared_ptr<RecordMsg<rmsgT> > make_record_msg_2(const RecordMsgHeader header, const std::shared_ptr<msgT> msg)
{
    std::cerr << "[make_record_msg] Function not implemented.\n";
    return nullptr;
}

template <typename msgT, typename rmsgT, typename smsgT>
std::shared_ptr<RecordMsg<rmsgT> > make_record_msg_3(const RecordMsgHeader header, 
    const std::shared_ptr<msgT> msg, 
    std::shared_ptr<SaveQueue<smsgT> > saveQueue, 
    fs::path outputDir)
{
    std::cerr << "[make_record_msg] Function not implemented.\n";
    return nullptr;
}



/**
 * Function template specialization.
 */
template <>
std::shared_ptr<RecordMsg<VI_ImageMsg> > make_record_msg_3(const RecordMsgHeader header, 
    const std::shared_ptr<vehicle_interfaces::msg::Image> msg, 
    std::shared_ptr<SaveQueue<cv::Mat> > saveQueue, 
    fs::path outputDir)
{
    VI_ImageMsg rMsg;
    rMsg.header = msg->header;
    rMsg.format_type = msg->format_type;
    rMsg.cvmat_type = msg->cvmat_type;
    rMsg.depth_unit_type = msg->depth_unit_type;
    rMsg.width = msg->width;
    rMsg.height = msg->height;

    double timestamp = msg->header.stamp.sec + (double)msg->header.stamp.nanosec / 1000000000.0;
    std::string filename = "";
    if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_JPEG)
        filename = outputDir / (std::to_string(timestamp) + ".jpg");
    else if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_RAW && msg->cvmat_type == CV_32FC1)
        filename = outputDir / (std::to_string(timestamp) + ".tiff");
    else
    {
        std::cerr << "[make_record_msg<VI_ImageMsg,,> (" << rMsg.header.device_id << ")] Unknown format type.\n";
        return nullptr;
    }
    rMsg.file_name = filename;

    try
    {
        cv::Mat tmp;
        if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_JPEG)
            tmp = cv::imdecode(msg->data, 1);
        else if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_RAW && msg->cvmat_type == CV_32FC1)
        {
            float *depths = reinterpret_cast<float *>(&msg->data[0]);
            tmp = cv::Mat(msg->height, msg->width, CV_32FC1, depths);
        }
        saveQueue->push(filename, tmp.clone());
        return std::make_shared<RecordMsg<VI_ImageMsg> >(header, std::make_shared<VI_ImageMsg>(rMsg));
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    catch(...)
    {
        std::cerr << "[make_record_msg<VI_ImageMsg,,> (" << rMsg.header.device_id << ")] Unknown exception.\n";
    }
    return nullptr;
}
