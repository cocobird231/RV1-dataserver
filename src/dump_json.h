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

#include "vehicle_interfaces/msg/chassis.hpp"
#include "vehicle_interfaces/msg/distance.hpp"
#include "vehicle_interfaces/msg/environment.hpp"
#include "vehicle_interfaces/msg/gps.hpp"
#include "vehicle_interfaces/msg/ground_detect.hpp"
#include "vehicle_interfaces/msg/id_table.hpp"
#include "vehicle_interfaces/msg/image.hpp"
#include "vehicle_interfaces/msg/imu.hpp"
#include "vehicle_interfaces/msg/millit_brake_motor.hpp"
#include "vehicle_interfaces/msg/millit_power_motor.hpp"
#include "vehicle_interfaces/msg/motor_axle.hpp"
#include "vehicle_interfaces/msg/motor_steering.hpp"
#include "vehicle_interfaces/msg/qos_update.hpp"
#include "vehicle_interfaces/msg/steering_wheel.hpp"
#include "vehicle_interfaces/msg/ups.hpp"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#include "record_msg.h"

// Dependencies
#include <nlohmann/json.hpp>



/**
 * Convert vehicle_interfaces::msg::Header to JSON.
 * @param[out] json output JSON object.
 * @param[in] header vehicle_interfaces::msg::Header object.
 */
void VI_HeaderToJSON(nlohmann::json& json, const vehicle_interfaces::msg::Header& header)
{
    json["priority"] = header.priority;
    json["device_type"] = header.device_type;
    json["device_id"] = header.device_id;
    json["frame_id"] = header.frame_id;
    json["stamp_type"] = header.stamp_type;
    json["stamp"] = header.stamp.sec + (double)(header.stamp.nanosec / 1000000000.0);
    json["stamp_offset"] = header.stamp_offset;
    json["ref_publish_time_ms"] = header.ref_publish_time_ms;
}



/**
 * RecordMsgToJSON function template. Function called by BaseRecordMsgToJSON.
 * @param[out] json output JSON object.
 * @param[in] rMsg RecordMsg object.
 * @tparam rmsgT RecordMsg message type.
 * @note Function needs to be specialized for each message type.
 */
template<typename rmsgT>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<rmsgT> > rMsg)
{
    std::cerr << "[RecordMsgToJSON] Not implemented yet." << std::endl;
}



/**
 * RecordMsgToJSON function specialization for vehicle_interfaces messages.
 */

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::Chassis> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["unit_type"] = rMsg->msg_->unit_type;
    json["drive_motor"] = rMsg->msg_->drive_motor;
    json["steering_motor"] = rMsg->msg_->steering_motor;
    json["brake_motor"] = rMsg->msg_->brake_motor;
    json["parking_signal"] = rMsg->msg_->parking_signal;
    json["controller_frame_id"] = rMsg->msg_->controller_frame_id;
    json["controller_interrupt"] = rMsg->msg_->controller_interrupt;
    json["controller_name"] = rMsg->msg_->controller_name;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::Distance> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["unit_type"] = rMsg->msg_->unit_type;
    json["min"] = rMsg->msg_->min;
    json["max"] = rMsg->msg_->max;
    json["distance"] = rMsg->msg_->distance;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::Environment> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["unit_type"] = rMsg->msg_->unit_type;
    json["temperature"] = rMsg->msg_->temperature;
    json["relative_humidity"] = rMsg->msg_->relative_humidity;
    json["pressure"] = rMsg->msg_->pressure;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::GPS> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["unit_type"] = rMsg->msg_->unit_type;
    json["gps_status"] = rMsg->msg_->gps_status;
    json["latitude"] = rMsg->msg_->latitude;
    json["longitude"] = rMsg->msg_->longitude;
    json["altitude"] = rMsg->msg_->altitude;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::IDTable> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["idtable"] = rMsg->msg_->idtable;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::Image> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["format_type"] = rMsg->msg_->format_type;
    json["cvmat_type"] = rMsg->msg_->cvmat_type;
    json["depth_unit_type"] = rMsg->msg_->depth_unit_type;
    json["depth_valid_min"] = rMsg->msg_->depth_valid_min;
    json["depth_valid_max"] = rMsg->msg_->depth_valid_max;
    json["width"] = rMsg->msg_->width;
    json["height"] = rMsg->msg_->height;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::IMU> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["unit_type"] = rMsg->msg_->unit_type;
    json["orientation"] = rMsg->msg_->orientation;
    json["angular_velocity"] = rMsg->msg_->angular_velocity;
    json["linear_acceleration"] = rMsg->msg_->linear_acceleration;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::MillitBrakeMotor> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["travel_min"] = rMsg->msg_->travel_min;
    json["travel_max"] = rMsg->msg_->travel_max;
    json["travel"] = rMsg->msg_->travel;
    json["brake_percentage"] = rMsg->msg_->brake_percentage;
    json["external_control"] = rMsg->msg_->external_control;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::MillitPowerMotor> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["motor_mode"] = rMsg->msg_->motor_mode;
    json["rpm"] = rMsg->msg_->rpm;
    json["torque"] = rMsg->msg_->torque;
    json["percentage"] = rMsg->msg_->percentage;
    json["voltage"] = rMsg->msg_->voltage;
    json["current"] = rMsg->msg_->current;
    json["temperature"] = rMsg->msg_->temperature;
    json["parking"] = rMsg->msg_->parking;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::MotorAxle> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["dir"] = rMsg->msg_->dir;
    json["pwm"] = rMsg->msg_->pwm;
    json["parking"] = rMsg->msg_->parking;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::MotorSteering> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["unit_type"] = rMsg->msg_->unit_type;
    json["min"] = rMsg->msg_->min;
    json["max"] = rMsg->msg_->max;
    json["center"] = rMsg->msg_->center;
    json["value"] = rMsg->msg_->value;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::QosUpdate> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["qid"] = rMsg->msg_->qid;
    json["topic_table"] = rMsg->msg_->topic_table;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::SteeringWheel> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["gear"] = rMsg->msg_->gear;
    json["steering"] = rMsg->msg_->steering;
    json["pedal_throttle"] = rMsg->msg_->pedal_throttle;
    json["pedal_brake"] = rMsg->msg_->pedal_brake;
    json["pedal_clutch"] = rMsg->msg_->pedal_clutch;
    json["func_0"] = rMsg->msg_->func_0;
    json["func_1"] = rMsg->msg_->func_1;
    json["func_2"] = rMsg->msg_->func_2;
    json["func_3"] = rMsg->msg_->func_3;
    json["controller_frame_id"] = rMsg->msg_->controller_frame_id;
    json["controller_interrupt"] = rMsg->msg_->controller_interrupt;
    json["controller_name"] = rMsg->msg_->controller_name;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::UPS> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["volt_in"] = rMsg->msg_->volt_in;
    json["amp_in"] = rMsg->msg_->amp_in;
    json["volt_out"] = rMsg->msg_->volt_out;
    json["amp_out"] = rMsg->msg_->amp_out;
    json["temperature"] = rMsg->msg_->temperature;
}

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<vehicle_interfaces::msg::WheelState> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["gear"] = rMsg->msg_->gear;
    json["steering"] = rMsg->msg_->steering;
    json["pedal_throttle"] = rMsg->msg_->pedal_throttle;
    json["pedal_brake"] = rMsg->msg_->pedal_brake;
    json["pedal_clutch"] = rMsg->msg_->pedal_clutch;
    json["button"] = rMsg->msg_->button;
    json["func"] = rMsg->msg_->func;
}



/**
 * RecordMsgToJSON function specialization for self-defined messages.
 */

template<>
void RecordMsgToJSON(nlohmann::json& json, const std::shared_ptr<RecordMsg<VI_ImageMsg> > rMsg)
{
    VI_HeaderToJSON(json, rMsg->msg_->header);
    json["format_type"] = rMsg->msg_->format_type;
    json["cvmat_type"] = rMsg->msg_->cvmat_type;
    json["depth_unit_type"] = rMsg->msg_->depth_unit_type;
    json["depth_valid_min"] = rMsg->msg_->depth_valid_min;
    json["depth_valid_max"] = rMsg->msg_->depth_valid_max;
    json["width"] = rMsg->msg_->width;
    json["height"] = rMsg->msg_->height;
    json["file_name"] = rMsg->msg_->file_name;
}



/**
 * Convert BaseRecordMsg to JSON.
 * @param[in] rMsg BaseRecordMsg object.
 * @return JSON object.
 * @note Function will call `RecordMsgToJSON` function determined by the type of `RecordMsg`
 */
nlohmann::json BaseRecordMsgToJSON(const std::shared_ptr<BaseRecordMsg> rMsg)
{
    nlohmann::json json;
    json["record_stamp_type"] = rMsg->header_.record_stamp_type;
    json["record_stamp"] = rMsg->header_.record_stamp;
    json["record_stamp_offset"] = rMsg->header_.record_stamp_offset;
    json["record_frame_id"] = rMsg->header_.record_frame_id;

    if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::Chassis> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::Distance> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::Environment> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::GPS> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::IDTable> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::Image> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::IMU> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::MillitBrakeMotor> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::MillitPowerMotor> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::MotorAxle> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::MotorSteering> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::QosUpdate> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::SteeringWheel> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::UPS> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<vehicle_interfaces::msg::WheelState> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    else if (auto tmp = std::dynamic_pointer_cast<RecordMsg<VI_ImageMsg> >(rMsg))
    {
        RecordMsgToJSON(json, tmp);
    }
    return json;
}