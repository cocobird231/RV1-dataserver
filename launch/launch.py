#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_dataserver3'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="sub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "msg_filter" : data['topic_monitor']['msg_filter'], 
                    "scan_period_ms" : data['topic_monitor']['scan_period_ms'], 
                    "sample_period_ms" : data['msg_record']['sample_period_ms'], 
                    "dump_period_s" : data['msg_record']['dump_period_s'], 
                    "countdown_duration_s" : data['msg_record']['countdown_duration_s'], 
                    "img_threads" : data['msg_record']['img_threads'], 
                    "gnd_threads" : data['msg_record']['gnd_threads'], 
                    "dump_path" : data['msg_record']['dump_path'], 
                    "enable_control" : data['msg_record']['enable_control'], 

                    "serviceName" : data['service_prop']['serviceName'] + '_' + str(data['generic_prop']['id']), 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])