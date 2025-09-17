#pragma once
#include "tcp_client_ros/TcpClient.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <object_information_msgs/Object.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>
#include <functional>

class DataSender {
public:
    DataSender(const std::string& ip, int text_port, int image_port, int vel_port = 0);

    void sendVelData(const geometry_msgs::Twist::ConstPtr& msg);
    void sendOdomData(const nav_msgs::Odometry::ConstPtr& msg);

    void sendObjData(const object_information_msgs::Object::ConstPtr& msg);             // 保留原来的
    void sendObjData(const object_information_msgs::Object::ConstPtr& msg, bool filtered); // 新增
    
    void sendBatteryData(const sensor_msgs::BatteryState::ConstPtr& msg); //在此新增话题

    void sendImageData(const sensor_msgs::ImageConstPtr& msg);

    // set callback to receive commands from server
    void setCommandCallback(std::function<void(const std::string&)> cb);

private:
    TcpClient client_;
};


