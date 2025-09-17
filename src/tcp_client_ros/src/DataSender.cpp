#include "tcp_client_ros/DataSender.h"

using json = nlohmann::json;

DataSender::DataSender(const std::string& ip, int text_port, int image_port, int vel_port)
    : client_(ip, text_port, image_port) {
    (void)vel_port; // 占位
}

void DataSender::setCommandCallback(std::function<void(const std::string&)> cb) {
    client_.setCommandCallback(cb);
}

void DataSender::sendVelData(const geometry_msgs::Twist::ConstPtr& msg) {
    json j;
    j["type"] = "Twist";
    j["linear"]["x"] = msg->linear.x;
    j["linear"]["y"] = msg->linear.y;
    j["linear"]["z"] = msg->linear.z;
    j["angular"]["x"] = msg->angular.x;
    j["angular"]["y"] = msg->angular.y;
    j["angular"]["z"] = msg->angular.z;

    client_.sendTextData(j.dump());
}

void DataSender::sendOdomData(const nav_msgs::Odometry::ConstPtr& msg) {
    json j;
    j["type"] = "Odometry";

    j["header"]["seq"] = msg->header.seq;
    j["header"]["stamp"]["secs"] = msg->header.stamp.sec;
    j["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec;
    j["header"]["frame_id"] = msg->header.frame_id;

    j["pose"]["position"]["x"] = msg->pose.pose.position.x;
    j["pose"]["position"]["y"] = msg->pose.pose.position.y;
    j["pose"]["position"]["z"] = msg->pose.pose.position.z;

    j["pose"]["orientation"]["x"] = msg->pose.pose.orientation.x;
    j["pose"]["orientation"]["y"] = msg->pose.pose.orientation.y;
    j["pose"]["orientation"]["z"] = msg->pose.pose.orientation.z;
    j["pose"]["orientation"]["w"] = msg->pose.pose.orientation.w;

    j["twist"]["linear"]["x"] = msg->twist.twist.linear.x;
    j["twist"]["linear"]["y"] = msg->twist.twist.linear.y;
    j["twist"]["linear"]["z"] = msg->twist.twist.linear.z;
    j["twist"]["angular"]["x"] = msg->twist.twist.angular.x;
    j["twist"]["angular"]["y"] = msg->twist.twist.angular.y;
    j["twist"]["angular"]["z"] = msg->twist.twist.angular.z;

    client_.sendTextData(j.dump());
}

void DataSender::sendObjData(const object_information_msgs::Object::ConstPtr& msg) {
    sendObjData(msg, true); // 默认当做 filtered 发送
}


void DataSender::sendObjData(const object_information_msgs::Object::ConstPtr& msg, bool filtered) {
    json j;
    j["type"] = filtered ? "FilteredObject" : "RawObject"; // 区分 raw / filtered

    j["header"]["seq"] = msg->header.seq;
    j["header"]["stamp"]["secs"] = msg->header.stamp.sec;
    j["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec;
    j["header"]["frame_id"] = msg->header.frame_id;

    j["detect_sequence"] = msg->detect_sequence;
    j["object_total"] = msg->object_total;
    j["object_sequence"] = msg->object_sequence;
    j["label"] = msg->label;
    j["probability"] = msg->probability;

    j["position"]["position"]["x"] = msg->position.position.x;
    j["position"]["position"]["y"] = msg->position.position.y;
    j["position"]["position"]["z"] = msg->position.position.z;
    j["position"]["orientation"]["x"] = msg->position.orientation.x;
    j["position"]["orientation"]["y"] = msg->position.orientation.y;
    j["position"]["orientation"]["z"] = msg->position.orientation.z;
    j["position"]["orientation"]["w"] = msg->position.orientation.w;

    j["size"]["x"] = msg->size.x;
    j["size"]["y"] = msg->size.y;
    j["size"]["z"] = msg->size.z;

    client_.sendTextData(j.dump());
}


void DataSender::sendBatteryData(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    json j;
    j["type"] = "BatteryState";

    // Header
    j["header"]["seq"] = msg->header.seq;
    j["header"]["stamp"]["secs"] = msg->header.stamp.sec;
    j["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec;
    j["header"]["frame_id"] = msg->header.frame_id;

    // 电池基本信息
    j["voltage"] = msg->voltage;
    j["current"] = msg->current;
    j["charge"] = msg->charge;
    j["capacity"] = msg->capacity;
    j["design_capacity"] = msg->design_capacity;
    j["percentage"] = msg->percentage;

    // 电源状态
    j["power_supply_status"] = msg->power_supply_status;
    j["power_supply_health"] = msg->power_supply_health;
    j["power_supply_technology"] = msg->power_supply_technology;
    j["present"] = msg->present;

    // 电池单体电压列表
    j["cell_voltage"] = json::array();
    for (const auto& v : msg->cell_voltage) {
        j["cell_voltage"].push_back(v);
    }

    // 额外信息
    j["location"] = msg->location;
    j["serial_number"] = msg->serial_number;

    // 发送
    client_.sendTextData(j.dump());
}

void DataSender::sendImageData(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        client_.sendImageData(img); // 二进制直接发送
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
