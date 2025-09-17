#include <ros/ros.h>
#include "tcp_client_ros/DataSender.h"
#include <object_information_msgs/Object.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

class ObjectFilterNode {
public:
    ObjectFilterNode(ros::NodeHandle& nh, DataSender& sender)
        : nh_(nh), sender_(sender), target_label_("chair"), 
          image_processing_enabled_(true) 
    {
        // 订阅图像 - 增加队列大小
        image_sub_ = nh_.subscribe("/usb_cam/image_raw", 3, &ObjectFilterNode::imageCallback, this);

        // 订阅 objects
        obj_sub_ = nh_.subscribe("/objects", 10, &ObjectFilterNode::objectCallback, this);

        // 订阅动态更新的目标标签
        label_sub_ = nh_.subscribe("/target_labels_cmd", 10, &ObjectFilterNode::targetLabelCallback, this);

        // 发布过滤后的对象和图像
        obj_pub_ = nh_.advertise<object_information_msgs::Object>("/filtered_objects", 10);
        img_pub_ = nh_.advertise<sensor_msgs::Image>("/filtered_image", 1);

        // 启动图像处理线程
        image_thread_ = std::thread(&ObjectFilterNode::imageProcessingWorker, this);

        ROS_INFO("ObjectFilterNode initialized. Default target: %s", target_label_.c_str());
    }

    ~ObjectFilterNode() {
        image_processing_enabled_ = false;
        image_cv_.notify_all();
        if (image_thread_.joinable()) {
            image_thread_.join();
        }
    }

private:
    ros::NodeHandle nh_;
    DataSender& sender_;
    ros::Subscriber image_sub_, obj_sub_, label_sub_;
    ros::Publisher obj_pub_, img_pub_;
    cv::Mat latest_image_;
    std::string target_label_;
    
    // 图像处理队列
    struct ImageTask {
        object_information_msgs::Object::ConstPtr object_msg;
        cv::Mat image;
        ros::Time timestamp;
    };
    
    std::queue<ImageTask> image_queue_;
    std::mutex image_mutex_;
    std::condition_variable image_cv_;
    std::thread image_thread_;
    std::atomic<bool> image_processing_enabled_;
    
    // 智能队列管理
    static constexpr size_t MAX_QUEUE_SIZE = 3; // 适中的队列大小
    static constexpr double MAX_QUEUE_AGE_SEC = 2.0; // 最大队列年龄

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try { 
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone(); 
        }
        catch (cv_bridge::Exception& e) { 
            ROS_ERROR("cv_bridge exception: %s", e.what()); 
        }
    }

    void objectCallback(const object_information_msgs::Object::ConstPtr& msg) {
        // 1. 先发送原始 object -> RawObject
        sender_.sendObjData(msg, false); // false -> raw

        // 2. 如果符合过滤条件 -> FilteredObject
        if (msg->label == target_label_ && msg->probability >= 0.5) {
            sender_.sendObjData(msg, true);  // true -> filtered

            // 发布 filtered object
            obj_pub_.publish(*msg);

            // 异步处理图像，避免阻塞
            queueImageTask(msg);
        }
    }

    void queueImageTask(const object_information_msgs::Object::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        
        if (latest_image_.empty()) return;
        
        // 智能队列管理：清理过期任务
        cleanExpiredTasks();
        
        // 如果队列已满，移除最旧的任务
        if (image_queue_.size() >= MAX_QUEUE_SIZE) {
            image_queue_.pop();
            ROS_WARN("Image queue full, dropping oldest frame");
        }
        
        // 添加新任务
        ImageTask task;
        task.object_msg = msg;
        task.image = latest_image_.clone();
        task.timestamp = ros::Time::now();
        
        image_queue_.push(task);
        image_cv_.notify_one();
    }

    void cleanExpiredTasks() {
        ros::Time now = ros::Time::now();
        while (!image_queue_.empty()) {
            const auto& front_task = image_queue_.front();
            if ((now - front_task.timestamp).toSec() > MAX_QUEUE_AGE_SEC) {
                image_queue_.pop();
                ROS_DEBUG("Dropped expired image task");
            } else {
                break;
            }
        }
    }

    void imageProcessingWorker() {
        while (image_processing_enabled_) {
            std::unique_lock<std::mutex> lock(image_mutex_);
            image_cv_.wait(lock, [this] { 
                return !image_queue_.empty() || !image_processing_enabled_; 
            });
            
            if (!image_processing_enabled_) break;
            
            ImageTask task = std::move(image_queue_.front());
            image_queue_.pop();
            lock.unlock();
            
            // 处理图像
            processImageTask(task);
        }
    }

    void processImageTask(const ImageTask& task) {
        try {
            cv::Mat img = task.image;
            const auto& msg = task.object_msg;
            
            // 快速图像处理
            int x = static_cast<int>(msg->position.position.x);
            int y = static_cast<int>(msg->position.position.y);
            int w = static_cast<int>(msg->size.x);
            int h = static_cast<int>(msg->size.y);

            // 绘制边界框
            cv::rectangle(img, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 2);
            
            // 绘制标签
            std::string label_text = msg->label + " " + std::to_string(static_cast<int>(msg->probability * 100)) + "%";
            cv::putText(img, label_text, cv::Point(x, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 1);

            // 发布到ROS话题
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            img_pub_.publish(img_msg);

            // 发送到TCP服务器
            sender_.sendImageData(img_msg);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Image processing error: %s", e.what());
        }
    }

    void targetLabelCallback(const std_msgs::String::ConstPtr& msg) {
        target_label_ = msg->data;
        ROS_INFO("Updated target label: %s", target_label_.c_str());
    }
};




// ----------------- 主函数 -----------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_client_node");
    ros::NodeHandle nh("~");

    std::string server_ip;
    int text_port, image_port, vel_port;

    nh.param<std::string>("server_ip", server_ip, "127.0.0.1");
    nh.param<int>("text_port", text_port, 9999);
    nh.param<int>("image_port", image_port, 8888);
    // nh.param<int>("vel_port", vel_port, 9002);

    DataSender sender(server_ip, text_port, image_port, vel_port);

    // 订阅 /cmd_vel
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10,
        boost::bind(&DataSender::sendVelData, &sender, _1));

    // 订阅 /odom
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/odom", 10,
        boost::bind(&DataSender::sendOdomData, &sender, _1));

    // 启动目标过滤模块
    ObjectFilterNode filter_node(nh, sender);

    ROS_INFO("TCP Client Node started. Connected to server: %s", server_ip.c_str());
    ros::spin();
    return 0;
}
