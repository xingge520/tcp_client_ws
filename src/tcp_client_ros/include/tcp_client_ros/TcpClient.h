#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <functional>
#include <atomic>

class TcpClient {
public:
    TcpClient(const std::string& ip, int text_port, int image_port);
    ~TcpClient();

    // send
    void sendTextData(const std::string& text);
    void sendImageData(const cv::Mat& img);

    // receive command callback: invoked when a command string arrives from server
    void setCommandCallback(std::function<void(const std::string&)> cb);

private:
    std::string server_ip_;
    int text_port_, image_port_;
    int text_socket_, image_socket_;

    // 发送队列（线程安全）
    std::queue<std::string> text_queue_;
    std::queue<std::vector<uchar>> image_queue_;
    std::mutex text_mutex_, image_mutex_;
    std::condition_variable text_cv_, image_cv_;
    std::atomic<bool> running_;

    std::thread text_thread_, image_thread_, text_recv_thread_;

    // callback for incoming commands
    std::function<void(const std::string&)> command_callback_;
    std::mutex cmd_cb_mutex_;

    void connectToServer();
    bool connectToPort(int& socket_fd, int port);
    bool sendData(int sock, char type, const void* data, size_t length);

    void textWorker();
    void imageWorker();
    void textRecvWorker(); // receive incoming messages on text socket (e.g., commands)
};
