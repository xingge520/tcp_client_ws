#include "tcp_client_ros/TcpClient.h"
#include <arpa/inet.h> // htons, htonl, inet_addr
#include <unistd.h>    // close, write, read
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <cstring>

TcpClient::TcpClient(const std::string& ip, int text_port, int image_port)
    : server_ip_(ip), text_port_(text_port), image_port_(image_port),
      text_socket_(-1), image_socket_(-1), running_(true) {
    connectToServer();

    text_thread_ = std::thread(&TcpClient::textWorker, this);
    image_thread_ = std::thread(&TcpClient::imageWorker, this);
    text_recv_thread_ = std::thread(&TcpClient::textRecvWorker, this);
}

TcpClient::~TcpClient() {
    running_ = false;
    text_cv_.notify_all();
    image_cv_.notify_all();

    if (text_thread_.joinable()) text_thread_.join();
    if (image_thread_.joinable()) image_thread_.join();
    if (text_recv_thread_.joinable()) text_recv_thread_.join();

    if (text_socket_ != -1) close(text_socket_);
    if (image_socket_ != -1) close(image_socket_);
}

void TcpClient::setCommandCallback(std::function<void(const std::string&)> cb) {
    std::lock_guard<std::mutex> lk(cmd_cb_mutex_);
    command_callback_ = cb;
}

void TcpClient::connectToServer() {
    // connect text socket
    if (!connectToPort(text_socket_, text_port_)) {
        std::cerr << "Warning: failed to connect text port " << text_port_ << std::endl;
    } else {
        std::cout << "Connected text socket to " << server_ip_ << ":" << text_port_ << std::endl;
    }

    // connect image socket
    if (!connectToPort(image_socket_, image_port_)) {
        std::cerr << "Warning: failed to connect image port " << image_port_ << std::endl;
    } else {
        std::cout << "Connected image socket to " << server_ip_ << ":" << image_port_ << std::endl;
    }
}

bool TcpClient::connectToPort(int& socket_fd, int port) {
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        perror("socket");
        return false;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    if (connect(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("connect");
        close(socket_fd);
        socket_fd = -1;
        return false;
    }
    return true;
}

bool TcpClient::sendData(int sock, char type, const void* data, size_t length) {
    if (sock < 0) return false;

    // header: 1 byte type + 4 bytes length (network order)
    uint32_t net_len = htonl(static_cast<uint32_t>(length));
    std::vector<char> packet;
    packet.reserve(1 + 4 + length);
    packet.push_back(type);
    packet.insert(packet.end(), reinterpret_cast<char*>(&net_len), reinterpret_cast<char*>(&net_len) + 4);
    packet.insert(packet.end(), (const char*)data, (const char*)data + length);

    size_t sent = 0;
    while (sent < packet.size()) {
        ssize_t n = write(sock, packet.data() + sent, packet.size() - sent);
        if (n <= 0) {
            perror("write");
            return false;
        }
        sent += n;
    }
    return true;
}

void TcpClient::sendTextData(const std::string& text) {
    {
        std::lock_guard<std::mutex> lk(text_mutex_);
        text_queue_.push(text);
    }
    text_cv_.notify_one();
}

void TcpClient::sendImageData(const cv::Mat& img) {
    std::vector<uchar> buf;
    if (!cv::imencode(".jpg", img, buf)) return;
    {
        std::lock_guard<std::mutex> lk(image_mutex_);
        image_queue_.push(std::move(buf));
    }
    image_cv_.notify_one();
}

void TcpClient::textWorker() {
    while (running_) {
        std::unique_lock<std::mutex> lk(text_mutex_);
        text_cv_.wait(lk, [&] { return !text_queue_.empty() || !running_; });
        if (!running_) break;
        std::string text = std::move(text_queue_.front());
        text_queue_.pop();
        lk.unlock();

        if (text_socket_ < 0) {
            // try reconnect
            if (!connectToPort(text_socket_, text_port_)) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }
        bool ok = sendData(text_socket_, 'T', text.data(), text.size());
        if (!ok) {
            close(text_socket_);
            text_socket_ = -1;
        }
    }
}

void TcpClient::imageWorker() {
    const size_t max_queue_size = 3; // 适中的队列大小
    const auto max_frame_age = std::chrono::milliseconds(1000); // 1秒最大年龄
    
    while (running_) {
        std::unique_lock<std::mutex> lk(image_mutex_);
        image_cv_.wait(lk, [&] { return !image_queue_.empty() || !running_; });
        if (!running_) break;

        // 智能队列管理：保留最新帧，但给一定缓冲
        while (image_queue_.size() > max_queue_size) {
            image_queue_.pop();
        }

        auto buf = std::move(image_queue_.front());
        image_queue_.pop();
        lk.unlock();

        // 尝试连接
        if (image_socket_ < 0) {
            if (!connectToPort(image_socket_, image_port_)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        // 发送数据
        bool ok = sendData(image_socket_, 'I', buf.data(), buf.size());
        if (!ok) {
            close(image_socket_);
            image_socket_ = -1;
        }

        // 轻微延迟，避免CPU占用过高，但不会造成明显延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void TcpClient::textRecvWorker() {
    // This thread listens for incoming messages on text_socket_ (e.g., commands).
    // Protocol: [1 byte type]['C' for command][4 bytes length][payload]
    while (running_) {
        if (text_socket_ < 0) {
            // try reconnect periodically
            if (!connectToPort(text_socket_, text_port_)) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }

        // read header (5 bytes)
        char header[5];
        ssize_t n = recv(text_socket_, header, 5, MSG_WAITALL);
        if (n == 0) {
            // remote closed
            close(text_socket_);
            text_socket_ = -1;
            continue;
        } else if (n < 0) {
            perror("recv header");
            close(text_socket_);
            text_socket_ = -1;
            continue;
        } else if (n < 5) {
            // partial, treat as error, reconnect
            close(text_socket_);
            text_socket_ = -1;
            continue;
        }

        char data_type = header[0];
        uint32_t len_net = 0;
        memcpy(&len_net, header + 1, 4);
        uint32_t length = ntohl(len_net);
        if (length == 0 || length > 10*1024*1024) {
            // suspicious length, skip
            std::cerr << "Invalid incoming length: " << length << std::endl;
            // drain or reconnect
            close(text_socket_);
            text_socket_ = -1;
            continue;
        }

        std::vector<char> buffer(length);
        n = recv(text_socket_, buffer.data(), length, MSG_WAITALL);
        if (n <= 0) {
            close(text_socket_);
            text_socket_ = -1;
            continue;
        }

        if (data_type == 'C') {
            std::string cmd(buffer.begin(), buffer.end());
            std::lock_guard<std::mutex> lk(cmd_cb_mutex_);
            if (command_callback_) {
                try { command_callback_(cmd); }
                catch (...) { /* swallow */ }
            }
        } else {
            // unknown incoming type on text channel
            std::cout << "Received non-command on text channel, type=" << data_type << std::endl;
        }
    }
}
