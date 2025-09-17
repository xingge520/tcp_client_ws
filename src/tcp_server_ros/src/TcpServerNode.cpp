#include "tcp_server_ros/TcpServerNode.h"
#include <iostream>
#include <thread>
#include <vector>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>

using json = nlohmann::json;

TcpServerNode::TcpServerNode(const std::string& ip, int port, char type)
    : server_ip_(ip), server_port_(port), server_fd_(-1), client_fd_(-1), server_type_(type) {

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(server_port_);
    server_addr_.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd_, 5) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on " << server_ip_ << ":" << server_port_ << std::endl;

    if (server_type_ == 'T') registerHandlers();
}

TcpServerNode::~TcpServerNode() {
    if (client_fd_ != -1) close(client_fd_);
    if (server_fd_ != -1) close(server_fd_);
}

void TcpServerNode::startServer() {
    socklen_t client_len = sizeof(client_addr_);
    client_fd_ = accept(server_fd_, (struct sockaddr*)&client_addr_, &client_len);
    if (client_fd_ < 0) {
        perror("accept");
        return;
    }
    std::cout << "Client connected on port " << server_port_ << std::endl;
    handleConnection(client_fd_);
}

void TcpServerNode::handleConnection(int client_fd) {
    std::cout << "Client connected, waiting for data..." << std::endl;

    char header[5];
    while (true) {
        ssize_t n = recv(client_fd, header, 5, MSG_WAITALL);
        if (n <= 0) break;

        char data_type = header[0];
        uint32_t length;
        memcpy(&length, header + 1, 4);
        length = ntohl(length);

        std::vector<char> buffer(length);
        n = recv(client_fd, buffer.data(), length, MSG_WAITALL);
        if (n <= 0) break;

        if (server_type_ == 'T' && data_type == 'T') {
            processJsonData(std::string(buffer.begin(), buffer.end()));
        } else if (server_type_ == 'I' && data_type == 'I') {
            processImageData(buffer.data(), length);
        }
    }

    std::cout << "Client disconnected." << std::endl;
}

void TcpServerNode::processJsonData(const std::string& msg_str) {
    try {
        auto j = json::parse(msg_str);
        std::string type = j.value("type", "Unknown");

        auto it = handlers_.find(type);
        if (it != handlers_.end()) {
            it->second(j);
        } else {
            std::cout << "[Unknown JSON type] " << msg_str << std::endl;
        }

    } catch (std::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
    }
}

void TcpServerNode::registerHandlers() {
    handlers_["Twist"] = [this](const json& j){ handleTwist(j); };
    handlers_["Odometry"] = [this](const json& j){ handleOdometry(j); };
    handlers_["Object"] = [this](const json& j){ handleObject(j); };
    handlers_["FilteredObject"] = [this](const json& j){ handleObject(j); };
    handlers_["RawObject"] = [this](const json& j){ handleObject(j); };
    handlers_["BatteryState"] = [this](const json& j){ handleBatteryState(j); };
}

void TcpServerNode::handleTwist(const json& j) {
    std::cout << "[Twist] linear=("
              << j["linear"]["x"] << "," 
              << j["linear"]["y"] << "," 
              << j["linear"]["z"] << "), angular=("
              << j["angular"]["x"] << "," 
              << j["angular"]["y"] << "," 
              << j["angular"]["z"] << ")"
              << std::endl;
}

void TcpServerNode::handleOdometry(const json& j) {
    auto pose = j["pose"];
    auto twist = j["twist"];
    std::cout << "[Odometry] seq=" << j["header"]["seq"]
              << " pos=("
              << pose["position"]["x"] << "," 
              << pose["position"]["y"] << "," 
              << pose["position"]["z"] << "), orient=("
              << pose["orientation"]["x"] << "," 
              << pose["orientation"]["y"] << "," 
              << pose["orientation"]["z"] << "," 
              << pose["orientation"]["w"] << "), twist_linear=("
              << twist["linear"]["x"] << "," 
              << twist["linear"]["y"] << "," 
              << twist["linear"]["z"] << "), twist_angular=("
              << twist["angular"]["x"] << "," 
              << twist["angular"]["y"] << "," 
              << twist["angular"]["z"] << ")"
              << std::endl;
}

void TcpServerNode::handleObject(const json& j) {
    auto pos = j["position"]["position"];
    auto orient = j["position"]["orientation"];
    auto size = j["size"];
    std::string type = j.value("type", "Object");

    std::cout << "[" << type << "] seq=" << j["header"]["seq"]
              << ", objseq=" << j["detect_sequence"]
              << ", total=" << j["object_total"]
              << ", label=" << j["label"]
              << ", prob=" << j["probability"]
              << ", pos=(" << pos["x"] << "," << pos["y"] << "," << pos["z"] << ")"
              << ", orient=(" << orient["x"] << "," << orient["y"] << "," << orient["z"] << "," << orient["w"] << ")"
              << ", size=(" << size["x"] << "," << size["y"] << "," << size["z"] << ")"
              << std::endl;
}



void TcpServerNode::handleBatteryState(const json& j) {
    std::cout << "[BatteryState] seq=" << j["header"]["seq"]
              << ", voltage=" << j["voltage"]
              << ", current=" << j["current"]
              << ", charge=" << j["charge"]
              << ", capacity=" << j["capacity"]
              << ", percentage=" << j["percentage"]
              << ", power_supply_status=" << j["power_supply_status"]
              << ", power_supply_health=" << j["power_supply_health"]
              << ", power_supply_technology=" << j["power_supply_technology"]
              << std::endl;
}

void TcpServerNode::processImageData(const char* buffer, size_t length) {
    std::vector<uchar> data(buffer, buffer + length);
    cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);
    if (!img.empty()) {
        cv::imshow("Received Image", img);
        cv::waitKey(1);
        std::cout << "[Port " << server_port_ << "] Received Image: " 
                  << img.cols << "x" << img.rows << std::endl;
    }
}

// ---------------- 新增实现 sendCommandToClient ----------------
void TcpServerNode::sendCommandToClient(const std::string &cmd) {
    if (client_fd_ < 0) {
        std::cerr << "[Warning] No client connected to send command." << std::endl;
        return;
    }

    uint32_t len = htonl(cmd.size());
    std::vector<char> packet;
    packet.push_back('C'); // command type
    packet.insert(packet.end(), reinterpret_cast<char*>(&len), reinterpret_cast<char*>(&len)+4);
    packet.insert(packet.end(), cmd.begin(), cmd.end());

    size_t sent = 0;
    while (sent < packet.size()) {
        ssize_t n = write(client_fd_, packet.data() + sent, packet.size() - sent);
        if (n <= 0) {
            perror("write command");
            break;
        }
        sent += n;
    }
    std::cout << "[Command Sent] " << cmd << std::endl;
}
