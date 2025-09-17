#ifndef TCP_SERVER_NODE_H
#define TCP_SERVER_NODE_H

#include <ros/ros.h>
#include <string>
#include <netinet/in.h>  // sockaddr_in
#include <cstddef>       // size_t
#include <unordered_map>
#include <functional>
#include <nlohmann/json.hpp>

class TcpServerNode {
public:
    TcpServerNode(const std::string& ip, int port, char type);
    ~TcpServerNode();

    void startServer();

    // ---------------- 新增 ----------------
    void sendCommandToClient(const std::string &cmd);

private:
    void handleConnection(int client_sock);
    void processJsonData(const std::string& msg);
    void processImageData(const char* buffer, size_t length);

    void registerHandlers();
    void handleTwist(const nlohmann::json& j);
    void handleOdometry(const nlohmann::json& j);
    void handleObject(const nlohmann::json& j);
    void handleBatteryState(const nlohmann::json& j);

private:
    std::string server_ip_;
    int server_port_;
    int server_fd_;
    int client_fd_;
    struct sockaddr_in server_addr_;
    struct sockaddr_in client_addr_;
    char server_type_; // 'T'=Text/JSON, 'I'=Image

    std::unordered_map<std::string, std::function<void(const nlohmann::json&)>> handlers_;
};

#endif
