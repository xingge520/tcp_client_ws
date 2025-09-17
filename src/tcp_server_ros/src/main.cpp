#include "tcp_server_ros/TcpServerNode.h"
#include <thread>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

int main() {
    TcpServerNode text_server("0.0.0.0", 9999, 'T');
    TcpServerNode image_server("0.0.0.0", 8888, 'I');

    // 启动服务器线程
    std::thread t1(&TcpServerNode::startServer, &text_server);
    std::thread t2(&TcpServerNode::startServer, &image_server);

    // 命令输入线程
    std::thread cmd_thread([&text_server](){
        std::string input;
        while (true) {
            std::cout << "Enter target label to filter (or 'exit' to quit): ";
            std::getline(std::cin, input);
            if (input == "exit") break;

            nlohmann::json j;
            j["type"] = "SetTargetLabel";
            j["label"] = input;

            text_server.sendCommandToClient(j.dump());
        }
    });

    t1.join();
    t2.join();
    cmd_thread.join();

    return 0;
}
