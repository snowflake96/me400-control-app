/*
Server class for receiving data on Raspberry Pi over TCP/IP
Created by You, Jisang 2025/3/13
*/

#pragma once
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <thread>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "data.hpp"

class Server{
    using HandlerType = std::function<void(DataPacket)>;

private:
    int server_fd_;
    int port_;
    std::vector<std::thread> client_threads_;
    std::atomic<bool> running_;

public:
    Server(int port = 12345) : server_fd_(-1), port_(port), running_(true) {
        RCLCPP_INFO(rclcpp::get_logger("server"), "Starting up server...");
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
        server_addr.sin_port = htons(port);  

        bind(server_fd_, (sockaddr*)&server_addr, sizeof(server_addr));
        listen(server_fd_, 3);

        RCLCPP_INFO(rclcpp::get_logger("server"), "Server running on port %d", port);
    }

    ~Server(){
        stop(); // This should have been called already but call again just to be safe
        // Join all client threads
        for (auto& t : client_threads_) {
            if (t.joinable()) t.join();
        }
        if (server_fd_ != -1) {
            RCLCPP_INFO(rclcpp::get_logger("server"), "Shutting down server...");
            close(server_fd_);
        }
    }

    void stop(){
        running_ = false;
    }

    void launchServer(HandlerType handler) {
        while(running_){
            RCLCPP_INFO(rclcpp::get_logger("server"), "Waiting for client connection...");
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (sockaddr*)&client_addr, &client_len);
            RCLCPP_INFO(rclcpp::get_logger("server"), "Client connected from IP: %s", inet_ntoa(client_addr.sin_addr));

            std::thread client_thread([this, client_fd, handler]() {
                handleClient(client_fd, handler);
            });
            client_threads_.push_back(std::move(client_thread));
        }
    }

    void handleClient(int client_fd, HandlerType handler) {
        while (running_) {
            DataPacket packet;
            ssize_t bytes_read = recv(client_fd, &packet, sizeof(DataPacket), 0);
            if (bytes_read <= 0) {
                RCLCPP_INFO(rclcpp::get_logger("server"), "Client disconnected");
                close(client_fd);
                break;
            }
            handler(packet);
        }
    }

private:
    bool sendData(const DataPacket& packet, int client_fd){ 
        if (client_fd < 0) return false;
        int bytes_sent = send(client_fd, &packet, sizeof(DataPacket), 0);
        return bytes_sent > 0;
    }
};
