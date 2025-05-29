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
#include <mutex>
#include <algorithm>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include "data.hpp"

class Server{
    using HandlerType = std::function<void(const DataPacket&, int)>;
    using Callback = std::function<void()>;

private:
    int server_fd_;
    int port_;
    std::atomic<bool> running_;
    std::vector<int> client_fds_;
    std::mutex mtx_;
    Callback fatal_callback;

public:
    Server(int port) : server_fd_(-1), port_(port), running_(true) {
      // 1) Create socket
      server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
      if (server_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("server"), "socket() failed: %s", strerror(errno));
        throw std::runtime_error("socket() failed");
      }
    
      // 2) Allow immediate reuse of address & port
      int opt = 1;
      if (::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0 ||
          ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0)
      {
        RCLCPP_WARN(rclcpp::get_logger("server"),
                    "setsockopt(SO_REUSEADDR|SO_REUSEPORT) failed: %s", strerror(errno));
        // not fatal, so we continue
      }
    
      // 3) Bind to all interfaces on the given port
      sockaddr_in server_addr{};
      server_addr.sin_family      = AF_INET;
      server_addr.sin_addr.s_addr = INADDR_ANY;
      server_addr.sin_port        = htons(port_);
      if (::bind(server_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("server"), 
                     "bind(%d) failed: %s", port_, strerror(errno));
        ::close(server_fd_);
        throw std::runtime_error("bind() failed");
      }
    
      // 4) Start listening
      if (::listen(server_fd_, SOMAXCONN) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("server"), 
                     "listen() failed: %s", strerror(errno));
        ::close(server_fd_);
        throw std::runtime_error("listen() failed");
      }
    
      RCLCPP_INFO(rclcpp::get_logger("server"), "Server listening on port %d", port_);
    }
    
    ~Server(){
        stop(); // Stop accepting new connections
        // Close all client connections
        std::lock_guard lock(mtx_);
        for(int client_fd : client_fds_) {
            close(client_fd);
        }
        client_fds_.clear();
        
        if (server_fd_ != -1) {
            RCLCPP_INFO(rclcpp::get_logger("server"), "Shutting down server...");
            close(server_fd_);
        }
    }

    template<typename F>
    void setFatalCallback(F&& f){
        fatal_callback = std::forward<F>(f); // perfect forward into std::function
    }

    void stop(){
        running_ = false;
    }

    void launchServer(HandlerType handler) {
        while (running_) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) continue;  // error check omitted for brevity

            mtx_.lock();
            client_fds_.push_back(client_fd);
            mtx_.unlock();

            std::string msg = std::format("Server accepted connection from {}:{}", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            RCLCPP_INFO(rclcpp::get_logger("Server"), "%s", msg.c_str());
            sendLogToClient(msg, client_fd);

            // spawn and detach immediately
            std::thread(&Server::handleClient, this, client_fd, handler).detach();
        }
    }

    void handleClient(int client_fd, HandlerType handler) {
        while (running_) {
            DataPacket packet;
            ssize_t bytes_read = recv(client_fd, &packet, sizeof(DataPacket), 0);
            if (bytes_read <= 0) {
                RCLCPP_INFO(rclcpp::get_logger("server"), "Client disconnected");
                close(client_fd);

                mtx_.lock();
                std::erase(client_fds_, client_fd);
                // If all clients disconnected and fatal_callback is set, then call it
                if(client_fds_.empty() && fatal_callback){
                    fatal_callback();
                }
                mtx_.unlock();

                break;
            }
            handler(packet, client_fd); // Call server_handler from master node
        }
    }

    void sendDataToClient(const DataPacket& packet, int client_fd = -1){ 
        if(client_fd == -1){
            // Broadcast to all clients
            std::lock_guard lock(mtx_);
            for(int fd : client_fds_){
                send(fd, &packet, sizeof(DataPacket), 0);
            }
        }
        else{
            send(client_fd, &packet, sizeof(DataPacket), 0);
        }
    }

    void sendLogToClient(std::string_view sv, int client_fd = -1) {
        DataPacket packet{};  
        packet.type = DataPacket::Type::Log;
        size_t len = std::min(sv.size(), sizeof(packet.data.text)-1);
        std::memcpy(packet.data.text, sv.data(), len);
        packet.data.text[len] = '\0';

        sendDataToClient(packet, client_fd);
    }
};