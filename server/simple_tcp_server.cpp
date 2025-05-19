// simple_tcp_server.cpp
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

int main() {
    int server_fd, client_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // 1. Create socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return 1;
    }

    // 2. Set socket options
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        return 1;
    }

    // 3. Bind to port 12345
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
    address.sin_port = htons(12345);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return 1;
    }

    // 4. Listen
    if (listen(server_fd, 1) < 0) {
        perror("listen");
        return 1;
    }

    std::cout << "Server listening on port 12345..." << std::endl;

    // 5. Accept a connection
    if ((client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        perror("accept");
        return 1;
    }

    std::cout << "Client connected!" << std::endl;

    // 6. Keep the connection open for testing
    while (true) {
        sleep(1);
    }

    // 7. Cleanup (never reached in this example)
    close(client_fd);
    close(server_fd);
    return 0;
}
