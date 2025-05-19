#include <iostream>
#include <cstring>
#include <netinet/in.h>
#include <unistd.h>
#include <limits>

constexpr int PORT = 5555;
constexpr uint8_t BBOX_TYPE = 13; // DataPacketType::BboxPos
constexpr size_t PACKET_SIZE = 65;

int main() {
    int server_fd, client_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Create socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 1) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    std::cout << "Waiting for connection on port " << PORT << "..." << std::endl;
    if ((client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    std::cout << "Client connected! Sending NaN bbox packets..." << std::endl;

    // Prepare packet
    uint8_t packet[PACKET_SIZE] = {0};
    packet[0] = BBOX_TYPE;
    double nan = std::numeric_limits<double>::quiet_NaN();
    // Pack 4 NaNs into the first 4 doubles of the data section
    for (int i = 0; i < 4; ++i) {
        std::memcpy(packet + 1 + i * 8, &nan, sizeof(double));
    }
    // The rest of the data section (8 doubles total) is left as zero

    // Send the packet every second
    while (true) {
        ssize_t sent = send(client_fd, packet, PACKET_SIZE, 0);
        if (sent != PACKET_SIZE) {
            std::cerr << "Failed to send packet!" << std::endl;
            break;
        }
        std::cout << "Sent NaN bbox packet." << std::endl;
        sleep(1);
    }
    close(client_fd);
    close(server_fd);
    return 0;
} 