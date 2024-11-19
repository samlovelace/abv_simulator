
#include "abv_simulator/UdpServer.h"
#include <iostream>
#include <cstring>

UdpServer::UdpServer(int port, Callback callback)
    : mPort(port), mCallback(callback), mRunning(false) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock.");
    }
#endif

    mSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (mSocket < 0) {
        throw std::runtime_error("Failed to create socket.");
    }

    struct sockaddr_in serverAddress;
    std::memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(mPort);

    if (bind(mSocket, reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress)) < 0) {
        throw std::runtime_error("Failed to bind socket.");
    }
}

UdpServer::~UdpServer() {
    stop();
#ifdef _WIN32
    closesocket(mSocket);
    WSACleanup();
#else
    close(mSocket);
#endif
}

void UdpServer::start() {
    if (mRunning.exchange(true)) {
        return; // Already running
    }

    mListenThread = std::thread(&UdpServer::listen, this);
}

void UdpServer::stop() {
    if (!mRunning.exchange(false)) {
        return; // Already stopped
    }

    if (mListenThread.joinable()) {
        mListenThread.join();
    }
}

void UdpServer::listen() {
    char buffer[1024];
    struct sockaddr_in clientAddress;
    socklen_t clientAddressLen = sizeof(clientAddress);

    while (mRunning) {
        int receivedBytes = recvfrom(mSocket, buffer, sizeof(buffer) - 1, 0,
                                     reinterpret_cast<struct sockaddr*>(&clientAddress), &clientAddressLen);
        if (receivedBytes > 0) {
            buffer[receivedBytes] = '\0'; // Null-terminate the received data

            std::string clientIP = inet_ntoa(clientAddress.sin_addr);
            int clientPort = ntohs(clientAddress.sin_port);
            std::string message(buffer);

            // Call the user-defined callback function
            if (mCallback) {
                mCallback(message, clientIP, clientPort);
            }
        } else if (receivedBytes < 0) {
            if (mRunning) {
                std::cerr << "Error receiving data." << std::endl;
            }
        }
    }
}
