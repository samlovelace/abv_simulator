#ifndef UDPSERVER_H
#define UDPSERVER_H

#include <functional>
#include <string>
#include <thread>
#include <atomic>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <unistd.h>
#endif

class UdpServer {
public:
    using Callback = std::function<void(const std::string& message, const std::string& clientIP, int clientPort)>;

    UdpServer(int port, Callback callback);
    ~UdpServer();

    void start();
    void stop();

private:
    void listen();

    int mSocket;
    int mPort;
    Callback mCallback;
    std::atomic<bool> mRunning;
    std::thread mListenThread;
};

#endif // UDPSERVER_H
