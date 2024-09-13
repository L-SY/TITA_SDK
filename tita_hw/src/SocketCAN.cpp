#include "tita_hw/SocketCAN.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>  // for std::strncpy
#include <iostream>
#include <thread>
#include <string>
#include <cstdio>  // for std::perror

namespace tita_hw
{
SocketCAN::~SocketCAN()
{
    if (this->isOpen())
        this->close();
}

bool SocketCAN::open(const std::string& interface, boost::function<void(const canfd_frame& frame)> handler, int thread_priority)
{
    reception_handler = std::move(handler);
    // Request a socket
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ == -1)
    {
        std::cerr << "Error: Unable to create a CAN socket" << std::endl;
        return false;
    }
    char name[16] = {};  // avoid stringop-truncation
    std::strncpy(name, interface.c_str(), interface.size());
    std::strncpy(interface_request_.ifr_name, name, IFNAMSIZ);
    // Get the index of the network interface
    if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1)
    {
        std::cerr << "Unable to select CAN interface " << name << ": I/O control error" << std::endl;
        // Invalidate unusable socket
        close();
        return false;
    }
    // Bind the socket to the network interface
    address_.can_family = AF_CAN;
    address_.can_ifindex = interface_request_.ifr_ifindex;
    int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&address_), sizeof(address_));
    if (rc == -1)
    {
        std::cerr << "Failed to bind socket to " << name << " network interface" << std::endl;
        close();
        return false;
    }
    // Start a separate, event-driven thread for frame reception
    return startReceiverThread(thread_priority);
}

void SocketCAN::close()
{
    terminate_receiver_thread_ = true;
    while (receiver_thread_running_)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Wait for the receiver thread to terminate

    if (!isOpen())
        return;
    // Close the file descriptor for our socket
    ::close(sock_fd_);
    sock_fd_ = -1;
}

bool SocketCAN::isOpen() const
{
    return (sock_fd_ != -1);
}

void SocketCAN::write(can_frame* frame) const
{
    if (!isOpen())
    {
        std::cerr << "Unable to write: Socket not open" << std::endl;
        return;
    }
    if (::write(sock_fd_, frame, sizeof(can_frame)) == -1)
        std::cerr << "Unable to write: The tx buffer may be full" << std::endl;
}

void SocketCAN::write(canfd_frame* frame) const
{
    if (!isOpen())
    {
        std::cerr << "Unable to write: Socket not open" << std::endl;
        return;
    }
    if (::write(sock_fd_, frame, sizeof(canfd_frame)) == -1)
        std::cerr << "Unable to write: The tx buffer may be full" << std::endl;
}

static void* socketcan_receiver_thread(void* argv)
{
    /*
     * The first and only argument to this function
     * is the pointer to the object, which started the thread.
     */
    auto* sock = (SocketCAN*)argv;
    // Holds the set of descriptors, that 'select' shall monitor
    fd_set descriptors;
    // Highest file descriptor in set
    int maxfd = sock->sock_fd_;
    // How long 'select' shall wait before returning with timeout
    struct timeval timeout;
    // Buffer to store incoming frame
    canfd_frame rx_frame{};
    // Run until termination signal received
    sock->receiver_thread_running_ = true;
    while (!sock->terminate_receiver_thread_)
    {
        timeout.tv_sec = 1;  // Should be set each loop
        timeout.tv_usec = 0;
        // Clear descriptor set
        FD_ZERO(&descriptors);
        // Add socket descriptor
        FD_SET(sock->sock_fd_, &descriptors);
        // Wait until timeout or activity on any descriptor
        if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout) > 0)
        {
            ssize_t len = read(sock->sock_fd_, &rx_frame, CAN_MTU);
            if (len < 0)
                continue;
            if (sock->reception_handler != nullptr)
                sock->reception_handler(rx_frame);
        }
    }
    sock->receiver_thread_running_ = false;
    return nullptr;
}

bool SocketCAN::startReceiverThread(int thread_priority)
{
    // Frame reception is accomplished in a separate, event-driven thread.
    // See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
    terminate_receiver_thread_ = false;
    int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
    if (rc != 0)
    {
        std::cerr << "Unable to start receiver thread" << std::endl;
        return false;
    }
    std::cout << "Successfully started receiver thread with ID " << receiver_thread_id_ << std::endl;
    sched_param sched{ .sched_priority = thread_priority };
    pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &sched);
    return true;
}

}  // namespace tita_hw
