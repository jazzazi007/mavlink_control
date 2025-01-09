#include "c_library_v2/common/mavlink.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    int sockfd;
    struct sockaddr_in server_addr;
    uint8_t buf[2041];
    ssize_t len;
    mavlink_message_t msg;

    // Create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        perror("Error creating socket");
        return -1;
    }

    // Set up the server address (SITL's IP address and UDP port)
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(14550); // SITL listening port
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Local address

    while (1) {
        // Prepare a MAVLink heartbeat message (example)
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_GCS;
        heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
        heartbeat.base_mode = MAV_MODE_MANUAL_ARMED;
        heartbeat.custom_mode = 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;

        mavlink_msg_heartbeat_encode(1, 200, &msg, &heartbeat);
        len = mavlink_msg_to_send_buffer(buf, &msg);

        // Send the message
        sendto(sockfd, buf, len, 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

        // Check for incoming data
        struct timeval tv;
        fd_set readfds;

        tv.tv_sec = 1; // 1 second timeout
        tv.tv_usec = 0;

        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);

        int retval = select(sockfd + 1, &readfds, NULL, NULL, &tv);

        if (retval == -1) {
            perror("select()");
            close(sockfd);
            return 1;
        } else if (retval) {
            // Data is available to read
            char recv_buf[1024];
            socklen_t addr_len = sizeof(server_addr);
            int recv_len = recvfrom(sockfd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr*)&server_addr, &addr_len);
            if (recv_len > 0) {
                printf("Received data: %s\n", recv_buf);
            }
        } else {
            // No data within the timeout period
            printf("No data\n");
        }

        // Sleep for a while before sending the next heartbeat
        sleep(1);
    }

    // Close the socket
    close(sockfd);
    return 0;
}
