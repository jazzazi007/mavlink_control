/*
 * File: test.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the implementation of a MAVLink heartbeat sender and receiver.
 *              It sends heartbeat messages to a specified IP address and UDP port, and listens
 *              for incoming MAVLink messages.
 */
#include "../include/gnc.h"

#define BUFFER_LENGTH 2041
#define DEFAULT_TARGET_SYSTEM 1     // Default to system ID 1 (typical for autopilot)
#define DEFAULT_TARGET_COMPONENT 1  // Default to component ID 1 (flight controller)
#define GCS_SYSTEM_ID 255          // Ground Control Station ID
#define GCS_COMPONENT_ID 0         // GCS component ID

static void heart_prep(mavlink_message_t *msg, uint8_t *buf, ssize_t *len)
{
    mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_GCS;
        heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
        heartbeat.base_mode = MAV_MODE_MANUAL_ARMED;
        heartbeat.custom_mode = 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_encode(1, 200, &msg, &heartbeat);
        len = mavlink_msg_to_send_buffer(buf, &msg);
}
static int open_scket()
{
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket");
        return -1;
    }
    fprintf(stderr, "Socket opened \n");
    return sock;
}

void send_rc_override(int sockfd, struct sockaddr_in* target_addr) {
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    mavlink_rc_channels_override_t rc_override;
    uint16_t len;

    // Set RC channel values
    rc_override.chan1_raw = 1500;    // Roll
    rc_override.chan2_raw = 1500;    // Pitch
    rc_override.chan3_raw = 1500;    // Throttle
    rc_override.chan4_raw = 1500;    // Yaw
    rc_override.chan5_raw = 1600;
    rc_override.chan6_raw = 2000;
    rc_override.chan7_raw = 1500;
    rc_override.chan8_raw = 1100;
    rc_override.target_system = DEFAULT_TARGET_SYSTEM;
    rc_override.target_component = DEFAULT_TARGET_COMPONENT;

    // Pack message
    mavlink_msg_rc_channels_override_encode(GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg, &rc_override);
    
    // Convert to buffer and send
    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (struct sockaddr*)target_addr, sizeof(struct sockaddr_in));
    
    printf("Sent RC_OVERRIDE message\n");
}

int send_mavlink_message(int sockfd, struct sockaddr_in* target_addr) {
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    // Pack heartbeat message
    mavlink_msg_heartbeat_pack(
        GCS_SYSTEM_ID,           // Source system
        GCS_COMPONENT_ID,        // Source component
        &msg,
        MAV_TYPE_GCS,           // Type = Ground Control Station
        MAV_AUTOPILOT_INVALID,  // Autopilot type
        MAV_MODE_MANUAL_ARMED,  // System mode
        0,                      // Custom mode
        MAV_STATE_ACTIVE       // System state
    );

    // Copy message to send buffer
    len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send message
    ssize_t bytes_sent = sendto(sockfd, buf, len, 0, 
                               (struct sockaddr*)target_addr, 
                               sizeof(struct sockaddr_in));
    
    if (bytes_sent == -1) {
        perror("Error sending message");
        return -1;
    }

    printf("Sent message with length: %d\n", len);
    return 0;
}

int main(int ac, char **av) {
    struct sockaddr_in autopilot_addr;
    uint8_t buf[2041];
    ssize_t len;
    mavlink_message_t msg;

    if (ac != 3) {
        fprintf(stderr, "Usage: %s <IP address> <UDP port>\n", av[0]);
        return 1;
    }
    int udp_port = atoi(av[2]);
    char *ip_addr = av[1];
    printf("IP address: %s, UDP port: %d\n", ip_addr, udp_port);
    // Create socket
    int sockfd = open_scket();
    // Set up the server address (SITL's IP address and UDP port)
    memset(&autopilot_addr, 0, sizeof(autopilot_addr));
    autopilot_addr.sin_family = AF_INET;
    autopilot_addr.sin_port = htons(udp_port); // Updated port
    autopilot_addr.sin_addr.s_addr = inet_addr(ip_addr); // Updated address
    if (bind(sockfd, (struct sockaddr*)&autopilot_addr, sizeof(autopilot_addr)) < 0) {
        perror("bind");
        close(sockfd);
        return -1;
    }
    printf("Socket bound\n");

    while (1) {
        // Prepare a MAVLink heartbeat message (example)
        heart_prep(&msg, buf, &len);

        // Send the message
        sendto(sockfd, buf, len, 0, (struct sockaddr*)&autopilot_addr, sizeof(autopilot_addr));

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
            socklen_t addr_len = sizeof(autopilot_addr);
            int recv_len = recvfrom(sockfd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr*)&autopilot_addr, &addr_len);
            if (recv_len > 0) {
                // Decode the MAVLink message
                mavlink_status_t status;
                for (int i = 0; i < recv_len; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, recv_buf[i], &msg, &status)) {
                        // Print out the received message details
                        printf("Received message with ID: %d\n", msg.msgid);
                        
                        // Process the specific message (e.g., HEARTBEAT)
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            mavlink_heartbeat_t heartbeat_msg;
                            mavlink_msg_heartbeat_decode(&msg, &heartbeat_msg);
                            printf("Received HEARTBEAT: Type: %d, Autopilot: %d\n", heartbeat_msg.type, heartbeat_msg.autopilot);
                        }
                        if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
                        {
                            mavlink_gps_raw_int_t gps_raw_int;
                            mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
                            printf("Received GPS_RAW_INT: Lat: %d, Lon: %d\n", gps_raw_int.lat, gps_raw_int.lon);
                            printf("Received GPS_RAW_INT: Alt: %d, VD: %d\n", gps_raw_int.alt, gps_raw_int.vel);
                        }
                    }
                }
            }
            /*mavlink_rc_channels_override_t rc_override;
            rc_override.chan1_raw = 1600;
            rc_override.chan2_raw = 1500;
            rc_override.chan3_raw = 1600;
            rc_override.chan4_raw = 1500;
            rc_override.chan5_raw = 0;
            rc_override.chan6_raw = 0;
            rc_override.chan7_raw = 0;
            rc_override.chan8_raw = 0;
            rc_override.target_system = 1;
            rc_override.target_component = 1;
            mavlink_msg_rc_channels_override_encode(1, 200, &msg, &rc_override);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sockfd, buf, len, 0, (struct sockaddr*)&autopilot_addr, sizeof(autopilot_addr));
            */
            // In main loop where you receive messages
            if (recvfrom(sockfd, buf, BUFFER_LENGTH, 0, 
                         (struct sockaddr *)&autopilot_addr, &addr_len) > 0) {
                // ... handle received message ...
                
                // Send response
                send_mavlink_message(sockfd, &autopilot_addr);
            }
        } else 
        {
            // No data within the timeout period
            printf("No data\n");
        }

        // Sleep for a while before sending the next heartbeat
        sleep(0.5);

        send_rc_override(sockfd, &autopilot_addr);
    }

    // Close the socket
    close(sockfd);
    return 0;
}
