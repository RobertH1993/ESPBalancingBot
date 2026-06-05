#ifndef _TNC_H
#define _TNC_H
#include <stdint.h>

#define UDP_MAX_SUBSCRIBERS 3
#define UDP_PACKET_QUEUE_SIZE 8
#define UDP_MAX_PACKET_SIZE 0XFF

typedef struct{
    // UDP control and telemetry port
    uint16_t udp_port;
} tnc_cfg_t;

typedef struct __attribute__((packed)) {
    uint8_t data[UDP_MAX_PACKET_SIZE];
    uint8_t len;
} data_packet_t;

// Starts the telemetry and control service
void tnc_start(const tnc_cfg_t* config);

// Stops the telemetry and control service
void tnc_stop();

// Send out a telemetry packet
int tnc_push_data(void* data, uint8_t len);

#endif