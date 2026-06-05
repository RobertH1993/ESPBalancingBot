
#include "telemetryandcontrol.h"
#include "mdns.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include <stdbool.h>
#include <string.h>

#include "pb_decode.h"
#include "robot.pb.h"
#include "state.h"

#define MDNS_HOSTNAME "balancingbot"
#define MDNS_FRIENDLY "Balancing robot controller"

tnc_cfg_t my_config;

static bool tnc_enabled = false;

int udp_sock = -1;
TaskHandle_t udp_output_task_h = NULL;
TaskHandle_t udp_input_task_h = NULL;
QueueHandle_t udp_tx_queue = NULL;

uint8_t n_udp_subscribers = 0;
struct sockaddr_storage udp_subscribers[UDP_MAX_SUBSCRIBERS];


esp_event_handler_instance_t wifi_event_handler = NULL;

// Private
void udp_output_task(void *pvParamters){
    data_packet_t pkt;

    while(true){
        if(xQueueReceive(udp_tx_queue, &pkt, portMAX_DELAY)){
            // Send packet to all subscribers
            for(uint8_t i = 0; i < n_udp_subscribers; i++){
                int rc = sendto(udp_sock, pkt.data, pkt.len, 0, (struct sockaddr*)&udp_subscribers[i], sizeof(struct sockaddr_storage));

                if(rc < 0){
                    ESP_LOGW("TNC", "Failed to send data");
                }


            }
        }
    }
    ESP_LOGW("UDPOutputTask", "Task stopped!");

}


void udp_input_task(void *pvParameters) {
    uint8_t rx_buffer[256];

    while (true) {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        // Receive raw bytes
        int len = recvfrom(udp_sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            continue;
        }

        // Create a nanopb stream
        ConfigMessage msg = ConfigMessage_init_default;
        pb_istream_t stream = pb_istream_from_buffer(rx_buffer, len);

        // Decode raw bytes to a struct
        if (!pb_decode(&stream, ConfigMessage_fields, &msg)) {
            ESP_LOGE("UDP", "Decoding failed: %s", PB_GET_ERROR(&stream));
            continue;
        }

        // Parse message based on payload type
        ESP_LOGI("UDP", "Received ConfigMessage, ID: %lu", msg.transaction_id);
        switch (msg.which_payload) {

            // Set PID parameters message
            case ConfigMessage_set_pid_paramsp_tag: {
                SetPidParams *p = &msg.payload.set_pid_paramsp;
                int idx = p->target;
                
                rstate.pids[idx].Kp = p->kp;
                rstate.pids[idx].Ki = p->ki;
                rstate.pids[idx].Kd = p->kd;
                
                ESP_LOGI("UDP", "PID %d updated: P=%f, I=%f, D=%f", idx, p->kp, p->ki, p->kd);
                
                // TODO: send ack
                break;
            }

            // Set PID setpoint
            case ConfigMessage_set_pid_setpointp_tag: {
                SetPidSetpoint *s = &msg.payload.set_pid_setpointp;
                rstate.pids[s->target].setpoint = s->setpoint;
                ESP_LOGI("UDP", "Setpoint PID %d set to %f", s->target, s->setpoint);

                // TODO: send ack
                break;
            }

            // Get PID params
            case ConfigMessage_get_pid_paramsp_tag: {
                // TODO: Create a GetPidParamsReply and send it back
                ESP_LOGI("UDP", "Requested params for PID %d", msg.payload.get_pid_paramsp.target);
                break;
            }

            default: {
                ESP_LOGW("UDP", "Received unknown payload tag: %d", msg.which_payload);
                break;
            }
        }
    }
}


// private
bool start_mdns(){
    esp_err_t err = mdns_init();
    if(err){
        ESP_LOGW("TNC", "Cannot start the mDNS service");
    }else{
        // Configure mDNS
        mdns_hostname_set(MDNS_HOSTNAME);
        mdns_instance_name_set(MDNS_FRIENDLY);

        // Configure telemetry and control port
        mdns_service_add(NULL, "_tnc", "_udp", my_config.udp_port, NULL, 0);

        ESP_LOGI("TNC", "mDNS service started");
        return true;
    }

    return false;
}

// private
void start_udp_tasks(){
    if(udp_sock < 0){
        // Create socket
        udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_TCP);

        struct sockaddr_in listen_addr;
        listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons(my_config.udp_port);

        bind(udp_sock, (const struct sockaddr*)&listen_addr, sizeof(listen_addr));
    }


    if(!udp_output_task_h){ // Create the output task
        if(xTaskCreatePinnedToCore( // Start output task
                    udp_output_task,
                    "udp_output_task",
                    4096,
                    NULL,
                    5,
                    &udp_output_task_h,
                    0
        ) != pdPASS){
            ESP_LOGE("TNC", "Failed to start UDP output task!");
        }
    }

    if(!udp_input_task_h){ // Create the input task
        if(xTaskCreatePinnedToCore( // Start output task
                    udp_input_task,
                    "udp_input_task",
                    4096,
                    NULL,
                    5,
                    &udp_input_task_h,
                    0
        ) != pdPASS){
            ESP_LOGE("TNC", "Failed to start UDP input task!");
        }
    }
}

// Private
void stop_udp_tasks(){
    if(udp_output_task_h){
        vTaskDelete(udp_output_task_h);
        udp_output_task_h = NULL;
    }
    if(udp_input_task_h){
        vTaskDelete(udp_input_task_h);
        udp_input_task_h = NULL;
    }

    if(udp_sock > -1){
        closesocket(udp_sock);
        udp_sock = -1;
    }
}


void tnc_start(const tnc_cfg_t* config){
    if(tnc_enabled) return;
    mempcpy(&my_config, config, sizeof(my_config));
    start_mdns();

    // Create the output queue
    if(!udp_tx_queue){
        udp_tx_queue = xQueueCreate(UDP_PACKET_QUEUE_SIZE, sizeof(data_packet_t));
    }

    // DEBUG set our pc as standard subscriber 
    struct sockaddr_in *own_pc = (struct sockaddr_in *)&udp_subscribers[0];
    memset(&udp_subscribers[0], 0, sizeof(struct sockaddr_storage));
    own_pc->sin_family = AF_INET;
    own_pc->sin_port = htons(47269);
    own_pc->sin_addr.s_addr = inet_addr("192.168.2.24");
    n_udp_subscribers = 1;

    start_udp_tasks();

    ESP_LOGI("TNC", "Telemetry and control service started!");

    tnc_enabled = true;
}

void tnc_stop(){
    mdns_free();
    stop_udp_tasks();
    
    // Remove the output queue on full stop, pending messages are gone
    if(udp_tx_queue){
        vQueueDelete(udp_tx_queue);
    }

    ESP_LOGI("TNC", "Telemetry and control service stopped!");

    tnc_enabled = false;
}

int tnc_push_data(void* data, uint8_t len){
    if(!data) return -1;
    if(!tnc_enabled){
        ESP_LOGE("TNC", "Tried to push data while TNC was not enabled!");
    }

    data_packet_t packet;
    packet.len = len;
    memcpy(packet.data, data, len);

    if (xQueueSend(udp_tx_queue, &packet, 0) == errQUEUE_FULL) {
        data_packet_t dummy;
        xQueueReceive(udp_tx_queue, &dummy, 0); // Remove oldest
        xQueueSend(udp_tx_queue, &packet, 0);   // Insert newest
    }
    return 0;
}
