
#include "telemetryandcontrol.h"
#include "mdns.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include "pb_decode.h"
#include "pb_encode.h"
#include "robot.pb.h"
#include "state.h"

#define MDNS_HOSTNAME "balancingbot"
#define MDNS_FRIENDLY "Balancing robot controller"

static tnc_cfg_t my_config;
static bool tnc_enabled = false;

static volatile int udp_sock = -1;
static TaskHandle_t udp_output_task_h = NULL;
static volatile bool udp_output_task_should_stop = false;
static TaskHandle_t udp_input_task_h = NULL;
static QueueHandle_t udp_tx_queue = NULL;

static uint8_t n_udp_subscribers = 0;
static struct sockaddr_storage udp_subscribers[UDP_MAX_SUBSCRIBERS];

// Private
static void udp_output_task(void *pvParamters){
    data_packet_t pkt;
    udp_output_task_should_stop = false;

    // Create the output queue
    if(!udp_tx_queue){
        udp_tx_queue = xQueueCreate(UDP_PACKET_QUEUE_SIZE, sizeof(data_packet_t));
        if(!udp_tx_queue){
            ESP_LOGE("TNC", "Failed to create output queue, system out of memory?");
            goto cleanup;
        }
    }else{
        ESP_LOGE("TNC", "Output queue already existed, this should not happen!");
    }

    while(!udp_output_task_should_stop){
        if(xQueueReceive(udp_tx_queue, &pkt, portMAX_DELAY)){
            if(pkt.len == 0){ // Skip empty and sentinel packets
                continue;
            }

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

cleanup:
    // Remove the queue including all pending messages
    if(udp_tx_queue){
        vQueueDelete(udp_tx_queue);
        udp_tx_queue = NULL;
    }

    udp_output_task_h = NULL;
    vTaskDelete(NULL);
    return;
}

// Private
static void udp_input_task(void *pvParameters) {
    uint8_t rx_buffer[256];

    // Create the socket we need
    if(udp_sock < 0){
        udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(udp_sock < 0){
            ESP_LOGE("TNC", "Failed to create UDP socket!");
            return;
        }

        struct sockaddr_in listen_addr;
        listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons(my_config.udp_port);

        if(bind(udp_sock, (const struct sockaddr*)&listen_addr, sizeof(listen_addr)) < 0){
            ESP_LOGE("TNC", "Failed to bind UDP socket!");
            closesocket(udp_sock);
            udp_sock = -1;
            return;
        }
    }

    while (true) {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        // Receive raw bytes
        int len = recvfrom(udp_sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            //WARNING: We dont filter on errno's here so we also close on EAGAIN and EWOULDBLOCK.
            // This should never be an issue with an infinite timeout and blocking socket. So we keep it simple.
            ESP_LOGE("UDP", "Failed to receive data: %d, error: %d", len, errno);
            break;
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

                // Prevent out of bounds access
                if(idx < 0 || idx >= PID_COUNT){
                    ESP_LOGE("UDP", "Invalid PID index: %d", idx);
                    break;
                }
                
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

                // Prevent out of bounds access
                if(s->target < 0 || s->target >= PID_COUNT){
                    ESP_LOGE("UDP", "Invalid PID index: %d", s->target);
                    break;
                }

                rstate.pids[s->target].setpoint = s->setpoint;
                ESP_LOGI("UDP", "Setpoint PID %d set to %f", s->target, s->setpoint);

                // TODO: send ack
                break;
            }

            // Set robot control (speed and turn rate)
            case ConfigMessage_set_robot_controlp_tag: {
                SetRobotControl *c = &msg.payload.set_robot_controlp;
                rstate.target_speed = c->speed;
                rstate.target_turn_rate = c->turn_rate;
                ESP_LOGI("UDP", "Robot control set: speed=%.2f cm/s, turn_rate=%.2f deg/s", 
                         c->speed, c->turn_rate);

                // TODO: send ack
                break;
            }

            // Get PID params
            case ConfigMessage_get_pid_paramsp_tag: {
                GetPidParams *p = &msg.payload.get_pid_paramsp;
                if(p->target < 0 || p->target >= PID_COUNT){
                    ESP_LOGE("UDP", "Invalid PID index: %d", p->target);
                    break;
                }

                ESP_LOGI("UDP", "Requested params for PID %d", p->target);

                // Create the reply message
                ConfigMessage reply = ConfigMessage_init_default;
                reply.transaction_id = msg.transaction_id;
                reply.which_payload  = ConfigMessage_get_pid_params_replyp_tag;
                reply.payload.get_pid_params_replyp.target = p->target;
                reply.payload.get_pid_params_replyp.kp = rstate.pids[p->target].Kp;
                reply.payload.get_pid_params_replyp.ki = rstate.pids[p->target].Ki;
                reply.payload.get_pid_params_replyp.kd = rstate.pids[p->target].Kd;
                reply.payload.get_pid_params_replyp.setpoint = rstate.pids[p->target].setpoint;

                // Encode the reply message
                uint8_t tx_buffer[ConfigMessage_size];
                pb_ostream_t stream = pb_ostream_from_buffer(tx_buffer, sizeof(tx_buffer));
                if(!pb_encode(&stream, ConfigMessage_fields, &reply)){
                    ESP_LOGE("UDP", "Encoding reply failed: %s", PB_GET_ERROR(&stream));
                    break;
                }

                // Directly send back to the sender
                int rc = sendto(udp_sock, tx_buffer, stream.bytes_written, 0, (struct sockaddr*)&source_addr, socklen);
                if(rc < 0){
                    ESP_LOGE("UDP", "Failed to send reply: %d", rc);
                }

                break;
            }

            default: {
                ESP_LOGW("UDP", "Received unknown payload tag: %d", msg.which_payload);
                break;
            }
        }
    }

    // Task cleanup
    if(udp_sock > -1){
        closesocket(udp_sock);
        udp_sock = -1;
    }
    udp_input_task_h = NULL;
    vTaskDelete(NULL);
    return;
}


// Private
static bool start_mdns(){
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

// Private
static int start_udp_tasks(){

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
            return -1;
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
            return -1;
        }
    }
    return 0;
}

// Private
static void stop_udp_tasks(){
    if(udp_output_task_h){
        data_packet_t sentinel_pkt;
        sentinel_pkt.len = 0;

        // Send the task a signal to stop and send it a sentinel packet to break out of the queue wait
        // The task will delete itself and resets its own task handle to NULL
        udp_output_task_should_stop = true;
        if(udp_tx_queue){
            // Queue might not be created yet, so setting the flag should be enough
            xQueueSend(udp_tx_queue, &sentinel_pkt, 0); // Send sentinel packet to stop the task
        }
    }
    if(udp_input_task_h){
        int temp_sock = udp_sock;
        udp_sock = -1;
        // We close the socket so recvfrom returns an error and the task will fall into the cleanup section
        closesocket(temp_sock);
    }
}

// Public
int tnc_start(const tnc_cfg_t* config){
    if(tnc_enabled) return -1;
    if(!config) return -1;
    memcpy(&my_config, config, sizeof(my_config));

    // DEBUG set our pc as standard subscriber 
    struct sockaddr_in *own_pc = (struct sockaddr_in *)&udp_subscribers[0];
    memset(&udp_subscribers[0], 0, sizeof(struct sockaddr_storage));
    own_pc->sin_family = AF_INET;
    own_pc->sin_port = htons(47269);
    own_pc->sin_addr.s_addr = inet_addr("192.168.2.27");
    n_udp_subscribers = 1;

    if(start_udp_tasks() < 0){
        ESP_LOGE("TNC", "Failed to start UDP tasks!");
        return -1;
    }
    if(!start_mdns()){
        ESP_LOGE("TNC", "Failed to start mDNS service!, continuing anyway...");
    }
    ESP_LOGI("TNC", "Telemetry and control service started!");

    tnc_enabled = true;
    return 0;
}

// Public
void tnc_stop(){
    mdns_free();
    stop_udp_tasks();
    
    ESP_LOGI("TNC", "Telemetry and control service stopped!");
    tnc_enabled = false;
}

// Public
int tnc_push_data(const void* data, uint8_t len){
    if(!data) return -1;
    if(len == 0) return -1;
    if(!tnc_enabled || !udp_tx_queue){
        ESP_LOGE("TNC", "Tried to push data while TNC was not enabled!");
        return -1;
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
