#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "pico/async_context.h"

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

#ifndef MQTT_OUTPUT_RINGBUF_SIZE
#define MQTT_OUTPUT_RINGBUF_SIZE 256
#endif

typedef struct
{
    mqtt_client_t *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
    uint32_t next_reconnect_ms;
    bool connecting;

    void (*on_message)(const char *topic, const char *payload);
} MQTT_CLIENT_DATA_T;

extern volatile bool mqtt_connected;

void mqtt_client_init(MQTT_CLIENT_DATA_T *state, const char *client_id);
void mqtt_client_start(MQTT_CLIENT_DATA_T *state);
void mqtt_client_publish_temperature(MQTT_CLIENT_DATA_T *state);
void mqtt_client_publish_resistance(MQTT_CLIENT_DATA_T *state, char msg[128]);
void mqtt_client_publish_resistance_safe(MQTT_CLIENT_DATA_T *state, const char *msg);
void mqtt_client_stop(MQTT_CLIENT_DATA_T *state);
void mqtt_client_maintenance(MQTT_CLIENT_DATA_T *state);

int mqtt_client_get_inflight(void);

bool mqtt_client_isconnected(MQTT_CLIENT_DATA_T *state);

#endif
