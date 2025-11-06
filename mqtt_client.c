// mqtt_client.c  (patched)
#include "mqtt_client.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "lwip/mem.h"
#include "lwip/timeouts.h" // gives you sys_check_timeouts()
#include "lwip/stats.h"

volatile bool mqtt_connected = false; // Global MQTT connection flag

#define INFO_printf printf
#define ERROR_printf printf

#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 0
#define MQTT_PUBLISH_RETAIN 0

#define TEMP_PUBLISH_INTERVAL_MS 10000
#define MQTT_KEEP_ALIVE_S 60

#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C'
#endif

// ---- publish flow control ----
// Maximum number of outstanding publishes waiting for confirmation.
// Tune this between 2..8 depending on network/broker performance.
#define MQTT_MAX_INFLIGHT 4
static volatile int mqtt_inflight_count = 0;

static uint32_t ms_now(void)
{
    // time_us_32 is monotonic; convert to ms
    return time_us_32() / 1000u;
}

static float read_onboard_temperature(char unit)
{
    const float conversionFactor = 3.3f / (1 << 12);
    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return (unit == 'F') ? (tempC * 9 / 5 + 32) : tempC;
}

static void pub_request_cb(void *arg, err_t err)
{
    if (err != 0)
        ERROR_printf("Publish error %d\n", err);

    // Safely decrement inflight counter (don't go below 0)
    if (mqtt_inflight_count > 0)
        mqtt_inflight_count--;
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (!state)
        return;

    // Safe copy of topic into state->topic (ensures NUL termination)
    size_t tlen = strlen(topic);
    size_t tc = (tlen < sizeof(state->topic) - 1) ? tlen : (sizeof(state->topic) - 1);
    memcpy(state->topic, topic, tc);
    state->topic[tc] = '\0';
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (!state)
        return;

    // Safe copy for incoming payload: copy up to size-1 and NUL terminate
    size_t copy_len = (len < sizeof(state->data) - 1) ? len : (sizeof(state->data) - 1);
    if (copy_len > 0)
        memcpy(state->data, data, copy_len);
    state->data[copy_len] = '\0';

    // Extract only the last part of the topic (e.g. "cmd" or "log")
    const char *topic = state->topic;
    const char *last_slash = strrchr(topic, '/');
    const char *topic_tail = last_slash ? last_slash + 1 : topic; // after last '/'

    INFO_printf("Message on %s (type: %s): %s\n", topic, topic_tail, state->data);

    // Trigger user callback with the simplified topic name
    if (state->on_message)
        state->on_message(topic_tail, state->data);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (!state)
        return;

    state->connecting = false;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        state->connect_done = true;
        mqtt_connected = true;
        INFO_printf("MQTT connected\n");

        // (Re-subscribe as you had)
        const char *topics[] = {"/log", "/cmd"};
        char topic_with_id[48];

        for (int i = 0; i < 2; i++)
        {
            snprintf(topic_with_id, sizeof(topic_with_id), "/%s%s", state->mqtt_client_info.client_id, topics[i]);

            cyw43_arch_lwip_begin();
            err_t err = mqtt_sub_unsub(client, topic_with_id, MQTT_SUBSCRIBE_QOS, NULL, state, 1);
            cyw43_arch_lwip_end();

            if (err == ERR_OK)
                INFO_printf("Subscribed to %s\n", topic_with_id);
            else
                ERROR_printf("Subscribe failed for %s: %d\n", topic_with_id, err);

            snprintf(topic_with_id, sizeof(topic_with_id), "/all%s", topics[i]);

            cyw43_arch_lwip_begin();
            err = mqtt_sub_unsub(client, topic_with_id, MQTT_SUBSCRIBE_QOS, NULL, state, 1);
            cyw43_arch_lwip_end();

            if (err == ERR_OK)
                INFO_printf("Subscribed to %s\n", topic_with_id);
            else
                ERROR_printf("Subscribe failed for %s: %d\n", topic_with_id, err);
        }
    }
    else
    {
        ERROR_printf("MQTT connect failed: %d\n", status);
        mqtt_connected = false;

        // Free the client so the next start_client() doesn't leak
        if (state->mqtt_client_inst)
        {
            mqtt_client_free(state->mqtt_client_inst);
            state->mqtt_client_inst = NULL;
        }

        // modest backoff: try again in 3 seconds
        state->next_reconnect_ms = ms_now() + 3000u;
    }
}

static void start_client(MQTT_CLIENT_DATA_T *state)
{
    const int port = MQTT_PORT;

    // Avoid duplicate concurrent attempts
    if (state->connecting)
        return;
    state->connecting = true;

    // FREE any previous client before making a new one
    if (state->mqtt_client_inst)
    {
        mqtt_client_free(state->mqtt_client_inst);
        state->mqtt_client_inst = NULL;
    }

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst)
    {
        state->connecting = false;
        panic("MQTT client creation failed");
    }

    cyw43_arch_lwip_begin();
    mqtt_client_connect(state->mqtt_client_inst,
                        &state->mqtt_server_address,
                        port,
                        mqtt_connection_cb,
                        state,
                        &state->mqtt_client_info);
    mqtt_set_inpub_callback(state->mqtt_client_inst,
                            mqtt_incoming_publish_cb,
                            mqtt_incoming_data_cb,
                            state);
    cyw43_arch_lwip_end();
}

static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (ipaddr)
    {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    }
    else
    {
        ERROR_printf("DNS failed for %s\n", hostname);
    }
}

void mqtt_client_init(MQTT_CLIENT_DATA_T *state, const char *client_id)
{
    memset(state, 0, sizeof(MQTT_CLIENT_DATA_T));
    state->mqtt_client_info.client_id = client_id;
    state->mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
    state->next_reconnect_ms = 0;
    state->connecting = false;
}

void mqtt_client_start(MQTT_CLIENT_DATA_T *state)
{
    // Prevent hammering start while a connect is in progress
    if (state->connecting)
        return;

    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state->mqtt_server_address, dns_found, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
    {
        start_client(state);
    }
    else if (err != ERR_INPROGRESS)
    {
        ERROR_printf("DNS lookup failed\n");
        state->next_reconnect_ms = ms_now() + 3000u;
    }
}

void mqtt_client_publish_temperature(MQTT_CLIENT_DATA_T *state)
{
    float temp = read_onboard_temperature(TEMPERATURE_UNITS);
    char msg[32];
    snprintf(msg, sizeof(msg), "%.2f", temp);

    if (mqtt_inflight_count >= MQTT_MAX_INFLIGHT)
    {
        INFO_printf("MQTT temp publish skipped, inflight=%d\n", mqtt_inflight_count);
        return;
    }

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(state->mqtt_client_inst, "/temperature", msg, strlen(msg),
                             MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
        mqtt_inflight_count++;
    else
        ERROR_printf("mqtt_publish (temp) returned %d\n", err);
}

void mqtt_client_publish_resistance(MQTT_CLIENT_DATA_T *state, char msg[128])
{
    if (!state || !state->mqtt_client_inst)
        return;

    if (mqtt_inflight_count >= MQTT_MAX_INFLIGHT)
    {
        INFO_printf("MQTT resistance publish skipped, inflight=%d\n", mqtt_inflight_count);
        return;
    }

    char topic[48];
    snprintf(topic, sizeof(topic), "/%s/resistance", state->mqtt_client_info.client_id);

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(state->mqtt_client_inst, topic, msg, strlen(msg),
                             MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
        mqtt_inflight_count++;
    else
        ERROR_printf("mqtt_publish (resistance) returned %d\n", err);
}

// "safe" variant now also uses the callback so inflight returns to zero
void mqtt_client_publish_resistance_safe(MQTT_CLIENT_DATA_T *state, const char *msg)
{
    if (!mqtt_client_isconnected(state))
        return;

    if (mqtt_inflight_count >= MQTT_MAX_INFLIGHT)
    {
        printf("[MQTT] Skipping publish, inflight=%d\n", mqtt_inflight_count);
        return;
    }

    char topic[48];
    snprintf(topic, sizeof(topic), "/%s/resistance", state->mqtt_client_info.client_id);

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(state->mqtt_client_inst, topic, msg, strlen(msg),
                             MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
        mqtt_inflight_count++;
    else
        printf("[MQTT] Publish failed (err=%d)\n", err);
}

bool mqtt_client_publish_topic(MQTT_CLIENT_DATA_T *state,
                               const char *subtopic,
                               const char *payload,
                               int retain)
{
    if (!mqtt_client_isconnected(state))
        return false;

    if (mqtt_inflight_count >= MQTT_MAX_INFLIGHT)
    {
        INFO_printf("[MQTT] Skipping publish to %s, inflight=%d\n",
                    subtopic, mqtt_inflight_count);
        return false;
    }

    char topic[64];
    snprintf(topic, sizeof(topic), "/%s/%s",
             state->mqtt_client_info.client_id, subtopic);

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(state->mqtt_client_inst,
                             topic,
                             payload,
                             strlen(payload),
                             MQTT_PUBLISH_QOS,
                             retain ? 1 : 0,
                             pub_request_cb,
                             state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
    {
        mqtt_inflight_count++;
        return true;
    }
    else
    {
        ERROR_printf("[MQTT] publish to %s failed (err=%d)\n", topic, err);
        return false;
    }
}

void mqtt_client_stop(MQTT_CLIENT_DATA_T *state)
{
    if (!state || !state->mqtt_client_inst)
        return;

    cyw43_arch_lwip_begin();
    mqtt_disconnect(state->mqtt_client_inst);
    cyw43_arch_lwip_end();

    mqtt_client_free(state->mqtt_client_inst);
    state->mqtt_client_inst = NULL;
    mqtt_connected = false;
}

bool mqtt_client_isconnected(MQTT_CLIENT_DATA_T *state)
{
    if (!state || !state->mqtt_client_inst)
        return false;
    return mqtt_client_is_connected(state->mqtt_client_inst);
}

// Called every ~10 seconds from your main loop

void mqtt_client_maintenance(MQTT_CLIENT_DATA_T *state)
{
    cyw43_arch_lwip_begin();
    sys_check_timeouts();
    cyw43_arch_lwip_end();

    // Reconnect logic with backoff
    if (state && !mqtt_client_isconnected(state) && state->connect_done && !state->connecting)
    {
        uint32_t now = ms_now();
        if (now >= state->next_reconnect_ms)
        {
            printf("[MQTT] Lost connection, attempting reconnect...\n");
            mqtt_client_start(state);
            // if start fails synchronously, next_reconnect_ms will be set there
            // otherwise, connecting=true prevents spam
        }
    }
}

int mqtt_client_get_inflight(void)
{
    return mqtt_inflight_count;
}
