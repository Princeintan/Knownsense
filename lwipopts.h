#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Need more memory for TLS (we'll unify MEM_SIZE below)
#ifdef MQTT_CERT_INC
// (leave empty; we override MEM_SIZE later to one value)
#endif

// Use the common baseline
#include "lwipopts_examples_common.h"

// ---- OVERRIDES / ADDS ----

// Enable detailed stats so HEALTH prints can read them
#undef LWIP_STATS
#define LWIP_STATS 1
#undef LWIP_STATS_DISPLAY
#define LWIP_STATS_DISPLAY 0
#undef MEM_STATS
#define MEM_STATS 1
#undef MEMP_STATS
#define MEMP_STATS 1

// Timeouts count
#define MEMP_NUM_SYS_TIMEOUT (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 1)

// Pooled/heap sizing (good defaults for MQTT telemetry)
#undef MEM_SIZE
#define MEM_SIZE (16 * 1024)
#undef PBUF_POOL_SIZE
#define PBUF_POOL_SIZE 32
#undef MEMP_NUM_TCP_SEG
#define MEMP_NUM_TCP_SEG 32
#undef TCP_SND_BUF
#define TCP_SND_BUF (6 * TCP_MSS)
#undef TCP_WND
#define TCP_WND (8 * TCP_MSS)

// TLS case: ensure window is large enough (keep your note)
#ifdef MQTT_CERT_INC
#undef TCP_WND
#define TCP_WND 16384
#endif

// MQTT inflight (broker/library limit)
#define MQTT_REQ_MAX_IN_FLIGHT 5

#endif
