/**
 * @file lwipopts.h
 * @brief lwIP configuration for Pico W with FreeRTOS and MQTT telemetry.
 *
 *  - Target: RP2040 + CYW43 Wi-Fi (Pico W)
 *  - RTOS:   FreeRTOS
 *  - Stack:  cyw43_arch_lwip_sys_freertos
 *  - Usage:  MQTT, IMU telemetry, PID debug publishing
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* =========================================================================
 *  System integration
 * ========================================================================= */
#define NO_SYS                          0    /*  0 for FreeRTOS integration  1 for bare-metal */
#define LWIP_PROVIDE_ERRNO              1    /* ✅ Let lwIP provide errno */   
#define LWIP_COMPAT_MUTEX_ALLOWED       1   /* ✅ Allow mutexes */
#define SYS_LIGHTWEIGHT_PROT            1
#define LWIP_COMPAT_MUTEX               1
#define LWIP_TIMERS                     1
#define LWIP_TIMERS_CUSTOM              0

/* =========================================================================
 *  Memory and buffer management
 * ========================================================================= */
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (32 * 1024)
#define MEMP_NUM_PBUF                   16
#define MEMP_NUM_TCP_PCB                5
#define MEMP_NUM_TCP_SEG                16
#define MEMP_NUM_SYS_TIMEOUT            10

#define PBUF_POOL_SIZE                  16
#define PBUF_POOL_BUFSIZE               1536

#define LWIP_RAM_HEAP_POINTER           0
#define MEMP_MEM_MALLOC                 1
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT 1

/* =========================================================================
 *  TCP/IP & Networking
 * ========================================================================= */
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0

#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_DHCP                       1
#define LWIP_DNS                        1
#define LWIP_SNTP                       1

#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define TCP_TTL                         255
#define TCP_QUEUE_OOSEQ                 0
#define TCP_MSS                         1460
#define TCP_SND_BUF                     (8 * TCP_MSS)
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                (2 * TCP_SND_BUF / TCP_MSS)
#define LWIP_TCP_KEEPALIVE              1
#define LWIP_TCP_TIMESTAMPS             0

/* =========================================================================
 *  API & Socket layers
 * ========================================================================= */
#define LWIP_NETCONN                    1     /* ✅ Needed for MQTT client */
#define LWIP_SOCKET                     0     /* Disable BSD sockets to save RAM */
#define LWIP_NETIF_API                  1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1

/* =========================================================================
 *  DHCP/DNS
 * ========================================================================= */
#define LWIP_DHCP_CHECK_LINK_UP         1
#define LWIP_DHCP_DOES_ARP_CHECK        1
#define LWIP_DHCP_MAX_TRIES             5
#define LWIP_DNS_SECURE                 7
#define DNS_MAX_NAME_LENGTH             64
#define DNS_MAX_SERVERS                 2

/* =========================================================================
 *  Threading (FreeRTOS)
 * ========================================================================= */
#define TCPIP_THREAD_NAME               "lwIP"
#define TCPIP_THREAD_STACKSIZE          4096
#define TCPIP_THREAD_PRIO               (configMAX_PRIORITIES - 2)

#define DEFAULT_THREAD_STACKSIZE        2048
#define DEFAULT_THREAD_PRIO             (configMAX_PRIORITIES - 3)
#define DEFAULT_RAW_RECVMBOX_SIZE       8
#define DEFAULT_UDP_RECVMBOX_SIZE       8
#define DEFAULT_TCP_RECVMBOX_SIZE       8
#define DEFAULT_ACCEPTMBOX_SIZE         8

/* =========================================================================
 *  Checksum configuration
 * ========================================================================= */
#define CHECKSUM_BY_HARDWARE            0
#define LWIP_CHECKSUM_CTRL_PER_NETIF    0
#define LWIP_CHECKSUM_ON_COPY           1

#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

/* =========================================================================
 *  Debugging & Statistics
 * ========================================================================= */
#define LWIP_DEBUG                      0
#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define DHCP_DEBUG                      LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF
#define TCP_DEBUG                       LWIP_DBG_OFF
#define MQTT_DEBUG                      LWIP_DBG_OFF

/* =========================================================================
 *  Integration with Pico W Wi-Fi (CYW43)
 * ========================================================================= */
#define CYW43_LWIP                      1
#define CYW43_ENABLE_BSD                0
#define CYW43_THREAD_SAFE               1
#define CYW43_USE_FREERTOS              1
#define CYW43_USE_LWIP                  1
#define CYW43_SLEEP_ENABLE              1

#endif /* __LWIPOPTS_H__ */