#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/icmp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "ping";

#define PING_DATA_SIZE 32
#define PING_TIMEOUT_MS 1000

static uint16_t checksum(void *b, int len)
{
    uint16_t *buf = b;
    unsigned int sum = 0;
    uint16_t result;

    for (sum = 0; len > 1; len -= 2)
        sum += *buf++;
    if (len == 1)
        sum += *(uint8_t *)buf;
    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    result = ~sum;
    return result;
}

void ping_google()
{
    struct addrinfo hints = {0};
    hints.ai_family = AF_INET; // IPv4
    struct addrinfo *res;
    int err = getaddrinfo("google.com", NULL, &hints, &res);

    if (err != 0 || res == NULL)
    {
        ESP_LOGE(TAG, "DNS lookup failed for google.com");
        return;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = ((struct sockaddr_in *)res->ai_addr)->sin_addr.s_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(0); // ICMP doesn't use port numbers

    ESP_LOGI(TAG, "Pinging %s", inet_ntoa(dest_addr.sin_addr));

    // Create socket
    int sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Failed to create socket");
        freeaddrinfo(res);
        return;
    }

    struct icmp_echo_hdr echo_req;
    echo_req.type = ICMP_ECHO;
    echo_req.code = 0;
    echo_req.chksum = 0;
    echo_req.id = esp_random() & 0xFFFF;
    echo_req.seqno = htons(1); // Sequence number

    // Fill ICMP payload with pattern
    char send_buf[sizeof(echo_req) + PING_DATA_SIZE];
    memcpy(send_buf, &echo_req, sizeof(echo_req));
    memset(send_buf + sizeof(echo_req), 0xA5, PING_DATA_SIZE); // Fill payload with pattern 0xA5

    // Calculate checksum
    echo_req.chksum = checksum(send_buf, sizeof(send_buf));
    memcpy(send_buf, &echo_req, sizeof(echo_req)); // Copy back the checksum into packet

    // Send ping request
    if (sendto(sock, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
    {
        ESP_LOGE(TAG, "Ping request failed");
        close(sock);
        freeaddrinfo(res);
        return;
    }

    ESP_LOGI(TAG, "Ping request sent");

    // Receive ping response
    char recv_buf[64];
    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);

    fd_set read_set;
    struct timeval timeout;
    timeout.tv_sec = PING_TIMEOUT_MS / 1000;
    timeout.tv_usec = (PING_TIMEOUT_MS % 1000) * 1000;

    FD_ZERO(&read_set);
    FD_SET(sock, &read_set);

    int ret = select(sock + 1, &read_set, NULL, NULL, &timeout);
    if (ret > 0 && FD_ISSET(sock, &read_set))
    {
        int len = recvfrom(sock, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&source_addr, &addr_len);
        if (len > 0)
        {
            ESP_LOGI(TAG, "Received ping response from %s", inet_ntoa(source_addr.sin_addr));
        }
        else
        {
            ESP_LOGE(TAG, "Ping response reception failed");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Ping request timed out");
    }

    // Clean up
    close(sock);
    freeaddrinfo(res);
}
