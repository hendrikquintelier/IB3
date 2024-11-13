#include <stdio.h>
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "http_server";

/* Handler for the `/log` endpoint */
esp_err_t log_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "API endpoint `/log` was called.");

    // Send a simple response back to the client
    const char *resp = "Log event received";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* Function to start the HTTP server */
httpd_handle_t start_webserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the HTTP server
    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Register the handler for the `/log` endpoint
        httpd_uri_t log_uri = {
            .uri = "/log",
            .method = HTTP_GET,
            .handler = log_get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &log_uri);
        return server;
    }

    ESP_LOGI(TAG, "Failed to start HTTP server!");
    return NULL;
}

/* Function to stop the HTTP server */
void stop_webserver(httpd_handle_t server)
{
    // Stop the server if it is running
    if (server)
    {
        httpd_stop(server);
    }
}
