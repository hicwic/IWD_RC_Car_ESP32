#pragma once
#include "metrics.h"
#include "esp_http_server.h"

#define MAX_CLIENTS 4

class WebServer {
public:
    static WebServer& getInstance();

    esp_err_t start();
    esp_err_t stop();

    void send_metrics(); // envoyer les metrics à tous les clients WebSocket connectés

    void send_to_all_clients(const std::string& json);

    static esp_err_t root_get_handler(httpd_req_t *req);
    static esp_err_t ws_handler(httpd_req_t *req);
    static esp_err_t velocity_handler(httpd_req_t *req);

    int client_fds_[MAX_CLIENTS];
    int client_count_ = 0;

private:
    WebServer() = default;  // Constructeur privé pour le singleton
    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;


    httpd_handle_t server_{nullptr};

};
