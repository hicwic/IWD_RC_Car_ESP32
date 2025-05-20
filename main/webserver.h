#pragma once
#include "metrics.h"
#include "esp_http_server.h"

class WebServer {
public:
    WebServer(Metrics& metrics);
    ~WebServer();

    esp_err_t start();
    esp_err_t stop();

    void send_metrics(); // envoyer les metrics à tous les clients WebSocket connectés

private:
    static esp_err_t root_get_handler(httpd_req_t *req);
    static esp_err_t ws_handler(httpd_req_t *req);

    Metrics& metrics_;

    httpd_handle_t server_{nullptr};

    static constexpr int max_clients_ = 5;
    httpd_req_t* clients_[max_clients_];
    int client_count_ = 0;

    // Pour accès static -> non-static
    static WebServer* instance_;

    void add_client(httpd_req_t* req);
    void remove_client(httpd_req_t* req);

    void send_to_all_clients(const std::string& json);
};
