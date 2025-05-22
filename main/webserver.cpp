#include "webserver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "WebServer";


WebServer& WebServer::getInstance() {
    static WebServer instance;  // 🔒 Unique instance (thread-safe avec C++11+)
    return instance;
}

esp_err_t WebServer::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting HTTP Server...");
    esp_err_t ret = httpd_start(&server_, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP Server");
        return ret;
    }

    // URI handler racine (page HTML)
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &root_uri);

    // URI handler websocket
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = nullptr,
        .is_websocket = true
    };
    httpd_register_uri_handler(server_, &ws_uri);

    httpd_uri_t velocity_uri = {
        .uri = "/velocity",
        .method = HTTP_GET,
        .handler = velocity_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server_, &velocity_uri);
    

    return ESP_OK;
}

esp_err_t WebServer::stop() {
    if (server_ != nullptr) {
        return httpd_stop(server_);
    }
    return ESP_OK;
}


void WebServer::send_to_all_clients(const std::string& json) {
    if (client_count_ == 0) return;

    httpd_ws_frame_t ws_pkt = {};
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t*)json.c_str();
    ws_pkt.len = json.length();

    int i = 0;
    while (i < client_count_) {
        int sockfd = client_fds_[i];
        esp_err_t ret = httpd_ws_send_frame_async(server_, sockfd, &ws_pkt);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Client %d déconnecté (%s), on le supprime.", i, esp_err_to_name(ret));
            // Supprimer le client de la liste
            for (int j = i; j < client_count_ - 1; ++j) {
                client_fds_[j] = client_fds_[j + 1];
            }
            client_count_--;
        } else {
            i++;
        }
    }
}

esp_err_t WebServer::root_get_handler(httpd_req_t *req) {
    const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>RC Metrics</title></head>
<body>
<h1>ESC Control Metrics (WebSocket)</h1>
<pre id="metrics">Connecting...</pre>
<script>
let ws = new WebSocket('ws://' + location.host + '/ws');
ws.onmessage = function(evt) {
    let data = JSON.parse(evt.data);
    let display = '';
    for (let key in data) {
        display += key + ': ' + data[key] + '\n';
    }
    document.getElementById('metrics').textContent = display;
};
ws.onopen = function() {
    document.getElementById('metrics').textContent = 'Connected, waiting for data...';
};
ws.onclose = function() {
    document.getElementById('metrics').textContent = 'Disconnected';
};
</script>
</body>
</html>
    )rawliteral";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}


esp_err_t WebServer::velocity_handler(httpd_req_t *req) {
    extern const char velocity_html_start[] asm("_binary_velocity_html_start");
    extern const char velocity_html_end[] asm("_binary_velocity_html_end");    
    const size_t html_len = velocity_html_end - velocity_html_start;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, velocity_html_start, html_len);
}

esp_err_t WebServer::ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    WebServer& ws = WebServer::getInstance();

    if (ws.client_count_ < MAX_CLIENTS) {
        ws.client_fds_[ws.client_count_++] = fd;
        ESP_LOGI(TAG, "Client WS connecté, socket: %d, total: %d", fd, ws.client_count_);
    }

    // On ne lit rien, on ne fait que envoyer en push
    return ESP_OK;
}
