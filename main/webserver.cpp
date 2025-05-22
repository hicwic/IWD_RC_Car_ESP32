#include "webserver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "WebServer";

WebServer* WebServer::instance_ = nullptr;

WebServer::WebServer(Metrics& metrics) : metrics_(metrics) {
    instance_ = this;
}

WebServer::~WebServer() {
    stop();
    instance_ = nullptr;
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

    return ESP_OK;
}

esp_err_t WebServer::stop() {
    if (server_ != nullptr) {
        return httpd_stop(server_);
    }
    return ESP_OK;
}

void WebServer::add_client(httpd_req_t* req) {
    if (client_count_ < max_clients_) {
        clients_[client_count_++] = req;
        ESP_LOGI(TAG, "Client added, total: %d", client_count_);
    } else {
        ESP_LOGW(TAG, "Max clients reached, reject new client");
    }
}


void WebServer::remove_client(httpd_req_t* req) {
    for (int i = 0; i < client_count_; i++) {
        if (clients_[i] == req) {
            for (int j = i; j < client_count_ - 1; j++) {
                clients_[j] = clients_[j + 1];
            }
            clients_[client_count_ - 1] = nullptr;
            client_count_--;
            ESP_LOGI(TAG, "Client removed, total: %d", client_count_);
            break;
        }
    }
}


void WebServer::send_to_all_clients(const std::string& json) {
    if (client_count_ == 0) return;

    httpd_ws_frame_t ws_pkt = {};
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t*)json.c_str();
    ws_pkt.len = json.length();

    for (int i = 0; i < client_count_; i++) {
        httpd_req_t* client_req = clients_[i];
        int sockfd = httpd_req_to_sockfd(client_req);
        esp_err_t ret = httpd_ws_send_frame_async(server_, sockfd, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send async frame to client %d: %s", i, esp_err_to_name(ret));
        }
    }
}

void WebServer::send_metrics() {
    std::string json = metrics_.to_json();
    send_to_all_clients(json);
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

esp_err_t WebServer::ws_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "WebSocket client connected");

    if (!instance_) {
        ESP_LOGE(TAG, "WebServer instance not set!");
        return ESP_FAIL;
    }

    instance_->add_client(req);

    while (true) {
        httpd_ws_frame_t ws_pkt = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = nullptr,
            .len = 0
        };

        // Étape 1 : lire l'entête du frame WebSocket (sans payload)
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK) {
            if (ret == ESP_ERR_INVALID_ARG || ret == ESP_ERR_HTTPD_INVALID_REQ) {
                ESP_LOGI(TAG, "Client WebSocket déconnecté (err=%d)", ret);
                break;
            } else {
                ESP_LOGD(TAG, "Lecture WebSocket échouée (err=%d), on réessaie", ret);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
        }

        if (ws_pkt.len > 0) {
            // Étape 2 : allouer un buffer pour recevoir le message
            uint8_t buf[128] = {0};  // ajuste si besoin
            if (ws_pkt.len >= sizeof(buf)) {
                ESP_LOGW(TAG, "WebSocket frame too big (%u bytes), skipped", ws_pkt.len);
                continue;
            }

            ws_pkt.payload = buf;
            ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Received WS message: %.*s", ws_pkt.len, buf);
                // Ici, tu pourrais parser `buf` si nécessaire
            }
        }

        // ✅ Pause minimale dans tous les cas
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    instance_->remove_client(req);
    return ESP_OK;
}
