#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include <esp_http_server.h>
#include "sf8xxx_nm.h"
#include <string.h>
#include <stdio.h>

#define EXAMPLE_ESP_WIFI_SSID "ESP32_WebServer"
#define EXAMPLE_ESP_WIFI_PASS "11111111"
#define EXAMPLE_MAX_STA_CONN 5

static const char *TAG = "WEB_SERVER";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "Wi-Fi AP стартовал");
    }
}

void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi softAP инициализирован. SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    char query_buf[128];
    char set_status_message[128] = "";
    char get_status_message[128] = "";

    if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) == ESP_OK) {
        ESP_LOGI(TAG, "Query string: %s", query_buf);
        if (httpd_query_key_value(query_buf, "set_status", set_status_message, sizeof(set_status_message)) == ESP_OK) {
            ESP_LOGI(TAG, "Set status message: %s", set_status_message);
        }
        if (httpd_query_key_value(query_buf, "get_status", get_status_message, sizeof(get_status_message)) == ESP_OK) {
            ESP_LOGI(TAG, "Get status message: %s", get_status_message);
        }
    }

    char message_html[200] = "";
    if (strlen(set_status_message) > 0) {
        snprintf(message_html + strlen(message_html), sizeof(message_html) - strlen(message_html),
                 "<p class='status-message set-status'>Команда SET: %s</p>", set_status_message);
    }
    if (strlen(get_status_message) > 0) {
        snprintf(message_html + strlen(message_html), sizeof(message_html) - strlen(message_html),
                 "<p class='status-message get-status'>Команда GET: %s</p>", get_status_message);
    }

    char html_buffer[2048];
    snprintf(html_buffer, sizeof(html_buffer),
             "<!DOCTYPE html>"
             "<html>"
             "<head>"
             "<title>ESP32 Web Server</title>"
             "<meta charset='UTF-8'>"
             "<meta name='viewport' content='width=device-width, initial-scale=1'>"
             "<style>"
             "body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }"
             "input[type=text], input[type=number], button { padding: 10px; margin: 5px; border-radius: 5px; border: 1px solid #ddd; }"
             "button { background-color: #4CAF50; color: white; cursor: pointer; }"
             "button:hover { opacity: 0.8; }"
             ".status-message { margin-top: 10px; padding: 8px; border-radius: 5px; }"
             ".set-status { background-color: #e0ffe0; color: #338833; border-color: #338833; }"
             ".get-status { background-color: #e0f0ff; color: #3366cc; border-color: #3366cc; }"
             ".error-message { background-color: #ffe0e0; color: #cc3333; border-color: #cc3333; }"
             "</style>"
             "</head>"
             "<body>"
             "<h1>Управление SF8xxx NM Driver</h1>"
             "<h2>Отправка команды SET (P-type)</h2>"
             "<form action='/set_param' method='post'>"
             "Номер параметра (HEX): <input type='text' name='param_num' value='0x0000' pattern='0x[0-9A-Fa-f]{1,4}' title='Введите шестнадцатеричное значение (0xXXXX)'><br>"
             "Значение (DEC): <input type='number' name='param_val' min='0' max='65535' value='0'><br>"
             "<button type='submit'>Установить</button>"
             "</form>"
             "<h2>Получение команды GET (J-type)</h2>"
             "<form action='/get_param' method='post'>"
             "Номер параметра (HEX): <input type='text' name='param_num_get' value='0x0000' pattern='0x[0-9A-Fa-f]{1,4}' title='Введите шестнадцатеричное значение (0xXXXX)'><br>"
             "<button type='submit'>Получить</button>"
             "</form>"
             "%s"
             "</body>"
             "</html>",
             message_html);

    httpd_resp_send(req, html_buffer, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_param_post_handler(httpd_req_t *req) {
    char content_buf[128];
    int ret;
    uint16_t param_num = 0;
    uint16_t param_val = 0;
    char status_message[128];

    ret = httpd_req_recv(req, content_buf, sizeof(content_buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content_buf[ret] = '\0';

    ESP_LOGI(TAG, "Получены данные POST для SET: %s", content_buf);

    char *param_num_start = strstr(content_buf, "param_num=");
    char *param_val_start = strstr(content_buf, "param_val=");

    if (param_num_start) {
        sscanf(param_num_start + strlen("param_num="), "%hx", &param_num);
    }
    if (param_val_start) {
        char *val_end = strchr(param_val_start, '&');
        if (val_end) *val_end = '\0';

        sscanf(param_val_start + strlen("param_val="), "%hu", &param_val);
        if (val_end) *val_end = '&';
    }


    ESP_LOGI(TAG, "SET command: param_num=0x%04hx, param_val=%hu", param_num, param_val);

    sf8xxx_nm_err_t sf_err = sf8xxx_nm_set_parameter(param_num, param_val);

    if (sf_err == SF8XXX_NM_OK) {
        snprintf(status_message, sizeof(status_message), "Параметр 0x%04hx успешно установлен в %hu.", param_num, param_val);
    } else {
        snprintf(status_message, sizeof(status_message), "Ошибка установки параметра 0x%04hx (код: %d).", param_num, sf_err);
    }

    char redirect_url[256];
    snprintf(redirect_url, sizeof(redirect_url), "/?set_status=%s", status_message);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", redirect_url);
    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}

static esp_err_t get_param_post_handler(httpd_req_t *req) {
    char content_buf[128];
    int ret;
    uint16_t param_num = 0;
    uint16_t received_value = 0;
    char status_message[128];

    ret = httpd_req_recv(req, content_buf, sizeof(content_buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content_buf[ret] = '\0';

    ESP_LOGI(TAG, "Получены данные POST для GET: %s", content_buf);

    char *param_num_start = strstr(content_buf, "param_num_get=");
    if (param_num_start) {
        sscanf(param_num_start + strlen("param_num_get="), "%hx", &param_num);
    }

    ESP_LOGI(TAG, "GET command (HEX): param_num=0x%04hx", param_num);

    sf8xxx_nm_err_t sf_err = sf8xxx_nm_get_parameter(param_num, &received_value);

    if (sf_err == SF8XXX_NM_OK) {
        snprintf(status_message, sizeof(status_message), "%d", received_value);
    } else {
        snprintf(status_message, sizeof(status_message), "Ошибка получения параметра 0x%04hx (код: %d).", param_num, sf_err);
    }

    char redirect_url[256];
    snprintf(redirect_url, sizeof(redirect_url), "/?get_status=%s", status_message);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", redirect_url);
    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}


static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 4;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 16384;

    ESP_LOGI(TAG, "Запускаем HTTP-сервер на порту: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Регистрируем обработчики URI");
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t set_param_uri = {
            .uri = "/set_param",
            .method = HTTP_POST,
            .handler = set_param_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &set_param_uri);

        httpd_uri_t get_param_uri = {
            .uri = "/get_param",
            .method = HTTP_POST,
            .handler = get_param_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &get_param_uri);

        return server;
    }

    ESP_LOGI(TAG, "Ошибка запуска сервера!");
    return NULL;
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    sf8xxx_nm_init();

    wifi_init_softap();
    start_webserver();
}