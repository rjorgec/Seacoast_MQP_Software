#include "web_server.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include "recipes.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "web";

static httpd_handle_t s_server = NULL;

static const char *INDEX_HTML =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Seacoast Recipes</title></head><body style='font-family:sans-serif;max-width:720px;margin:20px'>"
"<h2>Seacoast Inoculator • Recipe Upload</h2>"
"<p>Upload a CSV file. It will be stored on the ESP32 at <code>/spiffs/recipes.csv</code>.</p>"
"<input id='f' type='file' accept='.csv,text/csv'/>"
"<button id='b'>Upload</button>"
"<pre id='out' style='padding:10px;background:#f3f3f3;white-space:pre-wrap'></pre>"
"<p><a href='/recipes.csv'>Download current recipes.csv</a></p>"
"<p><a href='/recipes'>View recipe list (JSON)</a></p>"
"<script>"
"const out=(s)=>document.getElementById('out').textContent=s;"
"document.getElementById('b').onclick=async()=>{"
" const fi=document.getElementById('f');"
" if(!fi.files||!fi.files[0]){out('Choose a CSV first');return;}"
" const file=fi.files[0];"
" out('Uploading '+file.name+' ('+file.size+' bytes)...');"
" const buf=await file.arrayBuffer();"
" const res=await fetch('/upload',{method:'POST',headers:{'Content-Type':'text/csv'},body:buf});"
" out('Status: '+res.status+'\\n'+await res.text());"
"};"
"</script></body></html>";

static esp_err_t send_text(httpd_req_t *req, const char *txt, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_send(req, txt, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t handle_root(httpd_req_t *req)
{
    return send_text(req, INDEX_HTML, "text/html");
}

static esp_err_t handle_upload(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_OK;
    }
    if (total > 64 * 1024) {
        httpd_resp_send_err(req, HTTPD_413_PAYLOAD_TOO_LARGE, "Too large (max 64KB)");
        return ESP_OK;
    }

    uint8_t *buf = (uint8_t *)malloc((size_t)total);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No mem");
        return ESP_OK;
    }

    int read = 0;
    while (read < total) {
        int r = httpd_req_recv(req, (char *)buf + read, total - read);
        if (r <= 0) {
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv failed");
            return ESP_OK;
        }
        read += r;
    }

    esp_err_t err = recipes_save_csv(buf, (size_t)total);
    free(buf);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "recipes_save_csv failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Upload OK (%d bytes)", total);
    return send_text(req, "OK: recipes.csv saved\n", "text/plain");
}

static esp_err_t handle_recipes_csv(httpd_req_t *req)
{
    if (!recipes_exists()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No recipes.csv uploaded yet");
        return ESP_OK;
    }

    FILE *f = fopen(RECIPES_PATH, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Open failed");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/csv");

    char chunk[512];
    size_t n;
    while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, n) != ESP_OK) {
            fclose(f);
            httpd_resp_sendstr_chunk(req, NULL);
            return ESP_OK;
        }
    }
    fclose(f);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t handle_recipes_json(httpd_req_t *req)
{
    char *json = NULL;
    esp_err_t err = recipes_list_json(&json);
    if (err != ESP_OK || !json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON build failed");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return ESP_OK;
}

esp_err_t web_server_start(void)
{
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 4096;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    ESP_LOGI(TAG, "Starting web server on port %d", cfg.server_port);
    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;

    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handle_root,
    };
    httpd_uri_t uri_upload = {
        .uri = "/upload",
        .method = HTTP_POST,
        .handler = handle_upload,
    };
    httpd_uri_t uri_csv = {
        .uri = "/recipes.csv",
        .method = HTTP_GET,
        .handler = handle_recipes_csv,
    };
    httpd_uri_t uri_list = {
        .uri = "/recipes",
        .method = HTTP_GET,
        .handler = handle_recipes_json,
    };

    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_upload);
    httpd_register_uri_handler(s_server, &uri_csv);
    httpd_register_uri_handler(s_server, &uri_list);

    return ESP_OK;
}
