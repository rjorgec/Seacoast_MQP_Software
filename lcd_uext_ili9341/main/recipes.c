#include "recipes.h"

#include "esp_log.h"
#include "esp_spiffs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/stat.h>

static const char *TAG = "recipes";
static bool s_mounted = false;

static char *trim(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) end--;
    *end = '\0';
    return s;
}

esp_err_t recipes_init(void)
{
    if (s_mounted) return ESP_OK;

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return err;
    }

    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS mounted total=%u used=%u", (unsigned)total, (unsigned)used);

    s_mounted = true;
    return ESP_OK;
}

bool recipes_exists(void)
{
    struct stat st;
    return stat(RECIPES_PATH, &st) == 0 && st.st_size > 0;
}

esp_err_t recipes_save_csv(const uint8_t *data, size_t len)
{
    if (!s_mounted) {
        esp_err_t err = recipes_init();
        if (err != ESP_OK) return err;
    }
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    if (len > 64 * 1024) return ESP_ERR_INVALID_SIZE; // guardrail

    FILE *f = fopen(RECIPES_PATH, "wb");
    if (!f) return ESP_FAIL;

    size_t w = fwrite(data, 1, len, f);
    fclose(f);

    if (w != len) return ESP_FAIL;

    ESP_LOGI(TAG, "Saved %u bytes to %s", (unsigned)len, RECIPES_PATH);
    return ESP_OK;
}

static bool parse_csv_line_basic(char *line, uint16_t *out_id, char *out_name, size_t name_cap, float *out_target_g)
{
    //only reads first two fields and then try to find a numeric field later as target_g.
    //will make stricter later

    // split by comma
    char *fields[16] = {0};
    int n = 0;

    char *p = line;
    while (p && *p && n < (int)(sizeof(fields)/sizeof(fields[0]))) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }

    if (n < 2) return false;

    char *id_s = trim(fields[0]);
    char *name_s = trim(fields[1]);

    char *endp = NULL;
    long id = strtol(id_s, &endp, 10);
    if (endp == id_s) return false;

    if (out_id) *out_id = (uint16_t)id;
    if (out_name && name_cap) {
        snprintf(out_name, name_cap, "%s", name_s);
    }

    //find target_g
    float tgt = -1.0f;
    for (int i = 2; i < n; i++) {
        char *fs = trim(fields[i]);
        if (!*fs) continue;
        char *e2 = NULL;
        float v = strtof(fs, &e2);
        if (e2 != fs) { tgt = v; break; }
    }
    if (out_target_g) *out_target_g = tgt;

    return true;
}

esp_err_t recipes_list_json(char **out_json)
{
    if (!out_json) return ESP_ERR_INVALID_ARG;
    *out_json = NULL;

    if (!recipes_exists()) {
        *out_json = strdup("[]");
        return (*out_json) ? ESP_OK : ESP_ERR_NO_MEM;
    }

    FILE *f = fopen(RECIPES_PATH, "rb");
    if (!f) return ESP_FAIL;

    //track unique IDs
    uint16_t ids[64];
    char names[64][32];
    int count = 0;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        //skip header if starts with non-digit
        char *t = trim(line);
        if (!*t) continue;
        if (!isdigit((unsigned char)*t) && *t != '-') continue;

        uint16_t id = 0;
        char name[32] = {0};
        float dummy = 0;
        if (!parse_csv_line_basic(t, &id, name, sizeof(name), &dummy)) continue;

        bool seen = false;
        for (int i = 0; i < count; i++) {
            if (ids[i] == id) { seen = true; break; }
        }
        if (!seen && count < 64) {
            ids[count] = id;
            snprintf(names[count], sizeof(names[count]), "%s", name[0] ? name : "Recipe");
            count++;
        }
    }
    fclose(f);

    // Build JSON
    // worst-case size ~ count * 64
    size_t cap = 16 + (size_t)count * 80;
    char *json = (char *)malloc(cap);
    if (!json) return ESP_ERR_NO_MEM;

    size_t off = 0;
    off += snprintf(json + off, cap - off, "[");
    for (int i = 0; i < count; i++) {
        off += snprintf(json + off, cap - off,
                       "%s{\"id\":%u,\"name\":\"%s\"}",
                       (i ? "," : ""), (unsigned)ids[i], names[i]);
    }
    off += snprintf(json + off, cap - off, "]");

    *out_json = json;
    return ESP_OK;
}

esp_err_t recipes_get_target_g(uint16_t recipe_id, float *out_target_g)
{
    if (!out_target_g) return ESP_ERR_INVALID_ARG;
    *out_target_g = 0.0f;

    if (!recipes_exists()) return ESP_ERR_NOT_FOUND;

    FILE *f = fopen(RECIPES_PATH, "rb");
    if (!f) return ESP_FAIL;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *t = trim(line);
        if (!*t) continue;
        if (!isdigit((unsigned char)*t) && *t != '-') continue;

        uint16_t id = 0;
        char name[32];
        float tgt = -1.0f;
        if (!parse_csv_line_basic(t, &id, name, sizeof(name), &tgt)) continue;

        if (id == recipe_id && tgt >= 0.0f) {
            fclose(f);
            *out_target_g = tgt;
            return ESP_OK;
        }
    }

    fclose(f);
    return ESP_ERR_NOT_FOUND;
}
