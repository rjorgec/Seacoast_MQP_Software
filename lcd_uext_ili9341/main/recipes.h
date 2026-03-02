#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RECIPES_PATH "/spiffs/recipes.csv"

//mounts SPIFFS and ensures /spiffs exists
esp_err_t recipes_init(void);

//save uploaded CSV into RECIPES_PATH
esp_err_t recipes_save_csv(const uint8_t *data, size_t len);

//does recipes.csv exist
bool recipes_exists(void);

//returns json string
esp_err_t recipes_list_json(char **out_json);

//returns target_g for recipe (first matching row) or ESP_ERR_NOT_FOUND
esp_err_t recipes_get_target_g(uint16_t recipe_id, float *out_target_g);

#ifdef __cplusplus
}
#endif
