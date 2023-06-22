
#include "nvs_flash.h"
#include <stdio.h>
#include "aws-iot.h"
#include "DHT22.h"

void app_main()
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    APP_DHT22_task();

    initialise_wifi();
    aws_iot_start();
}