#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "board.h"
#include "dht.h"

#define TAG "BOARD"

static const dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;

int16_t temperature = 0;

int16_t humidity = 0;

int16_t board_dht_get_temperature(void)
{
    return temperature * 10;
}

int16_t board_dht_get_humidity(void)
{
    return humidity * 10;
}

void board_dht_read(void)
{
    dht_read_data(sensor_type, DHT_GPIO, &humidity, &temperature);
}

void board_init(void)
{
    board_dht_read();
}