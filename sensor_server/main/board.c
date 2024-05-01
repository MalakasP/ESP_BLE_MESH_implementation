#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "board.h"
#include "dht.h"
#include "max1704x.h"
#include "i2cdev.h"

#define TAG "BOARD"

#define I2C_MASTER_SDA 0x12
#define I2C_MASTER_SCL 0x13

static const dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;

int16_t temperature = 0;

int16_t humidity = 0;

max1704x_t dev = { 0 };

float voltage = 0;

float soc_percent = 0;

float rate_change = 0;

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

void board_max17048_init(void)
{
    // max1704x_config_t config = { 0 };
    max1704x_status_t status = { 0 };
    uint16_t version = 0;

    /**
     * Set up I2C bus to communicate with MAX1704X
     */
    dev.model = MAX17048_9;

    ESP_ERROR_CHECK(max1704x_init_desc(&dev, I2C_NUM_0, I2C_MASTER_SDA, I2C_MASTER_SCL));
    ESP_ERROR_CHECK(max1704x_quickstart(&dev));
    ESP_ERROR_CHECK(max1704x_get_version(&dev, &version));
    ESP_LOGI(TAG, "Version: %d\n", version);

    /**
     * Get MAX1704X configuration
     */ 
    ESP_LOGI(TAG, "--- MAX1704X config register ---");
    ESP_ERROR_CHECK(max1704x_get_config(&dev));
    ESP_LOGI(TAG, "Alert Status: %d", dev.config.alert_status);
    ESP_LOGI(TAG, "Sleep Mode: %d", dev.config.sleep_mode);
    ESP_LOGI(TAG, "SOC Change Alert Mode: %d", dev.config.soc_change_alert);
    ESP_LOGI(TAG, "Empty Alert Threshold: %d%%", dev.config.empty_alert_thresh);
    ESP_LOGI(TAG, "RCOMP Value: %d (%x)", dev.config.rcomp, dev.config.rcomp);
    ESP_LOGI(TAG, "--- End Configuration ---\n");

    // Change configuration settings
    // config.soc_change_alert = true;
    // config.empty_alert_thresh = 10;
    // ESP_LOGI(TAG, "Setting new MAX1704X configuration");
    // ESP_ERROR_CHECK(max1704x_set_config(&dev, &config));

    // ESP_LOGI(TAG, "--- MAX1704X config register after updating configurations ---");
    // ESP_ERROR_CHECK(max1704x_get_config(&dev));
    // ESP_LOGI(TAG, "Alert Status: %d", dev.config.alert_status);
    // ESP_LOGI(TAG, "Sleep Mode: %d", dev.config.sleep_mode);
    // ESP_LOGI(TAG, "SOC Change Alert Mode: %d", dev.config.soc_change_alert);
    // ESP_LOGI(TAG, "Empty Alert Threshold: %d%%", dev.config.empty_alert_thresh);
    // ESP_LOGI(TAG, "RCOMP Value: %d (%x)", dev.config.rcomp, dev.config.rcomp);
    // ESP_LOGI(TAG, "--- End Configuration ---\n");

    /**
     * Get current MAX1704X status
     */
    ESP_LOGI(TAG, "--- MAX1704X status register ---");
    ESP_ERROR_CHECK(max1704x_get_status(&dev));
    ESP_LOGI(TAG, "Reset Indicator: %d", dev.status.reset_indicator);
    ESP_LOGI(TAG, "Voltage High Alert: %d", dev.status.voltage_high);
    ESP_LOGI(TAG, "Voltage Low Alert: %d", dev.status.voltage_low);
    ESP_LOGI(TAG, "Voltage Reset Alert: %d", dev.status.voltage_reset);
    ESP_LOGI(TAG, "SOC Low Alert: %d", dev.status.soc_low);
    ESP_LOGI(TAG, "SOC Change Alert: %d", dev.status.soc_change);
    ESP_LOGI(TAG, "Voltage Reset Alert Enabled: %d", dev.status.vreset_alert);
    ESP_LOGI(TAG, "--- End Status ---\n");

    // Update MAX1704X status register to clear the reset indicator
    ESP_LOGI(TAG, "Setting the status register to clear reset indicator");
    status.reset_indicator = false;

    ESP_ERROR_CHECK(max1704x_set_status(&dev, &status));

    ESP_LOGI(TAG, "--- MAX1704X status register after updating status register ---");
    ESP_ERROR_CHECK(max1704x_get_status(&dev));
    ESP_LOGI(TAG, "Reset Indicator: %d", dev.status.reset_indicator);
    ESP_LOGI(TAG, "Voltage High Alert: %d", dev.status.voltage_high);
    ESP_LOGI(TAG, "Voltage Low Alert: %d", dev.status.voltage_low);
    ESP_LOGI(TAG, "Voltage Reset Alert: %d", dev.status.voltage_reset);
    ESP_LOGI(TAG, "SOC Low Alert: %d", dev.status.soc_low);
    ESP_LOGI(TAG, "SOC Change Alert: %d", dev.status.soc_change);
    ESP_LOGI(TAG, "Voltage Reset Alert Enabled: %d", dev.status.vreset_alert);
    ESP_LOGI(TAG, "--- End Status ---\n");
}

void board_max17048_read(void)
{
    esp_err_t err;
    
    err = max1704x_get_voltage(&dev, &voltage);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Voltage: %.2fV", voltage);
    }
    else
        ESP_LOGI(TAG, "Error %d: %s", err, esp_err_to_name(err));

    err = max1704x_get_soc(&dev, &soc_percent);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SOC: %.2f%%", soc_percent);
    }
    else
        ESP_LOGI(TAG, "Error %d: %s", err, esp_err_to_name(err));

    err = max1704x_get_crate(&dev, &rate_change);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SOC rate of change: %.2f%%", rate_change);
    }
    else
        ESP_LOGI(TAG, "Error %d: %s", err, esp_err_to_name(err));
}

void task_i2cscanner() {
	ESP_LOGD(TAG, ">> i2cScanner");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA;
	conf.scl_io_num = I2C_MASTER_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_NUM_1, &conf);

	i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}

void board_init(void)
{
    // ESP_ERROR_CHECK(i2cdev_init());
    board_dht_read();
    // board_max17048_init();
    // task_i2cscanner();
}