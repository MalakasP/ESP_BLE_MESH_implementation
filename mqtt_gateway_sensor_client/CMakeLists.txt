# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_init)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_client_mqtt_gateway)

target_add_binary_data(sensor_client_mqtt_gateway.elf "main/esp_client_crt.pem" TEXT)
target_add_binary_data(sensor_client_mqtt_gateway.elf "main/esp_client_key.pem" TEXT)
target_add_binary_data(sensor_client_mqtt_gateway.elf "main/server_root_cert.pem" TEXT)