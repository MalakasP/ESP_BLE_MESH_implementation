# ESP_BLE_MESH_implementation
Masters Degree Project. IoT ESP32 microcontrollers secure communication method. 
This project implements custom application layer built on Espressif ESP-BLE-MESH protocol.
Application has two builds:
1. Sensor model build for nodes that read data from DHT temperature and humidity sensor and publish it to the network group.
2. Sensor and MQTT client build for node that acts as a gateway in the ESP-BLE-MESH network. It subscribes to the network group and relays sensor data to appropriate MQTT channel.
