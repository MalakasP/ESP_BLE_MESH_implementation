ESP BLE Mesh Node with mqtt client
==========================

This BLE Mesh node has the following features:

- Two elements
- Two SIG models
	- **Configuration Server model**: The role of this model is mainly to configure Provisioner device’s AppKey and set up its relay function, TTL size, subscription, etc.
   - **Sensor Client model**: This model implements the basic function of subscribing to sensor server models publish event and relaying that data to MQTT broker using MQTT client.
