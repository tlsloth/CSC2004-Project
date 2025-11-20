#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "Zenden"
#define WIFI_PASSWORD "01020304"

// MQTT Configuration
#define MQTT_BROKER_IP "192.168.1.100"  // Your MQTT broker IP
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "pico_line_follower"

// MQTT Topics
#define MQTT_TOPIC_POSITION "robot/line/position"
#define MQTT_TOPIC_STATE "robot/line/state"
#define MQTT_TOPIC_SPEED "robot/line/speed"
#define MQTT_TOPIC_ERROR "robot/line/error"
#define MQTT_TOPIC_BARCODE "robot/barcode"
#define MQTT_TOPIC_OBSTACLE "robot/obstacle"

#endif // CONFIG_H