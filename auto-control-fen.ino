#include <Wire.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
#define WIFI_SSID "04"
#define WIFI_PASSWORD "00000000"

// MQTT Broker settings
#define MQTT_BROKER "192.168.27.189"
#define MQTT_PORT 1883
#define MQTT_TOPIC_TEMP "home/temperature"
#define MQTT_TOPIC_HUM "home/humidity"
#define MQTT_TOPIC_AIR "home/airquality"
#define MQTT_TOPIC_ALERT "home/alerts"

#define LED_PIN D2
#define LED_PIN_2 D4
#define LED_PIN_3 D6 // New LED for air quality sensor
#define DHT_PIN D5
#define DHT_TYPE DHT11
#define AIR_SENSOR_PIN A0 // Pin for air quality sensor

#define TEMP_THRESHOLD 30 
#define HUM_THRESHOLD 60
#define AIR_QUALITY_THRESHOLD 300 // Threshold for air quality sensor

DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient espClient;
PubSubClient client(espClient);

float temperature = 0.0;
float humidity = 0.0;
int airQualityValue = 0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT Broker...");
    if (client.connect("ESP8266Client")) {
      Serial.println(" Connected!");
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT); // Initialize new LED pin
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(LED_PIN_3, LOW); // Set new LED pin to LOW

  dht.begin();
  connectToWiFi();
  client.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  airQualityValue = analogRead(AIR_SENSOR_PIN); // Read air quality value

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    delay(5000);
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C  Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Air Quality Value: ");
  Serial.println(airQualityValue);

  // Publish sensor data to MQTT topics
  client.publish(MQTT_TOPIC_TEMP, String(temperature).c_str(), true);
  client.publish(MQTT_TOPIC_HUM, String(humidity).c_str(), true);
  client.publish(MQTT_TOPIC_AIR, String(airQualityValue).c_str(), true);

  // Handle temperature alert
  if (temperature > TEMP_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
    client.publish(MQTT_TOPIC_ALERT, "High temperature detected!", true);
    Serial.println("High temperature detected! FAN TURNED ON.");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("Temperature is normal. FAN TURNED OFF.");
  }

  // Handle humidity alert
  if (humidity > HUM_THRESHOLD) {
    digitalWrite(LED_PIN_2, HIGH);
    client.publish(MQTT_TOPIC_ALERT, "High humidity detected!", true);
    Serial.println("High humidity detected! FAN TURNED ON.");
  } else {
    digitalWrite(LED_PIN_2, LOW);
    Serial.println("Humidity is normal. FAN TURNED OFF.");
  }

  // Handle air quality alert
  if (airQualityValue > AIR_QUALITY_THRESHOLD) {
    digitalWrite(LED_PIN_3, HIGH);
    client.publish(MQTT_TOPIC_ALERT, "Poor air quality detected!", true);
    Serial.println("Poor air quality detected! FAN TURNED ON.");
  } else {
    digitalWrite(LED_PIN_3, LOW);
    Serial.println("Air quality is good. FAN TURNED OFF.");
  }

  delay(5000);
}