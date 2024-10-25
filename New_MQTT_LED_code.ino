#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>

// home credentials
//const char* ssid = "slt_filber";           
//const char* password = "########";

// My Ohne credentials
const char* ssid = "Note8";           
const char* password = "12345678@";

// MQTT broker settings
const char* mqtt_broker = "broker.emqx.io";
const char* topic = "lakindu/esp32";
const char* mqtt_username = "MasterLWA";
const char* mqtt_password = "SLIITLAB";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// MPU6050
MPU6050 mpu;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// LED Pin (Built-in LED of ESP32, usually GPIO 2)
const int ledPin = 2; // GPIO 2 for built-in LED

// Buffer settings for accelerometer data
#define SAMPLE_SIZE 10
#define MOVING_AVERAGE_SIZE 5
float accXBuffer[SAMPLE_SIZE];
float accYBuffer[SAMPLE_SIZE];
float accZBuffer[SAMPLE_SIZE];
float accXMovingAvg[MOVING_AVERAGE_SIZE] = {0};
float accYMovingAvg[MOVING_AVERAGE_SIZE] = {0};
float accZMovingAvg[MOVING_AVERAGE_SIZE] = {0};
int bufferIndex = 0;  // Index for circular buffer
int movingAvgIndex = 0; // Index for moving average buffer

// Threshold for filtering noise
const float THRESHOLD = 1000; // Adjust based on your noise levels

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize LED
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);  // Turn LED off initially

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1); // Halt if the sensor isn't connected
    }

    // Connect to Wi-Fi
    connectToWiFi();
    
    // Connect to MQTT broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    connectToMQTT();
}

void loop() {
    client.loop(); // Maintain MQTT connection
    processSensorData(); // Read sensor data and send to MQTT
}

// Connect to Wi-Fi
void connectToWiFi() {
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" connected!");
}

// Connect to the MQTT broker
void connectToMQTT() {
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
            blinkLED(3); // Blink LED 3 times on successful connection
            client.publish(topic, "Hi, I'm ESP32 ^^");
            client.subscribe(topic);
        } else {
            Serial.print("Failed with state: ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

// Process MPU6050 sensor data and publish to MQTT
void processSensorData() {
    static unsigned long lastSendTime = 0;  // Track last send time
    unsigned long currentTime = millis();

    // Read sensor data
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    // Simple low-pass filter
    const float alpha = 0.5; // Smoothing factor
    static float filteredAccX = 0, filteredAccY = 0, filteredAccZ = 0;

    // Apply the low-pass filter
    filteredAccX = alpha * accX + (1 - alpha) * filteredAccX;
    filteredAccY = alpha * accY + (1 - alpha) * filteredAccY;
    filteredAccZ = alpha * accZ + (1 - alpha) * filteredAccZ;

    // Update moving average buffers
    accXMovingAvg[movingAvgIndex] = filteredAccX;
    accYMovingAvg[movingAvgIndex] = filteredAccY;
    accZMovingAvg[movingAvgIndex] = filteredAccZ;

    // Update index for circular buffer
    movingAvgIndex = (movingAvgIndex + 1) % MOVING_AVERAGE_SIZE;

    // Calculate moving averages
    float avgAccX = 0, avgAccY = 0, avgAccZ = 0;
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        avgAccX += accXMovingAvg[i];
        avgAccY += accYMovingAvg[i];
        avgAccZ += accZMovingAvg[i];
    }
    avgAccX /= MOVING_AVERAGE_SIZE;
    avgAccY /= MOVING_AVERAGE_SIZE;
    avgAccZ /= MOVING_AVERAGE_SIZE;

    // Check for significant movement and send data if conditions are met
    if ((currentTime - lastSendTime >= 100) && 
        (abs(avgAccX) > THRESHOLD || abs(avgAccY) > THRESHOLD || abs(avgAccZ) > THRESHOLD)) { // Send data every 100ms
        // Create payload
        String payload = String("{\"accX\": ") + avgAccX + 
                         ", \"accY\": " + avgAccY + 
                         ", \"accZ\": " + avgAccZ + 
                         ", \"gyroX\": " + gyroX + 
                         ", \"gyroY\": " + gyroY + 
                         ", \"gyroZ\": " + gyroZ + "}";

        Serial.print("Sending data: ");
        Serial.println(payload); // Print data being sent to the serial monitor

        // Publish data to MQTT
        if (client.publish("sensor/mpu6050", payload.c_str())) {
            Serial.println("Data sent successfully!");
            digitalWrite(ledPin, HIGH); // Turn on LED to indicate successful data send
            delay(100);                  // Keep LED on for a short period
            digitalWrite(ledPin, LOW);  // Turn off LED
        } else {
            Serial.println("Failed to send data.");
        }

        lastSendTime = currentTime; // Update last send time
    }
}

// Blink LED on successful connection
void blinkLED(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(ledPin, HIGH); // LED ON
        delay(200);
        digitalWrite(ledPin, LOW); // LED OFF
        delay(200);
    }
}

// Callback for incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}
