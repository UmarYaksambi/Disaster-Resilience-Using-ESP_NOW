// ===================== COMMON DEFINITIONS =====================
#include <ESP8266WiFi.h>
#include <espnow.h> // For ESP8266, this should be the correct include
#include <FS.h>
#include <Wire.h> // For I2C communication (MPU6050)
// #include <ArduinoJson.h> // Not used in this version, can be removed if not planned
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h> // Added for GPS

#define ROLE_SENDER
// #define ROLE_REBROADCASTER
// #define ROLE_RECEIVER

#define DHTPIN 2      // GPIO2 (D4 on NodeMCU)
#define DHTTYPE DHT22
#define ACC_THRESHOLD 1.5 // Threshold in G's
#define TTL_INIT 3

// GPS Pins (using SoftwareSerial)
#define GPS_RX_PIN 14 // GPIO14 (D5 on NodeMCU) - Connect to GPS TX
#define GPS_TX_PIN 12 // GPIO12 (D6 on NodeMCU) - Connect to GPS RX (often not needed for basic location)

DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // Define SoftwareSerial for GPS

struct SOSPacket {
  char message_id[32];
  uint8_t sender_mac[6];
  uint32_t timestamp;
  double lat;
  double lon;
  bool earthquake;
  float motion;
  bool gas_alert;
  float temperature;
  uint8_t priority;
  uint8_t ttl;
};

// --- Corrected Seen IDs Logic (Circular Buffer) ---
#define MAX_SEEN_IDS 10
String seen_ids[MAX_SEEN_IDS];
int seen_ids_index = 0;
int current_seen_ids_count = 0; // Number of actual elements in the buffer

bool alreadySeen(const char* id) {
  for (int i = 0; i < current_seen_ids_count; i++) {
    if (seen_ids[i] == id) {
      return true;
    }
  }
  return false;
}

void markSeen(const char* id) {
  seen_ids[seen_ids_index] = id; // String constructor makes a copy
  seen_ids_index = (seen_ids_index + 1) % MAX_SEEN_IDS;
  if (current_seen_ids_count < MAX_SEEN_IDS) {
    current_seen_ids_count++;
  }
}
// --- End of Corrected Seen IDs Logic ---

void initWiFiAndEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Disconnect from any previous AP

  if (esp_now_init() != 0) { // ESP-NOW Arduino wrapper for ESP8266 returns 0 on success
    Serial.println("ESP-NOW initialization failed!");
    ESP.restart(); // Restart if critical init fails
    // while (true); // Unreachable after ESP.restart()
  } else {
    Serial.println("ESP-NOW initialized successfully.");
  }
  // For ESP8266, esp_now_set_self_role() can be used to set role explicitly if needed.
  // Roles: 0 (ESP_NOW_ROLE_IDLE), 1 (ESP_NOW_ROLE_CONTROLLER), 2 (ESP_NOW_ROLE_SLAVE), 3 (ESP_NOW_ROLE_COMBO)
  // Default after init should be okay for sending/receiving with callbacks registered.
  // e.g., esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
}

void storePacketToSPIFFS(const SOSPacket& pkt) {
  File file = SPIFFS.open("/packets.dat", "a"); // Append mode
  if (!file) {
    Serial.println("Failed to open packets.dat for appending.");
    return;
  }
  if (file.write((uint8_t*)&pkt, sizeof(pkt))) {
    // Serial.println("Packet stored to SPIFFS."); // Optional: for verbose logging
  } else {
    Serial.println("SPIFFS write failed.");
  }
  file.close();
}

void printPacket(const SOSPacket& pkt) {
  Serial.printf("ID: %s | MAC: %02X:%02X:%02X:%02X:%02X:%02X | TS: %lu | Lat: %.4f | Lon: %.4f | EQ: %d | Motion: %.2f | Gas: %d | Temp: %.2fC | Prio: %d | TTL: %d\n",
                pkt.message_id,
                pkt.sender_mac[0], pkt.sender_mac[1], pkt.sender_mac[2], pkt.sender_mac[3], pkt.sender_mac[4], pkt.sender_mac[5],
                pkt.timestamp, pkt.lat, pkt.lon,
                pkt.earthquake, pkt.motion, pkt.gas_alert, pkt.temperature,
                pkt.priority, pkt.ttl);
}

// ===================== SENDER =====================
#ifdef ROLE_SENDER
void setup() {
  Serial.begin(115200);
  Serial.println("SENDER initializing...");

  gpsSerial.begin(9600); // Initialize SoftwareSerial for GPS

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed. Formatting...");
    if (SPIFFS.format()) {
        Serial.println("SPIFFS Formatted. Please reboot.");
    } else {
        Serial.println("SPIFFS Format Failed.");
    }
    while(1) delay(100); // Halt
  } else {
    Serial.println("SPIFFS Mounted.");
  }
  
  Wire.begin(); // Initialize I2C (default SDA=GPIO4, SCL=GPIO5)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check connections.");
    while (1) {
      delay(10);
    }
  } else {
    Serial.println("MPU6050 Found.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Example: Set accelerometer range
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);   // Example: Set gyro range
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Example: Set filter bandwidth
  }

  dht.begin();
  Serial.println("DHT sensor initialized.");

  pinMode(0, INPUT_PULLUP); // SOS button (GPIO0, D3 on NodeMCU - typically FLASH button)
  Serial.println("SOS Button (GPIO0) configured.");

  initWiFiAndEspNow(); // Initialize WiFi and ESP-NOW
  Serial.println("Sender setup complete. Waiting for triggers...");
}

void loop() {
  // Process GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
        // Optional: display GPS data when new sentence is complete
        // if (gps.location.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated()) {
        //   Serial.print("GPS Location: Lat="); Serial.print(gps.location.lat(), 6);
        //   Serial.print(" Lng="); Serial.println(gps.location.lng(), 6);
        // }
    }
  }

  sensors_event_t acc, gyro, temp_mpu; // MPU's temp is different from DHT temp
  mpu.getEvent(&acc, &gyro, &temp_mpu);

  // Normalize accelerometer readings to G's if they are in m/s^2
  // SENSORS_GRAVITY_STANDARD is approx 9.80665F
  float acc_x_g = acc.acceleration.x / SENSORS_GRAVITY_STANDARD;
  float acc_y_g = acc.acceleration.y / SENSORS_GRAVITY_STANDARD;
  float acc_z_g = acc.acceleration.z / SENSORS_GRAVITY_STANDARD;
  float motion = sqrt(pow(acc_x_g, 2) + pow(acc_y_g, 2) + pow(acc_z_g, 2));
  
  bool earthquake = motion > ACC_THRESHOLD;
  
  float temperature = dht.readTemperature(); // Read from DHT22
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature from DHT sensor!");
    temperature = -999.0; // Or some other error indicator (e.g. previous valid reading)
  }

  bool gas_alert = false; // Placeholder for gas sensor logic (e.g., analogRead(A0) > threshold)

  // Check for trigger conditions: earthquake, gas alert, or SOS button press
  if (earthquake || gas_alert || digitalRead(0) == LOW) {
    Serial.println("SOS Condition Triggered!");
    SOSPacket pkt;

    // Create a unique message ID: MAC_timestamp
    String mac_addr_str = WiFi.macAddress();
    // mac_addr_str.replace(":", ""); // Optional: remove colons for shorter ID if needed
    snprintf(pkt.message_id, sizeof(pkt.message_id), "%s_%lu", mac_addr_str.c_str(), millis());

    uint8_t mac[6];
    WiFi.macAddress(mac); // Get MAC address as byte array
    memcpy(pkt.sender_mac, mac, 6);

    pkt.timestamp = millis(); // Using millis() for timestamp as time() might not be set
                              // For Unix time, an RTC or NTP sync is needed.

    if (gps.location.isValid()) {
      pkt.lat = gps.location.lat();
      pkt.lon = gps.location.lng();
    } else {
      pkt.lat = 0.0; // Indicate invalid or unavailable GPS data
      pkt.lon = 0.0;
      Serial.println("GPS data invalid or unavailable for this packet.");
    }

    pkt.earthquake = earthquake;
    pkt.motion = motion; // Motion in G's
    pkt.gas_alert = gas_alert;
    pkt.temperature = temperature;
    pkt.priority = 1; // High priority for SOS
    pkt.ttl = TTL_INIT;

    // Send packet via ESP-NOW (broadcast to NULL peer)
    // uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Alternative for broadcast
    int send_result = esp_now_send(NULL, (uint8_t*)&pkt, sizeof(pkt)); // For ESP8266, returns int (0 for success)

    if (send_result == 0) { // 0 means success for ESP8266
      Serial.println("SOS Packet sent successfully via ESP-NOW.");
    } else {
      Serial.print("Error sending ESP-NOW packet. Code: ");
      Serial.println(send_result);
    }

    storePacketToSPIFFS(pkt); // Store a local copy
    printPacket(pkt);         // Print to local Serial for confirmation

    // Debounce or prevent immediate re-trigger if button is held
    if (digitalRead(0) == LOW) {
        Serial.println("SOS Button pressed. Waiting for release or delay...");
        unsigned long pressStartTime = millis();
        while(digitalRead(0) == LOW && (millis() - pressStartTime < 2000)) { // Wait up to 2s or button release
            delay(50);
        }
        delay(1000); // Additional delay after release or timeout
    }
  }

  delay(1000); // Check sensors every 1 second (adjust as needed)
}
#endif

// ===================== REBROADCASTER =====================
#ifdef ROLE_REBROADCASTER
// ESP-NOW Receive Callback for ESP8266
// Note: ESP8266 recv_cb has (mac_addr, data, len)
void onDataRecvRebroadcast(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  if (len != sizeof(SOSPacket)) {
    Serial.println("Received packet with incorrect size.");
    return;
  }
  SOSPacket pkt;
  memcpy(&pkt, data, sizeof(SOSPacket));

  if (!alreadySeen(pkt.message_id)) {
    markSeen(pkt.message_id);
    Serial.print("Rebroadcasting: ");
    printPacket(pkt);

    if (pkt.ttl > 0) {
      pkt.ttl--; // Decrement Time To Live
      
      int send_result = esp_now_send(NULL, (uint8_t *)&pkt, sizeof(pkt)); // ESP8266 returns int
      if (send_result == 0) { // 0 is success
        // Serial.println("Packet rebroadcast successfully."); // Optional
      } else {
        Serial.print("Error rebroadcasting packet. Code: ");
        Serial.println(send_result);
      }
    } else {
      Serial.println("Packet TTL expired, not rebroadcasting.");
    }
  } else {
    Serial.print("Packet already seen, not rebroadcasting: ");
    Serial.println(pkt.message_id);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("REBROADCASTER initializing...");

  initWiFiAndEspNow();
  // For ESP8266, esp_now_register_recv_cb returns int (0 for success)
  if (esp_now_register_recv_cb(onDataRecvRebroadcast) != 0) {
    Serial.println("Failed to register ESP-NOW receive callback.");
    ESP.restart();
  } else {
    Serial.println("ESP-NOW receive callback registered.");
  }
  Serial.println("Rebroadcaster setup complete. Waiting for packets...");
}

void loop() {
  delay(1000); 
}
#endif

// ===================== RECEIVER =====================
#ifdef ROLE_RECEIVER
// ESP-NOW Receive Callback for ESP8266
void onDataRecvStore(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  if (len != sizeof(SOSPacket)) {
    Serial.println("Received packet with incorrect size.");
    return;
  }
  SOSPacket pkt;
  memcpy(&pkt, data, sizeof(SOSPacket));

  if (!alreadySeen(pkt.message_id)) {
    markSeen(pkt.message_id);
    Serial.print("Receiver Processing New Packet: ");
    printPacket(pkt);
    storePacketToSPIFFS(pkt); // Store the received packet
  } else {
    Serial.print("Receiver: Packet already seen/stored: ");
    Serial.println(pkt.message_id);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("RECEIVER initializing...");

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed. Formatting...");
    if (SPIFFS.format()) {
        Serial.println("SPIFFS Formatted. Please reboot.");
    } else {
        Serial.println("SPIFFS Format Failed.");
    }
    while(1) delay(100); // Halt
  } else {
    Serial.println("SPIFFS Mounted.");
  }

  initWiFiAndEspNow();
  // For ESP8266, esp_now_register_recv_cb returns int (0 for success)
  if (esp_now_register_recv_cb(onDataRecvStore) != 0) {
    Serial.println("Failed to register ESP-NOW receive callback.");
    ESP.restart();
  } else {
    Serial.println("ESP-NOW receive callback registered.");
  }
  Serial.println("Receiver setup complete. Waiting for packets...");
}

void loop() {
  delay(1000);
}
#endif
