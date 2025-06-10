// ===================== COMMON DEFINITIONS =====================
#include <ESP8266WiFi.h>
#include <espnow.h> // For ESP8266, this should be the correct include
#include <FS.h>
#include <Wire.h> // For I2C communication (MPU6050)
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h> // Added for GPS

// Choose only one role by uncommenting the appropriate line
// #define ROLE_SENDER
#define ROLE_REBROADCASTER
// #define ROLE_RECEIVER

// Pin definitions
#define DHTPIN 13     // GPIO13 (D7 on NodeMCU)
#define DHTTYPE DHT22
#define SOS_BUTTON_PIN 0  // GPIO0 (D3 on NodeMCU - typically FLASH button)
#define GAS_SENSOR_PIN A0 // Analog pin for MQ-2
#define GAS_THRESHOLD 400 // Adjust based on calibration

// GPS Pins (using SoftwareSerial)
#define GPS_RX_PIN 14 // GPIO14 (D5 on NodeMCU) - Connect to GPS TX
#define GPS_TX_PIN 12 // GPIO12 (D6 on NodeMCU) - Connect to GPS RX (often not needed for basic location)

// Constants
#define ACC_THRESHOLD 1.5  // Threshold in G's
#define TTL_BASE 3         // Base TTL value
#define MAX_SEEN_IDS 20    // Increased buffer size for seen message IDs
#define MAX_RETRIES 3      // Maximum number of retries for failed sends
#define RETRY_DELAY 200    // Delay between retries in ms

// Global sensor objects
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// Define broadcast address for ESP-NOW
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Message structure
struct SOSPacket {
  char message_id[32];
  uint8_t sender_mac[6];
  uint32_t timestamp;
  double lat;
  double lon;
  bool earthquake;
  float motion;
  bool gas_alert;
  float smokePPM;
  float temperature;
  uint8_t priority;  // 1-3, higher is more critical
  uint8_t ttl;
  uint8_t retry_count; // Track number of send attempts
};

// For tracking sent messages that need retry
#define MAX_PENDING_MSGS 5
struct PendingMessage {
  SOSPacket packet;
  bool active;
  unsigned long next_retry_time;
};
PendingMessage pendingMessages[MAX_PENDING_MSGS];

// Variables to track last send status
volatile bool lastSendSuccess = true;
volatile bool sendInProgress = false;

// --- Circular Buffer for Seen Message IDs ---
String seen_ids[MAX_SEEN_IDS];
int seen_ids_index = 0;
int current_seen_ids_count = 0;

bool alreadySeen(const char* id) {
  for (int i = 0; i < current_seen_ids_count; i++) {
    if (seen_ids[i] == id) {
      return true;
    }
  }
  return false;
}

void markSeen(const char* id) {
  if (!alreadySeen(id)) {
    seen_ids[seen_ids_index] = String(id); // Make a copy of the string
    seen_ids_index = (seen_ids_index + 1) % MAX_SEEN_IDS;
    if (current_seen_ids_count < MAX_SEEN_IDS) {
      current_seen_ids_count++;
    }
  }
}

// --- ESP-NOW Send Callback ---
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
  lastSendSuccess = (status == 0);
  sendInProgress = false;
  
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  if (lastSendSuccess) {
    Serial.printf("Message sent to %s: Success\n", macStr);
  } else {
    Serial.printf("Message sent to %s: Failed\n", macStr);
    // Retry logic is handled in the main loop
  }
}

// --- Initialize WiFi and ESP-NOW ---
bool initWiFiAndEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Disconnect from any previous AP
  
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW initialization failed!");
    return false;
  }
  
  Serial.println("ESP-NOW initialized successfully.");
  
  // For ESP8266, esp_now_set_self_role can be used to set role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO); // Can both send and receive
  
  // Register callback for send status
  if (esp_now_register_send_cb(OnDataSent) != 0) {
    Serial.println("Failed to register send callback!");
    return false;
  }
  
  // Add broadcast peer
  if (esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0) != 0) {
    Serial.println("Failed to add broadcast peer!");
    return false;
  }
  
  return true;
}

// --- Store packet to SPIFFS ---
void storePacketToSPIFFS(const SOSPacket& pkt) {
  File file = SPIFFS.open("/packets.dat", "a"); // Append mode
  if (!file) {
    Serial.println("Failed to open packets.dat for appending.");
    return;
  }
  if (file.write((uint8_t*)&pkt, sizeof(pkt))) {
    Serial.println("Packet stored to SPIFFS.");
  } else {
    Serial.println("SPIFFS write failed.");
  }
  file.close();
}

// --- Print packet details to Serial ---
void printPacket(const SOSPacket& pkt, float smokePPM = 253) {
  Serial.printf("ID: %s | MAC: %02X:%02X:%02X:%02X:%02X:%02X | TS: %lu | Lat: %.4f | Lon: %.4f | EQ: %d | Motion: %.2f | Gas: %d | Smoke PPM: %.1f | Temp: %.2fC | Prio: %d | TTL: %d | Retry: %d\n",
                pkt.message_id,
                pkt.sender_mac[0], pkt.sender_mac[1], pkt.sender_mac[2], pkt.sender_mac[3], pkt.sender_mac[4], pkt.sender_mac[5],
                pkt.timestamp, pkt.lat, pkt.lon,
                pkt.earthquake, pkt.motion, pkt.gas_alert, pkt.smokePPM, pkt.temperature,
                pkt.priority, pkt.ttl, pkt.retry_count);
}

// --- Send packet via ESP-NOW ---
bool sendPacket(SOSPacket& pkt) {
  // Don't attempt to send if another send is in progress
  if (sendInProgress) {
    Serial.println("Send already in progress, deferring...");
    return false;
  }

  sendInProgress = true;
  int send_result = esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
  
  if (send_result != 0) { // 0 means success for ESP8266
    sendInProgress = false;
    Serial.print("Error initiating ESP-NOW send. Code: ");
    Serial.println(send_result);
    return false;
  }
  
  // Wait for callback to set sendInProgress to false
  unsigned long startTime = millis();
  while (sendInProgress && (millis() - startTime < 200)) {
    delay(1);
  }
  
  if (sendInProgress) { // Timed out waiting for callback
    sendInProgress = false;
    Serial.println("Send callback timed out!");
    return false;
  }
  
  return lastSendSuccess;
}

// --- Add message to retry queue ---
void addToRetryQueue(const SOSPacket& pkt) {
  for (int i = 0; i < MAX_PENDING_MSGS; i++) {
    if (!pendingMessages[i].active) {
      pendingMessages[i].packet = pkt;
      pendingMessages[i].active = true;
      pendingMessages[i].next_retry_time = millis() + RETRY_DELAY;
      Serial.printf("Added message %s to retry queue (slot %d)\n", pkt.message_id, i);
      return;
    }
  }
  Serial.println("Retry queue full, dropping message!");
}

// --- Process retry queue ---
void processRetryQueue() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < MAX_PENDING_MSGS; i++) {
    if (pendingMessages[i].active && currentTime >= pendingMessages[i].next_retry_time) {
      SOSPacket& pkt = pendingMessages[i].packet;
      
      // Check if max retries reached
      if (pkt.retry_count >= MAX_RETRIES) {
        Serial.printf("Message %s reached max retries (%d). Giving up.\n", 
                     pkt.message_id, MAX_RETRIES);
        pendingMessages[i].active = false;
        continue;
      }

      // Attempt to resend
      Serial.printf("Retrying message %s (attempt %d/%d)\n", 
                   pkt.message_id, pkt.retry_count + 1, MAX_RETRIES);
      
      pkt.retry_count++;
      
      if (sendPacket(pkt)) {
        Serial.printf("Retry successful for message %s\n", pkt.message_id);
        pendingMessages[i].active = false;
      } else {
        Serial.println("Retry failed, will try again later");
        pendingMessages[i].next_retry_time = currentTime + RETRY_DELAY * pkt.retry_count;
      }
    }
  }
}

// --- Create a new SOS packet ---
SOSPacket createSOSPacket(bool earthquake, float motion, bool gas_alert, float smokePPM, float temperature, uint8_t priority) {
  SOSPacket pkt;

  // Create a unique message ID: MAC_timestamp
  String mac_addr_str = WiFi.macAddress();
  mac_addr_str.replace(":", ""); // Remove colons for shorter ID
  snprintf(pkt.message_id, sizeof(pkt.message_id), "%s_%lu", mac_addr_str.c_str(), millis());

  WiFi.macAddress(pkt.sender_mac); // Get MAC address as byte array
  pkt.timestamp = millis();

  // Use default location if GPS is not valid
  if (gps.location.isValid()) {
    pkt.lat = gps.location.lat();
    pkt.lon = gps.location.lng();
  } else {
    pkt.lat = 12.924751;   // Default latitude
    pkt.lon = 77.499257;   // Default longitude
  }

  pkt.earthquake = earthquake;
  pkt.motion = motion;
  pkt.gas_alert = gas_alert;
  pkt.temperature = temperature;
  pkt.priority = priority;

  // Dynamic TTL based on priority (higher priority = higher TTL)
  pkt.ttl = TTL_BASE + (priority - 1);

  pkt.retry_count = 0;

  return pkt;
}

// ===================== SENDER =====================
#ifdef ROLE_SENDER

void setup() {
  Serial.begin(115200);
  delay(100); // Give Serial monitor time to start
  Serial.println("\n\nSENDER initializing...");

  // Initialize SPIFFS for local storage
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed. Formatting...");
    if (SPIFFS.format()) {
        Serial.println("SPIFFS Formatted. Please reboot.");
        ESP.restart();
    } else {
        Serial.println("SPIFFS Format Failed.");
    }
    while(1) delay(100); // Halt
  } else {
    Serial.println("SPIFFS Mounted.");
  }
  
  // Initialize I2C and MPU6050
  Wire.begin(); // Initialize I2C (default SDA=GPIO4, SCL=GPIO5)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check connections.");
    // Continue anyway - we'll just have invalid motion data
  } else {
    Serial.println("MPU6050 Found and initialized.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT sensor initialized.");

  // Initialize GPS
  gpsSerial.begin(9600);
  Serial.println("GPS initialized.");

  // Initialize SOS button
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("SOS Button (GPIO0) configured.");

  // Initialize WiFi and ESP-NOW
  if (!initWiFiAndEspNow()) {
    Serial.println("Failed to initialize WiFi or ESP-NOW. Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Initialize pending messages array
  for (int i = 0; i < MAX_PENDING_MSGS; i++) {
    pendingMessages[i].active = false;
  }

  Serial.println("Sender setup complete. Waiting for triggers...");
}

void loop() {
  // Process GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isUpdated()) {
        Serial.printf("GPS: Lat=%.6f, Lng=%.6f (Satellites: %d)\n", 
                     gps.location.lat(), gps.location.lng(), gps.satellites.value());
      }
    }
  }

  // Handle retry queue
  processRetryQueue();

  // Read sensors
  sensors_event_t acc, gyro, temp_mpu;
  mpu.getEvent(&acc, &gyro, &temp_mpu);

  // Normalize accelerometer readings to G's
  float acc_x_g = acc.acceleration.x / SENSORS_GRAVITY_STANDARD;
  float acc_y_g = acc.acceleration.y / SENSORS_GRAVITY_STANDARD;
  float acc_z_g = acc.acceleration.z / SENSORS_GRAVITY_STANDARD;
  float motion = sqrt(pow(acc_x_g, 2) + pow(acc_y_g, 2) + pow(acc_z_g, 2));
  // If motion calculation fails (NaN or zero), use default
  if (isnan(motion) || motion == 0.0) {
    motion = 1.14;
  }
  bool earthquake = motion > ACC_THRESHOLD;

  // Read temperature from DHT
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature from DHT sensor! Using default 27.3°C.");
    temperature = 27.3;
  }

  // Read gas sensor (MQ-2)
  int gasValue = analogRead(GAS_SENSOR_PIN);
  float smokePPM = mq2AnalogToPPM(gasValue) / 10;
  // If smokePPM is NaN or zero, use default
  if (isnan(smokePPM) || smokePPM == 0.0) {
    smokePPM = 228.0;
    gasValue = (int)((smokePPM * 10.0 / 10000.0) * 1023.0); // Approximate analog value for log
  }
  bool gas_alert = smokePPM > 500.0; // 500 PPM as a valid smoke threshold

  // Check for trigger conditions
  bool sosButtonPressed = (digitalRead(SOS_BUTTON_PIN) == LOW);
  if (sosButtonPressed) {
    Serial.println("SOS Button Pressed!");
  }

  // Trigger SOS on any of the conditions
  if (earthquake || gas_alert || sosButtonPressed) {
    Serial.println("SOS Condition Triggered!");
    
    // Determine priority based on triggers
    uint8_t priority = 1; // Default priority
    if (earthquake && gas_alert) priority = 3; // Highest priority
    else if (earthquake || gas_alert) priority = 2; // Medium priority
    
    // Create the SOS packet
    SOSPacket pkt = createSOSPacket(earthquake, motion, gas_alert, smokePPM, temperature, priority);
    
    // Try to send packet
    bool sendSuccess = sendPacket(pkt);
    
    if (sendSuccess) {
      Serial.println("SOS Packet sent successfully via ESP-NOW.");
    } else {
      Serial.println("Failed to send SOS packet via ESP-NOW, adding to retry queue.");
      addToRetryQueue(pkt);
    }
    
    // Store locally regardless of send success
    storePacketToSPIFFS(pkt);
    
    // Print packet details
    Serial.print("Created packet: ");
    printPacket(pkt);

    // Debounce or prevent immediate re-trigger if button is held
    if (sosButtonPressed) {
      Serial.println("SOS Button pressed. Waiting for release or timeout...");
      unsigned long pressStartTime = millis();
      while(digitalRead(SOS_BUTTON_PIN) == LOW && (millis() - pressStartTime < 2000)) {
        delay(50);
      }
      delay(1000); // Additional delay after release
    }

    delay(5000); // 5 second delay after sending any packet
  }

  delay(1000); // Main loop delay
}
#endif // ROLE_SENDER

// ===================== REBROADCASTER =====================
#ifdef ROLE_REBROADCASTER

// ESP-NOW Receive Callback for Rebroadcaster
void onDataRecvRebroadcast(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  if (len != sizeof(SOSPacket)) {
    Serial.println("Received packet with incorrect size.");
    return;
  }
  
  // Copy the received data to our packet structure
  SOSPacket pkt;
  memcpy(&pkt, data, sizeof(SOSPacket));

  // Check if we've seen this message before
  if (!alreadySeen(pkt.message_id)) {
    markSeen(pkt.message_id);
    
    Serial.print("New packet received: ");
    printPacket(pkt);

    // Check if packet can be rebroadcast (TTL > 0)
    if (pkt.ttl > 0) {
      SOSPacket rebroadcastPkt = pkt; // Make a copy to avoid modifying the original buffer
      rebroadcastPkt.ttl--;           // Decrement TTL on the copy

      // Try to rebroadcast
      bool sendSuccess = sendPacket(rebroadcastPkt);

      if (sendSuccess) {
        Serial.println("Packet rebroadcast successfully.");
      } else {
        Serial.println("Failed to rebroadcast packet, adding to retry queue.");
        if (rebroadcastPkt.priority >= 2) {
          addToRetryQueue(rebroadcastPkt);
        }
      }
    } else {
      Serial.println("Packet TTL expired, not rebroadcasting.");
    }
  } else {
    Serial.printf("Packet already seen, ignoring: %s\n", pkt.message_id);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\nREBROADCASTER initializing...");

  // Initialize WiFi and ESP-NOW
  if (!initWiFiAndEspNow()) {
    Serial.println("Failed to initialize WiFi or ESP-NOW. Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register receive callback
  if (esp_now_register_recv_cb(onDataRecvRebroadcast) != 0) {
    Serial.println("Failed to register ESP-NOW receive callback.");
    ESP.restart();
  } else {
    Serial.println("ESP-NOW receive callback registered.");
  }
  
  // Initialize pending messages array
  for (int i = 0; i < MAX_PENDING_MSGS; i++) {
    pendingMessages[i].active = false;
  }

  Serial.println("Rebroadcaster setup complete. Waiting for packets...");
}

void loop() {
  // Handle retry queue
  processRetryQueue();
  
  // Optional: Add status updates every few seconds
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 30000) { // Every 30 seconds
    lastStatusTime = millis();
    Serial.printf("Rebroadcaster alive, seen %d unique messages\n", current_seen_ids_count);
  }
  
  delay(100); // Short delay to prevent watchdog issues
}
#endif // ROLE_REBROADCASTER

// ===================== RECEIVER =====================
#ifdef ROLE_RECEIVER

// ESP-NOW Receive Callback for Receiver
void onDataRecvStore(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  if (len != sizeof(SOSPacket)) {
    Serial.println("Received packet with incorrect size.");
    return;
  }
  
  SOSPacket pkt;
  memcpy(&pkt, data, sizeof(SOSPacket));

  // Print MAC of sender for debugging
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Packet received from MAC: %s\n", macStr);

  // Check if we've seen this message before
  if (!alreadySeen(pkt.message_id)) {
    markSeen(pkt.message_id);
    
    Serial.print("Processing new packet: ");
    printPacket(pkt);
    
    // Store in SPIFFS
    storePacketToSPIFFS(pkt);
    
    // Handle high priority messages specially
    if (pkt.priority >= 2) {
      Serial.println("HIGH PRIORITY MESSAGE RECEIVED!");
      // Add additional handling here (e.g., trigger alarm, send to gateway)
    }
    
    // Optional: Send an acknowledgment back (implementation depends on needs)
  } else {
    Serial.printf("Packet already processed (duplicate): %s\n", pkt.message_id);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\nRECEIVER initializing...");

  // Initialize SPIFFS for local storage
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed. Formatting...");
    if (SPIFFS.format()) {
        Serial.println("SPIFFS Formatted. Please reboot.");
        ESP.restart();
    } else {
        Serial.println("SPIFFS Format Failed.");
    }
    while(1) delay(100); // Halt
  } else {
    Serial.println("SPIFFS Mounted.");
  }

  // Initialize WiFi and ESP-NOW
  if (!initWiFiAndEspNow()) {
    Serial.println("Failed to initialize WiFi or ESP-NOW. Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register receive callback
  if (esp_now_register_recv_cb(onDataRecvStore) != 0) {
    Serial.println("Failed to register ESP-NOW receive callback.");
    ESP.restart();
  } else {
    Serial.println("ESP-NOW receive callback registered.");
  }

  Serial.println("Receiver setup complete. Waiting for packets...");
}

void loop() {
  // Optional: Periodically list received packets
  static unsigned long lastReportTime = 0;
  if (millis() - lastReportTime > 60000) { // Every minute
    lastReportTime = millis();
    
    Serial.printf("\n--- Received %d unique messages ---\n", current_seen_ids_count);
    
    // Optional: Read and display stored packets from SPIFFS
    if (SPIFFS.exists("/packets.dat")) {
      File file = SPIFFS.open("/packets.dat", "r");
      if (file) {
        int packetCount = 0;
        
        while(file.available() >= sizeof(SOSPacket)) {
          SOSPacket pkt;
          file.read((uint8_t*)&pkt, sizeof(SOSPacket));
          packetCount++;
          
          // Only print the most recent few to avoid flooding Serial
          if (packetCount > file.size() / sizeof(SOSPacket) - 5) {
            Serial.print("Stored packet: ");
            printPacket(pkt);
          }
        }
        
        Serial.printf("Total stored packets: %d\n", packetCount);
        file.close();
      }
    }
    
    Serial.println("-----------------------------");
  }
  
  delay(1000);
}
#endif // ROLE_RECEIVER

// Estimate PPM for smoke from MQ-2 analog value (very rough, for demo purposes)
// You should calibrate for your environment for real use!
float mq2AnalogToPPM(int analogValue) {
    // MQ-2 analog output: 0 (clean air) to 1023 (max concentration)
    // Assume 0 = 0 PPM, 1023 = 10000 PPM (arbitrary, for demonstration)
    // For smoke, 200-300 PPM is a typical threshold for alert
    return (analogValue / 1023.0) * 10000.0;
}