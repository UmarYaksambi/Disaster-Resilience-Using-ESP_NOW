# 🆘 Disaster Resilience Using ESP-NOW Mesh Network

This project enables real-time SOS alerts using ESP8266-based wireless nodes that communicate over **ESP-NOW** and visualize critical events (earthquake detection, gas alerts, motion triggers) in a **Streamlit web dashboard**.

📡 Built for disaster zones where internet may be unavailable, it uses low-power ESP8266 devices to transmit emergency alerts across a mesh, visualize on a laptop dashboard, and log sensor data for analysis.

-----

## 📁 Project Structure

```
umaryaksambi-disaster-resilience-using-esp_now/
├── README.md               # You’re here
├── app.py                  # Streamlit dashboard (reads JSON logs)
├── bridge.py               # Serial to JSON bridge (writes logs)
├── ESP.ino                 # Main ESP8266 firmware (Sender / Receiver / Rebroadcaster)
├── SOS-Sender.ino          # Trial firmware for earlier testing (legacy)
└── requirements.txt        # Python dependencies
```

-----

## 🛠️ Requirements

  - 🖥️ **Python ≥ 3.8**
  - 📲 **ESP8266 board** (NodeMCU or similar)
  - 📦 **Sensors**: MPU6050, DHT22, (optional: gas sensor, GPS)
  - 📦 **Arduino libraries**: `Adafruit_MPU6050`, `DHT`, `TinyGPS++`, `espnow`, `FS`

Install Python dependencies with:

```bash
pip install -r requirements.txt
```

-----

## 🚀 How It Works

**ESP8266 nodes** use **ESP-NOW** to transmit alert packets containing:

  * Earthquake detection (via MPU6050)
  * Gas leak alert (optional analog sensor)
  * Temperature (via DHT22)
  * GPS coordinates (optional)

A laptop runs `bridge.py`, connected via USB to a **Receiver node**.

  * Parses incoming serial data
  * Logs structured JSON packets to `esp_logs/esp_log_*.jsonl`

`app.py` is a **Streamlit web dashboard**:

  * Reads recent packets from `esp_logs/`
  * Visualizes data, shows alerts, plots charts and maps

-----

## 🧭 ESP Node Roles

The main firmware file (`ESP.ino`) supports 3 roles:

| Role          | Description                                                    |
| :------------ | :------------------------------------------------------------- |
| **SENDER** | Collects sensor data and sends SOS packets via ESP-NOW         |
| **REBROADCASTER** | Forwards packets from other nodes to extend the network        |
| **RECEIVER** | Receives packets, logs them to SPIFFS, and sends them over serial (USB) |

Before flashing, configure the role by editing the top of `ESP.ino`:

🟢 To configure as a **SENDER** (default):

```arduino
#define ROLE_SENDER
// #define ROLE_REBROADCASTER
// #define ROLE_RECEIVER
```

🟠 To configure as a **REBROADCASTER**:

```arduino
// #define ROLE_SENDER
#define ROLE_REBROADCASTER
// #define ROLE_RECEIVER
```

🔴 To configure as a **RECEIVER**:

```arduino
// #define ROLE_SENDER
// #define ROLE_REBROADCASTER
#define ROLE_RECEIVER
```

⚠️ **Only one role should be uncommented per device.** Recompile and upload after changing the role.

-----

## 📥 Flashing Code with Arduino IDE

Follow these steps to upload code to the ESP8266:

1.  **Install the ESP8266 Board Package**

      * Open Arduino IDE
      * Go to `File > Preferences`
      * In the "Additional Boards Manager URLs" field, add:
        ```
        http://arduino.esp8266.com/stable/package_esp8266com_index.json
        ```
      * Then go to `Tools > Board > Boards Manager`
      * Search for `ESP8266` and click **Install**

2.  **Connect Your ESP8266 Board**

      * Plug in your NodeMCU/ESP8266 via USB
      * Go to `Tools > Board`, select "**NodeMCU 1.0 (ESP-12E Module)**"
      * Set **Port** under `Tools > Port` (e.g., `COM3` or `/dev/ttyUSB0`)

3.  **Install Required Libraries**

      * Go to `Sketch > Include Library > Manage Libraries`
      * Install the following:
          * `Adafruit_MPU6050`
          * `DHT sensor library`
          * `TinyGPS++`
          * `espnow`
          * `FS` (already included in ESP8266 core)

4.  **Open the Firmware File**

      * Open `ESP.ino` in the Arduino IDE
      * Set the correct role macro at the top (`#define ROLE_SENDER`, etc.)

5.  **Compile and Upload**

      * Click **Verify** (✔️) to compile
      * Click **Upload** (➡️) to flash the firmware
      * Ensure the baud rate is set to **115200** under `Tools > Serial Monitor`

-----

## 🔌 How to Run

1.  Flash `ESP.ino` to each ESP8266 device, setting the appropriate role.

2.  Connect the **Receiver node** to your laptop via USB.

3.  Start the serial-to-JSON bridge:

    ```bash
    python bridge.py --port COM3 --log
    ```

    (replace `COM3` with your actual port name)

4.  In another terminal, start the Streamlit dashboard:

    ```bash
    streamlit run app.py
    ```

5.  Trigger an SOS alert from the **sender node**:

      * Press the SOS button
      * Shake the device (to simulate earthquake motion)
      * Trigger gas sensor if connected

-----

## 📸 Features

  * ⚡ **ESP-NOW mesh communication** (no Wi-Fi required)
  * 📡 **Earthquake detection** via MPU6050
  * 🌡️ **Temperature readings** via DHT22
  * 🔥 Optional **gas sensor alert**
  * 🛰️ Optional **GPS location** via TinyGPS++
  * 🔁 **Rebroadcasting** via TTL and message ID deduplication
  * 📊 **Streamlit dashboard** with charts and maps
  * 📁 **Local logging** in JSONL format for offline analysis

-----

## 🧪 Debugging & Tips

  * Only one app can access the serial port at a time. Do not open Arduino Serial Monitor while running `bridge.py`.
  * Ensure the correct baud rate (112500) is set in both firmware and `bridge.py`.
  * Logs are saved in `esp_logs/esp_log_YYYYMMDD_HHMMSS.jsonl`.

-----

## 📚 Notes

  * `ESP.ino` is the main working firmware.
  * `SOS-Sender.ino` is an earlier prototype (not required).
  * Data is stored in JSON Lines format, easy to parse and process.

-----

## ☕ Support the Developer

If you find this project useful, consider supporting me:

[![Buy Me A Coffee](https://img.buymeacoffee.com/button-api/?text=Buy%20me%20a%20coffee&emoji=☕&slug=umaryaksambi&button_colour=FFDD00&font_colour=000000&font_family=Comic&outline_colour=000000&coffee_colour=ffffff)](https://www.buymeacoffee.com/umaryaksambi)

---
