import serial
import json
import time
import datetime
import argparse
import re
import os

# Create a folder for log files if it doesn't exist
LOG_DIR = "esp_logs"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# Parse command line arguments
parser = argparse.ArgumentParser(description='ESP8266 Serial to JSON Lines Logger')
parser.add_argument('--port', type=str, required=True, help='Serial port to connect to (e.g., COM3 or /dev/ttyUSB0)')
parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
parser.add_argument('--log', action='store_true', help='Enable file logging to esp_logs/ directory')
args = parser.parse_args()

# Configure logging
log_file_handle = None
if args.log:
    timestamp_fn = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(LOG_DIR, f"esp_log_{timestamp_fn}.jsonl") # .jsonl for JSON Lines
    try:
        log_file_handle = open(log_filename, "w", encoding='utf-8')
        print(f"Logging to: {log_filename}")
    except IOError as e:
        print(f"Error opening log file {log_filename}: {e}")
        log_file_handle = None # Ensure it's None if opening failed

def parse_packet_line(line):
    """
    Parse an ESP-NOW packet line and convert to a dictionary.
    Expected format from ESP8266 printPacket function:
    ID: ... | ... | Smoke PPM: %.1f | ...
    """
    try:
        id_match = re.search(r'ID: ([^\s|]+)', line)
        mac_match = re.search(r'MAC: ([0-9A-F:]{17})', line) # Matches XX:XX:XX:XX:XX:XX
        ts_match = re.search(r'TS: (\d+)', line)
        lat_match = re.search(r'Lat: ([-\d.]+)', line)
        lon_match = re.search(r'Lon: ([-\d.]+)', line)
        eq_match = re.search(r'EQ: (\d)', line)
        motion_match = re.search(r'Motion: ([\d.]+)', line)
        gas_match = re.search(r'Gas: (\d)', line)
        temp_match = re.search(r'Temp: ([-\d.]+)C', line) # Corrected: Expects 'C' at the end
        prio_match = re.search(r'Prio: (\d)', line)
        ttl_match = re.search(r'TTL: (\d+)', line)
        retry_match = re.search(r'Retry: (\d+)', line)
        smokeppm_match = re.search(r'Smoke PPM: ([\d.]+)', line)
        
        packet = {
            "message_id": id_match.group(1) if id_match else "parse_error",
            "mac": mac_match.group(1) if mac_match else "parse_error",
            "device_timestamp_ms": int(ts_match.group(1)) if ts_match else 0,
            "received_time_iso": datetime.datetime.now().isoformat(),
            "latitude": float(lat_match.group(1)) if lat_match else 0.0,
            "longitude": float(lon_match.group(1)) if lon_match else 0.0,
            "earthquake_triggered": bool(int(eq_match.group(1))) if eq_match else False,
            "motion_g": float(motion_match.group(1)) if motion_match else 0.0,
            "gas_alert_triggered": bool(int(gas_match.group(1))) if gas_match else False,
            "temperature_celsius": float(temp_match.group(1)) if temp_match else -999.0, # ESP error value
            "priority": int(prio_match.group(1)) if prio_match else 0,
            "ttl": int(ttl_match.group(1)) if ttl_match else 0,
            "retry_count": int(retry_match.group(1)) if retry_match else 0,
            "smoke_ppm": float(smokeppm_match.group(1)) if smokeppm_match else 0.0,
        }
        return packet
    except Exception as e:
        print(f"Error parsing packet line '{line}': {e}")
        return None

def write_to_log(data_dict):
    """Write data dictionary to log file as a JSON Line."""
    if log_file_handle:
        try:
            log_file_handle.write(json.dumps(data_dict) + "\n")
            log_file_handle.flush() # Ensure data is written to disk periodically
        except IOError as e:
            print(f"Error writing to log file: {e}")
        except Exception as e:
            print(f"Unexpected error during log write: {e}")


def main():
    print(f"Starting ESP8266 Serial Bridge on {args.port} at {args.baud} baud.")
    if args.log and not log_file_handle:
        print("File logging was enabled but failed to open log file. Continuing without file logging.")
    print("Press Ctrl+C to exit.")
    
    ser_conn = None
    try:
        ser_conn = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Successfully connected to {args.port}.")
        
        while True:
            try:
                line = ser_conn.readline().decode('utf-8', errors='replace').strip()
            except serial.SerialException as e:
                print(f"Serial error during read: {e}. Attempting to reconnect...")
                if ser_conn and ser_conn.is_open:
                    ser_conn.close()
                time.sleep(5) # Wait before trying to reconnect
                try:
                    ser_conn = serial.Serial(args.port, args.baud, timeout=1)
                    print(f"Reconnected to {args.port}.")
                except serial.SerialException as recon_e:
                    print(f"Failed to reconnect: {recon_e}. Exiting.")
                    break
                continue # Skip to next iteration after attempting reconnection
            except Exception as e:
                print(f"Error reading line: {e}")
                line = None


            if line:
                print(f"RAW > {line}") # Print raw line for debugging
                
                log_entry = {
                    "type": "unknown",
                    "timestamp_iso": datetime.datetime.now().isoformat(),
                    "raw_message": line
                }

                # Check if this line contains the specific packet data signature
                if all(kw in line for kw in ["ID:", "MAC:", "Lat:", "Lon:", "Temp:"]):
                    packet_data = parse_packet_line(line)
                    if packet_data:
                        log_entry["type"] = "packet_data"
                        log_entry["parsed_data"] = packet_data
                        # Optionally print a summary of the parsed packet
                        print(f"  [PARSED PACKET] ID: {packet_data['message_id']}, MAC: {packet_data['mac']}, Temp: {packet_data['temperature_celsius']}C, Motion: {packet_data['motion_g']}g")
                        if packet_data['earthquake_triggered']:
                            print("  ⚠️ EARTHQUAKE ALERT DETECTED!")
                        if packet_data['gas_alert_triggered']:
                            print("  ⚠️ GAS ALERT DETECTED!")
                    else:
                        log_entry["type"] = "parse_error"
                else:
                    log_entry["type"] = "generic_log"
                
                if args.log:
                    write_to_log(log_entry)
            
            # time.sleep(0.01) # Small delay if needed, but readline with timeout should be okay

    except KeyboardInterrupt:
        print("\nExiting by user request (Ctrl+C).")
    except serial.SerialException as e:
        print(f"Serial connection error: {e}. Please check port and device.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser_conn and ser_conn.is_open:
            ser_conn.close()
            print("Serial port closed.")
        if log_file_handle:
            log_file_handle.close()
            print("Log file closed.")

if __name__ == '__main__':
    main()