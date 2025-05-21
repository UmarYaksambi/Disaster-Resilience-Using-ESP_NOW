import streamlit as st
import serial
import serial.tools.list_ports
import json
import pandas as pd
import time
import threading
import queue
import re
import datetime
import numpy as np
import plotly.express as px
import plotly.graph_objects as go
from collections import deque
from typing import Dict, List, Optional, Tuple, Deque

# Set page config
st.set_page_config(
    page_title="ESP-NOW Mesh Network Monitor",
    page_icon="🛰️",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Define constants
MAX_MESSAGES = 100  # Maximum number of messages to keep in memory
BAUD_RATE = 115200  # Serial baud rate

# Initialize session state variables if they don't exist
if 'connected' not in st.session_state:
    st.session_state.connected = False
if 'serial_port' not in st.session_state:
    st.session_state.serial_port = None
if 'messages' not in st.session_state:
    st.session_state.messages = []
if 'serial_buffer' not in st.session_state:
    st.session_state.serial_buffer = ""
if 'message_queue' not in st.session_state:
    st.session_state.message_queue = queue.Queue()
if 'last_update' not in st.session_state:
    st.session_state.last_update = time.time()
if 'node_status' not in st.session_state:
    st.session_state.node_status = {}  # Dictionary to track status of each node
if 'map_data' not in st.session_state:
    st.session_state.map_data = pd.DataFrame(columns=['lat', 'lon', 'status', 'message_id', 'temperature', 'motion', 'time'])
if 'event_log' not in st.session_state:
    st.session_state.event_log = deque(maxlen=100)  # Keep last 100 events

# CSS styling
st.markdown("""
<style>
    .main .block-container {
        padding-top: 2rem;
        padding-bottom: 2rem;
    }
    .status-connected {
        color: green;
        font-weight: bold;
    }
    .status-disconnected {
        color: red;
        font-weight: bold;
    }
    .status-warning {
        color: orange;
        font-weight: bold;
    }
    .status-alert {
        color: red;
        font-weight: bold;
        animation: blinker 1s linear infinite;
    }
    @keyframes blinker {
        50% {
            opacity: 0;
        }
    }
    .event-normal {
        color: #1E88E5; /* Blue */
    }
    .event-warning {
        color: #FFC107; /* Amber */
    }
    .event-error {
        color: #D32F2F; /* Red */
    }
    .event-success {
        color: #4CAF50; /* Green */
    }
</style>
""", unsafe_allow_html=True)

# Helper functions
def find_available_ports():
    """Return a list of available serial ports."""
    return [p.device for p in serial.tools.list_ports.comports()]

def connect_to_serial(port):
    """Connect to the selected serial port."""
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        st.session_state.connected = True
        st.session_state.serial_port = ser
        log_event(f"Connected to {port}", "success")
        return True
    except Exception as e:
        st.error(f"Failed to connect to {port}: {str(e)}")
        log_event(f"Connection failed: {str(e)}", "error")
        return False

def disconnect_serial():
    """Disconnect from the serial port."""
    if st.session_state.serial_port:
        try:
            st.session_state.serial_port.close()
        except:
            pass # Ignore errors on close
        st.session_state.connected = False
        st.session_state.serial_port = None
        log_event("Disconnected from serial port", "normal")

def extract_packet_data(line):
    """
    Extract packet data from a serial line.
    Expected format from ESP8266 printPacket function:
    ID: %s | MAC: %02X:%02X:%02X:%02X:%02X:%02X | TS: %lu | Lat: %.4f | Lon: %.4f | EQ: %d | Motion: %.2f | Gas: %d | Temp: %.2fC | Prio: %d | TTL: %d | Retry: %d
    Example: "ID: AB:CD:EF:12:34:56_12345 | MAC: AB:CD:EF:12:34:56 | TS: 12345 | Lat: 0.0000 | Lon: 0.0000 | EQ: 0 | Motion: 0.12 | Gas: 0 | Temp: 23.50C | Prio: 1 | TTL: 3 | Retry: 0"
    """
    try:
        id_match = re.search(r'ID: ([^\s|]+)', line)
        message_id = id_match.group(1) if id_match else "unknown_id"
        
        mac_match = re.search(r'MAC: ([0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2})', line)
        mac = mac_match.group(1) if mac_match else "unknown_mac"
        
        ts_match = re.search(r'TS: (\d+)', line)
        timestamp = int(ts_match.group(1)) if ts_match else 0
        
        lat_match = re.search(r'Lat: ([-\d.]+)', line)
        lat = float(lat_match.group(1)) if lat_match else 0.0
        
        lon_match = re.search(r'Lon: ([-\d.]+)', line)
        lon = float(lon_match.group(1)) if lon_match else 0.0
        
        eq_match = re.search(r'EQ: (\d)', line) # Usually 0 or 1
        earthquake = bool(int(eq_match.group(1))) if eq_match else False
        
        motion_match = re.search(r'Motion: ([\d.]+)', line)
        motion = float(motion_match.group(1)) if motion_match else 0.0
        
        gas_match = re.search(r'Gas: (\d)', line) # Usually 0 or 1
        gas_alert = bool(int(gas_match.group(1))) if gas_match else False
        
        temp_match = re.search(r'Temp: ([-\d.]+)C', line) # Corrected to include 'C'
        temp = float(temp_match.group(1)) if temp_match else -999.0 # ESP error value
        
        priority_match = re.search(r'Prio: (\d)', line)
        priority = int(priority_match.group(1)) if priority_match else 0
        
        ttl_match = re.search(r'TTL: (\d+)', line)
        ttl = int(ttl_match.group(1)) if ttl_match else 0
        
        retry_match = re.search(r'Retry: (\d+)', line)
        retry = int(retry_match.group(1)) if retry_match else 0
        
        alert_level = "normal"
        if earthquake:
            alert_level = "alert"
        elif gas_alert:
            alert_level = "warning" # Or "alert" depending on severity
        elif motion > 1.0: # Significant motion but below earthquake threshold
            alert_level = "warning"
        
        packet = {
            "message_id": message_id,
            "mac": mac,
            "timestamp": timestamp, # ESP8266 millis()
            "received_time": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "latitude": lat,
            "longitude": lon,
            "earthquake": earthquake,
            "motion": motion,
            "gas_alert": gas_alert,
            "temperature": temp,
            "priority": priority,
            "ttl": ttl,
            "retry_count": retry,
            "alert_level": alert_level
        }
        
        update_node_status(mac, packet)
        
        if earthquake:
            log_event(f"EARTHQUAKE DETECTED by {mac} - Motion: {motion:.2f}g, Prio: {priority}", "error")
        elif gas_alert:
            log_event(f"GAS ALERT from {mac}, Prio: {priority}", "warning")
        elif motion > 1.0 and motion < 1.5 : # ACC_THRESHOLD is 1.5 in C++
             log_event(f"Significant motion: {motion:.2f}g by {mac}, Prio: {priority}", "warning")

        if lat != 0.0 and lon != 0.0:
            status_map = "alert" if earthquake or gas_alert else "normal"
            new_point = pd.DataFrame([{
                'lat': lat, 'lon': lon, 'status': status_map,
                'message_id': message_id, 'temperature': temp,
                'motion': motion, 'time': datetime.datetime.now().strftime("%H:%M:%S")
            }])
            st.session_state.map_data = pd.concat([st.session_state.map_data, new_point], ignore_index=True)
            if len(st.session_state.map_data) > 100: # Keep last 100 points for map
                st.session_state.map_data = st.session_state.map_data.iloc[-100:]
        
        return packet
    except Exception as e:
        log_event(f"Error parsing packet data from line '{line}': {str(e)}", "error")
        return None

def update_node_status(mac, packet):
    """Update the status of a node based on received packet."""
    now = time.time()
    if mac not in st.session_state.node_status:
        st.session_state.node_status[mac] = {
            "last_seen": now,
            "packets_received": 1,
            "last_packet": packet,
            "first_seen": now
        }
        log_event(f"New node discovered: {mac}", "success")
    else:
        st.session_state.node_status[mac]["last_seen"] = now
        st.session_state.node_status[mac]["packets_received"] += 1
        st.session_state.node_status[mac]["last_packet"] = packet

def log_event(message, level="normal"):
    """Add event to log with timestamp."""
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    st.session_state.event_log.appendleft({
        "timestamp": timestamp,
        "message": message,
        "level": level
    })

def serial_reader():
    """Read from serial port and add to message queue."""
    while st.session_state.connected:
        try:
            if st.session_state.serial_port and st.session_state.serial_port.is_open:
                line = st.session_state.serial_port.readline().decode('utf-8', errors='replace').strip()
                if line:
                    st.session_state.message_queue.put(line)
            else: # Connection lost or port not open
                if st.session_state.connected: # if we thought we were connected
                    log_event("Serial port no longer open. Attempting to disconnect.", "error")
                    st.session_state.connected = False # Trigger UI update and stop loop
                break
        except serial.SerialException as e:
            log_event(f"Serial read error: {str(e)}. Disconnecting.", "error")
            st.session_state.connected = False # Ensure disconnected state
            break # Exit thread on serial error
        except Exception as e: # Catch any other unexpected error
            log_event(f"Unexpected error in serial_reader: {str(e)}. Disconnecting.", "error")
            st.session_state.connected = False
            break
        time.sleep(0.01)

# --- UI Layout ---
st.title("🛰️ ESP-NOW Mesh Network Monitor")

# Sidebar
with st.sidebar:
    st.header("Connection")
    port_list = find_available_ports()
    if not port_list:
        st.warning("No serial ports detected.")
        selected_port_disabled = True
    else:
        selected_port_disabled = False

    selected_port = st.selectbox("Select Serial Port", port_list, disabled=selected_port_disabled)
    
    col_con1, col_con2 = st.columns(2)
    with col_con1:
        if not st.session_state.connected:
            if st.button("🔌 Connect", disabled=selected_port_disabled):
                if selected_port and connect_to_serial(selected_port):
                    threading.Thread(target=serial_reader, daemon=True).start()
                    st.experimental_rerun() # Refresh UI after connection attempt
        else:
            if st.button("✖️ Disconnect"):
                disconnect_serial()
                st.experimental_rerun() # Refresh UI
    
    with col_con2:
        status_text = "Connected" if st.session_state.connected else "Disconnected"
        status_class = "status-connected" if st.session_state.connected else "status-disconnected"
        st.markdown(f"<p class='{status_class}'>Status: {status_text}</p>", unsafe_allow_html=True)

    st.header("Controls")
    if st.button("Clear Messages & Map"):
        st.session_state.messages = []
        st.session_state.map_data = pd.DataFrame(columns=['lat', 'lon', 'status', 'message_id', 'temperature', 'motion', 'time'])
        st.session_state.node_status = {} # Also clear node status for consistency
        log_event("Messages, Map, and Node Status cleared.", "normal")
        st.experimental_rerun()

# Main content Tabs
tab_dashboard, tab_messages, tab_nodes, tab_events = st.tabs([
    "📊 Dashboard", "📨 Message Log", "📶 Node Status", "📜 Event Log"
])

# Process message queue (common to multiple tabs)
messages_processed_this_run = 0
new_data_received = False
if st.session_state.connected or not st.session_state.message_queue.empty(): # Process if connected or if there's backlog
    while not st.session_state.message_queue.empty() and messages_processed_this_run < 20: # Process more messages per run
        line = st.session_state.message_queue.get_nowait() # Use get_nowait as we check empty()
        new_data_received = True
        
        # Check if line matches the expected packet format
        if all(kw in line for kw in ["ID:", "MAC:", "Lat:", "Lon:", "Temp:"]):
            packet_data = extract_packet_data(line)
            if packet_data:
                st.session_state.messages.append(packet_data)
                if len(st.session_state.messages) > MAX_MESSAGES:
                    st.session_state.messages = st.session_state.messages[-MAX_MESSAGES:]
        else:
            # For non-packet messages, log them to the event log if they are not empty
            if line.strip(): # Avoid logging empty lines
                log_event(f"RAW: {line}", "normal") # Distinguish raw serial lines
        messages_processed_this_run += 1

with tab_dashboard:
    st.subheader("🚀 Network Overview")
    
    cols_metrics = st.columns(4)
    node_count = len(st.session_state.node_status)
    cols_metrics[0].metric("Active Nodes", node_count if node_count > 0 else "0")
    cols_metrics[1].metric("Total Messages Seen", len(st.session_state.messages))
    
    alert_count = sum(1 for msg in st.session_state.messages if msg.get("alert_level") in ["warning", "alert"])
    cols_metrics[2].metric("Active Alerts", alert_count)

    if node_count > 0:
        now = time.time()
        # Consider node active if seen in the last 60 seconds
        recently_active_nodes = sum(1 for node in st.session_state.node_status.values() if (now - node["last_seen"]) < 60)
        health_percentage = int((recently_active_nodes / node_count) * 100)
    else:
        health_percentage = 0
    cols_metrics[3].metric("Network Health", f"{health_percentage}%")

    st.markdown("---")
    col_map, col_charts = st.columns([3, 2])

    with col_map:
        st.subheader("🌍 Node Map")
        if not st.session_state.map_data.empty and not st.session_state.map_data[['lat', 'lon']].eq(0).all().all():
            # Filter out 0,0 coordinates before plotting if they are not desired
            map_df_filtered = st.session_state.map_data[~((st.session_state.map_data['lat'] == 0) & (st.session_state.map_data['lon'] == 0))]
            if not map_df_filtered.empty:
                fig = px.scatter_mapbox(
                    map_df_filtered,
                    lat="lat", lon="lon",
                    hover_name="message_id",
                    hover_data=["temperature", "motion", "time", "status"],
                    color="status",
                    color_discrete_map={"normal": "green", "warning": "orange", "alert": "red"},
                    zoom=10, height=450
                )
                fig.update_layout(mapbox_style="open-street-map", margin={"r":0,"t":0,"l":0,"b":0})
                st.plotly_chart(fig, use_container_width=True)
            else:
                st.info("No valid GPS coordinates (non-zero) received for map display.")
        else:
            st.info("No GPS data received yet. Map will populate when location data is available.")

    with col_charts:
        st.subheader("📈 Sensor Trends (Last 20 Msgs)")
        if st.session_state.messages:
            # Temperature Chart
            temp_data_points = []
            for msg in st.session_state.messages[-20:]: # Last 20
                if "temperature" in msg and msg["temperature"] != -999.0:
                    temp_data_points.append({
                        "time": msg.get("received_time"), 
                        "temperature": msg["temperature"], 
                        "node": msg["mac"][-5:] # Short MAC
                    })
            if temp_data_points:
                temp_df = pd.DataFrame(temp_data_points)
                temp_fig = px.line(temp_df, x="time", y="temperature", color="node", title="Temperature (°C)", height=200)
                temp_fig.update_layout(margin={"r":0,"t":30,"l":0,"b":0}, legend_title_text='Node')
                st.plotly_chart(temp_fig, use_container_width=True)
            else:
                st.caption("No valid temperature data for chart.")

            # Motion Chart
            motion_data_points = []
            for msg in st.session_state.messages[-20:]: # Last 20
                if "motion" in msg:
                    motion_data_points.append({
                        "time": msg.get("received_time"), 
                        "motion": msg["motion"], 
                        "node": msg["mac"][-5:] # Short MAC
                    })
            if motion_data_points:
                motion_df = pd.DataFrame(motion_data_points)
                motion_fig = px.line(motion_df, x="time", y="motion", color="node", title="Motion (g)", height=200)
                motion_fig.update_layout(margin={"r":0,"t":30,"l":0,"b":0}, legend_title_text='Node')
                st.plotly_chart(motion_fig, use_container_width=True)
            else:
                st.caption("No motion data for chart.")
        else:
            st.info("No messages yet to display sensor trends.")
            
    st.markdown("---")
    st.subheader("🚨 Recent Alerts (Last 5)")
    recent_alerts = [msg for msg in reversed(st.session_state.messages) if msg.get("alert_level") in ["warning", "alert"]]
    
    if recent_alerts:
        for alert in recent_alerts[:5]: # Display latest 5
            alert_type_icon = "🌍" if alert.get("earthquake") else "🔥" if alert.get("gas_alert") else "🏃"
            alert_color = "red" if alert.get("alert_level") == "alert" else "orange"
            details = f"Node: **{alert.get('mac', '')}** (Prio: {alert.get('priority', 0)}) at {alert.get('received_time', '')}<br>"
            details += f"Motion: {alert.get('motion', 0):.2f}g | Temp: {alert.get('temperature', 0):.1f}°C"
            if alert.get("earthquake"):
                st.error(f"{alert_type_icon} EARTHQUAKE ALERT: {details}", icon="🚨")
            elif alert.get("gas_alert"):
                st.warning(f"{alert_type_icon} GAS ALERT: {details}", icon="🔥")
            elif alert.get("alert_level") == "warning": # Motion warning
                 st.warning(f"{alert_type_icon} MOTION WARNING: {details}", icon="⚠️")
    else:
        st.success("✔️ No active alerts.", icon="✅")


with tab_messages:
    st.subheader("📨 Detailed Message Log")
    if st.session_state.messages:
        display_df_data = []
        for msg in reversed(st.session_state.messages): # Show newest first
            status_icon = "🚨" if msg.get("alert_level") == "alert" else "⚠️" if msg.get("alert_level") == "warning" else "✔️"
            display_df_data.append({
                "Status": status_icon,
                "Received Time": msg.get("received_time", ""),
                "Node MAC": msg.get("mac", ""),
                "Message ID": msg.get("message_id", ""),
                "Lat": f"{msg.get('latitude', 0.0):.4f}",
                "Lon": f"{msg.get('longitude', 0.0):.4f}",
                "Temp (°C)": f"{msg.get('temperature', 0.0):.1f}",
                "Motion (g)": f"{msg.get('motion', 0.0):.2f}",
                "EQ": "Yes" if msg.get("earthquake") else "No",
                "Gas": "Yes" if msg.get("gas_alert") else "No",
                "Prio": msg.get("priority", 0),
                "TTL": msg.get("ttl", 0),
            })
        display_df = pd.DataFrame(display_df_data)
        st.dataframe(display_df, use_container_width=True)
    else:
        st.info("No messages received yet.")

with tab_nodes:
    st.subheader("📶 Node Status Details")
    if st.session_state.node_status:
        sorted_nodes = sorted(st.session_state.node_status.items(), key=lambda item: item[1]['last_seen'], reverse=True)
        for mac, status_data in sorted_nodes:
            with st.expander(f"Node: {mac}", expanded=True):
                cols_node = st.columns((1, 2, 2, 1))
                
                now = time.time()
                last_seen_delta = now - status_data["last_seen"]
                
                if last_seen_delta < 30: node_health = "<span class='status-connected'>Online</span>"
                elif last_seen_delta < 120: node_health = "<span class='status-warning'>Idle</span>"
                else: node_health = "<span class='status-disconnected'>Offline</span>"
                cols_node[0].markdown(f"**Status:**<br>{node_health}", unsafe_allow_html=True)

                cols_node[1].markdown(f"""
                    **Last Seen:** {int(last_seen_delta)}s ago<br>
                    **First Seen:** {datetime.datetime.fromtimestamp(status_data['first_seen']).strftime('%H:%M:%S')}<br>
                    **Packets Rx:** {status_data['packets_received']}
                """)
                
                last_pkt = status_data.get("last_packet", {})
                cols_node[2].markdown(f"""
                    **Last Data:**<br>
                    Temp: {last_pkt.get('temperature', 'N/A'):.1f}°C | Motion: {last_pkt.get('motion', 'N/A'):.2f}g<br>
                    Lat: {last_pkt.get('latitude', 'N/A'):.4f} | Lon: {last_pkt.get('longitude', 'N/A'):.4f}
                """)

                alert_status = ""
                if last_pkt.get("earthquake"): alert_status = "<span class='status-alert'>EARTHQUAKE</span>"
                elif last_pkt.get("gas_alert"): alert_status = "<span class='status-alert'>GAS ALERT</span>"
                elif last_pkt.get("motion", 0) > 1.0: alert_status = "<span class='status-warning'>High Motion</span>"
                else: alert_status = "<span>Normal</span>"
                cols_node[3].markdown(f"**Node Alert:**<br>{alert_status}", unsafe_allow_html=True)
    else:
        st.info("No nodes discovered yet. Connect to a device to see node statuses.")

with tab_events:
    st.subheader("📜 Application & Raw Serial Event Log")
    if st.session_state.event_log:
        for event in st.session_state.event_log: # deque appends left, so iterate normally for chronological
            st.markdown(f"<span class='event-{event['level']}'>[{event['timestamp']}] {event['message']}</span>", unsafe_allow_html=True)
    else:
        st.info("No events logged yet.")

# Auto-refresh logic
if new_data_received or (time.time() - st.session_state.last_update > 2): # Refresh if new data or 2s elapsed
    st.session_state.last_update = time.time()
    if st.session_state.connected or messages_processed_this_run > 0 : # Only rerun if connected or if we just processed data
         st.experimental_rerun()