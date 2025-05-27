import streamlit as st
import json
import pandas as pd
import time
import threading
import queue
import datetime
import glob
import os
from collections import deque
from typing import Dict, List, Optional, Tuple, Deque
import plotly.express as px
import plotly.graph_objects as go

# Set page config
st.set_page_config(
    page_title="ESP-NOW Mesh Network Monitor",
    page_icon="🛰️",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Define constants
MAX_MESSAGES = 100  # Maximum number of messages to keep in memory
LOG_CHECK_INTERVAL = 0.2  # Check for new log lines every 0.2 seconds

# Initialize session state variables if they don't exist
if 'connected' not in st.session_state:
    st.session_state.connected = False
if 'log_reader_thread' not in st.session_state:
    st.session_state.log_reader_thread = None
if 'messages' not in st.session_state:
    st.session_state.messages = []
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
if 'processed_files' not in st.session_state:
    st.session_state.processed_files = set()  # Track which files we've already processed
if 'file_positions' not in st.session_state:
    st.session_state.file_positions = {}  # Track position in each file
if 'selected_log_path' not in st.session_state:
    st.session_state.selected_log_path = "esp_logs/"
if 'current_log_file' not in st.session_state:
    st.session_state.current_log_file = None  # Track the currently monitored file

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
def find_log_files(log_path):
    """Find all ESP log files in the specified directory."""
    if not os.path.exists(log_path):
        return []
    
    pattern = os.path.join(log_path, "esp_log_*.jsonl")
    return sorted(glob.glob(pattern))

def get_most_recent_log_file(log_path):
    """Get the most recent log file based on timestamp in filename."""
    log_files = find_log_files(log_path)
    if not log_files:
        return None
    
    # Sort by modification time as fallback, then by filename timestamp
    log_files_with_time = []
    for file_path in log_files:
        filename = os.path.basename(file_path)
        try:
            # Extract timestamp from filename like esp_log_20250521_143124.jsonl
            timestamp_part = filename.replace('esp_log_', '').replace('.jsonl', '')
            # Convert to datetime for proper sorting
            file_time = datetime.datetime.strptime(timestamp_part, '%Y%m%d_%H%M%S')
            log_files_with_time.append((file_path, file_time))
        except ValueError:
            # If timestamp parsing fails, use file modification time
            mod_time = datetime.datetime.fromtimestamp(os.path.getmtime(file_path))
            log_files_with_time.append((file_path, mod_time))
    
    # Sort by timestamp (most recent first)
    log_files_with_time.sort(key=lambda x: x[1], reverse=True)
    return log_files_with_time[0][0] if log_files_with_time else None

def connect_to_logs(log_path):
    """Start monitoring the most recent log file in the specified directory."""
    try:
        if not os.path.exists(log_path):
            st.error(f"Log directory '{log_path}' does not exist.")
            return False
        
        most_recent_file = get_most_recent_log_file(log_path)
        if not most_recent_file:
            st.warning(f"No .jsonl log files found in '{log_path}' matching pattern 'esp_log_*.jsonl'")
            return False
        
        st.session_state.connected = True
        st.session_state.selected_log_path = log_path
        st.session_state.current_log_file = most_recent_file
        # Reset file position for the new file
        st.session_state.file_positions = {most_recent_file: 0}
        
        log_event(f"Started monitoring most recent log file: {os.path.basename(most_recent_file)}", "success")
        return True
    except Exception as e:
        st.error(f"Failed to start monitoring logs: {str(e)}")
        log_event(f"Log monitoring failed: {str(e)}", "error")
        return False

def disconnect_logs():
    """Stop monitoring log files."""
    st.session_state.connected = False
    st.session_state.current_log_file = None
    log_event("Stopped monitoring log files", "normal")

def process_json_line(json_line):
    """Process a single JSON line from the log file."""
    try:
        data = json.loads(json_line.strip())
        
        if data.get("type") == "packet_data" and "parsed_data" in data:
            # Extract packet data from the parsed_data field
            parsed = data["parsed_data"]
            
            # Convert to the format expected by the rest of the application
            packet = {
                "message_id": parsed.get("message_id", "unknown_id"),
                "mac": parsed.get("mac", "unknown_mac"),
                "timestamp": parsed.get("device_timestamp_ms", 0),
                "received_time": datetime.datetime.fromisoformat(parsed.get("received_time_iso", data.get("timestamp_iso", ""))).strftime("%Y-%m-%d %H:%M:%S"),
                "latitude": parsed.get("latitude", 0.0),
                "longitude": parsed.get("longitude", 0.0),
                "earthquake": parsed.get("earthquake_triggered", False),
                "motion": parsed.get("motion_g", 0.0),
                "gas_alert": parsed.get("gas_alert_triggered", False),
                "temperature": parsed.get("temperature_celsius", -999.0),
                "priority": parsed.get("priority", 0),
                "ttl": parsed.get("ttl", 0),
                "retry_count": parsed.get("retry_count", 0),
                "alert_level": "normal"
            }
            
            # Determine alert level
            if packet["earthquake"]:
                packet["alert_level"] = "alert"
            elif packet["gas_alert"]:
                packet["alert_level"] = "warning"
            elif packet["motion"] > 1.0:
                packet["alert_level"] = "warning"
            
            update_node_status(packet["mac"], packet)
            
            # Log events for alerts
            if packet["earthquake"]:
                log_event(f"EARTHQUAKE DETECTED by {packet['mac']} - Motion: {packet['motion']:.2f}g, Prio: {packet['priority']}", "error")
            elif packet["gas_alert"]:
                log_event(f"GAS ALERT from {packet['mac']}, Prio: {packet['priority']}", "warning")
            elif packet["motion"] > 1.0 and packet["motion"] < 1.5:
                log_event(f"Significant motion: {packet['motion']:.2f}g by {packet['mac']}, Prio: {packet['priority']}", "warning")

            # Update map data
            if packet["latitude"] != 0.0 and packet["longitude"] != 0.0:
                status_map = "alert" if packet["earthquake"] or packet["gas_alert"] else "normal"
                new_point = pd.DataFrame([{
                    'lat': packet["latitude"], 'lon': packet["longitude"], 'status': status_map,
                    'message_id': packet["message_id"], 'temperature': packet["temperature"],
                    'motion': packet["motion"], 'time': datetime.datetime.now().strftime("%H:%M:%S")
                }])
                dfs = [df for df in [st.session_state.map_data, new_point] if not df.empty]
                if dfs:
                    st.session_state.map_data = pd.concat(dfs, ignore_index=True)
                    if len(st.session_state.map_data) > 100:
                        st.session_state.map_data = st.session_state.map_data.iloc[-100:]
            
            return packet
        
        elif data.get("type") == "generic_log":
            # Log generic messages to event log
            raw_msg = data.get("raw_message", "")
            if raw_msg.strip():
                if "HIGH PRIORITY" in raw_msg:
                    log_event(f"LOG: {raw_msg}", "warning")
                else:
                    log_event(f"LOG: {raw_msg}", "normal")
        
        return None
    except json.JSONDecodeError as e:
        log_event(f"Error parsing JSON line: {str(e)}", "error")
        return None
    except Exception as e:
        log_event(f"Error processing JSON data: {str(e)}", "error")
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

def log_file_reader(log_path, log_file, message_queue, stop_event):
    """Read from the most recent log file and add to message queue."""
    file_positions = {log_file: 0}
    while not stop_event.is_set():
        try:
            # Check for new log file (implement if needed)
            if log_file:
                current_pos = file_positions.get(log_file, 0)
                try:
                    with open(log_file, 'r', encoding='utf-8') as f:
                        f.seek(current_pos)
                        for line in f:
                            if line.strip():
                                message_queue.put(line.strip())
                        file_positions[log_file] = f.tell()
                except (FileNotFoundError, PermissionError):
                    pass
            time.sleep(LOG_CHECK_INTERVAL)
        except Exception:
            time.sleep(LOG_CHECK_INTERVAL)

# --- UI Layout ---
st.title("🛰️ ESP-NOW Mesh Network Monitor")

# Sidebar
with st.sidebar:
    st.header("Log File Connection")
    
    # Log directory input
    log_path = st.text_input("Log Directory Path", value=st.session_state.selected_log_path)
    
    # Show available log files and current file being monitored
    if os.path.exists(log_path):
        log_files = find_log_files(log_path)
        if log_files:
            st.success(f"Found {len(log_files)} .jsonl log file(s)")
            
            # Show the most recent file that would be selected
            most_recent = get_most_recent_log_file(log_path)
            if most_recent:
                st.info(f"Most recent: {os.path.basename(most_recent)}")
            
            # Show currently monitored file if connected
            if st.session_state.connected and st.session_state.current_log_file:
                st.success(f"📖 Currently monitoring: {os.path.basename(st.session_state.current_log_file)}")
            
            with st.expander("All Log Files", expanded=False):
                for f in reversed(log_files):  # Show newest first
                    file_time = "Unknown"
                    try:
                        filename = os.path.basename(f)
                        timestamp_part = filename.replace('esp_log_', '').replace('.jsonl', '')
                        file_time = datetime.datetime.strptime(timestamp_part, '%Y%m%d_%H%M%S').strftime('%Y-%m-%d %H:%M:%S')
                    except ValueError:
                        pass
                    st.text(f"{os.path.basename(f)} ({file_time})")
        else:
            st.warning("No .jsonl log files found matching pattern 'esp_log_*.jsonl'")
    else:
        st.error(f"Directory '{log_path}' does not exist")

    col_con1, col_con2 = st.columns(2)
    with col_con1:
        if not st.session_state.connected:
            if st.button("🔌 Connect"):
                if connect_to_logs(log_path):
                    stop_event = threading.Event()
                    st.session_state.stop_event = stop_event
                    st.session_state.log_reader_thread = threading.Thread(
                        target=log_file_reader,
                        args=(log_path, st.session_state.current_log_file, st.session_state.message_queue, stop_event),
                        daemon=True
                    )
                    st.session_state.log_reader_thread.start()
                    st.rerun()
        else:
            if st.button("✖️ Disconnect"):
                if 'stop_event' in st.session_state:
                    st.session_state.stop_event.set()
                disconnect_logs()
                st.rerun()
    
    with col_con2:
        status_text = "Connected" if st.session_state.connected else "Disconnected"
        status_class = "status-connected" if st.session_state.connected else "status-disconnected"
        st.markdown(f"<p class='{status_class}'>Status: {status_text}</p>", unsafe_allow_html=True)

    st.header("Controls")
    if st.button("Clear Messages & Map"):
        st.session_state.messages = []
        st.session_state.map_data = pd.DataFrame(columns=['lat', 'lon', 'status', 'message_id', 'temperature', 'motion', 'time'])
        st.session_state.node_status = {}
        # Only reset position for current file, not all files
        if st.session_state.current_log_file:
            st.session_state.file_positions = {st.session_state.current_log_file: 0}
        log_event("Messages, Map, Node Status cleared. File position reset to beginning.", "normal")
        st.rerun()

# Main content Tabs
tab_dashboard, tab_messages, tab_nodes, tab_events = st.tabs([
    "📊 Dashboard", "📨 Message Log", "📶 Node Status", "📜 Event Log"
])

# Process message queue (common to multiple tabs)
messages_processed_this_run = 0
new_data_received = False
if st.session_state.connected or not st.session_state.message_queue.empty():
    while not st.session_state.message_queue.empty() and messages_processed_this_run < 20:
        json_line = st.session_state.message_queue.get_nowait()
        new_data_received = True
        
        packet_data = process_json_line(json_line)
        if packet_data:
            st.session_state.messages.append(packet_data)
            if len(st.session_state.messages) > MAX_MESSAGES:
                st.session_state.messages = st.session_state.messages[-MAX_MESSAGES:]
        
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
            map_df_filtered = st.session_state.map_data[~((st.session_state.map_data['lat'] == 0) & (st.session_state.map_data['lon'] == 0))]
            if not map_df_filtered.empty:
                fig = px.scatter_map(
                    map_df_filtered,
                    lat="lat", lon="lon",
                    hover_name="message_id",
                    hover_data=["temperature", "motion", "time", "status"],
                    color="status",
                    color_discrete_map={"normal": "green", "warning": "orange", "alert": "red"},
                    zoom=10, height=450
                )
                fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})
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
            for msg in st.session_state.messages[-20:]:
                if "temperature" in msg and msg["temperature"] != -999.0:
                    temp_data_points.append({
                        "time": msg.get("received_time"), 
                        "temperature": msg["temperature"], 
                        "node": msg["mac"][-5:]
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
            for msg in st.session_state.messages[-20:]:
                if "motion" in msg:
                    motion_data_points.append({
                        "time": msg.get("received_time"), 
                        "motion": msg["motion"], 
                        "node": msg["mac"][-5:]
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
        for alert in recent_alerts[:5]:
            alert_type_icon = "🌍" if alert.get("earthquake") else "🔥" if alert.get("gas_alert") else "🏃"
            details = f"Node: **{alert.get('mac', '')}** (Prio: {alert.get('priority', 0)}) at {alert.get('received_time', '')}<br>"
            details += f"Motion: {alert.get('motion', 0):.2f}g | Temp: {alert.get('temperature', 0):.1f}°C"
            if alert.get("earthquake"):
                st.error(f"{alert_type_icon} EARTHQUAKE ALERT: {details}", icon="🚨")
            elif alert.get("gas_alert"):
                st.warning(f"{alert_type_icon} GAS ALERT: {details}", icon="🔥")
            elif alert.get("alert_level") == "warning":
                 st.warning(f"{alert_type_icon} MOTION WARNING: {details}", icon="⚠️")
    else:
        st.success("✔️ No active alerts.", icon="✅")


with tab_messages:
    st.subheader("📨 Detailed Message Log")
    if st.session_state.messages:
        display_df_data = []
        for msg in reversed(st.session_state.messages):
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
        st.info("No nodes discovered yet. Connect to log files to see node statuses.")
6
with tab_events:
    st.subheader("📜 Application & Log Event Log")
    if st.session_state.event_log:
        for event in st.session_state.event_log:
            st.markdown(f"<span class='event-{event['level']}'>[{event['timestamp']}] {event['message']}</span>", unsafe_allow_html=True)
    else:
        st.info("No events logged yet.")

# Auto-refresh logic
if new_data_received:
    st.session_state.last_update = time.time()
    st.rerun()
elif time.time() - st.session_state.last_update > 2:
    st.session_state.last_update = time.time()
    if st.session_state.connected or messages_processed_this_run > 0:
        st.rerun()