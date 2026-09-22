#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi IMU Monitor - Simulation Mode

This version simulates MPU6050 data for testing without hardware.
Useful for development and testing the web interface.

Usage:
    python app_simulation.py

Then open http://localhost:5000 in your browser.
"""

import os
import sys
import math
import time
import threading
import csv
from datetime import datetime
from collections import deque
from flask import Flask, jsonify, request, render_template_string
import random

# ==================== Configuration ====================
DATA_UPDATE_INTERVAL = 0.1  # 100 ms (10 Hz)
MAX_DATA_POINTS = 100
DATA_FOLDER = "data"

# ==================== Global Variables ====================
app = Flask(__name__)

# IMU Data Storage
imu_data = {
    'ax': 0.0, 'ay': 0.0, 'az': 0.0,
    'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'timestamp': 0,
    'error': None
}

# Historical data for charts (last 100 points)
history_data = {
    'timestamps': deque(maxlen=MAX_DATA_POINTS),
    'roll': deque(maxlen=MAX_DATA_POINTS),
    'pitch': deque(maxlen=MAX_DATA_POINTS),
    'yaw': deque(maxlen=MAX_DATA_POINTS),
}

# Recording variables
recording_state = {
    'is_recording': False,
    'csv_file': None,
    'csv_writer': None,
    'start_time': None,
    'filename': None,
    'label': 'unknown'
}

# Calibration variables
calibration_state = {
    'is_calibrating': False,
    'progress': 0,
    'accel_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'is_calibrated': False
}

# Thread control
data_lock = threading.Lock()
should_exit = False

# Simulation state
sim_state = {
    'roll_base': 0.0,
    'pitch_base': 0.0,
    'yaw_base': 0.0,
    'roll_vel': random.uniform(-1, 1),
    'pitch_vel': random.uniform(-1, 1),
    'yaw_vel': random.uniform(-1, 1),
}

# ==================== Simulation Data ====================
def generate_simulated_data():
    """Generate realistic simulated IMU data"""
    global sim_state
    
    # Update velocities (random walk)
    sim_state['roll_vel'] += random.uniform(-0.1, 0.1)
    sim_state['pitch_vel'] += random.uniform(-0.1, 0.1)
    sim_state['yaw_vel'] += random.uniform(-0.1, 0.1)
    
    # Clamp velocities
    sim_state['roll_vel'] = max(-5, min(5, sim_state['roll_vel']))
    sim_state['pitch_vel'] = max(-5, min(5, sim_state['pitch_vel']))
    sim_state['yaw_vel'] = max(-5, min(5, sim_state['yaw_vel']))
    
    # Update base angles
    sim_state['roll_base'] += sim_state['roll_vel'] * 0.1
    sim_state['pitch_base'] += sim_state['pitch_vel'] * 0.1
    sim_state['yaw_base'] += sim_state['yaw_vel'] * 0.1
    
    # Add noise
    roll = sim_state['roll_base'] + random.gauss(0, 0.5)
    pitch = sim_state['pitch_base'] + random.gauss(0, 0.5)
    yaw = sim_state['yaw_base'] + random.gauss(0, 0.5)
    
    # Calculate accelerometer values from angles
    roll_rad = roll * math.pi / 180
    pitch_rad = pitch * math.pi / 180
    
    # Simulate accelerometer (in g)
    ax = math.sin(pitch_rad)
    ay = math.sin(roll_rad) * math.cos(pitch_rad)
    az = math.cos(roll_rad) * math.cos(pitch_rad)
    
    # Simulate gyroscope (in °/s)
    gx = sim_state['roll_vel'] + random.gauss(0, 1)
    gy = sim_state['pitch_vel'] + random.gauss(0, 1)
    gz = sim_state['yaw_vel'] + random.gauss(0, 1)
    
    return {
        'ax': ax, 'ay': ay, 'az': az,
        'gx': gx, 'gy': gy, 'gz': gz,
        'roll': roll, 'pitch': pitch, 'yaw': yaw
    }

# ==================== IMU Data Reading ====================
def read_imu_data():
    """Read (simulated) data from MPU6050"""
    global imu_data, history_data, recording_state, data_lock
    
    # Generate simulated data
    sim_data = generate_simulated_data()
    
    # Update global data with lock
    with data_lock:
        imu_data['ax'] = round(sim_data['ax'], 4)
        imu_data['ay'] = round(sim_data['ay'], 4)
        imu_data['az'] = round(sim_data['az'], 4)
        imu_data['gx'] = round(sim_data['gx'], 2)
        imu_data['gy'] = round(sim_data['gy'], 2)
        imu_data['gz'] = round(sim_data['gz'], 2)
        imu_data['roll'] = round(sim_data['roll'], 2)
        imu_data['pitch'] = round(sim_data['pitch'], 2)
        imu_data['yaw'] = round(sim_data['yaw'], 2)
        imu_data['timestamp'] = time.time()
        imu_data['error'] = None
        
        # Update history
        history_data['timestamps'].append(len(history_data['timestamps']))
        history_data['roll'].append(sim_data['roll'])
        history_data['pitch'].append(sim_data['pitch'])
        history_data['yaw'].append(sim_data['yaw'])
        
        # Write to CSV if recording
        if recording_state['is_recording'] and recording_state['csv_writer'] is not None:
            elapsed_time = time.time() - recording_state['start_time']
            try:
                recording_state['csv_writer'].writerow([
                    f"{elapsed_time:.3f}",
                    imu_data['ax'],
                    imu_data['ay'],
                    imu_data['az'],
                    imu_data['gx'],
                    imu_data['gy'],
                    imu_data['gz'],
                    imu_data['roll'],
                    imu_data['pitch'],
                    imu_data['yaw'],
                    recording_state['label']
                ])
                recording_state['csv_file'].flush()
            except Exception as e:
                print(f"✗ Error writing to CSV: {e}")

def data_reader_thread():
    """Background thread for reading IMU data"""
    global should_exit
    
    print("► Data reader thread started (simulation mode)")
    
    while not should_exit:
        read_imu_data()
        time.sleep(DATA_UPDATE_INTERVAL)
    
    print("► Data reader thread stopped")

# ==================== CSV Recording ====================
def start_recording(label):
    """Start recording IMU data to CSV"""
    global recording_state, data_lock
    
    with data_lock:
        if recording_state['is_recording']:
            return {"success": False, "message": "Already recording"}
        
        # Create data folder if not exists
        if not os.path.exists(DATA_FOLDER):
            os.makedirs(DATA_FOLDER)
        
        # Generate filename
        now = datetime.now()
        filename = f"imu_{now.strftime('%Y%m%d_%H%M%S')}_{label}.csv"
        filepath = os.path.join(DATA_FOLDER, filename)
        
        try:
            csv_file = open(filepath, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            
            # Write header
            csv_writer.writerow(['time', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'roll', 'pitch', 'yaw', 'label'])
            csv_file.flush()
            
            recording_state['is_recording'] = True
            recording_state['csv_file'] = csv_file
            recording_state['csv_writer'] = csv_writer
            recording_state['start_time'] = time.time()
            recording_state['filename'] = filename
            recording_state['label'] = label
            
            print(f"✓ Started recording to {filename}")
            return {"success": True, "message": f"Recording started: {filename}"}
            
        except Exception as e:
            print(f"✗ Error starting recording: {e}")
            return {"success": False, "message": f"Error: {str(e)}"}

def stop_recording():
    """Stop recording IMU data to CSV"""
    global recording_state, data_lock
    
    with data_lock:
        if not recording_state['is_recording']:
            return {"success": False, "message": "Not recording"}
        
        try:
            if recording_state['csv_file'] is not None:
                recording_state['csv_file'].close()
            
            filename = recording_state['filename']
            recording_state['is_recording'] = False
            recording_state['csv_file'] = None
            recording_state['csv_writer'] = None
            recording_state['filename'] = None
            
            print(f"✓ Stopped recording: {filename}")
            return {"success": True, "message": f"Recording stopped: {filename}"}
            
        except Exception as e:
            print(f"✗ Error stopping recording: {e}")
            return {"success": False, "message": f"Error: {str(e)}"}

# ==================== Calibration ====================
def start_calibration():
    """Start calibration (simulated)"""
    global calibration_state, data_lock
    
    with data_lock:
        if calibration_state['is_calibrating']:
            return {"success": False, "message": "Calibration already in progress"}
        
        calibration_state['is_calibrating'] = True
        calibration_state['progress'] = 0
        
    print("► Starting calibration (10 seconds)...")
    
    # Run calibration in a separate thread
    calib_thread = threading.Thread(target=_calibration_worker, daemon=True)
    calib_thread.start()
    
    return {"success": True, "message": "Calibration started"}

def _calibration_worker():
    """Worker thread for calibration"""
    global calibration_state, data_lock
    
    try:
        samples = 100  # 10 seconds * 10 Hz
        
        for i in range(samples):
            # Simulate small offsets
            with data_lock:
                calibration_state['accel_offset']['x'] = -0.05 + random.gauss(0, 0.01)
                calibration_state['accel_offset']['y'] = 0.03 + random.gauss(0, 0.01)
                calibration_state['accel_offset']['z'] = -0.15 + random.gauss(0, 0.01)
                
                calibration_state['gyro_offset']['x'] = 1.5 + random.gauss(0, 0.2)
                calibration_state['gyro_offset']['y'] = -0.8 + random.gauss(0, 0.2)
                calibration_state['gyro_offset']['z'] = 0.2 + random.gauss(0, 0.2)
                
                calibration_state['progress'] = int((i + 1) / samples * 100)
            
            time.sleep(0.1)
        
        with data_lock:
            calibration_state['is_calibrating'] = False
            calibration_state['is_calibrated'] = True
            calibration_state['progress'] = 100
        
        print("✓ Calibration completed successfully")
        
    except Exception as e:
        print(f"✗ Calibration error: {e}")
        with data_lock:
            calibration_state['is_calibrating'] = False
            calibration_state['is_calibrated'] = False

def get_calibration_status():
    """Get calibration status"""
    global calibration_state, data_lock
    
    with data_lock:
        return {
            'is_calibrating': calibration_state['is_calibrating'],
            'progress': calibration_state['progress'],
            'is_calibrated': calibration_state['is_calibrated'],
            'accel_offset': calibration_state['accel_offset'].copy(),
            'gyro_offset': calibration_state['gyro_offset'].copy()
        }

def reset_calibration():
    """Reset calibration to default values"""
    global calibration_state, data_lock
    
    with data_lock:
        calibration_state['accel_offset'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        calibration_state['gyro_offset'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        calibration_state['is_calibrated'] = False
        calibration_state['progress'] = 0
    
    print("✓ Calibration reset to defaults")
    return {"success": True, "message": "Calibration reset"}

# ==================== Flask Routes ====================
@app.route('/')
def index():
    """Main web interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/data')
def get_data():
    """Get current IMU data and history"""
    with data_lock:
        return jsonify({
            'current': imu_data.copy(),
            'history': {
                'timestamps': list(history_data['timestamps']),
                'roll': list(history_data['roll']),
                'pitch': list(history_data['pitch']),
                'yaw': list(history_data['yaw']),
            }
        })

@app.route('/start_recording', methods=['POST'])
def api_start_recording():
    """API endpoint to start recording"""
    label = request.json.get('label', 'unknown')
    result = start_recording(label)
    return jsonify(result)

@app.route('/stop_recording', methods=['POST'])
def api_stop_recording():
    """API endpoint to stop recording"""
    result = stop_recording()
    return jsonify(result)

@app.route('/recording_status')
def recording_status():
    """Get recording status"""
    with data_lock:
        return jsonify({
            'is_recording': recording_state['is_recording'],
            'filename': recording_state['filename'],
            'label': recording_state['label']
        })

@app.route('/start_calibration', methods=['POST'])
def api_start_calibration():
    """API endpoint to start calibration"""
    result = start_calibration()
    return jsonify(result)

@app.route('/calibration_status')
def api_calibration_status():
    """Get calibration status"""
    status = get_calibration_status()
    return jsonify(status)

@app.route('/reset_calibration', methods=['POST'])
def api_reset_calibration():
    """API endpoint to reset calibration"""
    result = reset_calibration()
    return jsonify(result)

# ==================== HTML Template ====================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Raspberry Pi IMU Monitor (Simulation)</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 10px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.2);
        }
        
        .sim-badge {
            text-align: center;
            color: #ffeb3b;
            font-size: 0.9em;
            margin-bottom: 20px;
            font-weight: bold;
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 8px 16px rgba(0, 0, 0, 0.1);
            transition: transform 0.3s ease, box-shadow 0.3s ease;
        }
        
        .card:hover {
            transform: translateY(-5px);
            box-shadow: 0 12px 24px rgba(0, 0, 0, 0.15);
        }
        
        .card h2 {
            color: #667eea;
            font-size: 1.2em;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        
        .data-group {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        
        .data-item {
            background: #f8f9fa;
            padding: 12px;
            border-radius: 6px;
            border-left: 4px solid #667eea;
        }
        
        .data-label {
            font-size: 0.85em;
            color: #666;
            text-transform: uppercase;
            font-weight: 600;
            margin-bottom: 5px;
        }
        
        .data-value {
            font-size: 1.5em;
            color: #333;
            font-weight: bold;
            font-family: 'Courier New', monospace;
        }
        
        .chart-container {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 8px 16px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            position: relative;
            height: 400px;
        }
        
        .chart-title {
            color: #667eea;
            font-size: 1.2em;
            font-weight: bold;
            margin-bottom: 15px;
        }
        
        .control-card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 8px 16px rgba(0, 0, 0, 0.1);
        }
        
        .control-card h2 {
            color: #667eea;
            font-size: 1.2em;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        
        .control-group {
            margin-bottom: 20px;
        }
        
        .control-group label {
            display: block;
            color: #333;
            font-weight: 600;
            margin-bottom: 8px;
        }
        
        select, button {
            padding: 10px 15px;
            border: none;
            border-radius: 6px;
            font-size: 1em;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        select {
            width: 100%;
            background: #f8f9fa;
            color: #333;
            border: 2px solid #ddd;
        }
        
        select:focus {
            outline: none;
            border-color: #667eea;
            background: white;
        }
        
        .button-group {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        
        button {
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        
        .btn-start {
            background: #4CAF50;
            color: white;
        }
        
        .btn-start:hover:not(:disabled) {
            background: #45a049;
            box-shadow: 0 4px 12px rgba(76, 175, 80, 0.3);
        }
        
        .btn-stop {
            background: #f44336;
            color: white;
        }
        
        .btn-stop:hover:not(:disabled) {
            background: #da190b;
            box-shadow: 0 4px 12px rgba(244, 67, 54, 0.3);
        }
        
        button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        
        .status-box {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 6px;
            border-left: 4px solid #2196F3;
            margin-top: 15px;
        }
        
        .status-label {
            color: #666;
            font-size: 0.9em;
            margin-bottom: 5px;
        }
        
        .status-value {
            color: #333;
            font-weight: bold;
            font-family: 'Courier New', monospace;
        }
        
        .recording-active {
            border-left-color: #4CAF50;
            background: #e8f5e9;
        }
        
        .recording-inactive {
            border-left-color: #f44336;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Raspberry Pi IMU Monitor</h1>
        <div class="sim-badge">⚠️ SIMULATION MODE - This is simulated data for testing</div>
        
        <!-- Angles Display -->
        <div class="grid">
            <div class="card">
                <h2>📐 Roll</h2>
                <div class="data-value" id="roll-display">0.0°</div>
            </div>
            <div class="card">
                <h2>📏 Pitch</h2>
                <div class="data-value" id="pitch-display">0.0°</div>
            </div>
            <div class="card">
                <h2>🧭 Yaw</h2>
                <div class="data-value" id="yaw-display">0.0°</div>
            </div>
        </div>
        
        <!-- Raw Data Display -->
        <div class="card" style="margin-bottom: 30px;">
            <h2>📊 Raw IMU Data</h2>
            <h3 style="color: #666; margin-top: 15px; margin-bottom: 10px;">Accelerometer (g)</h3>
            <div class="data-group">
                <div class="data-item">
                    <div class="data-label">AX</div>
                    <div class="data-value" id="ax-display">0.0000</div>
                </div>
                <div class="data-item">
                    <div class="data-label">AY</div>
                    <div class="data-value" id="ay-display">0.0000</div>
                </div>
                <div class="data-item">
                    <div class="data-label">AZ</div>
                    <div class="data-value" id="az-display">0.0000</div>
                </div>
            </div>
            
            <h3 style="color: #666; margin-top: 20px; margin-bottom: 10px;">Gyroscope (°/s)</h3>
            <div class="data-group">
                <div class="data-item">
                    <div class="data-label">GX</div>
                    <div class="data-value" id="gx-display">0.00</div>
                </div>
                <div class="data-item">
                    <div class="data-label">GY</div>
                    <div class="data-value" id="gy-display">0.00</div>
                </div>
                <div class="data-item">
                    <div class="data-label">GZ</div>
                    <div class="data-value" id="gz-display">0.00</div>
                </div>
            </div>
        </div>
        
        <!-- Chart -->
        <div class="chart-container">
            <div class="chart-title">📈 Real-time Waveform</div>
            <canvas id="imuChart"></canvas>
        </div>
        
        <!-- Calibration Panel -->
        <div class="control-card">
            <h2>⚙️ Calibration</h2>
            
            <div class="control-group">
                <p style="color: #666; font-size: 0.9em; margin-bottom: 15px;">
                    Keep the sensor stationary for 10 seconds. Calibration removes sensor biases.
                </p>
            </div>
            
            <div class="button-group" style="grid-template-columns: 1fr 1fr;">
                <button class="btn-start" id="calibrate-btn" onclick="startCalibration()">🔧 Start Calibration</button>
                <button class="btn-stop" id="reset-calib-btn" onclick="resetCalibration()" style="background: #ff9800;">↺ Reset</button>
            </div>
            
            <div id="calibration-progress" style="display: none; margin-top: 15px;">
                <div style="color: #666; margin-bottom: 8px;">Progress: <span id="calib-progress-text">0%</span></div>
                <div style="background: #e0e0e0; height: 8px; border-radius: 4px; overflow: hidden;">
                    <div id="calib-progress-bar" style="background: #4CAF50; height: 100%; width: 0%; transition: width 0.3s ease;"></div>
                </div>
            </div>
            
            <div class="status-box" id="calib-status-box" style="margin-top: 15px;">
                <div class="status-label">Calibration Status</div>
                <div class="status-value" id="calib-status-text">❌ Not Calibrated</div>
            </div>
            
            <div id="calib-offsets-box" style="display: none; background: #f5f5f5; padding: 12px; border-radius: 6px; margin-top: 12px; font-size: 0.85em;">
                <div style="color: #333; margin-bottom: 8px;"><strong>Current Offsets:</strong></div>
                <div style="font-family: monospace; color: #666; line-height: 1.6;">
                    Accel: x=<span id="accel-x">0.0000</span> y=<span id="accel-y">0.0000</span> z=<span id="accel-z">0.0000</span><br>
                    Gyro: x=<span id="gyro-x">0.00</span> y=<span id="gyro-y">0.00</span> z=<span id="gyro-z">0.00</span>
                </div>
            </div>
        </div>
        
        <!-- Recording Panel -->
        <div class="control-card" style="margin-top: 20px;">
            <h2>🎮 Recording Control</h2>
            
            <div class="control-group">
                <label for="label-select">Select Label:</label>
                <select id="label-select">
                    <option value="rest">Rest</option>
                    <option value="flexion">Flexion</option>
                    <option value="extension">Extension</option>
                    <option value="hold">Hold</option>
                    <option value="unknown" selected>Unknown</option>
                </select>
            </div>
            
            <div class="button-group">
                <button class="btn-start" id="start-btn" onclick="startRecording()">▶ Start Recording</button>
                <button class="btn-stop" id="stop-btn" onclick="stopRecording()" disabled>⏹ Stop Recording</button>
            </div>
            
            <div class="status-box" id="status-box">
                <div class="status-label">Recording Status</div>
                <div class="status-value" id="status-text">🔴 Not Recording</div>
            </div>
            
            <div class="status-box" id="filename-box" style="display: none;">
                <div class="status-label">Current File</div>
                <div class="status-value" id="filename-text">-</div>
            </div>
        </div>
    </div>
    
    <script>
        // Chart.js Configuration
        const ctx = document.getElementById('imuChart').getContext('2d');
        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Roll (°)',
                        data: [],
                        borderColor: '#667eea',
                        backgroundColor: 'rgba(102, 126, 234, 0.1)',
                        borderWidth: 2,
                        tension: 0.4,
                        pointRadius: 0,
                        pointHoverRadius: 4
                    },
                    {
                        label: 'Pitch (°)',
                        data: [],
                        borderColor: '#f093fb',
                        backgroundColor: 'rgba(240, 147, 251, 0.1)',
                        borderWidth: 2,
                        tension: 0.4,
                        pointRadius: 0,
                        pointHoverRadius: 4
                    },
                    {
                        label: 'Yaw (°)',
                        data: [],
                        borderColor: '#4facfe',
                        backgroundColor: 'rgba(79, 172, 254, 0.1)',
                        borderWidth: 2,
                        tension: 0.4,
                        pointRadius: 0,
                        pointHoverRadius: 4
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: 'index',
                    intersect: false
                },
                plugins: {
                    legend: {
                        position: 'top',
                        labels: {
                            boxWidth: 15,
                            padding: 15,
                            font: { size: 12, weight: 'bold' }
                        }
                    }
                },
                scales: {
                    y: {
                        beginAtZero: true,
                        min: -180,
                        max: 180,
                        grid: {
                            color: 'rgba(0, 0, 0, 0.05)'
                        }
                    },
                    x: {
                        grid: {
                            display: false
                        }
                    }
                }
            }
        });
        
        // Update data every 100ms
        async function updateData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                
                const current = data.current;
                
                document.getElementById('roll-display').textContent = current.roll + '°';
                document.getElementById('pitch-display').textContent = current.pitch + '°';
                document.getElementById('yaw-display').textContent = current.yaw + '°';
                
                document.getElementById('ax-display').textContent = current.ax.toFixed(4);
                document.getElementById('ay-display').textContent = current.ay.toFixed(4);
                document.getElementById('az-display').textContent = current.az.toFixed(4);
                document.getElementById('gx-display').textContent = current.gx.toFixed(2);
                document.getElementById('gy-display').textContent = current.gy.toFixed(2);
                document.getElementById('gz-display').textContent = current.gz.toFixed(2);
                
                const history = data.history;
                chart.data.labels = history.timestamps;
                chart.data.datasets[0].data = history.roll;
                chart.data.datasets[1].data = history.pitch;
                chart.data.datasets[2].data = history.yaw;
                chart.update('none');
                
            } catch (error) {
                console.error('Error fetching data:', error);
            }
        }
        
        async function updateRecordingStatus() {
            try {
                const response = await fetch('/recording_status');
                const status = await response.json();
                
                const statusBox = document.getElementById('status-box');
                const statusText = document.getElementById('status-text');
                const filenameBox = document.getElementById('filename-box');
                const filenameText = document.getElementById('filename-text');
                const startBtn = document.getElementById('start-btn');
                const stopBtn = document.getElementById('stop-btn');
                
                if (status.is_recording) {
                    statusText.textContent = '🟢 Recording...';
                    statusBox.classList.remove('recording-inactive');
                    statusBox.classList.add('recording-active');
                    filenameText.textContent = status.filename;
                    filenameBox.style.display = 'block';
                    startBtn.disabled = true;
                    stopBtn.disabled = false;
                } else {
                    statusText.textContent = '🔴 Not Recording';
                    statusBox.classList.remove('recording-active');
                    statusBox.classList.add('recording-inactive');
                    filenameBox.style.display = 'none';
                    startBtn.disabled = false;
                    stopBtn.disabled = true;
                }
            } catch (error) {
                console.error('Error updating recording status:', error);
            }
        }
        
        async function startRecording() {
            const label = document.getElementById('label-select').value;
            
            try {
                const response = await fetch('/start_recording', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ label: label })
                });
                
                const result = await response.json();
                
                if (result.success) {
                    updateRecordingStatus();
                } else {
                    alert('Error: ' + result.message);
                }
            } catch (error) {
                alert('Failed to start recording: ' + error);
            }
        }
        
        async function stopRecording() {
            try {
                const response = await fetch('/stop_recording', {
                    method: 'POST'
                });
                
                const result = await response.json();
                
                if (result.success) {
                    updateRecordingStatus();
                } else {
                    alert('Error: ' + result.message);
                }
            } catch (error) {
                alert('Failed to stop recording: ' + error);
            }
        }
        
        // Calibration functions
        async function startCalibration() {
            try {
                const response = await fetch('/start_calibration', {
                    method: 'POST'
                });
                
                const result = await response.json();
                
                if (result.success) {
                    document.getElementById('calibration-progress').style.display = 'block';
                    document.getElementById('calibrate-btn').disabled = true;
                    document.getElementById('reset-calib-btn').disabled = true;
                    
                    const progressInterval = setInterval(async () => {
                        const status = await getCalibrationStatus();
                        updateCalibrationUI(status);
                        
                        if (!status.is_calibrating) {
                            clearInterval(progressInterval);
                            document.getElementById('calibrate-btn').disabled = false;
                            document.getElementById('reset-calib-btn').disabled = false;
                        }
                    }, 100);
                } else {
                    alert('Error: ' + result.message);
                }
            } catch (error) {
                alert('Failed to start calibration: ' + error);
            }
        }
        
        async function getCalibrationStatus() {
            try {
                const response = await fetch('/calibration_status');
                return await response.json();
            } catch (error) {
                console.error('Error getting calibration status:', error);
                return {};
            }
        }
        
        function updateCalibrationUI(status) {
            if (!status) return;
            
            const statusBox = document.getElementById('calib-status-box');
            const statusText = document.getElementById('calib-status-text');
            const progressBar = document.getElementById('calib-progress-bar');
            const progressText = document.getElementById('calib-progress-text');
            const offsetsBox = document.getElementById('calib-offsets-box');
            
            progressText.textContent = status.progress + '%';
            progressBar.style.width = status.progress + '%';
            
            if (status.is_calibrating) {
                statusText.textContent = '🟡 Calibrating...';
                statusBox.classList.remove('recording-inactive');
                statusBox.classList.add('recording-active');
            } else if (status.is_calibrated) {
                statusText.textContent = '✅ Calibrated';
                statusBox.classList.remove('recording-inactive');
                statusBox.classList.add('recording-active');
                document.getElementById('calibration-progress').style.display = 'none';
            } else {
                statusText.textContent = '❌ Not Calibrated';
                statusBox.classList.add('recording-inactive');
                statusBox.classList.remove('recording-active');
            }
            
            if (status.is_calibrated) {
                offsetsBox.style.display = 'block';
                document.getElementById('accel-x').textContent = status.accel_offset.x.toFixed(4);
                document.getElementById('accel-y').textContent = status.accel_offset.y.toFixed(4);
                document.getElementById('accel-z').textContent = status.accel_offset.z.toFixed(4);
                document.getElementById('gyro-x').textContent = status.gyro_offset.x.toFixed(2);
                document.getElementById('gyro-y').textContent = status.gyro_offset.y.toFixed(2);
                document.getElementById('gyro-z').textContent = status.gyro_offset.z.toFixed(2);
            } else {
                offsetsBox.style.display = 'none';
            }
        }
        
        async function resetCalibration() {
            if (confirm('Reset calibration to defaults? This will remove all calibration offsets.')) {
                try {
                    const response = await fetch('/reset_calibration', {
                        method: 'POST'
                    });
                    
                    const result = await response.json();
                    
                    if (result.success) {
                        await updateCalibrationStatus();
                    } else {
                        alert('Error: ' + result.message);
                    }
                } catch (error) {
                    alert('Failed to reset calibration: ' + error);
                }
            }
        }
        
        async function updateCalibrationStatus() {
            const status = await getCalibrationStatus();
            updateCalibrationUI(status);
        }
        
        window.addEventListener('load', function() {
            updateRecordingStatus();
            updateCalibrationStatus();
            setInterval(updateData, 100);
            setInterval(updateRecordingStatus, 500);
            setInterval(updateCalibrationStatus, 1000);
        });
    </script>
</body>
</html>
"""

# ==================== Main Entry Point ====================
if __name__ == '__main__':
    print("\n" + "="*60)
    print("Raspberry Pi IMU Monitor - Simulation Mode")
    print("="*60)
    print("\n✓ Simulated data generator active")
    
    # Start data reading thread
    reader_thread = threading.Thread(target=data_reader_thread, daemon=True)
    reader_thread.start()
    
    print(f"✓ Flask server starting on http://0.0.0.0:5000")
    print("✓ Press Ctrl+C to stop\n")
    print("📝 Note: This is SIMULATED data. Replace with real MPU6050 code when deploying to Raspberry Pi.\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        should_exit = True
        print("\n\n► Shutting down...")
        
        # Stop recording if active
        if recording_state['is_recording']:
            stop_recording()
        
        print("✓ Goodbye!")
