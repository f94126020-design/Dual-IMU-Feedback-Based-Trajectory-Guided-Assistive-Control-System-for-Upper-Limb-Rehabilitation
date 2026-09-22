#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi Dual IMU Monitor with Pose Estimation
- Two MPU6050 sensors via TCA9548A multiplexer
- Real-time pose angle calculation (YZ angle, shoulder raise, elbow angle)
- Motion state detection with EMG muscle prediction
- Web interface with real-time data display
- Forearm IMU (Channel 1) at 0x68
- Raspberry Pi I2C Bus 1
"""

import os
import math
import time
import threading
import csv
import struct
from datetime import datetime
from collections import deque
from flask import Flask, jsonify, request, render_template_string

# Try to import smbus2 for direct I2C access
try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    print("✗ smbus2 library not found. Install with: pip install smbus2")
    SMBUS_AVAILABLE = False

# ==================== Configuration ====================
I2C_BUS = 1
MPU_ADDRESS = 0x68  # MPU6050 standard address
TCA_ADDRESS = 0x70  # TCA9548A multiplexer address
DATA_UPDATE_INTERVAL = 0.1  # 100 ms (10 Hz sampling)
MAX_DATA_POINTS = 100
DATA_FOLDER = "data"

# Multi-IMU Configuration
IMU_CHANNELS = {
    'upper_arm': 0,   # Channel 0: Upper arm on shoulder
    'forearm': 1      # Channel 1: Forearm on elbow
}

# ==================== Pose Estimation Constants ====================
# Sign corrections for angle directions (set to -1 to reverse)
UPPER_ARM_SIGN = 1      # Upper arm YZ angle sign
FOREARM_SIGN = 1        # Forearm YZ angle sign
ELBOW_SIGN = 1          # Elbow angle sign (forearm relative to upper arm)

# Motion detection thresholds
ANGLE_THRESHOLD = 5.0       # degree - minimum angle change
VELOCITY_THRESHOLD = 8.0    # deg/s - minimum velocity for motion detection

# Calibration settings
GYRO_CALIBRATION_SAMPLES = 100  # 10 seconds at 10 Hz
POSE_CALIBRATION_MODE = "manual"  # 'manual' or 'auto'

# ==================== MPU6050 Direct I2C Reader ====================
class MPU6050Direct:
    """Direct I2C reader for MPU6050 with TCA9548A support"""
    
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    WHO_AM_I = 0x75
    
    ACCEL_SCALE = 16384.0  # ±2g
    GYRO_SCALE = 131.0     # ±250°/s
    
    def __init__(self, bus_num=1, tca_addr=None, mpu_addr=0x68, channel=None):
        self.bus = smbus2.SMBus(bus_num)
        self.mpu_addr = mpu_addr
        self.tca_addr = tca_addr
        self.channel = channel
        self.bus_num = bus_num
        
    def select_channel(self, channel):
        """Select TCA9548A channel (must be called with i2c_lock held)"""
        if self.tca_addr is None or channel is None:
            return True
        
        try:
            self.bus.write_byte(self.tca_addr, 1 << channel)
            return True
        except Exception as e:
            print(f"✗ Failed to select TCA channel {channel}: {e}")
            return False
    
    def init_mpu(self):
        """Initialize MPU6050"""
        self.select_channel(self.channel)
        
        try:
            # Wake up MPU
            self.bus.write_byte_data(self.mpu_addr, self.PWR_MGMT_1, 0x00)
            time.sleep(0.05)
            
            # Verify WHO_AM_I
            who = self.bus.read_byte_data(self.mpu_addr, self.WHO_AM_I)
            if who != 0x68:
                print(f"✗ WHO_AM_I mismatch: got 0x{who:02x}, expected 0x68")
                return False
            
            # Configure
            configs = [
                (0x19, 9),      # Sample rate divider = 100 Hz
                (0x1A, 0x06),   # DLPF config
                (0x1C, 0x00),   # Accel config ±2g
                (0x1B, 0x00)    # Gyro config ±250°/s
            ]
            
            for reg, val in configs:
                self.bus.write_byte_data(self.mpu_addr, reg, val)
            
            time.sleep(0.05)
            return True
            
        except Exception as e:
            print(f"✗ MPU6050 initialization failed: {e}")
            return False
    
    def read_raw_data(self, reg, count):
        """Read raw data from register (must be called with i2c_lock held)"""
        try:
            self.select_channel(self.channel)
            data = self.bus.read_i2c_block_data(self.mpu_addr, reg, count)
            return data
        except Exception as e:
            print(f"✗ Failed to read from register 0x{reg:02x}: {e}")
            return None
    
    def get_data(self):
        """Read accelerometer and gyroscope data (must be called with i2c_lock held)"""
        accel_data = self.read_raw_data(self.ACCEL_XOUT_H, 6)
        if accel_data is None:
            return None
        
        gyro_data = self.read_raw_data(self.GYRO_XOUT_H, 6)
        if gyro_data is None:
            return None
        
        try:
            ax_raw = struct.unpack('>h', bytes(accel_data[0:2]))[0]
            ay_raw = struct.unpack('>h', bytes(accel_data[2:4]))[0]
            az_raw = struct.unpack('>h', bytes(accel_data[4:6]))[0]
            
            gx_raw = struct.unpack('>h', bytes(gyro_data[0:2]))[0]
            gy_raw = struct.unpack('>h', bytes(gyro_data[2:4]))[0]
            gz_raw = struct.unpack('>h', bytes(gyro_data[4:6]))[0]
            
            return {
                'x': ax_raw / self.ACCEL_SCALE,
                'y': ay_raw / self.ACCEL_SCALE,
                'z': az_raw / self.ACCEL_SCALE,
                'gx': gx_raw / self.GYRO_SCALE,
                'gy': gy_raw / self.GYRO_SCALE,
                'gz': gz_raw / self.GYRO_SCALE
            }
        except Exception as e:
            print(f"✗ Error converting data: {e}")
            return None

# ==================== Global Variables ====================
app = Flask(__name__)

# I2C synchronization lock - ensures atomic channel selection + read
i2c_lock = threading.Lock()

# IMU Data Storage - supports multiple IMUs
imu_data = {
    'upper_arm': {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'yz_angle': 0.0,
        'timestamp': 0, 'error': None
    },
    'forearm': {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'yz_angle': 0.0,
        'timestamp': 0, 'error': None
    }
}

# Pose estimation data
pose_data = {
    'upper_arm_raise_angle': 0.0,
    'upper_arm_raise_velocity': 0.0,
    'elbow_angle': 0.0,
    'elbow_velocity': 0.0,
    'motion_state': 'rest',
    'expected_muscle': 'unknown',
    'timestamp': 0
}

# Previous pose values for velocity calculation
previous_pose_data = {
    'upper_arm_raise_angle': 0.0,
    'elbow_angle': 0.0,
    'timestamp': time.time()
}

# Historical data for sensors (last 100 points)
history_data = {
    'upper_arm': {
        'roll': deque(maxlen=MAX_DATA_POINTS),
        'pitch': deque(maxlen=MAX_DATA_POINTS),
        'yz_angle': deque(maxlen=MAX_DATA_POINTS),
    },
    'forearm': {
        'roll': deque(maxlen=MAX_DATA_POINTS),
        'pitch': deque(maxlen=MAX_DATA_POINTS),
        'yz_angle': deque(maxlen=MAX_DATA_POINTS),
    },
    'pose': {
        'upper_arm_raise_angle': deque(maxlen=MAX_DATA_POINTS),
        'upper_arm_raise_velocity': deque(maxlen=MAX_DATA_POINTS),
        'elbow_angle': deque(maxlen=MAX_DATA_POINTS),
        'elbow_velocity': deque(maxlen=MAX_DATA_POINTS),
    }
}

# Calibration state
calibration_state = {
    'upper_arm': {
        'is_calibrating_gyro': False,
        'is_calibrating_pose': False,
        'progress': 0,
        'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'is_gyro_calibrated': False,
        'yz_zero': 0.0,
        'roll_zero': 0.0,
        'pitch_zero': 0.0,
        'is_pose_calibrated': False
    },
    'forearm': {
        'is_calibrating_gyro': False,
        'is_calibrating_pose': False,
        'progress': 0,
        'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'is_gyro_calibrated': False,
        'yz_zero': 0.0,
        'roll_zero': 0.0,
        'pitch_zero': 0.0,
        'is_pose_calibrated': False
    }
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

# Thread control
data_lock = threading.Lock()
mpu_upper_arm = None  # TCA channel 0
mpu_forearm = None    # TCA channel 1
should_exit = False

# ==================== MPU6050 Initialization ====================
def init_mpu():
    """Initialize multiple MPU6050 sensors"""
    global mpu_upper_arm, mpu_forearm
    
    if not SMBUS_AVAILABLE:
        print("✗ smbus2 not available")
        return False
    
    print("► Initializing IMU sensors...")
    initialized_count = 0
    
    # Initialize upper arm IMU
    try:
        upper_arm_imu = MPU6050Direct(I2C_BUS, TCA_ADDRESS, MPU_ADDRESS, IMU_CHANNELS['upper_arm'])
        if upper_arm_imu.init_mpu():
            mpu_upper_arm = upper_arm_imu
            print(f"✓ Upper Arm IMU initialized at TCA channel {IMU_CHANNELS['upper_arm']}")
            initialized_count += 1
        else:
            print(f"⚠ Upper Arm IMU (channel 0) not found - continuing without it")
    except Exception as e:
        print(f"⚠ Upper Arm IMU (channel 0) error: {e} - continuing without it")
    
    # Initialize forearm IMU
    try:
        forearm_imu = MPU6050Direct(I2C_BUS, TCA_ADDRESS, MPU_ADDRESS, IMU_CHANNELS['forearm'])
        if forearm_imu.init_mpu():
            mpu_forearm = forearm_imu
            print(f"✓ Forearm IMU initialized at TCA channel {IMU_CHANNELS['forearm']}")
            initialized_count += 1
        else:
            print(f"⚠ Forearm IMU (channel 1) not found - continuing without it")
    except Exception as e:
        print(f"⚠ Forearm IMU (channel 1) error: {e} - continuing without it")
    
    if initialized_count == 0:
        print("✗ No IMU sensors initialized")
        return False
    
    print(f"✓ {initialized_count} IMU sensor(s) initialized successfully")
    return True

# ==================== Angle Calculations ====================
def calculate_angles(ax, ay, az):
    """
    Calculate roll and pitch from accelerometer data
    
    roll = atan2(ay, az) * 180 / pi
    pitch = atan2(-ax, sqrt(ay^2 + az^2)) * 180 / pi
    """
    roll = math.atan2(ay, az) * 180 / math.pi
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return roll, pitch

def calculate_yz_angle(ay, az):
    """
    Calculate Y-Z plane angle from accelerometer
    
    yz_angle = atan2(ay, az) * 180 / pi
    
    This angle represents the inclination of the IMU in the Y-Z plane.
    Important: Set Pose Zero when Y-axis is vertical (perpendicular to ground).
    """
    return math.atan2(ay, az) * 180 / math.pi

def calculate_pose(upper_arm_yz, forearm_yz, 
                   upper_arm_yz_zero, forearm_yz_zero,
                   previous_upper_arm_raise, previous_elbow_angle, dt):
    """
    Calculate human pose angles and velocities
    
    Parameters:
    - upper_arm_yz: current Y-Z angle of upper arm IMU
    - forearm_yz: current Y-Z angle of forearm IMU
    - upper_arm_yz_zero: calibrated zero position of upper arm Y-Z
    - forearm_yz_zero: calibrated zero position of forearm Y-Z
    - dt: time delta since last update (seconds)
    
    Returns: dict with pose angles and velocities
    """
    # Shoulder raise angle: positive when arm is raised
    ua_yz_zeroed = upper_arm_yz_zero - upper_arm_yz
    upper_arm_raise_angle = UPPER_ARM_SIGN * ua_yz_zeroed
    upper_arm_raise_velocity = (upper_arm_raise_angle - previous_upper_arm_raise) / dt if dt > 0 else 0
    
    # Elbow angle: positive when flexed (arm bends)
    fa_yz_zeroed = forearm_yz_zero - forearm_yz
    elbow_angle = ELBOW_SIGN * (fa_yz_zeroed - ua_yz_zeroed)
    elbow_velocity = (elbow_angle - previous_elbow_angle) / dt if dt > 0 else 0
    
    # Determine motion state based on thresholds
    if (upper_arm_raise_angle > ANGLE_THRESHOLD and 
        upper_arm_raise_velocity > VELOCITY_THRESHOLD):
        motion_state = "shoulder_raise"
        expected_muscle = "deltoid"
    elif upper_arm_raise_velocity < -VELOCITY_THRESHOLD:
        motion_state = "shoulder_lower"
        expected_muscle = "unknown"
    elif elbow_velocity > VELOCITY_THRESHOLD:
        motion_state = "elbow_flexion"
        expected_muscle = "biceps"
    elif elbow_velocity < -VELOCITY_THRESHOLD:
        motion_state = "elbow_extension"
        expected_muscle = "triceps"
    elif (abs(elbow_velocity) <= VELOCITY_THRESHOLD and 
          abs(upper_arm_raise_velocity) <= VELOCITY_THRESHOLD):
        motion_state = "hold"
        expected_muscle = "unknown"
    else:
        motion_state = "rest"
        expected_muscle = "unknown"
    
    return {
        'upper_arm_raise_angle': upper_arm_raise_angle,
        'upper_arm_raise_velocity': upper_arm_raise_velocity,
        'elbow_angle': elbow_angle,
        'elbow_velocity': elbow_velocity,
        'motion_state': motion_state,
        'expected_muscle': expected_muscle
    }

def _read_imu_raw(mpu_sensor):
    """Helper function to read raw IMU data (must be called with i2c_lock held)"""
    if mpu_sensor is None:
        return None
    
    try:
        if isinstance(mpu_sensor, MPU6050Direct):
            data = mpu_sensor.get_data()
        else:
            data = None
        return data
    except Exception as e:
        print(f"✗ Error reading IMU: {e}")
        return None

# ==================== IMU Data Reading ====================
def read_imu_data():
    """Read data from both IMU sensors with pose estimation"""
    global imu_data, mpu_upper_arm, mpu_forearm, history_data, recording_state
    global data_lock, calibration_state, pose_data, previous_pose_data
    
    if mpu_upper_arm is None and mpu_forearm is None:
        return
    
    # Read all I2C data OUTSIDE the lock to avoid blocking Flask requests
    raw_data_upper = None
    raw_data_forearm = None
    
    with i2c_lock:
        if mpu_upper_arm is not None:
            raw_data_upper = _read_imu_raw(mpu_upper_arm)
        
        if mpu_forearm is not None:
            raw_data_forearm = _read_imu_raw(mpu_forearm)
    
    # Now hold lock only for updating shared data
    with data_lock:
        current_time = time.time()
        dt = current_time - previous_pose_data['timestamp']
        
        # Process Upper Arm IMU
        if raw_data_upper is not None:
            ax = raw_data_upper['x'] - calibration_state['upper_arm']['gyro_offset']['x']
            ay = raw_data_upper['y'] - calibration_state['upper_arm']['gyro_offset']['y']
            az = raw_data_upper['z'] - calibration_state['upper_arm']['gyro_offset']['z']
            
            gx = raw_data_upper['gx'] - calibration_state['upper_arm']['gyro_offset']['x']
            gy = raw_data_upper['gy'] - calibration_state['upper_arm']['gyro_offset']['y']
            gz = raw_data_upper['gz'] - calibration_state['upper_arm']['gyro_offset']['z']
            
            roll, pitch = calculate_angles(ax, ay, az)
            yz_angle = calculate_yz_angle(ay, az)
            yaw = gz
            
            imu_data['upper_arm'].update({
                'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
                'gx': round(gx, 2), 'gy': round(gy, 2), 'gz': round(gz, 2),
                'roll': round(roll, 2), 'pitch': round(pitch, 2), 'yaw': round(yaw, 2),
                'yz_angle': round(yz_angle, 2),
                'timestamp': current_time, 'error': None
            })
            
            history_data['upper_arm']['roll'].append(roll)
            history_data['upper_arm']['pitch'].append(pitch)
            history_data['upper_arm']['yz_angle'].append(yz_angle)
        
        # Process Forearm IMU
        if raw_data_forearm is not None:
            ax = raw_data_forearm['x'] - calibration_state['forearm']['gyro_offset']['x']
            ay = raw_data_forearm['y'] - calibration_state['forearm']['gyro_offset']['y']
            az = raw_data_forearm['z'] - calibration_state['forearm']['gyro_offset']['z']
            
            gx = raw_data_forearm['gx'] - calibration_state['forearm']['gyro_offset']['x']
            gy = raw_data_forearm['gy'] - calibration_state['forearm']['gyro_offset']['y']
            gz = raw_data_forearm['gz'] - calibration_state['forearm']['gyro_offset']['z']
            
            roll, pitch = calculate_angles(ax, ay, az)
            yz_angle = calculate_yz_angle(ay, az)
            yaw = gz
            
            imu_data['forearm'].update({
                'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
                'gx': round(gx, 2), 'gy': round(gy, 2), 'gz': round(gz, 2),
                'roll': round(roll, 2), 'pitch': round(pitch, 2), 'yaw': round(yaw, 2),
                'yz_angle': round(yz_angle, 2),
                'timestamp': current_time, 'error': None
            })
            
            history_data['forearm']['roll'].append(roll)
            history_data['forearm']['pitch'].append(pitch)
            history_data['forearm']['yz_angle'].append(yz_angle)
        
        # Calculate pose if both calibrations are done
        if (calibration_state['upper_arm']['is_pose_calibrated'] and 
            calibration_state['forearm']['is_pose_calibrated']):
            
            new_pose = calculate_pose(
                imu_data['upper_arm']['yz_angle'],
                imu_data['forearm']['yz_angle'],
                calibration_state['upper_arm']['yz_zero'],
                calibration_state['forearm']['yz_zero'],
                previous_pose_data['upper_arm_raise_angle'],
                previous_pose_data['elbow_angle'],
                dt
            )
            
            pose_data.update(new_pose)
            pose_data['timestamp'] = current_time
            
            # Update history
            history_data['pose']['upper_arm_raise_angle'].append(new_pose['upper_arm_raise_angle'])
            history_data['pose']['upper_arm_raise_velocity'].append(new_pose['upper_arm_raise_velocity'])
            history_data['pose']['elbow_angle'].append(new_pose['elbow_angle'])
            history_data['pose']['elbow_velocity'].append(new_pose['elbow_velocity'])
            
            # Update previous values
            previous_pose_data['upper_arm_raise_angle'] = new_pose['upper_arm_raise_angle']
            previous_pose_data['elbow_angle'] = new_pose['elbow_angle']
            previous_pose_data['timestamp'] = current_time
        
        # Write to CSV if recording
        if recording_state['is_recording'] and recording_state['csv_writer']:
            try:
                elapsed = current_time - recording_state['start_time']
                recording_state['csv_writer'].writerow([
                    f"{elapsed:.3f}",
                    imu_data['upper_arm']['ax'], imu_data['upper_arm']['ay'], imu_data['upper_arm']['az'],
                    imu_data['upper_arm']['gx'], imu_data['upper_arm']['gy'], imu_data['upper_arm']['gz'],
                    imu_data['upper_arm']['roll'], imu_data['upper_arm']['pitch'], imu_data['upper_arm']['yaw'],
                    imu_data['upper_arm']['yz_angle'],
                    imu_data['forearm']['ax'], imu_data['forearm']['ay'], imu_data['forearm']['az'],
                    imu_data['forearm']['gx'], imu_data['forearm']['gy'], imu_data['forearm']['gz'],
                    imu_data['forearm']['roll'], imu_data['forearm']['pitch'], imu_data['forearm']['yaw'],
                    imu_data['forearm']['yz_angle'],
                    pose_data['upper_arm_raise_angle'],
                    pose_data['upper_arm_raise_velocity'],
                    pose_data['elbow_angle'],
                    pose_data['elbow_velocity'],
                    pose_data['motion_state'],
                    pose_data['expected_muscle'],
                    recording_state['label']
                ])
                recording_state['csv_file'].flush()
            except Exception as e:
                print(f"✗ CSV write error: {e}")

def data_reader_thread():
    """Background thread for reading IMU data"""
    global should_exit, mpu_upper_arm, mpu_forearm
    
    if mpu_upper_arm is None and mpu_forearm is None:
        print("⚠ No IMU available, running in simulation mode")
        return
    
    print("► Data reader thread started")
    
    while not should_exit:
        read_imu_data()
        time.sleep(DATA_UPDATE_INTERVAL)
    
    print("► Data reader thread stopped")

# ==================== Calibration Functions ====================
def calibrate_gyro(imu_name='all'):
    """Calibrate gyroscope bias by averaging stationary readings"""
    global calibration_state, mpu_upper_arm, mpu_forearm, data_lock, i2c_lock
    
    if imu_name not in ['upper_arm', 'forearm', 'all']:
        return {"success": False, "message": f"Invalid IMU name: {imu_name}"}
    
    print(f"► Starting gyro calibration for {imu_name}...")
    
    # Calibrate upper arm
    if imu_name in ['upper_arm', 'all'] and mpu_upper_arm is not None:
        try:
            gyro_sum = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            for i in range(GYRO_CALIBRATION_SAMPLES):
                with i2c_lock:
                    data = _read_imu_raw(mpu_upper_arm)
                
                if data:
                    gyro_sum['x'] += data['gx']
                    gyro_sum['y'] += data['gy']
                    gyro_sum['z'] += data['gz']
                
                with data_lock:
                    calibration_state['upper_arm']['progress'] = int((i + 1) / GYRO_CALIBRATION_SAMPLES * 100)
                
                time.sleep(DATA_UPDATE_INTERVAL)
            
            with data_lock:
                calibration_state['upper_arm']['gyro_offset']['x'] = gyro_sum['x'] / GYRO_CALIBRATION_SAMPLES
                calibration_state['upper_arm']['gyro_offset']['y'] = gyro_sum['y'] / GYRO_CALIBRATION_SAMPLES
                calibration_state['upper_arm']['gyro_offset']['z'] = gyro_sum['z'] / GYRO_CALIBRATION_SAMPLES
                calibration_state['upper_arm']['is_gyro_calibrated'] = True
                calibration_state['upper_arm']['is_calibrating_gyro'] = False
                calibration_state['upper_arm']['progress'] = 100
            
            print("✓ Upper Arm gyro calibration completed")
        except Exception as e:
            print(f"✗ Upper arm gyro calibration error: {e}")
            with data_lock:
                calibration_state['upper_arm']['is_calibrating_gyro'] = False
    
    # Calibrate forearm
    if imu_name in ['forearm', 'all'] and mpu_forearm is not None:
        try:
            gyro_sum = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            for i in range(GYRO_CALIBRATION_SAMPLES):
                with i2c_lock:
                    data = _read_imu_raw(mpu_forearm)
                
                if data:
                    gyro_sum['x'] += data['gx']
                    gyro_sum['y'] += data['gy']
                    gyro_sum['z'] += data['gz']
                
                with data_lock:
                    calibration_state['forearm']['progress'] = int((i + 1) / GYRO_CALIBRATION_SAMPLES * 100)
                
                time.sleep(DATA_UPDATE_INTERVAL)
            
            with data_lock:
                calibration_state['forearm']['gyro_offset']['x'] = gyro_sum['x'] / GYRO_CALIBRATION_SAMPLES
                calibration_state['forearm']['gyro_offset']['y'] = gyro_sum['y'] / GYRO_CALIBRATION_SAMPLES
                calibration_state['forearm']['gyro_offset']['z'] = gyro_sum['z'] / GYRO_CALIBRATION_SAMPLES
                calibration_state['forearm']['is_gyro_calibrated'] = True
                calibration_state['forearm']['is_calibrating_gyro'] = False
                calibration_state['forearm']['progress'] = 100
            
            print("✓ Forearm gyro calibration completed")
        except Exception as e:
            print(f"✗ Forearm gyro calibration error: {e}")
            with data_lock:
                calibration_state['forearm']['is_calibrating_gyro'] = False
    
    return {"success": True, "message": f"Gyro calibration completed for {imu_name}"}

def set_pose_zero(imu_name='all'):
    """Set current pose as zero reference point"""
    global calibration_state, imu_data, data_lock
    
    if imu_name not in ['upper_arm', 'forearm', 'all']:
        return {"success": False, "message": f"Invalid IMU name: {imu_name}"}
    
    with data_lock:
        if imu_name in ['upper_arm', 'all']:
            calibration_state['upper_arm']['yz_zero'] = imu_data['upper_arm']['yz_angle']
            calibration_state['upper_arm']['roll_zero'] = imu_data['upper_arm']['roll']
            calibration_state['upper_arm']['pitch_zero'] = imu_data['upper_arm']['pitch']
            calibration_state['upper_arm']['is_pose_calibrated'] = True
            print(f"✓ Upper Arm pose zero set: yz={calibration_state['upper_arm']['yz_zero']:.2f}°")
        
        if imu_name in ['forearm', 'all']:
            calibration_state['forearm']['yz_zero'] = imu_data['forearm']['yz_angle']
            calibration_state['forearm']['roll_zero'] = imu_data['forearm']['roll']
            calibration_state['forearm']['pitch_zero'] = imu_data['forearm']['pitch']
            calibration_state['forearm']['is_pose_calibrated'] = True
            print(f"✓ Forearm pose zero set: yz={calibration_state['forearm']['yz_zero']:.2f}°")
    
    return {"success": True, "message": f"Pose zero set for {imu_name}"}

def reset_calibration(imu_name='all'):
    """Reset calibration to default values"""
    global calibration_state, data_lock
    
    if imu_name not in ['upper_arm', 'forearm', 'all']:
        return {"success": False, "message": f"Invalid IMU name: {imu_name}"}
    
    with data_lock:
        if imu_name in ['upper_arm', 'all']:
            calibration_state['upper_arm'].update({
                'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'is_gyro_calibrated': False,
                'yz_zero': 0.0,
                'roll_zero': 0.0,
                'pitch_zero': 0.0,
                'is_pose_calibrated': False,
                'progress': 0
            })
        
        if imu_name in ['forearm', 'all']:
            calibration_state['forearm'].update({
                'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'is_gyro_calibrated': False,
                'yz_zero': 0.0,
                'roll_zero': 0.0,
                'pitch_zero': 0.0,
                'is_pose_calibrated': False,
                'progress': 0
            })
    
    print(f"✓ Calibration reset for {imu_name}")
    return {"success": True, "message": f"Calibration reset for {imu_name}"}

# ==================== Recording Functions ====================
def start_recording(label):
    """Start recording IMU data to CSV"""
    global recording_state, data_lock
    
    with data_lock:
        if recording_state['is_recording']:
            return {"success": False, "message": "Already recording"}
        
        os.makedirs(DATA_FOLDER, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(DATA_FOLDER, f"imu_data_{timestamp}_{label}.csv")
        
        try:
            recording_state['csv_file'] = open(filename, 'w', newline='')
            recording_state['csv_writer'] = csv.writer(recording_state['csv_file'])
            
            # Write header
            recording_state['csv_writer'].writerow([
                'time',
                'ua_ax', 'ua_ay', 'ua_az', 'ua_gx', 'ua_gy', 'ua_gz',
                'ua_roll', 'ua_pitch', 'ua_yaw', 'ua_yz_angle',
                'fa_ax', 'fa_ay', 'fa_az', 'fa_gx', 'fa_gy', 'fa_gz',
                'fa_roll', 'fa_pitch', 'fa_yaw', 'fa_yz_angle',
                'upper_arm_raise_angle', 'upper_arm_raise_velocity',
                'elbow_angle', 'elbow_velocity',
                'motion_state', 'expected_muscle', 'label'
            ])
            
            recording_state['is_recording'] = True
            recording_state['start_time'] = time.time()
            recording_state['filename'] = filename
            recording_state['label'] = label
            
            print(f"✓ Recording started: {filename}")
            return {"success": True, "message": f"Recording started: {filename}"}
        
        except Exception as e:
            print(f"✗ Failed to start recording: {e}")
            return {"success": False, "message": f"Failed to start recording: {e}"}

def stop_recording():
    """Stop recording IMU data"""
    global recording_state, data_lock
    
    with data_lock:
        if not recording_state['is_recording']:
            return {"success": False, "message": "Not recording"}
        
        try:
            if recording_state['csv_file']:
                recording_state['csv_file'].close()
            
            filename = recording_state['filename']
            recording_state['is_recording'] = False
            recording_state['csv_file'] = None
            recording_state['csv_writer'] = None
            
            print(f"✓ Recording stopped: {filename}")
            return {"success": True, "message": f"Recording stopped: {filename}"}
        
        except Exception as e:
            print(f"✗ Failed to stop recording: {e}")
            return {"success": False, "message": f"Failed to stop recording: {e}"}

def get_recording_status():
    """Get recording status"""
    with data_lock:
        if recording_state['is_recording']:
            elapsed = time.time() - recording_state['start_time']
            return {
                "is_recording": True,
                "filename": recording_state['filename'],
                "elapsed": round(elapsed, 1),
                "label": recording_state['label']
            }
        else:
            return {
                "is_recording": False,
                "filename": None,
                "elapsed": 0,
                "label": None
            }

def get_calibration_status(imu_name='all'):
    """Get calibration status for specified IMU(s)"""
    with data_lock:
        if imu_name == 'upper_arm':
            cal = calibration_state['upper_arm']
            return {
                "imu": "upper_arm",
                "is_gyro_calibrated": cal['is_gyro_calibrated'],
                "is_pose_calibrated": cal['is_pose_calibrated'],
                "progress": cal['progress'],
                "yz_zero": round(cal['yz_zero'], 2)
            }
        elif imu_name == 'forearm':
            cal = calibration_state['forearm']
            return {
                "imu": "forearm",
                "is_gyro_calibrated": cal['is_gyro_calibrated'],
                "is_pose_calibrated": cal['is_pose_calibrated'],
                "progress": cal['progress'],
                "yz_zero": round(cal['yz_zero'], 2)
            }
        else:  # 'all'
            return {
                "upper_arm": {
                    "is_gyro_calibrated": calibration_state['upper_arm']['is_gyro_calibrated'],
                    "is_pose_calibrated": calibration_state['upper_arm']['is_pose_calibrated'],
                    "progress": calibration_state['upper_arm']['progress'],
                    "yz_zero": round(calibration_state['upper_arm']['yz_zero'], 2)
                },
                "forearm": {
                    "is_gyro_calibrated": calibration_state['forearm']['is_gyro_calibrated'],
                    "is_pose_calibrated": calibration_state['forearm']['is_pose_calibrated'],
                    "progress": calibration_state['forearm']['progress'],
                    "yz_zero": round(calibration_state['forearm']['yz_zero'], 2)
                }
            }

# ==================== Flask Routes ====================
@app.route('/')
def index():
    """Main web interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/data')
def get_data():
    """Get current IMU data, pose data, and history"""
    with data_lock:
        return jsonify({
            'upper_arm': {
                'current': imu_data['upper_arm'].copy(),
                'history': {
                    'timestamps': list(range(len(history_data['upper_arm']['yz_angle']))),
                    'yz_angle': list(history_data['upper_arm']['yz_angle']),
                    'roll': list(history_data['upper_arm']['roll']),
                    'pitch': list(history_data['upper_arm']['pitch']),
                }
            },
            'forearm': {
                'current': imu_data['forearm'].copy(),
                'history': {
                    'timestamps': list(range(len(history_data['forearm']['yz_angle']))),
                    'yz_angle': list(history_data['forearm']['yz_angle']),
                    'roll': list(history_data['forearm']['roll']),
                    'pitch': list(history_data['forearm']['pitch']),
                }
            },
            'pose': {
                'current': pose_data.copy(),
                'history': {
                    'timestamps': list(range(len(history_data['pose']['elbow_angle']))),
                    'upper_arm_raise_angle': list(history_data['pose']['upper_arm_raise_angle']),
                    'upper_arm_raise_velocity': list(history_data['pose']['upper_arm_raise_velocity']),
                    'elbow_angle': list(history_data['pose']['elbow_angle']),
                    'elbow_velocity': list(history_data['pose']['elbow_velocity']),
                }
            }
        })

@app.route('/start_recording', methods=['POST'])
def api_start_recording():
    """API endpoint to start recording"""
    label = request.json.get('label', 'unknown') if request.json else 'unknown'
    result = start_recording(label)
    return jsonify(result)

@app.route('/stop_recording', methods=['POST'])
def api_stop_recording():
    """API endpoint to stop recording"""
    result = stop_recording()
    return jsonify(result)

@app.route('/recording_status')
def api_recording_status():
    """Get recording status"""
    status = get_recording_status()
    return jsonify(status)

@app.route('/calibrate_gyro', methods=['POST'])
def api_calibrate_gyro():
    """API endpoint to calibrate gyroscope"""
    imu_name = request.json.get('imu_name', 'all') if request.json else 'all'
    
    # Run in background thread
    thread = threading.Thread(target=calibrate_gyro, args=(imu_name,), daemon=True)
    thread.start()
    
    return jsonify({"success": True, "message": f"Gyro calibration started for {imu_name}"})

@app.route('/set_pose_zero', methods=['POST'])
def api_set_pose_zero():
    """API endpoint to set current pose as zero"""
    imu_name = request.json.get('imu_name', 'all') if request.json else 'all'
    result = set_pose_zero(imu_name)
    return jsonify(result)

@app.route('/reset_calibration', methods=['POST'])
def api_reset_calibration():
    """API endpoint to reset calibration"""
    imu_name = request.json.get('imu_name', 'all') if request.json else 'all'
    result = reset_calibration(imu_name)
    return jsonify(result)

@app.route('/calibration_status')
def api_calibration_status():
    """Get calibration status for both IMUs"""
    imu_name = request.args.get('imu_name', 'all')
    status = get_calibration_status(imu_name)
    return jsonify(status)

# ==================== HTML Template ====================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dual IMU Pose Estimation</title>
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
            max-width: 1600px;
            margin: 0 auto;
        }
        
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.2);
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .card {
            background: white;
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2);
            transition: transform 0.3s ease;
        }
        
        .card:hover {
            transform: translateY(-5px);
        }
        
        .card h2 {
            color: #667eea;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        
        .card h3 {
            color: #764ba2;
            font-size: 1.1em;
            margin-top: 15px;
            margin-bottom: 10px;
        }
        
        .data-row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #f0f0f0;
        }
        
        .data-label {
            font-weight: 600;
            color: #555;
        }
        
        .data-value {
            color: #000;
            font-family: monospace;
            font-weight: bold;
        }
        
        .controls {
            display: flex;
            gap: 10px;
            margin-top: 15px;
            flex-wrap: wrap;
        }
        
        button {
            flex: 1;
            min-width: 120px;
            padding: 10px 15px;
            border: none;
            border-radius: 8px;
            background: #667eea;
            color: white;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        button:hover {
            background: #764ba2;
            transform: scale(1.05);
        }
        
        button:disabled {
            background: #ccc;
            cursor: not-allowed;
            transform: none;
        }
        
        .status {
            padding: 10px;
            border-radius: 5px;
            margin-top: 10px;
            font-size: 0.9em;
        }
        
        .status.success {
            background: #d4edda;
            color: #155724;
        }
        
        .status.error {
            background: #f8d7da;
            color: #721c24;
        }
        
        .status.info {
            background: #d1ecf1;
            color: #0c5460;
        }
        
        .motion-state {
            font-size: 1.5em;
            font-weight: bold;
            padding: 15px;
            border-radius: 10px;
            text-align: center;
            margin-top: 10px;
        }
        
        .motion-state.rest { background: #e8f5e9; color: #2e7d32; }
        .motion-state.shoulder_raise { background: #fff3e0; color: #e65100; }
        .motion-state.shoulder_lower { background: #f3e5f5; color: #6a1b9a; }
        .motion-state.elbow_flexion { background: #e3f2fd; color: #1565c0; }
        .motion-state.elbow_extension { background: #fce4ec; color: #c2185b; }
        .motion-state.hold { background: #fff9c4; color: #f57f17; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Dual IMU Pose Estimation System</h1>
        
        <!-- Control Panel -->
        <div class="card">
            <h2>⚙️ Control Panel</h2>
            
            <h3>Pose Zero Setup</h3>
            <p style="margin-top: 10px; color: #666; font-size: 0.9em;">
                Hold arm in neutral position, Y-axis perpendicular to ground
            </p>
            <div class="controls">
                <button onclick="setPoseZero('upper_arm')">Set Pose (Upper Arm)</button>
                <button onclick="setPoseZero('forearm')">Set Pose (Forearm)</button>
                <button onclick="setPoseZero('all')">Set Pose (Both)</button>
            </div>
            
            <h3>Recording Control</h3>
            <div style="display: flex; gap: 10px; margin-top: 10px;">
                <input type="text" id="labelInput" placeholder="Motion label (e.g., flexion)" value="unknown" 
                       style="flex: 1; padding: 10px; border: 1px solid #ddd; border-radius: 5px;">
                <button onclick="startRecording()">Start Record</button>
                <button onclick="stopRecording()">Stop Record</button>
            </div>
            
            <div id="statusMessage" style="margin-top: 15px;"></div>
        </div>
        
        <!-- Calibration Status -->
        <div class="grid" style="margin-top: 20px;">
            <div class="card" id="calibStatus_upper"></div>
            <div class="card" id="calibStatus_forearm"></div>
        </div>
        
        <!-- IMU Data Display -->
        <div class="grid">
            <div class="card" id="imuCard_upper"></div>
            <div class="card" id="imuCard_forearm"></div>
        </div>
        
        <!-- Pose Data Display -->
        <div class="card">
            <h2>🏋️ Pose Estimation Results</h2>
            <div id="poseData"></div>
        </div>

    </div>
    
    <script>

        
        async function updateData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                
                // Update Upper Arm IMU
                updateIMUDisplay('upper_arm', data.upper_arm);
                updateIMUDisplay('forearm', data.forearm);
                updatePoseDisplay(data.pose);
                
            } catch (error) {
                console.error('Failed to fetch data:', error);
            }
        }
        
        function updateIMUDisplay(imuName, imuData) {
            const current = imuData.current;
            const displayName = imuName === 'upper_arm' ? 'Upper Arm IMU (Channel 0)' : 'Forearm IMU (Channel 1)';
            const cardId = imuName === 'upper_arm' ? 'imuCard_upper' : 'imuCard_forearm';
            
            let html = `<h2>${imuName === 'upper_arm' ? '💪' : '🦾'} ${displayName}</h2>`;
            html += `
                <h3>Acceleration (g)</h3>
                <div class="data-row"><div class="data-label">Ax:</div><div class="data-value">${current.ax.toFixed(4)}</div></div>
                <div class="data-row"><div class="data-label">Ay:</div><div class="data-value">${current.ay.toFixed(4)}</div></div>
                <div class="data-row"><div class="data-label">Az:</div><div class="data-value">${current.az.toFixed(4)}</div></div>
                
                <h3>Angular Velocity (°/s)</h3>
                <div class="data-row"><div class="data-label">Gx:</div><div class="data-value">${current.gx.toFixed(2)}</div></div>
                <div class="data-row"><div class="data-label">Gy:</div><div class="data-value">${current.gy.toFixed(2)}</div></div>
                <div class="data-row"><div class="data-label">Gz:</div><div class="data-value">${current.gz.toFixed(2)}</div></div>
                
                <h3>Angles</h3>
                <div class="data-row"><div class="data-label">Roll:</div><div class="data-value">${current.roll.toFixed(2)}°</div></div>
                <div class="data-row"><div class="data-label">Pitch:</div><div class="data-value">${current.pitch.toFixed(2)}°</div></div>
                <div class="data-row"><div class="data-label">YZ Angle:</div><div class="data-value">${current.yz_angle.toFixed(2)}°</div></div>
            `;
            
            document.getElementById(cardId).innerHTML = html;
        }
        
        function updatePoseDisplay(poseData) {
            const current = poseData.current;
            const history = poseData.history;
            
            let html = `
                <h3>Upper Arm Raise</h3>
                <div class="data-row"><div class="data-label">Angle:</div><div class="data-value">${current.upper_arm_raise_angle.toFixed(2)}°</div></div>
                <div class="data-row"><div class="data-label">Velocity:</div><div class="data-value">${current.upper_arm_raise_velocity.toFixed(2)}°/s</div></div>
                
                <h3>Elbow</h3>
                <div class="data-row"><div class="data-label">Angle:</div><div class="data-value">${current.elbow_angle.toFixed(2)}°</div></div>
                <div class="data-row"><div class="data-label">Velocity:</div><div class="data-value">${current.elbow_velocity.toFixed(2)}°/s</div></div>
                
                <h3>Motion State</h3>
                <div class="motion-state ${current.motion_state}">${current.motion_state.toUpperCase()}</div>
                
                <h3>Expected Active Muscle</h3>
                <div class="data-row"><div class="data-label">Muscle:</div><div class="data-value">${current.expected_muscle.toUpperCase()}</div></div>
            `;
            
            document.getElementById('poseData').innerHTML = html;
        }
        
        // 图表功能已精简（数据历史为0时不显示）
        
        async function updateCalibrationStatus() {
            try {
                const response = await fetch('/calibration_status');
                const status = await response.json();
                
                const updateCalibCard = (imuName) => {
                    const cal = status[imuName];
                    const displayName = imuName === 'upper_arm' ? 'Upper Arm IMU' : 'Forearm IMU';
                    const cardId = imuName === 'upper_arm' ? 'calibStatus_upper' : 'calibStatus_forearm';
                    
                    let html = `<h2>${displayName} Calibration</h2>`;
                    html += `
                        <div class="data-row">
                            <div class="data-label">Gyro Calibrated:</div>
                            <div class="data-value">${cal.is_gyro_calibrated ? '✅ Yes' : '❌ No'}</div>
                        </div>
                        <div class="data-row">
                            <div class="data-label">Pose Zero Set:</div>
                            <div class="data-value">${cal.is_pose_calibrated ? '✅ Yes' : '❌ No'}</div>
                        </div>
                        <div class="data-row">
                            <div class="data-label">YZ Zero:</div>
                            <div class="data-value">${cal.yz_zero.toFixed(2)}°</div>
                        </div>
                    `;
                    
                    document.getElementById(cardId).innerHTML = html;
                };
                
                updateCalibCard('upper_arm');
                updateCalibCard('forearm');
            } catch (error) {
                console.error('Failed to fetch calibration status:', error);
            }
        }
        
        async function updateRecordingStatus() {
            try {
                const response = await fetch('/recording_status');
                const status = await response.json();
                
                const statusDiv = document.getElementById('statusMessage');
                if (status.is_recording) {
                    statusDiv.innerHTML = `<div class="status info">📝 Recording: ${status.filename} (${status.elapsed}s)</div>`;
                } else {
                    statusDiv.innerHTML = '';
                }
            } catch (error) {
                console.error('Failed to fetch recording status:', error);
            }
        }
        
        async function setPoseZero(imuName) {
            try {
                const response = await fetch('/set_pose_zero', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ imu_name: imuName })
                });
                const result = await response.json();
                showStatus(result.message, result.success ? 'success' : 'error');
            } catch (error) {
                showStatus('Failed to set pose: ' + error, 'error');
            }
        }
        
        async function startRecording() {
            const label = document.getElementById('labelInput').value || 'unknown';
            try {
                const response = await fetch('/start_recording', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ label: label })
                });
                const result = await response.json();
                showStatus(result.message, result.success ? 'success' : 'error');
            } catch (error) {
                showStatus('Failed to start recording: ' + error, 'error');
            }
        }
        
        async function stopRecording() {
            try {
                const response = await fetch('/stop_recording', {
                    method: 'POST'
                });
                const result = await response.json();
                showStatus(result.message, result.success ? 'success' : 'error');
            } catch (error) {
                showStatus('Failed to stop recording: ' + error, 'error');
            }
        }
        
        function showStatus(message, type) {
            const statusDiv = document.getElementById('statusMessage');
            statusDiv.innerHTML = `<div class="status ${type}">${message}</div>`;
            setTimeout(() => {
                statusDiv.innerHTML = '';
            }, 5000);
        }
        
        // Update every 100ms
        setInterval(() => {
            updateData();
            updateCalibrationStatus();
            updateRecordingStatus();
        }, 100);
        
        // Initial update
        updateData();
        updateCalibrationStatus();
        updateRecordingStatus();
    </script>
</body>
</html>
"""

# ==================== Main Application ====================
if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("Raspberry Pi Dual IMU Pose Estimation System")
    print("=" * 60)
    
    if not SMBUS_AVAILABLE:
        print("✗ FATAL: smbus2 not available. Install with:")
        print("  pip install smbus2")
        exit(1)
    
    # Initialize IMU sensors
    if not init_mpu():
        print("✗ FATAL: Could not initialize any IMU sensor")
        exit(1)
    
    # Start data reader thread
    reader_thread = threading.Thread(target=data_reader_thread, daemon=True)
    reader_thread.start()
    
    # Start Flask server
    print("\n✓ Starting Flask web server...")
    print("✓ Web interface: http://0.0.0.0:5000")
    print("✓ Press Ctrl+C to exit\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\n► Shutting down...")
        should_exit = True
        time.sleep(1)
        print("✓ Goodbye!")
