#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual IMU Motor Control System
- Two MPU6050 sensors via TCA9548A multiplexer
- Three motors controlled by PID based on IMU pose detection:
  * Deltoid (Shoulder raise) - Motor 1
  * Biceps (Elbow flexion) - Motor 2
  * Triceps (Elbow extension) - Motor 3
- Web interface with real-time motor speed display
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

# Try to import required libraries
try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    print("✗ smbus2 library not found. Install with: pip install smbus2")
    SMBUS_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("⚠ RPi.GPIO not available - running in simulation mode")
    GPIO_AVAILABLE = False

# ==================== Configuration ====================
I2C_BUS = 1
MPU_ADDRESS = 0x68
TCA_ADDRESS = 0x70
DATA_UPDATE_INTERVAL = 0.05  # 50 ms (20 Hz sampling)
MAX_DATA_POINTS = 100
DATA_FOLDER = "data"

# GPIO Pin Configuration (BCM pins)
MOTOR_CONFIG = {
    'deltoid': {
        'name': 'Deltoid (Shoulder)',
        'pwm_pin': 17,    # GPIO17 for PWM
        'dir_pin': 27,    # GPIO27 for direction
        'frequency': 1000  # 1 kHz PWM frequency
    },
    'biceps': {
        'name': 'Biceps (Flexion)',
        'pwm_pin': 22,    # GPIO22 for PWM
        'dir_pin': 23,    # GPIO23 for direction
        'frequency': 1000
    },
    'triceps': {
        'name': 'Triceps (Extension)',
        'pwm_pin': 24,    # GPIO24 for PWM
        'dir_pin': 25,    # GPIO25 for direction
        'frequency': 1000
    }
}

# Multi-IMU Configuration
IMU_CHANNELS = {
    'upper_arm': 0,   # Channel 0: Upper arm on shoulder
    'forearm': 1      # Channel 1: Forearm on elbow
}

# ==================== PID Controller ====================
class PIDController:
    """PID controller for motor speed regulation"""
    
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, setpoint=0.0, output_min=-100, output_max=100):
        self.kp = kp            # Proportional gain
        self.ki = ki            # Integral gain
        self.kd = kd            # Derivative gain
        self.setpoint = setpoint
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
    
    def update(self, current_value, current_time=None):
        """
        Update PID controller and return output
        
        Args:
            current_value: Current measured value
            current_time: Current time (optional, uses time.time() if not provided)
        
        Returns:
            PID output (-100 to 100)
        """
        if current_time is None:
            current_time = time.time()
        
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.001
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(self.output_min, min(self.output_max, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))
        
        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def set_setpoint(self, setpoint):
        """Set new setpoint"""
        self.setpoint = setpoint
        self.integral = 0.0
        self.previous_error = 0.0
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.previous_error = 0.0

# ==================== Motor Controller ====================
class MotorController:
    """Control motors with PWM using GPIO"""
    
    def __init__(self, motor_name, config):
        self.name = motor_name
        self.config = config
        self.pwm = None
        self.current_speed = 0
        self.target_speed = 0
        self.enabled = False
    
    def init_gpio(self):
        """Initialize GPIO pins for motor control"""
        if not GPIO_AVAILABLE:
            print(f"⚠ GPIO not available for {self.name}")
            return False
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.config['pwm_pin'], GPIO.OUT)
            GPIO.setup(self.config['dir_pin'], GPIO.OUT)
            
            # Create PWM object
            self.pwm = GPIO.PWM(self.config['pwm_pin'], self.config['frequency'])
            self.pwm.start(0)  # Start with 0% duty cycle
            
            # Set initial direction
            GPIO.output(self.config['dir_pin'], GPIO.LOW)
            
            self.enabled = True
            print(f"✓ {self.name} motor initialized")
            return True
            
        except Exception as e:
            print(f"✗ Failed to initialize {self.name}: {e}")
            return False
    
    def set_speed(self, speed):
        """
        Set motor speed (-100 to 100)
        Positive: forward, Negative: backward
        """
        if not self.enabled:
            self.target_speed = 0
            return
        
        # Clamp speed to valid range
        speed = max(-100, min(100, speed))
        self.target_speed = speed
        
        if self.pwm is None:
            return
        
        try:
            # Set direction based on speed sign
            if speed >= 0:
                GPIO.output(self.config['dir_pin'], GPIO.LOW)
            else:
                GPIO.output(self.config['dir_pin'], GPIO.HIGH)
            
            # Set PWM duty cycle (always positive)
            duty_cycle = abs(speed)
            self.pwm.ChangeDutyCycle(duty_cycle)
            self.current_speed = speed
            
        except Exception as e:
            print(f"✗ Error setting speed for {self.name}: {e}")
    
    def stop(self):
        """Stop motor"""
        self.set_speed(0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.pwm:
            self.pwm.stop()
        if self.enabled:
            try:
                GPIO.output(self.config['pwm_pin'], GPIO.LOW)
                GPIO.output(self.config['dir_pin'], GPIO.LOW)
            except:
                pass

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
        """Select TCA9548A channel"""
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
            self.bus.write_byte_data(self.mpu_addr, self.PWR_MGMT_1, 0x00)
            time.sleep(0.05)
            
            who = self.bus.read_byte_data(self.mpu_addr, self.WHO_AM_I)
            if who != 0x68:
                print(f"✗ WHO_AM_I mismatch: got 0x{who:02x}, expected 0x68")
                return False
            
            configs = [
                (0x19, 9),      # Sample rate divider
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
        """Read raw data from register"""
        try:
            self.select_channel(self.channel)
            data = self.bus.read_i2c_block_data(self.mpu_addr, reg, count)
            return data
        except Exception as e:
            print(f"✗ Failed to read from register 0x{reg:02x}: {e}")
            return None
    
    def get_data(self):
        """Read accelerometer and gyroscope data"""
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

# Thread synchronization
i2c_lock = threading.Lock()
data_lock = threading.Lock()

# IMU Data Storage
imu_data = {
    'upper_arm': {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'yz_angle': 0.0,
        'timestamp': 0
    },
    'forearm': {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'yz_angle': 0.0,
        'timestamp': 0
    }
}

# Pose estimation data
pose_data = {
    'upper_arm_raise_angle': 0.0,
    'upper_arm_raise_velocity': 0.0,
    'elbow_angle': 0.0,
    'elbow_velocity': 0.0,
    'motion_state': 'rest',
    'expected_muscle': 'unknown'
}

# Motor control data
motor_data = {
    'deltoid': {'speed': 0, 'target': 0, 'enabled': False},
    'biceps': {'speed': 0, 'target': 0, 'enabled': False},
    'triceps': {'speed': 0, 'target': 0, 'enabled': False}
}

# Control settings
control_settings = {
    'auto_mode': True,  # Automatic control based on IMU
    'manual_mode': False,  # Manual override
    'deltoid_target': 0.0,  # Manual target angle for deltoid
    'biceps_target': 0.0,   # Manual target angle for biceps
    'triceps_target': 0.0,  # Manual target angle for triceps
    # PID parameters for each motor
    'pid_params': {
        'deltoid': {'kp': 1.5, 'ki': 0.1, 'kd': 0.05},
        'biceps': {'kp': 1.2, 'ki': 0.1, 'kd': 0.05},
        'triceps': {'kp': 1.2, 'ki': 0.1, 'kd': 0.05}
    },
    # Angle limits
    'angle_limits': {
        'deltoid_max': 90.0,    # Max shoulder raise angle
        'elbow_max': 120.0      # Max elbow flexion angle
    }
}

# Calibration state
calibration_state = {
    'upper_arm': {
        'is_pose_calibrated': False,
        'yz_zero': 0.0,
        'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    },
    'forearm': {
        'is_pose_calibrated': False,
        'yz_zero': 0.0,
        'gyro_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }
}

# Previous pose values for velocity calculation
previous_pose_data = {
    'upper_arm_raise_angle': 0.0,
    'elbow_angle': 0.0,
    'timestamp': time.time()
}

# Hardware instances
mpu_upper_arm = None
mpu_forearm = None
motor_deltoid = None
motor_biceps = None
motor_triceps = None
pid_deltoid = None
pid_biceps = None
pid_triceps = None
should_exit = False

# ==================== Math Functions ====================
def calculate_angles(ax, ay, az):
    """Calculate roll and pitch from accelerometer data"""
    roll = math.atan2(ay, az) * 180 / math.pi
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return roll, pitch

def calculate_yz_angle(ay, az):
    """Calculate Y-Z plane angle from accelerometer"""
    return math.atan2(ay, az) * 180 / math.pi

def calculate_pose(upper_arm_yz, forearm_yz, upper_arm_yz_zero, forearm_yz_zero,
                   previous_upper_arm_raise, previous_elbow_angle, dt):
    """Calculate human pose angles and velocities"""
    
    # Shoulder raise angle
    ua_yz_zeroed = upper_arm_yz_zero - upper_arm_yz
    upper_arm_raise_angle = ua_yz_zeroed
    upper_arm_raise_velocity = (upper_arm_raise_angle - previous_upper_arm_raise) / dt if dt > 0 else 0
    
    # Elbow angle (positive when flexed)
    fa_yz_zeroed = forearm_yz_zero - forearm_yz
    elbow_angle = fa_yz_zeroed - ua_yz_zeroed
    elbow_velocity = (elbow_angle - previous_elbow_angle) / dt if dt > 0 else 0
    
    # Determine motion state
    ANGLE_THRESHOLD = 5.0
    VELOCITY_THRESHOLD = 8.0
    
    if upper_arm_raise_angle > ANGLE_THRESHOLD and upper_arm_raise_velocity > VELOCITY_THRESHOLD:
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
    elif abs(elbow_velocity) <= VELOCITY_THRESHOLD and abs(upper_arm_raise_velocity) <= VELOCITY_THRESHOLD:
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

# ==================== IMU Data Reading ====================
def read_imu_data():
    """Read data from both IMU sensors with pose estimation"""
    global imu_data, mpu_upper_arm, mpu_forearm, data_lock, calibration_state, pose_data, previous_pose_data
    
    if mpu_upper_arm is None and mpu_forearm is None:
        return
    
    raw_data_upper = None
    raw_data_forearm = None
    
    # Read IMU data
    with i2c_lock:
        if mpu_upper_arm is not None:
            try:
                raw_data_upper = mpu_upper_arm.get_data()
            except:
                pass
        
        if mpu_forearm is not None:
            try:
                raw_data_forearm = mpu_forearm.get_data()
            except:
                pass
    
    with data_lock:
        current_time = time.time()
        dt = current_time - previous_pose_data['timestamp']
        
        # Process Upper Arm IMU
        if raw_data_upper is not None:
            ax = raw_data_upper['x'] - calibration_state['upper_arm']['gyro_offset']['x']
            ay = raw_data_upper['y'] - calibration_state['upper_arm']['gyro_offset']['y']
            az = raw_data_upper['z'] - calibration_state['upper_arm']['gyro_offset']['z']
            
            roll, pitch = calculate_angles(ax, ay, az)
            yz_angle = calculate_yz_angle(ay, az)
            
            imu_data['upper_arm'].update({
                'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
                'roll': round(roll, 2), 'pitch': round(pitch, 2),
                'yz_angle': round(yz_angle, 2),
                'timestamp': current_time
            })
        
        # Process Forearm IMU
        if raw_data_forearm is not None:
            ax = raw_data_forearm['x'] - calibration_state['forearm']['gyro_offset']['x']
            ay = raw_data_forearm['y'] - calibration_state['forearm']['gyro_offset']['y']
            az = raw_data_forearm['z'] - calibration_state['forearm']['gyro_offset']['z']
            
            roll, pitch = calculate_angles(ax, ay, az)
            yz_angle = calculate_yz_angle(ay, az)
            
            imu_data['forearm'].update({
                'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
                'roll': round(roll, 2), 'pitch': round(pitch, 2),
                'yz_angle': round(yz_angle, 2),
                'timestamp': current_time
            })
        
        # Calculate pose if calibrated
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
            previous_pose_data['upper_arm_raise_angle'] = new_pose['upper_arm_raise_angle']
            previous_pose_data['elbow_angle'] = new_pose['elbow_angle']
            previous_pose_data['timestamp'] = current_time

def data_reader_thread():
    """Background thread for reading IMU data"""
    global should_exit, mpu_upper_arm, mpu_forearm
    
    if mpu_upper_arm is None and mpu_forearm is None:
        print("⚠ No IMU available")
        return
    
    print("► Data reader thread started")
    
    while not should_exit:
        read_imu_data()
        time.sleep(DATA_UPDATE_INTERVAL)
    
    print("► Data reader thread stopped")

# ==================== Motor Control Logic ====================
def update_motor_control():
    """Update motor speeds based on IMU data or manual control"""
    global motor_deltoid, motor_biceps, motor_triceps, pid_deltoid, pid_biceps, pid_triceps
    global pose_data, control_settings, motor_data, data_lock
    
    with data_lock:
        current_time = time.time()
        
        if control_settings['auto_mode'] and not control_settings['manual_mode']:
            # Automatic mode: based on IMU pose
            
            # Deltoid motor: controlled by shoulder raise angle
            shoulder_angle = pose_data['upper_arm_raise_angle']
            if pid_deltoid:
                pid_deltoid.set_setpoint(0)  # Target is upright position
                deltoid_speed = pid_deltoid.update(shoulder_angle, current_time)
                # Scale based on angle magnitude
                deltoid_speed = shoulder_angle * 0.5  # Simple proportional control
            else:
                deltoid_speed = shoulder_angle * 0.5
            
            # Biceps motor: controlled by positive elbow angle (flexion)
            elbow_angle = pose_data['elbow_angle']
            if elbow_angle > 0:
                if pid_biceps:
                    biceps_speed = pid_biceps.update(elbow_angle, current_time)
                else:
                    biceps_speed = min(100, abs(elbow_angle) * 1.0)
            else:
                if pid_biceps:
                    biceps_speed = 0
                else:
                    biceps_speed = 0
            
            # Triceps motor: controlled by negative elbow angle (extension)
            if elbow_angle < 0:
                if pid_triceps:
                    triceps_speed = pid_triceps.update(abs(elbow_angle), current_time)
                else:
                    triceps_speed = min(100, abs(elbow_angle) * 1.0)
            else:
                if pid_triceps:
                    triceps_speed = 0
                else:
                    triceps_speed = 0
            
        else:
            # Manual mode: use user-set targets
            deltoid_speed = control_settings['deltoid_target']
            biceps_speed = control_settings['biceps_target']
            triceps_speed = control_settings['triceps_target']
        
        # Apply motor speeds
        if motor_deltoid:
            motor_deltoid.set_speed(deltoid_speed)
            motor_data['deltoid']['speed'] = motor_deltoid.current_speed
        
        if motor_biceps:
            motor_biceps.set_speed(biceps_speed)
            motor_data['biceps']['speed'] = motor_biceps.current_speed
        
        if motor_triceps:
            motor_triceps.set_speed(triceps_speed)
            motor_data['triceps']['speed'] = motor_triceps.current_speed

def motor_control_thread():
    """Background thread for motor control"""
    global should_exit
    
    print("► Motor control thread started")
    
    while not should_exit:
        update_motor_control()
        time.sleep(DATA_UPDATE_INTERVAL)
    
    print("► Motor control thread stopped")

# ==================== Initialization ====================
def init_mpu():
    """Initialize IMU sensors"""
    global mpu_upper_arm, mpu_forearm
    
    if not SMBUS_AVAILABLE:
        print("✗ smbus2 not available")
        return False
    
    print("► Initializing IMU sensors...")
    initialized_count = 0
    
    try:
        upper_arm_imu = MPU6050Direct(I2C_BUS, TCA_ADDRESS, MPU_ADDRESS, IMU_CHANNELS['upper_arm'])
        if upper_arm_imu.init_mpu():
            mpu_upper_arm = upper_arm_imu
            print(f"✓ Upper Arm IMU initialized at TCA channel {IMU_CHANNELS['upper_arm']}")
            initialized_count += 1
        else:
            print(f"⚠ Upper Arm IMU not found")
    except Exception as e:
        print(f"⚠ Upper Arm IMU error: {e}")
    
    try:
        forearm_imu = MPU6050Direct(I2C_BUS, TCA_ADDRESS, MPU_ADDRESS, IMU_CHANNELS['forearm'])
        if forearm_imu.init_mpu():
            mpu_forearm = forearm_imu
            print(f"✓ Forearm IMU initialized at TCA channel {IMU_CHANNELS['forearm']}")
            initialized_count += 1
        else:
            print(f"⚠ Forearm IMU not found")
    except Exception as e:
        print(f"⚠ Forearm IMU error: {e}")
    
    if initialized_count == 0:
        print("✗ No IMU sensors initialized")
        return False
    
    return True

def init_motors():
    """Initialize motors"""
    global motor_deltoid, motor_biceps, motor_triceps
    global pid_deltoid, pid_biceps, pid_triceps
    
    print("► Initializing motors...")
    initialized_count = 0
    
    # Initialize deltoid motor
    motor_deltoid = MotorController('Deltoid', MOTOR_CONFIG['deltoid'])
    if motor_deltoid.init_gpio():
        initialized_count += 1
        pid_params = control_settings['pid_params']['deltoid']
        pid_deltoid = PIDController(
            kp=pid_params['kp'], 
            ki=pid_params['ki'], 
            kd=pid_params['kd']
        )
    
    # Initialize biceps motor
    motor_biceps = MotorController('Biceps', MOTOR_CONFIG['biceps'])
    if motor_biceps.init_gpio():
        initialized_count += 1
        pid_params = control_settings['pid_params']['biceps']
        pid_biceps = PIDController(
            kp=pid_params['kp'],
            ki=pid_params['ki'],
            kd=pid_params['kd']
        )
    
    # Initialize triceps motor
    motor_triceps = MotorController('Triceps', MOTOR_CONFIG['triceps'])
    if motor_triceps.init_gpio():
        initialized_count += 1
        pid_params = control_settings['pid_params']['triceps']
        pid_triceps = PIDController(
            kp=pid_params['kp'],
            ki=pid_params['ki'],
            kd=pid_params['kd']
        )
    
    if initialized_count == 0 and GPIO_AVAILABLE:
        print("✗ No motors initialized")
        return False
    elif initialized_count > 0:
        print(f"✓ {initialized_count} motor(s) initialized")
    
    return True

# ==================== Flask Routes ====================
@app.route('/')
def index():
    """Serve web interface"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>IMU Motor Control System</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {
                font-family: Arial, sans-serif;
                max-width: 1200px;
                margin: 0 auto;
                padding: 20px;
                background-color: #f5f5f5;
            }
            h1 { color: #333; }
            h2 { color: #555; border-bottom: 2px solid #007bff; padding-bottom: 10px; }
            
            .container {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 20px;
                margin-bottom: 30px;
            }
            
            .card {
                background: white;
                border-radius: 8px;
                padding: 20px;
                box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            }
            
            .sensor-data {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 10px;
                margin: 10px 0;
            }
            
            .data-item {
                background: #f0f0f0;
                padding: 10px;
                border-radius: 4px;
                font-size: 14px;
            }
            
            .data-label { font-weight: bold; color: #555; }
            .data-value { color: #007bff; font-size: 16px; margin-top: 5px; }
            
            .motor-control {
                background: white;
                border-radius: 8px;
                padding: 20px;
                margin-bottom: 20px;
            }
            
            .motor-item {
                background: #f0f0f0;
                padding: 15px;
                border-radius: 4px;
                margin: 10px 0;
                border-left: 4px solid #007bff;
            }
            
            .motor-name { font-weight: bold; font-size: 16px; color: #333; }
            .motor-speed { 
                font-size: 24px; 
                color: #007bff; 
                margin-top: 10px;
            }
            
            .slider-container {
                margin: 10px 0;
            }
            
            input[type="range"] {
                width: 100%;
                height: 6px;
                border-radius: 3px;
                background: #ddd;
                outline: none;
                margin: 10px 0;
            }
            
            input[type="range"]::-webkit-slider-thumb {
                -webkit-appearance: none;
                appearance: none;
                width: 20px;
                height: 20px;
                border-radius: 50%;
                background: #007bff;
                cursor: pointer;
            }
            
            input[type="range"]::-moz-range-thumb {
                width: 20px;
                height: 20px;
                border-radius: 50%;
                background: #007bff;
                cursor: pointer;
            }
            
            button {
                background-color: #007bff;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 4px;
                cursor: pointer;
                font-size: 14px;
                margin: 5px 5px 5px 0;
            }
            
            button:hover { background-color: #0056b3; }
            button.danger { background-color: #dc3545; }
            button.danger:hover { background-color: #c82333; }
            button.success { background-color: #28a745; }
            button.success:hover { background-color: #218838; }
            
            .mode-toggle {
                margin: 20px 0;
                padding: 15px;
                background: #e7f3ff;
                border-radius: 4px;
                border-left: 4px solid #007bff;
            }
            
            .status {
                padding: 10px;
                border-radius: 4px;
                margin: 10px 0;
                font-weight: bold;
            }
            
            .status.active { background: #d4edda; color: #155724; }
            .status.inactive { background: #f8d7da; color: #721c24; }
        </style>
    </head>
    <body>
        <h1>🤖 IMU Motor Control System</h1>
        
        <div class="mode-toggle">
            <h3>Control Mode</h3>
            <label>
                <input type="radio" name="mode" value="auto" checked onchange="setMode('auto')">
                <strong>Automatic</strong> - Controlled by IMU
            </label><br>
            <label>
                <input type="radio" name="mode" value="manual" onchange="setMode('manual')">
                <strong>Manual</strong> - Manual speed control
            </label>
        </div>
        
        <div class="container">
            <div class="card">
                <h2>📊 Upper Arm IMU</h2>
                <div class="data-item">
                    <div class="data-label">YZ Angle</div>
                    <div class="data-value" id="ua-yz">0.0°</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Roll</div>
                    <div class="data-value" id="ua-roll">0.0°</div>
                </div>
            </div>
            
            <div class="card">
                <h2>📊 Forearm IMU</h2>
                <div class="data-item">
                    <div class="data-label">YZ Angle</div>
                    <div class="data-value" id="fa-yz">0.0°</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Roll</div>
                    <div class="data-value" id="fa-roll">0.0°</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>🎯 Pose Estimation</h2>
            <div class="sensor-data">
                <div class="data-item">
                    <div class="data-label">Shoulder Raise Angle</div>
                    <div class="data-value" id="pose-shoulder">0.0°</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Elbow Angle</div>
                    <div class="data-value" id="pose-elbow">0.0°</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Motion State</div>
                    <div class="data-value" id="pose-state">rest</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Expected Muscle</div>
                    <div class="data-value" id="pose-muscle">unknown</div>
                </div>
            </div>
        </div>
        
        <div class="motor-control">
            <h2>⚙️ Motor Control</h2>
            
            <div class="motor-item" style="border-left-color: #ff6b6b;">
                <div class="motor-name">🔴 Deltoid (Shoulder)</div>
                <div class="motor-speed" id="motor-deltoid-speed">0%</div>
                <div id="motor-deltoid-slider" style="display:none;">
                    <input type="range" id="slider-deltoid" min="-100" max="100" value="0" 
                           oninput="updateMotorSpeed('deltoid', this.value)">
                    <div id="motor-deltoid-value">0</div>
                </div>
            </div>
            
            <div class="motor-item" style="border-left-color: #4ecdc4;">
                <div class="motor-name">🔵 Biceps (Flexion)</div>
                <div class="motor-speed" id="motor-biceps-speed">0%</div>
                <div id="motor-biceps-slider" style="display:none;">
                    <input type="range" id="slider-biceps" min="-100" max="100" value="0" 
                           oninput="updateMotorSpeed('biceps', this.value)">
                    <div id="motor-biceps-value">0</div>
                </div>
            </div>
            
            <div class="motor-item" style="border-left-color: #ffd93d;">
                <div class="motor-name">🟡 Triceps (Extension)</div>
                <div class="motor-speed" id="motor-triceps-speed">0%</div>
                <div id="motor-triceps-slider" style="display:none;">
                    <input type="range" id="slider-triceps" min="-100" max="100" value="0" 
                           oninput="updateMotorSpeed('triceps', this.value)">
                    <div id="motor-triceps-value">0</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>🎛️ Calibration</h2>
            <button onclick="calibrateIMU()">🔧 Calibrate All IMUs</button>
            <button onclick="setPoseZero()">📍 Set Pose Zero</button>
            <button onclick="resetCalibration()">🔄 Reset Calibration</button>
            <button class="danger" onclick="stopAllMotors()">🛑 Stop All Motors</button>
        </div>
        
        <script>
            // Auto-refresh data every 200ms
            setInterval(updateData, 200);
            
            function updateData() {
                fetch('/api/data')
                    .then(r => r.json())
                    .then(data => {
                        // IMU data
                        document.getElementById('ua-yz').textContent = data.imu.upper_arm.yz_angle.toFixed(2) + '°';
                        document.getElementById('ua-roll').textContent = data.imu.upper_arm.roll.toFixed(2) + '°';
                        document.getElementById('fa-yz').textContent = data.imu.forearm.yz_angle.toFixed(2) + '°';
                        document.getElementById('fa-roll').textContent = data.imu.forearm.roll.toFixed(2) + '°';
                        
                        // Pose data
                        document.getElementById('pose-shoulder').textContent = data.pose.upper_arm_raise_angle.toFixed(2) + '°';
                        document.getElementById('pose-elbow').textContent = data.pose.elbow_angle.toFixed(2) + '°';
                        document.getElementById('pose-state').textContent = data.pose.motion_state;
                        document.getElementById('pose-muscle').textContent = data.pose.expected_muscle;
                        
                        // Motor data
                        document.getElementById('motor-deltoid-speed').textContent = data.motors.deltoid.speed.toFixed(0) + '%';
                        document.getElementById('motor-biceps-speed').textContent = data.motors.biceps.speed.toFixed(0) + '%';
                        document.getElementById('motor-triceps-speed').textContent = data.motors.triceps.speed.toFixed(0) + '%';
                    });
            }
            
            function setMode(mode) {
                fetch('/api/set-mode', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({mode: mode})
                }).then(r => r.json()).then(data => {
                    const autoSliders = ['motor-deltoid-slider', 'motor-biceps-slider', 'motor-triceps-slider'];
                    autoSliders.forEach(id => {
                        document.getElementById(id).style.display = mode === 'manual' ? 'block' : 'none';
                    });
                });
            }
            
            function updateMotorSpeed(motor, value) {
                fetch('/api/motor-speed', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({motor: motor, speed: parseInt(value)})
                });
                document.getElementById('motor-' + motor + '-value').textContent = value;
            }
            
            function calibrateIMU() {
                fetch('/api/calibrate', {method: 'POST'})
                    .then(r => r.json())
                    .then(data => alert(data.message));
            }
            
            function setPoseZero() {
                fetch('/api/pose-zero', {method: 'POST'})
                    .then(r => r.json())
                    .then(data => alert(data.message));
            }
            
            function resetCalibration() {
                fetch('/api/reset-calibration', {method: 'POST'})
                    .then(r => r.json())
                    .then(data => alert(data.message));
            }
            
            function stopAllMotors() {
                if (confirm('Stop all motors?')) {
                    fetch('/api/stop-motors', {method: 'POST'})
                        .then(r => r.json())
                        .then(data => alert(data.message));
                }
            }
        </script>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/api/data')
def get_data():
    """Get current sensor and motor data"""
    with data_lock:
        return jsonify({
            'imu': imu_data,
            'pose': pose_data,
            'motors': motor_data
        })

@app.route('/api/set-mode', methods=['POST'])
def set_mode():
    """Set control mode (auto or manual)"""
    global control_settings
    
    data = request.get_json()
    mode = data.get('mode', 'auto')
    
    with data_lock:
        if mode == 'manual':
            control_settings['auto_mode'] = False
            control_settings['manual_mode'] = True
        else:
            control_settings['auto_mode'] = True
            control_settings['manual_mode'] = False
    
    return jsonify({'success': True, 'mode': mode})

@app.route('/api/motor-speed', methods=['POST'])
def set_motor_speed():
    """Set motor speed in manual mode"""
    global control_settings
    
    data = request.get_json()
    motor = data.get('motor')
    speed = data.get('speed', 0)
    
    with data_lock:
        if motor == 'deltoid':
            control_settings['deltoid_target'] = speed
        elif motor == 'biceps':
            control_settings['biceps_target'] = speed
        elif motor == 'triceps':
            control_settings['triceps_target'] = speed
    
    return jsonify({'success': True})

@app.route('/api/calibrate', methods=['POST'])
def calibrate():
    """Calibrate gyroscope"""
    from_channel = 0
    samples = 100
    
    with data_lock:
        gyro_sum = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    with i2c_lock:
        if mpu_upper_arm:
            for _ in range(samples):
                data = mpu_upper_arm.get_data()
                if data:
                    gyro_sum['x'] += data['gx']
                    gyro_sum['y'] += data['gy']
                    gyro_sum['z'] += data['gz']
                time.sleep(0.01)
    
    with data_lock:
        calibration_state['upper_arm']['gyro_offset']['x'] = gyro_sum['x'] / samples
        calibration_state['upper_arm']['gyro_offset']['y'] = gyro_sum['y'] / samples
        calibration_state['upper_arm']['gyro_offset']['z'] = gyro_sum['z'] / samples
    
    return jsonify({'success': True, 'message': '✓ Calibration complete'})

@app.route('/api/pose-zero', methods=['POST'])
def pose_zero():
    """Set pose zero reference"""
    with data_lock:
        calibration_state['upper_arm']['yz_zero'] = imu_data['upper_arm']['yz_angle']
        calibration_state['forearm']['yz_zero'] = imu_data['forearm']['yz_angle']
        calibration_state['upper_arm']['is_pose_calibrated'] = True
        calibration_state['forearm']['is_pose_calibrated'] = True
    
    return jsonify({'success': True, 'message': '✓ Pose zero set'})

@app.route('/api/reset-calibration', methods=['POST'])
def reset_cal():
    """Reset calibration"""
    with data_lock:
        for imu_name in ['upper_arm', 'forearm']:
            calibration_state[imu_name]['gyro_offset'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            calibration_state[imu_name]['is_pose_calibrated'] = False
            calibration_state[imu_name]['yz_zero'] = 0.0
    
    return jsonify({'success': True, 'message': '✓ Calibration reset'})

@app.route('/api/stop-motors', methods=['POST'])
def stop_motors():
    """Stop all motors"""
    if motor_deltoid:
        motor_deltoid.stop()
    if motor_biceps:
        motor_biceps.stop()
    if motor_triceps:
        motor_triceps.stop()
    
    return jsonify({'success': True, 'message': '✓ All motors stopped'})

# ==================== Main ====================
def main():
    global should_exit
    
    print("=" * 60)
    print("🤖 IMU Motor Control System")
    print("=" * 60)
    
    # Initialize hardware
    imu_ok = init_mpu()
    motors_ok = init_motors()
    
    if not imu_ok and not motors_ok:
        print("✗ No hardware initialized, exiting")
        return
    
    # Start background threads
    reader_thread = threading.Thread(target=data_reader_thread, daemon=True)
    reader_thread.start()
    
    motor_thread = threading.Thread(target=motor_control_thread, daemon=True)
    motor_thread.start()
    
    print("\n► Starting web server at http://0.0.0.0:5000")
    print("► Press Ctrl+C to stop\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n► Shutting down...")
        should_exit = True
        
        # Stop motors
        if motor_deltoid:
            motor_deltoid.cleanup()
        if motor_biceps:
            motor_biceps.cleanup()
        if motor_triceps:
            motor_triceps.cleanup()
        
        # Cleanup GPIO
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass
        
        print("✓ Shutdown complete")

if __name__ == '__main__':
    main()
