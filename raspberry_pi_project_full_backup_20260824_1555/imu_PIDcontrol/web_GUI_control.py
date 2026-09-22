import csv
import json
import math
import os
import threading
import time
from datetime import datetime

from flask import Flask, jsonify, request, render_template_string

from imu_control import IMURehabSystem
from rom_calibration import ROMCalibrationSession
from muscle_allocator import MuscleAllocator, MusclePWM
from esp32motor import ESP32MotorBridge
from frequency_analysis import FrequencyIdentificationSession

app = Flask(__name__)

state_lock = threading.RLock()
sensor = None
reader_thread = None
reader_running = False
imu_last_attempt_monotonic = 0.0
imu_last_success_monotonic = 0.0
imu_init_started_monotonic = 0.0
imu_read_failure_count = 0
imu_last_error = ""
imu_channel_health_snapshot = {
    "upper_arm": {
        "channel": 0, "label": "上臂 IMU", "connected": False,
        "checked": False, "failure_count": 0, "last_error": "",
        "last_success_age_seconds": None,
    },
    "forearm": {
        "channel": 1, "label": "前臂 IMU", "connected": False,
        "checked": False, "failure_count": 0, "last_error": "",
        "last_success_age_seconds": None,
    },
}
IMU_MAX_SINGLE_SAMPLE_JUMP_DEG = 35.0
IMU_RECOVERY_STABLE_SAMPLES_REQUIRED = 25
imu_safety_fault_latched = False
imu_safety_fault_reason = ""
imu_safety_fault_time = None
imu_recovery_stable_samples = 0
imu_recovery_ready = False
imu_last_valid_angles = None
IMU_READER_STALL_TIMEOUT = 2.0
IMU_READER_HARD_STALL_TIMEOUT = 15.0
IMU_INIT_STALL_TIMEOUT = 15.0
imu_recovery_requested_monotonic = 0.0
latest_frame = None
history = []
rom_calibration = ROMCalibrationSession()
frequency_session = FrequencyIdentificationSession()

system_status = "尚未初始化"
initialized = False
initializing = False
recording = False
csv_file = None
csv_writer = None
current_csv_path = None
record_count = 0
current_label = "CONTROL_TEST"
participant_id = "S01"
session_name = "session_01"
output_dir = os.path.abspath("control_data")

# ===== Motor output to ESP32 =====
# Stable CP2102 device path observed on this Raspberry Pi.
ESP32_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
ENABLE_AUTO_CONNECT_ESP32 = True  # Opening Serial is safe; ESP32 remains IDLE until ARM.
MOTOR_PWM_LIMIT = 255
MOTOR_CONTROL_PROFILE = "three_muscle"
# Applied exactly once at the final ESP32 gateway for rehabilitation,
# identification, demo, pulse-test, and manual-jog paths.
MOTOR_DIRECTION_SIGN = (-1, 1, 1)
# Global physical output calibration, also applied exactly once at the final
# gateway. The biceps motor is mechanically stronger, so reduce both winding
# and payout commands without changing any controller's internal balance.
MOTOR_OUTPUT_SCALE_DEFAULT = (0.8, 1.0, 1.0)
MOTOR_OUTPUT_SCALE_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "motor_output_scales.json"
)
CABLE_SETTINGS_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "cable_settings.json"
)
CABLE_SETTINGS_DEFAULT = {
    "antagonist_release_gain": 1.0,
    "triceps_release_ratio": 0.5,
    "biceps_release_ratio": 0.8,
    "cable_return_gain": 1.0,
    "cable_return_horizon": 1.25,
    "cable_return_max_pwm": 60.0,
    "shoulder_elbow_coupling_pwm": 50.0,
    "shoulder_coupling_rewind_ratio": 1.0,
    "wind_slew_rate": 1200.0,
    "release_slew_rate": 2000.0,
    "following_triceps_wind_limit": 100.0,
    "elbow_soft_landing_target": 90.0,
    "elbow_soft_landing_zone": 25.0,
    "elbow_soft_landing_min_ratio": 0.25,
}
CABLE_SETTING_LIMITS = {
    "antagonist_release_gain": (0.1, 1.5),
    "triceps_release_ratio": (0.05, 1.0),
    "biceps_release_ratio": (0.05, 1.0),
    "cable_return_gain": (1.0, 2.0),
    "cable_return_horizon": (0.2, 5.0),
    "cable_return_max_pwm": (0.0, 255.0),
    "shoulder_elbow_coupling_pwm": (0.0, 150.0),
    "shoulder_coupling_rewind_ratio": (0.1, 2.0),
    "wind_slew_rate": (50.0, 5000.0),
    "release_slew_rate": (50.0, 5000.0),
    "following_triceps_wind_limit": (0.0, 255.0),
    "elbow_soft_landing_target": (30.0, 150.0),
    "elbow_soft_landing_zone": (5.0, 60.0),
    "elbow_soft_landing_min_ratio": (0.0, 1.0),
}


def load_motor_output_scale_config():
    values = list(MOTOR_OUTPUT_SCALE_DEFAULT)
    try:
        with open(MOTOR_OUTPUT_SCALE_CONFIG, "r", encoding="utf-8") as handle:
            saved = json.load(handle)
        for index, name in enumerate(("biceps", "triceps", "deltoid")):
            values[index] = min(1.0, max(0.1, float(saved[name])))
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        pass
    return values


def save_motor_output_scale_config():
    payload = dict(zip(
        ("biceps", "triceps", "deltoid"), MOTOR_OUTPUT_SCALE
    ))
    temporary_path = MOTOR_OUTPUT_SCALE_CONFIG + ".tmp"
    with open(temporary_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_path, MOTOR_OUTPUT_SCALE_CONFIG)


def load_cable_settings_config():
    values = dict(CABLE_SETTINGS_DEFAULT)
    try:
        with open(CABLE_SETTINGS_CONFIG, "r", encoding="utf-8") as handle:
            saved = json.load(handle)
        for name, (minimum, maximum) in CABLE_SETTING_LIMITS.items():
            values[name] = min(maximum, max(minimum, float(saved[name])))
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        pass
    values["cable_return_gain"] = max(
        values["cable_return_gain"], values["antagonist_release_gain"]
    )
    return values


def save_cable_settings_config():
    temporary_path = CABLE_SETTINGS_CONFIG + ".tmp"
    with open(temporary_path, "w", encoding="utf-8") as handle:
        json.dump(CABLE_SETTINGS, handle, ensure_ascii=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_path, CABLE_SETTINGS_CONFIG)


MOTOR_OUTPUT_SCALE = load_motor_output_scale_config()
CABLE_SETTINGS = load_cable_settings_config()
motor_bridge = None
motor_bridge_connected = False
motor_bridge_error = ""
motor_test_running = False
motor_arm_in_progress = False
motor_arm_session = 0
MOTOR_TEST_PWM = 200
MOTOR_TEST_DURATION = 0.5
FOLLOWING_TRICEPS_WIND_LIMIT = CABLE_SETTINGS["following_triceps_wind_limit"]
motor_jog_active = False
motor_jog_session = 0
motor_jog_last_heartbeat = 0.0
motor_jog_motor = ""
motor_jog_direction = 0
motor_jog_pwm = 100
motor_channel_locks = {
    "biceps": False,
    "triceps": False,
    "deltoid": False,
}
MOTOR_JOG_DEFAULT_PWM = 100
MOTOR_JOG_MAX_PWM = 200
MOTOR_JOG_HEARTBEAT_TIMEOUT = 0.5

muscle_allocator = MuscleAllocator(
    pwm_limit=MOTOR_PWM_LIMIT,
    active_min_pwm=0,
    # GUI "following PWM" is the only therapeutic base floor.  Do not apply
    # a second hidden 200-PWM promotion in the allocator.
    motor_min_pwm=(0, 0, 0),
    antagonist_release_gain=CABLE_SETTINGS["antagonist_release_gain"],
    triceps_release_ratio=CABLE_SETTINGS["triceps_release_ratio"],
    biceps_release_ratio=CABLE_SETTINGS["biceps_release_ratio"],
    shoulder_release_pwm=60,
    command_filter_tau=0.02,
    wind_slew_rate=CABLE_SETTINGS["wind_slew_rate"],
    release_slew_rate=CABLE_SETTINGS["release_slew_rate"],
    reverse_deadtime=0.02,
    cable_return_gain=CABLE_SETTINGS["cable_return_gain"],
    cable_effort_limit=5000.0,
    cable_return_threshold=1.0,
    # Repay elbow PWM-time imbalance promptly enough to prevent slow cable
    # shortening during repeated following cycles. Extra payout remains capped
    # by cable_return_max_pwm below.
    cable_return_horizon=CABLE_SETTINGS["cable_return_horizon"],
    cable_return_max_pwm=CABLE_SETTINGS["cable_return_max_pwm"],
    shoulder_elbow_coupling_pwm=CABLE_SETTINGS["shoulder_elbow_coupling_pwm"],
    shoulder_coupling_rewind_ratio=CABLE_SETTINGS["shoulder_coupling_rewind_ratio"],
    shoulder_coupling_effort_limit=5000.0,
    elbow_soft_landing_target=CABLE_SETTINGS["elbow_soft_landing_target"],
    elbow_soft_landing_zone=CABLE_SETTINGS["elbow_soft_landing_zone"],
    elbow_soft_landing_min_ratio=CABLE_SETTINGS["elbow_soft_landing_min_ratio"],
    # Motor 1: biceps, Motor 2: triceps, Motor 3: deltoid
    # If one motor direction is reversed, change its sign to -1.
    motor_sign=(1, 1, 1),
)


def current_cable_settings():
    return {
        "antagonist_release_gain": float(muscle_allocator.antagonist_release_gain),
        "triceps_release_ratio": float(muscle_allocator.triceps_release_ratio),
        "biceps_release_ratio": float(muscle_allocator.biceps_release_ratio),
        "cable_return_gain": float(muscle_allocator.cable_return_gain),
        "cable_return_horizon": float(muscle_allocator.cable_return_horizon),
        "cable_return_max_pwm": float(muscle_allocator.cable_return_max_pwm),
        "shoulder_elbow_coupling_pwm": float(muscle_allocator.shoulder_elbow_coupling_pwm),
        "shoulder_coupling_rewind_ratio": float(muscle_allocator.shoulder_coupling_rewind_ratio),
        "wind_slew_rate": float(muscle_allocator.wind_slew_rate),
        "release_slew_rate": float(muscle_allocator.release_slew_rate),
        "following_triceps_wind_limit": float(FOLLOWING_TRICEPS_WIND_LIMIT),
        "elbow_soft_landing_target": float(muscle_allocator.elbow_soft_landing_target),
        "elbow_soft_landing_zone": float(muscle_allocator.elbow_soft_landing_zone),
        "elbow_soft_landing_min_ratio": float(muscle_allocator.elbow_soft_landing_min_ratio),
    }

HTML_PAGE = r"""
<!DOCTYPE html>
<html lang="zh-Hant">
<head>
  <meta charset="UTF-8">
  <title>IMU 上肢復健軌跡控制系統</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    body{font-family:Arial,"Microsoft JhengHei",sans-serif;margin:0;background:#f3f4f6;color:#111827}
    header{background:#111827;color:white;padding:16px 24px} header h1{margin:0;font-size:24px} header p{margin:6px 0 0;color:#d1d5db}
    .container{display:grid;grid-template-columns:minmax(500px,2fr) minmax(620px,3fr);gap:16px;padding:16px;align-items:start}
    .fixed-estop{position:fixed;right:18px;top:14px;z-index:1200;width:auto;min-width:190px;margin:0;padding:12px 18px;border:3px solid white;border-radius:999px;background:#dc2626;box-shadow:0 6px 22px rgba(127,29,29,.45);font-weight:800}
    .controls{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));grid-template-areas:"init task" "controller safety" "cable cable" "rom rom" "frequency frequency" "record record";gap:12px;align-items:start}
    .card{background:white;border-radius:12px;padding:16px;box-shadow:0 2px 8px rgba(0,0,0,.08);margin:0}.card h2{font-size:18px;margin:0 0 12px}.init-card{grid-area:init}.rom-card{grid-area:rom}.task-card{grid-area:task}.controller-card{grid-area:controller}.safety-card{grid-area:safety}.cable-card{grid-area:cable}.frequency-card{grid-area:frequency}.record-card{grid-area:record}
    .monitor-column{position:sticky;top:12px;align-self:start;display:grid;gap:12px}.monitor-column .card{padding:12px}.live-card{min-height:104px}.live-card .metrics{grid-template-columns:repeat(6,minmax(78px,1fr));gap:7px;align-items:stretch}.live-card .metric{padding:8px;height:70px;box-sizing:border-box;overflow:hidden;min-width:0}.live-card .metric-title{font-size:11px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.live-card .metric-value{font-size:18px;margin-top:4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;height:34px;line-height:1.2}.live-card #motionState{font-size:13px!important;white-space:normal;display:-webkit-box;-webkit-box-orient:vertical;-webkit-line-clamp:2;line-clamp:2}.chart-card h2{font-size:20px}.chart-card .small{margin-bottom:0}
    .controls .card{padding:13px}.controls .card h2{margin-bottom:9px}.controls label{margin:5px 0 3px}.controls input,.controls select{padding:7px}.controls button{padding:9px;margin:4px 0}.controls .parameter-panel{margin-top:8px;padding:10px}.controls .validation{padding:8px;margin:6px 0;line-height:1.4}.record-card .record-fields{grid-template-columns:repeat(3,minmax(0,1fr))}.record-card .record-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}.record-card #recordingState{margin:8px 0 2px}.quick-test-canvas{display:none;height:auto;max-height:620px;margin-top:8px}.quick-test-canvas.visible{display:block}.quick-results{font-weight:700;color:#1e3a8a}.quick-run-selector{display:grid;gap:6px;margin:7px 0 10px}.quick-run-item{display:flex;align-items:center;gap:8px;padding:7px 9px;border:1px solid #cbd5e1;border-radius:8px;background:#f8fafc;font-size:13px}.quick-run-item input{width:auto;margin:0;flex:0 0 auto}.quick-run-item label{margin:0;cursor:pointer;flex:1;color:#334155}
    .collapsible-card>h2{display:flex;align-items:center;justify-content:space-between;gap:10px;cursor:pointer;padding:3px 5px;border-radius:7px;user-select:none;transition:background .15s,color .15s}.collapsible-card>h2:hover{background:#eff6ff;color:#1d4ed8}.collapsible-card>h2:focus-visible{outline:3px solid #93c5fd;outline-offset:2px}.collapsible-card>h2::after{content:'－';display:grid;place-items:center;flex:0 0 28px;height:28px;border-radius:999px;background:#dbeafe;color:#1d4ed8;font-size:20px;line-height:1}.collapsible-card.collapsed>h2{margin-bottom:0}.collapsible-card.collapsed>h2::after{content:'＋';background:#e5e7eb;color:#374151}.collapsible-card.collapsed>:not(h2){display:none!important}
    label{display:block;font-size:13px;color:#374151;margin:7px 0 4px}input,select{width:100%;box-sizing:border-box;padding:8px;border:1px solid #cbd5e1;border-radius:8px}button{position:relative;width:100%;padding:10px;border:0;border-radius:8px;margin:5px 0;background:#2563eb;color:white;font-size:15px;cursor:pointer;transition:transform .1s,filter .15s,box-shadow .15s}button:hover{filter:brightness(.92);box-shadow:0 3px 8px rgba(15,23,42,.18)}button:active,button.button-pressed{transform:scale(.97);filter:brightness(.82)}button:disabled{background:#9ca3af;cursor:not-allowed;transform:none;box-shadow:none}button.button-busy{cursor:wait;animation:buttonPulse .8s ease-in-out infinite alternate}button.button-success{background:#16a34a!important}button.button-error{background:#dc2626!important}button.jog-active{box-shadow:0 0 0 4px #fbbf24;filter:brightness(1.15);transform:scale(.98)}.secondary{background:#64748b}.success{background:#16a34a}.danger{background:#dc2626}.warning{background:#f59e0b;color:#111827}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.status{background:#eef2ff;border-left:5px solid #4f46e5;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:14px}.validation{background:#f8fafc;border:1px solid #d1d5db;border-radius:8px;padding:10px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.5}.protocol{background:#fff7ed;border:1px solid #fdba74;border-radius:8px;padding:11px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.55}.progress-track{height:10px;background:#e5e7eb;border-radius:999px;overflow:hidden;margin:8px 0}.progress-bar{height:100%;width:0;background:#2563eb;transition:width .15s linear}.live-angle{background:#ecfeff;border:1px solid #67e8f9;border-radius:8px;padding:9px;margin:8px 0;white-space:pre-wrap;font-family:Consolas,monospace;font-size:12px}.pass{color:#15803d;font-weight:700}.fail{color:#b91c1c;font-weight:700}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.metric{background:#f8fafc;border:1px solid #e5e7eb;border-radius:10px;padding:12px}.metric-title{font-size:13px;color:#64748b}.metric-value{font-size:22px;font-weight:700;margin-top:6px}.small{font-size:13px;color:#64748b;line-height:1.45}.rec{color:#dc2626;font-weight:700}.idle{color:#64748b;font-weight:700}.parameter-panel{display:none;margin-top:10px;padding:12px;border:1px solid #dbeafe;border-radius:10px;background:#f8fbff}.parameter-panel.active{display:block;animation:panelIn .18s ease-out}.parameter-title{font-size:14px;font-weight:700;color:#1e3a8a;margin-bottom:6px}.mode-summary{margin-top:9px;padding:9px;border-radius:8px;background:#eff6ff;color:#1e40af;font-size:13px;line-height:1.45}.action-toast{position:fixed;right:18px;bottom:18px;z-index:1000;max-width:360px;padding:12px 16px;border-radius:10px;background:#1e293b;color:white;box-shadow:0 8px 24px rgba(15,23,42,.28);opacity:0;transform:translateY(14px);pointer-events:none;transition:.2s}.action-toast.show{opacity:1;transform:translateY(0)}.action-toast.success{background:#15803d}.action-toast.error{background:#b91c1c}.action-toast.info{background:#1e40af}canvas{width:100%;height:430px;background:white;border:1px solid #e5e7eb;border-radius:10px}#angleCanvas{height:clamp(640px,78vh,860px)}@keyframes panelIn{from{opacity:0;transform:translateY(-5px)}to{opacity:1;transform:none}}@keyframes buttonPulse{from{filter:brightness(.85)}to{filter:brightness(1.1)}}
    @media(max-width:1180px){.container{grid-template-columns:1fr}.monitor-column{position:static}.live-card .metrics{grid-template-columns:repeat(6,1fr)}}
    @media(max-width:820px){.controls{grid-template-columns:1fr;grid-template-areas:"init" "task" "controller" "safety" "cable" "rom" "frequency" "record"}.live-card .metrics{grid-template-columns:repeat(2,1fr)}.record-card .record-fields{grid-template-columns:1fr}#angleCanvas{height:520px}}
  </style>
</head>
<body>
<header>
  <h1>雙 IMU 目標軌跡導引式上肢復健控制系統</h1>
  <p>上肢復健控制與即時狀態</p>
</header>
<button class="danger fixed-estop" onclick="emergencyStop()" aria-label="Emergency Stop">緊急停止 Emergency Stop</button>
<div class="container">
  <div class="controls">
    <div class="card init-card">
      <h2>1. 初始化</h2>
      <button onclick="initializeSystem()">初始化並校正 IMU</button>
      <button class="secondary" onclick="refreshStatus()">更新狀態</button>
      <div class="status" id="statusBox">尚未初始化</div>
      <div class="validation" id="imuChannelStatus">上臂 IMU (Channel 0)：尚未檢查\n前臂 IMU (Channel 1)：尚未檢查</div>
    </div>

    <div class="card rom-card collapsible-card collapsed">
      <h2>2. 病患個人化活動範圍 ROM</h2>
      <p class="small">校正後依畫面指示完成四個動作，系統會自動估算活動範圍。</p>
      <label>每個動作自由活動時間（5–30 秒）</label><input id="romDurationInput" type="number" min="5" max="30" step="1" value="10">
      <button onclick="startValidation()">開始／重新記錄個人化 ROM</button>
      <button id="validationStageButton" class="secondary" onclick="prepareAndStartValidationStage()" disabled>2 秒準備後開始目前階段</button>
      <button id="validationApplyButton" class="success" onclick="applyValidation()" disabled>確認並套用個人化 ROM</button>
      <div class="validation" id="validationGuide">請先以手臂自然下垂姿勢初始化並校正 IMU。</div>
      <div class="progress-track"><div class="progress-bar" id="validationProgressBar"></div></div>
      <div class="protocol" id="validationProtocol">開始 ROM 後，這裡會顯示每個自由活動動作的方向。</div>
      <div class="live-angle" id="validationLiveAngles">即時角度：尚未初始化</div>
      <div class="validation" id="validationResults">尚無個人化 ROM 記錄。</div>
    </div>

    <div class="card task-card collapsible-card collapsed">
      <h2>3. 復健任務設定</h2>
      <label>目標關節 Target Joint</label>
      <select id="targetJointInput" onchange="updateJointLabels()">
        <option value="elbow" selected>肘關節 elbow</option>
        <option value="shoulder">肩關節 shoulder</option>
      </select>

      <label>復健軌跡 Trajectory</label>
      <select id="targetModeInput">
        <option value="fixed" selected>固定角度 fixed target</option>
        <option value="sine">正弦軌跡跟隨 sine tracking</option>
        <option value="step">階躍測試 step response</option>
      </select>

      <div id="trajectoryFixedPanel" class="parameter-panel trajectory-panel">
        <div class="parameter-title">固定角度參數</div>
        <label>固定目標角度</label><input id="fixedTargetInput" type="number" value="60" min="-10" max="180" step="0.5">
      </div>
      <div id="trajectorySinePanel" class="parameter-panel trajectory-panel">
        <div class="parameter-title">正弦軌跡參數</div>
        <div class="grid2">
          <div><label>最小角度</label><input id="trajectoryMinInput" type="number" value="30" min="-10" max="180" step="0.5"></div>
          <div><label>最大角度</label><input id="trajectoryMaxInput" type="number" value="90" min="-10" max="180" step="0.5"></div>
          <div><label>正弦週期 秒</label><input id="trajectoryPeriodInput" type="number" value="5" min="0.5" max="120" step="0.1"></div>
        </div>
      </div>
      <div id="trajectoryStepPanel" class="parameter-panel trajectory-panel">
        <div class="parameter-title">階躍測試參數</div>
        <div class="grid2">
          <div><label>最小角度</label><input id="stepMinMirror" type="number" value="30" min="-10" max="180" step="0.5"></div>
          <div><label>最大角度</label><input id="stepMaxMirror" type="number" value="90" min="-10" max="180" step="0.5"></div>
          <div><label>階躍保持 秒</label><input id="stepHoldInput" type="number" value="3" min="0.1" max="60" step="0.1"></div>
        </div>
      </div>
      <label>Trial label</label><input id="labelInput" value="elbow_sine_PID">
      <div id="trajectoryModeSummary" class="mode-summary"></div>
    </div>

    <div class="card controller-card collapsible-card collapsed">
      <h2>4. 控制器設定</h2>
      <label>Controller Mode</label>
      <select id="controllerModeInput">
        <option value="pid" selected>PID（IMU 跟隨＋落後補償）</option>
        <option value="following_only">間歇取樣跟隨（無目標、無補償）</option>
        <option value="adrc">LADRC 自抗擾控制</option>
        <option value="ilc_pid">ILC＋PID（週期學習）</option>
        <option value="ilc_adrc">ILC＋LADRC（週期學習）</option>
        <option value="trajectory_demo">開迴路軌跡展示（僅測試）</option>
      </select>
      <div class="validation warning" id="demoModeWarning" style="display:none">展示模式只依目標速度輸出 PWM，不使用 IMU 誤差修正。僅限空載或固定測試架，禁止連接病患；此模式不能證明馬達位置精確跟隨角度。</div>
      <div id="controllerCommonPanel" class="parameter-panel active">
        <div class="parameter-title">共用安全限制</div>
        <label>大幅落後補償上限 PWM（0–255）</label><input id="outputLimitInput" type="number" value="55" min="0" max="255" step="1">
      </div>
      <div id="pidParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">PID 回授參數</div>
        <div class="grid2">
          <div><label>Kp</label><input id="kpInput" type="number" value="5" step="0.1"></div>
          <div><label>Ki</label><input id="kiInput" type="number" value="0.02" step="0.01"></div>
          <div><label>Kd</label><input id="kdInput" type="number" value="0.05" step="0.01"></div>
          <div><label>積分限制</label><input id="integralLimitInput" type="number" value="100" min="0" step="1"></div>
          <div><label>落後補償門檻 度</label><input id="deadbandInput" type="number" value="4" min="3" max="5" step="0.5"></div>
        </div>
      </div>
      <div id="feedforwardParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">IMU 動作跟隨模式</div>
        <div id="followingJointChooser" style="display:none">
          <label>跟隨關節</label>
          <select id="followingJointInput" onchange="selectFollowingJoint()">
            <option value="elbow">肘關節（二頭肌／三頭肌）</option>
            <option value="shoulder">肩關節（三角肌＋肘部繩索連動）</option>
          </select>
        </div>
        <label>取樣後固定輸出 PWM</label><input id="feedforwardMinInput" type="number" value="10" min="0" max="255" step="1">
        <p class="small">偵測到使用者動作後，以設定的 PWM 協助收放線。</p>
      </div>
      <div id="adrcParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">LADRC／ESO 參數</div>
        <div class="grid2">
          <div><label>控制頻寬 ωc (rad/s)</label><input id="adrcWcInput" type="number" value="2.0" min="0.1" max="20" step="0.1"></div>
          <div><label>觀測器頻寬 ωo (rad/s)</label><input id="adrcWoInput" type="number" value="8.0" min="0.3" max="60" step="0.1"></div>
          <div><label>輸入增益 b0</label><input id="adrcB0Input" type="number" value="1.0" step="0.1"></div>
        </div>
      </div>
      <div id="ilcParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">ILC 週期學習參數</div>
        <div class="grid2">
          <div><label>學習增益 L</label><input id="ilcGainInput" type="number" value="0.08" min="0" max="2" step="0.01"></div>
          <div><label>忘卻因子 λ</label><input id="ilcForgettingInput" type="number" value="0.98" min="0" max="1" step="0.01"></div>
          <div><label>Q-filter 視窗</label><input id="ilcQWindowInput" type="number" value="9" min="1" max="51" step="2"></div>
          <div><label>單周期更新上限</label><input id="ilcUpdateLimitInput" type="number" value="3" min="0" max="30" step="0.5"></div>
          <div><label>Learned PWM 上限</label><input id="ilcLearnedLimitInput" type="number" value="20" min="0" max="100" step="1"></div>
        </div>
      </div>
      <div id="demoParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">開迴路展示限制</div>
        <label>展示模式 PWM 上限（最高 100）</label><input id="demoOutputLimitInput" type="number" value="100" min="0" max="100" step="1">
      </div>
      <div id="controllerModeSummary" class="mode-summary"></div>
      <button onclick="applyParams()">套用關節、軌跡與控制參數</button>
      <div id="ilcActionPanel" class="parameter-panel">
        <div class="grid2"><button class="secondary" onclick="setILCFreeze(true)">凍結 ILC 學習</button><button class="secondary" onclick="setILCFreeze(false)">恢復 ILC 學習</button></div>
        <button class="warning" onclick="clearILCLearning()">清除 ILC 學習曲線</button>
        <div class="validation" id="ilcStatus">ILC 尚未執行。</div>
      </div>
    </div>

    <div class="card cable-card collapsible-card collapsed">
      <h2>5. 收放線比例與平滑設定</h2>
      <div class="parameter-title">肘關節對向放線</div>
      <div class="grid3">
        <div><label>二頭收線時：三頭放線（%）</label><input id="cableTricepsReleasePercent" type="number" value="50" min="5" max="100" step="5"></div>
        <div><label>三頭收線時：二頭放線（%）</label><input id="cableBicepsReleasePercent" type="number" value="80" min="5" max="100" step="5"></div>
        <div><label>控制器肩下放倍率（%）</label><input id="cableShoulderReleasePercent" type="number" value="100" min="10" max="150" step="5"></div>
      </div>
      <div class="parameter-title">肘關節漸進式軟著陸</div>
      <div class="grid3">
        <div><label>軟著陸中心角度</label><input id="elbowSoftLandingTarget" type="number" value="90" min="30" max="150" step="1"></div>
        <div><label>提前漸降範圍（度）</label><input id="elbowSoftLandingZone" type="number" value="25" min="5" max="60" step="1"></div>
        <div><label>接近中心最低輸出（%）</label><input id="elbowSoftLandingMinPercent" type="number" value="25" min="0" max="100" step="5"></div>
      </div>
      <div class="parameter-title">線長回補</div>
      <div class="grid3">
        <div><label>回補倍率（%）</label><input id="cableReturnGainPercent" type="number" value="100" min="100" max="200" step="5"></div>
        <div><label>回補時間（秒）</label><input id="cableReturnHorizon" type="number" value="1.25" min="0.2" max="5" step="0.05"></div>
        <div><label>最大額外放線 PWM</label><input id="cableReturnMaxPwm" type="number" value="60" min="0" max="255" step="5"></div>
      </div>
      <div class="parameter-title">肩肘繩索連動</div>
      <div class="grid3">
        <div><label>肩抬舉共同放線 PWM</label><input id="cableShoulderCouplingPwm" type="number" value="50" min="0" max="150" step="5"></div>
        <div><label>肩下放共同收線（%）</label><input id="cableShoulderRewindPercent" type="number" value="100" min="10" max="200" step="5"></div>
        <div><label>跟隨模式三頭收線上限</label><input id="cableTricepsWindLimit" type="number" value="100" min="0" max="255" step="5"></div>
      </div>
      <div class="parameter-title">輸出平滑</div>
      <div class="grid2">
        <div><label>收線爬升（PWM/秒）</label><input id="cableWindSlew" type="number" value="1200" min="50" max="5000" step="50"></div>
        <div><label>放線爬升（PWM/秒）</label><input id="cableReleaseSlew" type="number" value="2000" min="50" max="5000" step="50"></div>
      </div>
      <div class="parameter-title">三顆馬達全局輸出比例</div>
      <div class="grid3">
        <div><label>二頭肌（%）</label><input id="motorScaleBiceps" type="number" value="80" min="10" max="100" step="5"></div>
        <div><label>三頭肌（%）</label><input id="motorScaleTriceps" type="number" value="100" min="10" max="100" step="5"></div>
        <div><label>三角肌（%）</label><input id="motorScaleDeltoid" type="number" value="100" min="10" max="100" step="5"></div>
      </div>
      <p class="small">二頭肌全局比例不縮放肩肘共同收放線，確保肩部動作時二、三頭繩索仍等量。</p>
      <button class="warning" onclick="applyCableSettings()">停止馬達後套用收放線設定</button>
      <div class="status" id="cableSettingsStatus">尚未讀取設定</div>
    </div>

    <div class="card safety-card collapsible-card collapsed">
      <h2>6. 馬達安全控制</h2>
      <div class="validation warning" id="dryRunBanner">啟動前請確認馬達與繩索固定，並確保急停可立即操作。</div>
      <div class="grid2">
        <button class="success" onclick="setMotor(true)">Enable Motor</button>
        <button class="secondary" onclick="setMotor(false)">Disable Motor</button>
      </div>
      <button class="danger" onclick="emergencyStop()">Emergency Stop</button>
      <button class="warning" onclick="clearEmergency()">解除急停</button>
      <div class="validation" id="imuSafetyLockStatus">IMU 安全鎖：正常</div>
      <button class="warning" id="clearImuSafetyButton" onclick="clearImuSafetyFault()" style="display:none">確認姿勢與繩索，解除 IMU 安全鎖</button>
      <p class="small" id="motorInterlockText">馬達預設停止；停止或急停後需重新啟用。</p>
      <div class="parameter-panel active">
        <div class="parameter-title">繩索滑脫：單顆馬達按住點動</div>
        <div class="validation warning">限無人體負載時整理繩索。按住才轉動，放開即停止；安全鎖會停用該馬達的所有輸出。</div>
        <div class="grid2">
          <div><label>點動 PWM（1–200）</label><input id="motorJogPwm" type="number" value="100" min="1" max="200" step="1"></div>
          <div><label>安全確認</label><label><input id="motorJogConfirmed" type="checkbox" style="width:auto"> 已 Disable、無人體負載</label></div>
        </div>
        <div class="grid3">
          <div><label>二頭肌 Motor 1</label><label><input id="motorJogLockBiceps" class="motor-jog-lock" type="checkbox" style="width:auto" onchange="setMotorJogLock('biceps',this.checked)"> 全系統鎖定二頭肌</label><button class="success jog-button" data-jog-motor="biceps" onpointerdown="startMotorJog(event,'biceps',1)">按住正轉＋</button><button class="secondary jog-button" data-jog-motor="biceps" onpointerdown="startMotorJog(event,'biceps',-1)">按住反轉－</button></div>
          <div><label>三頭肌 Motor 2</label><label><input id="motorJogLockTriceps" class="motor-jog-lock" type="checkbox" style="width:auto" onchange="setMotorJogLock('triceps',this.checked)"> 全系統鎖定三頭肌</label><button class="success jog-button" data-jog-motor="triceps" onpointerdown="startMotorJog(event,'triceps',1)">按住正轉＋</button><button class="secondary jog-button" data-jog-motor="triceps" onpointerdown="startMotorJog(event,'triceps',-1)">按住反轉－</button></div>
          <div><label>三角肌 Motor 3</label><label><input id="motorJogLockDeltoid" class="motor-jog-lock" type="checkbox" style="width:auto" onchange="setMotorJogLock('deltoid',this.checked)"> 全系統鎖定三角肌</label><button class="success jog-button" data-jog-motor="deltoid" onpointerdown="startMotorJog(event,'deltoid',1)">按住正轉＋</button><button class="secondary jog-button" data-jog-motor="deltoid" onpointerdown="startMotorJog(event,'deltoid',-1)">按住反轉－</button></div>
        </div>
        <button class="danger" onpointerdown="event.preventDefault();stopMotorJog(true)">立即停止點動</button>
        <div class="status" id="motorJogStatus">點動待機｜預設 100 PWM</div>
      </div>
    </div>

    <div class="card frequency-card collapsible-card collapsed">
      <h2>7. 系統識別與頻域分析</h2>
      <div class="validation warning">危險測試：Chirp／PRBS 會自動改變馬達方向。僅限空載或固定測試架，禁止連接人體。開始後系統會自行 ARM；完成、超出安全角度或通訊異常時自動 STOP。</div>
      <div class="grid2">
        <div><label>測試關節</label><select id="freqJointInput"><option value="elbow">肘關節</option><option value="shoulder">肩關節</option></select></div>
        <div><label>激振訊號</label><select id="freqSignalInput"><option value="chirp">Chirp 掃頻</option><option value="prbs">PRBS</option><option value="sine">單頻正弦</option></select></div>
        <div><label>PWM 振幅（1–30）</label><input id="freqAmplitudeInput" type="number" value="12" min="1" max="30" step="1"></div>
        <div><label>測試時間（10–180 秒）</label><input id="freqDurationInput" type="number" value="60" min="10" max="180" step="1"></div>
        <div><label>起始／單頻 Hz</label><input id="freqStartInput" type="number" value="0.1" min="0.05" max="10" step="0.05"></div>
        <div><label>結束頻率 Hz</label><input id="freqEndInput" type="number" value="3.0" min="0.05" max="10" step="0.05"></div>
        <div><label>PRBS 切換率 Hz</label><input id="freqPrbsRateInput" type="number" value="2.0" min="0.1" max="20" step="0.1"></div>
        <div><label>安全最小角度</label><input id="freqSafeMinInput" type="number" value="-10" min="-30" max="180" step="1"></div>
        <div><label>安全最大角度</label><input id="freqSafeMaxInput" type="number" value="120" min="-30" max="180" step="1"></div>
      </div>
      <button class="danger" onclick="startFrequencyTest()">確認固定測試架並開始</button>
      <button class="secondary" onclick="stopFrequencyTest()">停止測試並送出 STOP</button>
      <button onclick="analyzeFrequencyTest()">分析 Bode／Coherence／ARX</button>
      <div class="validation" id="frequencyStatus">尚未執行系統識別。</div>
      <canvas id="frequencyCanvas" width="950" height="430" style="display:none"></canvas>
    </div>

    <div class="card record-card collapsible-card collapsed">
      <h2>8. 資料紀錄</h2>
      <div class="parameter-title">20 秒簡報快速測試</div>
      <div class="grid3 record-fields">
        <div><label>測試條件</label><select id="quickConditionInput" onchange="updateQuickFlowInputs()"><option value="no_assist">無馬達輔助</option><option value="following_only">純跟隨模式</option><option value="pid">PID 跟隨＋補償</option><option value="adrc">LADRC 跟隨＋自抗擾補償</option></select></div>
        <div><label>測試時間（秒）</label><input id="quickDurationInput" type="number" value="20" min="10" max="120" step="5"></div>
        <div><label>誤差合格範圍（±度）</label><input id="quickToleranceInput" type="number" value="5" min="1" max="20" step="1"></div>
      </div>
      <div class="validation" id="quickParameterSummary">正在讀取主控制面板參數...</div>
      <div class="record-actions">
        <button class="success" id="quickStartButton" onclick="runQuickFullFlow()">套用設定 → 啟動 → 3 秒倒數 → 自動記錄</button>
        <button class="danger" id="quickStopButton" onclick="cancelQuickFullFlowOrTest()" disabled>取消流程／提前停止</button>
      </div>
      <div class="progress-track"><div class="progress-bar" id="quickTestProgress"></div></div>
      <div class="validation" id="quickTestStatus">在此完成全部設定。按一次後會自動套用、啟動、倒數與記錄；測試結束自動停止馬達。</div>
      <canvas id="quickTestCanvas" class="quick-test-canvas" width="1100" height="680"></canvas>
      <canvas id="quickSignalCanvas" class="quick-test-canvas" width="1100" height="680"></canvas>
      <div class="parameter-title" style="margin-top:10px">比較圖要顯示的測試紀錄</div>
      <div class="quick-run-selector" id="quickRunSelector"><span class="small">完成測試後，可勾選要放進比較圖的紀錄。</span></div>
      <canvas id="quickCompareCanvas" class="quick-test-canvas" width="1100" height="620"></canvas>
      <div class="grid2">
        <button id="quickDownloadPngButton" onclick="downloadQuickTestPng()" disabled>下載本次 PNG</button>
        <button id="quickDownloadSignalsButton" onclick="downloadQuickSignalsPng()" disabled>下載誤差／PWM 圖</button>
        <button id="quickDownloadCsvButton" onclick="downloadQuickTestCsv()" disabled>下載本次 CSV</button>
        <button id="quickDownloadCompareButton" onclick="downloadQuickComparePng()" disabled>下載比較圖 PNG</button>
      </div>
      <button class="secondary" onclick="clearQuickPresentationRuns()">清除快速測試結果</button>
      <hr style="border:0;border-top:1px solid #e5e7eb;margin:13px 0">
      <div class="parameter-title">完整原始資料紀錄</div>
      <div class="grid3 record-fields">
        <div><label>受試者 ID</label><input id="participantInput" value="S01"></div>
        <div><label>Session</label><input id="sessionInput" value="control_session_01"></div>
        <div><label>輸出資料夾</label><input id="outputDirInput" value="control_data"></div>
      </div>
      <div class="record-actions">
        <button class="success" onclick="startRecording()">開始錄製 CSV</button>
        <button class="danger" onclick="stopRecording()">停止錄製 CSV</button>
      </div>
      <p id="recordingState" class="idle">未錄製</p>
      <p class="small" id="csvPathText"></p>
    </div>
  </div>

  <div class="monitor-column">
    <div class="card live-card">
      <h2>即時資料</h2>
      <div class="metrics">
        <div class="metric"><div class="metric-title" id="measuredTitle">測量角度 measured</div><div class="metric-value" id="measuredAngle">--</div></div>
        <div class="metric"><div class="metric-title">目標 target</div><div class="metric-value" id="targetAngle">--</div></div>
        <div class="metric"><div class="metric-title">誤差 error</div><div class="metric-value" id="errorValue">--</div></div>
        <div class="metric"><div class="metric-title">肘角 elbow</div><div class="metric-value" id="elbowAngle">--</div></div>
        <div class="metric"><div class="metric-title">肩角 shoulder</div><div class="metric-value" id="shoulderAngle">--</div></div>
        <div class="metric"><div class="metric-title">控制狀態</div><div class="metric-value" id="motionState" style="font-size:16px">--</div></div>
      </div>
    </div>
    <div class="card chart-card">
      <h2>Target vs Measured</h2>
      <canvas id="angleCanvas" width="1100" height="680"></canvas>
      <p class="small">灰線：目標｜藍線：實際角度｜紅線：追蹤誤差｜垂直線：現在</p>
    </div>
  </div>
</div>
<div id="actionToast" class="action-toast info" role="status" aria-live="polite"></div>
<script>
let measuredHistory = [];
let errorHistory = [];
let timeHistory = [];
let latestTime = 0;
let validationFinishing = false;
let validationAutoRun = false;
let validationStageStarting = false;
let validationPreparing = false;
let statusMessageUntil = 0;
let validationGuideMessageUntil = 0;
let lastActionButton = null;
let actionToastTimer = null;
let plotFramePending = false;
let lastSlowUiUpdate = -Infinity;
const slowUiIntervalMs = 500;
const livePollDelayMs = 50;
const fullStatusPollDelayMs = 500;
let livePollFailures = 0;
let motorJogHeld = false;
let motorJogActive = false;
let motorJogStarting = false;
let motorJogGeneration = 0;
let motorJogButton = null;
let motorJogHeartbeatTimer = null;
let motorJogHeartbeatBusy = false;
let motorScaleLoaded = false;
let cableSettingsLoaded = false;
let quickTestActive = false;
let quickTestStartedAt = 0;
let quickTestDuration = 20;
let quickTestTolerance = 5;
let quickTestSamples = [];
let quickTestCurrent = null;
let quickTestRuns = [];
let quickTestTimer = null;
let quickFlowRunning = false;
let quickFlowCancelled = false;
let latestMotorOutputStatus = null;

// 顯示視窗設定：目前時間會畫在圖中間偏左，右側保留未來目標軌跡給使用者預看。
// The visible plot only needs the recent window. Keeping 1200 browser-side
// points made long-running sessions progressively slower.
const maxPoints = 160;
const pastWindowSeconds = 8;
const futureWindowSeconds = 8;

function setText(id, text){
  const node=document.getElementById(id),value=String(text ?? '');
  if(node && node.innerText!==value)node.innerText=value;
}
function schedulePlot(){
  if(plotFramePending||document.hidden)return;
  plotFramePending=true;
  requestAnimationFrame(()=>{plotFramePending=false;drawPlot();});
}
function showStatus(text,holdMs=2500){statusMessageUntil=Date.now()+holdMs;setText('statusBox',text);}
function showValidationGuide(text,holdMs=2500){validationGuideMessageUntil=Date.now()+holdMs;setText('validationGuide',text);}
function setLiveStatus(text){if(Date.now()>=statusMessageUntil)setText('statusBox',text);}
function setLiveValidationGuide(text){if(Date.now()>=validationGuideMessageUntil)setText('validationGuide',text);}
function showActionToast(text,type='info',holdMs=1600){
  const toast=document.getElementById('actionToast');
  toast.textContent=text;toast.className='action-toast '+type+' show';
  clearTimeout(actionToastTimer);actionToastTimer=setTimeout(()=>toast.classList.remove('show'),holdMs);
}
function setCardCollapsed(card,collapsed){
  card.classList.toggle('collapsed',collapsed);
  const heading=card.querySelector(':scope>h2');
  if(heading)heading.setAttribute('aria-expanded',String(!collapsed));
}
function setupCollapsibleCards(){
  document.querySelectorAll('.collapsible-card').forEach(card=>{
    const heading=card.querySelector(':scope>h2');
    if(!heading)return;
    heading.setAttribute('role','button');
    heading.setAttribute('tabindex','0');
    heading.setAttribute('aria-expanded',String(!card.classList.contains('collapsed')));
    const toggle=()=>setCardCollapsed(card,!card.classList.contains('collapsed'));
    heading.addEventListener('click',toggle);
    heading.addEventListener('keydown',event=>{
      if(event.key==='Enter'||event.key===' '){event.preventDefault();toggle();}
    });
  });
}
function claimActionButton(){const button=lastActionButton;lastActionButton=null;return button;}
function setButtonBusy(button,busy){
  if(!button)return;
  if(busy){button.dataset.actionLabel=button.textContent;button.classList.add('button-busy');button.disabled=true;button.textContent='處理中…';}
  else{button.classList.remove('button-busy');button.disabled=false;button.textContent=button.dataset.actionLabel||button.textContent;}
}
function finishButtonAction(button,ok){
  if(!button)return;
  setButtonBusy(button,false);button.classList.add(ok?'button-success':'button-error');
  const original=button.dataset.actionLabel||button.textContent;button.textContent=ok?'✓ 已完成':'⚠ 執行失敗';
  setTimeout(()=>{button.classList.remove('button-success','button-error');button.textContent=original;},1100);
}
document.addEventListener('click',event=>{
  const button=event.target.closest('button');if(!button||button.disabled)return;
  if(button.classList.contains('jog-button'))return;
  lastActionButton=button;button.classList.add('button-pressed');setTimeout(()=>button.classList.remove('button-pressed'),180);
  showActionToast('已按下：'+button.textContent.trim(),'info',900);
},true);
async function requestJSON(url,options={}){
  const controller=new AbortController();
  const actionButton=options.actionButton||null;
  const timeoutMs=options.timeoutMs||5000;
  const fetchOptions={...options};delete fetchOptions.timeoutMs;delete fetchOptions.actionButton;
  const timeout=setTimeout(()=>controller.abort(),timeoutMs);
  setButtonBusy(actionButton,true);
  try{
    const response=await fetch(url,{cache:'no-store',...fetchOptions,signal:controller.signal});
    let data={};
    try{data=await response.json();}catch(e){data={ok:false,message:'伺服器回傳格式錯誤。'};}
    if(!response.ok && !data.message)data.message='伺服器錯誤：HTTP '+response.status;
    finishButtonAction(actionButton,data.ok!==false&&response.ok);
    if(actionButton)showActionToast(data.message||(response.ok?'操作完成':'操作失敗'),response.ok&&data.ok!==false?'success':'error',2200);
    return data;
  }catch(e){
    const message=e.name==='AbortError'?'伺服器回應逾時，請檢查樹莓派連線。':'無法連線到 Flask Server：'+e.message;
    finishButtonAction(actionButton,false);if(actionButton)showActionToast(message,'error',2500);
    return {ok:false,message};
  }finally{clearTimeout(timeout);}
}
async function postJSON(url,data){return await requestJSON(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data||{}),actionButton:claimActionButton()});}
async function postJSONLong(url,data){return await requestJSON(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data||{}),timeoutMs:30000,actionButton:claimActionButton()});}
async function getJSON(url){return await requestJSON(url);}
function fval(id, def){const v=parseFloat(document.getElementById(id).value); return Number.isFinite(v)?v:def;}
function sleep(ms){return new Promise(resolve=>setTimeout(resolve,ms));}

const validationStageDetails={
  elbow_flexion:{
    title:'動作 1｜手臂下垂－最大二頭肌屈曲',
    body:'開始時上臂自然垂在身側、肘伸直。計時期間上臂保持不動，自由重複「肘完全伸直 ↔ 舒服且無痛的最大屈曲」。不需要配合固定節奏或停留；系統會自動選擇肘關節主要軸、方向及最大範圍。'
  },
  elbow_extension:{
    title:'動作 2｜肘 90°－三頭肌伸展至打直',
    body:'開始時上臂自然垂在身側、肘約 90°。計時期間自由重複「肘約 90° ↔ 完全伸直或舒服的微過伸」。不要把上臂舉過頭；系統會自動找出最伸直位置。'
  },
  shoulder_front:{
    title:'動作 3｜最大前平舉',
    body:'開始時手臂自然下垂、肘伸直。計時期間沿身體正前方自由重複「自然下垂 ↔ 舒服且無痛的最大前平舉」。系統會自動找最高角度與前舉方向特徵。'
  },
  shoulder_side:{
    title:'動作 4｜最大側平舉',
    body:'開始時手臂自然下垂、肘伸直。計時期間沿身體側面自由重複「自然下垂 ↔ 舒服且無痛的最大側平舉」。系統會自動找最高角度與側舉方向特徵。'
  }
};

function showValidationProtocol(stage){
  const detail=validationStageDetails[stage];
  setText('validationProtocol',detail?(detail.title+'\n'+detail.body):'等待 ROM 記錄階段。');
}

function params(){return {target_joint:document.getElementById('targetJointInput').value,target_mode:document.getElementById('targetModeInput').value,fixed_target_angle:fval('fixedTargetInput',60),trajectory_min_angle:fval('trajectoryMinInput',30),trajectory_max_angle:fval('trajectoryMaxInput',90),trajectory_period:fval('trajectoryPeriodInput',5),step_hold_time:fval('stepHoldInput',3),controller_mode:document.getElementById('controllerModeInput').value,deadband:Math.min(5,Math.max(3,fval('deadbandInput',4))),assist_delay:.10,assist_feedforward_gain:0,assist_feedforward_min_pwm:Math.min(Math.abs(fval('feedforwardMinInput',10)),255),demo_feedforward_gain:.35,demo_min_pwm:8,demo_output_limit:Math.min(Math.abs(fval('demoOutputLimitInput',100)),100),demo_start_delay:1.0,adrc_controller_bandwidth:fval('adrcWcInput',2),adrc_observer_bandwidth:fval('adrcWoInput',8),adrc_input_gain:fval('adrcB0Input',1),ilc_learning_gain:fval('ilcGainInput',.08),ilc_forgetting_factor:fval('ilcForgettingInput',.98),ilc_q_filter_window:Math.round(fval('ilcQWindowInput',9)),ilc_update_limit:fval('ilcUpdateLimitInput',3),ilc_learned_limit:fval('ilcLearnedLimitInput',20),ilc_bins:200,output_limit:Math.min(Math.abs(fval('outputLimitInput',55)),255),kp:fval('kpInput',5),ki:fval('kiInput',.02),kd:fval('kdInput',.05),integral_limit:fval('integralLimitInput',100)};}

const parameterStorageKey='imuRehabGuiParametersV1';
const persistentParameterIds=[
  'targetJointInput','targetModeInput','controllerModeInput','fixedTargetInput',
  'trajectoryMinInput','trajectoryMaxInput','trajectoryPeriodInput','stepHoldInput',
  'outputLimitInput','deadbandInput','feedforwardMinInput',
  'demoOutputLimitInput','adrcWcInput','adrcWoInput','adrcB0Input','ilcGainInput',
  'ilcForgettingInput','ilcQWindowInput','ilcUpdateLimitInput','ilcLearnedLimitInput',
  'kpInput','kiInput','kdInput','integralLimitInput','romDurationInput','motorJogPwm',
  'motorScaleBiceps','motorScaleTriceps','motorScaleDeltoid',
  'cableTricepsReleasePercent','cableBicepsReleasePercent','cableShoulderReleasePercent',
  'cableReturnGainPercent','cableReturnHorizon','cableReturnMaxPwm',
  'cableShoulderCouplingPwm','cableShoulderRewindPercent','cableTricepsWindLimit',
  'cableWindSlew','cableReleaseSlew','elbowSoftLandingTarget',
  'elbowSoftLandingZone','elbowSoftLandingMinPercent',
  'quickConditionInput','quickDurationInput','quickToleranceInput'
];
function saveParameterInputs(){
  try{
    const values={};
    persistentParameterIds.forEach(id=>{const element=document.getElementById(id);if(element)values[id]=element.value;});
    localStorage.setItem(parameterStorageKey,JSON.stringify(values));
  }catch(e){}
}
function restoreParameterInputs(){
  try{
    const values=JSON.parse(localStorage.getItem(parameterStorageKey)||'{}');
    // One-time repair for the removed quick-test duplicate controls.  Earlier
    // versions copied quickFollowingPwm into the real controller field; when
    // both saved values match, restore the project's previously tuned 10 PWM.
    if(!values.quickParameterInheritanceFixedV2){
      if(values.quickFollowingPwm!==undefined&&String(values.feedforwardMinInput)===String(values.quickFollowingPwm))values.feedforwardMinInput='10';
      ['quickFollowingPwm','quickOutputLimit','quickKp','quickKi','quickKd','quickAdrcWc','quickAdrcWo','quickAdrcB0','quickJointInput','quickTrajectoryMin','quickTrajectoryMax','quickTrajectoryPeriod'].forEach(id=>delete values[id]);
      values.quickParameterInheritanceFixedV2=true;
      localStorage.setItem(parameterStorageKey,JSON.stringify(values));
    }
    persistentParameterIds.forEach(id=>{
      const element=document.getElementById(id);
      if(element && Object.prototype.hasOwnProperty.call(values,id))element.value=values[id];
    });
  }catch(e){}
}
persistentParameterIds.forEach(id=>{
  const element=document.getElementById(id);
  if(element)element.addEventListener('change',()=>{saveParameterInputs();updateQuickFlowInputs();});
});

function resetPlot(){
  measuredHistory=[];
  errorHistory=[];
  timeHistory=[];
  latestTime=0;
  drawPlot();
}

function updateJointLabels(){
  const j=document.getElementById('targetJointInput').value;
  const mode=document.getElementById('targetModeInput').value;
  const controller=document.getElementById('controllerModeInput').value;
  if(j==='shoulder'){
    setText('measuredTitle','肩關節 measured');
    document.getElementById('labelInput').value='shoulder_'+mode+'_'+controller;
  }else{
    setText('measuredTitle','肘關節 measured');
    document.getElementById('labelInput').value='elbow_'+mode+'_'+controller;
  }
  updateTrajectoryPanels();
  resetPlot();
}

function syncFollowingJointSelector(){
  const source=document.getElementById('targetJointInput');
  const mirror=document.getElementById('followingJointInput');
  if(source&&mirror)mirror.value=source.value;
}

function selectFollowingJoint(){
  const source=document.getElementById('targetJointInput');
  const mirror=document.getElementById('followingJointInput');
  if(!source||!mirror)return;
  source.value=mirror.value;
  updateJointLabels();
  saveParameterInputs();
  showActionToast('已選擇跟隨'+(mirror.value==='shoulder'?'肩關節':'肘關節')+'，請按「套用任務與參數」。','success',1800);
}

function showOnlyPanels(selector,activeIds){
  document.querySelectorAll(selector).forEach(panel=>panel.classList.toggle('active',activeIds.includes(panel.id)));
}

function updateTrajectoryPanels(){
  const mode=document.getElementById('targetModeInput').value;
  const map={fixed:['trajectoryFixedPanel'],sine:['trajectorySinePanel'],step:['trajectoryStepPanel']};
  showOnlyPanels('.trajectory-panel',map[mode]||map.sine);
  const descriptions={
    fixed:'固定角度：控制器持續追蹤單一目標角度。',
    sine:'正弦軌跡：在最小與最大角度之間平滑往返。',
    step:'階躍測試：在最小與最大角度間依保持時間切換。'
  };
  setText('trajectoryModeSummary',descriptions[mode]||descriptions.sine);
  if(mode==='step'){
    document.getElementById('stepMinMirror').value=document.getElementById('trajectoryMinInput').value;
    document.getElementById('stepMaxMirror').value=document.getElementById('trajectoryMaxInput').value;
  }
}

function updateControllerMode(){
  const controllerMode=document.getElementById('controllerModeInput').value;
  const demo=controllerMode==='trajectory_demo';
  const followingOnly=controllerMode==='following_only';
  const periodic=demo||controllerMode==='ilc_pid'||controllerMode==='ilc_adrc';
  const target=document.getElementById('targetModeInput');
  // Pure following ignores the trajectory but must still know whether the
  // user is training the elbow or shoulder; the two tendon strategies conflict.
  document.getElementById('targetJointInput').disabled=false;
  document.getElementById('followingJointChooser').style.display=followingOnly?'block':'none';
  syncFollowingJointSelector();
  target.disabled=followingOnly;
  document.querySelectorAll('.trajectory-panel input').forEach(input=>input.disabled=followingOnly);
  document.getElementById('controllerCommonPanel').style.display=(followingOnly||demo)?'none':'block';
  document.getElementById('demoModeWarning').style.display=demo?'block':'none';
  Array.from(target.options).forEach(option=>{option.disabled=periodic && option.value!=='sine';});
  if(periodic && target.value!=='sine'){
    target.value='sine';
  }
  const panelMap={
    pid:['pidParameterPanel','feedforwardParameterPanel'],
    following_only:['feedforwardParameterPanel'],
    adrc:['adrcParameterPanel','feedforwardParameterPanel'],
    ilc_pid:['pidParameterPanel','ilcParameterPanel','feedforwardParameterPanel'],
    ilc_adrc:['adrcParameterPanel','ilcParameterPanel','feedforwardParameterPanel'],
    trajectory_demo:['demoParameterPanel']
  };
  showOnlyPanels('.controller-panel',panelMap[controllerMode]||panelMap.pid);
  document.getElementById('ilcActionPanel').classList.toggle('active',controllerMode==='ilc_pid'||controllerMode==='ilc_adrc');
  const descriptions={
    pid:'PID：馬達先依 IMU 實際動作收放線；大幅落後目標時才追加 PID 補償。',
    following_only:'折衷取樣跟隨：請先選擇肘或肩關節，只跟隨所選關節；40 ms 隔離＋40 ms 取樣＋200 ms 輸出，忽略目標軌跡。',
    adrc:'LADRC：IMU 實際動作負責收放線；大幅落後時由 ESO／LADRC 補償。',
    ilc_pid:'ILC＋PID：先跟隨病患實際動作；大幅落後時才使用 PID 與學習補償。',
    ilc_adrc:'ILC＋LADRC：先跟隨病患實際動作；大幅落後時才使用 LADRC 與學習補償。',
    trajectory_demo:'開迴路展示：只依目標速度輸出，不使用 IMU 誤差修正；禁止連接人體。'
  };
  setText('controllerModeSummary',descriptions[controllerMode]||descriptions.pid);
  updateJointLabels();
}

document.getElementById('targetModeInput').addEventListener('change', updateJointLabels);
document.getElementById('targetJointInput').addEventListener('change',()=>{syncFollowingJointSelector();updateJointLabels();});
document.getElementById('controllerModeInput').addEventListener('change', updateControllerMode);
['trajectoryMinInput','trajectoryMaxInput','trajectoryPeriodInput','stepHoldInput'].forEach(id=>{
  document.getElementById(id).addEventListener('change', resetPlot);
});
document.getElementById('stepMinMirror').addEventListener('input',event=>{document.getElementById('trajectoryMinInput').value=event.target.value;resetPlot();});
document.getElementById('stepMaxMirror').addEventListener('input',event=>{document.getElementById('trajectoryMaxInput').value=event.target.value;resetPlot();});

function getTrajectoryCenter(){
  let minA=fval('trajectoryMinInput',30);
  let maxA=fval('trajectoryMaxInput',90);
  if(maxA<minA){const tmp=minA; minA=maxA; maxA=tmp;}
  return 0.5*(minA+maxA);
}

async function initializeSystem(){
  resetPlot();
  showStatus('初始化與 5 秒校正中，請保持手臂自然下垂靜止...',1000);
  setText('validationProtocol','5 秒校正姿勢\n站直或坐直，肩膀放鬆，手臂自然垂在身側。\n肘完全伸直，前臂保持中立、手掌朝向身體。\n校正期間不要移動、不要碰觸 IMU。');
  const res=await postJSON('/api/init',params());
  showStatus(res.message||'初始化請求沒有回傳訊息。');
}
async function startValidation(){
  if(validationPreparing||validationStageStarting){
    showValidationGuide('目前正在準備或啟動 ROM 階段，請稍候。');
    return;
  }
  showValidationGuide('正在建立新的個人化 ROM 記錄...',800);
  showStatus('正在重設個人化 ROM...',800);
  const duration=Math.max(5,Math.min(30,fval('romDurationInput',10)));
  document.getElementById('romDurationInput').value=duration;
  const res=await postJSON('/api/rom/start',{duration_seconds:duration});
  if(!res.ok){showStatus(res.message);showValidationGuide(res.message);return;}
  showStatus(res.message||'個人化 ROM 已開始。');
  validationFinishing=false;
  validationAutoRun=false;
  await refreshStatus();
}
async function prepareAndStartValidationStage(){
  if(validationPreparing||validationStageStarting){
    showValidationGuide('準備倒數或階段啟動已在進行，請勿重複點擊。');
    return;
  }
  validationPreparing=true;
  document.getElementById('validationStageButton').disabled=true;
  document.getElementById('validationProgressBar').style.width='0%';
  setText('validationGuide','按鍵已收到，正在確認目前 ROM 階段...');
  showStatus('正在準備 ROM 動作...');
  const data=await getJSON('/api/status');
  if(data.ok===false && !data.rom_calibration){
    validationPreparing=false;
    showValidationGuide(data.message);
    showStatus(data.message);
    return;
  }
  const stage=(data.rom_calibration||{}).expected_stage;
  if(!stage){
    validationPreparing=false;
    const message=(data.rom_calibration||{}).current_stage?'目前已有動作正在記錄。':'目前沒有可開始的 ROM 階段，請先按「開始／重新記錄個人化 ROM」。';
    showValidationGuide(message);
    showStatus(message);
    await refreshStatus();
    return;
  }
  validationAutoRun=true;
  showValidationProtocol(stage);
  for(let count=2;count>=1;count--){
    const detail=validationStageDetails[stage];
    setText('validationGuide',(detail?detail.title:stage)+'\n按鍵已收到｜請擺好起始姿勢，'+count+' 秒後開始。');
    await sleep(1000);
  }
  validationPreparing=false;
  await startValidationStage();
}
async function startValidationStage(){
  if(validationStageStarting){showValidationGuide('ROM 階段正在啟動，請稍候。');return;}
  validationStageStarting=true;
  setText('validationGuide','倒數完成，正在啟動資料記錄...');
  const res=await postJSON('/api/rom/start_stage',{});
  validationStageStarting=false;
  if(!res.ok){validationAutoRun=false;showStatus(res.message);showValidationGuide(res.message);await refreshStatus();return;}
  validationFinishing=false;
  showStatus(res.message||'ROM 階段已開始。');
  await refreshStatus();
}
async function finishValidationStage(){
  if(validationFinishing)return;
  validationFinishing=true;
  showStatus('正在整理本階段 ROM 資料...');
  const res=await postJSON('/api/rom/finish_stage',{});
  showStatus(res.message||'ROM 階段沒有回傳訊息。');
  await refreshStatus();
  validationFinishing=false;
  if(res.ok && validationAutoRun && res.rom_calibration && res.rom_calibration.expected_stage){
    setTimeout(prepareAndStartValidationStage,400);
  }else if(res.ok){
    validationAutoRun=false;
  }
}
async function applyValidation(){
  showStatus('正在套用個人化 ROM...');
  const res=await postJSON('/api/rom/apply',{});
  if(!res.ok){showStatus(res.message);return;}
  if(res.rom){
    document.getElementById('trajectoryMinInput').value=fmtMetric(res.rom.elbow_extension_target_deg,1);
    document.getElementById('trajectoryMaxInput').value=fmtMetric(res.rom.elbow_flexion_target_deg,1);
    document.getElementById('fixedTargetInput').value=fmtMetric(res.rom.elbow_flexion_target_deg,1);
    saveParameterInputs();
    resetPlot();
  }
  showStatus(res.message);
  await refreshStatus();
}
async function refreshStatus(){
  const data=await requestJSON('/api/status',{actionButton:claimActionButton()});
  if(data.ok===false && !data.status){showStatus(data.message);return data;}
  updatePage(data);
  return data;
}
function syncAppliedControlSettings(applied){
  if(!applied)return;
  const direct={
    target_joint:'targetJointInput',target_mode:'targetModeInput',controller_mode:'controllerModeInput',
    fixed_target_angle:'fixedTargetInput',trajectory_min_angle:'trajectoryMinInput',
    trajectory_max_angle:'trajectoryMaxInput',trajectory_period:'trajectoryPeriodInput',
    step_hold_time:'stepHoldInput',deadband_on:'deadbandInput',
    assist_feedforward_min_pwm:'feedforwardMinInput',demo_output_limit:'demoOutputLimitInput',
    adrc_controller_bandwidth:'adrcWcInput',adrc_observer_bandwidth:'adrcWoInput',
    adrc_input_gain:'adrcB0Input',ilc_learning_gain:'ilcGainInput',
    ilc_forgetting_factor:'ilcForgettingInput',ilc_q_filter_window:'ilcQWindowInput',
    ilc_update_limit:'ilcUpdateLimitInput',ilc_learned_limit:'ilcLearnedLimitInput',
    output_limit:'outputLimitInput',kp:'kpInput',ki:'kiInput',kd:'kdInput',
    integral_limit:'integralLimitInput'
  };
  Object.entries(direct).forEach(([key,id])=>{
    const element=document.getElementById(id),value=applied[key];
    if(element&&value!==null&&value!==undefined)element.value=value;
  });
  document.getElementById('stepMinMirror').value=document.getElementById('trajectoryMinInput').value;
  document.getElementById('stepMaxMirror').value=document.getElementById('trajectoryMaxInput').value;
  updateControllerMode();
}
async function applyParams(){
  showStatus('正在套用控制參數...');
  const res=await postJSON('/api/apply_params',params());
  showStatus(res.message||'控制參數沒有回傳訊息。');
  if(!res.ok)return;
  syncAppliedControlSettings(res.applied);
  saveParameterInputs();
  resetPlot();
}
async function setILCFreeze(frozen){const res=await postJSON('/api/ilc/freeze',{frozen});showStatus(res.message||'ILC 狀態已更新。');await refreshStatus();}
async function clearILCLearning(){if(!window.confirm('確定清除目前 ILC learned feedforward 與所有周期統計嗎？'))return;const res=await postJSON('/api/ilc/reset',{});showStatus(res.message||'ILC 已重設。');await refreshStatus();}
async function setMotor(enabled){
  let confirmed=false;
  if(enabled){
    const demo=document.getElementById('controllerModeInput').value==='trajectory_demo';
    const warning=demo?'\n5. 展示模式僅限空載／固定測試架，禁止連接人體':'';
    confirmed=window.confirm('即將啟動實際馬達輸出。\n\n請確認：\n1. 馬達與驅動板已固定\n2. 繩索尚未連接人體\n3. 急停可以立即操作\n4. PWM 上限與轉向已確認'+warning+'\n\n確定要啟動嗎？');
    if(!confirmed){showStatus('已取消啟動，馬達維持停止。');return;}
  }
  showStatus(enabled?'正在連線 ESP32 並執行 ARM 確認...':'正在停用馬達...');
  const res=await requestJSON('/api/set_motor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled,confirmed}),timeoutMs:10000,actionButton:claimActionButton()});
  await sleep(120);
  const actual=await getJSON('/api/status');
  const actuallyEnabled=Boolean(actual.motor_enabled);
  if(enabled&&actuallyEnabled&&actual.motor_bridge_connected&&!actual.motor_arm_in_progress){
    showStatus('ESP32 ARM 已確認，馬達輸出已啟用。');
  }else if(enabled&&actual.motor_arm_in_progress){
    showStatus('ESP32 仍在 ARM 確認中；確認完成前不會輸出 PWM。');
  }else if(!enabled&&!actuallyEnabled){
    showStatus('Motor disabled：已送 STOP 給 ESP32。');
  }else{
    showStatus(res.message||'ARM 後端與實際狀態不一致，馬達已保持停止。');
  }
}
async function emergencyStop(){showStatus('正在執行 Emergency Stop...');const res=await postJSON('/api/emergency_stop',{});showStatus(res.message||'急停沒有回傳訊息。');}
async function clearEmergency(){showStatus('正在解除急停...');const res=await postJSON('/api/clear_emergency',{});showStatus(res.message||'解除急停沒有回傳訊息。');}
async function clearImuSafetyFault(){
  const confirmed=window.confirm('解除後仍不會自動啟動馬達。\n\n請確認：\n1. 病患姿勢與畫面角度一致\n2. 二頭、三頭、三角肌繩索鬆緊正常\n3. 必要時已用單顆點動重新理線\n\n解除時會把目前繩索狀態當作新的軟體基準。確定解除嗎？');
  if(!confirmed){showStatus('已取消，IMU 安全鎖維持。');return;}
  const res=await postJSON('/api/imu_fault/clear',{confirmed:true});
  showStatus(res.message||'IMU 安全鎖狀態沒有回傳訊息。');
  await refreshStatus();
}
function clearMotorJogUi(message='點動待機｜預設 100 PWM'){
  clearInterval(motorJogHeartbeatTimer);motorJogHeartbeatTimer=null;
  motorJogHeartbeatBusy=false;motorJogActive=false;motorJogStarting=false;
  if(motorJogButton)motorJogButton.classList.remove('button-busy','jog-active');
  motorJogButton=null;setText('motorJogStatus',message);
}
async function motorJogHeartbeat(){
  if(!motorJogActive||motorJogHeartbeatBusy)return;
  motorJogHeartbeatBusy=true;
  try{
    const response=await fetch('/api/motor_jog/heartbeat',{method:'POST',cache:'no-store',keepalive:true});
    if(!response.ok){motorJogHeld=false;await stopMotorJog(false);}
  }catch(e){motorJogHeld=false;await stopMotorJog(false);}
  finally{motorJogHeartbeatBusy=false;}
}
function updateMotorJogLocks(locks){
  const values=locks||{};
  const ids={biceps:'motorJogLockBiceps',triceps:'motorJogLockTriceps',deltoid:'motorJogLockDeltoid'};
  Object.entries(ids).forEach(([motor,id])=>{
    const locked=values[motor]===true;
    const checkbox=document.getElementById(id);
    if(checkbox)checkbox.checked=locked;
    document.querySelectorAll('.jog-button[data-jog-motor="'+motor+'"]').forEach(button=>button.disabled=locked);
  });
}
async function setMotorJogLock(motor,locked){
  if(locked && motorJogActive)await stopMotorJog(false);
  const res=await postJSON('/api/motor_locks',{motor,locked});
  if(res.locks)updateMotorJogLocks(res.locks);
  showActionToast(res.message||(locked?'馬達點動已鎖定。':'馬達點動鎖已解除。'),res.ok?'success':'error',1600);
}
async function applyMotorOutputScales(){
  const percentages={
    biceps:Math.min(100,Math.max(10,fval('motorScaleBiceps',80))),
    triceps:Math.min(100,Math.max(10,fval('motorScaleTriceps',100))),
    deltoid:Math.min(100,Math.max(10,fval('motorScaleDeltoid',100)))
  };
  const res=await postJSON('/api/motor_output_scales',{percentages});
  if(res.percentages){
    document.getElementById('motorScaleBiceps').value=res.percentages.biceps;
    document.getElementById('motorScaleTriceps').value=res.percentages.triceps;
    document.getElementById('motorScaleDeltoid').value=res.percentages.deltoid;
    motorScaleLoaded=true;
    saveParameterInputs();
  }
  showStatus(res.message||'全局輸出比例沒有回傳狀態。');
}

function setCableSettingsInputs(settings,percentages){
  const s=settings||{};
  const p=percentages||{};
  const values={
    cableTricepsReleasePercent:100*Number(s.triceps_release_ratio),
    cableBicepsReleasePercent:100*Number(s.biceps_release_ratio),
    cableShoulderReleasePercent:100*Number(s.antagonist_release_gain),
    cableReturnGainPercent:100*Number(s.cable_return_gain),
    cableReturnHorizon:Number(s.cable_return_horizon),
    cableReturnMaxPwm:Number(s.cable_return_max_pwm),
    cableShoulderCouplingPwm:Number(s.shoulder_elbow_coupling_pwm),
    cableShoulderRewindPercent:100*Number(s.shoulder_coupling_rewind_ratio),
    cableTricepsWindLimit:Number(s.following_triceps_wind_limit),
    cableWindSlew:Number(s.wind_slew_rate),
    cableReleaseSlew:Number(s.release_slew_rate),
    elbowSoftLandingTarget:Number(s.elbow_soft_landing_target),
    elbowSoftLandingZone:Number(s.elbow_soft_landing_zone),
    elbowSoftLandingMinPercent:100*Number(s.elbow_soft_landing_min_ratio),
    motorScaleBiceps:Number(p.biceps),
    motorScaleTriceps:Number(p.triceps),
    motorScaleDeltoid:Number(p.deltoid)
  };
  Object.entries(values).forEach(([id,value])=>{
    const element=document.getElementById(id);
    if(element&&Number.isFinite(value))element.value=Number(value.toFixed(2));
  });
  setText('cableSettingsStatus','已套用｜三頭放線 '+values.cableTricepsReleasePercent.toFixed(0)+'%｜二頭放線 '+values.cableBicepsReleasePercent.toFixed(0)+'%｜肘軟著陸 '+values.elbowSoftLandingTarget.toFixed(0)+'°／提前 '+values.elbowSoftLandingZone.toFixed(0)+'°／最低 '+values.elbowSoftLandingMinPercent.toFixed(0)+'%｜肩肘放線 '+values.cableShoulderCouplingPwm.toFixed(0)+' PWM｜全局輸出 '+values.motorScaleBiceps.toFixed(0)+'/'+values.motorScaleTriceps.toFixed(0)+'/'+values.motorScaleDeltoid.toFixed(0)+'%');
}

async function applyCableSettings(){
  const settings={
    triceps_release_ratio:fval('cableTricepsReleasePercent',50)/100,
    biceps_release_ratio:fval('cableBicepsReleasePercent',80)/100,
    antagonist_release_gain:fval('cableShoulderReleasePercent',100)/100,
    cable_return_gain:fval('cableReturnGainPercent',100)/100,
    cable_return_horizon:fval('cableReturnHorizon',1.25),
    cable_return_max_pwm:fval('cableReturnMaxPwm',60),
    shoulder_elbow_coupling_pwm:fval('cableShoulderCouplingPwm',50),
    shoulder_coupling_rewind_ratio:fval('cableShoulderRewindPercent',100)/100,
    following_triceps_wind_limit:fval('cableTricepsWindLimit',100),
    wind_slew_rate:fval('cableWindSlew',1200),
    release_slew_rate:fval('cableReleaseSlew',2000),
    elbow_soft_landing_target:fval('elbowSoftLandingTarget',90),
    elbow_soft_landing_zone:fval('elbowSoftLandingZone',25),
    elbow_soft_landing_min_ratio:fval('elbowSoftLandingMinPercent',25)/100
  };
  const percentages={
    biceps:fval('motorScaleBiceps',80),
    triceps:fval('motorScaleTriceps',100),
    deltoid:fval('motorScaleDeltoid',100)
  };
  const res=await postJSON('/api/cable_settings',{settings,percentages});
  if(res.ok&&res.settings&&res.percentages){
    setCableSettingsInputs(res.settings,res.percentages);
    cableSettingsLoaded=true;motorScaleLoaded=true;saveParameterInputs();
  }
  showStatus(res.message||'收放線設定沒有回傳狀態。');
}
async function startMotorJog(event,motor,direction){
  event.preventDefault();
  if(!document.getElementById('motorJogConfirmed').checked){
    showActionToast('請先勾選「已 Disable、無人體負載」。','error',2200);return;
  }
  if(motorJogActive||motorJogStarting)return;
  const pwm=Math.min(200,Math.max(1,Math.round(fval('motorJogPwm',100))));
  document.getElementById('motorJogPwm').value=pwm;
  const generation=++motorJogGeneration;
  motorJogHeld=true;motorJogStarting=true;motorJogButton=event.currentTarget;
  try{motorJogButton.setPointerCapture(event.pointerId);}catch(e){}
  motorJogButton.classList.add('button-busy');
  setText('motorJogStatus','正在 ARM 並啟動點動...');
  const res=await requestJSON('/api/motor_jog/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor,direction,pwm,confirmed:true})});
  // A newer press owns the UI now; an older response must not clear it.
  if(generation!==motorJogGeneration)return;
  if(!motorJogHeld){
    motorJogStarting=false;
    if(res.ok){try{await fetch('/api/motor_jog/stop',{method:'POST',cache:'no-store',keepalive:true});}catch(e){}}
    clearMotorJogUi('點動已停止｜輸出 0 PWM');
    return;
  }
  motorJogStarting=false;
  motorJogButton?.classList.remove('button-busy');
  if(!res.ok){motorJogHeld=false;clearMotorJogUi('點動未啟動｜'+(res.message||'請檢查系統狀態'));showActionToast(res.message||'點動啟動失敗','error',2500);return;}
  motorJogActive=true;motorJogButton?.classList.add('jog-active');
  const labels={biceps:'二頭肌',triceps:'三頭肌',deltoid:'三角肌'};
  setText('motorJogStatus','點動中｜'+labels[motor]+'｜'+(direction>0?'正轉＋':'反轉－')+'｜'+pwm+' PWM');
  motorJogHeartbeatTimer=setInterval(motorJogHeartbeat,150);
  if(!motorJogHeld)await stopMotorJog(false);
}
async function stopMotorJog(showToast=false){
  motorJogGeneration++;motorJogHeld=false;motorJogStarting=false;clearInterval(motorJogHeartbeatTimer);motorJogHeartbeatTimer=null;
  const wasActive=motorJogActive;motorJogActive=false;
  if(motorJogButton)motorJogButton.classList.remove('button-busy','jog-active');
  motorJogButton=null;setText('motorJogStatus','正在停止點動...');
  try{await fetch('/api/motor_jog/stop',{method:'POST',cache:'no-store',keepalive:true});}catch(e){}
  clearMotorJogUi('點動已停止｜輸出 0 PWM');
  if(showToast||wasActive)showActionToast('單顆馬達點動已 STOP。','success',1200);
}
window.addEventListener('pointerup',()=>{if(motorJogHeld||motorJogActive)stopMotorJog(false);});
window.addEventListener('pointercancel',()=>{if(motorJogHeld||motorJogActive)stopMotorJog(false);});
window.addEventListener('blur',()=>{if(motorJogHeld||motorJogActive)stopMotorJog(false);});
function stopQuickFlowOnPageExit(){
  if(!quickFlowRunning&&!quickTestActive)return;quickFlowCancelled=true;quickFlowRunning=false;quickTestActive=false;clearInterval(quickTestTimer);quickTestTimer=null;
  try{fetch('/api/set_motor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:false,confirmed:false}),cache:'no-store',keepalive:true});}catch(e){}
}
window.addEventListener('pagehide',()=>{if(motorJogHeld||motorJogActive)stopMotorJog(false);stopQuickFlowOnPageExit();});
document.addEventListener('visibilitychange',()=>{if(document.hidden){if(motorJogHeld||motorJogActive)stopMotorJog(false);stopQuickFlowOnPageExit();}});
function frequencyParams(){return {target_joint:document.getElementById('freqJointInput').value,signal_type:document.getElementById('freqSignalInput').value,amplitude:fval('freqAmplitudeInput',12),duration:fval('freqDurationInput',60),f_start:fval('freqStartInput',.1),f_end:fval('freqEndInput',3),prbs_rate:fval('freqPrbsRateInput',2),safe_min_angle:fval('freqSafeMinInput',-10),safe_max_angle:fval('freqSafeMaxInput',120)};}
async function startFrequencyTest(){
  const confirmed=window.confirm('系統識別會自動 ARM 並輸出往返 PWM。\n\n請確認：\n1. 完全沒有連接人體\n2. 馬達、驅動板與測試架已固定\n3. 繩索方向與安全角度已確認\n4. 急停可立即操作\n\n確定開始嗎？');
  if(!confirmed){showStatus('已取消系統識別，馬達維持停止。');return;}
  showStatus('正在確認 ESP32 ARM 並開始系統識別...',5000);
  const res=await postJSON('/api/frequency/start',{...frequencyParams(),confirmed_rig:true});
  showStatus(res.message||'系統識別沒有回傳訊息。',5000);
}
async function stopFrequencyTest(){const res=await postJSON('/api/frequency/stop',{});showStatus(res.message||'已停止系統識別。');}
async function analyzeFrequencyTest(){
  setText('frequencyStatus','正在以 NumPy 分析頻率響應並產生圖檔...');
  const res=await postJSONLong('/api/frequency/analyze',{});
  if(!res.ok){setText('frequencyStatus',res.message||'分析失敗。');return;}
  drawFrequencyPlot(res.analysis||{});
  showStatus(res.message||'頻域分析完成。',5000);
  await refreshStatus();
}
function drawFrequencyPlot(result){
  const canvas=document.getElementById('frequencyCanvas'),ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
  canvas.style.display='block';
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#fff';ctx.fillRect(0,0,w,h);
  const f=result.frequency_hz||[],g=result.gain_db||[],c=result.coherence||[];
  if(f.length<2){ctx.fillStyle='#64748b';ctx.fillText('完成分析後顯示 Bode magnitude 與 coherence。',20,30);return;}
  const left=65,right=w-20,top=25,mid=h/2,bottom=h-35;
  const logF=f.map(v=>Math.log10(Math.max(v,1e-4))),xmin=Math.min(...logF),xmax=Math.max(...logF);
  const gmin=Math.min(...g),gmax=Math.max(...g),gx=v=>left+(v-xmin)/(xmax-xmin||1)*(right-left);
  function line(values,yMap,color){ctx.beginPath();values.forEach((v,i)=>{const x=gx(logF[i]),y=yMap(v);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();}
  ctx.strokeStyle='#cbd5e1';ctx.strokeRect(left,top,right-left,mid-top-18);ctx.strokeRect(left,mid+15,right-left,bottom-mid-15);
  line(g,v=>top+8+(gmax-v)/(gmax-gmin||1)*(mid-top-34),'#2563eb');
  line(c,v=>bottom-8-v*(bottom-mid-31),'#16a34a');
  ctx.fillStyle='#111827';ctx.font='14px Arial';ctx.fillText('FRF magnitude (dB)',left,18);ctx.fillText('Coherence',left,mid+9);ctx.fillText(f[0].toFixed(2)+' Hz',left,bottom+22);ctx.fillText(f[f.length-1].toFixed(2)+' Hz',right-55,bottom+22);
}

function quickModeName(mode){
  return ({following_only:'純跟隨',pid:'PID 跟隨＋補償',adrc:'LADRC',ilc_pid:'ILC＋PID',ilc_adrc:'ILC＋LADRC',trajectory_demo:'開迴路展示'})[mode]||mode||'未知模式';
}
function quickJointName(joint){return joint==='shoulder'?'肩關節':'肘關節';}
function quickTargetName(mode){return ({sine:'正弦',fixed:'固定',step:'階躍'})[mode]||mode||'未知軌跡';}
function safeQuickFilename(text){return String(text||'quick_test').replace(/[^a-zA-Z0-9_\-\u4e00-\u9fff]+/g,'_').slice(0,70);}

function updateQuickFlowInputs(){
  const condition=document.getElementById('quickConditionInput').value;
  const modeName=({no_assist:'無馬達輔助（控制參數保留、輸出關閉）',following_only:'純跟隨模式',pid:'PID 跟隨＋補償',adrc:'LADRC 跟隨＋自抗擾補償'})[condition];
  const joint=document.getElementById('targetJointInput')?.value||'elbow';
  const targetMode=document.getElementById('targetModeInput')?.value||'sine';
  const trajectory=targetMode==='sine'
    ?fval('trajectoryMinInput',30)+'–'+fval('trajectoryMaxInput',90)+'°，週期 '+fval('trajectoryPeriodInput',5)+' 秒'
    :targetMode==='fixed'?'固定 '+fval('fixedTargetInput',60)+'°'
    :'階躍 '+fval('trajectoryMinInput',30)+'–'+fval('trajectoryMaxInput',90)+'°，保持 '+fval('stepHoldInput',3)+' 秒';
  const lines=[
    '本次沿用主控制面板：'+modeName,
    '關節：'+quickJointName(joint)+'｜軌跡：'+quickTargetName(targetMode)+' '+trajectory,
    '跟隨基礎 PWM='+fval('feedforwardMinInput',10)+'｜補償上限='+fval('outputLimitInput',55)+'｜Deadband='+fval('deadbandInput',4)+'°',
  ];
  if(condition==='pid')lines.push('PID：Kp='+fval('kpInput',5)+'｜Ki='+fval('kiInput',.02)+'｜Kd='+fval('kdInput',.05)+'｜積分限制='+fval('integralLimitInput',100));
  if(condition==='adrc')lines.push('LADRC：wc='+fval('adrcWcInput',2)+'｜wo='+fval('adrcWoInput',8)+'｜b0='+fval('adrcB0Input',1));
  const motor=latestMotorOutputStatus||{},cable=motor.cable_settings||{},scale=motor.motor_output_scale||{};
  if(latestMotorOutputStatus)lines.push('收放線：三頭放線 '+Number(cable.triceps_release_ratio||0).toFixed(2)+'｜二頭放線 '+Number(cable.biceps_release_ratio||0).toFixed(2)+'｜爬升 '+Number(cable.wind_slew_rate||0).toFixed(0)+' PWM/s');
  if(latestMotorOutputStatus)lines.push('全局比例：二頭 '+Math.round(100*Number(scale.biceps||0))+'%｜三頭 '+Math.round(100*Number(scale.triceps||0))+'%｜三角 '+Math.round(100*Number(scale.deltoid||0))+'%');
  setText('quickParameterSummary',lines.join('\n'));
}

function applyQuickFlowInputsToMainControls(){
  const condition=document.getElementById('quickConditionInput').value;
  if(condition!=='no_assist')document.getElementById('controllerModeInput').value=condition;
  updateControllerMode();saveParameterInputs();updateQuickFlowInputs();
}

async function stopMotorAfterQuickRun(message='測試結束，正在自動停止馬達...'){
  setText('quickTestStatus',document.getElementById('quickTestStatus').innerText+'\n'+message);
  const stopped=await requestJSON('/api/set_motor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:false,confirmed:false}),timeoutMs:10000});
  if(stopped.ok===false)showStatus(stopped.message||'自動停止確認失敗，請按 Emergency Stop。',5000);
  else showStatus('快速測試完成，馬達已自動停止。',5000);
}

async function runQuickFullFlow(){
  if(quickFlowRunning||quickTestActive){showStatus('快速測試流程已在執行。');return;}
  const actionButton=claimActionButton(),stopButton=document.getElementById('quickStopButton');
  quickFlowRunning=true;quickFlowCancelled=false;
  if(actionButton){actionButton.dataset.actionLabel=actionButton.textContent;actionButton.disabled=true;}
  stopButton.disabled=false;
  try{
    const initial=await requestJSON('/api/status');
    if(!initial.initialized){showStatus('請先完成 IMU 初始化與校正。');return;}
    const fault=((initial.imu_health||{}).safety_fault||{});
    if(fault.latched){showStatus('IMU 安全鎖尚未解除，不能開始快速測試。');return;}
    const condition=document.getElementById('quickConditionInput').value;
    const powered=condition!=='no_assist';
    const conditionName=({no_assist:'無馬達輔助',following_only:'純跟隨模式',pid:'PID 跟隨＋補償',adrc:'LADRC 跟隨＋自抗擾補償'})[condition];
    updateQuickFlowInputs();
    if(powered){
      const summary=document.getElementById('quickParameterSummary').innerText;
      const confirmed=window.confirm('快速測試將沿用主控制面板參數，切換為 '+conditionName+' 並啟動實際馬達。\n\n'+summary+'\n\n請確認：\n1. 上述實際參數正確\n2. 穿戴與繩索位置正確\n3. 急停可立即操作\n4. 倒數 3 秒期間馬達保持停止，歸零後才 ARM 並開始記錄\n5. 測試結束會自動停止馬達\n\n確定開始嗎？');
      if(!confirmed){showStatus('已取消快速測試，馬達狀態未改變。');return;}
    }
    setText('quickTestStatus','步驟 1/4：停止既有馬達輸出...');
    const disabled=await requestJSON('/api/set_motor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:false,confirmed:false}),timeoutMs:10000});
    if(disabled.ok===false){showStatus(disabled.message||'無法確認馬達停止。');return;}
    if(quickFlowCancelled)return;

    applyQuickFlowInputsToMainControls();
    setText('quickTestStatus','步驟 2/4：正在沿用目前任務、軌跡與控制器參數...');
    const applied=await requestJSON('/api/apply_params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(params()),timeoutMs:10000});
    if(applied.ok===false){showStatus(applied.message||'控制參數套用失敗。');return;}
    if(quickFlowCancelled)return;

    for(let count=3;count>=1;count--){
      setText('quickTestStatus','步驟 3/4：馬達保持停止，'+count+' 秒後開始 '+conditionName+'。請先準備姿勢。');
      await sleep(1000);if(quickFlowCancelled)return;
    }

    if(powered){
      setText('quickTestStatus','步驟 4/4：倒數完成，正在 ARM；確認成功後立即開始記錄...');
      const armed=await requestJSON('/api/set_motor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:true,confirmed:true}),timeoutMs:10000});
      if(armed.ok===false){showStatus(armed.message||'ESP32 ARM 失敗。');return;}
    }else{
      setText('quickTestStatus','步驟 4/4：倒數完成，無輔助組開始記錄，馬達保持停止。');
    }
    if(quickFlowCancelled)return;

    await startQuickPresentationTest();
    if(!quickTestActive)await stopMotorAfterQuickRun('流程未能開始記錄，已再次停止馬達。');
  }finally{
    quickFlowRunning=false;
    if(!quickTestActive){
      stopButton.disabled=true;
      if(actionButton){actionButton.disabled=false;actionButton.textContent=actionButton.dataset.actionLabel||'開始完整快速測試';}
    }
  }
}

async function cancelQuickFullFlowOrTest(){
  claimActionButton();quickFlowCancelled=true;
  if(quickTestActive){finishQuickPresentationTest(true);return;}
  setText('quickTestStatus','正在取消流程並停止馬達...');
  await requestJSON('/api/set_motor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:false,confirmed:false}),timeoutMs:10000});
  quickFlowRunning=false;document.getElementById('quickStopButton').disabled=true;document.getElementById('quickStartButton').disabled=false;
  setText('quickTestStatus','流程已取消，馬達已要求停止。');showStatus('快速測試流程已取消。');
}

async function startQuickPresentationTest(){
  if(quickTestActive){showStatus('快速測試已在執行。');return;}
  const status=await requestJSON('/api/status',{actionButton:claimActionButton()});
  if(!status.initialized){showStatus('請先完成 IMU 初始化與校正。');return;}
  const fault=((status.imu_health||{}).safety_fault||{});
  if(fault.latched){showStatus('IMU 安全鎖尚未解除，不能開始簡報測試。');return;}
  if(!status.frame){showStatus('尚未收到有效 IMU 角度，請稍後再試。');return;}

  const selected=document.getElementById('quickConditionInput').value;
  const actualMode=String(status.frame.controller_mode||document.getElementById('controllerModeInput').value);
  const motorEnabled=Boolean(status.motor_enabled);
  const joint=String(status.frame.target_joint||'elbow');
  const targetMode=String(status.frame.target_mode||document.getElementById('targetModeInput').value);
  const comparisonSignature=JSON.stringify({joint,targetMode,min:fval('trajectoryMinInput',30),max:fval('trajectoryMaxInput',90),period:fval('trajectoryPeriodInput',5),fixed:fval('fixedTargetInput',60),step:fval('stepHoldInput',3)});
  if(quickTestRuns.length&&quickTestRuns[0].comparison_signature!==comparisonSignature){showStatus('比較圖必須使用相同關節與軌跡參數；請恢復原設定或先清除快速測試結果。');return;}
  if(selected==='no_assist'&&motorEnabled){showStatus('「無馬達輔助」必須先 Disable Motor。');return;}
  if(selected==='following_only'&&(!motorEnabled||actualMode!=='following_only')){showStatus('請先選純跟隨模式、套用參數並 Enable Motor。');return;}
  if(selected==='pid'&&(!motorEnabled||actualMode!=='pid')){showStatus('請先選 PID、套用參數並 Enable Motor。');return;}
  if(selected==='adrc'&&(!motorEnabled||actualMode!=='adrc')){showStatus('請先選 LADRC、套用參數並 Enable Motor。');return;}

  quickTestDuration=Math.min(120,Math.max(10,fval('quickDurationInput',20)));
  quickTestTolerance=Math.min(20,Math.max(1,fval('quickToleranceInput',5)));
  let condition=selected==='auto'?(motorEnabled?quickModeName(actualMode):'無馬達輔助'):({no_assist:'無馬達輔助',following_only:'純跟隨模式',pid:'PID 跟隨＋補償',adrc:'LADRC 跟隨＋自抗擾補償'})[selected];
  quickTestSamples=[];
  quickTestStartedAt=performance.now();
  quickTestActive=true;
  quickTestCurrent={
    condition, joint, controller_mode:actualMode,
    target_mode:targetMode, comparison_signature:comparisonSignature,
    motor_enabled:motorEnabled, duration_requested:quickTestDuration,
    tolerance:quickTestTolerance, started_at:new Date().toISOString(),
    participant:(document.getElementById('participantInput').value||'S01').trim()
  };
  document.getElementById('quickStartButton').disabled=true;
  document.getElementById('quickStopButton').disabled=false;
  document.getElementById('quickTestProgress').style.width='0%';
  setText('quickTestStatus','記錄中：'+condition+'｜'+quickJointName(quickTestCurrent.joint)+'｜0.0 / '+quickTestDuration.toFixed(0)+' 秒');
  clearInterval(quickTestTimer);
  quickTestTimer=setInterval(()=>{
    const elapsed=(performance.now()-quickTestStartedAt)/1000;
    document.getElementById('quickTestProgress').style.width=Math.min(100,100*elapsed/quickTestDuration).toFixed(1)+'%';
    setText('quickTestStatus','記錄中：'+condition+'｜'+quickJointName(quickTestCurrent.joint)+'｜'+Math.min(elapsed,quickTestDuration).toFixed(1)+' / '+quickTestDuration.toFixed(0)+' 秒｜樣本 '+quickTestSamples.length);
    if(elapsed>=quickTestDuration)finishQuickPresentationTest(false);
  },100);
  showStatus('快速測試開始；請依照目標軌跡自然動作。',4000);
}

function captureQuickTestSample(frame,measured){
  if(!quickTestActive||!quickTestCurrent)return;
  const elapsed=(performance.now()-quickTestStartedAt)/1000;
  if(elapsed<0||elapsed>quickTestDuration+0.5)return;
  const systemTime=Number(frame.time),target=Number(frame.reference_target_angle??frame.target_angle),actual=Number(measured);
  if(!Number.isFinite(systemTime)||!Number.isFinite(target)||!Number.isFinite(actual))return;
  const last=quickTestSamples[quickTestSamples.length-1];
  if(last&&last.system_time===systemTime)return;
  // The analysis endpoint contains only JSON scalar values. Keep that complete
  // controller snapshot, then add the quick-test time/reference aliases used
  // by plotting and metric calculations.
  quickTestSamples.push({...frame,time:elapsed,system_time:systemTime,wall_time:new Date().toISOString(),target,measured:actual,error:target-actual});
}

function calculateQuickMetrics(samples,tolerance){
  const errors=samples.map(s=>s.error),n=errors.length;
  const rmse=Math.sqrt(errors.reduce((sum,e)=>sum+e*e,0)/Math.max(n,1));
  const mae=errors.reduce((sum,e)=>sum+Math.abs(e),0)/Math.max(n,1);
  const maxError=n?Math.max(...errors.map(Math.abs)):0;
  const within=n?100*errors.filter(e=>Math.abs(e)<=tolerance).length/n:0;
  const span=n>1?samples[n-1].time-samples[0].time:0;
  return {rmse,mae,max_error:maxError,within_tolerance_percent:within,sample_rate_hz:span>0?(n-1)/span:0,sample_count:n,duration:span};
}

function finishQuickPresentationTest(early=false){
  claimActionButton();
  if(!quickTestActive)return;
  const motorWasEnabled=Boolean(quickTestCurrent?.motor_enabled);
  quickTestActive=false;clearInterval(quickTestTimer);quickTestTimer=null;
  document.getElementById('quickStartButton').disabled=false;
  document.getElementById('quickStopButton').disabled=true;
  document.getElementById('quickTestProgress').style.width='100%';
  if(quickTestSamples.length<20){
    setText('quickTestStatus','有效樣本不足，未建立圖表；請確認 IMU 連線後重測。');
    showStatus('快速測試樣本不足。');if(motorWasEnabled)void stopMotorAfterQuickRun();return;
  }
  const run={...quickTestCurrent,samples:quickTestSamples.slice()};
  run.metrics=calculateQuickMetrics(run.samples,run.tolerance);
  run.run_number=quickTestRuns.length+1;
  run.include_in_comparison=true;
  quickTestRuns.push(run);quickTestCurrent=run;
  drawQuickTestResult(run);drawQuickSignalResult(run);renderQuickRunSelector();drawQuickComparison();
  document.getElementById('quickDownloadPngButton').disabled=false;
  document.getElementById('quickDownloadSignalsButton').disabled=false;
  document.getElementById('quickDownloadCsvButton').disabled=false;
  const m=run.metrics;
  setText('quickTestStatus',(early?'提前完成':'測試完成')+'｜'+run.condition+'｜樣本 '+m.sample_count+'｜'+m.sample_rate_hz.toFixed(1)+' Hz\nRMSE '+m.rmse.toFixed(2)+'°｜MAE '+m.mae.toFixed(2)+'°｜最大誤差 '+m.max_error.toFixed(2)+'°｜±'+run.tolerance.toFixed(1)+'° 跟隨率 '+m.within_tolerance_percent.toFixed(1)+'%');
  showStatus('快速測試完成，PNG 與 CSV 已可下載。',5000);
  if(run.motor_enabled)void stopMotorAfterQuickRun();
}

function abortQuickPresentationTest(reason){
  if(!quickTestActive)return;const motorWasEnabled=Boolean(quickTestCurrent?.motor_enabled);quickTestActive=false;clearInterval(quickTestTimer);quickTestTimer=null;quickTestSamples=[];quickTestCurrent=null;
  document.getElementById('quickStartButton').disabled=false;document.getElementById('quickStopButton').disabled=true;document.getElementById('quickTestProgress').style.width='0%';
  setText('quickTestStatus','測試已作廢：'+reason+'。本次資料未加入比較圖。');showStatus('快速測試已作廢：'+reason,5000);
  if(motorWasEnabled)void stopMotorAfterQuickRun();
}

function drawQuickTestResult(run){
  const c=document.getElementById('quickTestCanvas'),ctx=c.getContext('2d'),w=c.width,h=c.height;
  c.classList.add('visible');ctx.clearRect(0,0,w,h);ctx.fillStyle='#fff';ctx.fillRect(0,0,w,h);
  const s=run.samples,tMax=Math.max(s[s.length-1].time,1),all=s.flatMap(v=>[v.target,v.measured]);
  let yMin=Math.floor((Math.min(...all)-8)/10)*10,yMax=Math.ceil((Math.max(...all)+8)/10)*10;if(yMax-yMin<30)yMax=yMin+30;
  const ml=72,mr=32,pw=w-ml-mr,top=105,angleH=330,gap=62,errTop=top+angleH+gap,errH=120;
  const errLimit=Math.max(run.tolerance,Math.ceil(Math.max(...s.map(v=>Math.abs(v.error)),1)/5)*5);
  const x=t=>ml+t/tMax*pw,ya=v=>top+angleH-(v-yMin)/(yMax-yMin)*angleH,ye=v=>errTop+errH-(v+errLimit)/(2*errLimit)*errH;
  ctx.fillStyle='#111827';ctx.font='bold 25px Arial,"Microsoft JhengHei"';ctx.fillText('目標軌跡追蹤快速測試',ml,36);
  ctx.font='16px Arial,"Microsoft JhengHei"';ctx.fillStyle='#475569';ctx.fillText(run.condition+'｜'+quickJointName(run.joint)+'｜'+quickTargetName(run.target_mode)+'軌跡｜'+(run.motor_enabled?'馬達啟用':'馬達停用')+'｜'+new Date(run.started_at).toLocaleString(),ml,66);
  function axes(y0,hh,min,max,label){ctx.strokeStyle='#cbd5e1';ctx.lineWidth=1;ctx.strokeRect(ml,y0,pw,hh);ctx.fillStyle='#334155';ctx.font='14px Arial';ctx.fillText(label,ml,y0-9);for(let i=0;i<=4;i++){const v=min+(max-min)*i/4,y=y0+hh-i*hh/4;ctx.strokeStyle='#eef2f7';ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(ml+pw,y);ctx.stroke();ctx.fillStyle='#64748b';ctx.fillText(v.toFixed(0),ml-43,y+4);}for(let i=0;i<=4;i++){const t=tMax*i/4,xx=x(t);ctx.fillText(t.toFixed(1),xx-10,y0+hh+20);}}
  axes(top,angleH,yMin,yMax,'Angle (deg)');axes(errTop,errH,-errLimit,errLimit,'Error (deg)');
  function line(key,map,color,width,dash=[]){ctx.beginPath();s.forEach((v,i)=>{const xx=x(v.time),yy=map(v[key]);if(i===0)ctx.moveTo(xx,yy);else ctx.lineTo(xx,yy);});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);}
  line('target',ya,'#64748b',3,[9,6]);line('measured',ya,'#2563eb',3);line('error',ye,'#dc2626',2);
  ctx.strokeStyle='#16a34a';ctx.lineWidth=1;ctx.setLineDash([5,5]);[run.tolerance,-run.tolerance].forEach(v=>{ctx.beginPath();ctx.moveTo(ml,ye(v));ctx.lineTo(ml+pw,ye(v));ctx.stroke();});ctx.setLineDash([]);
  ctx.font='bold 15px Arial,"Microsoft JhengHei"';ctx.fillStyle='#64748b';ctx.fillText('--- Target',w-305,35);ctx.fillStyle='#2563eb';ctx.fillText('— Measured',w-195,35);ctx.fillStyle='#dc2626';ctx.fillText('— Error',w-80,35);
  const m=run.metrics;ctx.fillStyle='#0f172a';ctx.font='bold 17px Arial,"Microsoft JhengHei"';ctx.fillText('RMSE '+m.rmse.toFixed(2)+'°    MAE '+m.mae.toFixed(2)+'°    最大誤差 '+m.max_error.toFixed(2)+'°    ±'+run.tolerance.toFixed(1)+'° 跟隨率 '+m.within_tolerance_percent.toFixed(1)+'%    取樣率 '+m.sample_rate_hz.toFixed(1)+' Hz',ml,h-24);
}

function drawQuickSignalResult(run){
  const c=document.getElementById('quickSignalCanvas'),ctx=c.getContext('2d'),w=c.width,h=c.height,s=run.samples;
  if(!s.length)return;
  c.classList.add('visible');ctx.clearRect(0,0,w,h);ctx.fillStyle='#fff';ctx.fillRect(0,0,w,h);
  const ml=75,mr=35,pw=w-ml-mr,tMax=Math.max(s[s.length-1].time,1),x=v=>ml+v/tMax*pw;
  const errTop=82,errH=215,pwmTop=390,pwmH=215;
  const errPeak=Math.max(run.tolerance*1.4,5,...s.map(v=>Math.abs(Number(v.error)||0))),errLimit=Math.ceil(errPeak/5)*5;
  const pwmPeak=Math.max(10,...s.flatMap(v=>['pwm_biceps','pwm_triceps','pwm_deltoid'].map(k=>Math.abs(Number(v[k])||0)))),pwmLimit=Math.ceil(pwmPeak/10)*10;
  const yErr=v=>errTop+errH/2-(v/errLimit)*(errH/2),yPwm=v=>pwmTop+pwmH/2-(v/pwmLimit)*(pwmH/2);
  const frame=(top,height,label)=>{ctx.strokeStyle='#cbd5e1';ctx.strokeRect(ml,top,pw,height);ctx.strokeStyle='#e2e8f0';ctx.beginPath();ctx.moveTo(ml,top+height/2);ctx.lineTo(ml+pw,top+height/2);ctx.stroke();ctx.fillStyle='#475569';ctx.font='14px Arial,"Microsoft JhengHei"';ctx.fillText(label,ml,top-12);};
  const line=(key,y,color,width=2)=>{ctx.beginPath();s.forEach((v,i)=>{const px=x(v.time),py=y(Number(v[key])||0);if(i===0)ctx.moveTo(px,py);else ctx.lineTo(px,py);});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();};
  ctx.fillStyle='#111827';ctx.font='bold 25px Arial,"Microsoft JhengHei"';ctx.fillText(run.condition+'｜追蹤誤差與三馬達 PWM',ml,36);
  ctx.fillStyle='#475569';ctx.font='14px Arial';ctx.fillText(quickJointName(run.joint)+' / '+quickTargetName(run.target_mode)+' / '+run.started_at,ml,59);
  frame(errTop,errH,'Tracking error (deg)');
  ctx.setLineDash([7,5]);ctx.strokeStyle='#16a34a';[run.tolerance,-run.tolerance].forEach(v=>{ctx.beginPath();ctx.moveTo(ml,yErr(v));ctx.lineTo(ml+pw,yErr(v));ctx.stroke();});ctx.setLineDash([]);line('error',yErr,'#dc2626',2.4);
  ctx.fillStyle='#dc2626';ctx.fillText('Error',w-235,errTop-12);ctx.fillStyle='#16a34a';ctx.fillText('±'+run.tolerance.toFixed(1)+'°',w-150,errTop-12);
  frame(pwmTop,pwmH,'Signed motor PWM');line('pwm_biceps',yPwm,'#2563eb',2.2);line('pwm_triceps',yPwm,'#f97316',2.2);line('pwm_deltoid',yPwm,'#7c3aed',2.2);
  ctx.fillStyle='#2563eb';ctx.fillText('Biceps',w-335,pwmTop-12);ctx.fillStyle='#f97316';ctx.fillText('Triceps',w-235,pwmTop-12);ctx.fillStyle='#7c3aed';ctx.fillText('Deltoid',w-130,pwmTop-12);
  ctx.fillStyle='#64748b';ctx.fillText('0 s',ml,pwmTop+pwmH+25);ctx.fillText(tMax.toFixed(1)+' s',ml+pw-40,pwmTop+pwmH+25);
  ctx.fillStyle='#0f172a';ctx.font='bold 15px Arial,"Microsoft JhengHei"';ctx.fillText('上圖可檢查誤差與 ±容許範圍；下圖可檢查拮抗收放線、爆衝與 PWM 抖動。',ml,h-25);
}

function quickComparisonRuns(){return quickTestRuns.filter(run=>run.include_in_comparison!==false);}

function renderQuickRunSelector(){
  const host=document.getElementById('quickRunSelector');
  host.replaceChildren();
  if(!quickTestRuns.length){
    const empty=document.createElement('span');empty.className='small';empty.textContent='完成測試後，可勾選要放進比較圖的紀錄。';host.appendChild(empty);
    document.getElementById('quickDownloadCompareButton').disabled=true;
    return;
  }
  quickTestRuns.forEach(run=>{
    const row=document.createElement('div');row.className='quick-run-item';
    const box=document.createElement('input');box.type='checkbox';box.id='quickRunCompare'+run.run_number;box.checked=run.include_in_comparison!==false;
    const label=document.createElement('label');label.htmlFor=box.id;label.textContent='第 '+run.run_number+' 次｜'+run.condition+'｜RMSE '+run.metrics.rmse.toFixed(2)+'°｜跟隨率 '+run.metrics.within_tolerance_percent.toFixed(1)+'%';
    box.addEventListener('change',()=>{run.include_in_comparison=box.checked;drawQuickComparison();});
    row.append(box,label);host.appendChild(row);
  });
  document.getElementById('quickDownloadCompareButton').disabled=quickComparisonRuns().length===0;
}

function drawQuickComparison(){
  const runs=quickComparisonRuns(),c=document.getElementById('quickCompareCanvas'),ctx=c.getContext('2d'),w=c.width,h=c.height;
  document.getElementById('quickDownloadCompareButton').disabled=runs.length===0;
  ctx.clearRect(0,0,w,h);
  if(!runs.length){c.classList.remove('visible');return;}
  c.classList.add('visible');ctx.fillStyle='#fff';ctx.fillRect(0,0,w,h);
  const colors=['#64748b','#16a34a','#2563eb','#7c3aed','#f59e0b','#0891b2','#db2777','#65a30d'],left=75,top=90,bottom=h-95,mid=720;
  ctx.fillStyle='#111827';ctx.font='bold 25px Arial,"Microsoft JhengHei"';ctx.fillText('控制條件追蹤表現比較',left,38);
  const maxDeg=Math.max(5,...runs.map(r=>r.metrics.max_error))*1.15,groups=['RMSE','MAE','最大誤差'],groupW=(mid-left-35)/groups.length,barW=Math.min(42,groupW/(runs.length+1));
  ctx.strokeStyle='#cbd5e1';ctx.strokeRect(left,top,mid-left,bottom-top);ctx.fillStyle='#334155';ctx.font='14px Arial';ctx.fillText('Error (deg)',left,top-10);
  groups.forEach((label,g)=>{const center=left+groupW*(g+.5);ctx.fillStyle='#334155';ctx.fillText(label,center-28,bottom+24);runs.forEach((r,i)=>{const value=[r.metrics.rmse,r.metrics.mae,r.metrics.max_error][g],bh=value/maxDeg*(bottom-top),bx=center+(i-(runs.length-1)/2)*barW-barW*.38;ctx.fillStyle=colors[(r.run_number-1)%colors.length];ctx.fillRect(bx,bottom-bh,barW*.76,bh);ctx.font='11px Arial';ctx.fillText(value.toFixed(1),bx,bottom-bh-5);});});
  const rLeft=790,rRight=w-35;ctx.strokeStyle='#cbd5e1';ctx.strokeRect(rLeft,top,rRight-rLeft,bottom-top);ctx.fillStyle='#334155';ctx.font='14px Arial';ctx.fillText('±容許角度跟隨率 (%)',rLeft,top-10);
  runs.forEach((r,i)=>{const slot=(rRight-rLeft)/(runs.length+1),rateBarW=Math.min(48,slot*.62),bx=rLeft+slot*(i+1)-rateBarW/2,bh=r.metrics.within_tolerance_percent/100*(bottom-top);ctx.fillStyle=colors[(r.run_number-1)%colors.length];ctx.fillRect(bx,bottom-bh,rateBarW,bh);ctx.fillStyle='#334155';ctx.font='11px Arial';ctx.fillText(r.metrics.within_tolerance_percent.toFixed(1)+'%',bx,bottom-bh-7);});
  runs.forEach((r,i)=>{const column=i%3,row=Math.floor(i/3),lx=left+column*330,ly=h-52+row*21;ctx.fillStyle=colors[(r.run_number-1)%colors.length];ctx.fillRect(lx,ly-13,16,16);ctx.fillStyle='#334155';ctx.font='13px Arial,"Microsoft JhengHei"';ctx.fillText('第 '+r.run_number+' 次 '+r.condition,lx+22,ly);});
}

function downloadCanvas(canvasId,filename){const c=document.getElementById(canvasId);if(!c.classList.contains('visible'))return;const a=document.createElement('a');a.href=c.toDataURL('image/png');a.download=filename;a.click();}
function downloadQuickTestPng(){if(!quickTestCurrent)return;downloadCanvas('quickTestCanvas',safeQuickFilename(quickTestCurrent.participant+'_'+quickTestCurrent.condition+'_tracking')+'.png');}
function downloadQuickSignalsPng(){if(!quickTestCurrent)return;downloadCanvas('quickSignalCanvas',safeQuickFilename(quickTestCurrent.participant+'_'+quickTestCurrent.condition+'_error_pwm')+'.png');}
function downloadQuickComparePng(){if(!quickComparisonRuns().length)return;downloadCanvas('quickCompareCanvas',safeQuickFilename((quickTestCurrent?.participant||'S01')+'_control_comparison_selected')+'.png');}
function downloadQuickTestCsv(){
  if(!quickTestCurrent)return;const r=quickTestCurrent,m=r.metrics;
  const fields=['time','system_time','wall_time','target','measured','error','target_angle','reference_target_angle','target_velocity','reference_target_velocity','measured_angle_raw','elbow_angle','elbow_angle_raw','shoulder_angle','shoulder_angle_raw','upper_angle','forearm_angle','omega','alpha','jerk','error_for_control','integral_error','derivative_error','filtered_measurement_velocity','control_active','anti_windup_active','raw_pid_output','feedback_pid_output','feedforward_output','following_output','elbow_following_output','shoulder_following_output','controller_compensation_output','pid_output','motor_cmd','adrc_output','adrc_raw_output','adrc_estimated_angle','adrc_estimated_velocity','adrc_estimated_disturbance','adrc_observer_error','adrc_saturated','ilc_output','ilc_current_cycle','ilc_last_rmse','ilc_improvement_percent','ilc_learned_peak_pwm','ilc_frozen','desired_motor_cmd','desired_pwm_biceps','desired_pwm_triceps','desired_pwm_deltoid','pwm_biceps','pwm_triceps','pwm_deltoid','virtual_cable_effort_biceps','virtual_cable_effort_triceps','virtual_cable_effort_deltoid','elbow_soft_landing_scale','shoulder_motion_state','shoulder_release_command','shoulder_release_pwm','shoulder_motor_enable','motor_enabled','emergency_stop','motion_state','target_joint','target_mode','controller_mode'];
  const headings=fields.map(v=>({time:'test_time_s',system_time:'system_time_s',target:'reference_target_deg',measured:'measured_deg',error:'tracking_error_deg'})[v]||v);
  const rows=[['participant','condition','joint','controller_mode','target_mode','motor_enabled','tolerance_deg','rmse_deg','mae_deg','max_error_deg','within_tolerance_percent','sample_rate_hz'],[r.participant,r.condition,r.joint,r.controller_mode,r.target_mode,r.motor_enabled,r.tolerance,m.rmse,m.mae,m.max_error,m.within_tolerance_percent,m.sample_rate_hz],[],headings,...r.samples.map(s=>fields.map(key=>s[key]))];
  const csv=rows.map(row=>row.map(v=>'"'+String(v??'').replace(/"/g,'""')+'"').join(',')).join('\r\n');const url=URL.createObjectURL(new Blob(['\ufeff'+csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download=safeQuickFilename(r.participant+'_'+r.condition+'_full_analysis')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function clearQuickPresentationRuns(){
  claimActionButton();if(quickTestActive){showStatus('請先結束目前快速測試。');return;}quickTestRuns=[];quickTestCurrent=null;quickTestSamples=[];
  ['quickTestCanvas','quickSignalCanvas','quickCompareCanvas'].forEach(id=>{const c=document.getElementById(id);c.classList.remove('visible');c.getContext('2d').clearRect(0,0,c.width,c.height);});
  renderQuickRunSelector();['quickDownloadPngButton','quickDownloadSignalsButton','quickDownloadCsvButton','quickDownloadCompareButton'].forEach(id=>document.getElementById(id).disabled=true);document.getElementById('quickTestProgress').style.width='0%';setText('quickTestStatus','結果已清除；本功能只記錄，不會自動啟動馬達。');
}
async function startRecording(){
  resetPlot();
  setText('recordingState','正在建立 CSV...');
  const data={participant_id:document.getElementById('participantInput').value||'S01',session_name:document.getElementById('sessionInput').value||'session',output_dir:document.getElementById('outputDirInput').value||'control_data',label:document.getElementById('labelInput').value||'trial'};
  const res=await postJSON('/api/start_recording',data);
  if(!res.ok){setText('recordingState','未錄製');showStatus(res.message);return;}
  showStatus(res.message||'已開始錄製。');
  setText('recordingState','錄製中');document.getElementById('recordingState').className='rec';setText('csvPathText',res.csv_path);
}
async function stopRecording(){setText('recordingState','正在停止錄製...');const res=await postJSON('/api/stop_recording',{});setText('recordingState','未錄製');document.getElementById('recordingState').className='idle';setText('csvPathText',res.message||'停止錄製沒有回傳訊息。');showStatus(res.message||'停止錄製沒有回傳訊息。');}

function updateImuChannelStatus(health){
  const channels=health.channels||{};
  const upper=channels.upper_arm||{channel:0,label:'上臂 IMU',checked:false,connected:false,last_error:''};
  const forearm=channels.forearm||{channel:1,label:'前臂 IMU',checked:false,connected:false,last_error:''};
  const describe=channel=>{
    if(channel.connected)return '已連線';
    if(!channel.checked)return '尚未檢查';
    return '斷線／找不到'+(channel.last_error?' — '+channel.last_error:'');
  };
  const element=document.getElementById('imuChannelStatus');
  if(!element)return;
  element.textContent=(upper.label||'上臂 IMU')+' (Channel 0)：'+describe(upper)+'\n'
    +(forearm.label||'前臂 IMU')+' (Channel 1)：'+describe(forearm)
    +(health.reconnecting?'\n正在自動重連；馬達已停止。':'');
  const hasFailure=[upper,forearm].some(channel=>channel.checked&&!channel.connected);
  const allConnected=upper.connected&&forearm.connected;
  element.className='validation '+(hasFailure?'fail':allConnected?'pass':'');

  const fault=health.safety_fault||{};
  const faultElement=document.getElementById('imuSafetyLockStatus');
  const clearButton=document.getElementById('clearImuSafetyButton');
  if(faultElement){
    if(fault.latched){
      faultElement.textContent='IMU 安全鎖：已鎖定\n原因：'+(fault.reason||'IMU 資料異常')+'\n'
        +(fault.recovery_ready?'IMU 已穩定恢復，可檢查姿勢與繩索後解除。':'等待 IMU 穩定資料：'+(fault.stable_samples||0)+' / '+(fault.required_stable_samples||25));
      faultElement.className='validation fail';
    }else{
      faultElement.textContent='IMU 安全鎖：正常';
      faultElement.className='validation pass';
    }
  }
  if(clearButton)clearButton.style.display=fault.latched?'block':'none';
}

function updatePage(data){
  const now=performance.now();
  const updateSlowUi=(now-lastSlowUiUpdate)>=slowUiIntervalMs;
  if(updateSlowUi){
    lastSlowUiUpdate=now;
    setLiveStatus(data.status);
    updateValidationPanel(data.rom_calibration||{});
    updateFrequencyPanel(data.frequency_identification||{});
    updateILCPanel(data.ilc||{});
    updateMotorJogLocks(data.motor_locks||(data.motor_jog||{}).locks||{});
    updateImuChannelStatus(data.imu_health||{});
    const quickFault=((data.imu_health||{}).safety_fault||{});
    if(quickTestActive&&quickFault.latched)abortQuickPresentationTest('IMU 安全鎖啟動');
    else if(quickTestActive&&quickTestCurrent?.motor_enabled&&!data.motor_enabled)abortQuickPresentationTest('馬達在測試途中停止');
    else if(quickTestActive&&data.frame&&(data.frame.target_joint!==quickTestCurrent?.joint||data.frame.controller_mode!==quickTestCurrent?.controller_mode))abortQuickPresentationTest('測試途中更改關節或控制器');
  }
  if(updateSlowUi && data.motor_output){
    const s=data.motor_output;
    latestMotorOutputStatus=s;
    const outputScale=s.motor_output_scale||{biceps:.8,triceps:1,deltoid:1};
    const scalePercentages={
      biceps:Math.round(100*Number(outputScale.biceps||0)),
      triceps:Math.round(100*Number(outputScale.triceps||0)),
      deltoid:Math.round(100*Number(outputScale.deltoid||0))
    };
    if(!cableSettingsLoaded&&s.cable_settings){
      setCableSettingsInputs(s.cable_settings,scalePercentages);
      cableSettingsLoaded=true;motorScaleLoaded=true;
      saveParameterInputs();
    }
    updateQuickFlowInputs();
    const demo=data.frame && data.frame.controller_mode==='trajectory_demo';
    const followingOnly=data.frame && data.frame.controller_mode==='following_only';
    setText('dryRunBanner',(demo
      ?'開迴路展示僅限空載或固定測試架，禁止連接人體。'
      :followingOnly
      ?'跟隨模式：目前跟隨'+((data.frame||{}).target_joint==='shoulder'?'肩關節。':'肘關節。')
      :'復健控制：跟隨實際動作，大幅落後時增加控制器補償。'));
    const channelLocks=data.motor_locks||{};
    const lockedNames=[
      channelLocks.biceps?'二頭肌':'',
      channelLocks.triceps?'三頭肌':'',
      channelLocks.deltoid?'三角肌':''
    ].filter(Boolean);
    const lockSummary=lockedNames.length?'｜已鎖定：'+lockedNames.join('、'):'';
    setText('motorInterlockText',(s.patient_rom_active
      ?'ROM 邊界保護已啟用'
      :'尚未套用個人化 ROM')+lockSummary);
  }
  if(updateSlowUi && data.recording){setText('recordingState','錄製中，筆數：'+data.record_count);document.getElementById('recordingState').className='rec';setText('csvPathText',data.csv_path||'');}
  updateLiveFrame(data);
}

function updateLiveFrame(data){
  if(data.frame){
    const f=data.frame;
    setText('validationLiveAngles','即時角度｜肘='+f.elbow_angle.toFixed(1)+'°'
      +'｜roll候選='+f.elbow_roll_signed.toFixed(1)+'°｜pitch候選='+f.elbow_pitch_signed.toFixed(1)+'°'
      +'｜肩='+f.shoulder_angle.toFixed(1)+'°｜肩速度='+(f.shoulder_angle_velocity ?? 0).toFixed(1)+'°/s');
    const measured=(f.measured_angle!==undefined)?f.measured_angle:(f.target_joint==='shoulder'?f.shoulder_angle:f.elbow_angle);
    const followingOnly=f.controller_mode==='following_only';
    const showQuickTestCue=quickTestActive;
    setText('measuredAngle',measured.toFixed(1)+'°');
    setText('elbowAngle',f.elbow_angle.toFixed(1)+'°');
    setText('shoulderAngle',f.shoulder_angle.toFixed(1)+'°');
    const evaluationTarget=Number(f.reference_target_angle??f.target_angle);
    const evaluationError=evaluationTarget-measured;
    setText('targetAngle',followingOnly&&!showQuickTestCue?'忽略':evaluationTarget.toFixed(1)+'°'+(followingOnly?'（僅提示）':''));
    setText('errorValue',followingOnly&&!showQuickTestCue?'無補償':evaluationError.toFixed(1)+'°'+(followingOnly?'（不補償）':''));
    const pulseStateLabels={idle:'準備取樣',blanking:'馬達歸零隔離',sensing:'馬達關閉取樣',drive:'固定輸出保持',rearm:'等待靜止'};
    let motionText=f.rom_limit_active ? f.motion_state+'｜ROM 限制' : f.motion_state;
    if(followingOnly){const shoulder=f.target_joint==='shoulder';const state=shoulder?f.shoulder_following_state:f.elbow_following_state;motionText='跟隨'+(shoulder?'肩':'肘')+'｜'+(pulseStateLabels[state]||state||'--')+(f.rom_limit_active?'｜ROM 限制':'');}
    setText('motionState',motionText);

    if(f.time!==latestTime){
      captureQuickTestSample(f,measured);
      measuredHistory.push(measured);
      errorHistory.push(followingOnly&&showQuickTestCue?evaluationError:f.error);
      timeHistory.push(f.time);
      latestTime = f.time;

      if(measuredHistory.length>maxPoints){
        measuredHistory.shift();
        errorHistory.shift();
        timeHistory.shift();
      }
      schedulePlot();
    }
  }
}

function updateILCPanel(v){
  if(!v||v.enabled!==true){setText('ilcStatus','ILC 尚未初始化。');return;}
  let lines=['狀態：'+(v.frozen?'學習凍結':'學習啟用')+'｜目前周期 '+(v.current_cycle===null?'--':v.current_cycle)];
  lines.push('有效／無效周期：'+v.valid_cycles+'／'+v.invalid_cycles+'｜覆蓋率 '+fmtMetric((v.current_cycle_coverage||0)*100,1)+'%');
  lines.push('最近 RMSE '+fmtMetric(v.last_rmse,2)+'°｜改善 '+fmtMetric(v.improvement_percent,1)+'%｜learned peak '+fmtMetric(v.learned_peak_pwm,1)+' PWM');
  if(v.last_cycle_reason)lines.push(v.last_cycle_reason);
  setText('ilcStatus',lines.join('\n'));
}

function updateFrequencyPanel(v){
  if(!v||!v.state){return;}
  let lines=['狀態：'+v.state+'｜樣本：'+(v.sample_count||0)];
  if(v.state==='running')lines.push('剩餘 '+fmtMetric(v.remaining,1)+' 秒｜目前命令 '+fmtMetric(v.current_command,1)+' PWM');
  if(v.reason)lines.push(v.reason);
  const s=v.summary||{};
  if(v.state==='analyzed'){
    lines.push('有效取樣率 '+fmtMetric(s.effective_sample_rate_hz,1)+' Hz｜timing jitter '+fmtMetric((s.timing_jitter_std_seconds||0)*1000,2)+' ms');
    lines.push('可信頻點 '+(s.trusted_frequency_bins||0)+'｜共振 '+fmtMetric(s.resonance_frequency_hz,2)+' Hz｜-3dB 頻寬 '+fmtMetric(s.bandwidth_3db_hz,2)+' Hz');
    lines.push('等效相位延遲 '+fmtMetric((s.equivalent_phase_delay_seconds||0)*1000,1)+' ms');
  }
  if(v.csv_path)lines.push('CSV：'+v.csv_path);
  if(v.png_path)lines.push('PNG：'+v.png_path);
  setText('frequencyStatus',lines.join('\n'));
}

function fmtMetric(value, digits=1){
  return (typeof value==='number' && Number.isFinite(value))?value.toFixed(digits):'--';
}
function updateValidationPanel(v){
  const stageButton=document.getElementById('validationStageButton');
  const applyButton=document.getElementById('validationApplyButton');

  if(validationPreparing){
    stageButton.disabled=true;
    applyButton.disabled=true;
    return;
  }

  if(!v.state || v.state==='idle'){
    setLiveValidationGuide('請先以手臂自然下垂姿勢初始化並完成重力校正，再開始個人化 ROM。');
    setText('validationResults','尚無個人化 ROM 記錄。');
    document.getElementById('validationProgressBar').style.width='0%';
    stageButton.disabled=true;applyButton.disabled=true;return;
  }

  if(v.current_stage){
    showValidationProtocol(v.current_stage);
    const duration=v.stage_duration_seconds||10;
    const progress=Math.max(0,Math.min(100,100*(v.elapsed||0)/duration));
    document.getElementById('validationProgressBar').style.width=progress.toFixed(1)+'%';
    const guide='現在執行：'+v.current_stage_label+'\n目前動作：'+(v.phase||'')
      +'\n剩餘 '+fmtMetric(v.remaining,1)+' 秒｜樣本 '+(v.sample_count||0);
    setLiveValidationGuide(guide);
    stageButton.disabled=true;
    if(v.remaining<=0.05)finishValidationStage();
  }else if(v.expected_stage){
    showValidationProtocol(v.expected_stage);
    document.getElementById('validationProgressBar').style.width='0%';
    setLiveValidationGuide('下一階段：'+v.expected_stage_label+'\n先閱讀下方完整動作內容，再按「2 秒準備後開始目前階段」。');
    stageButton.disabled=false;
  }else if(v.ready_to_apply && !v.applied){
    document.getElementById('validationProgressBar').style.width='100%';
    setLiveValidationGuide('四個 ROM 動作已記錄。請檢查數值後，按「確認並套用個人化 ROM」。');
    stageButton.disabled=true;
  }else if(v.applied){
    document.getElementById('validationProgressBar').style.width='100%';
    setLiveValidationGuide('病患個人化 ROM 已套用，本次初始化有效。');
    stageButton.disabled=true;
  }else{
    document.getElementById('validationProgressBar').style.width='100%';
    setLiveValidationGuide('ROM 記錄不完整，請重新開始缺少的動作。');
    stageButton.disabled=true;
  }
  applyButton.disabled=!v.ready_to_apply||v.applied;

  let lines=[];
  const results=v.results||{};
  if(results.elbow_flexion){
    const r=results.elbow_flexion;
    lines.push('已記錄｜最大肘屈曲：'+fmtMetric(r.flexion_max_deg)+'°｜軸='+r.elbow_axis+'｜sign='+r.elbow_sign);
  }
  if(results.elbow_extension){
    const r=results.elbow_extension;
    lines.push('已記錄｜肘伸展終點：'+fmtMetric(r.extension_angle_deg)+'°｜90°起始量測：'+fmtMetric(r.start_90_angle_deg)+'°');
  }
  if(results.shoulder_front){
    const r=results.shoulder_front;
    lines.push('已記錄｜最大前平舉：'+fmtMetric(r.maximum_deg)+'°｜回到下垂：'+fmtMetric(r.return_angle_deg)+'°');
  }
  if(results.shoulder_side){
    const r=results.shoulder_side;
    lines.push('已記錄｜最大側平舉：'+fmtMetric(r.maximum_deg)+'°｜回到下垂：'+fmtMetric(r.return_angle_deg)+'°');
  }
  if(v.rom){
    const r=v.rom;
    lines.push('建議 90% 訓練目標：');
    lines.push('  肘屈曲 '+fmtMetric(r.elbow_flexion_target_deg)+'°｜肘伸展 '+fmtMetric(r.elbow_extension_target_deg)+'°');
    lines.push('  前平舉 '+fmtMetric(r.shoulder_front_target_deg)+'°｜側平舉 '+fmtMetric(r.shoulder_side_target_deg)+'°');
  }
  (v.warnings||[]).forEach(warning=>lines.push('注意：'+warning));
  setText('validationResults',lines.length?lines.join('\n'):'等待第一個 ROM 動作。');
}

function targetAtTime(t){
  let minA=fval('trajectoryMinInput',30);
  let maxA=fval('trajectoryMaxInput',90);
  if(maxA<minA){const tmp=minA; minA=maxA; maxA=tmp;}
  const center=0.5*(minA+maxA);
  const amp=0.5*(maxA-minA);
  const mode=document.getElementById('targetModeInput').value;

  if(mode==='fixed') return fval('fixedTargetInput',60);

  if(mode==='step'){
    const hold=Math.max(fval('stepHoldInput',3),0.1);
    const idx=Math.floor(Math.max(t,0)/hold);
    return (idx%2===1)?maxA:minA;
  }

  const period=Math.max(fval('trajectoryPeriodInput',5),0.1);
  return center - amp*Math.cos(2*Math.PI*Math.max(t,0)/period);
}

function drawPlot(){
  const c=document.getElementById('angleCanvas'),ctx=c.getContext('2d'),w=c.width,h=c.height;
  ctx.clearRect(0,0,w,h);

  // 共用時間視窗：左邊顯示已發生資料，右邊顯示未來 target preview
  const currentTime = latestTime || 0;
  const windowStart = Math.max(0, currentTime - pastWindowSeconds);
  const windowEnd = currentTime + futureWindowSeconds;
  const windowSpan = Math.max(windowEnd - windowStart, 0.1);

  // 版面切成上下兩張圖：上面角度，下面誤差
  const ml=55,mr=20,topMt=20,topH=285,gap=42,errMt=topMt+topH+gap,errH=135,mb=35;
  const pw=w-ml-mr;
  const angleYMin=0, angleYMax=120;

  let minA=fval('trajectoryMinInput',30);
  let maxA=fval('trajectoryMaxInput',90);
  if(maxA<minA){const tmp=minA; minA=maxA; maxA=tmp;}
  // error 使用獨立 y 軸，0° 在中間。範圍依目標角度自動給足。
  const errorLimit=Math.max(30, Math.ceil(Math.max(Math.abs(minA), Math.abs(maxA), Math.abs(maxA-minA))/10)*10);

  function mxTime(t){return ml + ((t-windowStart)/windowSpan)*pw;}
  function myAngle(v){v=Math.max(angleYMin,Math.min(v,angleYMax));return topMt+topH-(v-angleYMin)/(angleYMax-angleYMin)*topH;}
  function myErr(v){v=Math.max(-errorLimit,Math.min(v,errorLimit));return errMt+errH-(v+errorLimit)/(2*errorLimit)*errH;}

  ctx.font='12px Arial';

  // ---------- 上圖：target / measured angle ----------
  ctx.strokeStyle='#d1d5db';ctx.strokeRect(ml,topMt,pw,topH);
  ctx.fillStyle='#374151';ctx.fillText('Angle (deg)',ml,topMt-7);

  ctx.strokeStyle='#eef2f7';ctx.fillStyle='#6b7280';
  [0,30,60,90,120].forEach(a=>{
    const y=myAngle(a);
    ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(ml+pw,y);ctx.stroke();
    ctx.fillText(String(a),ml-35,y+4);
  });

  const xNow = mxTime(currentTime);
  ctx.strokeStyle='#111827';ctx.lineWidth=1;ctx.setLineDash([5,5]);
  ctx.beginPath();ctx.moveTo(xNow,topMt);ctx.lineTo(xNow,topMt+topH);ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle='#111827';ctx.fillText('現在',xNow+5,topMt+topH-8);

  const followingOnly=document.getElementById('controllerModeInput').value==='following_only';
  // 灰線：完整目標軌跡預覽，純跟隨模式不顯示目標。
  if(!followingOnly||quickTestActive){
    ctx.strokeStyle='#9ca3af';ctx.lineWidth=2;ctx.beginPath();
    const samples=160;
    for(let i=0;i<samples;i++){
      const tt = windowStart + (i/(samples-1))*windowSpan;
      const x = mxTime(tt);
      const y = myAngle(targetAtTime(tt));
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  function drawAngleSeries(times, vals, color, width){
    if(times.length<2)return;
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();
    let started=false;
    for(let i=0;i<times.length;i++){
      const t=times[i];
      if(t<windowStart || t>currentTime) continue;
      const x=mxTime(t), y=myAngle(vals[i]);
      if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
    }
    if(started) ctx.stroke();
  }
  drawAngleSeries(timeHistory, measuredHistory, '#2563eb', 3);

  ctx.fillStyle='#2563eb';ctx.fillText('Measured selected joint',ml+10,topMt+18);
  ctx.fillStyle='#6b7280';ctx.fillText(followingOnly?(quickTestActive?'Target cue only':'Target ignored'):'Target preview',ml+190,topMt+18);

  // ---------- 下圖：tracking error，獨立 y 軸 ----------
  ctx.strokeStyle='#d1d5db';ctx.strokeRect(ml,errMt,pw,errH);
  ctx.fillStyle='#374151';ctx.fillText('Tracking error (deg)',ml,errMt-7);

  // Error 水平格線：+limit、0、-limit。0° 誤差線在中間。
  ctx.strokeStyle='#eef2f7';ctx.fillStyle='#6b7280';
  [-errorLimit,0,errorLimit].forEach(e=>{
    const y=myErr(e);
    ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(ml+pw,y);ctx.stroke();
    ctx.fillText(String(e),ml-42,y+4);
  });

  // 0 誤差線加粗，讓使用者清楚知道紅線貼近這條線就是跟上目標
  ctx.strokeStyle='#9ca3af';ctx.lineWidth=1.2;
  ctx.beginPath();ctx.moveTo(ml,myErr(0));ctx.lineTo(ml+pw,myErr(0));ctx.stroke();

  // 目前時間線也畫在 error 圖
  ctx.strokeStyle='#111827';ctx.lineWidth=1;ctx.setLineDash([5,5]);
  ctx.beginPath();ctx.moveTo(xNow,errMt);ctx.lineTo(xNow,errMt+errH);ctx.stroke();
  ctx.setLineDash([]);

  function drawErrorSeries(times, vals, color, width){
    if(times.length<2)return;
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();
    let started=false;
    for(let i=0;i<times.length;i++){
      const t=times[i];
      if(t<windowStart || t>currentTime) continue;
      const x=mxTime(t), y=myErr(vals[i]);
      if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
    }
    if(started) ctx.stroke();
  }
  drawErrorSeries(timeHistory, errorHistory, '#dc2626', 2.2);

  ctx.fillStyle='#dc2626';ctx.fillText('Error = target - measured',ml+10,errMt+18);
  ctx.fillStyle='#6b7280';ctx.fillText('0° error',ml+150,myErr(0)-6);

  // 時間標籤
  ctx.fillStyle='#6b7280';
  ctx.fillText(windowStart.toFixed(1)+'s',ml,errMt+errH+22);
  ctx.fillText(currentTime.toFixed(1)+'s',xNow-12,errMt+errH+22);
  ctx.fillText(windowEnd.toFixed(1)+'s',ml+pw-50,errMt+errH+22);
}


async function pollStatus(){
  const data=await getJSON('/api/status');
  if(data.ok===false && !data.status)showStatus(data.message,1000);
  else updatePage(data);
  setTimeout(pollStatus,document.hidden?1500:fullStatusPollDelayMs);
}
async function pollLive(){
  // Request the richer scalar frame only while a quick test is recording.
  // Normal GUI polling remains compact so the page stays responsive.
  const data=await getJSON(quickTestActive?'/api/live_status?analysis=1':'/api/live_status');
  if(data.ok===false){
    livePollFailures=Math.min(livePollFailures+1,5);
  }else{
    livePollFailures=0;
    updateLiveFrame(data);
  }
  const retryDelay=livePollFailures?Math.min(1000,livePollDelayMs*Math.pow(2,livePollFailures)):livePollDelayMs;
  setTimeout(pollLive,document.hidden?1000:retryDelay);
}
setupCollapsibleCards();
restoreParameterInputs();
updateQuickFlowInputs();
pollStatus();
pollLive();
updateControllerMode();
drawPlot();
</script>
</body>
</html>
"""

def safe_float(value, default):
    try:
        return float(value)
    except Exception:
        return default

def safe_bool(value):
    return bool(value)

def normalize_controller_mode(mode):
    value = str(mode or "pid").lower()
    aliases = {
        "assist": "pid",
        "assist_as_needed": "pid",
        "aan": "pid",
        "continuous_assist": "pid",
        "continuous": "pid",
        "feedforward_assist": "pid",
        "feedforward_adrc": "adrc",
    }
    value = aliases.get(value, value)
    return value if value in ("pid", "adrc", "ilc_pid", "ilc_adrc", "following_only", "trajectory_demo") else "pid"

def set_status(message):
    global system_status
    with state_lock:
        system_status = message


def imu_channels_connected(channel_status=None):
    channels = imu_channel_health_snapshot if channel_status is None else channel_status
    return all(
        bool((channels.get(name) or {}).get("connected", False))
        for name in ("upper_arm", "forearm")
    )


def validate_imu_frame(frame, previous_angles=None, channel_status=None):
    """Reject missing sensors, non-finite angles and impossible one-sample jumps."""
    if not imu_channels_connected(channel_status):
        missing = [
            (channel_status or imu_channel_health_snapshot).get(name, {}).get("label", label)
            for name, label in (("upper_arm", "上臂 IMU"), ("forearm", "前臂 IMU"))
            if not (channel_status or imu_channel_health_snapshot).get(name, {}).get("connected", False)
        ]
        return False, f"{'、'.join(missing) or 'IMU'} 斷線"
    if not isinstance(frame, dict):
        return False, "IMU 沒有回傳角度資料"
    current = {}
    for key, label in (("elbow_angle_raw", "肘角"), ("shoulder_angle", "肩角")):
        try:
            current[key] = float(frame[key])
        except (KeyError, TypeError, ValueError):
            return False, f"{label}資料遺失"
        if not math.isfinite(current[key]):
            return False, f"{label}出現非有限值"
    if previous_angles:
        for key, label in (("elbow_angle_raw", "肘角"), ("shoulder_angle", "肩角")):
            jump = abs(current[key] - float(previous_angles[key]))
            if jump > IMU_MAX_SINGLE_SAMPLE_JUMP_DEG:
                return False, (
                    f"{label}單次跳變 {jump:.1f}°，超過安全門檻 "
                    f"{IMU_MAX_SINGLE_SAMPLE_JUMP_DEG:.0f}°"
                )
    return True, ""


def latch_imu_safety_fault(reason):
    """Stop therapeutic output and require an explicit human recovery action."""
    global imu_safety_fault_latched, imu_safety_fault_reason, imu_safety_fault_time
    global imu_recovery_stable_samples, imu_recovery_ready, imu_last_valid_angles
    with state_lock:
        imu_safety_fault_latched = True
        imu_safety_fault_reason = str(reason) or "IMU 資料異常"
        imu_safety_fault_time = datetime.now().isoformat(timespec="seconds")
        imu_recovery_stable_samples = 0
        imu_recovery_ready = False
        imu_last_valid_angles = None
        if sensor is not None:
            sensor.motor_enabled = False
        if frequency_session.status().get("state") == "running":
            frequency_session.stop(
                f"IMU 安全鎖定：{imu_safety_fault_reason}", aborted=True
            )
    stop_motor_bridge_safely()
    muscle_allocator.reset_conditioner()
    set_status(
        f"IMU 安全鎖已啟動：{imu_safety_fault_reason}。\n"
        "即使 IMU 自動重連，復健馬達也不會自動恢復；請先檢查姿勢與繩索。"
    )


def note_valid_imu_frame(frame):
    """Track valid angles and readiness; never clear a latched fault automatically."""
    global imu_last_valid_angles, imu_recovery_stable_samples, imu_recovery_ready
    current = {
        "elbow_angle_raw": float(frame["elbow_angle_raw"]),
        "shoulder_angle": float(frame["shoulder_angle"]),
    }
    with state_lock:
        imu_last_valid_angles = current
        if imu_safety_fault_latched:
            imu_recovery_stable_samples = min(
                IMU_RECOVERY_STABLE_SAMPLES_REQUIRED,
                imu_recovery_stable_samples + 1,
            )
            imu_recovery_ready = (
                imu_recovery_stable_samples >= IMU_RECOVERY_STABLE_SAMPLES_REQUIRED
            )

def serialize_frame(frame):
    if frame is None:
        return None
    keys_float = [
        "time","upper_angle","forearm_angle","signed_elbow_angle","elbow_roll_signed","elbow_pitch_signed","elbow_sign","elbow_angle_raw","elbow_angle",
        "measured_angle","measured_angle_raw","upper_roll","upper_pitch","forearm_roll","forearm_pitch",
        "omega","alpha","jerk","shoulder_angle","shoulder_angle_raw","shoulder_angle_velocity","shoulder_release_pwm","trajectory_time","trajectory_wait_remaining","frequency_command",
        "shoulder_dx","shoulder_dy","shoulder_dz","target_angle","target_velocity","reference_target_angle","reference_target_velocity","error","error_for_control","error_active_time",
        "integral_error","derivative_error","filtered_measurement_velocity","raw_pid_output","feedback_pid_output","feedforward_output","following_output","elbow_following_output","shoulder_following_output","controller_compensation_output","elbow_motor_cmd","shoulder_motor_cmd","pid_output","motor_cmd","servo_cmd",
        "adrc_output","adrc_raw_output","adrc_estimated_angle","adrc_estimated_velocity","adrc_estimated_disturbance","adrc_observer_error",
        "ilc_output","ilc_current_cycle","ilc_last_rmse","ilc_improvement_percent","ilc_learned_peak_pwm",
        "deadband_on","deadband_off",
        "desired_motor_cmd","desired_pwm_biceps","desired_pwm_triceps","desired_pwm_deltoid",
        "pwm_biceps","pwm_triceps","pwm_deltoid",
        "virtual_cable_effort_biceps","virtual_cable_effort_triceps","virtual_cable_effort_deltoid","shoulder_coupling_pwm","shoulder_coupling_effort","biceps_balance_release_extra_pwm","triceps_balance_release_extra_pwm","elbow_soft_landing_scale",
        "encoder_count_biceps","encoder_count_triceps","encoder_count_deltoid",
        "encoder_vel_biceps","encoder_vel_triceps","encoder_vel_deltoid"
    ]
    out = {}
    for k in keys_float:
        try:
            out[k] = float(frame.get(k, 0.0))
        except Exception:
            out[k] = 0.0
    for k in ["motion_state","shoulder_motion_state","shoulder_plane","shoulder_coupling_phase","elbow_axis","target_mode","target_joint","controller_mode","serial_tx"]:
        out[k] = str(frame.get(k, ""))
    for k in ["shoulder_assist","shoulder_release_command","shoulder_motor_enable","shoulder_accel_valid","control_active","anti_windup_active","adrc_saturated","ilc_frozen","motor_enabled","emergency_stop","patient_rom_active","rom_limit_active","target_rom_clamped"]:
        out[k] = bool(frame.get(k, False))
    return out

def serialize_live_frame(frame):
    """Small payload for the 20 Hz display path; control runs independently."""
    if frame is None:
        return None
    out = {}
    for key in (
        "time", "measured_angle", "elbow_angle", "shoulder_angle",
        "target_angle", "reference_target_angle", "error", "elbow_roll_signed", "elbow_pitch_signed",
        "shoulder_angle_velocity",
    ):
        try:
            out[key] = float(frame.get(key, 0.0))
        except Exception:
            out[key] = 0.0
    for key in (
        "target_joint", "controller_mode", "motion_state",
        "elbow_following_state", "shoulder_following_state",
    ):
        out[key] = str(frame.get(key, ""))
    out["rom_limit_active"] = bool(frame.get("rom_limit_active", False))
    return out

def apply_params_to_sensor(data, reset_controller=True):
    global sensor
    if sensor is None:
        return
    params = {
        "target_joint": data.get("target_joint", "elbow"),
        "target_mode": data.get("target_mode", "fixed"),
        "fixed_target_angle": safe_float(data.get("fixed_target_angle"), 60.0),
        "trajectory_min_angle": safe_float(data.get("trajectory_min_angle"), 30.0),
        "trajectory_max_angle": safe_float(data.get("trajectory_max_angle"), 90.0),
        "trajectory_period": safe_float(data.get("trajectory_period"), 5.0),
        "step_hold_time": safe_float(data.get("step_hold_time"), 3.0),
        "controller_mode": normalize_controller_mode(data.get("controller_mode", "pid")),
        "deadband": min(max(safe_float(data.get("deadband"), 4.0), 3.0), 5.0),
        "deadband_on": min(max(safe_float(data.get("deadband"), 4.0), 3.0), 5.0),
        "deadband_off": max(1.0, min(max(safe_float(data.get("deadband"), 4.0), 3.0), 5.0) - 2.0),
        "assist_delay": safe_float(data.get("assist_delay"), 0.10),
        "assist_feedforward_gain": 0.0,
        "assist_feedforward_min_pwm": min(max(0.0, safe_float(data.get("assist_feedforward_min_pwm"), 10.0)), 255.0),
        "assist_feedforward_velocity_threshold": 2.0,
        "demo_feedforward_gain": max(0.0, safe_float(data.get("demo_feedforward_gain"), 0.35)),
        "demo_min_pwm": max(0.0, safe_float(data.get("demo_min_pwm"), 8.0)),
        "demo_output_limit": min(abs(safe_float(data.get("demo_output_limit"), 100.0)), 100.0),
        "demo_start_delay": 1.0,
        "adrc_controller_bandwidth": min(max(safe_float(data.get("adrc_controller_bandwidth"), 2.0), 0.1), 20.0),
        "adrc_observer_bandwidth": min(max(safe_float(data.get("adrc_observer_bandwidth"), 8.0), 0.3), 60.0),
        "adrc_input_gain": safe_float(data.get("adrc_input_gain"), 1.0),
        "ilc_learning_gain": min(max(safe_float(data.get("ilc_learning_gain"), 0.08), 0.0), 2.0),
        "ilc_forgetting_factor": min(max(safe_float(data.get("ilc_forgetting_factor"), 0.98), 0.0), 1.0),
        "ilc_q_filter_window": int(min(max(safe_float(data.get("ilc_q_filter_window"), 9), 1), 51)),
        "ilc_update_limit": min(max(abs(safe_float(data.get("ilc_update_limit"), 3.0)), 0.0), 30.0),
        "ilc_learned_limit": min(max(abs(safe_float(data.get("ilc_learned_limit"), 20.0)), 0.0), 100.0),
        "ilc_bins": int(min(max(safe_float(data.get("ilc_bins"), 200), 50), 500)),
        "output_limit": min(abs(safe_float(data.get("output_limit"), 55.0)), 255.0),
        "kp": safe_float(data.get("kp"), 5.0),
        "ki": safe_float(data.get("ki"), 0.02),
        "kd": safe_float(data.get("kd"), 0.05),
        "integral_limit": safe_float(data.get("integral_limit"), 100.0),
    }
    if hasattr(sensor, "configure_control"):
        sensor.configure_control(reset_controller=reset_controller, **params)
    else:
        for k, v in params.items():
            setattr(sensor, k, v)


def current_control_settings():
    if sensor is None:
        return None
    names = (
        "target_joint", "target_mode", "controller_mode", "fixed_target_angle",
        "trajectory_min_angle", "trajectory_max_angle", "trajectory_period",
        "step_hold_time", "deadband_on", "assist_feedforward_min_pwm",
        "demo_output_limit", "adrc_controller_bandwidth",
        "adrc_observer_bandwidth", "adrc_input_gain", "ilc_learning_gain",
        "ilc_forgetting_factor", "ilc_q_filter_window", "ilc_update_limit",
        "ilc_learned_limit", "output_limit", "kp", "ki", "kd",
        "integral_limit",
    )
    return {name: getattr(sensor, name, None) for name in names}

def connect_motor_bridge_if_needed():
    global motor_bridge, motor_bridge_connected, motor_bridge_error

    if motor_bridge_connected and motor_bridge is not None:
        return True

    try:
        motor_bridge = ESP32MotorBridge(
            port=ESP32_PORT,
            baudrate=115200,
            pwm_limit=MOTOR_PWM_LIMIT,
        )
        motor_bridge.connect()
        motor_bridge_connected = True
        motor_bridge_error = ""
        return True
    except Exception as e:
        motor_bridge_connected = False
        motor_bridge_error = str(e)
        return False

def stop_motor_bridge_safely():
    global motor_bridge, motor_bridge_connected, motor_bridge_error
    try:
        if motor_bridge is not None:
            motor_bridge.stop()
    except Exception as e:
        motor_bridge_error = str(e)
        motor_bridge_connected = False


def apply_motor_channel_locks(pwm):
    """Force locked physical motor channels to zero for every output mode."""
    with state_lock:
        locks = dict(motor_channel_locks)
    return MusclePWM(
        biceps=0 if locks["biceps"] else int(pwm.biceps),
        triceps=0 if locks["triceps"] else int(pwm.triceps),
        deltoid=0 if locks["deltoid"] else int(pwm.deltoid),
    )


def apply_motor_direction_sign(pwm):
    """Convert logical muscle directions to physical ESP32 channel signs."""
    return MusclePWM(
        biceps=int(pwm.biceps) * MOTOR_DIRECTION_SIGN[0],
        triceps=int(pwm.triceps) * MOTOR_DIRECTION_SIGN[1],
        deltoid=int(pwm.deltoid) * MOTOR_DIRECTION_SIGN[2],
    )


def apply_motor_output_scale(pwm):
    """Apply per-channel physical strength calibration before direction signs."""
    return MusclePWM(
        biceps=int(round(int(pwm.biceps) * MOTOR_OUTPUT_SCALE[0])),
        triceps=int(round(int(pwm.triceps) * MOTOR_OUTPUT_SCALE[1])),
        deltoid=int(round(int(pwm.deltoid) * MOTOR_OUTPUT_SCALE[2])),
    )


def preserve_biceps_component_through_global_scale(pwm, component_pwm):
    """Pre-compensate one biceps component that must remain unscaled.

    Shoulder coupling is a cable-path compensation rather than elbow assist.
    Its biceps payout/rewind must therefore remain equal to the triceps side
    even though ordinary biceps commands are globally reduced to 0.8.
    """
    scale = float(MOTOR_OUTPUT_SCALE[0])
    if abs(scale) < 1e-9 or abs(scale - 1.0) < 1e-9:
        return pwm
    adjustment = float(component_pwm) * (1.0 / scale - 1.0)
    return MusclePWM(
        biceps=int(round(float(pwm.biceps) + adjustment)),
        triceps=int(pwm.triceps),
        deltoid=int(pwm.deltoid),
    )


def send_motor_pwm_with_locks(pwm_biceps, pwm_triceps, pwm_deltoid):
    """Single final gateway for all non-STOP PWM sent to the ESP32."""
    locked_command = apply_motor_channel_locks(
        MusclePWM(pwm_biceps, pwm_triceps, pwm_deltoid)
    )
    scaled_command = apply_motor_output_scale(locked_command)
    physical_command = apply_motor_direction_sign(scaled_command)
    motor_bridge.set_pwm(
        physical_command.biceps,
        physical_command.triceps,
        physical_command.deltoid,
    )
    return physical_command


def stop_reader_safely(timeout=1.0):
    """Stop and join the current reader before any new SMBus owner is created."""
    global reader_running, reader_thread
    reader_running = False
    thread = reader_thread
    if thread is not None and thread.is_alive() and thread is not threading.current_thread():
        thread.join(timeout=max(0.0, float(timeout)))
    stopped = thread is None or not thread.is_alive()
    if stopped:
        reader_thread = None
    return stopped


def imu_watchdog_loop():
    """Soft-interrupt a stalled bus first; hard restart only as a last resort."""
    global imu_recovery_requested_monotonic
    while True:
        now = time.monotonic()
        if initializing and imu_init_started_monotonic > 0.0:
            age = now - imu_init_started_monotonic
            if age > IMU_INIT_STALL_TIMEOUT:
                try:
                    stop_motor_bridge_safely()
                finally:
                    print(
                        f"WATCHDOG: IMU initialization stalled for {age:.1f}s; "
                        "exiting for supervised restart",
                        flush=True,
                    )
                    os._exit(70)
        elif initialized and reader_running and imu_last_success_monotonic > 0.0:
            age = now - imu_last_success_monotonic
            if age > IMU_READER_STALL_TIMEOUT:
                if imu_recovery_requested_monotonic <= 0.0:
                    imu_recovery_requested_monotonic = now
                    latch_imu_safety_fault(
                        f"IMU reader 超過 {age:.1f} 秒沒有有效資料"
                    )
                    if sensor is not None and hasattr(sensor, "interrupt_bus"):
                        sensor.interrupt_bus()
                    print(
                        f"WATCHDOG: IMU reader stalled for {age:.1f}s; "
                        "requested soft reconnect",
                        flush=True,
                    )
                elif now - imu_recovery_requested_monotonic > IMU_READER_HARD_STALL_TIMEOUT:
                    try:
                        if sensor is not None:
                            sensor.motor_enabled = False
                        stop_motor_bridge_safely()
                    finally:
                        print(
                            "WATCHDOG: IMU soft reconnect could not unblock I2C; "
                            "exiting for supervised restart",
                            flush=True,
                        )
                        os._exit(70)
            else:
                imu_recovery_requested_monotonic = 0.0
        time.sleep(0.2)


def reader_loop():
    global latest_frame, record_count, motor_bridge_connected, motor_bridge_error
    global imu_last_attempt_monotonic, imu_last_success_monotonic
    global imu_read_failure_count, imu_last_error
    global imu_recovery_requested_monotonic
    global imu_channel_health_snapshot
    global imu_last_valid_angles

    while reader_running:
        try:
            # 1) Read IMU + calculate target/error/PID motor_cmd
            imu_last_attempt_monotonic = time.monotonic()
            frame = sensor.read_frame()
            imu_channel_health_snapshot = sensor.imu_channel_status()
            frame_valid, frame_error = validate_imu_frame(
                frame, imu_last_valid_angles, imu_channel_health_snapshot
            )
            if not frame_valid:
                latch_imu_safety_fault(frame_error)
                time.sleep(0.02)
                continue
            note_valid_imu_frame(frame)
            if imu_safety_fault_latched:
                # Reconnection restores measurement/display only.  The frame
                # produced by read_frame may predate this assignment, so also
                # overwrite its output gate explicitly.
                sensor.motor_enabled = False
                frame["motor_enabled"] = False
            imu_last_success_monotonic = time.monotonic()
            imu_last_error = ""
            imu_recovery_requested_monotonic = 0.0
            rom_calibration.add_sample(frame)

            # 2) Read ESP32 encoder feedback if connected
            fb = None
            if motor_bridge_connected and motor_bridge is not None:
                try:
                    fb = motor_bridge.get_feedback()
                except Exception as e:
                    motor_bridge_error = str(e)
                    motor_bridge_connected = False
                    fb = None

            # 3) Allocate joint-level control output to 3 muscle motors.
            # desired_pwm: 只用來顯示「如果啟用馬達，控制器想輸出什麼」。
            # output_pwm: 真正會送到 ESP32；只有 motor_enabled=True 且沒有急停時才非 0。
            frequency_running = frequency_session.status().get("state") == "running"
            frequency_now = time.monotonic()
            desired_cmd = frequency_session.command(frequency_now) if frequency_running else frame.get("pid_output", 0)
            if frequency_running:
                frame["controller_mode"] = "system_identification"
                frame["target_mode"] = frequency_session.config.get("signal_type", "chirp")
                frame["frequency_command"] = desired_cmd
            therapeutic_following = (
                not frequency_running
                and frame.get("controller_mode") != "trajectory_demo"
            )
            if therapeutic_following:
                trained_joint = str(frame.get("target_joint", "elbow")).lower()
                elbow_cmd = frame.get("elbow_motor_cmd", 0.0)
                shoulder_cmd = frame.get("shoulder_motor_cmd", 0.0)
                desired_cmd = (
                    shoulder_cmd
                    if trained_joint in ("shoulder", "shoulder_joint", "肩關節")
                    else elbow_cmd
                )

                elbow_pwm = muscle_allocator.allocate(
                    target_joint="elbow",
                    motor_cmd=elbow_cmd,
                    control_profile=MOTOR_CONTROL_PROFILE,
                    motor_enabled=True,
                    emergency_stop=False,
                    # The GUI following PWM is the exact base amplitude.
                    # Never promote it through a second allocator floor.
                    enforce_minimum=False,
                    feedback=fb,
                    now=frame.get("time"),
                )
                if (
                    frame.get("controller_mode") == "following_only"
                    and trained_joint not in ("shoulder", "shoulder_joint", "肩關節")
                ):
                    elbow_pwm = muscle_allocator.cap_triceps_wind(
                        elbow_pwm, FOLLOWING_TRICEPS_WIND_LIMIT
                    )
                    active_elbow_winding = max(
                        0, elbow_pwm.biceps, elbow_pwm.triceps
                    )
                    elbow_pwm = muscle_allocator.apply_elbow_cable_balance(
                        elbow_pwm, active_elbow_winding
                    )
                if trained_joint not in ("shoulder", "shoulder_joint", "肩關節"):
                    elbow_pwm = muscle_allocator.apply_elbow_soft_landing(
                        elbow_pwm, frame.get("elbow_angle", 0.0)
                    )
                    frame["elbow_soft_landing_scale"] = (
                        muscle_allocator.elbow_soft_landing_last_scale
                    )
                else:
                    frame["elbow_soft_landing_scale"] = 1.0
                shoulder_pwm = muscle_allocator.allocate(
                    target_joint="shoulder",
                    motor_cmd=shoulder_cmd,
                    control_profile=MOTOR_CONTROL_PROFILE,
                    motor_enabled=True,
                    emergency_stop=False,
                    shoulder_release_command=frame.get("shoulder_release_command", False),
                    shoulder_release_pwm=frame.get("shoulder_release_pwm", 0.0),
                    shoulder_motor_enable=frame.get("shoulder_motor_enable", True),
                    enforce_minimum=False,
                    feedback=fb,
                    now=frame.get("time"),
                )
                desired_pwm = MusclePWM(
                    elbow_pwm.biceps,
                    elbow_pwm.triceps,
                    shoulder_pwm.deltoid,
                )
                # Shoulder elevation changes the path length of both elbow
                # tendons. Add an equal common-mode payout while raising, then
                # rewind only the PWM-time released by this coupling while
                # lowering. Elbow differential control remains superimposed.
                coupling_deltoid_pwm = (
                    shoulder_pwm.deltoid
                    if not motor_channel_locks.get("deltoid", False)
                    else 0
                )
                desired_pwm = muscle_allocator.apply_shoulder_elbow_coupling(
                    desired_pwm,
                    coupling_deltoid_pwm,
                    motor_enabled=frame.get("motor_enabled", False),
                    emergency_stop=frame.get("emergency_stop", False),
                    deltoid_enabled=not motor_channel_locks.get("deltoid", False),
                    now=frame.get("time"),
                )
                coupling_status = muscle_allocator.shoulder_coupling_status()
                desired_pwm = preserve_biceps_component_through_global_scale(
                    desired_pwm,
                    coupling_status["command_pwm"],
                )
                # A slack triceps cable can take up suddenly at the requested
                # 150+ PWM. Limit winding in pure following mode; payout is
                # intentionally unaffected.
                if frame.get("controller_mode") == "following_only":
                    desired_pwm = muscle_allocator.cap_triceps_wind(
                        desired_pwm, FOLLOWING_TRICEPS_WIND_LIMIT
                    )
                frame["desired_elbow_cmd"] = elbow_cmd
                frame["desired_shoulder_cmd"] = shoulder_cmd
                frame["shoulder_coupling_pwm"] = coupling_status["command_pwm"]
                frame["shoulder_coupling_effort"] = coupling_status["released_effort_pwm_seconds"]
                frame["shoulder_coupling_phase"] = coupling_status["phase"]
                balance_status = muscle_allocator.elbow_cable_balance_status()
                frame["biceps_balance_release_extra_pwm"] = balance_status["biceps_release_extra_pwm"]
                frame["triceps_balance_release_extra_pwm"] = balance_status["triceps_release_extra_pwm"]
            else:
                desired_pwm = muscle_allocator.allocate(
                    target_joint=frame.get("target_joint", "elbow"),
                    motor_cmd=desired_cmd,
                    control_profile=MOTOR_CONTROL_PROFILE,
                    motor_enabled=True,
                    emergency_stop=False,
                    shoulder_release_command=False if frequency_running else frame.get("shoulder_release_command", False),
                    shoulder_release_pwm=frame.get("shoulder_release_pwm", 0.0),
                    shoulder_motor_enable=True if frequency_running else frame.get("shoulder_motor_enable", True),
                    # Identification/demo modes retain their deliberately low
                    # caps and are not promoted to the therapeutic threshold.
                    enforce_minimum=False,
                    feedback=fb,
                    now=frame.get("time"),
                )

            # Global physical-channel locks are applied after every controller
            # and allocator, so no rehabilitation/test mode can bypass them.
            desired_pwm = apply_motor_channel_locks(desired_pwm)

            if frame.get("target_joint") == "shoulder":
                effective_joint_pwm = desired_pwm.deltoid
            elif desired_pwm.biceps > 0:
                effective_joint_pwm = desired_pwm.biceps
            elif desired_pwm.triceps > 0:
                effective_joint_pwm = -desired_pwm.triceps
            else:
                effective_joint_pwm = 0

            frequency_event = None
            if frequency_running:
                frequency_event = frequency_session.add_sample(
                    frame,
                    desired_cmd,
                    effective_joint_pwm,
                    desired_pwm,
                    now=frequency_now,
                )
                if frequency_event:
                    sensor.motor_enabled = False
                    frame["motor_enabled"] = False
                    muscle_allocator.reset_conditioner()
                    try:
                        frequency_session.save_raw()
                    except Exception as exc:
                        frequency_session.reason += f" 原始 CSV 儲存失敗：{exc}"

            can_output = (
                frame.get("motor_enabled", False)
                and not frame.get("emergency_stop", False)
                and not frequency_event
                and not imu_safety_fault_latched
            )
            output_pwm = desired_pwm if can_output else MusclePWM()
            observed_pwm = (
                apply_motor_direction_sign(MusclePWM(fb.pwm1, fb.pwm2, fb.pwm3))
                if can_output and fb is not None
                else output_pwm
            )
            cable_effort = muscle_allocator.observe_output(
                observed_pwm,
                now=frame.get("time"),
            )

            frame["desired_motor_cmd"] = desired_cmd
            frame["desired_pwm_biceps"] = desired_pwm.biceps
            frame["desired_pwm_triceps"] = desired_pwm.triceps
            frame["desired_pwm_deltoid"] = desired_pwm.deltoid

            frame["pwm_biceps"] = output_pwm.biceps
            frame["pwm_triceps"] = output_pwm.triceps
            frame["pwm_deltoid"] = output_pwm.deltoid
            frame["virtual_cable_effort_biceps"] = cable_effort["biceps"]
            frame["virtual_cable_effort_triceps"] = cable_effort["triceps"]
            frame["virtual_cable_effort_deltoid"] = cable_effort["deltoid"]

            if fb is not None:
                frame["encoder_count_biceps"] = fb.count1
                frame["encoder_count_triceps"] = fb.count2
                frame["encoder_count_deltoid"] = fb.count3
                frame["encoder_vel_biceps"] = fb.vel1
                frame["encoder_vel_triceps"] = fb.vel2
                frame["encoder_vel_deltoid"] = fb.vel3
            else:
                frame["encoder_count_biceps"] = 0
                frame["encoder_count_triceps"] = 0
                frame["encoder_count_deltoid"] = 0
                frame["encoder_vel_biceps"] = 0.0
                frame["encoder_vel_triceps"] = 0.0
                frame["encoder_vel_deltoid"] = 0.0

            # 4) Send PWM to ESP32 only when motor is enabled and ESP32 is connected
            if motor_bridge_connected and motor_bridge is not None:
                try:
                    if motor_test_running:
                        frame["serial_tx"] = "MANUAL_MOTOR_TEST"
                    elif motor_arm_in_progress:
                        # api_set_motor owns the STOP -> ARM transaction.
                        # Do not queue a stale STOP behind ARM confirmation.
                        frame["serial_tx"] = "ARMING_WAIT"
                    elif (frame.get("emergency_stop", False)
                            or not frame.get("motor_enabled", False)
                            or imu_safety_fault_latched):
                        frame["serial_tx"] = "STOP"
                        motor_bridge.stop()
                    else:
                        frame["serial_tx"] = f"PWM,{output_pwm.biceps},{output_pwm.triceps},{output_pwm.deltoid}"
                        locked_command = send_motor_pwm_with_locks(
                            output_pwm.biceps, output_pwm.triceps, output_pwm.deltoid
                        )
                        frame["serial_tx"] = (
                            f"PWM,{locked_command.biceps},"
                            f"{locked_command.triceps},{locked_command.deltoid}"
                        )
                except Exception as e:
                    motor_bridge_error = str(e)
                    motor_bridge_connected = False
                    if frequency_session.status().get("state") == "running":
                        frequency_session.stop(f"ESP32 通訊錯誤：{e}", aborted=True)
                        sensor.motor_enabled = False
                        muscle_allocator.reset_conditioner()

            if "serial_tx" not in frame:
                frame["serial_tx"] = "ESP32_NOT_CONNECTED" if not motor_bridge_connected else "NO_COMMAND"

            if frequency_event:
                set_status(
                    "系統識別已完成並自動停止馬達；可以執行頻域分析。"
                    if frequency_event == "completed"
                    else f"系統識別已中止並停止馬達：{frequency_session.reason}"
                )

            # 5) Save latest frame and CSV
            with state_lock:
                latest_frame = frame
                history.append(frame)
                if len(history) > 500:
                    history.pop(0)

                if recording and csv_writer is not None:
                    csv_writer.writerow([
                        datetime.now().isoformat(timespec="milliseconds"),
                        frame.get("time"), participant_id, session_name, current_label,
                        frame.get("target_joint"), frame.get("target_mode"), frame.get("controller_mode"),
                        frame.get("target_angle"), frame.get("target_velocity"), frame.get("measured_angle"), frame.get("measured_angle_raw"), frame.get("error"),
                        frame.get("elbow_angle"), frame.get("elbow_angle_raw"), frame.get("shoulder_angle"), frame.get("shoulder_angle_raw"),
                        frame.get("error_for_control"), frame.get("integral_error"), frame.get("derivative_error"),
                        frame.get("filtered_measurement_velocity"), frame.get("control_active"), frame.get("anti_windup_active"),
                        frame.get("raw_pid_output"), frame.get("feedback_pid_output"), frame.get("feedforward_output"), frame.get("pid_output"), frame.get("motor_cmd"),
                        frame.get("adrc_output"), frame.get("adrc_raw_output"), frame.get("adrc_estimated_angle"), frame.get("adrc_estimated_velocity"), frame.get("adrc_estimated_disturbance"), frame.get("adrc_observer_error"), frame.get("adrc_saturated"),
                        frame.get("ilc_output"), frame.get("ilc_current_cycle"), frame.get("ilc_last_rmse"), frame.get("ilc_improvement_percent"), frame.get("ilc_learned_peak_pwm"), frame.get("ilc_frozen"),
                        frame.get("desired_motor_cmd"), frame.get("desired_pwm_biceps"), frame.get("desired_pwm_triceps"), frame.get("desired_pwm_deltoid"),
                        frame.get("pwm_biceps"), frame.get("pwm_triceps"), frame.get("pwm_deltoid"),
                        frame.get("virtual_cable_effort_biceps"), frame.get("virtual_cable_effort_triceps"), frame.get("virtual_cable_effort_deltoid"), frame.get("elbow_soft_landing_scale"),
                        frame.get("encoder_count_biceps"), frame.get("encoder_count_triceps"), frame.get("encoder_count_deltoid"),
                        frame.get("encoder_vel_biceps"), frame.get("encoder_vel_triceps"), frame.get("encoder_vel_deltoid"),
                        frame.get("upper_angle"), frame.get("forearm_angle"), frame.get("omega"), frame.get("alpha"), frame.get("jerk"),
                        frame.get("shoulder_motion_state"), frame.get("shoulder_release_command"), frame.get("shoulder_release_pwm"), frame.get("shoulder_motor_enable"),
                        frame.get("motor_enabled"), frame.get("emergency_stop"), frame.get("motion_state"),
                    ])
                    record_count += 1
                    if record_count % 50 == 0:
                        csv_file.flush()

            # About 50 Hz is sufficient for rehabilitation motion and halves
            # I2C traffic compared with the former 10 ms loop.
            time.sleep(0.02)

        except Exception as e:
            imu_read_failure_count += 1
            imu_last_error = str(e)
            if sensor is not None and hasattr(sensor, "imu_channel_status"):
                imu_channel_health_snapshot = sensor.imu_channel_status()
            latch_imu_safety_fault(f"IMU／控制迴圈錯誤：{e}")
            if imu_recovery_requested_monotonic <= 0.0:
                imu_recovery_requested_monotonic = time.monotonic()
            try:
                channels = sensor.reconnect_imus(retries=5, retry_delay=0.25)
                imu_channel_health_snapshot = sensor.imu_channel_status()
                imu_last_success_monotonic = time.monotonic()
                imu_last_error = ""
                imu_recovery_requested_monotonic = 0.0
                set_status(
                    f"IMU 已自動重連，通道 {channels}；已沿用原校正。"
                    "復健輸出仍由 IMU 安全鎖禁止；請確認姿勢與繩索後手動解除。"
                )
            except Exception as reconnect_error:
                imu_last_error = str(reconnect_error)
                if sensor is not None and hasattr(sensor, "imu_channel_status"):
                    imu_channel_health_snapshot = sensor.imu_channel_status()
                set_status(
                    f"IMU 自動重連中：{reconnect_error}\n"
                    "馬達維持停止；請檢查接頭是否已插回。"
                )
                time.sleep(0.5)

def init_worker(params):
    global sensor, reader_thread, reader_running, initialized, initializing
    global imu_last_attempt_monotonic, imu_last_success_monotonic
    global imu_init_started_monotonic, imu_read_failure_count, imu_last_error
    global imu_recovery_requested_monotonic
    global imu_channel_health_snapshot
    global imu_safety_fault_latched, imu_safety_fault_reason, imu_safety_fault_time
    global imu_recovery_stable_samples, imu_recovery_ready, imu_last_valid_angles
    new_sensor = None
    try:
        set_status("正在建立 IMU 系統...")
        existing_emergency_stop = bool(
            (sensor is not None and getattr(sensor, "emergency_stop", False))
            or (
                motor_bridge is not None
                and getattr(motor_bridge, "safety_state", "") in ("ESTOP", "EMERGENCY_STOP")
            )
        )
        new_sensor = IMURehabSystem(
            target_joint=params.get("target_joint", "elbow"),
            target_mode=params.get("target_mode", "fixed"),
            fixed_target_angle=safe_float(params.get("fixed_target_angle"), 60.0),
            trajectory_min_angle=safe_float(params.get("trajectory_min_angle"), 30.0),
            trajectory_max_angle=safe_float(params.get("trajectory_max_angle"), 90.0),
            trajectory_period=safe_float(params.get("trajectory_period"), 5.0),
            step_hold_time=safe_float(params.get("step_hold_time"), 3.0),
            controller_mode=normalize_controller_mode(params.get("controller_mode", "pid")),
            deadband=min(max(safe_float(params.get("deadband"), 4.0), 3.0), 5.0),
            deadband_on=min(max(safe_float(params.get("deadband"), 4.0), 3.0), 5.0),
            deadband_off=max(1.0, min(max(safe_float(params.get("deadband"), 4.0), 3.0), 5.0) - 2.0),
            assist_delay=safe_float(params.get("assist_delay"), 0.10),
            assist_feedforward_gain=0.0,
            assist_feedforward_min_pwm=min(max(0.0, safe_float(params.get("assist_feedforward_min_pwm"), 10.0)), 255.0),
            assist_feedforward_velocity_threshold=2.0,
            following_velocity_threshold=5.0,
            demo_feedforward_gain=max(0.0, safe_float(params.get("demo_feedforward_gain"), 0.35)),
            demo_min_pwm=max(0.0, safe_float(params.get("demo_min_pwm"), 8.0)),
            demo_output_limit=min(abs(safe_float(params.get("demo_output_limit"), 100.0)), 100.0),
            demo_start_delay=1.0,
            adrc_controller_bandwidth=min(max(safe_float(params.get("adrc_controller_bandwidth"), 2.0), 0.1), 20.0),
            adrc_observer_bandwidth=min(max(safe_float(params.get("adrc_observer_bandwidth"), 8.0), 0.3), 60.0),
            adrc_input_gain=safe_float(params.get("adrc_input_gain"), 1.0),
            ilc_learning_gain=min(max(safe_float(params.get("ilc_learning_gain"), 0.08), 0.0), 2.0),
            ilc_forgetting_factor=min(max(safe_float(params.get("ilc_forgetting_factor"), 0.98), 0.0), 1.0),
            ilc_q_filter_window=int(min(max(safe_float(params.get("ilc_q_filter_window"), 9), 1), 51)),
            ilc_update_limit=min(max(abs(safe_float(params.get("ilc_update_limit"), 3.0)), 0.0), 30.0),
            ilc_learned_limit=min(max(abs(safe_float(params.get("ilc_learned_limit"), 20.0)), 0.0), 100.0),
            ilc_bins=int(min(max(safe_float(params.get("ilc_bins"), 200), 50), 500)),
            output_limit=min(abs(safe_float(params.get("output_limit"), 55.0)), 255.0),
            kp=safe_float(params.get("kp"), 5.0),
            ki=safe_float(params.get("ki"), 0.02),
            kd=safe_float(params.get("kd"), 0.05),
            integral_limit=safe_float(params.get("integral_limit"), 100.0),
            motor_enabled=False,
            emergency_stop=existing_emergency_stop,
        )
        imu_channel_health_snapshot = new_sensor.imu_channel_status()
        set_status("正在掃描與初始化 MPU6050...")
        channels = new_sensor.scan_and_init()
        imu_channel_health_snapshot = new_sensor.imu_channel_status()
        set_status(f"初始化成功，可用通道: {channels}\n請保持手臂自然下垂並靜止，開始 5 秒校正。")
        def progress(elapsed, remaining):
            set_status(f"校正中... 剩餘 {remaining:.1f} 秒")
        result = new_sensor.calibrate(seconds=5, progress_callback=progress)
        imu_channel_health_snapshot = new_sensor.imu_channel_status()
        muscle_allocator.reset_conditioner()
        muscle_allocator.reset_cable_tracker()
        with state_lock:
            sensor = new_sensor
            rom_calibration.reset()
            initialized = True
            initializing = False
            imu_safety_fault_latched = False
            imu_safety_fault_reason = ""
            imu_safety_fault_time = None
            imu_recovery_stable_samples = 0
            imu_recovery_ready = False
            imu_last_valid_angles = None
        if ENABLE_AUTO_CONNECT_ESP32:
            ok = connect_motor_bridge_if_needed()
            if ok:
                set_status(f"校正完成，樣本數: {result['sample_count']}。\nESP32 motor bridge 已連線。\n馬達維持停止，按 Enable Motor 並確認後才會啟動。")
            else:
                set_status(f"校正完成，樣本數: {result['sample_count']}。\nESP32 尚未連線：{motor_bridge_error}\n馬達維持停止；可先記錄病患個人化 ROM。")
        else:
            set_status(f"校正完成，樣本數: {result['sample_count']}。\n馬達維持停止，按 Enable Motor 並確認後才會啟動。")
        reader_running = True
        imu_last_attempt_monotonic = time.monotonic()
        imu_last_success_monotonic = imu_last_attempt_monotonic
        imu_init_started_monotonic = 0.0
        imu_read_failure_count = 0
        imu_last_error = ""
        imu_recovery_requested_monotonic = 0.0
        reader_thread = threading.Thread(target=reader_loop, daemon=True)
        reader_thread.start()
    except Exception as e:
        with state_lock:
            initializing = False
            initialized = False
        imu_init_started_monotonic = 0.0
        imu_last_error = str(e)
        if new_sensor is not None and hasattr(new_sensor, "imu_channel_status"):
            imu_channel_health_snapshot = new_sensor.imu_channel_status()
        set_status(f"初始化或校正失敗: {e}")

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/api/init", methods=["POST"])
def api_init():
    global initializing, reader_running, imu_init_started_monotonic
    params = dict(request.get_json(force=True) or {})
    params["controller_mode"] = normalize_controller_mode(params.get("controller_mode", "pid"))
    if str(params.get("controller_mode", "")).lower() == "trajectory_demo" and str(params.get("target_mode", "")).lower() != "sine":
        return jsonify({"ok": False, "message": "開迴路軌跡展示模式只允許正弦軌跡。"}), 400
    if str(params.get("controller_mode", "")).lower() in ("adrc", "feedforward_adrc", "ilc_adrc") and abs(safe_float(params.get("adrc_input_gain"), 1.0)) < 0.01:
        return jsonify({"ok": False, "message": "ADRC 輸入增益 b0 的絕對值必須至少為 0.01。"}), 400
    if str(params.get("controller_mode", "")).lower() in ("ilc_pid", "ilc_adrc") and str(params.get("target_mode", "")).lower() != "sine":
        return jsonify({"ok": False, "message": "ILC 只允許固定週期的正弦軌跡。"}), 400
    with state_lock:
        if initializing:
            return jsonify({"ok": False, "message": "系統正在初始化，請稍等。"})
        if sensor is not None:
            sensor.motor_enabled = False
        if frequency_session.status().get("state") == "running":
            frequency_session.stop("系統重新初始化。", aborted=True)
        rom_calibration.reset()
        initializing = True
    if not stop_reader_safely(timeout=1.0):
        with state_lock:
            initializing = False
        stop_motor_bridge_safely()
        threading.Timer(0.5, lambda: os._exit(70)).start()
        return jsonify({
            "ok": False,
            "message": "舊 IMU reader 沒有停止，系統將由 watchdog 安全重啟；馬達維持停止。"
        }), 503
    stop_motor_bridge_safely()
    imu_init_started_monotonic = time.monotonic()
    threading.Thread(target=init_worker, args=(params,), daemon=True).start()
    return jsonify({"ok": True, "message": "已開始初始化與 5 秒校正。"})

@app.route("/api/live_status")
def api_live_status():
    # Keep the high-rate GUI request cheap. ROM, ILC, frequency-analysis and
    # motor configuration are intentionally left to the slower full status call.
    with state_lock:
        return jsonify({
            "ok": True,
            "frame": (
                serialize_frame(latest_frame)
                if request.args.get("analysis", "").lower() in {"1", "true", "yes"}
                else serialize_live_frame(latest_frame)
            ),
        })

@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify({
            "status": system_status,
            "initialized": initialized,
            "initializing": initializing,
            "motor_enabled": bool(sensor is not None and getattr(sensor, "motor_enabled", False)),
            "recording": recording,
            "record_count": record_count,
            "csv_path": current_csv_path,
            "motor_bridge_connected": motor_bridge_connected,
            "motor_bridge_error": motor_bridge_error,
            "motor_bridge_last_line": motor_bridge.last_line if motor_bridge is not None else "",
            "motor_bridge_last_sent": motor_bridge.last_sent_line if motor_bridge is not None and hasattr(motor_bridge, "last_sent_line") else "",
            "motor_bridge_safety_state": getattr(motor_bridge, "safety_state", "DISCONNECTED") if motor_bridge is not None else "DISCONNECTED",
            "motor_arm_in_progress": motor_arm_in_progress,
            "motor_test": {
                "running": motor_test_running,
                "pwm": MOTOR_TEST_PWM,
                "duration_seconds": MOTOR_TEST_DURATION,
            },
            "motor_jog": {
                "active": motor_jog_active,
                "motor": motor_jog_motor,
                "direction": motor_jog_direction,
                "pwm": motor_jog_pwm,
                "default_pwm": MOTOR_JOG_DEFAULT_PWM,
                "max_pwm": MOTOR_JOG_MAX_PWM,
                "heartbeat_timeout_seconds": MOTOR_JOG_HEARTBEAT_TIMEOUT,
                "locks": dict(motor_channel_locks),
            },
            "motor_locks": dict(motor_channel_locks),
            "imu_health": {
                "reader_alive": bool(reader_thread is not None and reader_thread.is_alive()),
                "last_success_age_seconds": (
                    round(max(0.0, time.monotonic() - imu_last_success_monotonic), 3)
                    if imu_last_success_monotonic > 0.0 else None
                ),
                "read_failure_count": imu_read_failure_count,
                "last_error": imu_last_error,
                "watchdog_timeout_seconds": IMU_READER_STALL_TIMEOUT,
                "hard_stall_timeout_seconds": IMU_READER_HARD_STALL_TIMEOUT,
                "reconnecting": imu_recovery_requested_monotonic > 0.0,
                "reconnect_count": getattr(sensor, "reconnect_count", 0) if sensor is not None else 0,
                "transient_retry_count": getattr(sensor, "transient_i2c_retry_count", 0) if sensor is not None else 0,
                "channels": imu_channel_health_snapshot,
                "safety_fault": {
                    "latched": imu_safety_fault_latched,
                    "reason": imu_safety_fault_reason,
                    "latched_at": imu_safety_fault_time,
                    "recovery_ready": imu_recovery_ready,
                    "stable_samples": imu_recovery_stable_samples,
                    "required_stable_samples": IMU_RECOVERY_STABLE_SAMPLES_REQUIRED,
                    "last_valid_angles": dict(imu_last_valid_angles) if imu_last_valid_angles else None,
                },
            },
            "motor_output": {
                "mode": "live_on_confirmation",
                "control_profile": MOTOR_CONTROL_PROFILE,
                "pwm_limit": MOTOR_PWM_LIMIT,
                "motor_min_pwm": dict(zip(("biceps", "triceps", "deltoid"), muscle_allocator.motor_min_pwm)),
                "motor_direction_sign": dict(zip(
                    ("biceps", "triceps", "deltoid"), MOTOR_DIRECTION_SIGN
                )),
                "motor_output_scale": dict(zip(
                    ("biceps", "triceps", "deltoid"), MOTOR_OUTPUT_SCALE
                )),
                "cable_settings": current_cable_settings(),
                "antagonist_release_gain": muscle_allocator.antagonist_release_gain,
                "triceps_release_ratio": muscle_allocator.triceps_release_ratio,
                "biceps_release_ratio": muscle_allocator.biceps_release_ratio,
                "cable_return_gain": muscle_allocator.cable_return_gain,
                "virtual_cable_effort": muscle_allocator.cable_effort_status(),
                "shoulder_elbow_coupling": muscle_allocator.shoulder_coupling_status(),
                "elbow_cable_balance": muscle_allocator.elbow_cable_balance_status(),
                "elbow_balance_horizon_seconds": muscle_allocator.cable_return_horizon,
                "elbow_balance_max_extra_pwm": muscle_allocator.cable_return_max_pwm,
                "following_triceps_wind_limit": FOLLOWING_TRICEPS_WIND_LIMIT,
                "command_filter_tau": muscle_allocator.command_filter_tau,
                "wind_slew_rate": muscle_allocator.wind_slew_rate,
                "release_slew_rate": muscle_allocator.release_slew_rate,
                "reverse_deadtime": muscle_allocator.reverse_deadtime,
                "imu_following": True,
                "controller_modes": ["pid", "adrc", "ilc_pid", "ilc_adrc", "following_only", "trajectory_demo"],
                "feedforward_gain": 0.0,
                "feedforward_min_pwm": getattr(sensor, "assist_feedforward_min_pwm", 10.0) if sensor is not None else 10.0,
                "following_velocity_threshold": getattr(sensor, "following_velocity_threshold", 5.0) if sensor is not None else 5.0,
                "event_following": {
                    "sample_velocity_threshold": getattr(sensor, "following_intent_velocity_threshold", 5.0) if sensor is not None else 5.0,
                    "sample_duration_seconds": getattr(sensor, "following_intent_confirm_time", 0.04) if sensor is not None else 0.04,
                    "minimum_angle_degrees": getattr(sensor, "following_intent_min_angle", 0.4) if sensor is not None else 0.4,
                    "hold_duration_seconds": getattr(sensor, "following_pulse_duration", 0.20) if sensor is not None else 0.20,
                    "blanking_duration_seconds": getattr(sensor, "following_blanking_duration", 0.04) if sensor is not None else 0.04,
                    "rearm_velocity_threshold": getattr(sensor, "following_rearm_velocity", 3.0) if sensor is not None else 3.0,
                    "rearm_time_seconds": getattr(sensor, "following_rearm_time", 0.10) if sensor is not None else 0.10,
                    "maximum_session_seconds": getattr(sensor, "following_max_session_duration", 3.0) if sensor is not None else 3.0,
                    "safety_velocity": getattr(sensor, "following_safety_velocity", 80.0) if sensor is not None else 80.0,
                },
                "trajectory_demo": True,
                "demo_output_limit": getattr(sensor, "demo_output_limit", 100.0) if sensor is not None else 100.0,
                "adrc": True,
                "adrc_controller_bandwidth": getattr(sensor, "adrc_controller_bandwidth", 2.0) if sensor is not None else 2.0,
                "adrc_observer_bandwidth": getattr(sensor, "adrc_observer_bandwidth", 8.0) if sensor is not None else 8.0,
                "ilc": True,
                "shoulder_release_max_pwm": 60.0,
                "live_output_allowed": True,
                "encoder_safety_enabled": False,
                "patient_rom_active": bool(getattr(sensor, "patient_rom", None)) if sensor is not None else False,
                "rom_soft_zone_deg": getattr(sensor, "rom_soft_zone_deg", 5.0) if sensor is not None else 5.0,
                "rom_hysteresis_deg": getattr(sensor, "rom_hysteresis_deg", 3.0) if sensor is not None else 3.0,
                "lock_reason": "",
            },
            "rom_calibration": rom_calibration.status(),
            "frequency_identification": frequency_session.status(),
            "ilc": sensor.ilc.status() if sensor is not None and hasattr(sensor, "ilc") else {"enabled": False},
            "frame": serialize_frame(latest_frame),
        })

@app.route("/api/rom/start", methods=["POST"])
def api_rom_start():
    data = request.get_json(silent=True) or {}
    with state_lock:
        if sensor is None or not initialized:
            return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
        sensor.motor_enabled = False
        if frequency_session.status().get("state") == "running":
            frequency_session.stop("開始 ROM 記錄。", aborted=True)
        rom_calibration.start(data.get("duration_seconds", 10.0))
    stop_motor_bridge_safely()
    set_status("病患個人化 ROM 記錄已開始；記錄期間馬達維持停止。")
    return jsonify({"ok": True, "message": "ROM 已重設，即將從手臂下垂的最大肘屈曲開始。", "rom_calibration": rom_calibration.status()})

@app.route("/api/rom/start_stage", methods=["POST"])
def api_rom_start_stage():
    data = request.get_json(silent=True) or {}
    try:
        with state_lock:
            if sensor is None or not initialized:
                return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
            sensor.motor_enabled = False
            status = rom_calibration.start_stage(data.get("stage"))
        stop_motor_bridge_safely()
        return jsonify({"ok": True, "message": f"已開始記錄：{status['current_stage_label']}。", "rom_calibration": status})
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/rom/finish_stage", methods=["POST"])
def api_rom_finish_stage():
    try:
        status = rom_calibration.finish_stage()
        completed = list(status.get("results", {}).keys())[-1]
        return jsonify({
            "ok": True,
            "message": f"已記錄：{ROMCalibrationSession.STAGE_LABELS[completed]}",
            "rom_calibration": status,
        })
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/rom/apply", methods=["POST"])
def api_rom_apply():
    try:
        with state_lock:
            if sensor is None or not initialized:
                return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
            mapping = rom_calibration.apply()
            sensor.apply_rom_calibration(
                elbow_axis=mapping["elbow_axis"],
                elbow_sign=mapping["elbow_sign"],
                front_reference=mapping["front_reference"],
                side_reference=mapping["side_reference"],
                rom=mapping["rom"],
            )
            sensor.trajectory_min_angle = mapping["rom"]["elbow_extension_target_deg"]
            sensor.trajectory_max_angle = mapping["rom"]["elbow_flexion_target_deg"]
            sensor.fixed_target_angle = mapping["rom"]["elbow_flexion_target_deg"]
        return jsonify({
            "ok": True,
            "message": f"個人化 ROM 已套用：肘關節 {mapping['elbow_axis']} 軸、方向 {mapping['elbow_sign']:+.0f}，肘訓練範圍已設為 90%。",
            "rom": mapping["rom"],
            "rom_calibration": rom_calibration.status(),
        })
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/rom/reset", methods=["POST"])
def api_rom_reset():
    with state_lock:
        if sensor is not None:
            sensor.motor_enabled = False
        rom_calibration.reset()
    stop_motor_bridge_safely()
    return jsonify({"ok": True, "message": "病患個人化 ROM 已重設。", "rom_calibration": rom_calibration.status()})


@app.route("/api/frequency/start", methods=["POST"])
def api_frequency_start():
    data = request.get_json(force=True) or {}
    if not safe_bool(data.get("confirmed_rig", False)):
        return jsonify({
            "ok": False,
            "message": "尚未確認空載／固定測試架，禁止啟動系統識別。",
        }), 403
    try:
        config = frequency_session.validate_config(data)
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

    with state_lock:
        if sensor is None or not initialized or latest_frame is None:
            return jsonify({"ok": False, "message": "請先初始化、校正 IMU 並等待即時角度。"}), 400
        if imu_safety_fault_latched:
            return jsonify({
                "ok": False,
                "message": "IMU 安全鎖已啟動，禁止系統識別輸出；請先確認姿勢與繩索後解除。",
            }), 409
        if frequency_session.status().get("state") == "running":
            return jsonify({"ok": False, "message": "系統識別已在執行。"}), 409
        if sensor.motor_enabled:
            return jsonify({"ok": False, "message": "請先 Disable Motor，再由系統識別模式自行 ARM。"}), 409
        initial_angle = (
            latest_frame.get("shoulder_angle", 0.0)
            if config["target_joint"] == "shoulder"
            else latest_frame.get("elbow_angle", 0.0)
        )

    if not connect_motor_bridge_if_needed():
        return jsonify({
            "ok": False,
            "message": f"ESP32 連線失敗：{motor_bridge_error}。馬達維持停止。",
        }), 503
    try:
        motor_bridge.stop()
        motor_bridge.arm()
        with state_lock:
            sensor.target_joint = config["target_joint"]
            sensor.emergency_stop = False
            muscle_allocator.reset_conditioner()
            status = frequency_session.start(data, initial_angle=initial_angle)
            sensor.motor_enabled = True
    except Exception as exc:
        with state_lock:
            if sensor is not None:
                sensor.motor_enabled = False
            frequency_session.stop(f"啟動失敗：{exc}", aborted=True)
        stop_motor_bridge_safely()
        return jsonify({"ok": False, "message": f"系統識別啟動失敗：{exc}。馬達維持停止。"}), 503

    set_status("系統識別執行中；完成或異常時會自動停止馬達。")
    return jsonify({
        "ok": True,
        "message": "ESP32 已 ARM，系統識別開始。請保持急停可立即操作。",
        "frequency_identification": status,
    })


@app.route("/api/frequency/stop", methods=["POST"])
def api_frequency_stop():
    with state_lock:
        if sensor is not None:
            sensor.motor_enabled = False
        status = frequency_session.stop("使用者停止測試。", aborted=True)
        muscle_allocator.reset_conditioner()
        try:
            frequency_session.save_raw()
        except Exception as exc:
            frequency_session.reason += f" CSV 儲存失敗：{exc}"
    stop_motor_bridge_safely()
    set_status("系統識別已停止，馬達已送出 STOP。")
    return jsonify({
        "ok": True,
        "message": "系統識別已停止並送出 STOP；若樣本足夠仍可嘗試分析。",
        "frequency_identification": status,
    })


@app.route("/api/frequency/analyze", methods=["POST"])
def api_frequency_analyze():
    with state_lock:
        if sensor is not None:
            sensor.motor_enabled = False
        if frequency_session.status().get("state") == "running":
            return jsonify({"ok": False, "message": "測試仍在執行，請先停止。"}), 409
    stop_motor_bridge_safely()
    try:
        analysis = frequency_session.analyze()
    except (ValueError, OSError, RuntimeError) as exc:
        return jsonify({"ok": False, "message": f"頻域分析失敗：{exc}"}), 400
    return jsonify({
        "ok": True,
        "message": f"頻域分析完成。圖檔：{frequency_session.png_path}",
        "analysis": analysis,
        "frequency_identification": frequency_session.status(),
    })


@app.route("/api/ilc/reset", methods=["POST"])
def api_ilc_reset():
    with state_lock:
        if sensor is None or not hasattr(sensor, "ilc"):
            return jsonify({"ok": False, "message": "請先初始化 IMU 控制系統。"}), 400
        if sensor.motor_enabled:
            return jsonify({"ok": False, "message": "請先 Disable Motor，再清除 ILC 學習曲線。"}), 409
        sensor.ilc.clear_learning()
        return jsonify({"ok": True, "message": "ILC learned feedforward 與周期統計已清除。", "ilc": sensor.ilc.status()})


@app.route("/api/ilc/freeze", methods=["POST"])
def api_ilc_freeze():
    data = request.get_json(force=True) or {}
    frozen = safe_bool(data.get("frozen", True))
    with state_lock:
        if sensor is None or not hasattr(sensor, "ilc"):
            return jsonify({"ok": False, "message": "請先初始化 IMU 控制系統。"}), 400
        sensor.ilc.set_frozen(frozen)
        return jsonify({
            "ok": True,
            "message": "ILC 學習已凍結；保留目前曲線並繼續評估。" if frozen else "ILC 學習已恢復。",
            "ilc": sensor.ilc.status(),
        })


@app.route("/api/ilc/status")
def api_ilc_status():
    with state_lock:
        if sensor is None or not hasattr(sensor, "ilc"):
            return jsonify({"ok": False, "message": "請先初始化 IMU 控制系統。"}), 400
        return jsonify({"ok": True, "ilc": sensor.ilc.status(include_curve=True)})

@app.route("/api/apply_params", methods=["POST"])
def api_apply_params():
    data = dict(request.get_json(force=True) or {})
    data["controller_mode"] = normalize_controller_mode(data.get("controller_mode", "pid"))
    if str(data.get("controller_mode", "")).lower() == "trajectory_demo" and str(data.get("target_mode", "")).lower() != "sine":
        return jsonify({"ok": False, "message": "開迴路軌跡展示模式只允許正弦軌跡；請改選正弦軌跡。"}), 400
    if str(data.get("controller_mode", "")).lower() in ("adrc", "feedforward_adrc", "ilc_adrc") and abs(safe_float(data.get("adrc_input_gain"), 1.0)) < 0.01:
        return jsonify({"ok": False, "message": "ADRC 輸入增益 b0 的絕對值必須至少為 0.01。"}), 400
    if str(data.get("controller_mode", "")).lower() in ("ilc_pid", "ilc_adrc") and str(data.get("target_mode", "")).lower() != "sine":
        return jsonify({"ok": False, "message": "ILC 只允許固定週期的正弦軌跡。"}), 400
    with state_lock:
        if sensor is None:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        if frequency_session.status().get("state") == "running":
            return jsonify({"ok": False, "message": "系統識別執行中，不能變更控制參數。"}), 409
        apply_params_to_sensor(data, reset_controller=True)
        muscle_allocator.reset_conditioner()
    return jsonify({
        "ok": True,
        "message": "目標關節、復健軌跡與控制參數已套用，控制器狀態已重置。",
        "applied": current_control_settings(),
    })


def motor_test_worker(motor, direction):
    global motor_test_running
    commands = {
        "biceps": (direction * MOTOR_TEST_PWM, 0, 0),
        "triceps": (0, direction * MOTOR_TEST_PWM, 0),
        "deltoid": (0, 0, direction * MOTOR_TEST_PWM),
    }
    command = commands[motor]
    started = time.monotonic()
    try:
        while motor_test_running and time.monotonic() - started < MOTOR_TEST_DURATION:
            send_motor_pwm_with_locks(*command)
            time.sleep(0.05)
    except Exception as exc:
        set_status(f"單顆馬達測試失敗：{exc}")
    finally:
        try:
            if motor_bridge is not None:
                motor_bridge.stop()
        finally:
            motor_test_running = False
        set_status("單顆馬達測試結束，已自動送出 STOP。")


def motor_jog_worker(session_id, motor, direction, pwm):
    global motor_test_running, motor_jog_active
    command_map = {
        "biceps": (direction * pwm, 0, 0),
        "triceps": (0, direction * pwm, 0),
        "deltoid": (0, 0, direction * pwm),
    }
    command = command_map[motor]
    stop_reason = "按鈕已放開"
    try:
        while motor_test_running and motor_jog_active and session_id == motor_jog_session:
            if time.monotonic() - motor_jog_last_heartbeat > MOTOR_JOG_HEARTBEAT_TIMEOUT:
                stop_reason = "瀏覽器心跳逾時"
                break
            send_motor_pwm_with_locks(*command)
            time.sleep(0.05)
    except Exception as exc:
        stop_reason = f"通訊錯誤：{exc}"
    finally:
        if session_id == motor_jog_session:
            try:
                if motor_bridge is not None:
                    motor_bridge.stop()
            finally:
                motor_test_running = False
                motor_jog_active = False
                set_status(f"單顆馬達點動已 STOP（{stop_reason}）。")


@app.route("/api/motor_locks", methods=["POST"])
def api_motor_locks():
    global motor_test_running, motor_jog_active, motor_jog_session
    data = request.get_json(force=True) or {}
    motor = str(data.get("motor", "")).lower()
    if motor not in motor_channel_locks:
        return jsonify({"ok": False, "message": "馬達通道無效。", "locks": dict(motor_channel_locks)}), 400
    locked = safe_bool(data.get("locked", False))
    with state_lock:
        motor_channel_locks[motor] = locked
        stop_required = bool(
            locked and motor_jog_active and motor_jog_motor == motor
        )
        if stop_required:
            motor_jog_session += 1
            motor_test_running = False
            motor_jog_active = False
    if stop_required:
        stop_motor_bridge_safely()
    labels = {"biceps": "二頭肌", "triceps": "三頭肌", "deltoid": "三角肌"}
    action = "已鎖定" if locked else "已解除鎖定"
    suffix = "，進行中的點動已立即 STOP" if stop_required else ""
    message = f"{labels[motor]}馬達全系統輸出{action}{suffix}。"
    set_status(message)
    return jsonify({"ok": True, "message": message, "locks": dict(motor_channel_locks)})


@app.route("/api/motor_output_scales", methods=["POST"])
def api_motor_output_scales():
    """Update persistent, reduction-only physical output calibration."""
    data = request.get_json(silent=True) or {}
    percentages = data.get("percentages") or {}
    names = ("biceps", "triceps", "deltoid")
    try:
        requested = [
            min(100.0, max(10.0, float(percentages[name])))
            for name in names
        ]
    except (KeyError, TypeError, ValueError):
        return jsonify({
            "ok": False,
            "message": "請輸入二頭肌、三頭肌、三角肌三組 10–100% 比例。",
        }), 400

    with state_lock:
        output_active = bool(
            motor_arm_in_progress
            or motor_test_running
            or (sensor is not None and getattr(sensor, "motor_enabled", False))
            or frequency_session.status().get("state") == "running"
        )
        if output_active:
            return jsonify({
                "ok": False,
                "message": "請先 Disable Motor 並停止點動／測試，再套用全局輸出比例。",
                "percentages": dict(zip(
                    names, [round(value * 100.0, 1) for value in MOTOR_OUTPUT_SCALE]
                )),
            }), 409
        previous = list(MOTOR_OUTPUT_SCALE)
        MOTOR_OUTPUT_SCALE[:] = [value / 100.0 for value in requested]
        try:
            save_motor_output_scale_config()
        except OSError as exc:
            MOTOR_OUTPUT_SCALE[:] = previous
            return jsonify({
                "ok": False,
                "message": f"比例設定檔儲存失敗：{exc}",
            }), 500

    stop_motor_bridge_safely()
    applied = {
        name: round(value * 100.0, 1)
        for name, value in zip(names, MOTOR_OUTPUT_SCALE)
    }
    message = (
        "全局輸出比例已套用並保存："
        f"二頭 {applied['biceps']:g}%／"
        f"三頭 {applied['triceps']:g}%／"
        f"三角 {applied['deltoid']:g}%。ESP32 已保持 STOP。"
    )
    set_status(message)
    return jsonify({"ok": True, "message": message, "percentages": applied})


@app.route("/api/cable_settings", methods=["POST"])
def api_cable_settings():
    """Apply every user-facing winding/payout setting to the live allocator."""
    global FOLLOWING_TRICEPS_WIND_LIMIT
    data = request.get_json(silent=True) or {}
    raw_settings = data.get("settings") or {}
    raw_percentages = data.get("percentages") or {}
    try:
        requested = {
            name: min(maximum, max(minimum, float(raw_settings[name])))
            for name, (minimum, maximum) in CABLE_SETTING_LIMITS.items()
        }
        percentages = {
            name: min(100.0, max(10.0, float(raw_percentages[name])))
            for name in ("biceps", "triceps", "deltoid")
        }
    except (KeyError, TypeError, ValueError):
        return jsonify({
            "ok": False,
            "message": "收放線設定不完整或不是有效數字，未套用任何變更。",
        }), 400

    requested["cable_return_gain"] = max(
        requested["cable_return_gain"], requested["antagonist_release_gain"]
    )
    with state_lock:
        output_active = bool(
            motor_arm_in_progress
            or motor_test_running
            or motor_jog_active
            or (sensor is not None and getattr(sensor, "motor_enabled", False))
            or frequency_session.status().get("state") == "running"
        )
        if output_active:
            return jsonify({
                "ok": False,
                "message": "請先 Disable Motor 並停止點動／測試，再套用收放線設定。",
                "settings": current_cable_settings(),
                "percentages": dict(zip(
                    ("biceps", "triceps", "deltoid"),
                    [round(value * 100.0, 1) for value in MOTOR_OUTPUT_SCALE],
                )),
            }), 409

        previous_settings = current_cable_settings()
        previous_scales = list(MOTOR_OUTPUT_SCALE)
        CABLE_SETTINGS.clear()
        CABLE_SETTINGS.update(requested)
        muscle_allocator.antagonist_release_gain = requested["antagonist_release_gain"]
        muscle_allocator.triceps_release_ratio = requested["triceps_release_ratio"]
        muscle_allocator.biceps_release_ratio = requested["biceps_release_ratio"]
        muscle_allocator.cable_return_gain = requested["cable_return_gain"]
        muscle_allocator.cable_return_horizon = requested["cable_return_horizon"]
        muscle_allocator.cable_return_max_pwm = requested["cable_return_max_pwm"]
        muscle_allocator.shoulder_elbow_coupling_pwm = requested["shoulder_elbow_coupling_pwm"]
        muscle_allocator.shoulder_coupling_rewind_ratio = requested["shoulder_coupling_rewind_ratio"]
        muscle_allocator.wind_slew_rate = requested["wind_slew_rate"]
        muscle_allocator.release_slew_rate = requested["release_slew_rate"]
        muscle_allocator.elbow_soft_landing_target = requested["elbow_soft_landing_target"]
        muscle_allocator.elbow_soft_landing_zone = requested["elbow_soft_landing_zone"]
        muscle_allocator.elbow_soft_landing_min_ratio = requested["elbow_soft_landing_min_ratio"]
        FOLLOWING_TRICEPS_WIND_LIMIT = requested["following_triceps_wind_limit"]
        MOTOR_OUTPUT_SCALE[:] = [
            percentages[name] / 100.0
            for name in ("biceps", "triceps", "deltoid")
        ]
        try:
            save_cable_settings_config()
            save_motor_output_scale_config()
        except OSError as exc:
            CABLE_SETTINGS.clear()
            CABLE_SETTINGS.update(previous_settings)
            muscle_allocator.antagonist_release_gain = previous_settings["antagonist_release_gain"]
            muscle_allocator.triceps_release_ratio = previous_settings["triceps_release_ratio"]
            muscle_allocator.biceps_release_ratio = previous_settings["biceps_release_ratio"]
            muscle_allocator.cable_return_gain = previous_settings["cable_return_gain"]
            muscle_allocator.cable_return_horizon = previous_settings["cable_return_horizon"]
            muscle_allocator.cable_return_max_pwm = previous_settings["cable_return_max_pwm"]
            muscle_allocator.shoulder_elbow_coupling_pwm = previous_settings["shoulder_elbow_coupling_pwm"]
            muscle_allocator.shoulder_coupling_rewind_ratio = previous_settings["shoulder_coupling_rewind_ratio"]
            muscle_allocator.wind_slew_rate = previous_settings["wind_slew_rate"]
            muscle_allocator.release_slew_rate = previous_settings["release_slew_rate"]
            muscle_allocator.elbow_soft_landing_target = previous_settings["elbow_soft_landing_target"]
            muscle_allocator.elbow_soft_landing_zone = previous_settings["elbow_soft_landing_zone"]
            muscle_allocator.elbow_soft_landing_min_ratio = previous_settings["elbow_soft_landing_min_ratio"]
            FOLLOWING_TRICEPS_WIND_LIMIT = previous_settings["following_triceps_wind_limit"]
            MOTOR_OUTPUT_SCALE[:] = previous_scales
            return jsonify({
                "ok": False,
                "message": f"收放線設定檔儲存失敗：{exc}",
            }), 500
        muscle_allocator.reset_conditioner()
        muscle_allocator.reset_cable_tracker()

    stop_motor_bridge_safely()
    applied_percentages = {
        name: round(value * 100.0, 1)
        for name, value in zip(("biceps", "triceps", "deltoid"), MOTOR_OUTPUT_SCALE)
    }
    message = "收放線比例、平滑參數與全局輸出已實際套用並保存；ESP32 保持 STOP。"
    set_status(message)
    return jsonify({
        "ok": True,
        "message": message,
        "settings": current_cable_settings(),
        "percentages": applied_percentages,
    })


@app.route("/api/motor_jog/start", methods=["POST"])
def api_motor_jog_start():
    global motor_test_running, motor_jog_active, motor_jog_session
    global motor_jog_last_heartbeat, motor_jog_motor, motor_jog_direction, motor_jog_pwm
    data = request.get_json(force=True) or {}
    motor = str(data.get("motor", "")).lower()
    try:
        direction = int(data.get("direction", 0))
        pwm = int(round(float(data.get("pwm", MOTOR_JOG_DEFAULT_PWM))))
    except (TypeError, ValueError):
        direction, pwm = 0, 0
    pwm = min(max(pwm, 1), MOTOR_JOG_MAX_PWM)
    if not safe_bool(data.get("confirmed", False)):
        return jsonify({"ok": False, "message": "請先確認已 Disable Motor 且沒有人體負載。"}), 403
    if motor not in ("biceps", "triceps", "deltoid") or direction not in (-1, 1):
        return jsonify({"ok": False, "message": "馬達通道或方向無效。"}), 400
    with state_lock:
        if motor_channel_locks.get(motor, False):
            return jsonify({
                "ok": False,
                "message": f"{motor} 馬達已被全系統鎖定，請先解除該馬達安全鎖。",
                "locks": dict(motor_channel_locks),
            }), 409
        if sensor is not None and sensor.motor_enabled:
            return jsonify({"ok": False, "message": "請先 Disable Motor，才能使用單顆馬達點動。"}), 409
        software_estop = bool(sensor is not None and sensor.emergency_stop)
        hardware_estop = bool(
            motor_bridge is not None
            and getattr(motor_bridge, "safety_state", "") in ("ESTOP", "EMERGENCY_STOP")
        )
        if software_estop or hardware_estop:
            return jsonify({"ok": False, "message": "急停尚未解除，不能啟動點動。"}), 409
        if frequency_session.status().get("state") == "running" or motor_test_running:
            return jsonify({"ok": False, "message": "已有馬達輸出或測試正在執行。"}), 409
        motor_test_running = True
        motor_jog_active = True
        motor_jog_session += 1
        session_id = motor_jog_session
        motor_jog_last_heartbeat = time.monotonic()
        motor_jog_motor = motor
        motor_jog_direction = direction
        motor_jog_pwm = pwm
    if not connect_motor_bridge_if_needed():
        motor_test_running = False
        motor_jog_active = False
        return jsonify({"ok": False, "message": f"ESP32 連線失敗：{motor_bridge_error}"}), 503
    try:
        motor_bridge.stop()
        motor_bridge.arm()
    except Exception as exc:
        motor_test_running = False
        motor_jog_active = False
        stop_motor_bridge_safely()
        return jsonify({"ok": False, "message": f"ESP32 ARM 失敗：{exc}"}), 503
    with state_lock:
        start_cancelled = bool(
            session_id != motor_jog_session
            or not motor_test_running
            or not motor_jog_active
        )
    if start_cancelled:
        stop_motor_bridge_safely()
        return jsonify({"ok": False, "message": "點動啟動期間已放開按鈕，輸出維持 STOP。"}), 409
    threading.Thread(
        target=motor_jog_worker,
        args=(session_id, motor, direction, pwm),
        daemon=True,
    ).start()
    set_status(
        f"單顆馬達點動中：{motor} "
        f"{'正轉' if direction > 0 else '反轉'}，{pwm} PWM。"
    )
    return jsonify({
        "ok": True,
        "message": f"{motor} {'正轉' if direction > 0 else '反轉'} 點動中：{pwm} PWM。",
        "pwm": pwm,
    })


@app.route("/api/motor_jog/heartbeat", methods=["POST"])
def api_motor_jog_heartbeat():
    global motor_jog_last_heartbeat
    if not motor_test_running or not motor_jog_active:
        return jsonify({"ok": False, "message": "目前沒有進行馬達點動。"}), 409
    motor_jog_last_heartbeat = time.monotonic()
    return jsonify({"ok": True})


@app.route("/api/motor_jog/stop", methods=["POST"])
def api_motor_jog_stop():
    global motor_test_running, motor_jog_active, motor_jog_session
    motor_jog_session += 1
    motor_test_running = False
    motor_jog_active = False
    stop_motor_bridge_safely()
    set_status("單顆馬達點動已由使用者 STOP。")
    return jsonify({"ok": True, "message": "單顆馬達點動已 STOP。"})


@app.route("/api/motor_test/pulse", methods=["POST"])
def api_motor_test_pulse():
    global motor_test_running
    data = request.get_json(force=True) or {}
    motor = str(data.get("motor", "")).lower()
    try:
        raw_direction = int(data.get("direction", 0))
        direction = raw_direction if raw_direction in (-1, 1) else 0
    except (TypeError, ValueError):
        direction = 0
    if not safe_bool(data.get("confirmed_rig", False)):
        return jsonify({"ok": False, "message": "尚未確認固定測試架與無人體負載，馬達維持停止。"}), 403
    if motor not in ("biceps", "triceps", "deltoid") or direction == 0:
        return jsonify({"ok": False, "message": "馬達通道或方向無效。"}), 400
    with state_lock:
        if motor_channel_locks.get(motor, False):
            return jsonify({
                "ok": False,
                "message": f"{motor} 馬達已被全系統鎖定，不能執行單顆測試。",
            }), 409
        if sensor is None or not initialized:
            return jsonify({"ok": False, "message": "請先初始化系統，再執行單顆馬達測試。"}), 400
        if imu_safety_fault_latched:
            return jsonify({
                "ok": False,
                "message": "IMU 安全鎖已啟動；只能使用按住點動整理繩索，禁止脈衝測試。",
            }), 409
        if sensor.motor_enabled:
            return jsonify({"ok": False, "message": "請先 Disable Motor，才能執行單顆測試。"}), 409
        if sensor.emergency_stop:
            return jsonify({"ok": False, "message": "急停尚未解除，不能執行單顆測試。"}), 409
        if frequency_session.status().get("state") == "running" or motor_test_running:
            return jsonify({"ok": False, "message": "已有馬達測試正在執行。"}), 409
        motor_test_running = True
    if not connect_motor_bridge_if_needed():
        motor_test_running = False
        return jsonify({"ok": False, "message": f"ESP32 連線失敗：{motor_bridge_error}"}), 503
    try:
        motor_bridge.stop()
        motor_bridge.arm()
    except Exception as exc:
        motor_test_running = False
        stop_motor_bridge_safely()
        return jsonify({"ok": False, "message": f"ESP32 ARM 失敗：{exc}"}), 503
    threading.Thread(
        target=motor_test_worker,
        args=(motor, direction),
        daemon=True,
    ).start()
    direction_label = "正向＋" if direction > 0 else "反向－"
    return jsonify({
        "ok": True,
        "message": f"{motor} {direction_label} 測試開始：{MOTOR_TEST_PWM} PWM、{MOTOR_TEST_DURATION:.1f} 秒後自動 STOP。",
    })

@app.route("/api/set_motor", methods=["POST"])
def api_set_motor():
    global motor_test_running, motor_arm_in_progress, motor_arm_session
    data = request.get_json(force=True) or {}
    enabled = safe_bool(data.get("enabled", False))
    confirmed = safe_bool(data.get("confirmed", False))
    arm_session = None

    if motor_test_running:
        if enabled:
            return jsonify({"ok": False, "message": "單顆馬達測試執行中，不能 Enable Motor。"}), 409
        motor_test_running = False
        stop_motor_bridge_safely()

    with state_lock:
        if sensor is None:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        if enabled and imu_safety_fault_latched:
            sensor.motor_enabled = False
            return jsonify({
                "ok": False,
                "message": (
                    "IMU 安全鎖已啟動，禁止 Enable Motor。"
                    "請先確認病患姿勢與繩索，再按『解除 IMU 安全鎖』。"
                ),
            }), 409
        if enabled and frequency_session.status().get("state") == "running":
            return jsonify({"ok": False, "message": "系統識別已自行管理馬達輸出，不能重複 Enable。"}), 409
        if enabled and not confirmed:
            sensor.motor_enabled = False
            return jsonify({
                "ok": False,
                "message": "尚未確認啟動馬達，輸出維持停止。"
            }), 403
        if enabled and str(getattr(sensor, "controller_mode", "")).lower() == "trajectory_demo" and str(getattr(sensor, "target_mode", "")).lower() != "sine":
            sensor.motor_enabled = False
            return jsonify({"ok": False, "message": "展示模式只允許正弦軌跡，馬達維持停止。"}), 400
        if enabled and str(getattr(sensor, "controller_mode", "")).lower() in ("ilc_pid", "ilc_adrc") and str(getattr(sensor, "target_mode", "")).lower() != "sine":
            sensor.motor_enabled = False
            return jsonify({"ok": False, "message": "ILC 只允許正弦軌跡，馬達維持停止。"}), 400
        if enabled:
            if motor_arm_in_progress:
                return jsonify({
                    "ok": False,
                    "message": "ESP32 ARM 確認已在進行，請勿重複按下 Enable。",
                }), 409
            motor_arm_session += 1
            arm_session = motor_arm_session
            motor_arm_in_progress = True
            sensor.motor_enabled = False
        else:
            # Invalidate any in-flight ARM request before issuing STOP.
            motor_arm_session += 1
            motor_arm_in_progress = False
            sensor.motor_enabled = False

    if enabled:
        ok = connect_motor_bridge_if_needed()
        if not ok:
            with state_lock:
                if arm_session == motor_arm_session:
                    motor_arm_in_progress = False
                    sensor.motor_enabled = False
            msg = (
                f"ESP32 連線失敗：{motor_bridge_error}。請確認 USB 線、"
                f"port={ESP32_PORT}、ESP32 程式已燒錄。"
            )
            set_status(msg)
            return jsonify({
                "ok": False,
                "message": msg,
            }), 503
        try:
            # Keep STOP -> ARM atomic while reader_loop is held by the
            # motor_arm_in_progress output gate.
            motor_bridge.prepare_active(wait_timeout=1.5)
        except Exception as exc:
            with state_lock:
                if arm_session == motor_arm_session:
                    motor_arm_in_progress = False
                    sensor.motor_enabled = False
            stop_motor_bridge_safely()
            msg = f"ESP32 ARM 確認失敗：{exc}。馬達維持停止。"
            set_status(msg)
            return jsonify({
                "ok": False,
                "message": msg,
            }), 503

        with state_lock:
            cancelled = (
                arm_session != motor_arm_session
                or imu_safety_fault_latched
            )
            if not cancelled:
                sensor.motor_enabled = True
                motor_arm_in_progress = False
            else:
                motor_arm_in_progress = False
        if cancelled:
            stop_motor_bridge_safely()
            msg = "ESP32 ARM 已被後續 STOP／急停取消，馬達維持停止。"
            set_status(msg)
            return jsonify({"ok": False, "message": msg}), 409

        with state_lock:
            sensor.emergency_stop = False
            if hasattr(sensor, "reset_trajectory"):
                sensor.reset_trajectory()
            msg = "Motor enabled：PWM 會經由 USB Serial 輸出到 ESP32。展示模式會先在最小角度停留 1 秒。" if str(getattr(sensor, "controller_mode", "")).lower() == "trajectory_demo" else "Motor enabled：PWM 會經由 USB Serial 輸出到 ESP32。"
    else:
        with state_lock:
            if frequency_session.status().get("state") == "running":
                frequency_session.stop("由馬達停用按鍵停止。", aborted=True)
                try:
                    frequency_session.save_raw()
                except Exception:
                    pass
            if hasattr(sensor, "stop_trajectory"):
                sensor.stop_trajectory()
            msg = "Motor disabled：已送 STOP 給 ESP32。"
        stop_motor_bridge_safely()

    set_status(msg)
    return jsonify({"ok": True, "message": msg})


@app.route("/api/imu_fault/clear", methods=["POST"])
def api_clear_imu_fault():
    global imu_safety_fault_latched, imu_safety_fault_reason, imu_safety_fault_time
    global imu_recovery_stable_samples, imu_recovery_ready, imu_last_valid_angles
    data = request.get_json(force=True) or {}
    if not safe_bool(data.get("confirmed", False)):
        return jsonify({
            "ok": False,
            "message": "尚未確認姿勢與繩索，IMU 安全鎖維持。",
        }), 403
    with state_lock:
        if not imu_safety_fault_latched:
            return jsonify({"ok": True, "message": "IMU 安全鎖目前未啟動。"})
        if sensor is None or not initialized:
            return jsonify({
                "ok": False,
                "message": "IMU 尚未初始化，不能解除安全鎖。",
            }), 409
        if not imu_channels_connected() or not imu_recovery_ready:
            return jsonify({
                "ok": False,
                "message": (
                    "IMU 尚未穩定恢復，安全鎖維持。"
                    f"目前有效樣本 {imu_recovery_stable_samples} / "
                    f"{IMU_RECOVERY_STABLE_SAMPLES_REQUIRED}。"
                ),
            }), 409
        sensor.motor_enabled = False
        if hasattr(sensor, "configure_control"):
            sensor.configure_control(reset_controller=True)
        if hasattr(sensor, "stop_trajectory"):
            sensor.stop_trajectory()
        muscle_allocator.reset_conditioner()
        # There is no motor encoder, so the pre-fault PWM-time estimate cannot
        # be verified.  After the operator inspects/retensions the tendons, use
        # that physical state as the new virtual cable reference.
        muscle_allocator.reset_cable_tracker()
        imu_safety_fault_latched = False
        imu_safety_fault_reason = ""
        imu_safety_fault_time = None
        imu_recovery_stable_samples = 0
        imu_recovery_ready = False
        imu_last_valid_angles = None
    stop_motor_bridge_safely()
    message = (
        "IMU 安全鎖已解除；目前繩索狀態已設為新的軟體基準。"
        "馬達仍為 Disable，需再次按 Enable Motor 才會輸出。"
    )
    set_status(message)
    return jsonify({"ok": True, "message": message})

@app.route("/api/emergency_stop", methods=["POST"])
def api_emergency_stop():
    global motor_test_running, motor_arm_in_progress, motor_arm_session
    motor_test_running = False
    with state_lock:
        motor_arm_session += 1
        motor_arm_in_progress = False
        if sensor is not None:
            sensor.emergency_stop = True
            sensor.motor_enabled = False
            if hasattr(sensor, "stop_trajectory"):
                sensor.stop_trajectory()
        if frequency_session.status().get("state") == "running":
            frequency_session.stop("Emergency Stop。", aborted=True)
    if motor_bridge is None or not motor_bridge_connected:
        stop_motor_bridge_safely()
        set_status("Emergency Stop：Pi 已停止輸出，但 ESP32 未連線，無法確認 ESTOP。")
        return jsonify({
            "ok": False,
            "message": "Pi 已停止所有馬達輸出；ESP32 未連線，因此無法確認硬體 ESTOP。",
        }), 503
    try:
        motor_bridge.emergency_stop()
    except Exception as exc:
        stop_motor_bridge_safely()
        set_status(f"Emergency Stop：Pi 已停止輸出；ESP32 ESTOP 確認失敗：{exc}")
        return jsonify({
            "ok": False,
            "message": f"Pi 已停止所有馬達輸出；ESP32 ESTOP 未確認：{exc}",
        }), 503
    set_status("Emergency Stop 已啟動，ESP32 已確認 ESTOP。")
    return jsonify({"ok": True, "message": "Emergency Stop 已啟動，ESP32 已確認 ESTOP。"})

@app.route("/api/clear_emergency", methods=["POST"])
def api_clear_emergency():
    with state_lock:
        if sensor is not None:
            sensor.motor_enabled = False
    stop_motor_bridge_safely()
    if not connect_motor_bridge_if_needed():
        return jsonify({
            "ok": False,
            "message": f"ESP32 未連線，急停保持鎖定：{motor_bridge_error}",
        }), 503
    try:
        motor_bridge.clear_fault()
    except Exception as exc:
        stop_motor_bridge_safely()
        return jsonify({
            "ok": False,
            "message": f"ESP32 CLEAR 未確認，急停保持鎖定：{exc}",
        }), 503
    with state_lock:
        if sensor is not None:
            sensor.emergency_stop = False
    set_status("ESP32 已確認 CLEAR；馬達維持 Disable，需重新按 Enable Motor。")
    return jsonify({"ok": True, "message": "急停已解除；ESP32 已確認 CLEAR。請重新按 Enable Motor。"})

@app.route("/api/start_recording", methods=["POST"])
def api_start_recording():
    global recording, csv_file, csv_writer, current_csv_path, record_count, participant_id, session_name, output_dir, current_label
    data = request.get_json(force=True) or {}
    with state_lock:
        if sensor is None or not initialized:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        if recording:
            return jsonify({"ok": False, "message": "已經在錄製中。"})
        participant_id = str(data.get("participant_id", "S01")).strip() or "S01"
        session_name = str(data.get("session_name", "session")).strip() or "session"
        current_label = str(data.get("label", "trial")).strip() or "trial"
        output_dir = str(data.get("output_dir", "control_data")).strip() or "control_data"
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        current_csv_path = os.path.abspath(os.path.join(output_dir, f"{participant_id}_{session_name}_{current_label}_{timestamp}.csv"))
        try:
            csv_file = open(current_csv_path, "w", newline="", encoding="utf-8-sig")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "wall_time","elapsed_time","participant_id","session","trial_label",
                "target_joint","target_mode","controller_mode",
                "target_angle","target_velocity","measured_angle","measured_angle_raw","tracking_error",
                "elbow_angle","elbow_angle_raw","shoulder_angle","shoulder_angle_raw",
                "error_for_control","integral_error","derivative_error","filtered_measurement_velocity","control_active","anti_windup_active",
                "raw_pid_output","feedback_pid_output","feedforward_output","combined_output","motor_cmd",
                "adrc_output","adrc_raw_output","adrc_estimated_angle","adrc_estimated_velocity","adrc_estimated_disturbance","adrc_observer_error","adrc_saturated",
                "ilc_output","ilc_current_cycle","ilc_last_rmse","ilc_improvement_percent","ilc_learned_peak_pwm","ilc_frozen",
                "desired_motor_cmd","desired_pwm_biceps","desired_pwm_triceps","desired_pwm_deltoid",
                "pwm_biceps","pwm_triceps","pwm_deltoid",
                "virtual_cable_effort_biceps","virtual_cable_effort_triceps","virtual_cable_effort_deltoid","elbow_soft_landing_scale",
                "encoder_count_biceps","encoder_count_triceps","encoder_count_deltoid",
                "encoder_vel_biceps","encoder_vel_triceps","encoder_vel_deltoid",
                "upper_angle","forearm_angle","elbow_velocity","elbow_acceleration","jerk",
                "shoulder_motion_state","shoulder_release_command","shoulder_release_pwm","shoulder_motor_enable",
                "motor_enabled","emergency_stop","motion_state"
            ])
            record_count = 0
            recording = True
        except Exception as e:
            return jsonify({"ok": False, "message": f"建立 CSV 失敗: {e}"})
    return jsonify({"ok": True, "message": "開始錄製。", "csv_path": current_csv_path})

@app.route("/api/stop_recording", methods=["POST"])
def api_stop_recording():
    global recording, csv_file, csv_writer
    with state_lock:
        if not recording:
            return jsonify({"ok": True, "message": "目前沒有在錄製。"})
        recording = False
        if csv_file is not None:
            csv_file.flush(); csv_file.close()
        csv_file = None
        csv_writer = None
    return jsonify({"ok": True, "message": f"錄製完成，共 {record_count} 筆。檔案位置: {current_csv_path}"})

if __name__ == "__main__":
    print("======================================")
    print("IMU 關節軌跡控制 Web GUI 啟動")
    print("瀏覽器開啟：http://樹莓派IP:5000")
    print("本機：http://127.0.0.1:5000")
    print("======================================")
    threading.Thread(target=imu_watchdog_loop, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)
