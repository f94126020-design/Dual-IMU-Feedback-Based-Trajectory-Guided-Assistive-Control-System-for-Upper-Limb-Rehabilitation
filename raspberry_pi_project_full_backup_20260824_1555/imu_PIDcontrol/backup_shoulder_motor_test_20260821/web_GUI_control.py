import csv
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
motor_bridge = None
motor_bridge_connected = False
motor_bridge_error = ""

muscle_allocator = MuscleAllocator(
    pwm_limit=MOTOR_PWM_LIMIT,
    active_min_pwm=0,
    antagonist_release_gain=0.8,
    shoulder_release_pwm=60,
    command_filter_tau=0.08,
    wind_slew_rate=240.0,
    release_slew_rate=300.0,
    reverse_deadtime=0.08,
    cable_return_gain=1.0,
    cable_effort_limit=5000.0,
    cable_return_threshold=1.0,
    cable_return_horizon=2.0,
    cable_return_max_pwm=60.0,
    # Motor 1: biceps, Motor 2: triceps, Motor 3: deltoid
    # If one motor direction is reversed, change its sign to -1.
    motor_sign=(1, 1, 1),
)

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
    .container{display:grid;grid-template-columns:390px 1fr;gap:16px;padding:16px}.card{background:white;border-radius:12px;padding:16px;box-shadow:0 2px 8px rgba(0,0,0,.08);margin-bottom:16px}.card h2{font-size:18px;margin:0 0 12px}label{display:block;font-size:13px;color:#374151;margin:7px 0 4px}input,select{width:100%;box-sizing:border-box;padding:8px;border:1px solid #cbd5e1;border-radius:8px}button{position:relative;width:100%;padding:10px;border:0;border-radius:8px;margin:5px 0;background:#2563eb;color:white;font-size:15px;cursor:pointer;transition:transform .1s,filter .15s,box-shadow .15s}button:hover{filter:brightness(.92);box-shadow:0 3px 8px rgba(15,23,42,.18)}button:active,button.button-pressed{transform:scale(.97);filter:brightness(.82)}button:disabled{background:#9ca3af;cursor:not-allowed;transform:none;box-shadow:none}button.button-busy{cursor:wait;animation:buttonPulse .8s ease-in-out infinite alternate}button.button-success{background:#16a34a!important}button.button-error{background:#dc2626!important}.secondary{background:#64748b}.success{background:#16a34a}.danger{background:#dc2626}.warning{background:#f59e0b;color:#111827}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.status{background:#eef2ff;border-left:5px solid #4f46e5;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:14px}.validation{background:#f8fafc;border:1px solid #d1d5db;border-radius:8px;padding:10px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.5}.protocol{background:#fff7ed;border:1px solid #fdba74;border-radius:8px;padding:11px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.55}.progress-track{height:10px;background:#e5e7eb;border-radius:999px;overflow:hidden;margin:8px 0}.progress-bar{height:100%;width:0;background:#2563eb;transition:width .15s linear}.live-angle{background:#ecfeff;border:1px solid #67e8f9;border-radius:8px;padding:9px;margin:8px 0;white-space:pre-wrap;font-family:Consolas,monospace;font-size:12px}.pass{color:#15803d;font-weight:700}.fail{color:#b91c1c;font-weight:700}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.metric{background:#f8fafc;border:1px solid #e5e7eb;border-radius:10px;padding:12px}.metric-title{font-size:13px;color:#64748b}.metric-value{font-size:22px;font-weight:700;margin-top:6px}.small{font-size:13px;color:#64748b;line-height:1.45}.rec{color:#dc2626;font-weight:700}.idle{color:#64748b;font-weight:700}.parameter-panel{display:none;margin-top:10px;padding:12px;border:1px solid #dbeafe;border-radius:10px;background:#f8fbff}.parameter-panel.active{display:block;animation:panelIn .18s ease-out}.parameter-title{font-size:14px;font-weight:700;color:#1e3a8a;margin-bottom:6px}.mode-summary{margin-top:9px;padding:9px;border-radius:8px;background:#eff6ff;color:#1e40af;font-size:13px;line-height:1.45}.action-toast{position:fixed;right:18px;bottom:18px;z-index:1000;max-width:360px;padding:12px 16px;border-radius:10px;background:#1e293b;color:white;box-shadow:0 8px 24px rgba(15,23,42,.28);opacity:0;transform:translateY(14px);pointer-events:none;transition:.2s}.action-toast.show{opacity:1;transform:translateY(0)}.action-toast.success{background:#15803d}.action-toast.error{background:#b91c1c}.action-toast.info{background:#1e40af}canvas{width:100%;height:430px;background:white;border:1px solid #e5e7eb;border-radius:10px}@keyframes panelIn{from{opacity:0;transform:translateY(-5px)}to{opacity:1;transform:none}}@keyframes buttonPulse{from{filter:brightness(.85)}to{filter:brightness(1.1)}}@media(max-width:900px){.container{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}}
  </style>
</head>
<body>
<header>
  <h1>雙 IMU 目標軌跡導引式上肢復健控制系統</h1>
  <p>雙 IMU 肘／肩關節控制、個人化 ROM 與 ESP32 即時馬達安全節點</p>
</header>
<div class="container">
  <div>
    <div class="card">
      <h2>1. 初始化</h2>
      <button onclick="initializeSystem()">初始化並校正 IMU</button>
      <button class="secondary" onclick="refreshStatus()">更新狀態</button>
      <div class="status" id="statusBox">尚未初始化</div>
    </div>

    <div class="card">
      <h2>2. 病患個人化活動範圍 ROM</h2>
      <p class="small">完成自然下垂重力校正後，每個動作在指定時間內自由往返。系統以有效樣本的 5%／95% 百分位自動估算活動範圍，避免單次 IMU 尖峰被當成最大角度。</p>
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

    <div class="card">
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
        <label>固定目標角度</label><input id="fixedTargetInput" value="60">
      </div>
      <div id="trajectorySinePanel" class="parameter-panel trajectory-panel">
        <div class="parameter-title">正弦軌跡參數</div>
        <div class="grid2">
          <div><label>最小角度</label><input id="trajectoryMinInput" value="30"></div>
          <div><label>最大角度</label><input id="trajectoryMaxInput" value="90"></div>
          <div><label>正弦週期 秒</label><input id="trajectoryPeriodInput" value="5"></div>
        </div>
      </div>
      <div id="trajectoryStepPanel" class="parameter-panel trajectory-panel">
        <div class="parameter-title">階躍測試參數</div>
        <div class="grid2">
          <div><label>最小角度</label><input id="stepMinMirror" value="30"></div>
          <div><label>最大角度</label><input id="stepMaxMirror" value="90"></div>
          <div><label>階躍保持 秒</label><input id="stepHoldInput" value="3"></div>
        </div>
      </div>
      <label>Trial label</label><input id="labelInput" value="elbow_sine_PID">
      <div id="trajectoryModeSummary" class="mode-summary"></div>
      <p class="small">可選擇肘或肩關節並設定目標軌跡；只有按鍵確認且 ESP32 ARM 成功後，馬達才會啟用。</p>
    </div>

    <div class="card">
      <h2>4. 控制器設定</h2>
      <label>Controller Mode</label>
      <select id="controllerModeInput">
        <option value="pid">PID</option>
        <option value="assist">落後才介入 assist-as-needed</option>
        <option value="continuous_assist" selected>持續柔和助力（前饋＋PID）</option>
        <option value="adrc">LADRC 自抗擾控制</option>
        <option value="feedforward_adrc">前饋＋LADRC</option>
        <option value="ilc_pid">ILC＋PID（週期學習）</option>
        <option value="ilc_adrc">ILC＋LADRC（週期學習）</option>
        <option value="trajectory_demo">開迴路軌跡展示（僅測試）</option>
      </select>
      <div class="validation warning" id="demoModeWarning" style="display:none">展示模式只依目標速度輸出 PWM，不使用 IMU 誤差修正。僅限空載或固定測試架，禁止連接病患；此模式不能證明馬達位置精確跟隨角度。</div>
      <div id="controllerCommonPanel" class="parameter-panel active">
        <div class="parameter-title">共用安全限制</div>
        <label>控制器輸出限制 PWM</label><input id="outputLimitInput" value="50">
      </div>
      <div id="pidParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">PID 回授參數</div>
        <div class="grid2">
          <div><label>Kp</label><input id="kpInput" value="0.8"></div>
          <div><label>Ki</label><input id="kiInput" value="0.02"></div>
          <div><label>Kd</label><input id="kdInput" value="0.05"></div>
          <div><label>積分限制</label><input id="integralLimitInput" value="100"></div>
          <div><label>Deadband 度</label><input id="deadbandInput" value="1.5"></div>
          <div id="assistDelayField"><label>Assist delay 秒</label><input id="assistDelayInput" value="0.10"></div>
        </div>
      </div>
      <div id="feedforwardParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">速度前饋參數</div>
        <div class="grid2">
          <div><label>前饋增益 PWM/(°/s)</label><input id="feedforwardGainInput" value="0.35"></div>
          <div><label>移動時最低前饋 PWM</label><input id="feedforwardMinInput" value="8"></div>
        </div>
      </div>
      <div id="adrcParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">LADRC／ESO 參數</div>
        <div class="grid2">
          <div><label>控制頻寬 ωc (rad/s)</label><input id="adrcWcInput" value="2.0"></div>
          <div><label>觀測器頻寬 ωo (rad/s)</label><input id="adrcWoInput" value="8.0"></div>
          <div><label>輸入增益 b0</label><input id="adrcB0Input" value="1.0"></div>
        </div>
      </div>
      <div id="ilcParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">ILC 週期學習參數</div>
        <div class="grid2">
          <div><label>學習增益 L</label><input id="ilcGainInput" value="0.08"></div>
          <div><label>忘卻因子 λ</label><input id="ilcForgettingInput" value="0.98"></div>
          <div><label>Q-filter 視窗</label><input id="ilcQWindowInput" value="9"></div>
          <div><label>單周期更新上限</label><input id="ilcUpdateLimitInput" value="3"></div>
          <div><label>Learned PWM 上限</label><input id="ilcLearnedLimitInput" value="20"></div>
        </div>
      </div>
      <div id="demoParameterPanel" class="parameter-panel controller-panel">
        <div class="parameter-title">開迴路展示限制</div>
        <label>展示模式 PWM 上限（最高 30）</label><input id="demoOutputLimitInput" value="15" min="0" max="30">
      </div>
      <div id="controllerModeSummary" class="mode-summary"></div>
      <button onclick="applyParams()">套用關節、軌跡與控制參數</button>
      <div id="ilcActionPanel" class="parameter-panel">
        <div class="grid2"><button class="secondary" onclick="setILCFreeze(true)">凍結 ILC 學習</button><button class="secondary" onclick="setILCFreeze(false)">恢復 ILC 學習</button></div>
        <button class="warning" onclick="clearILCLearning()">清除 ILC 學習曲線</button>
        <div class="validation" id="ilcStatus">ILC 尚未執行。</div>
      </div>
    </div>

    <div class="card">
      <h2>5. 馬達安全控制</h2>
      <div class="validation warning" id="dryRunBanner">持續柔和助力 LIVE：軌跡移動時以前饋同步收／放線，落後時由 PID 增加助力。請先確認馬達固定、繩索未連接人體且急停可用。</div>
      <div class="grid2">
        <button class="success" onclick="setMotor(true)">Enable Motor</button>
        <button class="secondary" onclick="setMotor(false)">Disable Motor</button>
      </div>
      <button class="danger" onclick="emergencyStop()">Emergency Stop</button>
      <button class="warning" onclick="clearEmergency()">解除急停</button>
      <p class="small" id="motorInterlockText">馬達預設停止。每次初始化、停止或急停後，都必須重新按鍵確認啟動。</p>
    </div>

    <div class="card">
      <h2>6. 系統識別與頻域分析</h2>
      <div class="validation warning">危險測試：Chirp／PRBS 會自動改變馬達方向。僅限空載或固定測試架，禁止連接人體。開始後系統會自行 ARM；完成、超出安全角度或通訊異常時自動 STOP。</div>
      <div class="grid2">
        <div><label>測試關節</label><select id="freqJointInput"><option value="elbow">肘關節</option><option value="shoulder">肩關節</option></select></div>
        <div><label>激振訊號</label><select id="freqSignalInput"><option value="chirp">Chirp 掃頻</option><option value="prbs">PRBS</option><option value="sine">單頻正弦</option></select></div>
        <div><label>PWM 振幅（1–30）</label><input id="freqAmplitudeInput" value="12"></div>
        <div><label>測試時間（10–180 秒）</label><input id="freqDurationInput" value="60"></div>
        <div><label>起始／單頻 Hz</label><input id="freqStartInput" value="0.1"></div>
        <div><label>結束頻率 Hz</label><input id="freqEndInput" value="3.0"></div>
        <div><label>PRBS 切換率 Hz</label><input id="freqPrbsRateInput" value="2.0"></div>
        <div><label>安全最小角度</label><input id="freqSafeMinInput" value="-10"></div>
        <div><label>安全最大角度</label><input id="freqSafeMaxInput" value="120"></div>
      </div>
      <button class="danger" onclick="startFrequencyTest()">確認固定測試架並開始</button>
      <button class="secondary" onclick="stopFrequencyTest()">停止測試並送出 STOP</button>
      <button onclick="analyzeFrequencyTest()">分析 Bode／Coherence／ARX</button>
      <div class="validation" id="frequencyStatus">尚未執行系統識別。</div>
      <canvas id="frequencyCanvas" width="950" height="430"></canvas>
    </div>

    <div class="card">
      <h2>7. 資料紀錄</h2>
      <label>受試者 ID</label><input id="participantInput" value="S01">
      <label>Session</label><input id="sessionInput" value="control_session_01">
      <label>輸出資料夾</label><input id="outputDirInput" value="control_data">
      <button class="success" onclick="startRecording()">開始錄製 CSV</button>
      <button class="danger" onclick="stopRecording()">停止錄製 CSV</button>
      <p id="recordingState" class="idle">未錄製</p>
      <p class="small" id="csvPathText"></p>
    </div>
  </div>

  <div>
    <div class="card">
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
    <div class="card">
      <h2>Target vs Measured</h2>
      <canvas id="angleCanvas" width="950" height="520"></canvas>
      <p class="small">上圖：灰線為未來目標軌跡預覽，藍線為病人已完成的 measured angle。下圖：紅線為真實 tracking error，0° 誤差線固定在中間，不再用 60° 平移顯示。垂直線為目前時間，右側可預先看到等等要跟隨的軌跡。CSV 仍會完整紀錄所有動態資料。</p>
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

// 顯示視窗設定：目前時間會畫在圖中間偏左，右側保留未來目標軌跡給使用者預看。
const maxPoints = 1200;
const pastWindowSeconds = 8;
const futureWindowSeconds = 8;

function setText(id, text){document.getElementById(id).innerText=text;}
function showStatus(text,holdMs=2500){statusMessageUntil=Date.now()+holdMs;setText('statusBox',text);}
function showValidationGuide(text,holdMs=2500){validationGuideMessageUntil=Date.now()+holdMs;setText('validationGuide',text);}
function setLiveStatus(text){if(Date.now()>=statusMessageUntil)setText('statusBox',text);}
function setLiveValidationGuide(text){if(Date.now()>=validationGuideMessageUntil)setText('validationGuide',text);}
function showActionToast(text,type='info',holdMs=1600){
  const toast=document.getElementById('actionToast');
  toast.textContent=text;toast.className='action-toast '+type+' show';
  clearTimeout(actionToastTimer);actionToastTimer=setTimeout(()=>toast.classList.remove('show'),holdMs);
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
    const response=await fetch(url,{...fetchOptions,signal:controller.signal});
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

function params(){return {target_joint:document.getElementById('targetJointInput').value,target_mode:document.getElementById('targetModeInput').value,fixed_target_angle:fval('fixedTargetInput',60),trajectory_min_angle:fval('trajectoryMinInput',30),trajectory_max_angle:fval('trajectoryMaxInput',90),trajectory_period:fval('trajectoryPeriodInput',5),step_hold_time:fval('stepHoldInput',3),controller_mode:document.getElementById('controllerModeInput').value,deadband:fval('deadbandInput',1.5),assist_delay:fval('assistDelayInput',.10),assist_feedforward_gain:fval('feedforwardGainInput',.35),assist_feedforward_min_pwm:fval('feedforwardMinInput',8),demo_feedforward_gain:fval('feedforwardGainInput',.35),demo_min_pwm:fval('feedforwardMinInput',8),demo_output_limit:Math.min(Math.abs(fval('demoOutputLimitInput',15)),30),demo_start_delay:1.0,adrc_controller_bandwidth:fval('adrcWcInput',2),adrc_observer_bandwidth:fval('adrcWoInput',8),adrc_input_gain:fval('adrcB0Input',1),ilc_learning_gain:fval('ilcGainInput',.08),ilc_forgetting_factor:fval('ilcForgettingInput',.98),ilc_q_filter_window:Math.round(fval('ilcQWindowInput',9)),ilc_update_limit:fval('ilcUpdateLimitInput',3),ilc_learned_limit:fval('ilcLearnedLimitInput',20),ilc_bins:200,output_limit:Math.min(Math.abs(fval('outputLimitInput',50)),255),kp:fval('kpInput',.8),ki:fval('kiInput',.02),kd:fval('kdInput',.05),integral_limit:fval('integralLimitInput',100)};}

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
  const periodic=demo||controllerMode==='ilc_pid'||controllerMode==='ilc_adrc';
  const target=document.getElementById('targetModeInput');
  document.getElementById('demoModeWarning').style.display=demo?'block':'none';
  Array.from(target.options).forEach(option=>{option.disabled=periodic && option.value!=='sine';});
  if(periodic && target.value!=='sine'){
    target.value='sine';
  }
  const panelMap={
    pid:['pidParameterPanel'],
    assist:['pidParameterPanel'],
    continuous_assist:['pidParameterPanel','feedforwardParameterPanel'],
    adrc:['adrcParameterPanel'],
    feedforward_adrc:['adrcParameterPanel','feedforwardParameterPanel'],
    ilc_pid:['pidParameterPanel','ilcParameterPanel'],
    ilc_adrc:['adrcParameterPanel','ilcParameterPanel'],
    trajectory_demo:['feedforwardParameterPanel','demoParameterPanel']
  };
  showOnlyPanels('.controller-panel',panelMap[controllerMode]||panelMap.pid);
  document.getElementById('assistDelayField').style.display=(controllerMode==='assist'||controllerMode==='continuous_assist')?'block':'none';
  document.getElementById('ilcActionPanel').classList.toggle('active',controllerMode==='ilc_pid'||controllerMode==='ilc_adrc');
  const descriptions={
    pid:'PID：使用角度誤差、積分與誤差變化進行閉迴路追蹤。',
    assist:'按需輔助：病患落後目標並超過死區時才逐步介入。',
    continuous_assist:'持續柔和助力：速度前饋提供基本收放線，PID 修正追蹤誤差。',
    adrc:'LADRC：由 ESO 估測速度與總擾動，再進行擾動補償。',
    feedforward_adrc:'前饋＋LADRC：速度前饋搭配 ESO 擾動補償。',
    ilc_pid:'ILC＋PID：PID 負責單周期回授，ILC 根據前一周期誤差學習前饋。',
    ilc_adrc:'ILC＋LADRC：LADRC 抗擾，ILC 學習重複軌跡的前饋輸出。',
    trajectory_demo:'開迴路展示：只依目標速度輸出，不使用 IMU 誤差修正；禁止連接人體。'
  };
  setText('controllerModeSummary',descriptions[controllerMode]||descriptions.pid);
  updateJointLabels();
}

document.getElementById('targetModeInput').addEventListener('change', updateJointLabels);
document.getElementById('targetJointInput').addEventListener('change', updateJointLabels);
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
async function applyParams(){
  showStatus('正在套用控制參數...');
  const res=await postJSON('/api/apply_params',params());
  showStatus(res.message||'控制參數沒有回傳訊息。');
  if(!res.ok)return;
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
  const res=await postJSON('/api/set_motor',{enabled,confirmed});
  showStatus(res.message||'馬達控制沒有回傳訊息。');
}
async function emergencyStop(){showStatus('正在執行 Emergency Stop...');const res=await postJSON('/api/emergency_stop',{});showStatus(res.message||'急停沒有回傳訊息。');}
async function clearEmergency(){showStatus('正在解除急停...');const res=await postJSON('/api/clear_emergency',{});showStatus(res.message||'解除急停沒有回傳訊息。');}
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

function updatePage(data){
  setLiveStatus(data.status);
  updateValidationPanel(data.rom_calibration||{});
  updateFrequencyPanel(data.frequency_identification||{});
  updateILCPanel(data.ilc||{});
  if(data.motor_output){
    const s=data.motor_output;
    const demo=data.frame && data.frame.controller_mode==='trajectory_demo';
    setText('dryRunBanner',demo
      ?'開迴路展示 LIVE：只依目標速度輸出，IMU 僅記錄；上限 '+s.demo_output_limit+' PWM、啟動停留 1 秒。僅限空載／固定測試架，禁止連接人體。'
      :'持續柔和助力 LIVE：目標速度前饋＋誤差 PID；肘肌反向 1:'+s.antagonist_release_gain+'、低通 '+s.command_filter_tau+'s、換向停頓 '+Math.round(s.reverse_deadtime*1000)+'ms；按鍵確認及 ESP32 ARM 成功後才輸出。');
    setText('motorInterlockText','無 ROM／驗證通過鎖；初始化、停止或急停後必須重新確認啟動。');
  }
  if(data.recording){setText('recordingState','錄製中，筆數：'+data.record_count);document.getElementById('recordingState').className='rec';setText('csvPathText',data.csv_path||'');}
  if(data.frame){
    const f=data.frame;
    setText('validationLiveAngles','即時角度｜肘='+f.elbow_angle.toFixed(1)+'°'
      +'｜roll候選='+f.elbow_roll_signed.toFixed(1)+'°｜pitch候選='+f.elbow_pitch_signed.toFixed(1)+'°'
      +'｜肩='+f.shoulder_angle.toFixed(1)+'°｜肩速度='+(f.shoulder_angle_velocity ?? 0).toFixed(1)+'°/s');
    const measured=(f.measured_angle!==undefined)?f.measured_angle:(f.target_joint==='shoulder'?f.shoulder_angle:f.elbow_angle);
    setText('measuredAngle',measured.toFixed(1)+'°');
    setText('elbowAngle',f.elbow_angle.toFixed(1)+'°');
    setText('shoulderAngle',f.shoulder_angle.toFixed(1)+'°');
    setText('targetAngle',f.target_angle.toFixed(1)+'°');
    setText('errorValue',f.error.toFixed(1)+'°');
    setText('motionState',f.motion_state);

    measuredHistory.push(measured);
    errorHistory.push(f.error);
    timeHistory.push(f.time);
    latestTime = f.time;

    if(measuredHistory.length>maxPoints){
      measuredHistory.shift();
      errorHistory.shift();
      timeHistory.shift();
    }
    drawPlot();
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

  // 灰線：完整目標軌跡預覽，包含過去與未來
  ctx.strokeStyle='#9ca3af';ctx.lineWidth=2;ctx.beginPath();
  const samples=260;
  for(let i=0;i<samples;i++){
    const tt = windowStart + (i/(samples-1))*windowSpan;
    const x = mxTime(tt);
    const y = myAngle(targetAtTime(tt));
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();

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
  ctx.fillStyle='#6b7280';ctx.fillText('Target preview',ml+190,topMt+18);

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
  setTimeout(pollStatus,250);
}
pollStatus();
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

def set_status(message):
    global system_status
    with state_lock:
        system_status = message

def serialize_frame(frame):
    if frame is None:
        return None
    keys_float = [
        "time","upper_angle","forearm_angle","signed_elbow_angle","elbow_roll_signed","elbow_pitch_signed","elbow_sign","elbow_angle_raw","elbow_angle",
        "measured_angle","measured_angle_raw","upper_roll","upper_pitch","forearm_roll","forearm_pitch",
        "omega","alpha","jerk","shoulder_angle","shoulder_angle_raw","shoulder_angle_velocity","shoulder_release_pwm","trajectory_time","trajectory_wait_remaining","frequency_command",
        "shoulder_dx","shoulder_dy","shoulder_dz","target_angle","target_velocity","error","error_for_control","error_active_time",
        "integral_error","derivative_error","filtered_measurement_velocity","raw_pid_output","feedback_pid_output","feedforward_output","pid_output","motor_cmd","servo_cmd",
        "adrc_output","adrc_raw_output","adrc_estimated_angle","adrc_estimated_velocity","adrc_estimated_disturbance","adrc_observer_error",
        "ilc_output","ilc_current_cycle","ilc_last_rmse","ilc_improvement_percent","ilc_learned_peak_pwm",
        "deadband_on","deadband_off",
        "desired_motor_cmd","desired_pwm_biceps","desired_pwm_triceps","desired_pwm_deltoid",
        "pwm_biceps","pwm_triceps","pwm_deltoid",
        "virtual_cable_effort_biceps","virtual_cable_effort_triceps","virtual_cable_effort_deltoid",
        "encoder_count_biceps","encoder_count_triceps","encoder_count_deltoid",
        "encoder_vel_biceps","encoder_vel_triceps","encoder_vel_deltoid"
    ]
    out = {}
    for k in keys_float:
        try:
            out[k] = float(frame.get(k, 0.0))
        except Exception:
            out[k] = 0.0
    for k in ["motion_state","shoulder_motion_state","shoulder_plane","elbow_axis","target_mode","target_joint","controller_mode","serial_tx"]:
        out[k] = str(frame.get(k, ""))
    for k in ["shoulder_assist","shoulder_release_command","shoulder_motor_enable","shoulder_accel_valid","control_active","anti_windup_active","adrc_saturated","ilc_frozen","motor_enabled","emergency_stop"]:
        out[k] = bool(frame.get(k, False))
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
        "controller_mode": data.get("controller_mode", "continuous_assist"),
        "deadband": safe_float(data.get("deadband"), 1.5),
        "deadband_on": safe_float(data.get("deadband"), 1.5) + 0.5,
        "deadband_off": max(0.0, safe_float(data.get("deadband"), 1.5) - 0.5),
        "assist_delay": safe_float(data.get("assist_delay"), 0.10),
        "assist_feedforward_gain": max(0.0, safe_float(data.get("assist_feedforward_gain"), 0.35)),
        "assist_feedforward_min_pwm": max(0.0, safe_float(data.get("assist_feedforward_min_pwm"), 8.0)),
        "assist_feedforward_velocity_threshold": 2.0,
        "demo_feedforward_gain": max(0.0, safe_float(data.get("demo_feedforward_gain"), 0.35)),
        "demo_min_pwm": max(0.0, safe_float(data.get("demo_min_pwm"), 8.0)),
        "demo_output_limit": min(abs(safe_float(data.get("demo_output_limit"), 15.0)), 30.0),
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
        "output_limit": min(abs(safe_float(data.get("output_limit"), 50.0)), MOTOR_PWM_LIMIT),
        "kp": safe_float(data.get("kp"), 0.8),
        "ki": safe_float(data.get("ki"), 0.02),
        "kd": safe_float(data.get("kd"), 0.05),
        "integral_limit": safe_float(data.get("integral_limit"), 100.0),
    }
    if hasattr(sensor, "configure_control"):
        sensor.configure_control(reset_controller=reset_controller, **params)
    else:
        for k, v in params.items():
            setattr(sensor, k, v)

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
                    if sensor is not None:
                        sensor.motor_enabled = False
                    muscle_allocator.reset_conditioner()
                    stop_motor_bridge_safely()
                    if sensor is not None and hasattr(sensor, "interrupt_bus"):
                        sensor.interrupt_bus()
                    set_status(
                        "IMU 暫時斷線，正在自動重新開啟 I2C；"
                        "馬達已停止，既有校正會保留。"
                    )
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

    while reader_running:
        try:
            # 1) Read IMU + calculate target/error/PID motor_cmd
            imu_last_attempt_monotonic = time.monotonic()
            frame = sensor.read_frame()
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
            desired_pwm = muscle_allocator.allocate(
                target_joint=frame.get("target_joint", "elbow"),
                motor_cmd=desired_cmd,
                control_profile=MOTOR_CONTROL_PROFILE,
                motor_enabled=True,
                emergency_stop=False,
                shoulder_release_command=False if frequency_running else frame.get("shoulder_release_command", False),
                shoulder_release_pwm=frame.get("shoulder_release_pwm", 0.0),
                shoulder_motor_enable=True if frequency_running else frame.get("shoulder_motor_enable", True),
                feedback=fb,
                now=frame.get("time"),
            )

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
            )
            output_pwm = desired_pwm if can_output else MusclePWM()
            observed_pwm = (
                MusclePWM(fb.pwm1, fb.pwm2, fb.pwm3)
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
                    if (frame.get("emergency_stop", False)
                            or not frame.get("motor_enabled", False)):
                        frame["serial_tx"] = "STOP"
                        motor_bridge.stop()
                    else:
                        frame["serial_tx"] = f"PWM,{output_pwm.biceps},{output_pwm.triceps},{output_pwm.deltoid}"
                        motor_bridge.set_pwm(output_pwm.biceps, output_pwm.triceps, output_pwm.deltoid)
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
                        frame.get("virtual_cable_effort_biceps"), frame.get("virtual_cable_effort_triceps"), frame.get("virtual_cable_effort_deltoid"),
                        frame.get("encoder_count_biceps"), frame.get("encoder_count_triceps"), frame.get("encoder_count_deltoid"),
                        frame.get("encoder_vel_biceps"), frame.get("encoder_vel_triceps"), frame.get("encoder_vel_deltoid"),
                        frame.get("upper_angle"), frame.get("forearm_angle"), frame.get("omega"), frame.get("alpha"), frame.get("jerk"),
                        frame.get("shoulder_motion_state"), frame.get("shoulder_release_command"), frame.get("shoulder_release_pwm"), frame.get("shoulder_motor_enable"),
                        frame.get("motor_enabled"), frame.get("emergency_stop"), frame.get("motion_state"),
                    ])
                    record_count += 1
                    if record_count % 50 == 0:
                        csv_file.flush()

            time.sleep(0.01)

        except Exception as e:
            imu_read_failure_count += 1
            imu_last_error = str(e)
            if sensor is not None:
                sensor.motor_enabled = False
            if frequency_session.status().get("state") == "running":
                frequency_session.stop(f"IMU／控制迴圈錯誤：{e}", aborted=True)
            set_status(f"讀取 IMU / 馬達資料錯誤: {e}")
            stop_motor_bridge_safely()
            muscle_allocator.reset_conditioner()
            if imu_recovery_requested_monotonic <= 0.0:
                imu_recovery_requested_monotonic = time.monotonic()
            try:
                channels = sensor.reconnect_imus(retries=5, retry_delay=0.25)
                imu_last_success_monotonic = time.monotonic()
                imu_last_error = ""
                imu_recovery_requested_monotonic = 0.0
                set_status(
                    f"IMU 已自動重連，通道 {channels}；已沿用原校正。"
                    "馬達維持停止，請確認繩索後重新按 Enable Motor。"
                )
            except Exception as reconnect_error:
                imu_last_error = str(reconnect_error)
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
    try:
        set_status("正在建立 IMU 系統...")
        new_sensor = IMURehabSystem(
            target_joint=params.get("target_joint", "elbow"),
            target_mode=params.get("target_mode", "fixed"),
            fixed_target_angle=safe_float(params.get("fixed_target_angle"), 60.0),
            trajectory_min_angle=safe_float(params.get("trajectory_min_angle"), 30.0),
            trajectory_max_angle=safe_float(params.get("trajectory_max_angle"), 90.0),
            trajectory_period=safe_float(params.get("trajectory_period"), 5.0),
            step_hold_time=safe_float(params.get("step_hold_time"), 3.0),
            controller_mode=params.get("controller_mode", "continuous_assist"),
            deadband=safe_float(params.get("deadband"), 1.5),
            deadband_on=safe_float(params.get("deadband"), 1.5) + 0.5,
            deadband_off=max(0.0, safe_float(params.get("deadband"), 1.5) - 0.5),
            assist_delay=safe_float(params.get("assist_delay"), 0.10),
            assist_feedforward_gain=max(0.0, safe_float(params.get("assist_feedforward_gain"), 0.35)),
            assist_feedforward_min_pwm=max(0.0, safe_float(params.get("assist_feedforward_min_pwm"), 8.0)),
            assist_feedforward_velocity_threshold=2.0,
            demo_feedforward_gain=max(0.0, safe_float(params.get("demo_feedforward_gain"), 0.35)),
            demo_min_pwm=max(0.0, safe_float(params.get("demo_min_pwm"), 8.0)),
            demo_output_limit=min(abs(safe_float(params.get("demo_output_limit"), 15.0)), 30.0),
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
            output_limit=min(abs(safe_float(params.get("output_limit"), 50.0)), MOTOR_PWM_LIMIT),
            kp=safe_float(params.get("kp"), 0.8),
            ki=safe_float(params.get("ki"), 0.02),
            kd=safe_float(params.get("kd"), 0.05),
            integral_limit=safe_float(params.get("integral_limit"), 100.0),
            motor_enabled=False,
            emergency_stop=False,
        )
        set_status("正在掃描與初始化 MPU6050...")
        channels = new_sensor.scan_and_init()
        set_status(f"初始化成功，可用通道: {channels}\n請保持手臂自然下垂並靜止，開始 5 秒校正。")
        def progress(elapsed, remaining):
            set_status(f"校正中... 剩餘 {remaining:.1f} 秒")
        result = new_sensor.calibrate(seconds=5, progress_callback=progress)
        muscle_allocator.reset_conditioner()
        muscle_allocator.reset_cable_tracker()
        with state_lock:
            sensor = new_sensor
            rom_calibration.reset()
            initialized = True
            initializing = False
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
        set_status(f"初始化或校正失敗: {e}")

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/api/init", methods=["POST"])
def api_init():
    global initializing, reader_running, imu_init_started_monotonic
    params = request.get_json(force=True) or {}
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

@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify({
            "status": system_status,
            "initialized": initialized,
            "initializing": initializing,
            "recording": recording,
            "record_count": record_count,
            "csv_path": current_csv_path,
            "motor_bridge_connected": motor_bridge_connected,
            "motor_bridge_error": motor_bridge_error,
            "motor_bridge_last_line": motor_bridge.last_line if motor_bridge is not None else "",
            "motor_bridge_last_sent": motor_bridge.last_sent_line if motor_bridge is not None and hasattr(motor_bridge, "last_sent_line") else "",
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
            },
            "motor_output": {
                "mode": "live_on_confirmation",
                "control_profile": MOTOR_CONTROL_PROFILE,
                "pwm_limit": MOTOR_PWM_LIMIT,
                "antagonist_release_gain": 0.8,
                "cable_return_gain": muscle_allocator.cable_return_gain,
                "virtual_cable_effort": muscle_allocator.cable_effort_status(),
                "command_filter_tau": 0.08,
                "wind_slew_rate": 240.0,
                "release_slew_rate": 300.0,
                "reverse_deadtime": 0.08,
                "continuous_assist": True,
                "feedforward_gain": 0.35,
                "feedforward_min_pwm": 8.0,
                "trajectory_demo": True,
                "demo_output_limit": getattr(sensor, "demo_output_limit", 15.0) if sensor is not None else 15.0,
                "adrc": True,
                "adrc_controller_bandwidth": getattr(sensor, "adrc_controller_bandwidth", 2.0) if sensor is not None else 2.0,
                "adrc_observer_bandwidth": getattr(sensor, "adrc_observer_bandwidth", 8.0) if sensor is not None else 8.0,
                "ilc": True,
                "shoulder_release_max_pwm": 60.0,
                "live_output_allowed": True,
                "encoder_safety_enabled": False,
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
        sensor.emergency_stop = False
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
    data = request.get_json(force=True) or {}
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
    return jsonify({"ok": True, "message": "目標關節、復健軌跡與控制參數已套用，控制器狀態已重置。"})

@app.route("/api/set_motor", methods=["POST"])
def api_set_motor():
    data = request.get_json(force=True) or {}
    enabled = safe_bool(data.get("enabled", False))
    confirmed = safe_bool(data.get("confirmed", False))

    with state_lock:
        if sensor is None:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
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
        ok = connect_motor_bridge_if_needed()
        if not ok:
            with state_lock:
                sensor.motor_enabled = False
            return jsonify({
                "ok": False,
                "message": f"ESP32 連線失敗：{motor_bridge_error}。請確認 USB 線、port={ESP32_PORT}、ESP32 程式已燒錄。"
            })
        try:
            motor_bridge.stop()
            motor_bridge.arm()
        except Exception as exc:
            with state_lock:
                sensor.motor_enabled = False
            stop_motor_bridge_safely()
            return jsonify({
                "ok": False,
                "message": f"ESP32 ARM 確認失敗：{exc}。馬達維持停止。"
            }), 503

    with state_lock:
        sensor.motor_enabled = enabled
        if enabled:
            sensor.emergency_stop = False
            if hasattr(sensor, "reset_trajectory"):
                sensor.reset_trajectory()
            msg = "Motor enabled：PWM 會經由 USB Serial 輸出到 ESP32。展示模式會先在最小角度停留 1 秒。" if str(getattr(sensor, "controller_mode", "")).lower() == "trajectory_demo" else "Motor enabled：PWM 會經由 USB Serial 輸出到 ESP32。"
        else:
            sensor.emergency_stop = False
            if frequency_session.status().get("state") == "running":
                frequency_session.stop("由馬達停用按鍵停止。", aborted=True)
                try:
                    frequency_session.save_raw()
                except Exception:
                    pass
            if hasattr(sensor, "stop_trajectory"):
                sensor.stop_trajectory()
            stop_motor_bridge_safely()
            msg = "Motor disabled：已送 STOP 給 ESP32。"

    return jsonify({"ok": True, "message": msg})

@app.route("/api/emergency_stop", methods=["POST"])
def api_emergency_stop():
    with state_lock:
        if sensor is not None:
            sensor.emergency_stop = True
            sensor.motor_enabled = False
            if hasattr(sensor, "stop_trajectory"):
                sensor.stop_trajectory()
        if frequency_session.status().get("state") == "running":
            frequency_session.stop("Emergency Stop。", aborted=True)
    try:
        if motor_bridge is not None and motor_bridge_connected:
            motor_bridge.emergency_stop()
    except Exception:
        stop_motor_bridge_safely()
    return jsonify({"ok": True, "message": "Emergency stop 已啟動：已送 ESTOP 給 ESP32。"})

@app.route("/api/clear_emergency", methods=["POST"])
def api_clear_emergency():
    with state_lock:
        if sensor is not None:
            sensor.emergency_stop = False
    try:
        if motor_bridge is not None and motor_bridge_connected:
            motor_bridge.clear_fault()
    except Exception:
        stop_motor_bridge_safely()
    return jsonify({"ok": True, "message": "急停已解除。若要輸出控制命令，請重新按 Enable Motor。"})

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
                "virtual_cable_effort_biceps","virtual_cable_effort_triceps","virtual_cable_effort_deltoid",
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
